#include "input_block.h"
#include "../util/log.h"
#include "../vr/xr_input.h"
#include "../hooks/camera_rig_hook.h"

#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#include <MinHook.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <intrin.h>
#include <cstdlib>

namespace {

std::atomic<bool> g_blocking{ false };
// Set when blocking ends: the next real read of each device records what is
// still held, and those inputs stay hidden until they are released.
std::atomic<bool> g_captureKeyMask{ false };
std::atomic<bool> g_captureMouseMask{ false };
std::atomic<bool> g_capturePadMask[XUSER_MAX_COUNT] = {};

BYTE g_keyMask[256] = {};
BYTE g_mouseMask[8] = {};
WORD g_padButtonMask[XUSER_MAX_COUNT] = {};
bool g_padTriggerMask[XUSER_MAX_COUNT][2] = {};

// Mouse movement collected while blocking.
SRWLOCK g_mouseLock = SRWLOCK_INIT;
long g_mouseDx = 0, g_mouseDy = 0, g_mouseWheel = 0;
bool g_mouseButtons[3] = {};
std::atomic<ULONGLONG> g_lastDiMouseMs{ 0 };
// Raw view of the game's mouse reads, for the menu's once-a-second input log:
// if the pointer is dead or pinned, these say whether the data is empty, in
// a format we don't expect (a custom data format moves the offsets), or
// absolute rather than relative.
std::atomic<long> g_dbgStateCb{ 0 }, g_dbgStateX{ 0 }, g_dbgStateY{ 0 };
std::atomic<unsigned long> g_dbgStateCalls{ 0 }, g_dbgDataCalls{ 0 }, g_dbgDataItems{ 0 }, g_dbgDataOfsMask{ 0 };

// ---- Which DirectInput device is which --------------------------------
// GetCapabilities once per device pointer. The game has a handful of devices
// for its whole life, so a tiny table is plenty.
struct DeviceKind {
    void* device;
    BYTE type; // DI8DEVTYPE_*
};
DeviceKind g_kinds[16] = {};
SRWLOCK g_kindLock = SRWLOCK_INIT;

// The mod's own DirectInput pad (opened at the bottom of this file). The
// game's devices are filtered below; ours never is, or the menu would go deaf
// the moment it opened.
IDirectInputDevice8A* g_diPad = nullptr;
// A DirectInput pad's buttons, held when the menu closed, hidden until let go.
BYTE g_diPadMask[128] = {};
std::atomic<bool> g_captureDiPadMask{ false };

typedef HRESULT(STDMETHODCALLTYPE* GetDeviceState_t)(IDirectInputDevice8A*, DWORD, LPVOID);
typedef HRESULT(STDMETHODCALLTYPE* GetDeviceData_t)(IDirectInputDevice8A*, DWORD, LPDIDEVICEOBJECTDATA, LPDWORD, DWORD);
typedef HRESULT(STDMETHODCALLTYPE* GetCapabilities_t)(IDirectInputDevice8A*, LPDIDEVCAPS);

BYTE KindOf(IDirectInputDevice8A* dev)
{
    AcquireSRWLockShared(&g_kindLock);
    for (const DeviceKind& k : g_kinds) {
        if (k.device == dev) {
            const BYTE t = k.type;
            ReleaseSRWLockShared(&g_kindLock);
            return t;
        }
    }
    ReleaseSRWLockShared(&g_kindLock);

    DIDEVCAPS caps = {};
    caps.dwSize = sizeof(caps);
    // Straight through the vtable: GetCapabilities is not hooked, and the A
    // and W interfaces share the slot.
    const GetCapabilities_t getCaps = reinterpret_cast<GetCapabilities_t>((*reinterpret_cast<void***>(dev))[3]);
    BYTE type = 0;
    if (SUCCEEDED(getCaps(dev, &caps)))
        type = static_cast<BYTE>(GET_DIDEVICE_TYPE(caps.dwDevType));

    AcquireSRWLockExclusive(&g_kindLock);
    for (DeviceKind& k : g_kinds) {
        if (!k.device) {
            k.device = dev;
            k.type = type;
            break;
        }
    }
    ReleaseSRWLockExclusive(&g_kindLock);
    Log_Printf("InputBlock: DirectInput device %p is type 0x%02X (%s)", dev, type,
        type == DI8DEVTYPE_KEYBOARD ? "keyboard" : type == DI8DEVTYPE_MOUSE ? "mouse" : "other");
    return type;
}

void CollectMouse(long dx, long dy, long dz, const BYTE* buttons, int buttonCount)
{
    AcquireSRWLockExclusive(&g_mouseLock);
    g_mouseDx += dx;
    g_mouseDy += dy;
    g_mouseWheel += dz;
    for (int i = 0; i < 3 && i < buttonCount; ++i)
        g_mouseButtons[i] = (buttons[i] & 0x80) != 0;
    ReleaseSRWLockExclusive(&g_mouseLock);
    g_lastDiMouseMs.store(GetTickCount64(), std::memory_order_relaxed);
}

// Hides held inputs recorded at close until each one is let go.
void ApplyMask(BYTE* state, BYTE* mask, int n, std::atomic<bool>& capture)
{
    if (capture.exchange(false, std::memory_order_acquire)) {
        for (int i = 0; i < n; ++i)
            mask[i] = state[i] & 0x80;
    }
    for (int i = 0; i < n; ++i) {
        if (!mask[i])
            continue;
        if (!(state[i] & 0x80))
            mask[i] = 0;
        else
            state[i] = 0;
    }
}

// Counted while blocking, reported at close: which devices the game read.
std::atomic<unsigned long> g_diKeyboardReads{ 0 }, g_diMouseReads{ 0 }, g_diOtherReads{ 0 };

// A DirectInput pad the game is reading, made idle while the menu is open.
// Axes go to the centre of whatever range the game set for them, not to zero:
// a joystick axis usually runs 0 to 65535, where zero means hard left.
void IdleJoystickState(IDirectInputDevice8A* dev, LPVOID data, DWORD cb)
{
    constexpr int kAxes = 8; // lX lY lZ lRx lRy lRz slider0 slider1
    struct Centers {
        void* device;
        LONG axis[kAxes];
    };
    static Centers s_cache[8] = {};
    static SRWLOCK s_lock = SRWLOCK_INIT;

    LONG centers[kAxes] = {};
    bool known = false;
    AcquireSRWLockShared(&s_lock);
    for (const Centers& c : s_cache) {
        if (c.device == dev) {
            std::memcpy(centers, c.axis, sizeof(centers));
            known = true;
            break;
        }
    }
    ReleaseSRWLockShared(&s_lock);

    if (!known) {
        typedef HRESULT(STDMETHODCALLTYPE * GetProperty_t)(IDirectInputDevice8A*, REFGUID, LPDIPROPHEADER);
        const auto getProp = reinterpret_cast<GetProperty_t>((*reinterpret_cast<void***>(dev))[5]);
        for (int i = 0; i < kAxes; ++i) {
            DIPROPRANGE range = {};
            range.diph.dwSize = sizeof(range);
            range.diph.dwHeaderSize = sizeof(range.diph);
            range.diph.dwHow = DIPH_BYOFFSET;
            range.diph.dwObj = static_cast<DWORD>(i * sizeof(LONG));
            centers[i] = SUCCEEDED(getProp(dev, DIPROP_RANGE, &range.diph))
                ? (range.lMin + range.lMax) / 2
                : 0;
        }
        AcquireSRWLockExclusive(&s_lock);
        for (Centers& c : s_cache) {
            if (!c.device) {
                c.device = dev;
                std::memcpy(c.axis, centers, sizeof(centers));
                break;
            }
        }
        ReleaseSRWLockExclusive(&s_lock);
    }

    // DIJOYSTATE and DIJOYSTATE2 share this prefix: eight axes, four hats,
    // then the buttons. Everything past the hats reads as nothing pressed.
    auto* axes = static_cast<LONG*>(data);
    for (int i = 0; i < kAxes; ++i)
        axes[i] = centers[i];
    auto* pov = reinterpret_cast<DWORD*>(static_cast<BYTE*>(data) + 32);
    for (int i = 0; i < 4; ++i)
        pov[i] = 0xFFFFFFFF; // centred
    std::memset(static_cast<BYTE*>(data) + 48, 0, cb - 48);
}

HRESULT FilterDeviceState(IDirectInputDevice8A* dev, DWORD cb, LPVOID data, HRESULT hr)
{
    if (FAILED(hr) || !data || dev == g_diPad)
        return hr;
    const BYTE kind = KindOf(dev);
    if (g_blocking.load(std::memory_order_relaxed))
        (kind == DI8DEVTYPE_MOUSE ? g_diMouseReads : kind == DI8DEVTYPE_KEYBOARD ? g_diKeyboardReads : g_diOtherReads)
            .fetch_add(1, std::memory_order_relaxed);
    if (kind == DI8DEVTYPE_MOUSE && (cb == sizeof(DIMOUSESTATE) || cb == sizeof(DIMOUSESTATE2))) {
        const int buttons = cb == sizeof(DIMOUSESTATE2) ? 8 : 4;
        auto* m = static_cast<DIMOUSESTATE2*>(data); // DIMOUSESTATE is its prefix
        g_dbgStateCb.store(static_cast<long>(cb), std::memory_order_relaxed);
        g_dbgStateX.store(m->lX, std::memory_order_relaxed);
        g_dbgStateY.store(m->lY, std::memory_order_relaxed);
        g_dbgStateCalls.fetch_add(1, std::memory_order_relaxed);
        if (g_blocking.load(std::memory_order_relaxed)) {
            CollectMouse(m->lX, m->lY, m->lZ, m->rgbButtons, buttons);
            std::memset(data, 0, cb);
        } else {
            ApplyMask(m->rgbButtons, g_mouseMask, buttons, g_captureMouseMask);
            // 3DOF aiming rides in here (2026-09-16). The stick could never be
            // one to one: full deflection turns the gun 86 degrees a second
            // and a wrist flick is more than twice that. A mouse has no such
            // ceiling - the game turns by however many counts arrive - so the
            // aim servo hands its correction to the game's own mouse read
            // instead. Consumed once, so a frame the game doesn't read costs
            // nothing and one it reads twice doesn't double up.
            long aimDx = 0, aimDy = 0;
            if (CameraRigHook_TakeAimMouse(&aimDx, &aimDy)) {
                m->lX += aimDx;
                m->lY += aimDy;
            }
        }
    } else if (kind == DI8DEVTYPE_MOUSE) {
        // A mouse read in a format we don't parse: still hide it from the game.
        g_dbgStateCb.store(static_cast<long>(cb), std::memory_order_relaxed);
        g_dbgStateCalls.fetch_add(1, std::memory_order_relaxed);
        if (g_blocking.load(std::memory_order_relaxed))
            std::memset(data, 0, cb);
    } else if (kind == DI8DEVTYPE_KEYBOARD && cb == 256) {
        if (g_blocking.load(std::memory_order_relaxed))
            std::memset(data, 0, cb);
        else
            ApplyMask(static_cast<BYTE*>(data), g_keyMask, 256, g_captureKeyMask);
    } else if (cb == sizeof(DIJOYSTATE) || cb == sizeof(DIJOYSTATE2)) {
        // A pad the game reads through DirectInput (2026-09-15): a PlayStation
        // pad without Steam Input, or a generic USB one. Until now it walked
        // and shot straight through the open menu.
        const int buttons = cb == sizeof(DIJOYSTATE2) ? 128 : 32;
        if (g_blocking.load(std::memory_order_relaxed))
            IdleJoystickState(dev, data, cb);
        else
            ApplyMask(static_cast<BYTE*>(data) + 48, g_diPadMask, buttons, g_captureDiPadMask);
    }
    return hr;
}

HRESULT FilterDeviceData(IDirectInputDevice8A* dev, LPDIDEVICEOBJECTDATA items, LPDWORD count, DWORD flags, HRESULT hr)
{
    if (FAILED(hr) || !count || !items || (flags & DIGDD_PEEK) || !g_blocking.load(std::memory_order_relaxed))
        return hr;
    const BYTE kind = KindOf(dev);
    (kind == DI8DEVTYPE_MOUSE ? g_diMouseReads : kind == DI8DEVTYPE_KEYBOARD ? g_diKeyboardReads : g_diOtherReads)
        .fetch_add(1, std::memory_order_relaxed);
    // Any device: the game gets an empty buffer while the menu is open. Pads
    // used to be waved through here (2026-09-15).
    if (kind == DI8DEVTYPE_MOUSE) {
        // Buffered mouse: the same collection, event by event.
        long dx = 0, dy = 0, dz = 0;
        BYTE buttons[3] = {};
        AcquireSRWLockShared(&g_mouseLock);
        for (int i = 0; i < 3; ++i)
            buttons[i] = g_mouseButtons[i] ? 0x80 : 0;
        ReleaseSRWLockShared(&g_mouseLock);
        g_dbgDataCalls.fetch_add(1, std::memory_order_relaxed);
        g_dbgDataItems.fetch_add(*count, std::memory_order_relaxed);
        for (DWORD i = 0; i < *count; ++i) {
            const DIDEVICEOBJECTDATA& e = items[i];
            if (e.dwOfs < 32)
                g_dbgDataOfsMask.fetch_or(1ul << e.dwOfs, std::memory_order_relaxed);
            if (e.dwOfs == DIMOFS_X)
                dx += static_cast<LONG>(e.dwData);
            else if (e.dwOfs == DIMOFS_Y)
                dy += static_cast<LONG>(e.dwData);
            else if (e.dwOfs == DIMOFS_Z)
                dz += static_cast<LONG>(e.dwData);
            else if (e.dwOfs >= DIMOFS_BUTTON0 && e.dwOfs <= DIMOFS_BUTTON2)
                buttons[e.dwOfs - DIMOFS_BUTTON0] = static_cast<BYTE>(e.dwData & 0x80);
        }
        CollectMouse(dx, dy, dz, buttons, 3);
    }
    *count = 0; // the game sees an empty buffer
    return hr;
}

// ---- Every DirectInput implementation the game can reach --------------
// The first two builds hooked GetDeviceState/GetDeviceData as found on a
// throwaway KEYBOARD device, assuming every device shares that code. In play
// the mouse still went straight through and not one mouse read ever reached
// the hook, although the exe does create a GUID_SysMouse device - so the mouse
// is served by different functions. Now each distinct implementation gets its
// own slot: probed from throwaway keyboard AND mouse devices (the game's are
// usually created before we load), and again for every device the game
// creates afterwards, through a CreateDevice hook.
constexpr int kDiSlots = 6;
struct DiSlot {
    void* stateTarget;
    void* dataTarget;
};
DiSlot g_diSlots[kDiSlots] = {};
GetDeviceState_t oState[kDiSlots] = {};
GetDeviceData_t oData[kDiSlots] = {};
SRWLOCK g_diHookLock = SRWLOCK_INIT;

#define RE5VR_DI_DETOURS(i)                                                                                        \
    HRESULT STDMETHODCALLTYPE hkState##i(IDirectInputDevice8A* dev, DWORD cb, LPVOID data)                         \
    {                                                                                                              \
        return FilterDeviceState(dev, cb, data, oState[i](dev, cb, data));                                         \
    }                                                                                                              \
    HRESULT STDMETHODCALLTYPE hkData##i(IDirectInputDevice8A* dev, DWORD cb, LPDIDEVICEOBJECTDATA items, LPDWORD n, \
        DWORD flags)                                                                                               \
    {                                                                                                              \
        return FilterDeviceData(dev, items, n, flags, oData[i](dev, cb, items, n, flags));                         \
    }
RE5VR_DI_DETOURS(0)
RE5VR_DI_DETOURS(1)
RE5VR_DI_DETOURS(2)
RE5VR_DI_DETOURS(3)
RE5VR_DI_DETOURS(4)
RE5VR_DI_DETOURS(5)
#undef RE5VR_DI_DETOURS
const GetDeviceState_t kStateDetours[kDiSlots] = { hkState0, hkState1, hkState2, hkState3, hkState4, hkState5 };
const GetDeviceData_t kDataDetours[kDiSlots] = { hkData0, hkData1, hkData2, hkData3, hkData4, hkData5 };

// Hooks this device's GetDeviceState (vtable 9) and GetDeviceData (10) unless
// that exact code is hooked already.
void HookDeviceVtable(void* device, const char* what)
{
    void** vt = *reinterpret_cast<void***>(device);
    AcquireSRWLockExclusive(&g_diHookLock);
    bool stateDone = false, dataDone = false;
    int free = -1;
    for (int i = 0; i < kDiSlots; ++i) {
        if (g_diSlots[i].stateTarget == vt[9])
            stateDone = true;
        if (g_diSlots[i].dataTarget == vt[10])
            dataDone = true;
        if (free < 0 && !g_diSlots[i].stateTarget && !g_diSlots[i].dataTarget)
            free = i;
    }
    if (stateDone && dataDone) {
        ReleaseSRWLockExclusive(&g_diHookLock);
        return;
    }
    if (free < 0) {
        ReleaseSRWLockExclusive(&g_diHookLock);
        Log_Printf("InputBlock: no DirectInput hook slot left for %s", what);
        return;
    }
    MH_STATUS stState = MH_OK, stData = MH_OK;
    if (!stateDone) {
        stState = MH_CreateHook(vt[9], reinterpret_cast<void*>(kStateDetours[free]), reinterpret_cast<void**>(&oState[free]));
        if (stState == MH_OK)
            stState = MH_EnableHook(vt[9]);
        if (stState == MH_OK)
            g_diSlots[free].stateTarget = vt[9];
    }
    if (!dataDone) {
        stData = MH_CreateHook(vt[10], reinterpret_cast<void*>(kDataDetours[free]), reinterpret_cast<void**>(&oData[free]));
        if (stData == MH_OK)
            stData = MH_EnableHook(vt[10]);
        if (stData == MH_OK)
            g_diSlots[free].dataTarget = vt[10];
    }
    ReleaseSRWLockExclusive(&g_diHookLock);
    Log_Printf("InputBlock: %s - GetDeviceState %p %s, GetDeviceData %p %s (slot %d)", what, vt[9],
        stateDone ? "already hooked" : (stState == MH_OK ? "hooked" : "FAILED"), vt[10],
        dataDone ? "already hooked" : (stData == MH_OK ? "hooked" : "FAILED"), free);
}

// IDirectInput8::CreateDevice (vtable 3), A and W share the layout.
typedef HRESULT(STDMETHODCALLTYPE* DiCreateDevice_t)(void*, REFGUID, void**, LPUNKNOWN);
DiCreateDevice_t oCreateDeviceA = nullptr, oCreateDeviceW = nullptr;

void NoteCreatedDevice(REFGUID guid, void** out, HRESULT hr)
{
    if (FAILED(hr) || !out || !*out)
        return;
    const char* what = IsEqualGUID(guid, GUID_SysMouse) ? "game created a mouse device"
        : IsEqualGUID(guid, GUID_SysKeyboard)           ? "game created a keyboard device"
                                                        : "game created a DirectInput device";
    HookDeviceVtable(*out, what);
}
HRESULT STDMETHODCALLTYPE hkCreateDeviceA(void* self, REFGUID guid, void** out, LPUNKNOWN outer)
{
    const HRESULT hr = oCreateDeviceA(self, guid, out, outer);
    NoteCreatedDevice(guid, out, hr);
    return hr;
}
HRESULT STDMETHODCALLTYPE hkCreateDeviceW(void* self, REFGUID guid, void** out, LPUNKNOWN outer)
{
    const HRESULT hr = oCreateDeviceW(self, guid, out, outer);
    NoteCreatedDevice(guid, out, hr);
    return hr;
}

// ---- XInput ------------------------------------------------------------
typedef DWORD(WINAPI* XInputGetState_t)(DWORD, XINPUT_STATE*);
XInputGetState_t oXInputGetState = nullptr; // the game's xinput1_3, trampoline once hooked
XInputGetState_t g_padReader = nullptr;     // what the menu reads through (never blocked)

typedef DWORD(WINAPI* XInputGetCapabilities_t)(DWORD, DWORD, XINPUT_CAPABILITIES*);
typedef DWORD(WINAPI* XInputSetState_t)(DWORD, XINPUT_VIBRATION*);
XInputGetCapabilities_t oXInputGetCapabilities = nullptr;
XInputSetState_t oXInputSetState = nullptr;

// Finding a pad is two questions, not one (2026-09-16). RE5 polls
// XInputGetState about once a second looking for a pad that has been plugged
// in; we answered yes every time and it carried on ignoring us, because a
// game that finds a pad then asks what it is. Only XInputGetState was hooked,
// so XInputGetCapabilities still said nothing is there, and the game believed
// the second answer. Now both agree: a plain wired pad on slot 0.
DWORD WINAPI hkXInputGetCapabilities(DWORD index, DWORD flags, XINPUT_CAPABILITIES* caps)
{
    const DWORD r = oXInputGetCapabilities(index, flags, caps);
    if (r == ERROR_SUCCESS || !caps || index != 0)
        return r;

    XINPUT_GAMEPAD probe = {};
    if (!XrInput_GetPad(&probe))
        return r;

    // What a wired Xbox 360 pad reports: every standard control present, both
    // triggers and both sticks at full range, rumble on both motors.
    XINPUT_CAPABILITIES out = {};
    out.Type = XINPUT_DEVTYPE_GAMEPAD;
    out.SubType = XINPUT_DEVSUBTYPE_GAMEPAD;
    out.Flags = 0;
    out.Gamepad.wButtons = XINPUT_GAMEPAD_DPAD_UP | XINPUT_GAMEPAD_DPAD_DOWN | XINPUT_GAMEPAD_DPAD_LEFT
        | XINPUT_GAMEPAD_DPAD_RIGHT | XINPUT_GAMEPAD_START | XINPUT_GAMEPAD_BACK | XINPUT_GAMEPAD_LEFT_THUMB
        | XINPUT_GAMEPAD_RIGHT_THUMB | XINPUT_GAMEPAD_LEFT_SHOULDER | XINPUT_GAMEPAD_RIGHT_SHOULDER
        | XINPUT_GAMEPAD_A | XINPUT_GAMEPAD_B | XINPUT_GAMEPAD_X | XINPUT_GAMEPAD_Y;
    out.Gamepad.bLeftTrigger = 0xFF;
    out.Gamepad.bRightTrigger = 0xFF;
    out.Gamepad.sThumbLX = static_cast<SHORT>(0xFFC0);
    out.Gamepad.sThumbLY = static_cast<SHORT>(0xFFC0);
    out.Gamepad.sThumbRX = static_cast<SHORT>(0xFFC0);
    out.Gamepad.sThumbRY = static_cast<SHORT>(0xFFC0);
    out.Vibration.wLeftMotorSpeed = 0xFF;
    out.Vibration.wRightMotorSpeed = 0xFF;
    *caps = out;

    static bool logged = false;
    if (!logged) {
        logged = true;
        Log_Printf("InputBlock: told the game slot 0 is a gamepad (motion controllers standing in for one)");
    }
    return ERROR_SUCCESS;
}

// Rumble sent to a pad that isn't there. Saying it worked keeps a game from
// deciding the pad went away.
DWORD WINAPI hkXInputSetState(DWORD index, XINPUT_VIBRATION* vibration)
{
    const DWORD r = oXInputSetState(index, vibration);
    if (r == ERROR_SUCCESS || index != 0)
        return r;
    XINPUT_GAMEPAD probe = {};
    return XrInput_GetPad(&probe) ? ERROR_SUCCESS : r;
}

// Motion controllers, merged into whatever the real pad said (v0.4.3). Same
// idea as UEVR: OR the buttons in and add the sticks, so a real pad and the
// controllers work side by side. Slot 0 only, and only when a real pad hasn't
// already claimed that slot.
bool MergeMotionPad(XINPUT_GAMEPAD& g)
{
    XINPUT_GAMEPAD vr = {};
    if (!XrInput_GetPad(&vr))
        return false;

    g.wButtons |= vr.wButtons;
    if (vr.bLeftTrigger > g.bLeftTrigger)
        g.bLeftTrigger = vr.bLeftTrigger;
    if (vr.bRightTrigger > g.bRightTrigger)
        g.bRightTrigger = vr.bRightTrigger;
    const auto add = [](SHORT a, SHORT b) {
        const long sum = static_cast<long>(a) + static_cast<long>(b);
        return static_cast<SHORT>(sum < -32767 ? -32767 : (sum > 32767 ? 32767 : sum));
    };
    g.sThumbLX = add(g.sThumbLX, vr.sThumbLX);
    g.sThumbLY = add(g.sThumbLY, vr.sThumbLY);
    g.sThumbRX = add(g.sThumbRX, vr.sThumbRX);
    g.sThumbRY = add(g.sThumbRY, vr.sThumbRY);

    // 3DOF aiming: the servo's push, so the gun's pitch catches up with where
    // the controller points. It replaces the stick rather than adding to it,
    // and only while it has something to ask for, so your own stick still
    // wins the moment the gun is where you want it.
    float aimY = 0.0f;
    if (CameraRigHook_GetAimStickY(&aimY)) {
        const SHORT servo = static_cast<SHORT>(aimY * 32767.0f);
        if (std::abs(static_cast<int>(servo)) > std::abs(static_cast<int>(g.sThumbRY)))
            g.sThumbRY = servo;
    }
    return true;
}

DWORD WINAPI hkXInputGetState(DWORD index, XINPUT_STATE* state)
{
    const DWORD r = oXInputGetState(index, state);

    // Does RE5 even ask for a pad, and does it get ours? Once every 5 s while
    // motion controllers are in hand, and silent otherwise (2026-09-16).
    {
        static unsigned long s_calls = 0, s_merged = 0, s_synth = 0;
        static ULONGLONG s_lastMs = 0;
        ++s_calls;
        const ULONGLONG nowMs = GetTickCount64();
        XINPUT_GAMEPAD probe = {};
        const bool motionLive = XrInput_GetPad(&probe);
        if (motionLive && nowMs - s_lastMs > 5000) {
            s_lastMs = nowMs;
            Log_Printf("InputBlock: the game read XInput %lu times in the last stretch (slot %lu last), "
                       "%lu merged with the motion pad, %lu answered as a pad that isn't plugged in",
                s_calls, index, s_merged, s_synth);
            s_calls = s_merged = s_synth = 0;
        }
        if (motionLive && index == 0) {
            if (r == ERROR_SUCCESS)
                ++s_merged;
            else
                ++s_synth;
        }
    }

    // No pad plugged in, but motion controllers in hand: the game is told
    // slot 0 has a pad, which is how RE5 comes to believe in them at all.
    if (r != ERROR_SUCCESS && state && index == 0) {
        // Through MergeMotionPad, not XrInput_GetPad (2026-09-16): the merge
        // is also where the aim servo's stick goes in, and this path used to
        // skip it. With no pad plugged in this is the only path, so aiming by
        // pointing asked for full stick and the game never saw a thing.
        XINPUT_STATE vr = {};
        if (MergeMotionPad(vr.Gamepad)) {
            static DWORD s_packet = 0;
            vr.dwPacketNumber = ++s_packet;
            *state = vr;
            if (g_blocking.load(std::memory_order_relaxed))
                std::memset(&state->Gamepad, 0, sizeof(state->Gamepad));
            return ERROR_SUCCESS;
        }
    }

    if (r != ERROR_SUCCESS || !state || index >= XUSER_MAX_COUNT)
        return r;
    XINPUT_GAMEPAD& g = state->Gamepad;
    if (index == 0)
        MergeMotionPad(g);
    if (g_blocking.load(std::memory_order_relaxed)) {
        std::memset(&g, 0, sizeof(g));
        return r;
    }
    if (g_capturePadMask[index].exchange(false, std::memory_order_acquire)) {
        g_padButtonMask[index] = g.wButtons;
        g_padTriggerMask[index][0] = g.bLeftTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
        g_padTriggerMask[index][1] = g.bRightTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
    }
    g_padButtonMask[index] &= g.wButtons; // released buttons leave the mask
    g.wButtons &= ~g_padButtonMask[index];
    const auto trigger = [](bool& masked, BYTE& value) {
        if (!masked)
            return;
        if (value <= XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
            masked = false;
        else
            value = 0;
    };
    trigger(g_padTriggerMask[index][0], g.bLeftTrigger);
    trigger(g_padTriggerMask[index][1], g.bRightTrigger);
    return r;
}

bool g_installed = false;

void InstallDirectInput()
{
    HMODULE dinput = LoadLibraryA("dinput8.dll");
    using Create_t = HRESULT(WINAPI*)(HINSTANCE, DWORD, REFIID, LPVOID*, LPUNKNOWN);
    const Create_t create = dinput ? reinterpret_cast<Create_t>(GetProcAddress(dinput, "DirectInput8Create")) : nullptr;
    if (!create) {
        Log_Printf("InputBlock: dinput8.dll / DirectInput8Create not found - keyboard and mouse will reach the game "
                   "while the menu is open");
        return;
    }
    const HINSTANCE self = GetModuleHandleA(nullptr);

    const auto probe = [&](REFIID iid, const char* flavour, DiCreateDevice_t detour, DiCreateDevice_t* original) {
        IUnknown* di = nullptr;
        if (FAILED(create(self, DIRECTINPUT_VERSION, iid, reinterpret_cast<LPVOID*>(&di), nullptr)) || !di)
            return;
        void** diVt = *reinterpret_cast<void***>(di);
        const auto createDevice = reinterpret_cast<DiCreateDevice_t>(diVt[3]);
        char what[64];
        void* dev = nullptr;
        if (SUCCEEDED(createDevice(di, GUID_SysKeyboard, &dev, nullptr)) && dev) {
            snprintf(what, sizeof(what), "probe keyboard (%s)", flavour);
            HookDeviceVtable(dev, what);
            static_cast<IUnknown*>(dev)->Release();
        }
        dev = nullptr;
        if (SUCCEEDED(createDevice(di, GUID_SysMouse, &dev, nullptr)) && dev) {
            snprintf(what, sizeof(what), "probe mouse (%s)", flavour);
            HookDeviceVtable(dev, what);
            static_cast<IUnknown*>(dev)->Release();
        }
        MH_STATUS st = MH_CreateHook(diVt[3], reinterpret_cast<void*>(detour), reinterpret_cast<void**>(original));
        if (st == MH_OK)
            st = MH_EnableHook(diVt[3]);
        Log_Printf("InputBlock: IDirectInput8%s::CreateDevice hook -> %d", flavour, static_cast<int>(st));
        di->Release();
    };
    probe(IID_IDirectInput8A, "A", hkCreateDeviceA, &oCreateDeviceA);
    probe(IID_IDirectInput8W, "W", hkCreateDeviceW, &oCreateDeviceW);
}

void HookExport(HMODULE module, const char* name, void* detour, void** original);

void InstallXInput()
{
    // The game's own XInput. Hooking the export patches the function itself,
    // so it covers the game however it imported it (by name or ordinal).
    HMODULE game = GetModuleHandleA("xinput1_3.dll");
    if (!game)
        game = LoadLibraryA("xinput1_3.dll");
    const void* target = game ? GetProcAddress(game, "XInputGetState") : nullptr;
    if (target) {
        MH_STATUS st = MH_CreateHook(const_cast<void*>(target), reinterpret_cast<void*>(&hkXInputGetState),
            reinterpret_cast<void**>(&oXInputGetState));
        if (st == MH_OK)
            st = MH_EnableHook(const_cast<void*>(target));
        Log_Printf("InputBlock: xinput1_3 XInputGetState hook -> %d", static_cast<int>(st));
        if (st == MH_OK)
            g_padReader = oXInputGetState;
    } else {
        Log_Printf("InputBlock: xinput1_3.dll not found - the pad will reach the game while the menu is open");
    }
    // The other half of "is there a pad": see hkXInputGetCapabilities.
    if (game) {
        HookExport(game, "XInputGetCapabilities", reinterpret_cast<void*>(&hkXInputGetCapabilities),
            reinterpret_cast<void**>(&oXInputGetCapabilities));
        HookExport(game, "XInputSetState", reinterpret_cast<void*>(&hkXInputSetState),
            reinterpret_cast<void**>(&oXInputSetState));
    }

    if (!g_padReader) {
        static const char* const kDlls[] = { "xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll" };
        for (const char* name : kDlls) {
            HMODULE m = LoadLibraryA(name);
            if (m && (g_padReader = reinterpret_cast<XInputGetState_t>(GetProcAddress(m, "XInputGetState"))) != nullptr)
                break;
        }
    }
}

// ---- Win32: the cursor, key state and the message pump (2026-09-13) ----
// The first build only blocked DirectInput, and in play the mouse still moved
// the game's camera and clicked its menus straight through the open menu. The
// log said why: no DirectInput MOUSE was ever read. The unpacked exe imports
// GetCursorPos, GetKeyState and GetKeyboardState plus PeekMessage/GetMessage,
// so RE5 takes its mouse from Windows itself.
//
// Each of these is blocked only for calls made FROM THE GAME's own code (the
// return address lies inside re5dx9.exe). Our menu, dgVoodoo and the runtime
// keep seeing the real thing - the menu needs the real cursor to draw its own.
uintptr_t g_gameBegin = 0, g_gameEnd = 0;

bool CalledFromGame(void* returnAddress)
{
    const uintptr_t a = reinterpret_cast<uintptr_t>(returnAddress);
    return a >= g_gameBegin && a < g_gameEnd;
}

// What the game has asked for while the menu was open, logged when it closes,
// so the next report says exactly which route the input took.
struct BlockCounts {
    std::atomic<unsigned long> cursor{ 0 }, keyState{ 0 }, keyboardState{ 0 }, asyncKey{ 0 }, messages{ 0 }, sent{ 0 };
};
BlockCounts g_counts;

POINT g_frozenCursor = {};                  // what the game sees while blocked
std::atomic<bool> g_captureWinKeyMask{ false };
BYTE g_winKeyMask[256] = {};                // held at close, hidden until released

void (*g_messageSink)(UINT, WPARAM, LPARAM) = nullptr;

bool IsInputMessage(UINT m)
{
    // WM_SYSKEY* stay with the game so Alt+Tab and Alt+F4 keep working.
    return m == WM_KEYDOWN || m == WM_KEYUP || m == WM_CHAR || m == WM_DEADCHAR ||
        (m >= WM_MOUSEFIRST && m <= WM_MOUSELAST) || m == WM_INPUT;
}

typedef BOOL(WINAPI* GetCursorPos_t)(LPPOINT);
typedef SHORT(WINAPI* GetKeyState_t)(int);
typedef BOOL(WINAPI* GetKeyboardState_t)(PBYTE);
typedef BOOL(WINAPI* PeekMessage_t)(LPMSG, HWND, UINT, UINT, UINT);
typedef BOOL(WINAPI* GetMessage_t)(LPMSG, HWND, UINT, UINT);
typedef HCURSOR(WINAPI* SetCursor_t)(HCURSOR);
SetCursor_t oSetCursor = nullptr;
// The cursor the game last asked Windows for - its menu pointer graphic.
std::atomic<HCURSOR> g_gameCursor{ nullptr };

// The game sets its own pointer over and over on its menus. While our menu
// is open that re-showed the Windows cursor between our "no cursor" answers,
// which flickered (user, 2026-09-13). Game requests are now remembered - that
// handle is the graphic the menu draws - and turned into "no cursor".
HCURSOR WINAPI hkSetCursor(HCURSOR cursor)
{
    if (CalledFromGame(_ReturnAddress())) {
        if (cursor)
            g_gameCursor.store(cursor, std::memory_order_relaxed);
        if (g_blocking.load(std::memory_order_relaxed))
            return oSetCursor(nullptr);
    }
    return oSetCursor(cursor);
}
GetCursorPos_t oGetCursorPos = nullptr;
GetKeyState_t oGetKeyState = nullptr, oGetAsyncKeyState = nullptr;
GetKeyboardState_t oGetKeyboardState = nullptr;
PeekMessage_t oPeekMessageA = nullptr, oPeekMessageW = nullptr;
GetMessage_t oGetMessageA = nullptr, oGetMessageW = nullptr;

// Keys and buttons still held from the moment the menu closed read as up.
bool MaskedAfterClose(int vk)
{
    if (vk < 0 || vk > 255 || !g_winKeyMask[vk])
        return false;
    const SHORT real = oGetAsyncKeyState ? oGetAsyncKeyState(vk) : GetAsyncKeyState(vk);
    if (!(real & 0x8000)) {
        g_winKeyMask[vk] = 0;
        return false;
    }
    return true;
}

void CaptureWinKeyMaskIfPending()
{
    if (!g_captureWinKeyMask.exchange(false, std::memory_order_acquire))
        return;
    for (int vk = 1; vk < 256; ++vk) {
        const SHORT s = oGetAsyncKeyState ? oGetAsyncKeyState(vk) : GetAsyncKeyState(vk);
        g_winKeyMask[vk] = (s & 0x8000) ? 1 : 0;
    }
}

BOOL WINAPI hkGetCursorPos(LPPOINT p)
{
    if (p && g_blocking.load(std::memory_order_relaxed) && CalledFromGame(_ReturnAddress())) {
        g_counts.cursor.fetch_add(1, std::memory_order_relaxed);
        *p = g_frozenCursor;
        return TRUE;
    }
    return oGetCursorPos(p);
}

SHORT WINAPI hkGetKeyState(int vk)
{
    if (CalledFromGame(_ReturnAddress())) {
        if (g_blocking.load(std::memory_order_relaxed)) {
            g_counts.keyState.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }
        CaptureWinKeyMaskIfPending();
        if (MaskedAfterClose(vk))
            return 0;
    }
    return oGetKeyState(vk);
}

SHORT WINAPI hkGetAsyncKeyState(int vk)
{
    if (CalledFromGame(_ReturnAddress())) {
        if (g_blocking.load(std::memory_order_relaxed)) {
            g_counts.asyncKey.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }
        CaptureWinKeyMaskIfPending();
        if (MaskedAfterClose(vk))
            return 0;
    }
    return oGetAsyncKeyState(vk);
}

BOOL WINAPI hkGetKeyboardState(PBYTE keys)
{
    const BOOL r = oGetKeyboardState(keys);
    if (!r || !keys || !CalledFromGame(_ReturnAddress()))
        return r;
    if (g_blocking.load(std::memory_order_relaxed)) {
        g_counts.keyboardState.fetch_add(1, std::memory_order_relaxed);
        std::memset(keys, 0, 256);
        return r;
    }
    CaptureWinKeyMaskIfPending();
    for (int vk = 1; vk < 256; ++vk)
        if (g_winKeyMask[vk] && MaskedAfterClose(vk))
            keys[vk] = 0;
    return r;
}

// A keyboard or mouse message the game pulls off its queue while the menu is
// open goes to the menu instead, and the game gets a WM_NULL in its place.
// Only REMOVED messages go to the menu: a peek without PM_REMOVE sees the same
// message again later.
void DivertMessage(LPMSG msg, bool removed)
{
    if (!msg || !IsInputMessage(msg->message))
        return;
    g_counts.messages.fetch_add(1, std::memory_order_relaxed);
    if (removed && g_messageSink) {
        g_messageSink(msg->message, msg->wParam, msg->lParam);
        // The game never gets to translate a key it never sees, so do it here:
        // the WM_CHAR it posts comes through this same path to the menu, which
        // is what typing a value into a slider needs.
        if (msg->message == WM_KEYDOWN)
            TranslateMessage(msg);
    }
    msg->message = WM_NULL;
}

BOOL WINAPI hkPeekMessageA(LPMSG msg, HWND hwnd, UINT lo, UINT hi, UINT flags)
{
    const BOOL r = oPeekMessageA(msg, hwnd, lo, hi, flags);
    if (r && g_blocking.load(std::memory_order_relaxed) && CalledFromGame(_ReturnAddress()))
        DivertMessage(msg, (flags & PM_REMOVE) != 0);
    return r;
}
BOOL WINAPI hkPeekMessageW(LPMSG msg, HWND hwnd, UINT lo, UINT hi, UINT flags)
{
    const BOOL r = oPeekMessageW(msg, hwnd, lo, hi, flags);
    if (r && g_blocking.load(std::memory_order_relaxed) && CalledFromGame(_ReturnAddress()))
        DivertMessage(msg, (flags & PM_REMOVE) != 0);
    return r;
}
BOOL WINAPI hkGetMessageA(LPMSG msg, HWND hwnd, UINT lo, UINT hi)
{
    const BOOL r = oGetMessageA(msg, hwnd, lo, hi);
    if (r > 0 && g_blocking.load(std::memory_order_relaxed) && CalledFromGame(_ReturnAddress()))
        DivertMessage(msg, true);
    return r;
}
BOOL WINAPI hkGetMessageW(LPMSG msg, HWND hwnd, UINT lo, UINT hi)
{
    const BOOL r = oGetMessageW(msg, hwnd, lo, hi);
    if (r > 0 && g_blocking.load(std::memory_order_relaxed) && CalledFromGame(_ReturnAddress()))
        DivertMessage(msg, true);
    return r;
}

void HookExport(HMODULE module, const char* name, void* detour, void** original)
{
    void* target = module ? reinterpret_cast<void*>(GetProcAddress(module, name)) : nullptr;
    MH_STATUS st = MH_ERROR_FUNCTION_NOT_FOUND;
    if (target) {
        st = MH_CreateHook(target, detour, original);
        if (st == MH_OK)
            st = MH_EnableHook(target);
    }
    Log_Printf("InputBlock: %s hook -> %d", name, static_cast<int>(st));
}

void InstallWin32()
{
    const HMODULE exe = GetModuleHandleA(nullptr);
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(exe);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(reinterpret_cast<const BYTE*>(exe) + dos->e_lfanew);
    g_gameBegin = reinterpret_cast<uintptr_t>(exe);
    g_gameEnd = g_gameBegin + nt->OptionalHeader.SizeOfImage;

    HMODULE user32 = GetModuleHandleA("user32.dll");
    HookExport(user32, "GetCursorPos", reinterpret_cast<void*>(&hkGetCursorPos), reinterpret_cast<void**>(&oGetCursorPos));
    HookExport(user32, "GetKeyState", reinterpret_cast<void*>(&hkGetKeyState), reinterpret_cast<void**>(&oGetKeyState));
    HookExport(user32, "GetAsyncKeyState", reinterpret_cast<void*>(&hkGetAsyncKeyState),
        reinterpret_cast<void**>(&oGetAsyncKeyState));
    HookExport(user32, "GetKeyboardState", reinterpret_cast<void*>(&hkGetKeyboardState),
        reinterpret_cast<void**>(&oGetKeyboardState));
    HookExport(user32, "PeekMessageA", reinterpret_cast<void*>(&hkPeekMessageA), reinterpret_cast<void**>(&oPeekMessageA));
    HookExport(user32, "PeekMessageW", reinterpret_cast<void*>(&hkPeekMessageW), reinterpret_cast<void**>(&oPeekMessageW));
    HookExport(user32, "GetMessageA", reinterpret_cast<void*>(&hkGetMessageA), reinterpret_cast<void**>(&oGetMessageA));
    HookExport(user32, "GetMessageW", reinterpret_cast<void*>(&hkGetMessageW), reinterpret_cast<void**>(&oGetMessageW));
    HookExport(user32, "SetCursor", reinterpret_cast<void*>(&hkSetCursor), reinterpret_cast<void**>(&oSetCursor));
}

// ---- The mod's own DirectInput pad (2026-09-15) ------------------------
// XInput only sees XInput pads. A DualShock or DualSense without Steam Input,
// and most generic USB pads, speak DirectInput instead: they play the game
// fine, but the menu never saw them, so the both-sticks shortcut did nothing
// and the pad could not drive the menu. When no XInput pad answers we open our
// own DirectInput device and translate it into the XINPUT_STATE the rest of
// the menu already speaks. Ours is a separate, non-exclusive, background
// device, so the game keeps its own.
//
// Sony's DirectInput layout (DualShock 4 and DualSense, USB or Bluetooth):
//   buttons  0 square, 1 cross, 2 circle, 3 triangle, 4 L1, 5 R1, 6 L2, 7 R2,
//            8 Share/Create, 9 Options, 10 L3, 11 R3, 12 PS, 13 touchpad
//   axes     lX/lY left stick, lZ right stick X, lRz right stick Y
// Most other HID pads follow the same order, so it is the fallback too. The
// product name and VID/PID go in the log to sort out any that don't.
HWND g_padWindow = nullptr;
IDirectInput8A* g_di = nullptr;

DWORD g_diPacket = 0;

struct PadPick {
    GUID guid = {};
    char name[MAX_PATH] = {};
    DWORD vid = 0, pid = 0;
    bool sony = false;
    bool found = false;
};

BOOL CALLBACK EnumPadCallback(LPCDIDEVICEINSTANCEA inst, LPVOID context)
{
    auto* pick = static_cast<PadPick*>(context);
    const DWORD vid = inst->guidProduct.Data1 & 0xFFFF;
    const DWORD pid = (inst->guidProduct.Data1 >> 16) & 0xFFFF;
    const bool sony = vid == 0x054C;
    if (pick->found && !sony)
        return DIENUM_CONTINUE; // keep the first, unless a Sony pad turns up
    pick->guid = inst->guidInstance;
    lstrcpynA(pick->name, inst->tszProductName, MAX_PATH);
    pick->vid = vid;
    pick->pid = pid;
    pick->sony = sony;
    pick->found = true;
    return sony ? DIENUM_STOP : DIENUM_CONTINUE;
}

bool OpenDiPad()
{
    if (g_diPad)
        return true;
    if (!g_padWindow)
        return false;

    // Enumeration is not free, so try at most every 2 s.
    static ULONGLONG s_lastTryMs = 0;
    const ULONGLONG nowMs = GetTickCount64();
    if (s_lastTryMs && nowMs - s_lastTryMs < 2000)
        return false;
    s_lastTryMs = nowMs;

    if (!g_di) {
        HMODULE self = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&OpenDiPad), &self);
        if (FAILED(DirectInput8Create(self, DIRECTINPUT_VERSION, IID_IDirectInput8A,
                reinterpret_cast<void**>(&g_di), nullptr))) {
            g_di = nullptr;
            return false;
        }
    }

    PadPick pick;
    g_di->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumPadCallback, &pick, DIEDFL_ATTACHEDONLY);
    if (!pick.found)
        return false;

    IDirectInputDevice8A* dev = nullptr;
    if (FAILED(g_di->CreateDevice(pick.guid, &dev, nullptr)) || !dev)
        return false;
    if (FAILED(dev->SetDataFormat(&c_dfDIJoystick2))) {
        dev->Release();
        return false;
    }
    dev->SetCooperativeLevel(g_padWindow, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);

    // Every axis on the same scale, so the translation below is one formula.
    DIPROPRANGE range = {};
    range.diph.dwSize = sizeof(range);
    range.diph.dwHeaderSize = sizeof(range.diph);
    range.diph.dwHow = DIPH_DEVICE;
    range.lMin = -1000;
    range.lMax = 1000;
    dev->SetProperty(DIPROP_RANGE, &range.diph);
    dev->Acquire();

    g_diPad = dev;
    Log_Printf("InputBlock: no XInput pad, using the DirectInput pad \"%s\" (VID %04lX PID %04lX)%s", pick.name,
        pick.vid, pick.pid, pick.sony ? " - PlayStation layout" : " - assuming the usual layout");
    return true;
}

bool ReadDiPad(XINPUT_STATE* out)
{
    if (!OpenDiPad())
        return false;

    DIJOYSTATE2 js = {};
    HRESULT hr = g_diPad->Poll();
    if (hr == DIERR_INPUTLOST || hr == DIERR_NOTACQUIRED) {
        g_diPad->Acquire();
        g_diPad->Poll();
    }
    hr = g_diPad->GetDeviceState(sizeof(js), &js);
    if (FAILED(hr)) {
        if (hr == DIERR_INPUTLOST || hr == DIERR_NOTACQUIRED) {
            g_diPad->Acquire();
        } else {
            // Unplugged: let the next call look again.
            Log_Printf("InputBlock: the DirectInput pad stopped answering (hr=0x%08lX) - looking again", hr);
            g_diPad->Release();
            g_diPad = nullptr;
        }
        return false;
    }

    const auto down = [&js](int i) { return (js.rgbButtons[i] & 0x80) != 0; };

    // Which button is which, for pads that don't follow the usual order: the
    // first twenty presses go in the log with the index we read them at.
    {
        static BYTE s_prev[32] = {};
        static int s_left = 20;
        for (int i = 0; i < 32 && s_left > 0; ++i) {
            const BYTE now = js.rgbButtons[i] & 0x80;
            if (now && !s_prev[i]) {
                --s_left;
                Log_Printf("InputBlock: DirectInput pad button %d pressed%s", i,
                    i == 10 ? " (L3 on a PlayStation pad)" : i == 11 ? " (R3 on a PlayStation pad)" : "");
            }
            s_prev[i] = now;
        }
    }

    WORD b = 0;
    if (down(1))
        b |= XINPUT_GAMEPAD_A; // cross
    if (down(2))
        b |= XINPUT_GAMEPAD_B; // circle
    if (down(0))
        b |= XINPUT_GAMEPAD_X; // square
    if (down(3))
        b |= XINPUT_GAMEPAD_Y; // triangle
    if (down(4))
        b |= XINPUT_GAMEPAD_LEFT_SHOULDER;
    if (down(5))
        b |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
    if (down(8))
        b |= XINPUT_GAMEPAD_BACK; // Share / Create
    if (down(9))
        b |= XINPUT_GAMEPAD_START; // Options
    if (down(10))
        b |= XINPUT_GAMEPAD_LEFT_THUMB; // L3
    if (down(11))
        b |= XINPUT_GAMEPAD_RIGHT_THUMB; // R3

    // The hat, in hundredths of a degree clockwise from up. Centred is -1,
    // and some drivers only fill the low word.
    const DWORD pov = js.rgdwPOV[0];
    if ((pov & 0xFFFF) != 0xFFFF) {
        const int deg = static_cast<int>((pov / 100) % 360);
        if (deg >= 315 || deg <= 45)
            b |= XINPUT_GAMEPAD_DPAD_UP;
        if (deg >= 45 && deg <= 135)
            b |= XINPUT_GAMEPAD_DPAD_RIGHT;
        if (deg >= 135 && deg <= 225)
            b |= XINPUT_GAMEPAD_DPAD_DOWN;
        if (deg >= 225 && deg <= 315)
            b |= XINPUT_GAMEPAD_DPAD_LEFT;
    }

    const auto axis = [](LONG v) {
        const long scaled = v * 32767 / 1000;
        return static_cast<SHORT>(scaled < -32768 ? -32768 : (scaled > 32767 ? 32767 : scaled));
    };

    *out = XINPUT_STATE{};
    out->dwPacketNumber = ++g_diPacket;
    out->Gamepad.wButtons = b;
    // The triggers as buttons: the menu only needs them pressed or not, and
    // a pad without analogue trigger axes would otherwise read half-pulled.
    out->Gamepad.bLeftTrigger = down(6) ? 255 : 0;
    out->Gamepad.bRightTrigger = down(7) ? 255 : 0;
    out->Gamepad.sThumbLX = axis(js.lX);
    out->Gamepad.sThumbLY = axis(-js.lY); // DirectInput Y grows downward
    out->Gamepad.sThumbRX = axis(js.lZ);
    out->Gamepad.sThumbRY = axis(-js.lRz);
    return true;
}

} // namespace

void InputBlock_SetWindow(HWND hwnd)
{
    g_padWindow = hwnd;
}

void InputBlock_Install()
{
    if (g_installed)
        return;
    g_installed = true;
    const MH_STATUS init = MH_Initialize();
    if (init != MH_OK && init != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("InputBlock: MH_Initialize failed -> %d", static_cast<int>(init));
        return;
    }
    InstallDirectInput();
    InstallXInput();
    InstallWin32();
}

void InputBlock_SetBlocking(bool blocking)
{
    const bool was = g_blocking.exchange(blocking, std::memory_order_acq_rel);
    if (was == blocking)
        return;
    if (!blocking) {
        g_captureWinKeyMask.store(true, std::memory_order_release);
        Log_Printf("InputBlock: while blocked the game asked for - cursor %lu, GetKeyState %lu, GetAsyncKeyState %lu, "
                   "GetKeyboardState %lu, input messages %lu, DirectInput keyboard %lu / mouse %lu / other %lu "
                   "(all hidden from it)",
            g_counts.cursor.exchange(0), g_counts.keyState.exchange(0), g_counts.asyncKey.exchange(0),
            g_counts.keyboardState.exchange(0), g_counts.messages.exchange(0), g_diKeyboardReads.exchange(0),
            g_diMouseReads.exchange(0), g_diOtherReads.exchange(0));
        g_captureKeyMask.store(true, std::memory_order_release);
        g_captureMouseMask.store(true, std::memory_order_release);
        for (auto& c : g_capturePadMask)
            c.store(true, std::memory_order_release);
        g_captureDiPadMask.store(true, std::memory_order_release);
    } else {
        // The game keeps reading the cursor where it was when the menu opened.
        if (!(oGetCursorPos ? oGetCursorPos(&g_frozenCursor) : GetCursorPos(&g_frozenCursor)))
            g_frozenCursor = POINT{};
        AcquireSRWLockExclusive(&g_mouseLock);
        g_mouseDx = g_mouseDy = g_mouseWheel = 0;
        ReleaseSRWLockExclusive(&g_mouseLock);
    }
}

bool InputBlock_IsBlocking()
{
    return g_blocking.load(std::memory_order_relaxed);
}

namespace {

bool ReadXInputPad(XINPUT_STATE* out)
{
    if (!g_padReader)
        return false;
    static int s_pad = -1;
    static ULONGLONG s_lastScanMs = 0;
    static ULONGLONG s_firstScanMs = 0;
    static bool s_loggedNone = false;
    XINPUT_STATE st = {};
    if (s_pad >= 0 && g_padReader(static_cast<DWORD>(s_pad), &st) != ERROR_SUCCESS)
        s_pad = -1;
    if (s_pad < 0) {
        // Empty slots are slow to poll, so only rescan every 2 s.
        const ULONGLONG nowMs = GetTickCount64();
        if (!s_firstScanMs)
            s_firstScanMs = nowMs;
        if (nowMs - s_lastScanMs < 2000)
            return false;
        s_lastScanMs = nowMs;
        for (DWORD i = 0; i < XUSER_MAX_COUNT && s_pad < 0; ++i) {
            if (g_padReader(i, &st) == ERROR_SUCCESS)
                s_pad = static_cast<int>(i);
        }
        if (s_pad < 0)
            return false;
        Log_Printf("InputBlock: pad found on XInput slot %d", s_pad);
    }
    *out = st;
    return true;
}

} // namespace

bool InputBlock_ReadPad(XINPUT_STATE* out)
{
    // The menu reads past the block, so it sees motion controllers whether or
    // not a pad is plugged in: both stick clicks open it, the sticks move
    // through it, A selects.
    bool have = ReadXInputPad(out) || ReadDiPad(out);
    XINPUT_GAMEPAD motion = {};
    if (XrInput_GetPad(&motion)) {
        if (!have)
            *out = XINPUT_STATE{};
        MergeMotionPad(out->Gamepad);
        have = true;
    }
    if (have)
        return true;

    // Neither kind answered. Say so once, so a log explains a pad shortcut
    // that does nothing.
    static ULONGLONG s_firstMissMs = 0;
    static bool s_logged = false;
    const ULONGLONG nowMs = GetTickCount64();
    if (!s_firstMissMs)
        s_firstMissMs = nowMs;
    if (!s_logged && nowMs - s_firstMissMs > 15000) {
        s_logged = true;
        Log_Printf("InputBlock: no pad on XInput or DirectInput after 15 s - the both-sticks shortcut has nothing "
                   "to read; Insert still opens the menu");
    }
    return false;
}

void InputBlock_TakeMouse(InputBlockMouse& out)
{
    out = InputBlockMouse{};
    AcquireSRWLockExclusive(&g_mouseLock);
    out.dx = g_mouseDx;
    out.dy = g_mouseDy;
    out.wheel = g_mouseWheel;
    for (int i = 0; i < 3; ++i)
        out.buttons[i] = g_mouseButtons[i];
    g_mouseDx = g_mouseDy = g_mouseWheel = 0;
    ReleaseSRWLockExclusive(&g_mouseLock);
    const ULONGLONG last = g_lastDiMouseMs.load(std::memory_order_relaxed);
    out.fromDirectInput = last && GetTickCount64() - last < 500;
}

void InputBlock_SetMessageSink(void (*sink)(UINT msg, WPARAM w, LPARAM l))
{
    g_messageSink = sink;
}

void InputBlock_DescribeMouse(char* out, size_t size)
{
    snprintf(out, size, "DI mouse state reads %lu (cb %ld, last x %ld y %ld), buffered reads %lu with %lu item(s), offsets 0x%08lX",
        g_dbgStateCalls.exchange(0), g_dbgStateCb.load(), g_dbgStateX.load(), g_dbgStateY.load(),
        g_dbgDataCalls.exchange(0), g_dbgDataItems.exchange(0), g_dbgDataOfsMask.exchange(0));
}

HCURSOR InputBlock_GameCursor()
{
    return g_gameCursor.load(std::memory_order_relaxed);
}
