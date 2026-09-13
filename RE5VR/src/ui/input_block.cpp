#include "input_block.h"
#include "../util/log.h"

#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#include <MinHook.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <intrin.h>

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

HRESULT FilterDeviceState(IDirectInputDevice8A* dev, DWORD cb, LPVOID data, HRESULT hr)
{
    if (FAILED(hr) || !data)
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
    if (kind != DI8DEVTYPE_MOUSE && kind != DI8DEVTYPE_KEYBOARD)
        return hr;
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

DWORD WINAPI hkXInputGetState(DWORD index, XINPUT_STATE* state)
{
    const DWORD r = oXInputGetState(index, state);
    if (r != ERROR_SUCCESS || !state || index >= XUSER_MAX_COUNT)
        return r;
    XINPUT_GAMEPAD& g = state->Gamepad;
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

} // namespace

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

bool InputBlock_ReadPad(XINPUT_STATE* out)
{
    if (!g_padReader)
        return false;
    static int s_pad = -1;
    static ULONGLONG s_lastScanMs = 0;
    XINPUT_STATE st = {};
    if (s_pad >= 0 && g_padReader(static_cast<DWORD>(s_pad), &st) != ERROR_SUCCESS)
        s_pad = -1;
    if (s_pad < 0) {
        // Empty slots are slow to poll, so only rescan every 2 s.
        const ULONGLONG nowMs = GetTickCount64();
        if (nowMs - s_lastScanMs < 2000)
            return false;
        s_lastScanMs = nowMs;
        for (DWORD i = 0; i < XUSER_MAX_COUNT && s_pad < 0; ++i) {
            if (g_padReader(i, &st) == ERROR_SUCCESS)
                s_pad = static_cast<int>(i);
        }
        if (s_pad < 0)
            return false;
    }
    *out = st;
    return true;
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
