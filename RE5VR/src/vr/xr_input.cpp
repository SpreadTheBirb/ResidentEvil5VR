#include "xr_input.h"

#include "../util/build_config.h"
#include "../hooks/ammo.h"
#include "../util/sound.h"
#include "../util/log.h"
#include "../render/mat3.h"
#include "openxr_bridge.h"
#include "../hooks/arm_ik.h"
#include "../render/stereo_test.h"

#define XR_USE_PLATFORM_WIN32
#include <windows.h>
#include <openxr/openxr.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

// See xr_input.h for what this is and how the controls map. This file is only
// the OpenXR half: an action set, suggested bindings for every controller we
// know, a sync each frame, and a snapshot for the input block to merge into
// the game's pad.

namespace {

constexpr int kLeft = 0;
constexpr int kRight = 1;

XrInstance g_instance = XR_NULL_HANDLE;
XrSession g_session = XR_NULL_HANDLE;
XrActionSet g_actionSet = XR_NULL_HANDLE;
bool g_attached = false;

XrPath g_handPath[2] = {};

// One action per control, each with both hands as subaction paths: the hands
// then swap for left-handed play by swapping two indices.
XrAction g_trigger = XR_NULL_HANDLE;    // float
XrAction g_squeeze = XR_NULL_HANDLE;    // float (grip)
// The raw grip, published so a chord elsewhere can ask about it. The hand state
// itself is a local inside the pad build and nothing outside could see it.
std::atomic<float> g_gripNow[2] = {};
XrAction g_stick = XR_NULL_HANDLE;      // vector2
XrAction g_stickClick = XR_NULL_HANDLE; // bool
XrAction g_primary = XR_NULL_HANDLE;    // bool: A right, X left
XrAction g_secondary = XR_NULL_HANDLE;  // bool: B right, Y left
XrAction g_menu = XR_NULL_HANDLE;       // bool
XrAction g_thumbrest = XR_NULL_HANDLE;  // bool (touch), Touch controllers only
XrAction g_aimPose = XR_NULL_HANDLE;    // pose: where the controller points
XrAction g_haptic = XR_NULL_HANDLE;     // output: the only thing we send BACK
XrSpace g_aimSpace[2] = { XR_NULL_HANDLE, XR_NULL_HANDLE };

SRWLOCK g_lock = SRWLOCK_INIT;
XrInputSettings g_settings;
XINPUT_GAMEPAD g_pad = {};
ULONGLONG g_padMs = 0; // when the pad below was last filled in
char g_status[160] = "off";
// Where the gun hand points, in the reference space. Degrees, and only as
// fresh as the last sync (3DOF aiming, v0.4.3).
float g_gunYawDeg = 0.0f, g_gunPitchDeg = 0.0f;
ULONGLONG g_gunAimMs = 0;

// Both hands, whole (2026-09-17). 3DOF only ever needed the gun hand's
// heading; arm IK needs where each hand IS, and which way it is turned, in the
// runtime's raw tracking space - the same space VRBridge_GetTrackingFrame
// describes, so the two can be read off against each other.
XrHandPose g_handPose[2] = {};
// The knife, held out by the over-the-shoulder holster rather than by the
// player's own grip. RE5's knife is a HOLD on the left shoulder and not an
// inventory slot, so there is no press to send: the only way to draw one from
// a gesture is to keep holding the button afterwards. See the holster block.
bool g_knifeLatched = false;
volatile bool g_twoHandHeld = false;
volatile bool g_reloadCarrying = false;
// Carrying and reloading are separate: the first ends when the magazine
// touches the weapon, the second when the game has finished putting it in.
// Only the first should write the magazine bone.
volatile bool g_magInHand = false;
volatile bool g_gameReloading = false;
// What was still in the magazine when it came out. Not spent, not banked -
// remembered, and written back into the weapon for the single frame before
// the game is asked to reload, so the game tops the magazine up instead of
// filling it, and bills the inventory for the difference. It is the only way
// to keep those rounds without writing an inventory we cannot reach.
volatile long g_ejectedRounds = 0;
// Set when the magazine reaches the weapon, read a few hundred lines later
// where the pad is assembled. See the reload block for why the game is asked
// to do the loading rather than being told what the answer is.
ULONGLONG g_reloadAskAt = 0;
// When the magazine came out. The trigger is dead until something puts one
// back, and this is the only window in which the mod touches the trigger at
// all - outside it, an empty weapon behaves exactly as the game intends.
ULONGLONG g_magOutAt = 0;
volatile unsigned long long g_lastShotMs = 0;
volatile bool g_twoHandCupped = false;
// The twist the gun did not take, for the support hand to keep.
float g_supportRoll[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
volatile bool g_haveSupportRoll = false;
// Where the gun hand points, against where the head looks.
float g_menuPointX = 0.0f, g_menuPointY = 0.0f;
volatile bool g_haveMenuPoint = false;
volatile bool g_menuClick = false, g_menuRightClick = false;
float g_menuScroll = 0.0f;
float g_menuHandX = 0.0f, g_menuHandY = 0.0f;
volatile bool g_haveMenuHand = false;

const char* XrName(XrResult r)
{
    static char buf[XR_MAX_RESULT_STRING_SIZE];
    if (g_instance != XR_NULL_HANDLE && XR_SUCCEEDED(xrResultToString(g_instance, r, buf)))
        return buf;
    static char fallback[32];
    _snprintf_s(fallback, sizeof(fallback), _TRUNCATE, "XrResult %d", static_cast<int>(r));
    return fallback;
}

XrPath Path(const char* text)
{
    XrPath p = XR_NULL_PATH;
    const XrResult r = xrStringToPath(g_instance, text, &p);
    if (XR_FAILED(r))
        Log_Printf("XrInput: xrStringToPath(%s) -> %s", text, XrName(r));
    return p;
}

// A quaternion's forward direction. OpenXR looks down -Z, so this is the
// rotated -Z axis, which for an aim pose is the way the controller points.
void PoseForward(const XrQuaternionf& q, float out[3])
{
    out[0] = -2.0f * (q.x * q.z + q.w * q.y);
    out[1] = -2.0f * (q.y * q.z - q.w * q.x);
    out[2] = -(1.0f - 2.0f * (q.x * q.x + q.y * q.y));
}

XrAction CreateAction(XrActionType type, const char* name, const char* localized)
{
    XrActionCreateInfo info = { XR_TYPE_ACTION_CREATE_INFO };
    info.actionType = type;
    strcpy_s(info.actionName, name);
    strcpy_s(info.localizedActionName, localized);
    info.countSubactionPaths = 2;
    info.subactionPaths = g_handPath;
    XrAction action = XR_NULL_HANDLE;
    const XrResult r = xrCreateAction(g_actionSet, &info, &action);
    if (XR_FAILED(r)) {
        Log_Printf("XrInput: xrCreateAction(%s) failed -> %s", name, XrName(r));
        return XR_NULL_HANDLE;
    }
    return action;
}

struct Binding {
    XrAction action;
    const char* path;
};

// Suggest one controller's bindings. A runtime is free to ignore or remap
// these - SteamVR in particular lets the player rebind - so this is the
// default, not a decree.
void Suggest(const char* profile, const Binding* bindings, int count)
{
    XrActionSuggestedBinding list[32] = {};
    int n = 0;
    for (int i = 0; i < count && n < 32; ++i) {
        if (!bindings[i].action)
            continue;
        const XrPath p = Path(bindings[i].path);
        if (p == XR_NULL_PATH)
            continue;
        list[n].action = bindings[i].action;
        list[n].binding = p;
        ++n;
    }

    XrInteractionProfileSuggestedBinding suggestion = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestion.interactionProfile = Path(profile);
    suggestion.countSuggestedBindings = static_cast<uint32_t>(n);
    suggestion.suggestedBindings = list;
    const XrResult r = xrSuggestInteractionProfileBindings(g_instance, &suggestion);
    // A runtime that doesn't know a profile says so and carries on; that is
    // normal, not a failure of ours.
    Log_Printf("XrInput: %d bindings for %s -> %s", n, profile, XrName(r));
}

void SuggestEveryProfile()
{
    {
        const Binding b[] = {
            { g_trigger, "/user/hand/left/input/trigger/value" },
            { g_trigger, "/user/hand/right/input/trigger/value" },
            { g_squeeze, "/user/hand/left/input/squeeze/value" },
            { g_squeeze, "/user/hand/right/input/squeeze/value" },
            { g_stick, "/user/hand/left/input/thumbstick" },
            { g_stick, "/user/hand/right/input/thumbstick" },
            { g_stickClick, "/user/hand/left/input/thumbstick/click" },
            { g_stickClick, "/user/hand/right/input/thumbstick/click" },
            { g_primary, "/user/hand/left/input/x/click" },
            { g_primary, "/user/hand/right/input/a/click" },
            { g_secondary, "/user/hand/left/input/y/click" },
            { g_secondary, "/user/hand/right/input/b/click" },
            { g_menu, "/user/hand/left/input/menu/click" },
            { g_aimPose, "/user/hand/left/input/aim/pose" },
            { g_aimPose, "/user/hand/right/input/aim/pose" },
            { g_thumbrest, "/user/hand/left/input/thumbrest/touch" },
            { g_thumbrest, "/user/hand/right/input/thumbrest/touch" },
            { g_haptic, "/user/hand/left/output/haptic" },
            { g_haptic, "/user/hand/right/output/haptic" },
        };
        Suggest("/interaction_profiles/oculus/touch_controller", b, _countof(b));
    }
    {
        // Index: no thumbrest, and no menu button either (the system button
        // belongs to SteamVR), so a firm press on the trackpad is Start.
        const Binding b[] = {
            { g_trigger, "/user/hand/left/input/trigger/value" },
            { g_trigger, "/user/hand/right/input/trigger/value" },
            { g_squeeze, "/user/hand/left/input/squeeze/value" },
            { g_squeeze, "/user/hand/right/input/squeeze/value" },
            { g_stick, "/user/hand/left/input/thumbstick" },
            { g_stick, "/user/hand/right/input/thumbstick" },
            { g_stickClick, "/user/hand/left/input/thumbstick/click" },
            { g_stickClick, "/user/hand/right/input/thumbstick/click" },
            { g_primary, "/user/hand/left/input/a/click" },
            { g_primary, "/user/hand/right/input/a/click" },
            { g_secondary, "/user/hand/left/input/b/click" },
            { g_secondary, "/user/hand/right/input/b/click" },
            { g_menu, "/user/hand/left/input/trackpad/force" },
            { g_aimPose, "/user/hand/left/input/aim/pose" },
            { g_aimPose, "/user/hand/right/input/aim/pose" },
            { g_haptic, "/user/hand/left/output/haptic" },
            { g_haptic, "/user/hand/right/output/haptic" },
        };
        Suggest("/interaction_profiles/valve/index_controller", b, _countof(b));
    }
    {
        // Vive wands: a trackpad instead of a stick, and no face buttons.
        const Binding b[] = {
            { g_trigger, "/user/hand/left/input/trigger/value" },
            { g_trigger, "/user/hand/right/input/trigger/value" },
            { g_squeeze, "/user/hand/left/input/squeeze/click" },
            { g_squeeze, "/user/hand/right/input/squeeze/click" },
            { g_stick, "/user/hand/left/input/trackpad" },
            { g_stick, "/user/hand/right/input/trackpad" },
            { g_stickClick, "/user/hand/left/input/trackpad/click" },
            { g_stickClick, "/user/hand/right/input/trackpad/click" },
            { g_menu, "/user/hand/left/input/menu/click" },
            { g_aimPose, "/user/hand/left/input/aim/pose" },
            { g_aimPose, "/user/hand/right/input/aim/pose" },
            { g_haptic, "/user/hand/left/output/haptic" },
            { g_haptic, "/user/hand/right/output/haptic" },
        };
        Suggest("/interaction_profiles/htc/vive_controller", b, _countof(b));
    }
    {
        // Windows Mixed Reality: the trackpad click stands in for A and X.
        const Binding b[] = {
            { g_trigger, "/user/hand/left/input/trigger/value" },
            { g_trigger, "/user/hand/right/input/trigger/value" },
            { g_squeeze, "/user/hand/left/input/squeeze/click" },
            { g_squeeze, "/user/hand/right/input/squeeze/click" },
            { g_stick, "/user/hand/left/input/thumbstick" },
            { g_stick, "/user/hand/right/input/thumbstick" },
            { g_stickClick, "/user/hand/left/input/thumbstick/click" },
            { g_stickClick, "/user/hand/right/input/thumbstick/click" },
            { g_primary, "/user/hand/left/input/trackpad/click" },
            { g_primary, "/user/hand/right/input/trackpad/click" },
            { g_menu, "/user/hand/left/input/menu/click" },
            { g_aimPose, "/user/hand/left/input/aim/pose" },
            { g_aimPose, "/user/hand/right/input/aim/pose" },
            { g_haptic, "/user/hand/left/output/haptic" },
            { g_haptic, "/user/hand/right/output/haptic" },
        };
        Suggest("/interaction_profiles/microsoft/motion_controller", b, _countof(b));
    }
    {
        // The fallback every runtime must know: one button and a menu button.
        const Binding b[] = {
            { g_trigger, "/user/hand/left/input/select/click" },
            { g_trigger, "/user/hand/right/input/select/click" },
            { g_menu, "/user/hand/left/input/menu/click" },
            { g_aimPose, "/user/hand/left/input/aim/pose" },
            { g_aimPose, "/user/hand/right/input/aim/pose" },
            { g_haptic, "/user/hand/left/output/haptic" },
            { g_haptic, "/user/hand/right/output/haptic" },
        };
        Suggest("/interaction_profiles/khr/simple_controller", b, _countof(b));
    }
}

// ---- Reading one frame -------------------------------------------------

struct HandState {
    float trigger = 0.0f;
    float squeeze = 0.0f;
    float stickX = 0.0f, stickY = 0.0f;
    bool stickClick = false;
    bool primary = false;
    bool secondary = false;
    bool menu = false;
    bool thumbrest = false;
    bool anyActive = false;
    // Which actions the runtime actually bound to something on this hand. An
    // action can be bound yet always read zero, and it can be unbound while
    // its neighbours work, so this is the difference between the two.
    unsigned bound = 0;
};

enum BoundBit {
    kBoundTrigger = 1 << 0,
    kBoundSqueeze = 1 << 1,
    kBoundStick = 1 << 2,
    kBoundStickClick = 1 << 3,
    kBoundPrimary = 1 << 4,
    kBoundSecondary = 1 << 5,
    kBoundMenu = 1 << 6,
    kBoundThumbrest = 1 << 7,
};

void DescribeBound(unsigned bound, char* out, size_t size)
{
    static const char* const kNames[] = { "trigger", "grip", "stick", "stick-click", "A/X", "B/Y", "menu",
        "thumbrest" };
    out[0] = '\0';
    for (int i = 0; i < 8; ++i) {
        if (!(bound & (1u << i)))
            continue;
        if (out[0])
            strncat_s(out, size, " ", _TRUNCATE);
        strncat_s(out, size, kNames[i], _TRUNCATE);
    }
    if (!out[0])
        strncpy_s(out, size, "nothing", _TRUNCATE);
}

// The three readers all take the same pair: anyActive says "this hand is
// talking to us at all", and bound records which action it was.
bool ReadBool(XrAction action, XrPath hand, HandState* h = nullptr, unsigned bit = 0)
{
    bool* const active = h ? &h->anyActive : nullptr;
    if (!action)
        return false;
    XrActionStateGetInfo get = { XR_TYPE_ACTION_STATE_GET_INFO };
    get.action = action;
    get.subactionPath = hand;
    XrActionStateBoolean state = { XR_TYPE_ACTION_STATE_BOOLEAN };
    if (XR_FAILED(xrGetActionStateBoolean(g_session, &get, &state)))
        return false;
    if (state.isActive) {
        if (active)
            *active = true;
        if (h)
            h->bound |= bit;
    }
    return state.isActive && state.currentState;
}

float ReadFloat(XrAction action, XrPath hand, HandState* h = nullptr, unsigned bit = 0)
{
    bool* const active = h ? &h->anyActive : nullptr;
    if (!action)
        return 0.0f;
    XrActionStateGetInfo get = { XR_TYPE_ACTION_STATE_GET_INFO };
    get.action = action;
    get.subactionPath = hand;
    XrActionStateFloat state = { XR_TYPE_ACTION_STATE_FLOAT };
    if (XR_FAILED(xrGetActionStateFloat(g_session, &get, &state)))
        return 0.0f;
    if (state.isActive) {
        if (active)
            *active = true;
        if (h)
            h->bound |= bit;
    }
    return state.isActive ? state.currentState : 0.0f;
}

void ReadStick(XrAction action, XrPath hand, float* x, float* y, HandState* h = nullptr, unsigned bit = 0)
{
    bool* const active = h ? &h->anyActive : nullptr;
    *x = *y = 0.0f;
    if (!action)
        return;
    XrActionStateGetInfo get = { XR_TYPE_ACTION_STATE_GET_INFO };
    get.action = action;
    get.subactionPath = hand;
    XrActionStateVector2f state = { XR_TYPE_ACTION_STATE_VECTOR2F };
    if (XR_FAILED(xrGetActionStateVector2f(g_session, &get, &state)) || !state.isActive)
        return;
    if (active)
        *active = true;
    if (h)
        h->bound |= bit;
    *x = state.currentState.x;
    *y = state.currentState.y;
}

// A round deadzone, rescaled so the first movement past it is small rather
// than a jump.
void ApplyDeadzone(float* x, float* y, float deadzone)
{
    const float len = std::sqrt((*x) * (*x) + (*y) * (*y));
    if (len <= deadzone || len <= 0.0001f) {
        *x = *y = 0.0f;
        return;
    }
    const float scaled = (len - deadzone) / (1.0f - deadzone);
    const float k = (scaled > 1.0f ? 1.0f : scaled) / len;
    *x *= k;
    *y *= k;
}

SHORT ToAxis(float v)
{
    const float clamped = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
    const long scaled = static_cast<long>(clamped * 32767.0f);
    return static_cast<SHORT>(scaled);
}

// The d-pad, chosen from a stick once the modifier says so. 0.5 of full tilt
// to commit to a direction, the same threshold UEVR settled on.
WORD DpadFrom(float x, float y)
{
    WORD b = 0;
    if (y >= 0.5f)
        b |= XINPUT_GAMEPAD_DPAD_UP;
    if (y <= -0.5f)
        b |= XINPUT_GAMEPAD_DPAD_DOWN;
    if (x >= 0.5f)
        b |= XINPUT_GAMEPAD_DPAD_RIGHT;
    if (x <= -0.5f)
        b |= XINPUT_GAMEPAD_DPAD_LEFT;
    return b;
}

// OpenXR's profile paths are fixed by the spec and still say "oculus", years
// after the name changed. What the player reads shouldn't.
const char* FriendlyName(const char* path)
{
    struct Known {
        const char* path;
        const char* name;
    };
    static const Known kKnown[] = {
        { "/interaction_profiles/oculus/touch_controller", "Meta Quest Touch" },
        { "/interaction_profiles/meta/touch_controller_plus", "Meta Quest Touch Plus" },
        { "/interaction_profiles/meta/touch_pro_controller", "Meta Quest Touch Pro" },
        { "/interaction_profiles/valve/index_controller", "Valve Index" },
        { "/interaction_profiles/htc/vive_controller", "Vive wands" },
        { "/interaction_profiles/microsoft/motion_controller", "Windows Mixed Reality" },
        { "/interaction_profiles/bytedance/pico_neo3_controller", "Pico Neo 3" },
        { "/interaction_profiles/bytedance/pico4_controller", "Pico 4" },
        { "/interaction_profiles/khr/simple_controller", "basic controllers" },
    };
    for (const Known& k : kKnown) {
        if (strcmp(path, k.path) == 0)
            return k.name;
    }
    return nullptr;
}

void DescribeProfile(char* out, size_t size, const HandState& left, const HandState& right)
{
    char profile[XR_MAX_PATH_LENGTH] = "unknown controllers";
    if (g_session != XR_NULL_HANDLE) {
        XrInteractionProfileState state = { XR_TYPE_INTERACTION_PROFILE_STATE };
        if (XR_SUCCEEDED(xrGetCurrentInteractionProfile(g_session, g_handPath[kRight], &state))
            && state.interactionProfile != XR_NULL_PATH) {
            uint32_t len = 0;
            if (XR_SUCCEEDED(xrPathToString(g_instance, state.interactionProfile, XR_MAX_PATH_LENGTH, &len, profile))
                && len > 0) {
                if (const char* friendly = FriendlyName(profile)) {
                    strncpy_s(profile, friendly, _TRUNCATE);
                } else {
                    // Something we don't have a name for: show the tail of the
                    // path rather than nothing.
                    const char* tail = strstr(profile, "interaction_profiles/");
                    if (tail)
                        memmove(profile, tail + strlen("interaction_profiles/"), strlen(tail));
                    for (char* c = profile; *c; ++c) {
                        if (*c == '/' || *c == '_')
                            *c = ' ';
                    }
                }
            }
        }
    }
    const char* hands = left.anyActive && right.anyActive ? "both hands"
        : left.anyActive                                  ? "left hand only"
        : right.anyActive                                 ? "right hand only"
                                                          : "no hand answering";
    _snprintf_s(out, size, _TRUNCATE, "%s, %s", profile, hands);
}

} // namespace

void XrInput_SetSettings(const XrInputSettings& s)
{
    AcquireSRWLockExclusive(&g_lock);
    g_settings = s;
    // Clamped here rather than trusted: this one is written into the game's own
    // settings block, and a silly value out of a hand-edited ini would make the
    // gun uncontrollable rather than merely fast.
    if (!(g_settings.aimSpeedMult >= 1.0f))
        g_settings.aimSpeedMult = 1.0f;
    if (g_settings.aimSpeedMult > 8.0f)
        g_settings.aimSpeedMult = 8.0f;
    if (!(g_settings.aimServoMs >= 20.0f))
        g_settings.aimServoMs = 20.0f;
    if (g_settings.aimServoMs > 250.0f)
        g_settings.aimServoMs = 250.0f;
    if (!(g_settings.aimSteadiness >= 0.0f))
        g_settings.aimSteadiness = 0.0f;
    if (g_settings.aimSteadiness > 1.0f)
        g_settings.aimSteadiness = 1.0f;
    if (!(g_settings.aimMaxRateDeg >= 90.0f))
        g_settings.aimMaxRateDeg = 90.0f;
    if (g_settings.aimMaxRateDeg > 1200.0f)
        g_settings.aimMaxRateDeg = 1200.0f;
    if (!(g_settings.aimStickDeadzone >= 0.0f))
        g_settings.aimStickDeadzone = 0.0f;
    if (g_settings.aimStickDeadzone > 0.6f)
        g_settings.aimStickDeadzone = 0.6f;
    // 6DOF is one switch for a player (2026-09-18): driving the pose and rolling
    // the forearm are not separate choices, since without the first the arms
    // cannot hold a pose at all and without the second they move without turning.
    // A developer build keeps them apart, because taking one away at a time is
    // how any of this got worked out.
    if (!(g_settings.armIkWeight >= 0.0f))
        g_settings.armIkWeight = 0.0f;
    if (g_settings.armIkWeight > 1.0f)
        g_settings.armIkWeight = 1.0f;
    if (!(g_settings.armIkScale >= 0.5f))
        g_settings.armIkScale = 0.5f;
    if (g_settings.armIkScale > 2.0f)
        g_settings.armIkScale = 2.0f;
    ReleaseSRWLockExclusive(&g_lock);
}

unsigned long long XrInput_LastShotMs()
{
    return g_lastShotMs;
}

bool XrInput_GetMenuHand(float* xRad, float* yRad)
{
    if (!g_haveMenuHand)
        return false;
    if (xRad)
        *xRad = g_menuHandX;
    if (yRad)
        *yRad = g_menuHandY;
    return true;
}

float XrInput_GetMenuScroll()
{
    return g_menuScroll;
}

bool XrInput_GetMenuPointer(float* xRad, float* yRad, bool* click, bool* rightClick)
{
    if (!g_haveMenuPoint)
        return false;
    if (xRad)
        *xRad = g_menuPointX;
    if (yRad)
        *yRad = g_menuPointY;
    if (click)
        *click = g_menuClick;
    if (rightClick)
        *rightClick = g_menuRightClick;
    return true;
}

bool XrInput_GetSupportRoll(float out[9])
{
    if (!g_haveSupportRoll || !out)
        return false;
    std::memcpy(out, g_supportRoll, sizeof(g_supportRoll));
    return true;
}

bool XrInput_GripHeld(int hand)
{
    if (hand < 0 || hand > 1)
        return false;
    return g_gripNow[hand].load(std::memory_order_relaxed) > 0.5f;
}

bool XrInput_GetTwoHand(bool* cupped)
{
    if (cupped)
        *cupped = g_twoHandCupped;
    return g_twoHandHeld;
}

bool XrInput_ReloadCarrying()
{
    return g_reloadCarrying;
}

int XrInput_MagazineHeld()
{
    // In your hand only while you are actually carrying it. Once it touches
    // the weapon the game's reload is playing and the magazine is the
    // animation's business again - writing the bone through that would hold it
    // in a hand that has already let go.
    if (g_magInHand)
        return 2;
    // HELD STILL, NOT HANDED BACK (2026-09-25, user: "it would be nice to
    // freeze the magazine bone so you don't see it move in the animation when
    // the reload animation plays").
    //
    // Letting go entirely was better than holding it in a hand that had
    // finished, but it hands the magazine to an animation that is about to
    // eject it and put it back - and it has already been ejected, by you, a
    // few seconds ago. So the magazine is pinned where it belongs in the
    // weapon for as long as the reload lasts. The sound plays, the slide
    // works, the hands move, and the one part that would contradict what your
    // own hands just did stays exactly where you put it.
    if (g_gameReloading)
        return 3;
    return g_magOutAt ? 1 : 0;
}

bool XrInput_GunHandIsLeft()
{
    const XrInputSettings s = XrInput_GetSettings();
    return s.swapHands != s.characterLeftHanded;
}

XrInputSettings XrInput_GetSettings()
{
    AcquireSRWLockShared(&g_lock);
    const XrInputSettings s = g_settings;
    ReleaseSRWLockShared(&g_lock);
    return s;
}

bool XrInput_GetPad(XINPUT_GAMEPAD* out)
{
    if (!out)
        return false;
    AcquireSRWLockShared(&g_lock);
    const bool fresh = g_padMs && GetTickCount64() - g_padMs < 500;
    if (fresh)
        *out = g_pad;
    ReleaseSRWLockShared(&g_lock);
    return fresh;
}

// A pulse on one controller. Amplitude 0..1, duration in seconds. The runtime
// decides what that actually feels like - a Touch controller and an Index
// knuckle have very different motors - so these are intentions, not waveforms.
void XrInput_Pulse(int hand, float amplitude, float seconds)
{
    if (g_haptic == XR_NULL_HANDLE || g_session == XR_NULL_HANDLE || hand < 0 || hand > 1)
        return;
    if (amplitude <= 0.0f)
        return;
    if (amplitude > 1.0f)
        amplitude = 1.0f;
    if (seconds < 0.01f)
        seconds = 0.01f;
    XrHapticVibration v = { XR_TYPE_HAPTIC_VIBRATION };
    v.amplitude = amplitude;
    v.duration = static_cast<XrDuration>(seconds * 1e9);
    v.frequency = XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo info = { XR_TYPE_HAPTIC_ACTION_INFO };
    info.action = g_haptic;
    info.subactionPath = g_handPath[hand];
    xrApplyHapticFeedback(g_session, &info, reinterpret_cast<const XrHapticBaseHeader*>(&v));
}

bool XrInput_GetHandPose(int hand, XrHandPose& out)
{
    if (hand < 0 || hand > 1)
        return false;
    AcquireSRWLockShared(&g_lock);
    out = g_handPose[hand];
    ReleaseSRWLockShared(&g_lock);
    return out.tracked && out.ms && GetTickCount64() - out.ms < 500;
}

bool XrInput_GetGunAim(float* yawDeg, float* pitchDeg)
{
    AcquireSRWLockShared(&g_lock);
    const bool fresh = g_gunAimMs && GetTickCount64() - g_gunAimMs < 500;
    if (fresh) {
        if (yawDeg)
            *yawDeg = g_gunYawDeg;
        if (pitchDeg)
            *pitchDeg = g_gunPitchDeg;
    }
    ReleaseSRWLockShared(&g_lock);
    return fresh;
}

void XrInput_DescribeStatus(char* out, size_t size)
{
    if (!out || !size)
        return;
    AcquireSRWLockShared(&g_lock);
    strncpy_s(out, size, g_status, _TRUNCATE);
    ReleaseSRWLockShared(&g_lock);
}

void XrInput_OnInstanceCreated(XrInstance instance)
{
    if (g_actionSet != XR_NULL_HANDLE)
        return;
    g_instance = instance;

    XrActionSetCreateInfo setInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
    strcpy_s(setInfo.actionSetName, "gameplay");
    strcpy_s(setInfo.localizedActionSetName, "Gameplay");
    setInfo.priority = 0;
    const XrResult r = xrCreateActionSet(instance, &setInfo, &g_actionSet);
    if (XR_FAILED(r)) {
        Log_Printf("XrInput: xrCreateActionSet failed -> %s - motion controllers unavailable", XrName(r));
        g_actionSet = XR_NULL_HANDLE;
        return;
    }

    g_handPath[kLeft] = Path("/user/hand/left");
    g_handPath[kRight] = Path("/user/hand/right");

    g_trigger = CreateAction(XR_ACTION_TYPE_FLOAT_INPUT, "trigger", "Trigger");
    g_squeeze = CreateAction(XR_ACTION_TYPE_FLOAT_INPUT, "squeeze", "Grip");
    g_stick = CreateAction(XR_ACTION_TYPE_VECTOR2F_INPUT, "stick", "Thumbstick");
    g_stickClick = CreateAction(XR_ACTION_TYPE_BOOLEAN_INPUT, "stick_click", "Thumbstick click");
    g_primary = CreateAction(XR_ACTION_TYPE_BOOLEAN_INPUT, "primary", "A or X");
    g_secondary = CreateAction(XR_ACTION_TYPE_BOOLEAN_INPUT, "secondary", "B or Y");
    g_menu = CreateAction(XR_ACTION_TYPE_BOOLEAN_INPUT, "menu", "Menu");
    g_thumbrest = CreateAction(XR_ACTION_TYPE_BOOLEAN_INPUT, "thumbrest", "Thumbrest touch");
    g_aimPose = CreateAction(XR_ACTION_TYPE_POSE_INPUT, "aim_pose", "Where the controller points");
    g_haptic = CreateAction(XR_ACTION_TYPE_VIBRATION_OUTPUT, "haptic", "Vibration");

    SuggestEveryProfile();
    Log_Printf("XrInput: action set ready");
}

void XrInput_AttachToSession(XrSession session)
{
    if (g_actionSet == XR_NULL_HANDLE || g_attached)
        return;
    g_session = session;
    XrSessionActionSetsAttachInfo info = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    info.countActionSets = 1;
    info.actionSets = &g_actionSet;
    const XrResult r = xrAttachSessionActionSets(session, &info);
    g_attached = XR_SUCCEEDED(r);
    Log_Printf("XrInput: attaching the action set -> %s", XrName(r));
    if (!g_attached || !g_aimPose)
        return;

    // One space per hand that follows where that controller points. 3DOF
    // aiming only reads the rotation out of these.
    for (int i = 0; i < 2; ++i) {
        XrActionSpaceCreateInfo spaceInfo = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        spaceInfo.action = g_aimPose;
        spaceInfo.subactionPath = g_handPath[i];
        spaceInfo.poseInActionSpace.orientation = { 0, 0, 0, 1 };
        const XrResult sr = xrCreateActionSpace(session, &spaceInfo, &g_aimSpace[i]);
        if (XR_FAILED(sr)) {
            g_aimSpace[i] = XR_NULL_HANDLE;
            Log_Printf("XrInput: xrCreateActionSpace(%s) -> %s", i == kLeft ? "left" : "right", XrName(sr));
        }
    }
}

void XrInput_Sync(XrSession session, XrSpace baseSpace, XrTime displayTime)
{
    if (!g_attached || session != g_session)
        return;

    const XrInputSettings settings = XrInput_GetSettings();
    if (!settings.enabled) {
        AcquireSRWLockExclusive(&g_lock);
        g_padMs = 0;
        strcpy_s(g_status, "off");
        ReleaseSRWLockExclusive(&g_lock);
        return;
    }

    XrActiveActionSet active = {};
    active.actionSet = g_actionSet;
    active.subactionPath = XR_NULL_PATH;
    XrActionsSyncInfo sync = { XR_TYPE_ACTIONS_SYNC_INFO };
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    const XrResult r = xrSyncActions(session, &sync);
    if (XR_FAILED(r)) {
        static XrResult s_lastLogged = XR_SUCCESS;
        if (r != s_lastLogged) {
            s_lastLogged = r;
            Log_Printf("XrInput: xrSyncActions -> %s", XrName(r));
        }
        return;
    }
    // XR_SESSION_NOT_FOCUSED is a success code: the runtime's own menu is up,
    // every action reads inactive, and the pad below goes quiet by itself.

    HandState hand[2];
    for (int i = 0; i < 2; ++i) {
        HandState& h = hand[i];
        const XrPath p = g_handPath[i];
        h.trigger = ReadFloat(g_trigger, p, &h, kBoundTrigger);
        h.squeeze = ReadFloat(g_squeeze, p, &h, kBoundSqueeze);
        if (i >= 0 && i < 2)
            g_gripNow[i].store(h.squeeze, std::memory_order_relaxed);
        ReadStick(g_stick, p, &h.stickX, &h.stickY, &h, kBoundStick);
        h.stickClick = ReadBool(g_stickClick, p, &h, kBoundStickClick);
        h.primary = ReadBool(g_primary, p, &h, kBoundPrimary);
        h.secondary = ReadBool(g_secondary, p, &h, kBoundSecondary);
        h.menu = ReadBool(g_menu, p, &h, kBoundMenu);
        h.thumbrest = ReadBool(g_thumbrest, p, &h, kBoundThumbrest);
    }

    // Two different questions, and they had one answer between them
    // (2026-09-24, user: "the sheva checkbox flips all controls. We want to
    // keep the default control scheme. Left stick should still be move...
    // right stick should still be camera, face buttons should still be face
    // buttons. The only change is the grips changing").
    //
    // WHICH HAND HOLDS THE GUN decides the grip and the trigger, and it
    // changes when the character is left handed, because Sheva holds hers in
    // her left. That is the whole of what being left handed means here.
    //
    // WHICH CONTROLLER IS IN WHICH HAND decides everything a pad has: the
    // move stick, the look stick, the face buttons, the stick clicks. Sheva
    // being left handed says nothing about that, and treating it as though it
    // did moved every control to the other hand.
    const bool gunIsLeft = settings.swapHands != settings.characterLeftHanded;
    const HandState& gun = gunIsLeft ? hand[kLeft] : hand[kRight];
    const HandState& off = gunIsLeft ? hand[kRight] : hand[kLeft];
    const HandState& padLeft = settings.swapHands ? hand[kRight] : hand[kLeft];
    const HandState& padRight = settings.swapHands ? hand[kLeft] : hand[kRight];

    XINPUT_GAMEPAD pad = {};

    // Aim and fire. RE5 readies the weapon on the left trigger and fires on
    // the right, so the gun hand's grip is aim and its trigger is fire.
    pad.bLeftTrigger = gun.squeeze > 0.5f ? 255 : 0;
    pad.bRightTrigger = static_cast<BYTE>((gun.trigger > 1.0f ? 1.0f : gun.trigger) * 255.0f);

    // Recoil (2026-09-19). Fired on the trigger CROSSING, not on it being held,
    // so it is one kick per pull rather than a rattle - and only with the
    // weapon up, since a trigger pull with the gun down does not fire.
    {
        static bool s_wasFiring = false;
        const bool aiming = gun.squeeze > 0.5f;
        const bool firing = aiming && gun.trigger > 0.6f;
        if (firing && !s_wasFiring)
            XrInput_Pulse(gunIsLeft ? kLeft : kRight, 1.0f, 0.06f);
        s_wasFiring = firing;
        if (firing)
            g_lastShotMs = GetTickCount64();
    }

    // A and B live on the right controller and X and Y on the left, on every
    // headset that has them, and that does not change because Sheva is left
    // handed.
    if (padRight.primary)
        pad.wButtons |= XINPUT_GAMEPAD_A;
    if (padRight.secondary)
        pad.wButtons |= XINPUT_GAMEPAD_B;
    if (padLeft.primary)
        pad.wButtons |= XINPUT_GAMEPAD_X;
    if (padLeft.secondary)
        pad.wButtons |= XINPUT_GAMEPAD_Y;
    // The off grip is the knife only without holsters (2026-09-22). With them,
    // the knife comes from its holster and nowhere else, so an accidental
    // squeeze cannot pull it, and the grip is free to be the support hand.
    if (off.squeeze > 0.5f && !settings.holsters)
        pad.wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
    // The menu button is Start; with the off hand's grip held it is Back
    // (2026-09-22). RE5 skips a cutscene on Back, not Start - Start pauses -
    // and a Quest controller gives a game exactly one menu button. A hold and
    // a double tap were both tried and are out: Virtual Desktop takes both on
    // that button for its own menu. The grip does nothing in a cutscene, so
    // grip + menu cannot collide with anything, and a plain tap stays instant.
    {
        constexpr ULONGLONG kPressMs = 100;
        static bool s_wasMenu = false;
        static ULONGLONG s_startUntil = 0, s_backUntil = 0;
        const ULONGLONG nowMenu = GetTickCount64();
        const bool menuNow = off.menu || gun.menu;
        if (menuNow && !s_wasMenu) {
            if (off.squeeze > 0.5f) {
                s_backUntil = nowMenu + kPressMs;
                Log_Printf("Menu button with the grip held - Back (skip)");
            } else {
                s_startUntil = nowMenu + kPressMs;
            }
        }
        s_wasMenu = menuNow;
        if (nowMenu < s_startUntil)
            pad.wButtons |= XINPUT_GAMEPAD_START;
        if (nowMenu < s_backUntil)
            pad.wButtons |= XINPUT_GAMEPAD_BACK;
    }

    // Left stick moves, right stick looks, whoever is holding the gun.
    float moveX = padLeft.stickX, moveY = padLeft.stickY;
    float lookX = padRight.stickX, lookY = padRight.stickY;
    ApplyDeadzone(&moveX, &moveY, settings.deadzone);
    ApplyDeadzone(&lookX, &lookY, settings.deadzone);

    // Roomscale (2026-09-23). Whatever is left of your room movement after the
    // spine has leaned as far as it will goes to his legs, added to your thumb
    // rather than replacing it, so walking and stepping can happen at once and
    // neither cancels the other.
    //
    // It closes its own loop: the offset is measured against a reference that
    // walks with him, so every step he takes takes the gap away with it and he
    // stops on his own. Nothing here needs to know how fast he walks.
    float askRight = 0.0f, askFwd = 0.0f;
    if (settings.roomStep) {
        float toRight = 0.0f, toFront = 0.0f;
        if (StereoTest_GetRoomStep(&toRight, &toFront)) {
            const float away = std::sqrt(toRight * toRight + toFront * toFront);
            float dead = settings.roomStepDeadM;
            float full = settings.roomStepFullM;
            if (!(dead >= 0.0f))
                dead = 0.15f;
            if (!(full > dead + 0.05f))
                full = dead + 0.05f;
            if (away > dead) {
                float push = (away - dead) / (full - dead);
                if (push > 1.0f)
                    push = 1.0f;
                const float k = push / away;
                askRight = toRight * k;
                askFwd = toFront * k;
                moveX += askRight;
                moveY += askFwd;
                const float len = std::sqrt(moveX * moveX + moveY * moveY);
                if (len > 1.0f) {
                    moveX /= len;
                    moveY /= len;
                }
            }
        }
    }
    StereoTest_SetRoomAsk(askRight, askFwd);

    // The d-pad. Whichever stick is standing in for it stops being a stick
    // while it does, so the game never gets a direction and a push at once.
    bool dpadFromLook = false, dpadFromMove = false;
    switch (static_cast<XrDpadMethod>(settings.dpadMethod)) {
    // Named for the controller they are on, so they follow the controller.
    case XrDpadMethod::LeftTrigger:
        dpadFromLook = padLeft.trigger > 0.5f;
        break;
    case XrDpadMethod::RightThumbrest:
        dpadFromMove = padRight.thumbrest;
        break;
    case XrDpadMethod::LeftStickClick:
        dpadFromLook = padLeft.stickClick;
        break;
    case XrDpadMethod::RightStickAlways:
        dpadFromLook = true;
        break;
    case XrDpadMethod::Off:
    default:
        break;
    }

    if (dpadFromLook) {
        pad.wButtons |= DpadFrom(lookX, lookY);
        lookX = lookY = 0.0f;
    }
    if (dpadFromMove) {
        pad.wButtons |= DpadFrom(moveX, moveY);
        moveX = moveY = 0.0f;
    }

    // The stick clicks, unless a click is the d-pad modifier: L3 and R3
    // together open the mod menu, and that shouldn't fire while shifting.
    if (padLeft.stickClick && static_cast<XrDpadMethod>(settings.dpadMethod) != XrDpadMethod::LeftStickClick)
        pad.wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
    if (padRight.stickClick)
        pad.wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;

    pad.sThumbLX = ToAxis(moveX);
    pad.sThumbLY = ToAxis(moveY);
    // Turning with your head (2026-09-24). After the d-pad has had its say, so
    // that looking around cannot be read as a d-pad direction, and added to
    // your thumb rather than replacing it.
    if (settings.headTurn) {
        XRBridgeEyeView htL, htR;
        if (VRBridge_GetEyeViews(htL, htR)) {
            const float* look = &htL.rotationDelta[6];
            const float yawDeg = std::atan2(look[0], look[2]) * 57.2957795f;
            float dead = settings.headTurnDeadDeg;
            float full = settings.headTurnFullDeg;
            if (!(dead >= 0.0f))
                dead = 25.0f;
            if (!(full > dead + 5.0f))
                full = dead + 5.0f;
            const float away = yawDeg < 0.0f ? -yawDeg : yawDeg;
            if (away > dead) {
                float push = (away - dead) / (full - dead);
                if (push > 1.0f)
                    push = 1.0f;
                lookX += yawDeg < 0.0f ? -push : push;
                if (lookX > 1.0f)
                    lookX = 1.0f;
                if (lookX < -1.0f)
                    lookX = -1.0f;
            }
        }
    }

    pad.sThumbRX = ToAxis(lookX);
    // The head owns up and down unless you ask otherwise.
    pad.sThumbRY = settings.stickPitch ? ToAxis(lookY) : 0;

    // Where the gun hand points, for 3DOF aiming. Rotation only: the hand's
    // position in the room is deliberately ignored.
    const int gunHand = gunIsLeft ? kLeft : kRight;
    float gunYawDeg = 0.0f, gunPitchDeg = 0.0f;
    bool gunAimed = false;
    if (g_aimSpace[gunHand] != XR_NULL_HANDLE && baseSpace != XR_NULL_HANDLE) {
        XrSpaceLocation location = { XR_TYPE_SPACE_LOCATION };
        if (XR_SUCCEEDED(xrLocateSpace(g_aimSpace[gunHand], baseSpace, displayTime, &location))
            && (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            float f[3];
            PoseForward(location.pose.orientation, f);
            const float flat = std::sqrt(f[0] * f[0] + f[2] * f[2]);
            constexpr float kPi = 3.14159265358979f;
            gunPitchDeg = std::atan2(f[1], flat) * 180.0f / kPi;
            gunYawDeg = std::atan2(-f[0], -f[2]) * 180.0f / kPi;
            gunAimed = true;
        }
    }

    const bool anyHand = hand[kLeft].anyActive || hand[kRight].anyActive;
    char status[160];
    DescribeProfile(status, sizeof(status), hand[kLeft], hand[kRight]);

    // Both hands' poses, for arm IK. Located every sync whether or not
    // anything is using them: the cost is two xrLocateSpace calls, and having
    // them stale by a frame is exactly the kind of lag a hand shows up as.
    XrHandPose handPose[2] = {};
    for (int i = 0; i < 2 && baseSpace != XR_NULL_HANDLE; ++i) {
        if (g_aimSpace[i] == XR_NULL_HANDLE)
            continue;
        XrSpaceLocation loc = { XR_TYPE_SPACE_LOCATION };
        if (!XR_SUCCEEDED(xrLocateSpace(g_aimSpace[i], baseSpace, displayTime, &loc)))
            continue;
        const bool haveRot = (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
        const bool havePos = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
        if (!haveRot || !havePos)
            continue;
        handPose[i].tracked = true;
        handPose[i].posMeters[0] = loc.pose.position.x;
        handPose[i].posMeters[1] = loc.pose.position.y;
        handPose[i].posMeters[2] = loc.pose.position.z;
        const XrQuaternionf& q = loc.pose.orientation;
        const Mat3 m = QuaternionToMat3(q.x, q.y, q.z, q.w);
        for (int k = 0; k < 9; ++k)
            handPose[i].rot[k] = m.m[k];
        handPose[i].ms = GetTickCount64();
    }

    // ---- Two hands on the gun (2026-09-22) ----------------------------------
    // Squeeze the off grip with that hand on the gun, and it is holding it.
    // Where "on the gun" is came from measuring every weapon's own aiming
    // animation: the pistols and magnums cup 9 cm from the gun hand, the long
    // guns hold 26 to 58 cm out along the barrel, the minigun's handle sits
    // 16 cm up. So: close to the gun hand, or inside a sleeve around the line
    // the gun points along. Decided on the press, like the holsters, and held
    // until the grip lets go.
    //
    // Held, the gun points along the line between your hands. What is kept is
    // how the gun pointed relative to that line at the moment you took hold,
    // so nothing jumps: the gun keeps its aim, and from then on it is the pair
    // of hands that turns it, not one wrist. A pistol's cupping hand is too
    // close to steer by - a few centimetres of wobble would be degrees of aim -
    // so short holds only count as held.
    {
        static bool s_held = false;
        static bool s_wasGrip = false;
        static float s_holdDir[3] = {}; // the off hand's direction, in the gun hand's axes
        static float s_holdLen = 0.0f;
        const int gi = gunIsLeft ? kLeft : kRight;
        const int oi = gunIsLeft ? kRight : kLeft;
        const bool gripNow = off.squeeze > 0.5f;
        const bool tracked = handPose[gi].tracked && handPose[oi].tracked;
        const float* R = handPose[gi].rot; // rows: right, up, forward
        float d[3] = {};
        for (int k = 0; k < 3; ++k)
            d[k] = handPose[oi].posMeters[k] - handPose[gi].posMeters[k];
        const float dist = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

        // A hand bringing a magazine up to the weapon ends in exactly the
        // place a supporting hand does, holding exactly the same button, so
        // the reload has to be able to say so.
        if (!settings.twoHanded || !gripNow || !tracked || g_reloadCarrying) {
            if (s_held)
                Log_Printf("TwoHand: let go");
            s_held = false;
        } else if (!s_wasGrip && dist > 0.01f) {
            float l[3];
            for (int r = 0; r < 3; ++r)
                l[r] = R[r * 3] * d[0] + R[r * 3 + 1] * d[1] + R[r * 3 + 2] * d[2];
            // Along the gun's REAL barrel, not the controller's forward
            // (2026-09-22). The first pass used the controller's, and every
            // reach for a pump landed 20-25 cm above it and was refused -
            // the gun on the arm points about 23 degrees higher than the
            // controller does. The arm solve knows the real line; without it,
            // the measured tilt stands in.
            float b[3] = { 0.0f, 0.39f, 0.92f };
            float fromIk[3];
            if (ArmIk_GetBarrelInController(fromIk))
                std::memcpy(b, fromIk, sizeof(b));
            const float along = l[0] * b[0] + l[1] * b[1] + l[2] * b[2];
            float perp[3];
            for (int r = 0; r < 3; ++r)
                perp[r] = l[r] - b[r] * along;
            const float lateral = std::sqrt(perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]);
            constexpr float kCupReach = 0.22f;     // a pistol's cupping hand
            constexpr float kPistolLength = 0.22f; // closer than this is on a pistol
            constexpr float kSleeveRadius = 0.28f; // around the barrel: the grip does nothing else now
            constexpr float kSleeveFar = 0.85f;    // past the longest hold, the Ithaca's 52 cm
            const bool cupped = dist < kCupReach;
            const bool alongBarrel = along > 0.0f && along < kSleeveFar && lateral < kSleeveRadius;
            if (cupped || alongBarrel) {
                s_held = true;
                // Anything this close is a pistol, front or back, and a pistol
                // is supported from behind. The shortest long-gun hold measured
                // is the VZ61's 26 cm, so 22 cm leaves room either side.
                g_twoHandCupped = !alongBarrel || dist < kPistolLength;
                s_holdLen = dist;
                for (int r = 0; r < 3; ++r)
                    s_holdDir[r] = l[r] / dist;
                XrInput_Pulse(gi, 0.4f, 0.04f);
                XrInput_Pulse(oi, 0.4f, 0.04f);
                Log_Printf("TwoHand: holding, off hand %.2f m right %.2f up %.2f along the gun - %s", l[0],
                    l[1], l[2], g_twoHandCupped ? "cupped, the gun hand still aims" : "both hands aim it");
            } else {
                Log_Printf("TwoHand: grip at %.2f m right %.2f up %.2f along the gun - not on it", l[0], l[1],
                    l[2]);
            }
        }
        s_wasGrip = gripNow;
        g_twoHandHeld = s_held;

        if (s_held && !g_twoHandCupped && dist > 0.05f) {
            // Where the held direction points now if only the gun hand moved,
            // against where the off hand actually is. The turn between the two
            // is applied to the whole gun hand.
            float want[3] = { d[0] / dist, d[1] / dist, d[2] / dist };
            float have[3];
            for (int k = 0; k < 3; ++k)
                have[k] = R[k] * s_holdDir[0] + R[3 + k] * s_holdDir[1] + R[6 + k] * s_holdDir[2];
            const float c = have[0] * want[0] + have[1] * want[1] + have[2] * want[2];
            if (c > -0.95f) {
                const float v[3] = { have[1] * want[2] - have[2] * want[1], have[2] * want[0] - have[0] * want[2],
                    have[0] * want[1] - have[1] * want[0] };
                // Only part of the way (2026-09-22): the turn that would put
                // the gun on the line between your hands, scaled by how much
                // the support hand is meant to own. Rodrigues with the angle cut
                // down: I + sin(a)[k] + (1 - cos(a))[k]^2, k the unit axis.
                float w = settings.twoHandSteer;
                if (!(w > 0.0f))
                    w = 0.0f;
                if (w > 1.0f)
                    w = 1.0f;
                const float sn = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                float M[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
                if (sn > 1e-6f) {
                    const float kx = v[0] / sn, ky = v[1] / sn, kz = v[2] / sn;
                    const float a = std::atan2(sn, c) * w;
                    const float s = std::sin(a), t = 1.0f - std::cos(a);
                    M[0] = 1.0f + t * (kx * kx - 1.0f);
                    M[1] = -s * kz + t * kx * ky;
                    M[2] = s * ky + t * kx * kz;
                    M[3] = s * kz + t * kx * ky;
                    M[4] = 1.0f + t * (ky * ky - 1.0f);
                    M[5] = -s * kx + t * ky * kz;
                    M[6] = -s * ky + t * kx * kz;
                    M[7] = s * kx + t * ky * kz;
                    M[8] = 1.0f + t * (kz * kz - 1.0f);
                }
                float turned[9];
                for (int r = 0; r < 3; ++r)
                    for (int k = 0; k < 3; ++k)
                        turned[r * 3 + k] = M[k * 3] * R[r * 3] + M[k * 3 + 1] * R[r * 3 + 1] + M[k * 3 + 2] * R[r * 3 + 2];
                std::memcpy(handPose[gi].rot, turned, sizeof(turned));
                // And the aim, from the turned forward axis, the same way the
                // one-handed aim is read off the controller.
                const float* f = handPose[gi].rot + 6;
                const float flat = std::sqrt(f[0] * f[0] + f[2] * f[2]);
                constexpr float kPi = 3.14159265358979f;
                gunPitchDeg = std::atan2(f[1], flat) * 180.0f / kPi;
                gunYawDeg = std::atan2(-f[0], -f[2]) * 180.0f / kPi;
                gunAimed = true;
            }
        }

        // Roll, from the support hand (2026-09-23). The turn above is the
        // shortest one that puts the gun on the line between your hands, and
        // a shortest turn adds no roll at all - which is why the gun answered
        // to side to side and up and down and nothing else. So: how far has
        // your support hand twisted about the barrel since you took hold, and
        // the gun takes its share. Measured against the grip, so nothing
        // jumps at the moment you take hold, and about the gun's REAL barrel
        // rather than the controller's forward, which is some twenty degrees
        // away from it. A cupped pistol gets this even though it gets no
        // steering: the gun hand still aims, your other hand just squares it
        // up for the sights.
        static bool s_haveRoll = false;
        static float s_rollRef = 0.0f;
        static float s_rollTotal = 0.0f;
        if (!s_held) {
            s_haveRoll = false;
            s_rollTotal = 0.0f;
            g_haveSupportRoll = false;
        } else if (settings.twoHandRoll > 0.001f) {
            float* Rg = handPose[gi].rot;
            float barrel[3] = { 0.0f, 0.39f, 0.92f };
            float fromIk2[3];
            if (ArmIk_GetBarrelInController(fromIk2))
                std::memcpy(barrel, fromIk2, sizeof(barrel));
            float ax[3];
            for (int k = 0; k < 3; ++k)
                ax[k] = Rg[k] * barrel[0] + Rg[3 + k] * barrel[1] + Rg[6 + k] * barrel[2];
            const float al = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
            if (al > 1e-4f) {
                for (int k = 0; k < 3; ++k)
                    ax[k] /= al;
                float gu[3], ou[3];
                const float gd = Rg[3] * ax[0] + Rg[4] * ax[1] + Rg[5] * ax[2];
                const float od = handPose[oi].rot[3] * ax[0] + handPose[oi].rot[4] * ax[1]
                    + handPose[oi].rot[5] * ax[2];
                for (int k = 0; k < 3; ++k) {
                    gu[k] = Rg[3 + k] - ax[k] * gd;
                    ou[k] = handPose[oi].rot[3 + k] - ax[k] * od;
                }
                const float gl = std::sqrt(gu[0] * gu[0] + gu[1] * gu[1] + gu[2] * gu[2]);
                const float ol = std::sqrt(ou[0] * ou[0] + ou[1] * ou[1] + ou[2] * ou[2]);
                if (gl > 1e-3f && ol > 1e-3f) {
                    for (int k = 0; k < 3; ++k) {
                        gu[k] /= gl;
                        ou[k] /= ol;
                    }
                    const float cdot = gu[0] * ou[0] + gu[1] * ou[1] + gu[2] * ou[2];
                    const float cr[3] = { gu[1] * ou[2] - gu[2] * ou[1], gu[2] * ou[0] - gu[0] * ou[2],
                        gu[0] * ou[1] - gu[1] * ou[0] };
                    const float ang = std::atan2(cr[0] * ax[0] + cr[1] * ax[1] + cr[2] * ax[2], cdot);
                    // Added up frame by frame rather than measured against the
                    // grip outright (2026-09-23, user: "it has a threshold
                    // where you over roll and the IK spazzes out"). An angle
                    // taken straight has to wrap somewhere, and wherever it
                    // wraps the gun swings the long way round in one frame,
                    // which is a wrist the arm cannot follow. A running total
                    // never wraps, and it stops at a hundred degrees either
                    // way - past a real wrist, and well short of anything the
                    // arm would have to break to reach.
                    if (!s_haveRoll) {
                        s_rollRef = ang;
                        s_rollTotal = 0.0f;
                        s_haveRoll = true;
                    }
                    float step = ang - s_rollRef;
                    while (step > 3.14159265f)
                        step -= 6.28318531f;
                    while (step < -3.14159265f)
                        step += 6.28318531f;
                    s_rollRef = ang;
                    s_rollTotal += step;
                    // A hundred degrees was still far too much (2026-09-23,
                    // user: "I can still over turn and cause a gun spaz").
                    // The log says exactly what breaks: "FLIP on the right
                    // forearm - the applied roll jumped 88 deg in one frame".
                    // The arm works the wrist's twist out by splitting the
                    // wrist's rotation into a twist and a bend, and that split
                    // has two answers a half turn apart. Push the wrist far
                    // enough and the solver walks from one to the other, which
                    // is the spaz. So the roll never goes near it.
                    //
                    // Two limits, both soft. The angle eases onto about thirty
                    // five degrees however far you keep turning, rather than
                    // hitting a wall, and it can only change by two degrees a
                    // frame - a hundred and eighty a second, faster than a
                    // wrist, and slow enough that nothing arrives in one jump.
                    constexpr float kRollRoom = 2.6f; // how far you turn for all of it
                    if (s_rollTotal > kRollRoom)
                        s_rollTotal = kRollRoom;
                    if (s_rollTotal < -kRollRoom)
                        s_rollTotal = -kRollRoom;
                    float w = settings.twoHandRoll;
                    w = w > 1.0f ? 1.0f : w;
                    constexpr float kRollMost = 0.61f; // thirty five degrees
                    float a = kRollMost * std::tanh(s_rollTotal * w / kRollMost);
                    static float s_rollWas = 0.0f;
                    const float step2 = a - s_rollWas;
                    constexpr float kRollStep = 0.035f; // two degrees a frame
                    if (step2 > kRollStep)
                        a = s_rollWas + kRollStep;
                    else if (step2 < -kRollStep)
                        a = s_rollWas - kRollStep;
                    s_rollWas = a;
                    // What the gun did not take, the hand keeps (2026-09-23,
                    // user: "I think what was causing it is your other wrist
                    // not rotating enough with it, so they started fighting").
                    // Turn the gun by a and the hand on it turns by a as well,
                    // because it is locked to the gun - but your real hand has
                    // turned by the whole amount, so the arm is being asked for
                    // a wrist that is the difference away from the one you are
                    // holding. This is that difference, and the support hand
                    // gets it back further down.
                    {
                        const float left = s_rollTotal - a;
                        const float sl = std::sin(left), tl = 1.0f - std::cos(left);
                        g_supportRoll[0] = 1.0f + tl * (ax[0] * ax[0] - 1.0f);
                        g_supportRoll[1] = -sl * ax[2] + tl * ax[0] * ax[1];
                        g_supportRoll[2] = sl * ax[1] + tl * ax[0] * ax[2];
                        g_supportRoll[3] = sl * ax[2] + tl * ax[0] * ax[1];
                        g_supportRoll[4] = 1.0f + tl * (ax[1] * ax[1] - 1.0f);
                        g_supportRoll[5] = -sl * ax[0] + tl * ax[1] * ax[2];
                        g_supportRoll[6] = -sl * ax[1] + tl * ax[0] * ax[2];
                        g_supportRoll[7] = sl * ax[0] + tl * ax[1] * ax[2];
                        g_supportRoll[8] = 1.0f + tl * (ax[2] * ax[2] - 1.0f);
                        g_haveSupportRoll = true;
                    }
                    const float sn2 = std::sin(a), t2 = 1.0f - std::cos(a);
                    const float kx = ax[0], ky = ax[1], kz = ax[2];
                    float M[9];
                    M[0] = 1.0f + t2 * (kx * kx - 1.0f);
                    M[1] = -sn2 * kz + t2 * kx * ky;
                    M[2] = sn2 * ky + t2 * kx * kz;
                    M[3] = sn2 * kz + t2 * kx * ky;
                    M[4] = 1.0f + t2 * (ky * ky - 1.0f);
                    M[5] = -sn2 * kx + t2 * ky * kz;
                    M[6] = -sn2 * ky + t2 * kx * kz;
                    M[7] = sn2 * kx + t2 * ky * kz;
                    M[8] = 1.0f + t2 * (kz * kz - 1.0f);
                    float rolled[9];
                    for (int r = 0; r < 3; ++r)
                        for (int k = 0; k < 3; ++k)
                            rolled[r * 3 + k] = M[k * 3] * Rg[r * 3] + M[k * 3 + 1] * Rg[r * 3 + 1]
                                + M[k * 3 + 2] * Rg[r * 3 + 2];
                    std::memcpy(handPose[gi].rot, rolled, sizeof(rolled));
                    if (gunAimed) {
                        const float* f2 = handPose[gi].rot + 6;
                        const float flat2 = std::sqrt(f2[0] * f2[0] + f2[2] * f2[2]);
                        constexpr float kPi2 = 3.14159265358979f;
                        gunPitchDeg = std::atan2(f2[1], flat2) * 180.0f / kPi2;
                        gunYawDeg = std::atan2(-f2[0], -f2[2]) * 180.0f / kPi2;
                    }
                }
            }
        }
    }

    // ---- Pointing at the mod menu (2026-09-23) -----------------------------
    // The menu is drawn into the picture at a fixed place, so in a headset it
    // is fixed to your head - which means a place on it IS a direction from
    // where you are looking. So that is all this works out: the gun hand's
    // forward, turned into the head's own frame, as an angle across and an
    // angle up. The menu turns those into pixels, because only it knows how
    // wide it is drawn.
    //
    // Both poses are brought into the same frame the holsters use below: the
    // tracking frame turns a direction from the runtime's space into the
    // recentred one, and the eye's rotation is already in that.
    {
        const int pointHand = gunIsLeft ? kLeft : kRight;
        XRBridgeEyeView pL, pR;
        float pFrame[9];
        if (!settings.menuLaser || !handPose[pointHand].tracked || !VRBridge_GetEyeViews(pL, pR)
            || !VRBridge_GetTrackingFrame(pFrame)) {
            g_haveMenuPoint = false;
        } else {
            const float* fwd = handPose[pointHand].rot + 6;
            float inRef[3] = {};
            for (int axis = 0; axis < 3; ++axis)
                for (int k = 0; k < 3; ++k)
                    inRef[axis] += fwd[k] * pFrame[axis * 3 + k];
            const float* H = pL.rotationDelta; // rows: right, up, forward
            const float hx = H[0] * inRef[0] + H[1] * inRef[1] + H[2] * inRef[2];
            const float hy = H[3] * inRef[0] + H[4] * inRef[1] + H[5] * inRef[2];
            const float hz = H[6] * inRef[0] + H[7] * inRef[1] + H[8] * inRef[2];
            if (hz > 0.15f) { // pointing behind you is not pointing at it
                g_menuPointX = std::atan2(hx, hz);
                g_menuPointY = std::atan2(hy, hz);
                g_haveMenuPoint = true;
            } else {
                g_haveMenuPoint = false;
            }
            // And where the hand IS, by the same route: the line from your
            // head to your hand, put through the same mapping, is where the
            // controller appears on screen. Drawing from there to the cursor
            // is a beam that comes out of the thing in your hand.
            {
                float headM[3];
                for (int k = 0; k < 3; ++k)
                    headM[k] = (pL.positionMeters[k] + pR.positionMeters[k]) * 0.5f;
                const float toHand[3] = { handPose[pointHand].posMeters[0] - headM[0],
                    handPose[pointHand].posMeters[1] - headM[1],
                    handPose[pointHand].posMeters[2] - headM[2] };
                float handRef[3] = {};
                for (int axis = 0; axis < 3; ++axis)
                    for (int k = 0; k < 3; ++k)
                        handRef[axis] += toHand[k] * pFrame[axis * 3 + k];
                const float gx = H[0] * handRef[0] + H[1] * handRef[1] + H[2] * handRef[2];
                const float gy = H[3] * handRef[0] + H[4] * handRef[1] + H[5] * handRef[2];
                const float gz = H[6] * handRef[0] + H[7] * handRef[1] + H[8] * handRef[2];
                if (gz > 0.02f) {
                    g_menuHandX = std::atan2(gx, gz);
                    g_menuHandY = std::atan2(gy, gz);
                    g_haveMenuHand = true;
                } else {
                    g_haveMenuHand = false;
                }
            }
            g_menuClick = gun.trigger > 0.5f;
            g_menuRightClick = gun.squeeze > 0.5f;
            g_menuScroll = gun.stickY;
        }
#if RE5VR_DIAGNOSTICS
        {
            static unsigned long long s_toldPoint = 0;
            const unsigned long long nowPoint = GetTickCount64();
            if (nowPoint - s_toldPoint >= 2000) {
                s_toldPoint = nowPoint;
                Log_Printf("MenuLaser: %s - hand %s, pointing %.1f deg across and %.1f deg up, trigger %s",
                    g_haveMenuPoint ? "pointing" : "nothing to point with",
                    handPose[gunIsLeft ? kLeft : kRight].tracked ? "tracked" : "NOT tracked",
                    g_menuPointX * 57.2957795f, g_menuPointY * 57.2957795f, g_menuClick ? "down" : "up");
            }
        }
#endif
    }

    // ---- Holsters (2026-09-19) ---------------------------------------------
    // Reach to a place on your body and squeeze, and that place is a weapon.
    // The four d-pad zones are the slots you have already assigned in the
    // inventory, so the mod never learns what a weapon is; it presses the
    // direction you would have pressed.
    //
    // The zones are boxes in BODY coordinates: the gun hand minus the head,
    // read off against the recentre reference and turned back by how far you
    // have turned since. Every number below was MEASURED off the player rather
    // than guessed, over two passes, and the first pass is why it took two: it
    // put the right hip further behind the body than the lower back was, and
    // no box can separate those.
    //
    // What separates what, which is the whole design:
    //   * shoulders from hips, by height
    //   * left shoulder from right, by side
    //   * right shoulder from a raised gun, by front-to-back
    //   * right hip from the lower back, by side
    //   * left hip from the lower back, by front-to-back
    // Nothing is resolved on two weak signals at once, and there is a dead
    // band of height between the shoulders and the hips where nothing draws.
    //
    // The zone is read on the grip's PRESS, never continuously. Your hand
    // passes through a hip on its way to your back, but it is not squeezing
    // while it does, so the boxes can be generous without being trigger-happy.
    // That is the opposite of how the melee gesture had to behave, and the
    // reason is that this one has a button behind it.
    {
        // Height. Shoulders are at head level, hips are two thirds of a metre
        // below it, and the gap between is deliberately dead.
        constexpr float kHighUp = -0.25f;
        // Lower, and further out to the side, than the first measurements
        // asked for (2026-09-23, user: "I was leaning over a ladder and trying
        // to shoot someone and my grip was too close to the holster and
        // wouldn't go to aim mode ... I think we need to shrink the holsters a
        // bit"). Everything here is measured against your HEAD, so leaning
        // forward over a rail brings your head down to meet a hand that never
        // moved, and a hand held normally in front of you reads as a hand at
        // your hip. Costing a little reach is the right trade: a holster you
        // have to stretch for is an annoyance, a holster that eats the aim
        // button while something is shooting at you is not.
        constexpr float kLowUp = -0.55f;
        // Side. The right hip measured +0.30 and the lower back -0.12.
        constexpr float kRightSide = 0.22f;
        constexpr float kLeftSide = -0.22f;
        // The two shoulders get their own boundary, and it does not sit
        // halfway between them (2026-09-19). Measured, the left shoulder is
        // -0.30 and the right +0.06, so halfway would be -0.12 - but reaching
        // ACROSS your body is a longer movement than reaching over your own
        // shoulder, so a slightly short reach for the knife lands near the
        // middle and came out as the wrong one: the user reached over the left
        // shoulder and drew what was on the right. The boundary belongs nearer
        // whichever zone is harder to reach.
        constexpr float kShoulderSplit = -0.08f;
        // Front to back. The left hip measured +0.10 and the lower back -0.33.
        // The back asks for far less depth than that measurement suggests,
        // because of WHERE it is (2026-09-19): reaching behind your own back
        // puts the controller where the headset cannot see it, and an
        // inside-out runtime coasts on the last pose it had. The hand stops
        // updating somewhere around the hip, so it never reads as far back as
        // it really is, and across a whole session the zone did not fire once.
        // Asking for less depth is the fix that survives the tracking.
        constexpr float kBehind = -0.05f;
        // The back is also tested BEFORE the hips, and bounded by SIDE rather
        // than by depth, because the right hip measures further back (-0.25)
        // than anyone would guess and only its side tells the two apart.
        constexpr float kBackSide = 0.22f;
        // The centre of the chest was tried here and taken straight back out
        // (2026-09-19), on the user's objection rather than on a test: "I'm
        // afraid of the chest, due to the player aiming." Which is right. The
        // band of height between the shoulders and the hips is empty and a
        // chest zone fits it neatly, but the one pose a gun hand holds more
        // than any other passes through exactly there, and a holster that
        // sometimes fires while you aim is worse than a holster that is only
        // hard to reach. The dead band stays dead.
        // A raised gun sits around (+0.17, -0.20, +0.48), which is inside the
        // shoulders' height band, so both shoulder zones have to exclude it on
        // some other axis: the left one by side, the right one by depth.
        constexpr float kShoulderFwd = -0.05f;
        constexpr float kAcrossFwd = 0.25f;

        enum Zone { kNone, kKnife, kSlotRight, kSlotLeft, kSlotUp, kSlotDown };

        static bool s_wasGrip = false;
        static bool s_consumed = false; // this grip hold was a draw, so not an aim
        static float s_bodyYaw = 0.0f;
        static bool s_haveYaw = false;
        static ULONGLONG s_pressUntil = 0;
        static WORD s_pressButton = 0;
        static bool s_wristIn = false;
        static float s_wristWas = 0.0f;
        static ULONGLONG s_wristAgain = 0;

        const int gunIdx = gunIsLeft ? kLeft : kRight;
        const int offIdx = 1 - gunIdx;
        const ULONGLONG nowHol = GetTickCount64();
        const bool gripNow = gun.squeeze > 0.5f;

        XRBridgeEyeView eyeL, eyeR;
        float frame[9];
        const bool haveHead = VRBridge_GetEyeViews(eyeL, eyeR) && VRBridge_GetTrackingFrame(frame);

        float bodyRight = 0.0f, bodyUp = 0.0f, bodyFwd = 0.0f;
        float ref[3] = {};
        float headYaw = 0.0f;
        bool haveBody = false;
        if (haveHead && handPose[gunIdx].tracked) {
            float head[3];
            for (int k = 0; k < 3; ++k)
                head[k] = (eyeL.positionMeters[k] + eyeR.positionMeters[k]) * 0.5f;

            const float* look = &eyeL.rotationDelta[6];
            headYaw = std::atan2(look[0], look[2]);

            // The body follows the head lazily, and only past a deadband: you
            // turn your head to look at things constantly and your hips do not
            // go with it. Widened 40 -> 60 degrees after the measurement pass
            // (2026-09-19). At a 79 degree head turn with the body deliberately
            // held still, the old band had already dragged the model 15 degrees
            // round, which moved the hip zone about ten centimetres and would
            // have missed the draw. Looking over your shoulder is common and
            // turning on the spot is not, so the band should cover the look.
            constexpr float kPiF = 3.14159265358979f;
            constexpr float kDeadband = 60.0f * kPiF / 180.0f;
            if (!s_haveYaw) {
                s_bodyYaw = headYaw;
                s_haveYaw = true;
            } else {
                float turn = headYaw - s_bodyYaw;
                while (turn > kPiF)
                    turn -= 2.0f * kPiF;
                while (turn < -kPiF)
                    turn += 2.0f * kPiF;
                if (turn > kDeadband)
                    s_bodyYaw += turn - kDeadband;
                else if (turn < -kDeadband)
                    s_bodyYaw += turn + kDeadband;
            }

            float rel[3];
            for (int k = 0; k < 3; ++k)
                rel[k] = handPose[gunIdx].posMeters[k] - head[k];
            for (int axis = 0; axis < 3; ++axis)
                for (int k = 0; k < 3; ++k)
                    ref[axis] += rel[k] * frame[axis * 3 + k];
            const float ca = std::cos(-s_bodyYaw), sa = std::sin(-s_bodyYaw);
            bodyRight = ref[0] * ca + ref[2] * sa;
            bodyUp = ref[1];
            bodyFwd = -ref[0] * sa + ref[2] * ca;
            haveBody = true;
        }

        // The player's own handedness mirrors the zones, because that is a
        // property of the person and never changes between characters.
        //
        // The CHARACTER's handedness deliberately does not (2026-09-20).
        // Mirroring for Sheva would be the anatomically natural thing, since a
        // left hand reaches across to the RIGHT shoulder - but it would move
        // every holster the moment you changed character, and staying put is
        // worth more than being natural: "we should keep it consistent on the
        // holsters, otherwise players may get confused". The knife is over your
        // left shoulder whoever you are playing. Only the hand that draws it
        // changes.
        const float side = settings.swapHands ? -bodyRight : bodyRight;

        Zone zone = kNone;
        if (haveBody) {
            if (bodyUp > kHighUp) {
                if (side < kShoulderSplit && bodyFwd < kAcrossFwd)
                    zone = kKnife;
                else if (side >= kShoulderSplit && bodyFwd < kShoulderFwd)
                    zone = kSlotUp;
            } else if (bodyUp < kLowUp) {
                if (bodyFwd < kBehind && side < kBackSide)
                    zone = kSlotDown;
                else if (side > kRightSide)
                    zone = kSlotRight;
                else if (side < kLeftSide)
                    zone = kSlotLeft;
            }
        }

        if (gripNow && !s_wasGrip) {
            if (settings.holsterMeasure) {
                float wrist = -1.0f, wr = 0.0f, wu = 0.0f, wf = 0.0f;
                if (handPose[offIdx].tracked) {
                    float w[3];
                    for (int k = 0; k < 3; ++k)
                        w[k] = handPose[gunIdx].posMeters[k] - handPose[offIdx].posMeters[k];
                    wrist = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
                    const float* m = handPose[offIdx].rot;
                    for (int k = 0; k < 3; ++k) {
                        wr += w[k] * m[0 + k];
                        wu += w[k] * m[3 + k];
                        wf += w[k] * m[6 + k];
                    }
                }
                static const char* const kZoneName[] = { "nothing", "knife", "right hip", "left hip",
                    "right shoulder", "lower back" };
                constexpr float kDeg = 57.2957795f;
                Log_Printf("Holster: right %+.2f  up %+.2f  fwd %+.2f  (raw %+.2f %+.2f %+.2f)  "
                           "head %.0f deg  body %.0f deg | other hand %.2f m, in its own frame "
                           "right %+.2f up %+.2f fwd %+.2f -> %s",
                    bodyRight, bodyUp, bodyFwd, ref[0], ref[1], ref[2], headYaw * kDeg,
                    s_bodyYaw * kDeg, wrist, wr, wu, wf, kZoneName[zone]);
            }

            // A draw that lands nowhere is invisible, and a whole session went
            // by without knowing why the lower back did nothing. So a grip
            // that misses while the hand is somewhere holster-shaped says so.
            // A raised gun sits around up -0.20 and well in front, which is
            // neither, so ordinary aiming never reaches this.
            if (settings.holsters && zone == kNone && haveBody
                && (bodyUp < kLowUp || (bodyUp > kHighUp && bodyFwd < kShoulderFwd))) {
                static ULONGLONG s_saidMissedAt = 0;
                if (nowHol - s_saidMissedAt > 700) {
                    s_saidMissedAt = nowHol;
                    Log_Printf("Holster: reached to right %+.2f up %+.2f fwd %+.2f and found nothing",
                        bodyRight, bodyUp, bodyFwd);
                }
            }

            if (settings.holsters && zone != kNone) {
                // The press is CONSUMED. Otherwise every draw raises your
                // weapon on the way out, because this grip is also aim.
                s_consumed = true;
                if (zone == kKnife) {
                    g_knifeLatched = !g_knifeLatched;
                    Log_Printf("Holster: knife %s", g_knifeLatched ? "out" : "away");
                } else {
                    // Drawing anything else puts the knife away, which is what
                    // the character would do anyway.
                    g_knifeLatched = false;
                    s_pressButton = zone == kSlotRight ? XINPUT_GAMEPAD_DPAD_RIGHT
                        : zone == kSlotLeft            ? XINPUT_GAMEPAD_DPAD_LEFT
                        : zone == kSlotUp              ? XINPUT_GAMEPAD_DPAD_UP
                                                       : XINPUT_GAMEPAD_DPAD_DOWN;
                    s_pressUntil = nowHol + 100; // a tap, not a hold
                    Log_Printf("Holster: drew from the %s", zone == kSlotRight ? "right hip"
                            : zone == kSlotLeft ? "left hip"
                            : zone == kSlotUp   ? "right shoulder"
                                                : "lower back");
                }
                XrInput_Pulse(gunIdx, 0.6f, 0.05f);
            } else if (settings.holsters && g_knifeLatched) {
                // Gripping anywhere that is not a holster is you raising your
                // weapon, and you cannot aim with a knife out.
                g_knifeLatched = false;
            }
        }
        if (!gripNow)
            s_consumed = false;
        if (s_consumed)
            pad.bLeftTrigger = 0;
        if (nowHol < s_pressUntil)
            pad.wButtons |= s_pressButton;
        if (g_knifeLatched)
            pad.wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
        s_wasGrip = gripNow;

        // ---- The wrist tap -------------------------------------------------
        // You tap a watch, you do not squeeze it, so this one has no button
        // behind it at all. That means it has to be read from nothing, and the
        // melee lesson applies directly: a gesture read from nothing needs a
        // second signal. Here it is the approach. The hand has to ARRIVE,
        // closing on the wrist, not merely be near it, because hands rest near
        // each other all the time.
        //
        // The top of the wrist is a DIRECTION, so the offset is taken in the
        // other hand's own frame, where the measured tap sat at -0.17 along
        // that hand's right axis with almost nothing up or forward.
        if (settings.wristInventory && handPose[gunIdx].tracked && handPose[offIdx].tracked) {
            constexpr float kTapReach = 0.26f;   // metres between the two grips
            constexpr float kTapSide = 0.06f;    // and this far onto the watch-face side of that hand
            constexpr float kTapClosing = 0.22f; // metres a second, so a tap and not a rest

            float w[3];
            for (int k = 0; k < 3; ++k)
                w[k] = handPose[gunIdx].posMeters[k] - handPose[offIdx].posMeters[k];
            const float gap = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            float wr = 0.0f;
            const float* m = handPose[offIdx].rot;
            for (int k = 0; k < 3; ++k)
                wr += w[k] * m[0 + k];

            // WHICH SIDE IS THE WATCH ON (2026-09-25, user: "I find myself
            // accidentally opening the inventory up on wrist tap - even more so
            // if I'm playing as sheva").
            //
            // The "even more so" is the clue and it names the bug. This measures
            // along the OFF hand's own right axis and wants a negative number,
            // which was measured with the gun in the right hand. Swap the hands
            // over and that axis points the other way across your body, so the
            // test stops meaning "at the watch" and starts meaning "anywhere on
            // this side of the other hand" - an enormous region you pass through
            // constantly. As Sheva the gesture was barely a gesture.
            //
            // And the threshold is further out now, six centimetres rather than
            // three, with the hands twenty-six apart rather than thirty. The
            // place is what tightens, never the force: it took a slap to work
            // once before and that is not a road worth going back down.
            const bool gunInLeft = settings.swapHands != settings.characterLeftHanded;
            const float side = gunInLeft ? wr : -wr;
            const bool inside = gap < kTapReach && side > kTapSide;
            // The fastest it has closed RECENTLY, not on this one frame
            // (2026-09-23, user: "tapping the wrist requires a really
            // aggressive slap of the wrist, instead of a nice and simple
            // tap"). The speed was read off the single frame on which your
            // hand crossed into range, and a tap you aim rather than throw has
            // already begun slowing down by then - so the only thing that
            // reliably passed was a slap. Held over a fifth of a second and
            // decaying, a deliberate approach still counts when it lands, and
            // two hands resting near each other still never do.
            const float rate = s_wristWas > 0.0f ? (s_wristWas - gap) * 90.0f : 0.0f;
            static float s_wristPeak = 0.0f;
            s_wristPeak *= 0.88f;
            if (rate > s_wristPeak)
                s_wristPeak = rate;
            // And never with both hands on the gun. Two-handing brings your
            // hands together and holds them there, which is the one posture that
            // satisfies every part of this test by accident.
            const bool twoHanded = XrInput_GetTwoHand(nullptr);
            if (inside && !s_wristIn && s_wristPeak > kTapClosing && !gripNow && !twoHanded
                && !g_reloadCarrying && nowHol > s_wristAgain) {
                s_wristAgain = nowHol + 700;
                s_pressButton = XINPUT_GAMEPAD_Y;
                s_pressUntil = nowHol + 100;
                s_wristPeak = 0.0f;
                XrInput_Pulse(offIdx, 0.5f, 0.04f);
                Log_Printf("Holster: wrist tap at %.2f m, %.2f m onto the watch side, closing %.2f m/s", gap,
                    side, rate);
            }
            // WHERE IT IS, WHEN YOU CANNOT SEE IT (2026-09-26, user: "when we
            // hide the arms in hand mode, wrist tap doesnt work anymore cause
            // there is no wrist to tap on").
            //
            // The gesture itself is untouched by hiding - it is read from the
            // two controllers and never looks at the body - but it wants a
            // real place, six centimetres onto the watch side of a hand that
            // is no longer drawn. Aiming at it blind is guesswork.
            //
            // So the wrist announces itself. A faint tick on the OFF hand the
            // moment the gun hand crosses into the zone, well under the pulse
            // that confirms a tap, so sweeping your hand across finds the spot
            // by feel and the tap itself still feels like the event. Only when
            // there is nothing to look at: Arms and Full body both draw the
            // forearm, and a buzz every time your hands pass would be noise.
            if (inside && !s_wristIn && settings.bodyVisible >= 2 && !gripNow && !twoHanded
                && !g_reloadCarrying)
                XrInput_Pulse(offIdx, 0.12f, 0.02f);
            s_wristIn = inside;
            s_wristWas = gap;
        }

        // ---- Reloading by hand (2026-09-25) --------------------------------
        // Reach to your belt on your support side and squeeze: the magazine in
        // the weapon drops and a fresh one is in your hand. Bring that hand up
        // to the weapon and it goes in, filled with as much as you are carrying
        // - min(capacity, what is left) - which is why a gun upgraded to hold a
        // hundred takes a hundred and one with twelve rounds left takes twelve.
        // Take your hand away again and it is racked.
        //
        // Two things make this safe to read off a grip that already means
        // several other things. The first is WHERE it starts: at your belt, on
        // your support side, which is nowhere near the weapon and nowhere near
        // a holster - those are read off the GUN hand. The second is that once
        // it has started, the two gestures that share its finishing position
        // are told to stand down, so bringing a magazine to the gun cannot
        // two-hand it and cannot open the inventory.
        //
        // Letting go before the magazine arrives puts the old one back exactly
        // as it was. An abandoned reload should cost you the time it wasted and
        // nothing else.
        if (settings.manualReload && haveHead && handPose[gunIdx].tracked && handPose[offIdx].tracked) {
            // The belt, measured against the head like every other zone here,
            // so leaning forward brings the pouch to meet your hand.
            constexpr float kPouchUp = -0.42f;
            constexpr float kPouchNear = 0.02f; // on your support side, not across the middle
            constexpr float kPouchFar = 0.45f;
            constexpr float kPouchBack = -0.20f;
            constexpr float kPouchFwd = 0.45f;
            constexpr float kSeatMetres = 0.20f; // the magazine reaching the well
            constexpr ULONGLONG kGiveUpMs = 15000;

            static int s_stage = 0; // 0 idle, 1 carrying, 2 in and waiting for the rack
            static unsigned char* s_mag = nullptr;
            static int s_hadLoaded = 0, s_hadReserve = 0;
            static ULONGLONG s_stageAt = 0;
            static int s_seatedWith = 0; // what the magazine held when the game was asked
            static bool s_wasOffGrip = false;

            float headAt[3];
            for (int k = 0; k < 3; ++k)
                headAt[k] = (eyeL.positionMeters[k] + eyeR.positionMeters[k]) * 0.5f;
            float toOff[3], offRef[3] = {};
            for (int k = 0; k < 3; ++k)
                toOff[k] = handPose[offIdx].posMeters[k] - headAt[k];
            for (int axis = 0; axis < 3; ++axis)
                for (int k = 0; k < 3; ++k)
                    offRef[axis] += toOff[k] * frame[axis * 3 + k];
            const float cb = std::cos(-s_bodyYaw), sb = std::sin(-s_bodyYaw);
            const float offRight = offRef[0] * cb + offRef[2] * sb;
            const float offUp = offRef[1];
            const float offFwd = -offRef[0] * sb + offRef[2] * cb;
            // Your support side, whichever hand it happens to be.
            const float ownSide = gunIsLeft ? offRight : -offRight;
            const bool atThePouch = offUp < kPouchUp && ownSide > kPouchNear && ownSide < kPouchFar
                && offFwd > kPouchBack && offFwd < kPouchFwd;

            float span[3];
            for (int k = 0; k < 3; ++k)
                span[k] = handPose[offIdx].posMeters[k] - handPose[gunIdx].posMeters[k];
            const float toGun = std::sqrt(span[0] * span[0] + span[1] * span[1] + span[2] * span[2]);
            const bool offGrip = off.squeeze > 0.5f;

            // LET GO AND YOU DROP IT (2026-09-25, user: "if ya let go early,
            // dropping the mag from your hand to the floor would be great. But
            // having it re-eject from the gun is silly").
            //
            // One rule, and it is the one your hands already expect: a
            // magazine you stop holding falls. It does not fly back into the
            // weapon, and the weapon does not eject a second magazine it never
            // had. The gun stays empty and you reach again.
            //
            // Nothing is lost by this. The rounds that were in the old
            // magazine went into your spare ammunition the moment it came out,
            // so an abandoned reload costs exactly the time it wasted - which
            // is the right price, and the reason there is no undo here at all.
            const auto dropIt = [&](const char* why) {
                Log_Printf("Reload: %s - the magazine falls", why);
                // A dropped magazine keeps nothing: those rounds are on the
                // floor with it, and the next reload is a full one.
                g_ejectedRounds = 0;
                s_stage = 0;
                s_mag = nullptr;
                g_magInHand = false;
                g_reloadCarrying = false;
            };

            if (s_stage == 0) {
                if (offGrip && !s_wasOffGrip && atThePouch) {
                    AmmoSlot mag;
                    if (!Ammo_Held(mag)) {
                        Log_Printf("Reload: nothing in hand that takes a magazine");
                    } else if (mag.loaded >= mag.capacity) {
                        Log_Printf("Reload: already full, %d of %d", mag.loaded, mag.capacity);
                    } else if (mag.reserve <= 0) {
                        Log_Printf("Reload: no ammunition left for this weapon");
                        XrInput_Pulse(offIdx, 0.3f, 0.03f);
                    } else {
                        s_mag = mag.mag;
                        s_hadLoaded = mag.loaded;
                        s_hadReserve = mag.reserve;
                        // Usually already gone, on the reload button. Reaching
                        // for a fresh one with the old still in drops it too,
                        // so the gesture works on its own and the button is a
                        // shortcut rather than a step you can forget.
                        if (mag.loaded > 0) {
                            g_ejectedRounds = mag.loaded;
                            Ammo_Eject(settings.reloadKeepsRounds);
                            g_magOutAt = nowHol;
                            if (settings.reloadSounds)
                                Sound_Play("magout");
                        }
                        s_stage = 1;
                        s_stageAt = nowHol;
                        g_magInHand = true;
                        g_reloadCarrying = true;
                        XrInput_Pulse(offIdx, 0.7f, 0.05f);
                        Log_Printf("Reload: magazine out at belt right %+.2f up %+.2f fwd %+.2f - had %d of %d, %d spare",
                            offRight, offUp, offFwd, mag.loaded, mag.capacity, mag.reserve);
                    }
                }
            } else if (s_stage == 1) {
                if (!offGrip) {
                    dropIt("let go before it got there");
                } else if (nowHol - s_stageAt > kGiveUpMs) {
                    dropIt("carried too long");
                } else if (toGun < kSeatMetres) {
                    // THE GAME DOES THE LOADING (2026-09-25, user: "the nice
                    // thing about the in game reload function, is that it also
                    // has the sound effects of the reload ... and animations of
                    // a mag ejecting").
                    //
                    // Two reasons, and the second is the one that matters. The
                    // first is that a reload should sound and look like one,
                    // and the game already has both. The second is the spare
                    // rounds: writing them does not stick, because the count in
                    // the weapon is a mirror of an inventory that has not been
                    // found - so a reload this mod performs itself is free
                    // ammunition, however honest its arithmetic looks. Asking
                    // the game to reload hands the accounting back to the code
                    // that owns it, and upgrades, stacks and caps all come with
                    // it for nothing.
                    //
                    // What is still ours is WHEN. The magazine left on the
                    // button press and the gun has been empty since; this is
                    // the moment your hand puts a new one in, and the game's
                    // reload plays from here.
                    // The rounds that were in the old magazine, put back for
                    // one frame. The game reloads from whatever it finds, so
                    // eight in the weapon costs two rounds rather than ten,
                    // and the eight you were carrying are neither lost nor
                    // invented. Off, they leave with the magazine and the
                    // game bills you for a full one.
                    if (settings.reloadKeepsRounds && g_ejectedRounds > 0) {
                        if (Ammo_SetLoaded(static_cast<int>(g_ejectedRounds)))
                            Log_Printf("Reload: %ld round(s) carried over, so the game only pays for the rest",
                                g_ejectedRounds);
                    }
                    // WHAT WE HANDED THE GAME (2026-09-26, user: "when keep
                    // ammunition is on, the weapon animation plays out, it
                    // doesn't freeze mag as we intended").
                    //
                    // Because the next stage was waiting for the count to rise
                    // above zero, and with the rounds carried over it was
                    // already above zero on the frame it started. The reload
                    // read as finished before it began, the freeze came off
                    // immediately, and the animation had the magazine back.
                    //
                    // So it waits for the count to rise above what WE put
                    // there, not above nothing. With the carry over off that is
                    // zero and nothing changes.
                    s_seatedWith = settings.reloadKeepsRounds ? static_cast<int>(g_ejectedRounds) : 0;
                    g_ejectedRounds = 0;
                    s_stage = 2;
                    s_stageAt = nowHol;
                    g_reloadAskAt = nowHol;
                    // AND THE MOD LETS GO HERE (2026-09-25, user: "the magazine
                    // isn't disappearing from my hand and letting the animation
                    // drive it once you get close to the gun - funnily enough,
                    // the bullet does").
                    //
                    // It was held for the whole second the game took to load,
                    // because carrying and reloading were the same flag. They
                    // are two different things: your hand is finished the
                    // moment the magazine touches the weapon, and from that
                    // instant the animation owns it. The reload carries on, and
                    // the wrist tap and the support hand stay suppressed until
                    // it finishes, but nothing is written to the bone any more.
                    g_magInHand = false;
                    g_gameReloading = true;
                    // The magazine going home. The game's reload starts on the
                    // same frame, so this sits under the front of it rather
                    // than replacing it.
                    if (settings.reloadSounds)
                        Sound_Play("magin");
                    XrInput_Pulse(offIdx, 1.0f, 0.07f);
                    XrInput_Pulse(gunIdx, 1.0f, 0.07f);
                    Log_Printf("Reload: magazine in - asking the game to load it");
                }
            } else {
                // Waiting for that reload to land. It is the game's now, so all
                // this does is notice that it happened, and notice if it did
                // not: a press the game ignores would otherwise leave you
                // standing there with an empty weapon and no way to tell why.
                AmmoSlot now;
                const bool loaded = Ammo_Held(now) && now.loaded > s_seatedWith;
                if (loaded) {
                    s_stage = 0;
                    s_mag = nullptr;
                    g_gameReloading = false;
                    g_reloadCarrying = false;
                    Log_Printf("Reload: loaded, %d of %d", now.loaded, now.capacity);
                } else if (nowHol - s_stageAt > 2500) {
                    // It did not take. Put the rounds in ourselves rather than
                    // leave the weapon empty, and say so, because a fallback
                    // that runs quietly is a bug nobody ever finds.
                    int took = 0;
                    if (Ammo_Seat(&took))
                        Log_Printf("Reload: the game did not take the press - loaded %d round(s) by hand, and the "
                                   "spare count will not stick until the inventory is found",
                            took);
                    else
                        Log_Printf("Reload: the game did not take the press and there was nothing to load");
                    s_stage = 0;
                    s_mag = nullptr;
                    g_gameReloading = false;
                    g_reloadCarrying = false;
                }
            }
            s_wasOffGrip = offGrip;
        } else if (g_reloadCarrying || g_magInHand || g_gameReloading) {
            g_reloadCarrying = false;
            g_magInHand = false;
            g_gameReloading = false;
        }
    }

    // ---- Physical melee (2026-09-19) ---------------------------------------
    // Hold the knife grip and swing, and the character swings. RE5 has no melee
    // input of its own to go hunting for: the knife comes up on the left grip
    // and swings on the trigger, so this is the trigger half of a press the
    // player is already halfway through making.
    //
    // The grip is REQUIRED, and that is the whole safety of it (2026-09-19).
    // The first cut drew the knife for you off a bare swing, and the user
    // tripped it by whipping the gun around: "it may be best to require left
    // grip to be held, and just use the swing then to actually swing the
    // attack, rather than pull the knife." Which is right. A gesture that can
    // put a weapon in your hand has to be read from nothing at all, and a hand
    // moving fast is not a rare enough thing to read it from. Behind the grip
    // there is no false positive worth the name: the knife is already out, you
    // meant to use it, and the worst a stray swing costs is a swing.
    //
    // Speed is one hand measured AGAINST the other. Walking, turning and
    // leaning carry both hands along together and cancel to nothing; a swing
    // moves one hand and not the other, so the difference is the swing itself.
    // That costs no head pose and no extra locate.
    if (settings.meleeSwing) {
        static bool s_have = false;
        static XrTime s_prevTime = 0;
        static float s_prevPos[2][3] = {};
        static float s_speed = 0.0f;      // smoothed, one hand against the other
        static float s_lead = 0.0f;       // and where it is heading, a swing's wind-up ahead
        static float s_own[2] = {};       // each hand's own speed, to pick which one buzzes
        static ULONGLONG s_gripSince = 0; // when the knife grip went down
        static ULONGLONG s_strikeUntil = 0;
        static ULONGLONG s_lastSwing = 0;

        const ULONGLONG nowMelee = GetTickCount64();
        const bool bothTracked = handPose[kLeft].tracked && handPose[kRight].tracked;
        const double dt = s_have ? static_cast<double>(displayTime - s_prevTime) * 1e-9 : 0.0;
        if (bothTracked && s_have && dt > 0.002 && dt < 0.1) {
            float v[2][3];
            for (int i = 0; i < 2; ++i)
                for (int k = 0; k < 3; ++k)
                    v[i][k] = static_cast<float>((handPose[i].posMeters[k] - s_prevPos[i][k]) / dt);
            float rel[3];
            for (int k = 0; k < 3; ++k)
                rel[k] = v[kRight][k] - v[kLeft][k];
            const float relSpeed = std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
            // A third of the old value. It was half, which is a frame of lag,
            // and a frame of lag is the one thing this cannot afford: behind
            // the grip a noisy reading costs a spare swing, while a late one
            // costs a hit (2026-09-19).
            const float wasSpeed = s_speed;
            s_speed = s_speed * 0.35f + relSpeed * 0.65f;
            // Fire on where the swing is GOING, not where it is. By the time a
            // hand is fast enough to cross the line it is already mid-stroke,
            // and RE5's own knife wind-up runs after that, so the blade arrives
            // behind the target: "there's a few swings where I make it all the
            // way through the character". Projecting the speed forward by a
            // wind-up's worth fires at the start of the stroke instead, and a
            // hand that is SLOWING projects downward, so the tail of a swing
            // cannot start another one.
            // Trimmed hard (2026-09-19): the swing became "overly sensitive".
            // Two things had stacked up. A cap of 2.5 against a threshold of
            // 3.0 let a hand at barely 1.5 project its way over the line, so
            // the projection had stopped assisting the measurement and had
            // become it. And the holster latch made it worse in a way that has
            // nothing to do with this arithmetic: the knife now stays out on
            // its own, so the gesture is armed the whole time instead of only
            // while a grip is physically held.
            constexpr float kLeadSec = 0.07f;
            constexpr float kLeadCap = 1.0f; // metres a second; accel is noisy
            float accel = static_cast<float>((s_speed - wasSpeed) / dt);
            float lead = accel * kLeadSec;
            if (lead > kLeadCap)
                lead = kLeadCap;
            if (lead < -kLeadCap)
                lead = -kLeadCap;
            s_lead = s_speed + lead;
            for (int i = 0; i < 2; ++i)
                s_own[i] = std::sqrt(v[i][0] * v[i][0] + v[i][1] * v[i][1] + v[i][2] * v[i][2]);
        } else if (!bothTracked) {
            s_speed = 0.0f;
            s_lead = 0.0f;
        }
        for (int i = 0; i < 2; ++i)
            for (int k = 0; k < 3; ++k)
                s_prevPos[i][k] = handPose[i].posMeters[k];
        s_prevTime = displayTime;
        s_have = bothTracked;

        // The knife has to be UP, not merely called for: the grip going down
        // starts an animation, and a trigger that lands before the knife is
        // out is a wasted swing. A sixth of a second covers it and is under
        // what anyone can press and swing inside of anyway.
        // The latch counts as the knife being up. It is the holster holding
        // the button rather than the player, and the swing cannot tell the
        // difference - but this gate could, and would have silently refused
        // every swing off a shoulder draw.
        const bool knifeUp = (off.squeeze > 0.5f && !settings.holsters) || g_knifeLatched;
        if (!knifeUp)
            s_gripSince = 0;
        else if (!s_gripSince)
            s_gripSince = nowMelee;
        const bool ready = s_gripSince && nowMelee - s_gripSince > 150;

        const bool aiming = gun.squeeze > 0.5f;
        const bool swinging = nowMelee < s_strikeUntil;
        if (ready && !swinging && !aiming && s_lead > settings.meleeSwingSpeed
            && nowMelee - s_lastSwing > 400) {
            s_lastSwing = nowMelee;
            const float atSpeed = s_speed, atLead = s_lead;
            s_strikeUntil = nowMelee + 140;
            s_speed = s_lead = 0.0f; // so the tail of one swing cannot start another
            XrInput_Pulse(s_own[kRight] >= s_own[kLeft] ? kRight : kLeft, 0.8f, 0.07f);
            // Both numbers, because the gap between them IS the head start.
            Log_Printf("Melee: swing at %.1f m/s, heading for %.1f", atSpeed, atLead);
        }
        // The grip is the player's own and already reaches the pad as the left
        // shoulder, so all that is added here is the trigger.
        if (nowMelee < s_strikeUntil)
            pad.bRightTrigger = 255;
    }

    // ---- The reload button drops the magazine (2026-09-25) -----------------
    // User: "drop the magazine should just be done regularly holding the aim
    // button, pressing A - this is currently the default reload button".
    //
    // Which is the right place for it. Reaching to your belt is how you get a
    // fresh magazine; getting rid of the old one is not a reach, it is a
    // thumb, and the game already has a button that means exactly this. So the
    // press is TAKEN rather than passed on: the game's answer to it is a whole
    // reload in one go, and the whole point is that a reload is no longer one
    // thing. What comes back is the same press, later, when your hand has
    // actually brought a magazine to the weapon - and then the game's own
    // sound and animation play at the moment they belong to.
    //
    // Only while aiming. Away from the sights A is the action button - a door,
    // a ladder, a partner - and none of that changes because the mod reloads.
    if (settings.manualReload) {
        static bool s_wasReloadPress = false;
        static ULONGLONG s_askUntil = 0;
        const ULONGLONG nowPad = GetTickCount64();
        const bool aiming = pad.bLeftTrigger > 128;
        const bool pressed = (pad.wButtons & XINPUT_GAMEPAD_A) != 0;
        if (aiming) {
            pad.wButtons &= ~static_cast<WORD>(XINPUT_GAMEPAD_A);
            if (pressed && !s_wasReloadPress) {
                AmmoSlot mag;
                if (!Ammo_Held(mag)) {
                    Log_Printf("Reload: nothing in hand that takes a magazine");
                } else if (mag.loaded <= 0) {
                    // Fired dry. There is nothing to take out of the books, but
                    // there is still a magazine in the weapon and you have just
                    // asked for it to leave, which is exactly what anyone does
                    // after the last round. It drops, empty.
                    g_magOutAt = nowPad;
                    if (settings.reloadSounds)
                        Sound_Play("magout");
                    XrInput_Pulse(gunIsLeft ? kLeft : kRight, 0.7f, 0.05f);
                    Log_Printf("Reload: empty magazine dropped");
                } else {
                    g_ejectedRounds = mag.loaded;
                    Ammo_Eject(settings.reloadKeepsRounds);
                    g_magOutAt = nowPad;
                    if (settings.reloadSounds)
                        Sound_Play("magout");
                    XrInput_Pulse(gunIsLeft ? kLeft : kRight, 0.7f, 0.05f);
                    Log_Printf("Reload: magazine dropped, %d of %d was in it", mag.loaded, mag.capacity);
                }
            }
        }
        s_wasReloadPress = pressed && aiming;

        // ---- Nothing to fire (2026-09-25) ----------------------------------
        // User: "we'd also have to block the game from automatically reloading
        // if you try to shoot with 0 bullets in your mag. As it also triggers a
        // reload if a shot is attempted with an empty mag."
        //
        // So the trigger is the last thing the game hears about. Pulling on an
        // empty chamber does nothing at all, which is what an empty weapon
        // does, and the game never gets the chance to helpfully reload it for
        // you - because a reload it starts is the whole reload, and that is the
        // thing this feature exists to take apart.
        //
        // AN EMPTY GUN IS AN EMPTY GUN, HOWEVER IT GOT THERE (2026-09-25,
        // user: "when my current loaded ammo is 0 - i can still shoot again to
        // make the game do the reload - so we aren't blocking it").
        //
        // The first version only held the trigger inside the window the mod
        // itself had opened, from the moment WE dropped a magazine. I scoped it
        // that way on purpose, to keep the mod's hands off the trigger unless
        // it had caused the emptiness - but the commonest way to empty a
        // magazine is to fire it, and that path never opened a window, so the
        // game went on reloading for you exactly as before. A safety that only
        // covers the case you caused is not a safety.
        //
        // So the question is simply whether the weapon has anything in it.
        // Reading the count wrong would mean a weapon that will not fire, which
        // is why it takes a magazine the finder is confident about - one with a
        // real capacity and a sane count - and why it only ever touches the
        // trigger while you are aiming, which is the only time RE5 fires.
        {
            AmmoSlot mag;
            const bool empty = Ammo_Held(mag) && mag.capacity > 0 && mag.loaded <= 0;
            if (empty && aiming) {
                const bool pulling = pad.bRightTrigger > 128;
                pad.bRightTrigger = 0;
                static ULONGLONG s_clickAgain = 0;
                if (pulling && nowPad > s_clickAgain) {
                    s_clickAgain = nowPad + 400;
                    // A dry fire. Without it the weapon does not refuse, it
                    // just fails, and a trigger that produces silence reads as
                    // the mod having broken rather than the gun being empty.
                    if (settings.reloadSounds)
                        Sound_Play("dryfire");
                    XrInput_Pulse(gunIsLeft ? kLeft : kRight, 0.25f, 0.02f);
                }
            }
            if (!empty && g_magOutAt)
                g_magOutAt = 0;
        }

        // And the press given back. Aim goes with it because RE5 reloads from
        // the sights and your hands may well be down by the time the magazine
        // is in; a fifth of a second is long enough for the game to take it and
        // short enough that it is not you aiming.
        if (g_reloadAskAt) {
            s_askUntil = nowPad + 200;
            g_reloadAskAt = 0;
        }
        if (nowPad < s_askUntil) {
            pad.bLeftTrigger = 255;
            pad.wButtons |= XINPUT_GAMEPAD_A;
        }
    } else if (g_magOutAt) {
        // Turned off part way through a reload. Nothing stays held.
        g_magOutAt = 0;
    }

    AcquireSRWLockExclusive(&g_lock);
    g_pad = pad;
    g_padMs = anyHand ? GetTickCount64() : 0;
    g_gunYawDeg = gunYawDeg;
    g_gunPitchDeg = gunPitchDeg;
    g_gunAimMs = gunAimed ? GetTickCount64() : 0;
    g_handPose[kLeft] = handPose[kLeft];
    g_handPose[kRight] = handPose[kRight];
    strcpy_s(g_status, status);
    ReleaseSRWLockExclusive(&g_lock);

    // One line when the controllers arrive or go away, and never per frame.
    static bool s_hadHands = false;
    if (anyHand != s_hadHands) {
        s_hadHands = anyHand;
        Log_Printf("XrInput: motion controllers %s (%s)", anyHand ? "answering" : "silent", status);
    }

    // Which actions the runtime actually bound, once per set of hands. An
    // action set can attach, report active hands, and still have bound
    // nothing that matters.
    static unsigned s_loggedBound[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
    if (anyHand && (hand[kLeft].bound != s_loggedBound[kLeft] || hand[kRight].bound != s_loggedBound[kRight])) {
        s_loggedBound[kLeft] = hand[kLeft].bound;
        s_loggedBound[kRight] = hand[kRight].bound;
        char leftList[128], rightList[128];
        DescribeBound(hand[kLeft].bound, leftList, sizeof(leftList));
        DescribeBound(hand[kRight].bound, rightList, sizeof(rightList));
        Log_Printf("XrInput: bound on the left: %s", leftList);
        Log_Printf("XrInput: bound on the right: %s", rightList);
    }

    // What the controllers are actually saying, while they say anything, at
    // most once a second. This is the line that shows whether a press reaches
    // us at all, and what the pad it becomes looks like.
    const bool anyInput = hand[kLeft].trigger > 0.1f || hand[kRight].trigger > 0.1f || hand[kLeft].squeeze > 0.1f
        || hand[kRight].squeeze > 0.1f || pad.wButtons != 0 || pad.sThumbLX || pad.sThumbLY || pad.sThumbRX
        || pad.sThumbRY;
    static ULONGLONG s_lastValuesMs = 0;
    const ULONGLONG nowMs = GetTickCount64();
    if (anyInput && nowMs - s_lastValuesMs > 1000) {
        s_lastValuesMs = nowMs;
        Log_Printf("XrInput: left trig %.2f grip %.2f stick %.2f,%.2f%s%s%s%s | right trig %.2f grip %.2f "
                   "stick %.2f,%.2f%s%s%s%s -> pad buttons 0x%04X LT %u RT %u L %d,%d R %d,%d",
            hand[kLeft].trigger, hand[kLeft].squeeze, hand[kLeft].stickX, hand[kLeft].stickY,
            hand[kLeft].stickClick ? " click" : "", hand[kLeft].primary ? " X" : "",
            hand[kLeft].secondary ? " Y" : "", hand[kLeft].menu ? " menu" : "", hand[kRight].trigger,
            hand[kRight].squeeze, hand[kRight].stickX, hand[kRight].stickY, hand[kRight].stickClick ? " click" : "",
            hand[kRight].primary ? " A" : "", hand[kRight].secondary ? " B" : "", hand[kRight].menu ? " menu" : "",
            pad.wButtons, pad.bLeftTrigger, pad.bRightTrigger, pad.sThumbLX, pad.sThumbLY, pad.sThumbRX,
            pad.sThumbRY);
    }
}

void XrInput_OnSessionEnding()
{
    for (XrSpace& space : g_aimSpace) {
        if (space != XR_NULL_HANDLE) {
            xrDestroySpace(space);
            space = XR_NULL_HANDLE;
        }
    }
    g_session = XR_NULL_HANDLE;
    g_attached = false;
    AcquireSRWLockExclusive(&g_lock);
    g_padMs = 0;
    g_gunAimMs = 0;
    strcpy_s(g_status, "VR off");
    ReleaseSRWLockExclusive(&g_lock);
}

void XrInput_OnInstanceDestroyed()
{
    // The action set and its actions belong to the instance and go with it.
    g_actionSet = XR_NULL_HANDLE;
    g_trigger = g_squeeze = g_stick = g_stickClick = XR_NULL_HANDLE;
    g_primary = g_secondary = g_menu = g_thumbrest = XR_NULL_HANDLE;
    g_instance = XR_NULL_HANDLE;
    g_session = XR_NULL_HANDLE;
    g_attached = false;
}
