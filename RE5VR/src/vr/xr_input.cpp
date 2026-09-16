#include "xr_input.h"

#include "../util/log.h"

#define XR_USE_PLATFORM_WIN32
#include <windows.h>
#include <openxr/openxr.h>

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
XrAction g_stick = XR_NULL_HANDLE;      // vector2
XrAction g_stickClick = XR_NULL_HANDLE; // bool
XrAction g_primary = XR_NULL_HANDLE;    // bool: A right, X left
XrAction g_secondary = XR_NULL_HANDLE;  // bool: B right, Y left
XrAction g_menu = XR_NULL_HANDLE;       // bool
XrAction g_thumbrest = XR_NULL_HANDLE;  // bool (touch), Touch controllers only
XrAction g_aimPose = XR_NULL_HANDLE;    // pose: where the controller points
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
    ReleaseSRWLockExclusive(&g_lock);
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
        ReadStick(g_stick, p, &h.stickX, &h.stickY, &h, kBoundStick);
        h.stickClick = ReadBool(g_stickClick, p, &h, kBoundStickClick);
        h.primary = ReadBool(g_primary, p, &h, kBoundPrimary);
        h.secondary = ReadBool(g_secondary, p, &h, kBoundSecondary);
        h.menu = ReadBool(g_menu, p, &h, kBoundMenu);
        h.thumbrest = ReadBool(g_thumbrest, p, &h, kBoundThumbrest);
    }

    // Left-handed play swaps which physical controller plays which part.
    const HandState& gun = settings.swapHands ? hand[kLeft] : hand[kRight];
    const HandState& off = settings.swapHands ? hand[kRight] : hand[kLeft];

    XINPUT_GAMEPAD pad = {};

    // Aim and fire. RE5 readies the weapon on the left trigger and fires on
    // the right, so the gun hand's grip is aim and its trigger is fire.
    pad.bLeftTrigger = gun.squeeze > 0.5f ? 255 : 0;
    pad.bRightTrigger = static_cast<BYTE>((gun.trigger > 1.0f ? 1.0f : gun.trigger) * 255.0f);

    if (gun.primary)
        pad.wButtons |= XINPUT_GAMEPAD_A;
    if (gun.secondary)
        pad.wButtons |= XINPUT_GAMEPAD_B;
    if (off.primary)
        pad.wButtons |= XINPUT_GAMEPAD_X;
    if (off.secondary)
        pad.wButtons |= XINPUT_GAMEPAD_Y;
    if (off.squeeze > 0.5f)
        pad.wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
    if (off.menu || gun.menu)
        pad.wButtons |= XINPUT_GAMEPAD_START;

    float moveX = off.stickX, moveY = off.stickY;
    float lookX = gun.stickX, lookY = gun.stickY;
    ApplyDeadzone(&moveX, &moveY, settings.deadzone);
    ApplyDeadzone(&lookX, &lookY, settings.deadzone);

    // The d-pad. Whichever stick is standing in for it stops being a stick
    // while it does, so the game never gets a direction and a push at once.
    bool dpadFromLook = false, dpadFromMove = false;
    switch (static_cast<XrDpadMethod>(settings.dpadMethod)) {
    case XrDpadMethod::LeftTrigger:
        dpadFromLook = off.trigger > 0.5f;
        break;
    case XrDpadMethod::RightThumbrest:
        dpadFromMove = gun.thumbrest;
        break;
    case XrDpadMethod::LeftStickClick:
        dpadFromLook = off.stickClick;
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
    if (off.stickClick && static_cast<XrDpadMethod>(settings.dpadMethod) != XrDpadMethod::LeftStickClick)
        pad.wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
    if (gun.stickClick)
        pad.wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;

    pad.sThumbLX = ToAxis(moveX);
    pad.sThumbLY = ToAxis(moveY);
    pad.sThumbRX = ToAxis(lookX);
    pad.sThumbRY = ToAxis(lookY);

    // Where the gun hand points, for 3DOF aiming. Rotation only: the hand's
    // position in the room is deliberately ignored.
    const int gunHand = settings.swapHands ? kLeft : kRight;
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

    AcquireSRWLockExclusive(&g_lock);
    g_pad = pad;
    g_padMs = anyHand ? GetTickCount64() : 0;
    g_gunYawDeg = gunYawDeg;
    g_gunPitchDeg = gunPitchDeg;
    g_gunAimMs = gunAimed ? GetTickCount64() : 0;
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
