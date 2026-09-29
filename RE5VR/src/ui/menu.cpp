#include "menu.h"
#include "input_block.h"
#include "../vr/xr_input.h"
#include "../hooks/camera_rig_hook.h"
#include "../hooks/filter_patch.h"
#include "../hooks/laser_patch.h"
#include "../hooks/tremor_patch.h"
#include "../hooks/constant_probe.h"
#include "../util/sound.h"
#include "../render/stereo_test.h"
#include "../render/hud_shaders.h"
#include "../render/render_size.h"
#include "../vr/openxr_bridge.h"
#include "../vr/d3d12_addon_bridge.h"
#include "../util/build_config.h"
#include "../hooks/frame_times.h"
#include "../hooks/arm_ik.h"
#include "../hooks/ammo.h"
#include "../net/ik_sync.h"
#include "../util/log.h"
#include "../util/update_check.h"
#include "../util/version.h"

#include <imgui.h>
#include <imgui_impl_dx9.h>
#include <imgui_impl_win32.h>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {

// Defined below, next to the developer tab; used by the VR tab above it.
void DrawArmCalibration();
// Defined further down, with the input handling.
void OpenMenu(bool open);

// ---- The T-pose, as a gesture (2026-09-19) ------------------------------
// Hold both sticks and it recentres AND sizes the arms, which is the same
// gesture doing one job instead of two: you are already standing still with
// your arms out, so there is nothing else to ask for. Five seconds is enough to
// get into the pose after letting go of the sticks, and the countdown is drawn
// over the game so nobody has to guess when it fires.
ULONGLONG g_calFiresAtMs = 0;

void StartTPoseCalibration()
{
    g_calFiresAtMs = GetTickCount64() + 5000;
    Log_Printf("Menu: T-pose calibration in five seconds - arms out to the sides, palms down");
}

// A figure in the pose being asked for, drawn rather than shipped as an image:
// a stick person is legible at any resolution, costs nothing, and cannot go
// missing from a zip. The palms are the part that matters, so they are drawn as
// flats with arrows under them - "arms out" is obvious, "palms down" is not.
void DrawTPoseOverlay(float secondsLeft)
{
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 screen = io.DisplaySize;
    if (screen.x < 1.0f || screen.y < 1.0f)
        return;
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(screen);
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin("##tposecal", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav
            | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus
            | ImGuiWindowFlags_NoFocusOnAppearing);
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const float s = screen.y * 0.26f;
    const ImVec2 c(screen.x * 0.5f, screen.y * 0.44f);
    const ImU32 ink = IM_COL32(255, 255, 255, 235);
    const ImU32 accent = IM_COL32(120, 215, 255, 245);
    const ImU32 shade = IM_COL32(0, 0, 0, 150);
    const float t = s * 0.045f;

    // The captions are measured before the panel is drawn, so the panel can be
    // sized to hold them (2026-09-19). It used to end at a fixed 1.15 of the
    // figure's height, which was fine for one line and left the second hanging
    // off the bottom - and the font scales with the UI, so no fixed number was
    // ever going to be right for both.
    const char* line1 = "Arms straight out - palms down";
    const char* line2 = "Don't move your head - recentering";
    const ImVec2 t1 = ImGui::CalcTextSize(line1);
    const ImVec2 t2 = ImGui::CalcTextSize(line2);
    const float line1Y = c.y + s * 0.84f;
    const float line2Y = line1Y + t1.y * 1.25f;
    const float panelBottom = line2Y + t2.y + s * 0.12f;
    // Wide enough for the arms or the text, whichever needs more.
    const float halfWide
        = (std::max)(s * 0.95f, (std::max)(t1.x, t2.x) * 0.5f + s * 0.10f);

    // A panel behind it, so it reads against a bright jungle as well as a dark
    // corridor.
    dl->AddRectFilled(ImVec2(c.x - halfWide, c.y - s * 1.08f), ImVec2(c.x + halfWide, panelBottom), shade,
        s * 0.06f);

    const float armY = c.y - s * 0.18f;
    const float reach = s * 0.66f;
    // Head, spine, legs.
    dl->AddCircle(ImVec2(c.x, c.y - s * 0.46f), s * 0.12f, ink, 24, t);
    dl->AddLine(ImVec2(c.x, c.y - s * 0.34f), ImVec2(c.x, c.y + s * 0.26f), ink, t);
    dl->AddLine(ImVec2(c.x, c.y + s * 0.26f), ImVec2(c.x - s * 0.22f, c.y + s * 0.70f), ink, t);
    dl->AddLine(ImVec2(c.x, c.y + s * 0.26f), ImVec2(c.x + s * 0.22f, c.y + s * 0.70f), ink, t);
    // Arms, straight out to the sides.
    dl->AddLine(ImVec2(c.x - reach, armY), ImVec2(c.x + reach, armY), ink, t);

    // The palms: a flat, and an arrow under it pointing down.
    for (int side = -1; side <= 1; side += 2) {
        const float hx = c.x + reach * static_cast<float>(side);
        dl->AddLine(ImVec2(hx - s * 0.10f, armY + s * 0.05f), ImVec2(hx + s * 0.10f, armY + s * 0.05f), accent,
            t * 1.4f);
        const float ay = armY + s * 0.12f;
        dl->AddLine(ImVec2(hx, ay), ImVec2(hx, ay + s * 0.16f), accent, t);
        dl->AddLine(ImVec2(hx - s * 0.06f, ay + s * 0.09f), ImVec2(hx, ay + s * 0.17f), accent, t);
        dl->AddLine(ImVec2(hx + s * 0.06f, ay + s * 0.09f), ImVec2(hx, ay + s * 0.17f), accent, t);
    }

    // Above the figure's head, not across it (2026-09-19). The text is drawn
    // from its top-left, so at this size it reached down to about the crown.
    const float big = s * 0.30f;
    char num[16];
    _snprintf_s(num, sizeof(num), _TRUNCATE, "%d", static_cast<int>(secondsLeft) + 1);
    const ImVec2 n1 = ImGui::CalcTextSize(num);
    dl->AddText(nullptr, big, ImVec2(c.x - n1.x * big / ImGui::GetFontSize() * 0.5f, c.y - s * 1.02f), accent,
        num);
    dl->AddText(ImVec2(c.x - t1.x * 0.5f, line1Y), ink, line1);
    // The recentre goes with it, and holding still matters just as much as the
    // pose does - the view's idea of forward is taken from wherever your head is
    // at the moment it fires.
    dl->AddText(ImVec2(c.x - t2.x * 0.5f, line2Y), IM_COL32(255, 255, 255, 170), line2);
    ImGui::End();
}


constexpr const char* kTitle = "STB - TrueFP/VR Mod v" RE5VR_VERSION;

// ---- Settings that belong to the menu itself ---------------------------
struct MenuPrefs {
    float uiScale = 1.0f;
    // Convergence distance of the VR menu. NOT a player option: the user found
    // (2026-09-13) that moving it reads as the two eyes' copies spreading
    // apart, not as the menu moving away - size is what sets felt distance.
    float vrMenuDistance = 1.5f;
    bool padChord = true;        // a quick click of both sticks opens the menu
    bool startupHint = true;
    bool autoStartVr = false;
    bool desktopRightEye = true; // VR: which eye the game window shows (never both side by side)
    bool checkForUpdates = true; // ask Nexus for the latest version at startup
};

struct AllSettings {
    IkSyncSettings ikSync;
    CameraRigSettings cam;
    StereoSettings stereo;
    VRBridgeSettings vr;
    RenderSizeSettings res;
    XrInputSettings xrInput;
    bool filterRemoved = true;
    bool laserSight = true; // force RE5's laser sight on, even without a pad
    // On by default: the shake exists to make a gamepad crosshair drift, and
    // in a headset it only fights your hand and the arm IK. See
    // hooks/tremor_patch.cpp for the four sites and where they came from.
    bool steadyHands = true;
    MenuPrefs menu;
};

AllSettings g_defaults;
MenuPrefs g_prefs;

AllSettings CaptureSettings()
{
    AllSettings s;
    s.cam = CameraRigHook_GetSettings();
    s.stereo = StereoTest_GetSettings();
    s.vr = VRBridge_GetSettings();
    s.res = RenderSize_GetSettings();
    s.xrInput = XrInput_GetSettings();
    s.ikSync = IkSync_GetSettings();
    s.filterRemoved = FilterPatch_IsFilterRemoved();
    s.laserSight = LaserPatch_IsForcedOn();
    s.steadyHands = TremorPatch_IsOn();
    s.menu = g_prefs;
    return s;
}

void ApplySettings(const AllSettings& in)
{
    CameraRigHook_ApplySettings(in.cam);
    StereoSettings st = in.stereo;
    // Stereo on/off belongs to VR mode, which flips it from its own thread.
    // Never let a menu copy taken a moment earlier switch it back.
    st.stereoEnabled = StereoTest_IsEnabled();
    StereoTest_ApplySettings(st);
    VRBridge_ApplySettings(in.vr);
    RenderSize_ApplySettings(in.res);
    XrInput_SetSettings(in.xrInput);
    ArmIk_SetReachAllowance(in.xrInput.armIkReachAllowanceM);
    IkSync_SetSettings(in.ikSync);
    IkSync_Update();
    if (FilterPatch_IsAvailable())
        FilterPatch_SetFilterRemoved(in.filterRemoved);
        Sound_SetVolume(in.xrInput.reloadSounds ? in.xrInput.reloadSoundVolume : 0.0f);
        LaserPatch_SetForcedOn(in.laserSight);
    if (TremorPatch_IsAvailable())
        TremorPatch_SetOn(in.steadyHands);
    g_prefs = in.menu;
}

// Pointing the gun with the controller is one checkbox on the VR tab now, and
// its key is saved like any other setting (2026-09-17). It stopped being a
// servo that needed tuning per person when the pitch went straight into the
// game's aim and the yaw started turning the body: both absolute, nothing left
// to chase, nothing to explain. The rest of the keys below are the workings -
// the ceiling, the filter, the write method, the finders - and they stay in
// developer builds, where the Developer tab can still reach them.
#define RE5VR_SETTINGS_AIM(X) X("VR", "MotionPointToAim", xrInput.pointToAim)

// SAVED IN EVERY BUILD (2026-09-29, from a release build quietly deleting
// two dozen lines out of a working re5vr.ini).
//
// This list used to vanish when RE5VR_DIAGNOSTICS was off, and the file is
// written FRESH from RE5VR_SETTINGS every time - so the first save from a
// release build threw away every one of these settings permanently. Arm
// reach, shoulder travel, wrist pivot, the aim rate and ceiling: all of it,
// gone from the ini of anyone who ran a release build after a developer one.
//
// The original reasoning was sound but aimed at the wrong list: a dev switch
// like direct submit must not be left in an ini a release build then reads.
// Those switches are not in here. What IS in here is tuning that the release
// build uses whether or not it shows a slider for it.
//
// The UI stays developer-only. The data does not. A setting nobody can see
// is still a setting the mod reads.
#define RE5VR_SETTINGS_AIM_DEV(X)                                    \
    X("VR", "MotionSwapHands", xrInput.swapHands)                    \
    X("VR", "MotionAimTrim", xrInput.aimPitchTrimDeg)                \
    X("VR", "MotionAimFindWriter", xrInput.findAimWriter)            \
    X("VR", "MotionAimWriteField", xrInput.aimWriteField)            \
    X("VR", "AimSpeedMultiplier", xrInput.aimSpeedMult)              \
    X("VR", "AimServoMs", xrInput.aimServoMs)                        \
    X("VR", "AimStickDeadzone", xrInput.aimStickDeadzone)            \
    X("VR", "AimOverlay", xrInput.aimOverlay)                        \
    X("VR", "AimRateDrive", xrInput.aimRateDrive)                    \
    X("VR", "AimMaxRateDeg", xrInput.aimMaxRateDeg)                  \
    X("VR", "AimSteadiness", xrInput.aimSteadiness)                  \
    X("VR", "FindBodyFacing", xrInput.findBodyFacing)                \
    X("VR", "AbsoluteYaw", xrInput.absoluteYaw)                      \
    X("VR", "FindCullPlanes", xrInput.findCullPlanes)                \
    X("VR", "WideCullAlways", xrInput.wideCullAlways)                \
    X("VR", "FindArms", xrInput.findArms)                          \
    X("VR", "ArmIkWeight", xrInput.armIkWeight)                    \
    X("VR", "ArmIkScale", xrInput.armIkScale)                      \
    X("VR", "ArmIkShoulder", xrInput.armIkShoulder)                \
    X("VR", "ArmIkWristPivot", xrInput.armIkWristPivotM)           \
    X("VR", "ArmIkTest", xrInput.armIkTest)                        \
    X("VR", "HolsterMeasure", xrInput.holsterMeasure)              \
    X("VR", "ArmIkLateWrite", xrInput.armIkLateWrite)              \
    X("VR", "FindArmWriter", xrInput.findArmWriter)

// One list drives load, save and "did anything change". Developer switches
// are deliberately absent: a diagnostics build must never leave, say, direct
// submit turned off in an ini that a release build then reads.
#define RE5VR_SETTINGS(X)                                            \
    X("Camera", "FirstPerson", cam.firstPerson)                      \
    X("Camera", "ShowHeadDuringActions", cam.showHeadDuringActions)  \
    X("Camera", "RefuseMeleeCamera", cam.holdViewInMelee)            \
    X("Camera", "PartnerLocateNoCamera", cam.partnerNoCamera)        \
    X("Camera", "ScopeThirdPerson", cam.scopeThirdPerson)            \
    X("Camera", "FlatFov", cam.flatFovDeg)                           \
    X("Camera", "FlatEyeHeight", cam.flatEyeUp)                      \
    X("Camera", "FlatEyeForward", cam.flatEyeAhead)                  \
    X("Graphics", "RemoveColourFilter", filterRemoved)               \
    X("Graphics", "ForceLaserSight", laserSight)                     \
    X("Graphics", "SteadyHands", steadyHands)                        \
    X("VR", "StartInVR", menu.autoStartVr)                           \
    X("VR", "DesktopRightEye", menu.desktopRightEye)                 \
    X("VR", "FullResolutionPerEye", res.fullResPerEye)               \
    X("VR", "HeadTurnsCamera", cam.headFollow)                       \
    X("VR", "Stabilise", cam.vrStabilise)                            \
    X("VR", "MatchCullingToHeadset", cam.vrMatchCullFov)             \
    X("VR", "EyeHeight", cam.vrEyeUp)                                \
    X("VR", "EyeForward", cam.vrEyeAhead)                            \
    X("VR", "EyeSeparation", stereo.halfSeparation)                  \
    X("VR", "FovWiden", stereo.fovWiden)                             \
    X("VR", "MonoPostProcess", stereo.monoSmallTargets)              \
\
    X("VR", "HudScale", stereo.hudScale)                             \
    X("VR", "Theatre", stereo.theatre)                               \
    X("VR", "TheatreFollowsHead", stereo.theatreFollowsHead)         \
    X("VR", "TheatreDistance", stereo.theatreDistanceMeters)         \
    X("VR", "TheatreScale", stereo.theatreScale)                     \
    X("VR", "LeanAndPeek", stereo.headPositionTracking)              \
    X("VR", "LeanScale", stereo.leanScale)                           \
    X("VR", "NearClip", stereo.nearPlaneUnits)                       \
    X("Camera", "HeadLock", cam.headLock)                            \
    X("Camera", "HeadLockReach", cam.headLockReach)                  \
    X("Camera", "HeadLockDirection", cam.headLockDirection)          \
    X("Camera", "ViewTurnLimit", cam.viewTurnLimitDeg)               \
    X("VR", "MotionControllers", xrInput.enabled)                    \
    X("VR", "MotionDpadMethod", xrInput.dpadMethod)                  \
    X("VR", "MotionDeadzone", xrInput.deadzone)                      \
    X("VR", "SixDofArms", xrInput.armIk)                             \
    X("VR", "SixDofDrivesPose", xrInput.armIkWriteLocal)             \
    X("VR", "SixDofWrist", xrInput.armIkWrist)                       \
    X("VR", "HandsStopAtPeople", xrInput.armIkTouch)                 \
    X("VR", "HowSolid", xrInput.armIkTouchRadius)                    \
    X("VR", "SwingToKnife", xrInput.meleeSwing)                      \
    X("VR", "SwingStrength", xrInput.meleeSwingSpeed)                \
    X("VR", "ReachAllowance", xrInput.armIkReachAllowanceM)          \
    X("VR", "CharacterLeftHanded", xrInput.characterLeftHanded)      \
    X("VR", "StickPitch", xrInput.stickPitch)                        \
    X("VR", "HeadTurn", xrInput.headTurn)                            \
    X("VR", "HeadTurnDead", xrInput.headTurnDeadDeg)                 \
    X("VR", "HeadTurnFull", xrInput.headTurnFullDeg)                 \
    X("VR", "SpineLean", xrInput.spineLean)                          \
    X("VR", "RoomStep", xrInput.roomStep)                            \
    X("VR", "RoomStepDead", xrInput.roomStepDeadM)                   \
    X("VR", "RoomStepFull", xrInput.roomStepFullM)                   \
    X("VR", "SpineLeanAmount", xrInput.spineLeanAmount)              \
    X("VR", "SpineLeanMaxDeg", xrInput.spineLeanMaxDeg)              \
    X("VR", "Holsters", xrInput.holsters)                            \
    X("VR", "TwoHanded", xrInput.twoHanded)                          \
    X("VR", "TwoHandRaise", xrInput.twoHandRaiseCm)                  \
    X("VR", "TwoHandSteer", xrInput.twoHandSteer)                    \
    X("VR", "TwoHandRoll", xrInput.twoHandRoll)                      \
    X("VR", "GunScale", xrInput.gunScale)                            \
    X("VR", "GunLock", xrInput.gunLock)                              \
    X("VR", "GunKickOneHanded", xrInput.kickOneHanded)               \
    X("VR", "GunKickTwoHanded", xrInput.kickTwoHanded)               \
    X("VR", "MenuLaser", xrInput.menuLaser)                          \
    X("VR", "WristInventory", xrInput.wristInventory)                \
    X("VR", "ManualReload", xrInput.manualReload)                    \
    X("VR", "ReloadKeepsRounds", xrInput.reloadKeepsRounds)          \
    X("VR", "MagazineBone", xrInput.magazineBone)                    \
    X("VR", "MagazineWithBone", xrInput.magazineWithBone)            \
    X("VR", "BodyVisible", xrInput.bodyVisible)                      \
    X("VR", "BodyCutBack", xrInput.bodyCutBack)                      \
    X("VR", "CameraFollowsEye", xrInput.cameraFollowsEye)            \
    X("VR", "CutStepsArmsLeft", xrInput.cutStepsUp[0][0])            \
    X("VR", "CutStepsArmsRight", xrInput.cutStepsUp[0][1])           \
    X("VR", "CutStepsHandsLeft", xrInput.cutStepsUp[1][0])           \
    X("VR", "CutStepsHandsRight", xrInput.cutStepsUp[1][1])          \
    X("VR", "ReloadSounds", xrInput.reloadSounds)                    \
    X("VR", "ReloadSoundVolume", xrInput.reloadSoundVolume)          \
    X("CoOp", "SendMyArms", ikSync.enabled)                            \
    X("CoOp", "PartnerAddress", ikSync.partnerIp)                    \
    X("CoOp", "Port", ikSync.port)                                   \
    X("CoOp", "Loopback", ikSync.loopback)                           \
    RE5VR_SETTINGS_AIM(X)                                            \
    RE5VR_SETTINGS_AIM_DEV(X)                                        \
    X("VR", "HeadPredictionMs", vr.headPredictMs)                    \
    X("VR", "HeadRotationGain", vr.headRotationGain)                 \
    X("VR", "HeadSteadiness", cam.vrHeadSteady)                      \
    X("VR", "RunLead", cam.vrRunLead)                                \
    X("VR", "ViewSteady", cam.vrViewSteady)                          \
    X("VR", "EyeOnNeck", cam.vrEyeOnNeck)                            \
    X("VR", "EyeAboveNeck", cam.vrEyeAboveNeck)                      \
    X("Menu", "Scale", menu.uiScale)                                 \
\
    X("Menu", "OpenWithBothSticks", menu.padChord)                   \
    X("Menu", "StartupHint", menu.startupHint)                       \
    X("Menu", "CheckForUpdates", menu.checkForUpdates)

char g_iniPath[MAX_PATH] = "";

void ReadValue(const char* section, const char* key, bool& v)
{
    char buf[32];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), g_iniPath);
    if (buf[0])
        v = std::atoi(buf) != 0;
}
void ReadValue(const char* section, const char* key, int& v)
{
    char buf[32];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), g_iniPath);
    if (buf[0])
        v = std::atoi(buf);
}
void ReadValue(const char* section, const char* key, float& v)
{
    char buf[32];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), g_iniPath);
    if (buf[0])
        v = static_cast<float>(std::atof(buf));
}
// A text value, for the partner's address. Fixed size on purpose: it goes
// into a settings struct that is copied about freely, and a std::string in
// there would be the only allocation in the whole set.
template <size_t N>
void ReadValue(const char* section, const char* key, char (&v)[N])
{
    char buf[N];
    GetPrivateProfileStringA(section, key, "", buf, static_cast<DWORD>(sizeof(buf)), g_iniPath);
    strcpy_s(v, buf);
}
template <size_t N>
std::string FormatValue(const char (&v)[N])
{
    return std::string(v);
}
std::string FormatValue(bool v)
{
    return v ? "1" : "0";
}
std::string FormatValue(int v)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", v);
    return buf;
}
std::string FormatValue(float v)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.3f", v);
    return buf;
}

std::string Serialize(const AllSettings& s)
{
    std::string out;
#define X(section, key, member) out += std::string(section) + "." + key + "=" + FormatValue(s.member) + "\n";
    RE5VR_SETTINGS(X)
#undef X
    return out;
}

void LoadSettings(AllSettings& s)
{
#define X(section, key, member) ReadValue(section, key, s.member);
    RE5VR_SETTINGS(X)
#undef X
}

// The whole file is written fresh from RE5VR_SETTINGS, so a setting that is
// removed from the list also disappears from re5vr.ini. Writing key by key
// only ever added and updated: HudDistance, VRDistance, LaserStaysInWorld and
// DesktopSingleView all outlived their options (2026-09-13).
void SaveSettings(const AllSettings& s)
{
    std::string text = "; RE5VR settings - written by the in-game menu (Insert). Safe to delete.\r\n";
    std::string section;
#define X(sec, key, member)                                   \
    if (section != sec) {                                     \
        section = sec;                                        \
        text += "\r\n[" + section + "]\r\n";                  \
    }                                                         \
    text += std::string(key) + "=" + FormatValue(s.member) + "\r\n";
    RE5VR_SETTINGS(X)
#undef X
    // The list keeps each section's keys together, so one pass is enough.
    // Write beside the file and swap it in, so a crash mid-write can't leave
    // half a settings file behind.
    const std::string tmp = std::string(g_iniPath) + ".tmp";
    FILE* f = nullptr;
    if (fopen_s(&f, tmp.c_str(), "wb") != 0 || !f) {
        Log_Printf("Menu: could not write %s", tmp.c_str());
        return;
    }
    const bool ok = fwrite(text.data(), 1, text.size(), f) == text.size();
    fclose(f);
    if (!ok || !MoveFileExA(tmp.c_str(), g_iniPath, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        Log_Printf("Menu: could not save settings to %s", g_iniPath);
        DeleteFileA(tmp.c_str());
        return;
    }
    Log_Printf("Menu: settings saved to %s", g_iniPath);
}

// ---- State --------------------------------------------------------------
IDirect3DDevice9* g_device = nullptr;
HWND g_hwnd = nullptr;
WNDPROC g_gameWndProc = nullptr;
bool g_imguiReady = false;
bool g_open = false;
std::atomic<bool> g_drawing{ false };

std::string g_savedSnapshot;
ULONGLONG g_lastChangeMs = 0;
bool g_dirty = false;

ULONGLONG g_firstPresentMs = 0;
bool g_autoVrDone = false;
float g_fontPxBuilt = 0.0f;
float g_frameMsAvg = 0.0f;
LARGE_INTEGER g_qpcFreq = {}, g_lastPresentQpc = {};

float g_cursorX = 0.0f, g_cursorY = 0.0f; // in UI units
// Where the controller itself appears, in the same units, for the beam.
float g_laserFromX = 0.0f, g_laserFromY = 0.0f;
bool g_haveLaserFrom = false;
float g_sentX = -1e9f, g_sentY = -1e9f;     // last position handed to ImGui
float g_lastRealX = -1e9f, g_lastRealY = -1e9f;
int g_selectTab = -1;                     // set by the bumpers, applied next frame
int g_currentTab = 0;

// Window messages arrive on the game's window thread; ImGui is fed on the
// render thread. Copy them across rather than touch ImGui from two threads.
struct QueuedMsg {
    UINT msg;
    WPARAM w;
    LPARAM l;
};
SRWLOCK g_msgLock = SRWLOCK_INIT;
QueuedMsg g_msgs[256];
int g_msgCount = 0;

bool IsKeyboardMsg(UINT m)
{
    return m == WM_KEYDOWN || m == WM_KEYUP || m == WM_CHAR;
}
bool IsMouseMsg(UINT m)
{
    return (m >= WM_MOUSEFIRST && m <= WM_MOUSELAST) || m == WM_MOUSEHOVER || m == WM_MOUSELEAVE;
}

void QueueMessage(UINT msg, WPARAM w, LPARAM l)
{
    if (msg == WM_MOUSEMOVE || !(IsKeyboardMsg(msg) || IsMouseMsg(msg)))
        return;
    AcquireSRWLockExclusive(&g_msgLock);
    if (g_msgCount < static_cast<int>(_countof(g_msgs)))
        g_msgs[g_msgCount++] = { msg, w, l };
    ReleaseSRWLockExclusive(&g_msgLock);
}

// Most input is diverted earlier, where the game pulls it off its queue (see
// input_block.cpp); this catches whatever is sent straight to the window.
LRESULT CALLBACK MenuWndProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l)
{
    // The game is closing: VR has to be shut down while the process is still
    // whole, or d3d11.dll crashes on the way out.
    if (msg == WM_CLOSE || msg == WM_DESTROY)
        VRBridge_Shutdown(msg == WM_CLOSE ? "game window closing" : "game window destroyed");
    // One pointer, not two: over the game's own menus Windows shows its
    // cursor too, sitting on top of ours. Windows asks for the cursor shape
    // with WM_SETCURSOR on every mouse move, so answering "none" while the
    // menu is open hides it; closing hands the question back to the game.
    if (g_open && msg == WM_SETCURSOR && LOWORD(l) == HTCLIENT) {
        SetCursor(nullptr);
        return TRUE;
    }
    if (g_open && (IsKeyboardMsg(msg) || IsMouseMsg(msg))) {
        QueueMessage(msg, w, l);
        return 0; // the menu has them; the game does not
    }
    return CallWindowProcA(g_gameWndProc, hwnd, msg, w, l);
}

bool GameIsForeground()
{
    DWORD pid = 0;
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    return pid == GetCurrentProcessId();
}

bool ExeIsLargeAddressAware()
{
    const auto* base = reinterpret_cast<const BYTE*>(GetModuleHandleA(nullptr));
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    return (nt->FileHeader.Characteristics & IMAGE_FILE_LARGE_ADDRESS_AWARE) != 0;
}

// ---- Style and fonts ----------------------------------------------------
void ApplyStyle(float scale)
{
    ImGuiStyle& st = ImGui::GetStyle();
    st = ImGuiStyle();
    ImGui::StyleColorsDark(&st);
    st.WindowRounding = 6.0f;
    st.ChildRounding = 4.0f;
    st.FrameRounding = 4.0f;
    st.GrabRounding = 4.0f;
    st.TabRounding = 4.0f;
    st.PopupRounding = 4.0f;
    st.WindowPadding = ImVec2(12, 10);
    st.FramePadding = ImVec2(8, 4);
    st.ItemSpacing = ImVec2(8, 6);
    st.WindowTitleAlign = ImVec2(0.5f, 0.5f);

    const ImVec4 accent(0.64f, 0.16f, 0.12f, 1.00f);
    const ImVec4 accentHi(0.78f, 0.24f, 0.18f, 1.00f);
    const ImVec4 accentLo(0.40f, 0.11f, 0.09f, 1.00f);
    ImVec4* c = st.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.06f, 0.07f, 0.95f);
    c[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.05f, 0.05f, 1.00f);
    c[ImGuiCol_TitleBgActive] = accentLo;
    c[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.15f, 0.15f, 1.00f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.26f, 0.20f, 0.19f, 1.00f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.32f, 0.22f, 0.20f, 1.00f);
    c[ImGuiCol_CheckMark] = accentHi;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = accentHi;
    c[ImGuiCol_Button] = accentLo;
    c[ImGuiCol_ButtonHovered] = accent;
    c[ImGuiCol_ButtonActive] = accentHi;
    c[ImGuiCol_Header] = accentLo;
    c[ImGuiCol_HeaderHovered] = accent;
    c[ImGuiCol_HeaderActive] = accentHi;
    c[ImGuiCol_Tab] = ImVec4(0.18f, 0.10f, 0.09f, 1.00f);
    c[ImGuiCol_TabHovered] = accent;
    c[ImGuiCol_TabSelected] = accentLo;
    c[ImGuiCol_TabSelectedOverline] = accentHi;
    c[ImGuiCol_SeparatorHovered] = accent;
    c[ImGuiCol_SeparatorActive] = accentHi;
    c[ImGuiCol_ResizeGrip] = accentLo;
    c[ImGuiCol_ResizeGripHovered] = accent;
    c[ImGuiCol_ResizeGripActive] = accentHi;
    c[ImGuiCol_NavCursor] = accentHi;
    st.ScaleAllSizes(scale);
}

// Rebuilds the font atlas at a new pixel size. A real outline font rebuilt
// at the size needed stays sharp; scaling the built-in bitmap font would blur.
void EnsureFont(float px)
{
    px = (std::max)(12.0f, (std::min)(px, 72.0f));
    // Hysteresis: a size hovering on a .5 boundary must not rebuild every frame.
    if (g_fontPxBuilt > 0.0f && std::fabs(px - g_fontPxBuilt) < 0.75f)
        return;
    px = std::round(px);
    g_fontPxBuilt = px;
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplDX9_InvalidateDeviceObjects(); // font texture rebuilt at the next NewFrame
    io.Fonts->Clear();
    char path[MAX_PATH];
    const UINT n = GetWindowsDirectoryA(path, MAX_PATH);
    ImFont* font = nullptr;
    if (n && n < MAX_PATH - 32) {
        strcat_s(path, "\\Fonts\\segoeui.ttf");
        if (GetFileAttributesA(path) != INVALID_FILE_ATTRIBUTES)
            font = io.Fonts->AddFontFromFileTTF(path, px);
    }
    if (!font) {
        ImFontConfig cfg;
        cfg.SizePixels = px;
        io.Fonts->AddFontDefault(&cfg);
    }
    ApplyStyle(px / 18.0f);
}

// ---- Layout: flat, or once per eye --------------------------------------
struct Layout {
    bool stereo = false;
    float bbW = 0, bbH = 0;
    float unitsW = 0, unitsH = 0; // ImGui's display size
    float sx = 1.0f;              // UI unit -> backbuffer pixels, horizontally
    float ox[2] = {}, oy[2] = {}; // where the UI's origin lands, per eye
    float clipX0[2] = {}, clipX1[2] = {};
    float fontPx = 18.0f;
};

Layout ComputeLayout(UINT bbW, UINT bbH)
{
    Layout L;
    L.bbW = static_cast<float>(bbW);
    L.bbH = static_cast<float>(bbH);
    L.stereo = StereoTest_IsEnabled();
    if (!L.stereo) {
        L.unitsW = L.bbW;
        L.unitsH = L.bbH;
        L.clipX1[0] = L.bbW;
        // 20 px at 1080p, but never below 16 - a 720p window is still read
        // from a normal desk distance.
        L.fontPx = (std::max)(16.0f, 20.0f * (L.bbH / 1080.0f)) * g_prefs.uiScale;
        return L;
    }

    StereoPanelEye eyes[2];
    StereoTest_GetPanelPlacement(g_prefs.vrMenuDistance, bbW, bbH, eyes);
    // One UI unit is one vertical pixel. An eye's half is narrower in pixels
    // than the angle it covers, so horizontal units are squeezed by sx to
    // keep text and boxes square in the headset.
    const float pyt = eyes[0].pxPerTanY;
    L.sx = eyes[0].pxPerTanX / pyt;
    // About 50 x 53 degrees of view - big enough to read, small enough that
    // the edges stay out of the blurry part of the lens.
    L.unitsH = (std::min)(1.0f * pyt, L.bbH * 0.95f);
    L.unitsW = (std::min)(0.95f * pyt, eyes[0].halfWidth * 0.95f / L.sx);
    const float pw = L.unitsW * L.sx;
    for (int e = 0; e < 2; ++e) {
        const StereoPanelEye& eye = eyes[e];
        float x = eye.centreX - pw * 0.5f;
        float y = eye.centreY - L.unitsH * 0.5f;
        x = (std::max)(eye.halfX0, (std::min)(x, eye.halfX0 + eye.halfWidth - pw));
        y = (std::max)(0.0f, (std::min)(y, L.bbH - L.unitsH));
        L.ox[e] = std::round(x);
        L.oy[e] = std::round(y);
        L.clipX0[e] = eye.halfX0;
        L.clipX1[e] = eye.halfX0 + eye.halfWidth;
    }
    L.fontPx = 0.042f * pyt * g_prefs.uiScale;
    return L;
}

// ---- Input into ImGui ---------------------------------------------------
void FeedPad(ImGuiIO& io, const XINPUT_STATE* pad)
{
    const WORD b = pad ? pad->Gamepad.wButtons : 0;
    const auto key = [&](ImGuiKey k, WORD mask) { io.AddKeyEvent(k, (b & mask) != 0); };
    key(ImGuiKey_GamepadStart, XINPUT_GAMEPAD_START);
    key(ImGuiKey_GamepadBack, XINPUT_GAMEPAD_BACK);
    key(ImGuiKey_GamepadFaceLeft, XINPUT_GAMEPAD_X);
    key(ImGuiKey_GamepadFaceRight, XINPUT_GAMEPAD_B);
    key(ImGuiKey_GamepadFaceUp, XINPUT_GAMEPAD_Y);
    key(ImGuiKey_GamepadFaceDown, XINPUT_GAMEPAD_A);
    key(ImGuiKey_GamepadDpadLeft, XINPUT_GAMEPAD_DPAD_LEFT);
    key(ImGuiKey_GamepadDpadRight, XINPUT_GAMEPAD_DPAD_RIGHT);
    key(ImGuiKey_GamepadDpadUp, XINPUT_GAMEPAD_DPAD_UP);
    key(ImGuiKey_GamepadDpadDown, XINPUT_GAMEPAD_DPAD_DOWN);
    key(ImGuiKey_GamepadL1, XINPUT_GAMEPAD_LEFT_SHOULDER);
    key(ImGuiKey_GamepadR1, XINPUT_GAMEPAD_RIGHT_SHOULDER);
    key(ImGuiKey_GamepadL3, XINPUT_GAMEPAD_LEFT_THUMB);
    key(ImGuiKey_GamepadR3, XINPUT_GAMEPAD_RIGHT_THUMB);
    const float lt = pad ? pad->Gamepad.bLeftTrigger / 255.0f : 0.0f;
    const float rt = pad ? pad->Gamepad.bRightTrigger / 255.0f : 0.0f;
    io.AddKeyAnalogEvent(ImGuiKey_GamepadL2, lt > 0.3f, lt);
    io.AddKeyAnalogEvent(ImGuiKey_GamepadR2, rt > 0.3f, rt);
    const auto stick = [&](ImGuiKey k, short v, int sign) {
        constexpr float dead = 7849.0f;
        float f = (sign * static_cast<float>(v) - dead) / (32767.0f - dead);
        f = (std::max)(0.0f, (std::min)(f, 1.0f));
        io.AddKeyAnalogEvent(k, f > 0.1f, f);
    };
    const short lx = pad ? pad->Gamepad.sThumbLX : 0, ly = pad ? pad->Gamepad.sThumbLY : 0;
    stick(ImGuiKey_GamepadLStickLeft, lx, -1);
    stick(ImGuiKey_GamepadLStickRight, lx, +1);
    stick(ImGuiKey_GamepadLStickUp, ly, +1);
    stick(ImGuiKey_GamepadLStickDown, ly, -1);
}

// ---- Desktop view (2026-09-13) ---------------------------------------------
// In VR the game window showed both eyes side by side, which is useless to
// stream or record. The addon can show a region of the frame instead (see
// addon_main.cpp); this picks the region every frame: the left eye, centred
// on where that eye looks straight ahead (the per-eye projections are off
// centre), cropped to the window's shape. An eye's half is narrower in
// pixels than the angle it covers, so the crop is corrected by the eye's own
// pixels-per-tangent ratio to come out undistorted. The chosen rect also
// drives the mouse mapping below: the window now shows that region, not the
// whole frame.
bool g_dvActive = false;
int g_dvEye = 1; // 0 left, 1 right
float g_dvX = 0, g_dvY = 0, g_dvW = 0, g_dvH = 0;

void UpdateDesktopView(IDirect3DDevice9* device)
{
    const bool want = StereoTest_IsEnabled() && VRBridge_IsAvailable();
    if (!want) {
        if (g_dvActive)
            D3D12AddonBridge_SetDesktopView(false, 0, 0, 0, 0);
        g_dvActive = false;
        return;
    }
    IDirect3DSurface9* bb = nullptr;
    D3DSURFACE_DESC d = {};
    if (FAILED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;
    bb->GetDesc(&d);
    bb->Release();
    if (d.Width < 64 || d.Height < 64)
        return;

    StereoPanelEye eyes[2];
    StereoTest_GetPanelPlacement(1000.0f, d.Width, d.Height, eyes); // far away = straight ahead
    g_dvEye = g_prefs.desktopRightEye ? 1 : 0;
    const StereoPanelEye& e = eyes[g_dvEye];
    RECT rc;
    float aspect = 16.0f / 9.0f;
    if (g_hwnd && GetClientRect(g_hwnd, &rc) && rc.right > 0 && rc.bottom > 0)
        aspect = static_cast<float>(rc.right) / static_cast<float>(rc.bottom);
    const float sx = e.pxPerTanX / e.pxPerTanY;
    float h = static_cast<float>(d.Height);
    float w = aspect * h * sx;
    if (w > e.halfWidth) {
        w = e.halfWidth;
        h = w / (aspect * sx);
    }
    float x = e.centreX - w * 0.5f, y = e.centreY - h * 0.5f;
    x = (std::max)(e.halfX0, (std::min)(x, e.halfX0 + e.halfWidth - w));
    y = (std::max)(0.0f, (std::min)(y, static_cast<float>(d.Height) - h));
    g_dvX = std::round(x);
    g_dvY = std::round(y);
    g_dvW = std::round(w);
    g_dvH = std::round(h);
    D3D12AddonBridge_SetDesktopView(true, static_cast<int>(g_dvX), static_cast<int>(g_dvY), static_cast<int>(g_dvW),
        static_cast<int>(g_dvH));
    static int s_loggedEye = -1;
    if (!g_dvActive || s_loggedEye != g_dvEye) {
        s_loggedEye = g_dvEye;
        Log_Printf("Menu: desktop shows the %s eye, frame region %.0f,%.0f %.0fx%.0f", g_dvEye ? "right" : "left", g_dvX,
            g_dvY, g_dvW, g_dvH);
    }
    g_dvActive = true;
}

void FeedMouseAndKeys(ImGuiIO& io, const Layout& L)
{
    InputBlockMouse m;
    InputBlock_TakeMouse(m);

    QueuedMsg msgs[_countof(g_msgs)];
    AcquireSRWLockExclusive(&g_msgLock);
    const int count = g_msgCount;
    for (int i = 0; i < count; ++i)
        msgs[i] = g_msgs[i];
    g_msgCount = 0;
    ReleaseSRWLockExclusive(&g_msgLock);
    for (int i = 0; i < count; ++i) {
        // With a DirectInput mouse the buttons come from there instead.
        if (m.fromDirectInput && IsMouseMsg(msgs[i].msg) && msgs[i].msg != WM_MOUSEWHEEL)
            continue;
        ImGui_ImplWin32_WndProcHandler(g_hwnd, msgs[i].msg, msgs[i].w, msgs[i].l);
    }
    // Modifier state from the system, not the window thread's key state.
    io.AddKeyEvent(ImGuiMod_Ctrl, (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (GetAsyncKeyState(VK_MENU) & 0x8000) != 0);

    // Pointer: both sources, whichever actually moved. DirectInput deltas
    // are the only thing that moves while the game holds the mouse
    // exclusively (gameplay); the Windows cursor is what moves when it
    // doesn't (menus, windowed). The first build picked ONE source by whether
    // DirectInput had been read recently, which left the pointer dead in
    // whichever case it guessed wrong.
    //
    // The Windows cursor wins whenever it is alive. Measured 2026-09-13 on the
    // title screen: the game reads its DirectInput mouse there too, WITHOUT
    // holding it, so both sources move at once - and adding the deltas on top
    // drifted the pointer away from where Windows had it (hovering one row
    // while the real cursor sat on another). Deltas are only used while the
    // Windows cursor has been still for a moment, which is what an exclusive
    // (gameplay) mouse looks like.
    static ULONGLONG s_realMovedMs = 0;
    const ULONGLONG nowMs = GetTickCount64();
    POINT p;
    RECT rc;
    if (GetCursorPos(&p) && ScreenToClient(g_hwnd, &p) && GetClientRect(g_hwnd, &rc) && rc.right > 0 && rc.bottom > 0) {
        // Window pixel -> frame pixel. With the desktop view on, the window
        // shows only the region picked above.
        const float fx = g_dvActive ? g_dvX + p.x * g_dvW / rc.right : p.x * L.bbW / rc.right;
        const float fy = g_dvActive ? g_dvY + p.y * g_dvH / rc.bottom : p.y * L.bbH / rc.bottom;
        // In the eye the desktop shows, so the pointer lands on that eye's copy of the menu.
        const int eye = g_dvActive ? g_dvEye : 0;
        const float rx = (fx - L.ox[eye]) / L.sx, ry = fy - L.oy[eye];
        const bool first = g_lastRealX <= -1e8f;
        const bool realMoved = !first && (std::fabs(rx - g_lastRealX) >= 1.0f || std::fabs(ry - g_lastRealY) >= 1.0f);
        // On opening, start where the Windows cursor already is if it is over the menu area.
        const bool startHere = first && rx >= 0.0f && ry >= 0.0f && rx < L.unitsW && ry < L.unitsH;
        g_lastRealX = rx;
        g_lastRealY = ry;
        if (realMoved || startHere) {
            s_realMovedMs = nowMs;
            g_cursorX = rx;
            g_cursorY = ry;
        }
    }
    if ((m.dx || m.dy) && nowMs - s_realMovedMs > 250) {
        const float speed = L.stereo ? 1.0f : (std::max)(1.0f, L.bbH / 1080.0f);
        g_cursorX += m.dx * speed;
        g_cursorY += m.dy * speed;
    }
    if (m.fromDirectInput) {
        for (int b = 0; b < 3; ++b)
            io.AddMouseButtonEvent(b, m.buttons[b]);
        if (m.wheel)
            io.AddMouseWheelEvent(0.0f, m.wheel / 120.0f);
    }
    // The laser (2026-09-23). Where your gun hand points, as two angles from
    // where your head is looking, turned into a place on this menu. The menu
    // is drawn at a fixed place in the picture and the picture is fixed to
    // your head, so a direction from your head IS a place on it - and the
    // runtime tells us the real lens angles, so no tuning is needed: point at
    // a checkbox and the cursor is on the checkbox.
    //
    // Written after the mouse, so pointing wins while you are pointing, and
    // the mouse takes over the moment it moves because that happens after
    // this on the next frame round.
    {
        float aimX = 0.0f, aimY = 0.0f;
        bool click = false, rightClick = false;
        XRBridgeEyeView laserL, laserR;
        if (XrInput_GetMenuPointer(&aimX, &aimY, &click, &rightClick) && VRBridge_GetEyeViews(laserL, laserR)) {
            const float tanX = (std::max)(std::fabs(std::tan(laserL.angleLeft)), std::fabs(std::tan(laserL.angleRight)));
            const float tanY = (std::max)(std::fabs(std::tan(laserL.angleDown)), std::fabs(std::tan(laserL.angleUp)));
            if (tanX > 0.05f && tanY > 0.05f) {
                const float ndcX = std::tan(aimX) / tanX;
                const float ndcY = std::tan(aimY) / tanY;
                const float eyeW = L.stereo ? L.bbW * 0.5f : L.bbW;
                const float px = (0.5f + 0.5f * ndcX) * eyeW;
                const float py = (0.5f - 0.5f * ndcY) * L.bbH;
                g_cursorX = (px - L.ox[0]) / L.sx;
                g_cursorY = py - L.oy[0];
                float handX = 0.0f, handY = 0.0f;
                if (XrInput_GetMenuHand(&handX, &handY)) {
                    const float hpx = (0.5f + 0.5f * std::tan(handX) / tanX) * eyeW;
                    const float hpy = (0.5f - 0.5f * std::tan(handY) / tanY) * L.bbH;
                    g_laserFromX = (hpx - L.ox[0]) / L.sx;
                    g_laserFromY = hpy - L.oy[0];
                    g_haveLaserFrom = true;
                } else {
                    g_haveLaserFrom = false;
                }
            }
            static bool s_wasClick = false, s_wasRight = false;
            if (click != s_wasClick) {
                s_wasClick = click;
                io.AddMouseButtonEvent(0, click);
            }
            if (rightClick != s_wasRight) {
                s_wasRight = rightClick;
                io.AddMouseButtonEvent(1, rightClick);
            }
            // Holding the trigger means dragging, and a drag needs every
            // position, not just the ones that moved a whole unit
            // (2026-09-23, user: "cant scroll with the right stick nor drag
            // the slider/scroll bar"). The one-unit rule exists so a pointer
            // that twitches does not tell the interface the mouse is in
            // charge; while you are holding a button you plainly are.
            if (click || rightClick) {
                g_sentX = g_cursorX + 100.0f;
                g_sentY = g_cursorY;
            }
            // The look stick scrolls, a notch at a time, with a rest between.
            {
                const float push = XrInput_GetMenuScroll();
                static ULONGLONG s_nextNotch = 0;
                const ULONGLONG nowNotch = GetTickCount64();
                if (std::fabs(push) > 0.4f) {
                    if (nowNotch >= s_nextNotch) {
                        io.AddMouseWheelEvent(0.0f, push > 0.0f ? 1.0f : -1.0f);
                        const float speed = (std::fabs(push) - 0.4f) / 0.6f;
                        s_nextNotch = nowNotch + static_cast<ULONGLONG>(140.0f - 110.0f * speed);
                    }
                } else {
                    s_nextNotch = 0;
                }
            }
        }
    }

    g_cursorX = (std::max)(0.0f, (std::min)(g_cursorX, L.unitsW - 1.0f));
    g_cursorY = (std::max)(0.0f, (std::min)(g_cursorY, L.unitsH - 1.0f));
    // Only a real move is sent: ImGui treats ANY pointer movement as "the
    // mouse is in charge now" and hides the controller's highlight, so a
    // pointer that twitched every frame made the pad look dead.
    if (std::fabs(g_cursorX - g_sentX) >= 1.0f || std::fabs(g_cursorY - g_sentY) >= 1.0f) {
        io.AddMousePosEvent(g_cursorX, g_cursorY);
        g_sentX = g_cursorX;
        g_sentY = g_cursorY;
    }
}

// ---- Drawing --------------------------------------------------------------
void RenderToBackbuffer(IDirect3DDevice9* dev, const Layout& L)
{
    ImDrawData* dd = ImGui::GetDrawData();
    if (!dd || dd->CmdListsCount == 0 || dd->TotalVtxCount == 0)
        return;
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;

    // Whatever the game left bound goes back afterwards.
    IDirect3DSurface9* oldRt[4] = {};
    for (DWORD i = 0; i < 4; ++i)
        dev->GetRenderTarget(i, &oldRt[i]);
    IDirect3DSurface9* oldDs = nullptr;
    dev->GetDepthStencilSurface(&oldDs);
    D3DVIEWPORT9 oldVp = {};
    dev->GetViewport(&oldVp);

    dev->SetRenderTarget(0, bb);
    for (DWORD i = 1; i < 4; ++i)
        if (oldRt[i])
            dev->SetRenderTarget(i, nullptr);
    dev->SetDepthStencilSurface(nullptr);

    g_drawing.store(true, std::memory_order_release);
    StereoTest_SetSuppressed(true); // one plain draw each - no per-eye duplication
    const bool began = SUCCEEDED(dev->BeginScene());

    // The backend draws into a viewport at (0,0) the size of DisplaySize, so
    // make that the whole backbuffer and move the geometry instead.
    dd->DisplayPos = ImVec2(0, 0);
    dd->DisplaySize = ImVec2(L.bbW, L.bbH);
    std::vector<ImVec4> clips;
    for (int n = 0; n < dd->CmdListsCount; ++n)
        for (const ImDrawCmd& cmd : dd->CmdLists[n]->CmdBuffer)
            clips.push_back(cmd.ClipRect);

    const int passes = L.stereo ? 2 : 1;
    for (int e = 0; e < passes; ++e) {
        size_t ci = 0;
        for (int n = 0; n < dd->CmdListsCount; ++n) {
            ImDrawList* list = dd->CmdLists[n];
            for (ImDrawVert& v : list->VtxBuffer) {
                if (e == 0) {
                    v.pos.x = v.pos.x * L.sx + L.ox[0];
                    v.pos.y = v.pos.y + L.oy[0];
                } else {
                    v.pos.x += L.ox[1] - L.ox[0];
                    v.pos.y += L.oy[1] - L.oy[0];
                }
            }
            for (ImDrawCmd& cmd : list->CmdBuffer) {
                const ImVec4& c = clips[ci++];
                const float x0 = L.stereo ? L.clipX0[e] : 0.0f, x1 = L.stereo ? L.clipX1[e] : L.bbW;
                cmd.ClipRect.x = (std::max)(x0, c.x * L.sx + L.ox[e]);
                cmd.ClipRect.z = (std::min)(x1, c.z * L.sx + L.ox[e]);
                cmd.ClipRect.y = (std::max)(0.0f, c.y + L.oy[e]);
                cmd.ClipRect.w = (std::min)(L.bbH, c.w + L.oy[e]);
            }
        }
        ImGui_ImplDX9_RenderDrawData(dd);
    }

    if (began)
        dev->EndScene();
    StereoTest_SetSuppressed(false);
    g_drawing.store(false, std::memory_order_release);

    for (DWORD i = 0; i < 4; ++i) {
        if (i == 0 || oldRt[i])
            dev->SetRenderTarget(i, oldRt[i]);
        if (oldRt[i])
            oldRt[i]->Release();
    }
    dev->SetDepthStencilSurface(oldDs);
    if (oldDs)
        oldDs->Release();
    dev->SetViewport(&oldVp);
    bb->Release();
}

// ---- The menu itself ------------------------------------------------------
void HelpMarker(const char* text)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    ImGui::SetItemTooltip("%s", text);
}

bool ResetButton(const char* id)
{
    ImGui::PushID(id);
    const bool pressed = ImGui::SmallButton("Reset");
    ImGui::PopID();
    return pressed;
}

void StatusRow(const char* label, const char* fmt, ...)
{
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextDisabled("%s", label);
    ImGui::TableSetColumnIndex(1);
    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
}

bool BeginStatusTable(const char* id)
{
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_RowBg))
        return false;
    // Same label width in every table, so the values line up down the page.
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("Image age at submit  ").x);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

int TabFlags(int index)
{
    return g_selectTab == index ? ImGuiTabItemFlags_SetSelected : 0;
}

const ImVec4 kGood(0.5f, 0.85f, 0.5f, 1);
const ImVec4 kNotice(0.95f, 0.75f, 0.3f, 1);

void DrawCameraTab(AllSettings& s, bool& changed)
{
    ImGui::SeparatorText("First person");
    changed |= ImGui::Checkbox("First person camera", &s.cam.firstPerson);
    changed |= ImGui::Checkbox("Show head during action cameras", &s.cam.showHeadDuringActions);
    HelpMarker("When the game swings its camera out for a kick, a vault or a grab, Chris's head pops back in so "
               "you don't see a headless body. Off keeps the head hidden no matter where the camera goes.");
    // Nothing here is VR only: the game takes the camera on a flat screen
    // exactly the same way (2026-09-28, user: "it should also work in
    // flatscreen").
    changed |= ImGui::Checkbox("Refuse the melee and jump cameras", &s.cam.holdViewInMelee);
    HelpMarker("A stomp, an uppercut, a door kick or a vault hands the camera to the game, which swings it "
               "round to frame the shot. In a headset that is a shove, and on a flat screen it is still a "
               "view you did not ask for.\n\n"
               "This refuses the handover outright. Your ordinary camera stays in charge for the whole "
               "animation and you keep looking wherever you like, exactly as you can the rest of the time.\n\n"
               "A cutscene cut, a teleport or a level load still places the camera normally.");
    changed |= ImGui::Checkbox("Partner locate does not move the camera", &s.cam.partnerNoCamera);
    HelpMarker("Holding locate swings the view onto your partner. On a flat screen that is a convenience; in "
               "a headset it is somebody taking hold of your head and turning it.\n\n"
               "The locate icons still show. Only the camera move goes.");
    changed |= ImGui::Checkbox("Scoped weapons stay in third person", &s.cam.scopeThirdPerson);
    HelpMarker("A scope in a headset fills one eye and blocks everything else, so rifles that zoom are more "
               "or less unusable.\n\n"
               "RE5 already knows how to keep a scoped weapon in third person - it does it between shots with "
               "the S75 - and this forces it for every weapon, so the scope stays out where you can aim it.");
    // Both of these belong to first person rather than to VR: on a flat screen
    // you can look down into your own chest just as easily (2026-09-17).
    changed |= ImGui::Checkbox("Keep the camera on your head", &s.cam.headLock);
    HelpMarker("Puts the view on your head even when the game takes the camera - sprinting, vaulting, going "
               "through a window - instead of leaving you behind to watch your own neck. Cutscenes framed from "
               "elsewhere are left alone.");
    ImGui::BeginDisabled(!s.cam.headLock);
    changed |= ImGui::SliderFloat("Follow reach", &s.cam.headLockReach, 100.0f, 4000.0f, "%.0f units");
    HelpMarker("How far the game may throw the camera and still have it brought back to your head. Going through a "
               "window throws it a long way, and that is exactly when you want to go with it. Lower this only if a "
               "cutscene gets dragged onto your head.");
    changed |= ImGui::SliderFloat("Swing limit", &s.cam.viewTurnLimitDeg, 0.0f, 2000.0f, "%.0f deg/sec");
    HelpMarker("The fastest the game may turn your view for you. A stomp was measured swinging it at 1353 degrees a "
               "second, which no stick or mouse could ask for. The view still gets where the game wants it, just at "
               "a speed a person could have asked for. 0 lets the game do as it likes.");
    changed |= ImGui::Checkbox("Face where your character faces", &s.cam.headLockDirection);
    HelpMarker("A scripted camera swings the world around you: fine on a monitor, a fairground ride in a headset. "
               "While the game is driving, the view faces the way your character faces and your own head turning is "
               "added on top, so a revive or a vault stops spinning you.");
    ImGui::EndDisabled();
    changed |= ImGui::SliderFloat("Near clip", &s.stereo.nearPlaneUnits, 0.0f, 40.0f, "%.0f units");
    HelpMarker("How close something can get before it is clipped away. 0 keeps the game's own, which is tuned for "
               "a camera behind your shoulder and lets you look down into your own chest. Raise it until your body "
               "stops showing; too high and things you lean close to start disappearing.");

    ImGui::SeparatorText("Flat screen view");
    changed |= ImGui::SliderFloat("Field of view", &s.cam.flatFovDeg, 50.0f, 120.0f, "%.0f deg");
    HelpMarker("Horizontal. 90 is the default and felt best in testing.");
    changed |= ImGui::SliderFloat("Eye height##flat", &s.cam.flatEyeUp, 0.0f, 3.0f, "%.2f");
    HelpMarker("1.0 is the skeleton's eye, 0 is the head joint.");
    changed |= ImGui::SliderFloat("Eye forward##flat", &s.cam.flatEyeAhead, -1.0f, 2.0f, "%.2f");
    if (ResetButton("flatview")) {
        s.cam.flatFovDeg = g_defaults.cam.flatFovDeg;
        s.cam.flatEyeUp = g_defaults.cam.flatEyeUp;
        s.cam.flatEyeAhead = g_defaults.cam.flatEyeAhead;
        changed = true;
    }

    ImGui::SeparatorText("Picture");
    ImGui::BeginDisabled(!FilterPatch_IsAvailable());
    changed |= ImGui::Checkbox("Remove RE5's colour filter", &s.filterRemoved);
    ImGui::BeginDisabled(!LaserPatch_IsAvailable());
    changed |= ImGui::Checkbox("Force the laser sight on", &s.laserSight);
    if (TremorPatch_IsAvailable()) {
        changed |= ImGui::Checkbox("Steady the hands", &s.steadyHands);
        HelpMarker("RE5 shakes the character's hands while the gun is up. On a gamepad that is a difficulty "
                   "knob, making the crosshair drift so holding a shot costs something.\n\n"
                   "In a headset it is only noise: your hands are not shaking, and the arm tracking is trying "
                   "to hold the weapon exactly where your controller is while the game adds a wobble on top of "
                   "it. Off by nature, on by preference.");
    } else {
        ImGui::TextDisabled("Steady the hands - not available on this game build");
    }
    HelpMarker("RE5 shows the laser only when you aim with a pad, so the mod forces it on. Turn this OFF "
               "if another mod has its own laser setting - two of them writing the same three instructions "
               "is worth being able to stop.");
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    HelpMarker(FilterPatch_IsAvailable() ? "The heavy yellow grade over everything. Untick for the original look."
                                         : "This game executable doesn't match the one the patch was made for.");
}

void DrawVrTab(AllSettings& s, bool& changed)
{
    VRBridgeStatus vr;
    VRBridge_GetStatus(vr);

    ImGui::SeparatorText("Headset");
    if (!vr.available) {
        ImGui::BeginDisabled();
        bool off = false;
        ImGui::Checkbox("Enable VR", &off);
        ImGui::EndDisabled();
        ImGui::TextWrapped("This is the flat-screen install: dgVoodoo2 isn't next to the game, so there's nothing "
                           "to send to a headset. Install the VR package to play in VR.");
        return;
    }
    bool enabled = vr.modeEnabled;
    if (ImGui::Checkbox("Enable VR", &enabled))
        VRBridge_RequestXrMode(enabled);
    if (vr.modeEnabled && vr.sessionRunning) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "running");
    } else if (vr.modeEnabled) {
        ImGui::SameLine();
        ImGui::TextDisabled("starting...");
    } else if (vr.initFailed) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.3f, 1), "no headset / OpenXR runtime found last time");
    }
    changed |= ImGui::Checkbox("Start in VR automatically", &s.menu.autoStartVr);
    // One or the other, never both: the desktop always shows a single eye in VR.
    // Two checkboxes on one line (label first, as the user laid it out) that
    // behave as a switch - ticking one unticks the other, and the ticked one
    // can't be unticked on its own.
    {
        bool left = !s.menu.desktopRightEye, right = s.menu.desktopRightEye;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Left Eye Desktop View");
        ImGui::SameLine();
        if (ImGui::Checkbox("##desktopLeft", &left)) {
            s.menu.desktopRightEye = false;
            changed = true;
        }
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x * 4.0f);
        ImGui::TextUnformatted("Right Eye Desktop View");
        ImGui::SameLine();
        if (ImGui::Checkbox("##desktopRight", &right)) {
            s.menu.desktopRightEye = true;
            changed = true;
        }
        HelpMarker("Which eye the game window shows while in VR, so streaming and recording look normal. The "
                   "headset is not affected.");
    }

    ImGui::BeginDisabled(!vr.sessionRunning);
    if (ImGui::Button("Reset view"))
        VRBridge_RequestRecenter("menu");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("or hold both sticks in for a second");

    ImGui::SeparatorText("Motion controllers");
    {
        changed |= ImGui::Checkbox("Use motion controllers", &s.xrInput.enabled);
        HelpMarker("Your VR controllers act as a gamepad: right grip aims, right trigger fires, the left stick "
                   "moves and the right stick looks. The gun still points where the game points it; this is "
                   "buttons and sticks, not motion aiming.");

        char controllers[160] = "";
        XrInput_DescribeStatus(controllers, sizeof(controllers));
        ImGui::TextDisabled("Seen: %s", controllers);

        ImGui::BeginDisabled(!s.xrInput.enabled);
        static const char* const kDpadMethods[] = {
            "Hold left trigger, right stick",
            "Right thumbrest touch, left stick",
            "Hold left stick in, right stick",
            "Right stick is always the d-pad",
            "No d-pad",
        };
        changed |= ImGui::Combo("D-pad", &s.xrInput.dpadMethod, kDpadMethods, 5);
        HelpMarker("A pad has a d-pad and a controller doesn't, so one stick stands in for it while you hold the "
                   "modifier. The thumbrest option only exists on Quest and Rift Touch controllers.");
        changed |= ImGui::SliderFloat("Stick deadzone", &s.xrInput.deadzone, 0.0f, 0.5f, "%.2f");
        HelpMarker("How much of the stick's centre to ignore. Raise it if you drift while standing still.");
        changed |= ImGui::Checkbox("Look to turn", &s.xrInput.headTurn);
        HelpMarker("Turn your head and your character turns to face the same way, so his body is always where "
                   "you are looking. Your thumb still works; the two add together. Turning back stops him, "
                   "because he only turns while your head is ahead of him.");
        ImGui::BeginDisabled(!s.xrInput.headTurn);
        changed |= ImGui::SliderFloat("Before he turns", &s.xrInput.headTurnDeadDeg, 5.0f, 60.0f, "%.0f deg");
        HelpMarker("How far you can glance without him following. Too small and he chases every look; too "
                   "large and you have to crane round before anything happens.");
        changed |= ImGui::SliderFloat("Full speed at", &s.xrInput.headTurnFullDeg, 20.0f, 120.0f, "%.0f deg");
        HelpMarker("How far past that before he turns as fast as the stick would.");
        ImGui::EndDisabled();
        changed |= ImGui::Checkbox("Stick looks up and down", &s.xrInput.stickPitch);
        HelpMarker("Off, the look stick only turns you left and right and your head does the rest. On, it also "
                   "tilts the view up and down the way a pad does, which fights your head and is hard to put "
                   "back level.");
        // 3DOF aiming, as one checkbox (2026-09-17). It used to be a pile of
        // ini keys because it was a servo chasing the gun around and needed
        // tuning per person. It is not that any more: the pitch is written
        // straight into the game's own aim and the yaw turns the body, both
        // absolute, so there is nothing left to tune and nothing to explain.
        // Everything it needs - writing the rate rather than the stick, the
        // body turn, the ceiling, the filter - is set to what worked and moved
        // out of sight. The individual knobs live on in the Developer tab.
        //
        // Left-handed belongs here too and is still out: it decides which real
        // hand points the gun, which needs testing with someone left-handed
        // rather than a guess.
        changed |= ImGui::Checkbox("3DOF", &s.xrInput.pointToAim);
        HelpMarker("The gun turns to wherever your controller points. Aiming only - your arms stay on the "
                   "game's own animation.");
        changed |= ImGui::Checkbox("6DOF arms", &s.xrInput.armIk);
        // Not greyed out when 6DOF is off (2026-09-18): dimmed controls read as
        // absent, and the user went looking for these and could not find them.
        ImGui::Indent();
#if RE5VR_DIAGNOSTICS
        // On is the intended setting and always has been, so a release build
        // simply IS that setting rather than offering a switch nobody should
        // touch (2026-09-28, user: "condense all of our dev stuff that is
        // outside of the dev tab").
        changed |= ImGui::Checkbox("Drive the pose, not the result", &s.xrInput.armIkWriteLocal);
        HelpMarker("Sets each joint's own rotation, which is what the game builds the character from, "
                   "instead of painting over the result. Held items follow the hand with it on, and stay "
                   "behind with it off. On is the intended setting; it is here so it can be taken away.");
#endif
        changed |= ImGui::Checkbox("Turn the gun with your hand", &s.xrInput.armIkWrist);
        DrawArmCalibration();
        changed |= ImGui::SliderFloat("Arm reach", &s.xrInput.armIkScale, 0.5f, 2.0f, "%.2fx");
        changed |= ImGui::SliderFloat("Shoulder movement", &s.xrInput.armIkShoulder, 0.0f, 1.0f, "%.2f");
        HelpMarker("How much the shoulder travels when you reach past comfortable, the way a real one does. "
                   "0 pins it where the animation left it and the arm does all the work from the elbow.");
#if RE5VR_DIAGNOSTICS
        changed |= ImGui::SliderFloat("Reach allowance", &s.xrInput.armIkReachAllowanceM, 0.0f, 0.30f,
            "%.2f m");
        HelpMarker("How much further you can reach while playing than you could while holding the T-pose, "
                   "because your shoulder travels and the pose does not show that. It is ADDED to your "
                   "measured arm, so raising it makes the character's arms SHORTER. If your arms come out "
                   "stubby, lower it and calibrate again. 0 trusts the measurement exactly.");
#endif
        changed |= ImGui::SliderFloat("Wrist pivot", &s.xrInput.armIkWristPivotM, 0.0f, 0.15f, "%.2f m");
        changed |= ImGui::Checkbox("Hands stop at people", &s.xrInput.armIkTouch);
        HelpMarker("Your hands hold at Sheva and at your own body instead of passing through. Your real hand "
                   "carries on, and the two parting company is what reads as something solid.");
        ImGui::BeginDisabled(!s.xrInput.armIkTouch);
        changed |= ImGui::SliderFloat("How solid", &s.xrInput.armIkTouchRadius, 6.0f, 30.0f, "%.0f");
        HelpMarker("How thick a body is. Raise it if you can push into somebody before you feel them; lower it "
                   "if you cannot reach past an arm to what is behind it.");
        ImGui::EndDisabled();
        changed |= ImGui::Checkbox("Your body leans with you", &s.xrInput.spineLean);
        HelpMarker("Lean out from cover and your character leans too, from the base of his spine, instead of "
                   "only the camera moving. Needs leaning and peeking turned on, since it follows the same "
                   "head movement. Whether it moves you out of the way of a swing is something to find out.");
        ImGui::BeginDisabled(!s.xrInput.spineLean);
        changed |= ImGui::SliderFloat("How far he leans", &s.xrInput.spineLeanAmount, 0.0f, 1.5f, "%.2f");
        HelpMarker("1 puts his head where yours is. Less keeps him nearer upright, more exaggerates it.");
        changed |= ImGui::SliderFloat("Most he will bend", &s.xrInput.spineLeanMaxDeg, 5.0f, 60.0f, "%.0f deg");
        HelpMarker("A ceiling on the bend, so a big step in the room does not fold him in half.");
        ImGui::EndDisabled();
        changed |= ImGui::Checkbox("Step in your room and he steps", &s.xrInput.roomStep);
        HelpMarker("Walk or sidestep where you are standing and your character walks to catch up, so you and he "
                   "stay in the same place. Needs leaning and peeking on, since it follows the same head "
                   "movement, and it does nothing while you are aiming because the game will not let you walk "
                   "then. This is the only kind of dodging that moves what an enemy can hit.");
        ImGui::BeginDisabled(!s.xrInput.roomStep);
        changed |= ImGui::SliderFloat("Before he starts", &s.xrInput.roomStepDeadM, 0.05f, 0.40f, "%.2f m");
        HelpMarker("How far you can lean and shift about before his legs get involved. Too small and he "
                   "shuffles while you stand still; too large and a real step does nothing.");
        changed |= ImGui::SliderFloat("Flat out at", &s.xrInput.roomStepFullM, 0.20f, 1.50f, "%.2f m");
        HelpMarker("How far out of place you have to be before he is running to catch up.");
        ImGui::EndDisabled();
        changed |= ImGui::Checkbox("Holsters", &s.xrInput.holsters);
        HelpMarker("Reach to a spot on your body and squeeze the gun hand's grip to draw from it. Over your "
                   "left shoulder is the knife; your right hip, left hip, right shoulder and lower back are "
                   "the four d-pad slots you assigned in the inventory. That grip does not raise your "
                   "weapon while it is drawing.");
        changed |= ImGui::Checkbox("Two hands on the gun", &s.xrInput.twoHanded);
        HelpMarker("Put your other hand on the gun - cupping a pistol, or out along a rifle or shotgun - and "
                   "squeeze its grip. Long guns then point along the line between your hands. With holsters "
                   "on, that grip no longer readies the knife; the knife comes from its holster.");
        ImGui::BeginDisabled(!s.xrInput.twoHanded);
        changed |= ImGui::SliderFloat("Support hand height", &s.xrInput.twoHandRaiseCm, -5.0f, 15.0f, "%.1f cm");
        HelpMarker("Where your other hand sits on a rifle, shotgun or SMG. Raise it if the hand hangs under "
                   "the gun, lower it if it sinks into it. Changes show while you are holding.");
        changed |= ImGui::Checkbox("Lock the gun in the hand", &s.xrInput.gunLock);
        HelpMarker("The weapon keeps the place in your hand it already had, instead of being rebuilt every frame "
                   "from the character's animation. This is what takes out the shake while firing and the hop on "
                   "the way into aim. The correction is made inside the game's own code, where it cannot be "
                   "overwritten before the frame is drawn.");
        ImGui::BeginDisabled(!s.xrInput.gunLock);
        changed |= ImGui::SliderFloat("Firing kick, one hand", &s.xrInput.kickOneHanded, 0.0f, 1.0f, "%.2f");
        HelpMarker("How much the gun jumps in your hand when it fires, held one-handed. 1 is the game's full kick, "
                   "0 holds it dead still.");
        changed |= ImGui::SliderFloat("Firing kick, two hands", &s.xrInput.kickTwoHanded, 0.0f, 1.0f, "%.2f");
        HelpMarker("The same with your other hand on the gun.");
        ImGui::EndDisabled();
#if RE5VR_DIAGNOSTICS
        changed |= ImGui::Checkbox("Freeze the body while firing (experimental)", &s.xrInput.steadyHandsFiring);
        HelpMarker("Every bone of your character is pinned to the pose it had when the burst started, except the "
                   "arms, which keep following your hands. A test as much as a setting: if the gun still shakes "
                   "with the whole body frozen, nothing on the skeleton is moving it.");
#endif
        changed |= ImGui::SliderFloat("Gun size", &s.xrInput.gunScale, 0.6f, 1.2f, "%.2f");
        HelpMarker("How big the weapon in your hand is drawn. 1.00 is the game's own size, which testers found "
                   "too big in a headset. Only on your machine: a co-op partner still sees the normal size.");
        changed |= ImGui::SliderFloat("Support hand steering", &s.xrInput.twoHandSteer, 0.0f, 1.0f, "%.2f");
        HelpMarker("How much of a long gun's aim your other hand has. 1 is the line between your hands alone, "
                   "steady but heavy to swing between targets. 0 is your gun hand alone. In between, your gun "
                   "wrist still flicks the aim and the other hand steadies it.");
        changed |= ImGui::SliderFloat("Support hand roll", &s.xrInput.twoHandRoll, 0.0f, 1.0f, "%.2f");
        HelpMarker("How much the gun rolls with your support hand. Pointing the gun at your other hand cannot "
                   "roll it on its own, so without this a rifle stays flat however you turn your wrists. Works "
                   "on a cupped pistol as well, which makes lining up the sights easier.");
        ImGui::EndDisabled();
        changed |= ImGui::Checkbox("Point at this menu with your gun hand", &s.xrInput.menuLaser);
        HelpMarker("Aim your controller at the menu and the cursor goes where you point, trigger to click and "
                   "grip for a right click. The mouse and the pad still work; whichever you used last is the "
                   "one in charge.");
        changed |= ImGui::Checkbox("Tell the game where you are looking from", &s.xrInput.cameraFollowsEye);
        HelpMarker("The mod replaces the view the renderer draws with, but the game keeps its own camera "
                   "and sorts see-through surfaces, measures fade distances and picks detail levels from "
                   "that. When the two disagree - during a stomp, a punch or any action camera - "
                   "transparencies come out in the wrong order and look like a stipple that never "
                   "resolves.\n\n"
                   "This moves the game's own camera to your eye as well. It needs to know where that "
                   "camera keeps its position, which it works out by watching; the log says when it has "
                   "settled on an answer.");
        changed |= ImGui::Checkbox("Tap your wrist for the inventory", &s.xrInput.wristInventory);
        HelpMarker("Tap the top of your other wrist with your gun hand, the way you glance at a watch, and "
                   "the inventory opens. No button, and it has to be a tap rather than your hands resting "
                   "near each other.");
        {
            // HANDS ONLY IS GONE (2026-09-28, user: "we need to reimplement
            // arms only (skip hands only) with our collapsing of the vertex,
            // it is not perfect but it works").
            //
            // Two modes that work beat three where one never did. Arms is
            // the one worth keeping, and anything saved as Hands comes back
            // as Arms rather than as a number with no option behind it.
            if (s.xrInput.bodyVisible > 1)
                s.xrInput.bodyVisible = 1;
            const char* how[] = { "Full body", "Arms" };
            changed |= ImGui::Combo("How much of you is visible", &s.xrInput.bodyVisible, how, 2);
            HelpMarker("Full body is the whole character minus your head, which is inside the camera. Arms "
                       "hides everything that is not elbow to hands.\n\n"
                       "Done to the mesh as it is drawn rather than to the skeleton, so the camera and your "
                       "arm tracking are untouched and there is nothing to restore when you switch back. "
                       "Only bones near YOUR head and wrists are affected, so a co-op partner keeps theirs.");
            if (s.xrInput.bodyVisible > 0) {
                // ONE PAIR AT A TIME (2026-09-26, user: "I think they do share
                // settings though, cause as i flip between the settings, the
                // options i entered stay the same").
                //
                // They never shared anything - Arms has only ever read the
                // elbows and Hands only the wrists - but showing both pairs in
                // both modes made switching mode look like it changed nothing,
                // which is a fair reading of what was on screen. Only the pair
                // the current mode actually uses is shown now, so the boxes
                // change when the mode does.
                int* steps = s.xrInput.cutStepsUp[0];
                // Only as far as the arm goes. Anything beyond lands on the
                // same bone and reads as a slider that does nothing.
                int have[2] = { 4, 4 };
                const bool knowArm = ArmIk_ArmSteps(have);
                const int most = have[0] > have[1] ? have[0] : have[1];
                changed |= ImGui::SliderInt2("How much arm you keep, left and right", steps, 0,
                    most > 0 ? most : 1, "%d joint(s) up");
                if (knowArm)
                    ImGui::TextDisabled("    this arm has %d joint(s) on the left and %d on the right", have[0],
                        have[1]);
                HelpMarker("Counted up each arm from its own wrist, one joint at a time. Zero keeps the hand "
                           "alone, and each step up keeps a little more of the forearm. It stops on its own "
                           "at the shoulder, so it can never reach your torso.\n\n"
                           "There is one for each side because the two arms are not the same chain - the gun "
                           "hand carries extra twist and helper bones, so the same place on the arm is a "
                           "different number of steps on the left and the right.\n\n"
                           "The log says how many steps were available on each side before the shoulder.");
                changed |= ImGui::SliderFloat(
                    "How far back the cut sits", &s.xrInput.bodyCutBack, 0.06f, 0.20f, "%.2f m");
                HelpMarker("Where your arm stops being drawn, measured back from the elbow or the wrist. "
                           "Larger leaves more forearm and more of a cuff on the glove.\n\n"
                           "It is also what keeps the twist and helper bones that shape your hand, so "
                           "taking it too low starts costing you fingers. Around 0.11 suits Hands and "
                           "0.09 suits Arms.");
            }
        }
        changed |= ImGui::Checkbox("Reload by hand", &s.xrInput.manualReload);
        HelpMarker("Reach to your belt on your support side and squeeze the grip: the magazine drops and a "
                   "fresh one is in your hand. Bring that hand to the weapon and it goes in, with as much as "
                   "you are carrying. Take your hand away and it is racked. The gun is empty in between, and "
                   "letting go before the magazine arrives puts the old one back.");
        ImGui::BeginDisabled(!s.xrInput.manualReload);
        {
            static const char* const kMagModes[] = {
                "Arcade style - save current ammo",
                "Realistic - lose current ammo",
            };
            int magMode = s.xrInput.reloadKeepsRounds ? 0 : 1;
            if (ImGui::Combo("Part used magazines", &magMode, kMagModes, 2)) {
                s.xrInput.reloadKeepsRounds = magMode == 0;
                changed = true;
            }
            HelpMarker("Arcade style puts whatever was left in the magazine back into your spare "
                       "ammunition, so reloading early costs you nothing.\n\n"
                       "Realistic lets those rounds leave with the magazine, the way they would.");
        }
        if (ImGui::Checkbox("Magazine sounds", &s.xrInput.reloadSounds)) {
            changed = true;
            Sound_Forget(); // so a file swapped while the game runs is picked up
        }
        ImGui::BeginDisabled(!s.xrInput.reloadSounds);
        changed |= ImGui::SliderFloat("Magazine volume", &s.xrInput.reloadSoundVolume, 0.0f, 1.0f, "%.2f");
        HelpMarker("These play beside the game rather than through it, so RE5's own effects slider cannot "
                   "reach them. This is theirs.");
        ImGui::EndDisabled();
        HelpMarker("Plays re5vr_magout.wav when the magazine drops, re5vr_magin.wav when it goes in and "
                   "re5vr_dryfire.wav on an empty trigger, "
                   "from the Resident Evil 5 folder next to the game. The mod ships neither: put any short "
                   "WAV there and it is used. Nothing breaks if they are missing, those steps are just "
                   "silent. The eject is the one worth having, since the game's own reload noise only "
                   "plays when the magazine goes back in.");
        {
            int bone = -1, which = 0, outOf = 0;
            if (ArmIk_MagazinePick(bone, which, outOf)) {
                ImGui::Text("This weapon: part %d of %d is the magazine (bone %d)", which, outOf, bone);
                ImGui::BeginDisabled(outOf < 2);
                if (ImGui::Button("That was the wrong part"))
                    ArmIk_NextMagazinePick();
                ImGui::EndDisabled();
                HelpMarker("If the thing that drops out of your gun is a shell or a bolt rather than the "
                           "magazine, press this and reload again. The mod can measure what MOVES on any "
                           "weapon, but which mover is the magazine is a guess, and no rule about grips or "
                           "timings survives thirty different guns. You can see it; it cannot. One press "
                           "per weapon at most, and it is remembered for good.");
            } else {
                ImGui::TextDisabled("This weapon has not been watched through a reload yet.");
            }
        }
#if RE5VR_DIAGNOSTICS
        changed |= ImGui::InputInt("Magazine bone", &s.xrInput.magazineBone);
        changed |= ImGui::InputInt("Carried with it", &s.xrInput.magazineWithBone);
        HelpMarker("Which part of the weapon is its magazine, and which part travels with it. 7 and 8 are "
                   "right for the M92F and the MP5, so they are very likely a convention across the "
                   "models.\n\n"
                   "Set either to -1 for a weapon they are wrong on, and the mod watches one reload and "
                   "works that weapon out for itself, remembering it in re5vr_magbones.ini.");
#endif
        ImGui::EndDisabled();
        ImGui::SeparatorText("Co-op");
        changed |= ImGui::Checkbox("Send my arms to my partner", &s.ikSync.enabled);
        HelpMarker("Your co-op partner sees your arms doing what your arms are really doing, instead of "
                   "what the animation says - and you see theirs, if they are on this build.\n\n"
                   "It carries hand and shoulder positions and nothing else. It is not the game\x27s own "
                   "netcode and it cannot desync a session, because all it changes is how the other "
                   "character is drawn on your own machine. With nobody to talk to it does nothing, which "
                   "is why it is on by default.");
        {
            unsigned long long steamPartner = 0;
            unsigned steamSent = 0, steamGot = 0;
            if (IkSync_GetSteamStatus(&steamPartner, &steamSent, &steamGot) && steamPartner) {
                ImGui::SameLine();
                if (steamSent && !steamGot)
                    ImGui::TextDisabled("(sending, nothing back)");
                else if (steamGot)
                    ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "(connected)");
            }
        }

        ImGui::SeparatorText("This character");
        changed |= ImGui::Checkbox("Playing a left-handed character (Sheva)", &s.xrInput.characterLeftHanded);
        HelpMarker("Sheva holds her weapon and her knife in her LEFT hand, so playing her moves aiming, "
                   "drawing from a holster and the knife to your left controller. Separate from your "
                   "own handedness, and the two combine: a left-handed player playing Sheva is back to "
                   "the normal layout.");
        changed |= ImGui::Checkbox("Swing to knife", &s.xrInput.meleeSwing);
        HelpMarker("Draw the knife from its holster over your shoulder (or hold the knife grip, with holsters off), "
                   "then swing a hand instead of pulling the trigger. A swing on its own never puts a knife in your hand.");
        ImGui::BeginDisabled(!s.xrInput.meleeSwing);
        changed |= ImGui::SliderFloat("Swing strength", &s.xrInput.meleeSwingSpeed, 1.5f, 6.0f, "%.1f m/s");
        HelpMarker("How hard a swing has to be. Lower it if your swings are not landing, raise it if the knife "
                   "goes in while you are only moving about.");
        ImGui::EndDisabled();
#if RE5VR_DIAGNOSTICS
        changed |= ImGui::Checkbox("Measure holster spots", &s.xrInput.holsterMeasure);
        HelpMarker("Prints where your gun hand is, in body coordinates, every time you squeeze its grip. "
                   "Put your hand where a holster should be, squeeze, and read the line back to me. The "
                   "zones get set from your body instead of from a guess at it.");
        // Buttons, not a stepper (2026-09-19). Stepping up from -1 pokes the
        // spine, the neck and the head on the way to the wrist, which breaks
        // Chris before you ever reach the joint you wanted.
        ImGui::TextUnformatted("Poke joint");
        ImGui::SameLine();
        {
            const int choices[5] = { -1, 51, 52, 53, 54 };
            const char* labels[5] = { "Off", "51", "52", "53", "54" };
            for (int i = 0; i < 5; ++i) {
                if (i)
                    ImGui::SameLine();
                const bool on = s.xrInput.armIkPokeJoint == choices[i];
                if (on)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
                char id[16];
                _snprintf_s(id, sizeof(id), _TRUNCATE, "%s##poke%d", labels[i], i);
                if (ImGui::Button(id)) {
                    s.xrInput.armIkPokeJoint = choices[i];
                    changed = true;
                }
                if (on)
                    ImGui::PopStyleColor();
            }
        }
        HelpMarker("Turns one joint a long way so you can see what it drives. Off hands it straight back to "
                   "the animation. Say what moves for each - the forearm, the hand, the gun, the fingers, "
                   "or nothing.");
        changed |= ImGui::SliderFloat("Poke angle", &s.xrInput.armIkPokeDeg, -180.0f, 180.0f, "%.0f deg");
#endif
        HelpMarker("How far back from your controller the arm aims. A controller sits in your palm, so at 0 "
                   "the arm ends where your palm is and flexing your wrist swings the whole arm. Raise it "
                   "until the bend happens at your wrist instead of up your forearm - about 0.09 for most "
                   "controllers. Raising it shortens reach a little, so check Arm reach afterwards.");
        HelpMarker("Above 1 reaches further for the same real movement. Calibrating should make 1.00 right, "
                   "so if the arms still come up short, nudge it and tell me by how much - that is a "
                   "measurement, not a preference.");
        HelpMarker("Takes the forearm's direction and roll from your controller, which turns the gun over. "
                   "Known fault: the roll sits ninety degrees out, so the controller has to be held turned "
                   "for it to look straight. Off by default until that is fixed.");
        ImGui::Unindent();
        HelpMarker("Your controllers drive the character's arms: the hands go where you are holding them, "
                   "the forearms turn as you turn your wrists, and whatever is being held comes along. Aiming "
                   "still points the gun where you point it.");
        HelpMarker("The gun goes where you point, up and down and left and right, instead of where the stick puts "
                   "it. Chris turns to follow your hand. Raising the gun ties it to wherever you are pointing, so "
                   "lower it and raise it again to recentre after you turn your chair.");
        ImGui::EndDisabled();
    }

    ImGui::SeparatorText("Resolution");
    {
        RenderSizeStatus rs;
        RenderSize_GetStatus(rs);
        ImGui::BeginDisabled(!rs.available);
        changed |= ImGui::Checkbox("Full resolution per eye", &s.res.fullResPerEye);
        ImGui::EndDisabled();
        HelpMarker(rs.available
                ? "Renders each eye at your VR runtime's own resolution - set it in SteamVR or Virtual Desktop, then "
                  "turn VR off and on here. Off gives each eye half of the game's resolution, like older versions. Your own resolution comes back when VR "
                  "is off."
                : "This game executable doesn't match the one this was made for, so each eye gets half of the "
                  "game's resolution.");
        const ImVec4 warn(0.95f, 0.65f, 0.3f, 1);
        if (!s.res.fullResPerEye || !rs.available) {
            ImGui::TextDisabled("Each eye: half the game's frame");
        } else if (vr.recommendedEyeWidth) {
            UINT eyeW = 0, eyeH = 0;
            const bool capped =
                RenderSize_EyeSizeFor(vr.recommendedEyeWidth, vr.recommendedEyeHeight, &eyeW, &eyeH);
            ImGui::TextDisabled("Each eye: %u x %u", eyeW, eyeH);
            if (capped) {
                ImGui::SameLine();
                ImGui::TextColored(warn, "(runtime asks for %u x %u)", vr.recommendedEyeWidth, vr.recommendedEyeHeight);
                HelpMarker("Your runtime's resolution needs more video memory than dgVoodoo is set to give the "
                           "game, so it's scaled down to the most that fits. Raise VRAM in dgVoodoo.conf (2048 is "
                           "plenty), or pick a lower resolution preset in SteamVR or Virtual Desktop.");
            }
        } else {
            ImGui::TextDisabled("Each eye's size comes from your VR runtime when VR starts.");
        }
        const bool wantsVrSize = s.res.fullResPerEye && rs.available && !rs.failed;
        if (vr.eyeWidth && vr.modeEnabled && !rs.pending && wantsVrSize != rs.active) {
            ImGui::TextColored(warn, "Turn VR off and on to apply");
        }
        if (rs.failed)
            ImGui::TextColored(warn, "RE5 wouldn't switch to the VR size this session - see re5vr.log");
        if (ResetButton("res")) {
            s.res = g_defaults.res;
            changed = true;
        }
    }

    ImGui::SeparatorText("View");
    changed |= ImGui::Checkbox("Culling follows your head", &s.cam.headFollow);
    HelpMarker("The game draws whatever you look at, aiming included, so nothing vanishes over your shoulder. "
               "Aiming and walking stay on the mouse or stick. Off, things outside the game camera's view can "
               "disappear.");
    changed |= ImGui::Checkbox("Stabilise camera", &s.cam.vrStabilise);
    HelpMarker("Smooths Chris's idle-animation sway out of your view.");
    changed |= ImGui::Checkbox("Eyes on the neck bone", &s.cam.vrEyeOnNeck);
    HelpMarker("Carries your view on Chris's neck instead of his head bone. The head nods, rocks when he fires "
               "and swings when he runs; the neck does far less of all three. Set the height below to bring your "
               "eyes back to eye level.");
    ImGui::BeginDisabled(!s.cam.vrEyeOnNeck);
    changed |= ImGui::SliderFloat("Eye height above the neck", &s.cam.vrEyeAboveNeck, 0.0f, 45.0f, "%.0f units");
    HelpMarker("How far above the neck bone your eyes sit. About 20 units is head height; raise it if you feel "
               "short, lower it if you are looking down on everything. 11.5 units is roughly 13 cm.");
    ImGui::EndDisabled();
    changed |= ImGui::SliderFloat("View steadiness", &s.cam.vrViewSteady, 0.0f, 1.0f, "%.2f");
    HelpMarker("Keeps your character's idle animation out of your head. His neck moves while he breathes and "
               "shifts his weight, and your eye sits on it, so standing perfectly still can still jiggle. 0 is "
               "the bone exactly as animated; 1 holds it as still as it can while he is not going anywhere. "
               "Walking, turning and crouching all still move you.");
    changed |= ImGui::SliderFloat("Running lead", &s.cam.vrRunLead, 0.0f, 8.0f, "%.1f ticks");
    HelpMarker("Keeps your eyes in Chris's head when he runs. The view is moved on by how far he travelled in "
               "the last game tick, level and without any lean, so it lands where his head is rather than "
               "where it was. 0 is off. Raise it if you still see his neck or shoulders when running forward.");
    changed |= ImGui::SliderFloat("Head steadiness", &s.cam.vrHeadSteady, 0.0f, 1.0f, "%.2f");
    HelpMarker("Holds the game camera still against the small movements your head makes while you are talking, "
               "breathing or just sitting. What you see still follows your head exactly - this only steadies what "
               "the game does about it. 0 is off. Higher holds harder and takes a little longer to let go when you "
               "really turn.");
    // The same setting as on the Camera tab, under the name that describes
    // what it does for somebody wearing a headset: a camera that wanders out
    // of your body is a much bigger deal in VR than on a monitor.
    changed |= ImGui::Checkbox("Keep camera inside body", &s.cam.headLock);
    HelpMarker("Keeps the view in your head when the game tries to move it - sprinting, vaulting, going through a "
               "window - instead of leaving you behind or swinging out to third person. Cutscenes framed from "
               "elsewhere are left alone.");
    changed |= ImGui::Checkbox("Lean and peek", &s.stereo.headPositionTracking);
    HelpMarker("Your head's real movement moves the view: lean out from behind cover, duck, or put your face up "
               "to something to look closer. Only movement from where you were sitting when you switched it on "
               "counts, and Recentre takes wherever you are now as the new middle.");
    ImGui::BeginDisabled(!s.stereo.headPositionTracking);
    changed |= ImGui::SliderFloat("Lean distance", &s.stereo.leanScale, 0.0f, 3.0f, "%.2fx");
    HelpMarker("1.00x moves the game as far as you really move. Higher reaches further out from cover for the same "
               "lean, at the cost of feeling less like your own body.");
    if (ImGui::Button("Recentre lean"))
        StereoTest_RecentreLean();
    ImGui::EndDisabled();
    changed |= ImGui::SliderFloat("Eye height##vr", &s.cam.vrEyeUp, 0.0f, 3.0f, "%.2f");
    changed |= ImGui::SliderFloat("Eye forward##vr", &s.cam.vrEyeAhead, -1.0f, 2.0f, "%.2f");
    changed |= ImGui::SliderFloat("World scale", &s.stereo.halfSeparation, 0.5f, 8.0f, "%.2f");
    HelpMarker("Eye separation in game units. Higher makes the world look smaller, lower makes it look bigger. "
               "2.75 was measured to make Sheva, guns and doors feel life-size.");
    if (ResetButton("vrview")) {
        s.cam.vrEyeUp = g_defaults.cam.vrEyeUp;
        s.cam.vrEyeAhead = g_defaults.cam.vrEyeAhead;
        s.stereo.halfSeparation = g_defaults.stereo.halfSeparation;
        changed = true;
    }

    ImGui::SeparatorText("HUD");
    // No distance slider: changing the convergence only spread the two eyes'
    // copies apart (user, 2026-09-13). Size is what makes it feel nearer or further.
    // Presented as distance because that is how it reads in the headset: the
    // HUD's size (0.3-1.0 of an eye) mapped so 0 is closest and 100 furthest.
    float hudDistance = (1.0f - s.stereo.hudScale) / 0.7f * 100.0f;
    if (ImGui::SliderFloat("HUD distance", &hudDistance, 0.0f, 100.0f, "%.0f")) {
        s.stereo.hudScale = 1.0f - (std::max)(0.0f, (std::min)(hudDistance, 100.0f)) / 100.0f * 0.7f;
        changed = true;
    }
    HelpMarker("0 is closest, 100 is furthest.");
    if (ResetButton("hud")) {
        s.stereo.hudScale = g_defaults.stereo.hudScale;
        changed = true;
    }

    ImGui::SeparatorText("Ammunition");
    {
        // WHAT THE GUN IS HOLDING (2026-09-25). Manual reloading needs the
        // three numbers to be right before a gesture is worth writing, and the
        // fastest way to know they are is to put them on the screen next to
        // the gun. Everything here reads, except the two buttons, which do
        // exactly what the reload gesture will do and nothing more - so one
        // press each says whether a written round survives the game's next
        // sync, which is the only question left.
        if (!Ammo_Installed()) {
            ImGui::TextDisabled("The magazine hook is not on.");
            HelpMarker("The instruction that writes ammunition was not where this build expects it. The log "
                       "says what was there instead.");
        } else {
            AmmoSlot held;
            if (Ammo_Held(held)) {
                ImGui::Text("In the gun: %d of %d, %d spare", held.loaded, held.capacity, held.reserve);
            } else {
                ImGui::TextDisabled("Nothing that looks like a loaded weapon yet.");
                HelpMarker("Fire a shot, or switch weapons, and the game writes the magazine where this can "
                           "see it.");
            }
            static bool s_keepRounds = true;
            ImGui::Checkbox("Keep the rounds in an ejected magazine", &s_keepRounds);
            HelpMarker("On, whatever was still in the magazine goes back into your spare ammunition, so "
                       "reloading early costs you nothing. Off, those rounds leave with the magazine.");
            if (ImGui::Button("Drop the magazine"))
                Ammo_Eject(s_keepRounds);
            ImGui::SameLine();
            if (ImGui::Button("Put a fresh one in")) {
                int took = 0;
                if (!Ammo_Seat(&took))
                    Log_Printf("Ammo: nothing to put in, or nowhere to put it");
            }
            ImGui::SameLine();
            if (ImGui::Button("Write it to the log"))
                Ammo_Dump();
            if (ImGui::Button("What is the gun made of"))
                ArmIk_DumpGunBones();
            ImGui::SameLine();
            if (ImGui::Button("Watch the next reload"))
                ArmIk_WatchTheReload();
            HelpMarker("Press this and then reload once. Where a bone sits does not say what it is, but "
                       "what it does during a reload says it outright: the magazine leaves the weapon by "
                       "several centimetres and comes back, and nothing else on a pistol does that.");
            HelpMarker("Every bone of the weapon in your hand and where it sits relative to the grip. "
                       "Looking for a magazine: if the model has one as its own bone, it can be dropped "
                       "out of the gun and carried in your hand without any new geometry.");
            if (ImGui::Button("Find where the spare rounds live"))
                Ammo_FindTheBag();
            HelpMarker("Taking rounds out of the bag does not stick: the game puts the old number back a "
                       "moment later, which means it keeps the real count somewhere else. This looks for "
                       "that somewhere in three ways at once and writes what it finds to the log. Press it "
                       "with a weapon in hand and then fire a shot.");
            AmmoSlot all[16];
            const int n = Ammo_Slots(all, 16);
            if (n > 0 && ImGui::TreeNode("Every magazine seen")) {
                for (int i = 0; i < n; ++i) {
                    const AmmoSlot& a = all[i];
                    ImGui::Text("%p  %d of %d, %d spare  (%lu writes, mark %04X)%s", a.mag, a.loaded, a.capacity,
                        a.reserve, a.hits, a.mark, a.alive ? "" : "  gone");
                }
                ImGui::TreePop();
            }
        }
    }

#if RE5VR_DIAGNOSTICS
    // A trainer in two buttons, and it calls into arm_ik.cpp helpers that are
    // themselves diagnostics only - so a release build would not link
    // (2026-09-28, found the first time one was ever built).
    ImGui::SeparatorText("Find a number");
    {
        // A trainer in two buttons (2026-09-25). Five scans for patterns found
        // nothing but index buffers; watching a number change cannot be fooled
        // that way, and putting it behind a box means the next one costs a test
        // rather than a build.
        static int s_findValue = 10;
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputInt("Value", &s_findValue);
        if (s_findValue < 0)
            s_findValue = 0;
        if (ImGui::Button("Find"))
            MemFind_First(static_cast<unsigned>(s_findValue));
        ImGui::SameLine();
        if (ImGui::Button("Narrow"))
            MemFind_Next(static_cast<unsigned>(s_findValue));
        static char s_address[32] = "0BECB248";
        ImGui::SetNextItemWidth(140.0f);
        ImGui::InputText("Address (hex)", s_address, sizeof(s_address));
        if (ImGui::Button("Watch it")) {
            unsigned a = 0;
            if (sscanf_s(s_address, "%x", &a) == 1)
                MemFind_Watch(a);
            else
                Log_Printf("Watch: %s is not a hex address", s_address);
        }
        ImGui::SameLine();
        if (ImGui::Button("What points here")) {
            unsigned a = 0;
            if (sscanf_s(s_address, "%x", &a) == 1)
                MemFind_PointersTo(a);
            else
                Log_Printf("Points: %s is not a hex address", s_address);
        }
        HelpMarker("Finds everything holding a pointer to that address, or to just before it. A hit inside "
                   "re5dx9.exe is a fixed address and the end of the chase; a hit on the heap is one link, "
                   "so put it in the box and go again.");
        static char s_base[32] = "FE4378";
        static int s_plus = 195;
        ImGui::SetNextItemWidth(140.0f);
        ImGui::InputText("re5dx9.exe+", s_base, sizeof(s_base));
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputInt("then plus", &s_plus);
        if (ImGui::Button("Follow")) {
            unsigned b = 0;
            if (sscanf_s(s_base, "%x", &b) == 1)
                MemFind_Follow(b, static_cast<unsigned>(s_plus < 0 ? 0 : s_plus));
            else
                Log_Printf("Follow: %s is not a hex offset", s_base);
        }
        HelpMarker("Reads the pointer at that fixed offset in the game, steps forward, and prints what it "
                   "finds - loaded, capacity and reserve if the route is right. This is the one that has to "
                   "still work after a relaunch.");
        HelpMarker("Type a number you can see in the game - rounds loaded, health, money - and press Find. "
                   "Change it in game, type the new number, press Narrow. Repeat until one address is left. "
                   "It only reads; nothing is written. The results go to the log.");
    }
#endif

    ImGui::SeparatorText("Theatre");
    changed |= ImGui::Checkbox("Watch menus and cutscenes on a screen", &s.stereo.theatre);
    HelpMarker("Menus, films and cutscenes play flat, on a screen hanging in front of you, with nothing "
               "hidden. They are composed for a rectangle, and a camera that cuts and swings is kinder "
               "watched than worn. Anything the game still shows you a HUD through - a pause screen, a "
               "chest, an inventory - is not a cutscene and stays where you are.");
    changed |= ImGui::Checkbox("Screen follows your head", &s.stereo.theatreFollowsHead);
    HelpMarker("Off, the screen hangs where you were facing and you can look around it, the way a screen "
               "in a room behaves. On, it is painted in front of your eyes and goes wherever you look.");
    {
        const bool held = StereoTest_TheatreHeld();
        if (ImGui::Button(held ? "Put the screen away" : "Put the screen up now"))
            StereoTest_ToggleTheatre();
        HelpMarker("Hold both sticks for three seconds to do this without opening the menu, or press F10. "
                   "The screen comes down on its own once your HUD is back and the camera is on you again.");
    }
    changed |= ImGui::SliderFloat("Screen size", &s.stereo.theatreScale, 0.3f, 1.0f, "%.2f");
    changed |= ImGui::SliderFloat("Screen distance", &s.stereo.theatreDistanceMeters, 0.8f, 12.0f, "%.1f m");
    HelpMarker("Distance sets how far apart the two eyes' copies sit, which is what your eyes converge "
               "on. Size is what makes it feel big. They are worth setting together.");
    if (ResetButton("theatre")) {
        s.stereo.theatre = g_defaults.stereo.theatre;
        s.stereo.theatreFollowsHead = g_defaults.stereo.theatreFollowsHead;
        s.stereo.theatreScale = g_defaults.stereo.theatreScale;
        s.stereo.theatreDistanceMeters = g_defaults.stereo.theatreDistanceMeters;
        changed = true;
    }

    if (ImGui::CollapsingHeader("Advanced")) {
        changed |= ImGui::SliderFloat("Head prediction", &s.vr.headPredictMs, 0.0f, 60.0f, "%.0f ms");
        HelpMarker("Pushes the view ahead while your head is turning, to hide pipeline delay. Does nothing once "
                   "you stop. 0 is off.");
        changed |= ImGui::SliderFloat("Head rotation gain", &s.vr.headRotationGain, 0.5f, 3.0f, "%.1fx");
        HelpMarker("1.0 is 1:1 with your neck. Anything else overshoots when you stop turning - most people "
                   "should leave this alone.");
        changed |= ImGui::SliderFloat("Extra FOV", &s.stereo.fovWiden, 0.5f, 2.0f, "%.2fx");
        changed |= ImGui::Checkbox("Match culling to the headset's FOV", &s.cam.vrMatchCullFov);
        HelpMarker("Tells the game to draw everything the headset can see. Off falls back to the game's 90 "
                   "degrees, and things at the edge of your view vanish.");
        changed |= ImGui::Checkbox("Mono post-processing (light-leak fix)", &s.stereo.monoSmallTargets);
        if (ResetButton("advanced")) {
            s.vr.headPredictMs = g_defaults.vr.headPredictMs;
            s.vr.headRotationGain = g_defaults.vr.headRotationGain;
            s.stereo.fovWiden = g_defaults.stereo.fovWiden;
            s.cam.vrMatchCullFov = g_defaults.cam.vrMatchCullFov;
            s.stereo.monoSmallTargets = g_defaults.stereo.monoSmallTargets;
            changed = true;
        }
    }
}

void DrawUpdateRow()
{
    const UpdateStatus u = UpdateCheck_GetStatus();
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextDisabled("Updates");
    ImGui::TableSetColumnIndex(1);
    switch (u.state) {
    case UpdateState::Off:
        ImGui::TextDisabled("not checked (turned off in the Menu tab)");
        break;
    case UpdateState::Checking:
        ImGui::TextUnformatted("checking Nexus...");
        break;
    case UpdateState::UpToDate:
        ImGui::TextColored(kGood, "up to date");
        break;
    case UpdateState::Available:
        ImGui::TextColored(kNotice, "v%s is on Nexus", u.latest.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Open Nexus page##status"))
            UpdateCheck_OpenNexusPage();
        break;
    case UpdateState::Failed:
        ImGui::TextDisabled("couldn't reach Nexus");
        break;
    }
}

// Above the tabs, so it's seen without digging through the Status tab.
void DrawUpdateBanner()
{
    const UpdateStatus u = UpdateCheck_GetStatus();
    if (u.state != UpdateState::Available)
        return;
    ImGui::TextColored(kNotice, "Update available: v%s (you have v%s)", u.latest.c_str(), RE5VR_VERSION);
    ImGui::SameLine();
    if (ImGui::SmallButton("Open Nexus page##banner"))
        UpdateCheck_OpenNexusPage();
}

void DrawStatusTab()
{
    VRBridgeStatus vr;
    VRBridge_GetStatus(vr);
    CameraRigStatus cam;
    CameraRigHook_GetStatus(cam);

    // What the mod costs, measured rather than reasoned about. The worst frame
    // of each second is the number that matters: an average hides one slow frame
    // among a hundred and twenty good ones, and one slow frame a second is
    // exactly what a stutter is.
    FrameTimesReport ft;
    if (FrameTimes_Get(ft)) {
        ImGui::SeparatorText("What the mod costs, last second");
        if (BeginStatusTable("cost")) {
            StatusRow("Our work per frame", "%.2f ms average, worst %.2f ms (%s)", ft.averageFrameMs,
                ft.worstFrameMs, FrameTimes_PhaseName(ft.worstPhase));
            for (int i = 0; i < kPhaseCount; ++i) {
                StatusRow(FrameTimes_PhaseName(i), "%.2f ms average, worst %.2f ms", ft.averagePhaseMs[i],
                    ft.worstPhaseMs[i]);
            }
            StatusRow("Frames counted", "%lu EndScene, %lu Present", ft.frames, ft.presents);
            StatusRow("Present to Present", "%.2f ms average, worst %.2f ms", ft.averagePresentGapMs,
                ft.worstPresentGapMs);
            ImGui::EndTable();
        }
    }

    ImGui::SeparatorText("Mod");
    if (BeginStatusTable("mod")) {
#if RE5VR_DIAGNOSTICS
        StatusRow("Build", "Developer v%s (diagnostics on)", RE5VR_VERSION);
#else
        StatusRow("Build", "Release v%s", RE5VR_VERSION);
#endif
        DrawUpdateRow();
        StatusRow("Install", vr.available ? "VR (dgVoodoo2 found)" : "flat screen (no dgVoodoo2)");
        MEMORYSTATUSEX mem = {};
        mem.dwLength = sizeof(mem);
        const bool laa = ExeIsLargeAddressAware();
        if (GlobalMemoryStatusEx(&mem)) {
            const double totalGb = mem.ullTotalVirtual / 1073741824.0;
            const double usedGb = (mem.ullTotalVirtual - mem.ullAvailVirtual) / 1073741824.0;
            StatusRow("Address space", "%.2f of %.1f GB used", usedGb, totalGb);
        }
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("4GB patch");
        ImGui::TableSetColumnIndex(1);
        if (laa)
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "applied");
        else
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.3f, 1), "NOT applied - expect stutters and crashes");
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Rendering");
    if (BeginStatusTable("render")) {
        StatusRow("Game frame rate", "%.0f fps (%.1f ms)", g_frameMsAvg > 0 ? 1000.0f / g_frameMsAvg : 0.0f, g_frameMsAvg);
        D3DSURFACE_DESC d = {};
        IDirect3DSurface9* bb = nullptr;
        if (g_device && SUCCEEDED(g_device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) {
            bb->GetDesc(&d);
            bb->Release();
        }
        StatusRow("Backbuffer", "%u x %u", d.Width, d.Height);
        StatusRow("Stereo", StereoTest_IsEnabled() ? "on - each eye gets %u x %u" : "off", d.Width / 2, d.Height);
        RenderSizeStatus rs;
        RenderSize_GetStatus(rs);
        if (!rs.available)
            StatusRow("Render size", "game's own (full resolution per eye unavailable for this exe)");
        else if (rs.active)
            StatusRow("Render size", "%u x %u for VR (game's own %u x %u)", rs.vrWidth, rs.vrHeight, rs.gameWidth,
                rs.gameHeight);
        else if (rs.pending)
            StatusRow("Render size", "switching to %u x %u", rs.vrWidth, rs.vrHeight);
        else
            StatusRow("Render size", rs.failed ? "game's own (RE5 refused the VR size)" : "game's own");
        int hudVs = 0, hudPs = 0;
        unsigned seen = 0;
        HudShaders_GetCounts(&hudVs, &hudPs, &seen);
        StatusRow("HUD shaders", "%d of 2 vertex, %d of 1 pixel recognised (%u shaders seen)", hudVs, hudPs, seen);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("VR");
    if (BeginStatusTable("vr")) {
        StatusRow("State", !vr.available ? "unavailable" : vr.sessionRunning ? "running" : vr.modeEnabled ? "starting" : "off");
        if (vr.runtimeName[0])
            StatusRow("Runtime", "%s", vr.runtimeName);
        if (vr.systemName[0])
            StatusRow("Headset", "%s", vr.systemName);
        if (vr.recommendedEyeWidth)
            StatusRow("Runtime wants", "%u x %u per eye", vr.recommendedEyeWidth, vr.recommendedEyeHeight);
        if (vr.eyeWidth)
            StatusRow("We send", "%u x %u per eye%s", vr.eyeWidth, vr.eyeHeight,
                vr.recommendedEyeWidth > vr.eyeWidth ? " (upscaled by the runtime)" : "");
        if (vr.sessionRunning) {
            char controllers[160] = "";
            XrInput_DescribeStatus(controllers, sizeof(controllers));
            XINPUT_GAMEPAD motionPad = {};
            StatusRow("Motion controllers", "%s%s", controllers, XrInput_GetPad(&motionPad) ? " (in hand)" : "");
            const float pacedHz = vr.predictedDisplayPeriodMs > 0 ? 1000.0f / vr.predictedDisplayPeriodMs : 0.0f;
            if (vr.runtimeHalvingUs)
                StatusRow("Headset refresh", "%.0f Hz, but your runtime is only pacing us at %.0f Hz",
                    vr.requestedRefreshHz, pacedHz);
            else
                StatusRow("Headset refresh", "%.0f Hz", pacedHz);
            StatusRow("Frames submitted", "%.0f / s", vr.submitHz);
            StatusRow("Image age at submit", "%.1f ms", vr.imageAgeMs);
        }
        ImGui::EndTable();
    }
    // The single biggest cause of head-turn judder found so far (2026-09-17),
    // and nothing in the game or the mod can fix it from this side.
    if (vr.sessionRunning && vr.runtimeHalvingUs) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.3f, 1.0f));
        ImGui::TextWrapped("Your VR runtime is running us at a lower rate than the headset and inventing the frames "
                           "in between. We already draw a frame for every one the headset shows, so those invented "
                           "frames replace real ones, and they can only guess at how your head moved: that is what "
                           "makes turning your head judder. Turn off Synchronous Spacewarp in Virtual Desktop "
                           "(Streaming tab), or ASW / motion smoothing in the Oculus or SteamVR settings.");
        ImGui::PopStyleColor();
    }

    ImGui::SeparatorText("Camera");
    if (BeginStatusTable("cam")) {
        StatusRow("First person", CameraRigHook_IsEnabled() ? "on" : "off");
        static const char* const kPlayers[] = { "not identified yet", "Chris", "Sheva", "someone else" };
        if (cam.player)
            StatusRow("Player", "%s (%d joints)", kPlayers[cam.player], cam.playerJointCount);
        else
            StatusRow("Player", "%s", kPlayers[0]);
        if (cam.cameraHookAgeMs == ~0ull)
            StatusRow("Camera hook", "not seen yet");
        else if (cam.cameraHookAgeMs < 100)
            StatusRow("Camera hook", "live");
        else
            StatusRow("Camera hook", "quiet for %.1f s", cam.cameraHookAgeMs / 1000.0f);
        StatusRow("Culling follows head", cam.headFollowDriving ? "yes, right now" : "not right now");
        StatusRow("Action cameras", "%lu cut-away(s), %lu watchdog restore(s), %lu flicker lockout(s)",
            cam.headCutaways, cam.watchdogRestores, cam.flickerLockouts);
        ImGui::EndTable();
    }
}

void DrawMenuTab(AllSettings& s, bool& changed, bool& resetAll)
{
    ImGui::SeparatorText("Menu");
    changed |= ImGui::SliderFloat("Text size", &s.menu.uiScale, 0.75f, 2.0f, "%.2fx");
    changed |= ImGui::Checkbox("Open with a click of both sticks", &s.menu.padChord);
    changed |= ImGui::Checkbox("Show the menu hint at startup", &s.menu.startupHint);
    if (ImGui::Checkbox("Check Nexus for updates at startup", &s.menu.checkForUpdates)) {
        changed = true;
        if (s.menu.checkForUpdates)
            UpdateCheck_Start();
        else
            UpdateCheck_SetOff();
    }
    HelpMarker("Asks nexusmods.com for this mod's latest version number once per launch. Nothing about you or your "
               "game is sent. The answer shows at the top of this menu and in the Status tab.");
    if (s.menu.checkForUpdates) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Check now"))
            UpdateCheck_Start();
    }

    ImGui::SeparatorText("Controls");
    ImGui::BulletText("Insert, or click both sticks: open / close this menu");
    ImGui::BulletText("Hold both sticks in for a second: reset VR view");
    ImGui::BulletText("Pad: d-pad or left stick to move, A to select, B to close");
    ImGui::BulletText("LB / RB: switch tabs; on a slider, A then d-pad left / right");
    ImGui::BulletText("Keyboard: arrows to move, Space to select, Esc to close");
    ImGui::BulletText("On a slider: Space then arrows, or Ctrl+click to type a value");

    ImGui::SeparatorText("Settings");
    ImGui::TextWrapped("Changes save automatically to %s", g_iniPath);
    if (ImGui::Button("Reset everything to defaults"))
        ImGui::OpenPopup("confirm reset");
    if (ImGui::BeginPopupModal("confirm reset", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Put every option back to its default?");
        if (ImGui::Button("Reset")) {
            resetAll = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

// One path for this now: the button and the stick gesture both start the same
// countdown, and the menu gets out of the way so the figure is visible.
void DrawArmCalibration()
{
    if (ImGui::Button("Calibrate arms (T-pose)")) {
        StartTPoseCalibration();
        OpenMenu(false);
    }
    ImGui::SameLine();
    ImGui::TextDisabled(ArmIk_IsCalibrated() ? "calibrated" : "not calibrated");
    HelpMarker("Also on a hold of both sticks, which recentres at the same time. Stand with your arms "
               "straight out to the sides and your palms down. It measures where your shoulders are and "
               "how long each of your arms is, sizes the character to match, and takes the angle your "
               "controllers are at as level - so aiming does not need a trigger pull to find it.");
}

#if RE5VR_DIAGNOSTICS
void DrawDeveloperTab(AllSettings& s, bool& changed)
{
    ImGui::TextDisabled("Developer builds only. None of these are saved.");
    bool stereo = StereoTest_IsEnabled();
    if (ImGui::Checkbox("Stereo without VR (was F8)", &stereo))
        StereoTest_SetEnabled(stereo);
    // Picture turning was a five-way experiment. Double, culling fixed won and
    // is now the only one (2026-09-18) - see StereoTest_ApplySettings. Head
    // steadiness, on the VR tab, is what is left to tune here.
    ImGui::TextDisabled("Picture turning: double, culling fixed (no longer selectable)");
    changed |= ImGui::Checkbox("Direct submit (was Delete)", &s.vr.directSubmit);
    changed |= ImGui::Checkbox("Producer waits for consumer (was Insert)", &s.vr.waitForConsumer);
    {
        static const char* const kCommitModes[] = {
            "Off - the partner sees you frozen",
            "Moved flag only",
            "Step handler only",
            "Both - the old switch, dumps the magazine",
            "Both, then put the flag back",
        };
        changed |= ImGui::Combo("Co-op aim-walk commit", &s.cam.aimWalkCommit, kCommitModes, 5);
        changed |= ImGui::Checkbox("Aim-walk in third person too", &s.cam.aimWalkThirdPerson);
        HelpMarker("Normally this only works in first person. Turn it on and drop to third person to watch "
                   "the character from outside, which is the only way to see whether the legs walk or the "
                   "body just slides along the floor.");
        changed |= ImGui::SliderFloat("Commit rate", &s.cam.aimWalkCommitHz, 0.0f, 60.0f, "%.0f a second");
        HelpMarker("How often the step may be committed. 0 is every frame, which is what caused the "
                   "teleporting: the game does this about 25 times a second and at frame rate we were "
                   "doing it five times too often. Lower it until the teleporting stops, then tell me "
                   "whether your partner still sees you walk.");
        HelpMarker("How a step taken while aiming is committed so a co-op partner sees it. This used to be "
                   "one switch doing two things at once, which is why the magazine dump was never "
                   "explained. In a co-op session, work down the list and say for each one whether your "
                   "partner sees you walk, and whether the gun fires more than once a pull or fires with "
                   "no ammo. The last entry is the one I expect to do both jobs.");
    }

    // ---- Arms across the network ------------------------------------------
    ImGui::SeparatorText("Share your arms with a co-op partner");
    {
        // The switch itself is on the VR tab now, under Co-op. What stays
        // here is everything needed to argue with it when it will not work.
        ImGui::TextDisabled("The switch is on the VR tab, under Co-op. It is currently %s.",
            s.ikSync.enabled ? "ON" : "OFF");
        HelpMarker("Swaps hand positions with your co-op partner, so a VR player's arms do what their arms "
                   "are really doing instead of what the animation says. It carries nothing but hand and "
                   "shoulder positions, it is not the game's own netcode, and it cannot desync a session "
                   "because it only changes how the other character is DRAWN on your machine. Both of you "
                   "need this build.");

        // Steam leads, because Steam IS the route now (2026-09-19). It is
        // addressed by SteamID, so there is nothing to type and no NAT to get
        // through. The address and port are what remains of the UDP attempt,
        // and leaving them at the top read as though they were still the way
        // in - the user said so: "this should be updated to reflect the current
        // steamID partner and sending status".
        unsigned long long steamPartner = 0;
        unsigned steamSent = 0, steamGot = 0;
        if (IkSync_GetSteamStatus(&steamPartner, &steamSent, &steamGot)) {
            if (steamPartner) {
                ImGui::Text("Steam partner: %llu", steamPartner);
                ImGui::Text("Sending %u a second, receiving %u", steamSent, steamGot);
                if (steamSent && !steamGot)
                    ImGui::TextDisabled("Nothing back yet - are they on this build with it switched on?");
            } else {
                ImGui::TextUnformatted("Steam ready. Waiting for the game to speak to a partner.");
            }
        } else {
            ImGui::TextDisabled("Steam route unavailable - an address is the only way in.");
        }

        if (ImGui::TreeNode("Fallback: talk over an address instead")) {
            // Enter to commit, or every keystroke tears the socket down and
            // builds it again - nine restarts to type an address, each one a
            // chance to land on a bind failure (2026-09-19).
            changed |= ImGui::InputText("Partner's address", s.ikSync.partnerIp, sizeof(s.ikSync.partnerIp),
                ImGuiInputTextFlags_EnterReturnsTrue);
            HelpMarker("Only needed when Steam cannot be used. Press Enter to apply. 127.0.0.1 talks to "
                       "yourself, which tests everything except the network. Two home routers usually will "
                       "not let this through, which is exactly why Steam is the route.");
            changed |= ImGui::InputInt("Port", &s.ikSync.port);
            changed |= ImGui::Checkbox("Test on myself", &s.ikSync.loopback);
            HelpMarker("For testing without a second player. Your own hands are packed up exactly as they would "
                       "be sent, held back a tenth of a second the way a network would, and fed back in as "
                       "though your partner had sent them - so your partner's arms follow yours. Nothing goes "
                       "on the wire, so it proves everything except that Steam delivers it.");
            char netStatus[160];
            IkSync_DescribeStatus(netStatus, sizeof(netStatus));
            ImGui::Text("Link: %s", netStatus);
            ImGui::TreePop();
        }
        IkSyncHands hands;
        if (IkSync_GetPartnerHands(hands)) {
            ImGui::Text("Their right hand: %+.2f %+.2f %+.2f m from their head", hands.handFromHead[1][0],
                hands.handFromHead[1][1], hands.handFromHead[1][2]);
        }

        // Every step between a packet landing and an elbow moving, because
        // "their arms are not moving" was true of five different faults and
        // the panel could not tell them apart (2026-09-20).
        ArmIkPartnerStatus partner;
        ArmIk_GetPartnerStatus(partner);
        const auto step = [](const char* label, bool ok, const char* whyNot) {
            if (ok)
                ImGui::Text("%s: yes", label);
            else
                ImGui::TextDisabled("%s: no - %s", label, whyNot);
        };
        step("Partner on screen", partner.haveBody, "nobody else is being drawn");
        step("Their arms found", partner.armsFound, "could not make out two arms on that skeleton");
        step("Their hands arriving", partner.handsFresh, "no hands in the last half second");
        step("They have calibrated", partner.haveScale, "they must T-pose, or their reach has no scale");
        step("Tied to their skeleton", partner.tied, "waiting for the first usable hand");
        step("Driving their arms", partner.solving, "the solve is not running");
    }

    // The switch itself is on the VR tab now. What is left here is its
    // workings, for when they need to be argued with.
    ImGui::SeparatorText("3DOF aiming (the workings)");
    changed |= ImGui::Checkbox("Point the gun with the controller", &s.xrInput.pointToAim);
    HelpMarker("The gun's up and down angle follows where the controller points. Left and right is still the "
               "stick.");
    ImGui::BeginDisabled(!s.xrInput.pointToAim);
    changed |= ImGui::SliderFloat("Aim trim", &s.xrInput.aimPitchTrimDeg, -30.0f, 30.0f, "%.0f deg");
    // Only the two that work are offered. The three that wrote game values
    // all turned out to be downstream of the aim and moved nothing; they stay
    // in the code with their findings, reachable by setting
    // VR.MotionAimWriteField in re5vr.ini, but a tester shouldn't be asked to
    // tell four broken options apart.
    {
        static const char* const kAimMethods[] = { "Stick: gun only, capped at the game's turn rate",
            "Mouse: gun and body, no cap" };
        int method = s.xrInput.aimWriteField == 5 ? 1 : 0;
        if (ImGui::Combo("How to aim", &method, kAimMethods, 2)) {
            s.xrInput.aimWriteField = method == 1 ? 5 : 0;
            changed = true;
        }
        HelpMarker("Stick drives the game's own pad, so it can never turn faster than the game does, which is why "
                   "it feels sluggish. Mouse has no such ceiling, but the game switches to keyboard and mouse "
                   "prompts and the controllers become WASD and clicks.");
    }
    changed |= ImGui::Checkbox("Left-handed (swap the gun hand)", &s.xrInput.swapHands);
    ImGui::EndDisabled();
    // Outside the Point to aim gate on purpose: the aim rate is the game's own,
    // and both of these work with an ordinary pad on the desktop.
    if (s.xrInput.aimRateDrive) {
        changed |= ImGui::SliderFloat("Top aim speed", &s.xrInput.aimMaxRateDeg, 90.0f, 1200.0f, "%.0f deg/s");
        HelpMarker("The fastest the gun may turn. A wrist flick runs 150 to 300 degrees a second, so anything under "
                   "that will cap your fast turns and make you exaggerate them. RE5 on its own manages 86.");
    } else {
        changed |= ImGui::SliderFloat("Aim speed", &s.xrInput.aimSpeedMult, 1.0f, 8.0f, "%.1fx");
        HelpMarker("Multiplies the game's own aim speed, past what its fastest setting allows. 1.0x is RE5 as it "
                   "ships, where the gun turns 86 degrees a second at most, which is less than half a wrist flick. "
                   "Your sensitivity setting is not changed: the original goes back when this returns to 1.0x.");
    }
    changed |= ImGui::SliderFloat("Aim smoothing", &s.xrInput.aimServoMs, 20.0f, 250.0f, "%.0f ms");
    HelpMarker("How long the gun takes to close the gap to where the controller points. Lower is tighter and quicker "
               "to snap, higher is smoother and lazier. Only affects pointing with the controller, not the stick.");
    changed |= ImGui::SliderFloat("Hand steadiness", &s.xrInput.aimSteadiness, 0.0f, 1.0f, "%.2f");
    HelpMarker("How much of your own small hand movement reaches the gun. 0 passes everything, including the shimmer "
               "that is the tracking rather than your hand. 1 holds the gun as still as a stick you have let go of, "
               "which reads as a mounted turret rather than a person aiming.");
    changed |= ImGui::Checkbox("Set the speed, not the stick", &s.xrInput.aimRateDrive);
    HelpMarker("Holds the stick at a fixed push for direction only and sets how fast the gun turns by writing the "
               "game's own aim speed every frame. Those have no dead zone, so slow pointing glides instead of "
               "stepping. Off goes back to asking for speed with stick deflection, where the dead zone sets the "
               "slowest the gun can move.");
    ImGui::BeginDisabled(s.xrInput.aimRateDrive);
    changed |= ImGui::SliderFloat("Stick dead zone", &s.xrInput.aimStickDeadzone, 0.0f, 0.6f, "%.2f");
    HelpMarker("How much of the stick RE5 throws away around centre. Every push the servo makes is lifted clear of "
               "this, so gentle movement is not swallowed. Too low and small movements do nothing; too high and the "
               "gun creeps when you hold still. Raise it until slow pointing responds, then back off.");
    ImGui::EndDisabled();
    changed |= ImGui::Checkbox("Show the aim numbers over the game", &s.xrInput.aimOverlay);
    HelpMarker("Draws what the servo is doing on screen with the menu closed, so the dials above can be tuned with "
               "the gun up in the headset. The top line is the one that matters: how far the gun is off your hand.");
    changed |= ImGui::Checkbox("Find what drives the aim angle", &s.xrInput.findAimWriter);
    changed |= ImGui::Checkbox("Point with the body (absolute yaw)", &s.xrInput.absoluteYaw);
    HelpMarker("Turns the character to the heading your controller points at, instead of steering towards it. Tied "
               "to where you are pointing each time the gun comes up, which is also how you recentre.");
    changed |= ImGui::Checkbox("Find the body facing", &s.xrInput.findBodyFacing);
    changed |= ImGui::Checkbox("Widen culling always (costs frames)", &s.xrInput.wideCullAlways);
    HelpMarker("Pushes the culling frustum out on every camera the game builds, instead of only when it looks like "
               "a cutscene. A test: if what is missing comes back with this on, the frustum was the right lever and "
               "the cutscene detection was wrong.");
    changed |= ImGui::Checkbox("Find the culling planes", &s.xrInput.findCullPlanes);
    changed |= ImGui::Checkbox("Find the arms", &s.xrInput.findArms);
    HelpMarker("Works out which joints are the shoulders, elbows and wrists from the shape of the skeleton, and "
               "reports them with their bone lengths. The first step of arm IK.");
    changed |= ImGui::Checkbox("Find the gun's skeleton", &s.xrInput.findGunBones);
    HelpMarker("Looks for the held weapon's own bones and logs them, then which ones move while you fire. Best "
               "in VR with a gun in hand: aim a few seconds without firing, then fire a few bursts.");
    changed |= ImGui::Checkbox("Measure the firing shake", &s.xrInput.measureShake);
    HelpMarker("With 'Your hands drive the arms' OFF, aim and fire a few shots, then lower the gun; aim once "
               "without firing too. The log names the joints that move most while the gun is up.");
    changed |= ImGui::Checkbox("Measure two-handed grips", &s.xrInput.measureGrips);
    HelpMarker("With 'Your hands drive the arms' OFF, aim each weapon for a couple of seconds. Every time the gun "
               "goes down, the log says where the game's aiming animation put the off hand on that gun.");
    ImGui::Separator();
    ImGui::TextUnformatted("Arms (6DOF)");
    changed |= ImGui::Checkbox("Your hands drive the arms", &s.xrInput.armIk);
    HelpMarker("Puts the character's hands where you are holding the controllers, and bends the two bones above "
               "each one to match. Needs the arms to have been found, which happens on its own. What it does NOT "
               "do yet is turn the gun: the shot still goes where 3DOF aiming sends it.");
    changed |= ImGui::SliderFloat("How much of the way", &s.xrInput.armIkWeight, 0.0f, 1.0f, "%.2f");
    HelpMarker("How far from the animation towards your real hand each arm is moved. 1 is all the way. Lower "
               "leaves some of the game's own pose showing, which is a way to see the two apart.");
    changed |= ImGui::SliderFloat("Your arms against theirs", &s.xrInput.armIkScale, 0.5f, 2.0f, "%.2f");
    HelpMarker("Above 1 reaches further for the same real movement. The scale is taken from the character's own "
               "arm against a real one, so this is only here for when that guess does not suit you.");
    HelpMarker("Sets each joint's own rotation, which is what the game builds the character from, instead of "
               "overwriting the result afterwards. The hand, the fingers and anything held then follow on "
               "their own. Off moves the arm on screen and leaves everything it is carrying behind.");
    changed |= ImGui::Checkbox("Write into the skeleton build", &s.xrInput.armIkLateWrite);
    HelpMarker("Moves the arms at the end of the game building the pose instead of after it, so the hand is "
               "already where you are holding it when the weapon is hung off it. Off leaves the arms "
               "following and the gun behind. Takes effect on the next launch.");
    changed |= ImGui::Checkbox("Drive the pose, not the result", &s.xrInput.armIkWriteLocal);
    HelpMarker("Sets each joint's own rotation, which is what the game builds the character from, instead "
               "of overwriting the result afterwards. Held items then follow the hand. A release build ties "
               "this to the 6DOF checkbox; here it is separate so it can be taken away on its own.");
    changed |= ImGui::Checkbox("Turn the gun with your hand", &s.xrInput.armIkWrist);
    HelpMarker("Takes the forearm's direction and roll from your controller rather than from where your "
               "hand happens to be, which is what turns the gun over. Tied to the pose each time the gun "
               "comes up.");
    changed |= ImGui::Checkbox("Hold the arms out (write test)", &s.xrInput.armIkTest);
    HelpMarker("Ignores the controllers and holds both arms straight out in front. The one test that tells a "
               "write that never lands apart from a write that lands somewhere wrong: if the arms do not move, "
               "the animation is rebuilding the skeleton after we set it.");
    changed |= ImGui::Checkbox("Find what sets the arm pose", &s.xrInput.findArmWriter);
    HelpMarker("Watchpoints the right wrist's world matrix and logs every instruction that reads or writes it. "
               "The writers are the animation building the pose; a reader is whatever places the weapon the "
               "hand is holding. Very heavy for a few seconds: it suspends every thread to arm, then traps on "
               "every access.");
    HelpMarker("Watchpoints a live camera's frustum planes to find the routine that builds them, which is where "
               "culling can be widened for everything at once. Heavy: it suspends every thread to arm.");
    HelpMarker("Searches the character for whatever holds which way the body faces, by keeping only the values that "
               "still match after you turn. Aim, then turn a long way several times.");
    HelpMarker("Arms a hardware watchpoint on the aim pitch for a few seconds the next time you aim, and logs which "
               "instructions wrote it. Aim and sweep the right stick while it runs. Heavy: it suspends every thread "
               "to arm.");
}
#endif

void DrawMenu(const Layout& L)
{
    AllSettings s = CaptureSettings();
    bool changed = false, resetAll = false;

    if (L.stereo) {
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(L.unitsW, L.unitsH));
    } else {
        const float k = L.fontPx / 18.0f;
        ImGui::SetNextWindowSize(ImVec2(560 * k, 640 * k), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(L.unitsW * 0.5f, L.unitsH * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    }
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    if (L.stereo)
        flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool keepOpen = true;
    if (ImGui::Begin(kTitle, &keepOpen, flags)) {
        DrawUpdateBanner();
        if (ImGui::BeginTabBar("tabs")) {
            int index = 0;
            // Each tab gets its own scrolling area (2026-09-18). Tab contents used
            // to sit straight in the window, which only scrolls if the window can -
            // and in VR it is a fixed size that cannot be resized, so a long tab
            // simply ran off the bottom with no way to reach what was down there.
            // The scrollbar is always shown rather than only when needed, so it is
            // obvious there is more below instead of something to discover.
            const auto tab = [&](const char* name) {
                const bool open = ImGui::BeginTabItem(name, nullptr, TabFlags(index));
                if (open) {
                    g_currentTab = index;
                    ImGui::BeginChild("tabbody", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None,
                        ImGuiWindowFlags_AlwaysVerticalScrollbar);
                }
                ++index;
                return open;
            };
            const auto endTab = [] {
                ImGui::EndChild();
                ImGui::EndTabItem();
            };
            if (tab("Camera")) {
                DrawCameraTab(s, changed);
                endTab();
            }
            if (tab("VR")) {
                DrawVrTab(s, changed);
                endTab();
            }
            if (tab("Status")) {
                DrawStatusTab();
                endTab();
            }
            if (tab("Menu")) {
                DrawMenuTab(s, changed, resetAll);
                endTab();
            }
#if RE5VR_DIAGNOSTICS
            if (tab("Developer")) {
                DrawDeveloperTab(s, changed);
                endTab();
            }
#endif
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    g_selectTab = -1;

    if (resetAll) {
        s = g_defaults;
        changed = true;
        Log_Printf("Menu: everything reset to defaults");
    }
    if (changed) {
        ApplySettings(s);
        g_dirty = true;
        g_lastChangeMs = GetTickCount64();
    }
    if (!keepOpen)
        g_open = false;
}

// The menu's own mouse pointer. ImGui's built-in software cursor never showed
// in gameplay (2026-09-13: the log had the pointer moving and hovering while
// the user saw nothing - on menus they had only ever been looking at the
// game's Windows cursor, which RE5 hides once you are playing). Drawn as plain
// polygons in the foreground list, so it needs nothing from the font atlas and
// lands in both eyes in VR like the rest of the menu.
// The game's own pointer graphic, copied into a texture so the menu's
// pointer looks the same everywhere, gameplay and VR included (user request,
// 2026-09-13). Windows cursors carry their transparency as a mask or an alpha
// channel; drawing the cursor once over black and once over white recovers
// both kinds the same way: alpha = 255 - (white - black), colour = black / alpha.
IDirect3DTexture9* g_cursorTex = nullptr;
HCURSOR g_cursorTexFor = nullptr;
int g_cursorW = 0, g_cursorH = 0, g_cursorHotX = 0, g_cursorHotY = 0;

// Only a cursor the game itself set, and never one of Windows' stock cursors:
// the first version fell back to the window class cursor, which at startup is
// the plain Windows arrow (handle 00010003), and locked that in before RE5's
// own pointer ever appeared (user, 2026-09-13: "shows the windows cursor").
bool IsSystemCursor(HCURSOR cursor)
{
    static const LPCSTR kIds[] = { IDC_ARROW, IDC_IBEAM, IDC_WAIT, IDC_CROSS, IDC_UPARROW, IDC_SIZENWSE, IDC_SIZENESW,
        IDC_SIZEWE, IDC_SIZENS, IDC_SIZEALL, IDC_NO, IDC_HAND, IDC_APPSTARTING, IDC_HELP };
    for (LPCSTR id : kIds)
        if (LoadCursorA(nullptr, id) == cursor)
            return true;
    return false;
}

HCURSOR GameCursorHandle()
{
    const HCURSOR c = InputBlock_GameCursor();
    return c && !IsSystemCursor(c) ? c : nullptr;
}

bool BuildCursorTexture(IDirect3DDevice9* dev, HCURSOR cursor)
{
    ICONINFO ii = {};
    if (!GetIconInfo(cursor, &ii))
        return false;
    BITMAP bm = {};
    int w = 0, h = 0;
    if (ii.hbmColor && GetObjectA(ii.hbmColor, sizeof(bm), &bm)) {
        w = bm.bmWidth;
        h = bm.bmHeight;
    } else if (ii.hbmMask && GetObjectA(ii.hbmMask, sizeof(bm), &bm)) {
        w = bm.bmWidth;
        h = bm.bmHeight / 2; // monochrome cursors stack AND and XOR masks
    }
    const int hotX = static_cast<int>(ii.xHotspot), hotY = static_cast<int>(ii.yHotspot);
    if (ii.hbmColor)
        DeleteObject(ii.hbmColor);
    if (ii.hbmMask)
        DeleteObject(ii.hbmMask);
    if (w <= 0 || h <= 0 || w > 256 || h > 256)
        return false;

    HDC screen = GetDC(nullptr);
    HDC mem = CreateCompatibleDC(screen);
    ReleaseDC(nullptr, screen);
    BITMAPINFO bi = {};
    bi.bmiHeader.biSize = sizeof(bi.bmiHeader);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h; // top-down
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;
    std::vector<BYTE> onBlack, onWhite;
    for (int pass = 0; pass < 2; ++pass) {
        void* bits = nullptr;
        HBITMAP dib = CreateDIBSection(mem, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        if (!dib || !bits) {
            DeleteDC(mem);
            return false;
        }
        HGDIOBJ old = SelectObject(mem, dib);
        std::memset(bits, pass ? 0xFF : 0x00, static_cast<size_t>(w) * h * 4);
        DrawIconEx(mem, 0, 0, cursor, w, h, 0, nullptr, DI_NORMAL);
        (pass ? onWhite : onBlack).assign(static_cast<BYTE*>(bits), static_cast<BYTE*>(bits) + static_cast<size_t>(w) * h * 4);
        SelectObject(mem, old);
        DeleteObject(dib);
    }
    DeleteDC(mem);

    IDirect3DTexture9* tex = nullptr;
    if (FAILED(dev->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &tex, nullptr)) || !tex)
        return false;
    D3DLOCKED_RECT lr;
    if (FAILED(tex->LockRect(0, &lr, nullptr, 0))) {
        tex->Release();
        return false;
    }
    int opaque = 0;
    for (int y = 0; y < h; ++y) {
        auto* row = static_cast<BYTE*>(lr.pBits) + y * lr.Pitch;
        for (int x = 0; x < w; ++x) {
            const BYTE* b = &onBlack[(y * w + x) * 4];
            const BYTE* wt = &onWhite[(y * w + x) * 4];
            int spread = 0;
            for (int c = 0; c < 3; ++c)
                spread = (std::max)(spread, wt[c] - b[c]);
            const int a = 255 - (std::min)(255, (std::max)(0, spread));
            for (int c = 0; c < 3; ++c)
                row[x * 4 + c] = a ? static_cast<BYTE>((std::min)(255, b[c] * 255 / a)) : 0;
            row[x * 4 + 3] = static_cast<BYTE>(a);
            if (a > 32)
                ++opaque;
        }
    }
    tex->UnlockRect(0);

    // In gameplay RE5 hides its pointer by setting a fully transparent cursor
    // (log, 2026-09-13: handle 082C0DCB, 32x32, and the menu pointer vanished
    // in game). A graphic with next to nothing visible is never adopted.
    if (opaque < 8) {
        tex->Release();
        Log_Printf("Menu: game cursor %p is blank (%d visible pixel(s)) - keeping the pointer we have", cursor, opaque);
        return false;
    }

    if (g_cursorTex)
        g_cursorTex->Release();
    g_cursorTex = tex;
    g_cursorTexFor = cursor;
    g_cursorW = w;
    g_cursorH = h;
    g_cursorHotX = hotX;
    g_cursorHotY = hotY;
    Log_Printf("Menu: pointer graphic taken from the game's cursor %p (%dx%d, hotspot %d,%d)", cursor, w, h, hotX, hotY);
    return true;
}

// Captures the game's menu pointer graphic ONCE, the first time it shows a
// visible one - normally on the title screen, before our menu is ever opened.
// Kept from then on: the game swaps handles (menu pointer, a brief variant, a
// blank one in gameplay) and the user wants the menu pointer look everywhere.
void CaptureGameCursorOnce()
{
    if (g_cursorTex || !g_device)
        return;
    const HCURSOR cursor = GameCursorHandle();
    static HCURSOR s_tried[8] = {};
    if (!cursor)
        return;
    for (HCURSOR t : s_tried)
        if (t == cursor)
            return;
    for (HCURSOR& t : s_tried) {
        if (!t) {
            t = cursor;
            break;
        }
    }
    BuildCursorTexture(g_device, cursor);
}

void DrawPointer(const Layout& L)
{
    const ImGuiIO& io = ImGui::GetIO();
    if (!ImGui::IsMousePosValid(&io.MousePos))
        return;

    // The beam (2026-09-23, user: "I have a cursor on the menu to be clear,
    // just not a laser drawn to the cursor position, easier to justify where
    // your controller is pointing"). From where the controller appears on
    // screen to where it is pointing. It is a flat line rather than a thing in
    // the world, but it starts at your hand and ends at the cursor, and that
    // is the whole of what makes a laser readable: you can see it coming
    // before it arrives.
    if (g_haveLaserFrom) {
        ImDrawList* beam = ImGui::GetForegroundDrawList();
        const ImVec2 from(g_laserFromX, g_laserFromY);
        const ImVec2 to(io.MousePos.x, io.MousePos.y);
        const float wide = (std::max)(1.0f, L.fontPx / 9.0f);
        // A dark line under a bright one, so it reads on any background.
        beam->AddLine(from, to, IM_COL32(0, 0, 0, 90), wide * 2.0f);
        beam->AddLine(from, to, IM_COL32(255, 70, 60, 150), wide);
        beam->AddCircleFilled(to, wide * 1.8f, IM_COL32(255, 120, 110, 200));
    }

    if (g_cursorTex) {
        // Native size at 1080p, growing with the text beyond that.
        const float s = (std::max)(1.0f, L.fontPx / 20.0f);
        const ImVec2 p0(io.MousePos.x - g_cursorHotX * s, io.MousePos.y - g_cursorHotY * s);
        const ImVec2 p1(p0.x + g_cursorW * s, p0.y + g_cursorH * s);
        ImGui::GetForegroundDrawList()->AddImage(reinterpret_cast<ImTextureID>(g_cursorTex), p0, p1);
        return;
    }

    // Fallback until the game has shown a cursor: a plain drawn arrow.
    const float k = (std::max)(1.0f, L.fontPx / 16.0f);
    static const ImVec2 kArrow[] = { { 0, 0 }, { 0, 17 }, { 4, 13 }, { 7, 20 }, { 10, 19 }, { 7, 12 }, { 12, 12 } };
    ImVec2 pts[_countof(kArrow)], shadow[_countof(kArrow)];
    for (size_t i = 0; i < _countof(kArrow); ++i) {
        pts[i] = ImVec2(io.MousePos.x + kArrow[i].x * k, io.MousePos.y + kArrow[i].y * k);
        shadow[i] = ImVec2(pts[i].x + 1.5f * k, pts[i].y + 1.5f * k);
    }
    // The concave fill left ragged edges (user screenshot, 2026-09-13).
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    // The notch makes the arrow concave, so it is filled as three triangles
    // and the tail quad, with fill anti-aliasing off so the pieces don't leave
    // faint seams where they meet. The outline on top keeps the edge smooth.
    const auto fill = [&](const ImVec2* v, ImU32 col) {
        dl->AddTriangleFilled(v[0], v[1], v[2], col);
        dl->AddTriangleFilled(v[0], v[2], v[5], col);
        dl->AddTriangleFilled(v[0], v[5], v[6], col);
        const ImVec2 tail[] = { v[2], v[3], v[4], v[5] };
        dl->AddConvexPolyFilled(tail, _countof(tail), col);
    };
    const ImDrawListFlags flags = dl->Flags;
    dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
    fill(shadow, IM_COL32(0, 0, 0, 90));
    fill(pts, IM_COL32_WHITE);
    dl->Flags = flags;
    dl->AddPolyline(pts, _countof(kArrow), IM_COL32_BLACK, ImDrawFlags_Closed, (std::max)(1.0f, 1.2f * k));
}

// A costume change takes your calibration with it, and nothing said so
// (2026-09-25, user: "any time a costume change happens, can we have a message
// pop up"). The arms simply stopped fitting, and the only explanation was in a
// log file nobody is reading while wearing a headset.
//
// Centred and high so it is unmissable, and it names the gesture rather than
// sending anyone to a menu - which is the same gesture that has always started
// the T-pose countdown, so there is nothing new to learn and nothing new to
// bind.
void DrawCostumeNotice(const Layout& L, float alpha)
{
    const float pad = L.fontPx;
    ImGui::SetNextWindowPos(ImVec2(L.unitsW * 0.5f, L.stereo ? L.unitsH * 0.22f : pad), ImGuiCond_Always,
        ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.85f * alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::Begin("##costume", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted("Costume changed, new T-pose calibration required.");
    ImGui::TextDisabled("Hold left stick and right stick to calibrate.");
    ImGui::End();
    ImGui::PopStyleVar();
}

void DrawHint(const Layout& L, float alpha, float secondsLeft)
{
    const float pad = L.fontPx;
    ImGui::SetNextWindowPos(ImVec2(L.stereo ? L.unitsW * 0.5f : pad, pad), ImGuiCond_Always,
        ImVec2(L.stereo ? 0.5f : 0.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.85f * alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::Begin("##hint", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted(kTitle);
    ImGui::TextDisabled(g_prefs.padChord ? "Insert or click both sticks for the menu" : "Insert for the menu");
    ImGui::TextDisabled("This hides in %d seconds", static_cast<int>(std::ceil((std::max)(0.0f, secondsLeft))));
    ImGui::End();
    ImGui::PopStyleVar();
}

#if RE5VR_MOTION_AIM_DEV
// The aim tuning readout (2026-09-17). Three dials that interact, and the one
// place you can feel them is with the gun up in the headset, where the menu is
// necessarily closed - so the numbers have to come to you. Centred and high,
// clear of the reticle, out of the way of anything you would shoot at.
void DrawAimOverlay(const Layout& L)
{
    AimServoStatus a{};
    if (!CameraRigHook_GetAimServoStatus(a))
        return;
    const float pad = L.fontPx;
    ImGui::SetNextWindowPos(ImVec2(L.unitsW * 0.5f, L.stereo ? L.unitsH * 0.2f : pad), ImGuiCond_Always,
        ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.7f);
    ImGui::Begin("##aimtune", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    // The one number that matters: where the gun is against where you point.
    // Green while it is close enough to hit what you mean.
    const float off = std::fabs(a.errorDeg);
    ImGui::PushStyleColor(ImGuiCol_Text,
        off < 2.0f ? ImVec4(0.4f, 1.0f, 0.4f, 1.0f)
                   : off < 6.0f ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f));
    ImGui::Text("gun is %+.1f deg off your hand", a.errorDeg);
    ImGui::PopStyleColor();
    ImGui::TextDisabled("hand %.0f deg/s up-down, %.0f deg/s left-right", a.wristPitchRate, a.wristYawRate);
    ImGui::TextDisabled("stick asked %+.2f, %+.2f   ceiling %.0f deg/s", a.stickY, a.stickX, a.maxRateDeg);
    ImGui::TextDisabled("owed left-right %+.1f deg   fastest seen %.0f deg/s", a.yawDebtDeg, a.fastestSeenDeg);
    // The pair that says whether the game is capping us: if the gun's own rate
    // stops climbing while the scale keeps going up, something past these
    // floats is holding it and more speed will not help.
    ImGui::TextDisabled("gun turning %.0f deg/s   speed written x%.1f up-down, x%.1f left-right", a.gunYawRate,
        a.pitchScale, a.yawScale);
    ImGui::End();
}
#endif

// Top-right corner on the monitor; in VR, centred under where the startup
// hint sits, since a corner of the frame is out of view in the headset.
void DrawUpdateNotice(const Layout& L, float alpha, const std::string& latest)
{
    const float pad = L.fontPx;
    if (L.stereo)
        ImGui::SetNextWindowPos(ImVec2(L.unitsW * 0.5f, pad * 6.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    else
        ImGui::SetNextWindowPos(ImVec2(L.unitsW - pad, pad), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.85f * alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::Begin("##update", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextColored(kNotice, "TrueFP/VR update available: v%s (you have v%s)", latest.c_str(), RE5VR_VERSION);
    ImGui::TextDisabled(g_prefs.padChord ? "Open the menu (Insert or click both sticks) for the Nexus link"
                                         : "Open the menu (Insert) for the Nexus link");
    ImGui::End();
    ImGui::PopStyleVar();
}

bool EnsureImGui(IDirect3DDevice9* device)
{
    if (g_imguiReady)
        return true;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // our settings live in re5vr.ini; window layout isn't worth a file
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad;
    io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
    if (!ImGui_ImplWin32_Init(g_hwnd) || !ImGui_ImplDX9_Init(device)) {
        Log_Printf("Menu: ImGui backend init failed - no menu this session");
        ImGui::DestroyContext();
        return false;
    }
    g_imguiReady = true;
    Log_Printf("Menu: Dear ImGui %s ready", IMGUI_VERSION);
    return true;
}

void OpenMenu(bool open)
{
    if (open == g_open)
        return;
    g_open = open;
}

} // namespace

void Menu_Install(IDirect3DDevice9* device)
{
    g_device = device;
    QueryPerformanceFrequency(&g_qpcFreq);

    D3DDEVICE_CREATION_PARAMETERS cp = {};
    if (SUCCEEDED(device->GetCreationParameters(&cp)))
        g_hwnd = cp.hFocusWindow;
    if (!g_hwnd) {
        IDirect3DSwapChain9* sc = nullptr;
        if (SUCCEEDED(device->GetSwapChain(0, &sc)) && sc) {
            D3DPRESENT_PARAMETERS pp = {};
            if (SUCCEEDED(sc->GetPresentParameters(&pp)))
                g_hwnd = pp.hDeviceWindow;
            sc->Release();
        }
    }
    if (g_hwnd) {
        g_gameWndProc = reinterpret_cast<WNDPROC>(
            SetWindowLongPtrA(g_hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&MenuWndProc)));
    }
    Log_Printf("Menu: game window %p, window procedure %s", g_hwnd, g_gameWndProc ? "hooked" : "NOT hooked");
    InputBlock_SetWindow(g_hwnd);
    InputBlock_SetMessageSink(&QueueMessage);

    // re5vr.ini next to the game's exe.
    GetModuleFileNameA(nullptr, g_iniPath, MAX_PATH);
    char* slash = strrchr(g_iniPath, '\\');
    if (slash)
        strcpy_s(slash + 1, MAX_PATH - (slash + 1 - g_iniPath), "re5vr.ini");

    // Everything as the code ships it, before the file has a say - this is
    // what the Reset buttons go back to.
    g_defaults = CaptureSettings();
    AllSettings s = g_defaults;
    const bool haveFile = GetFileAttributesA(g_iniPath) != INVALID_FILE_ATTRIBUTES;
    if (haveFile) {
        LoadSettings(s);
        ApplySettings(s);
        Log_Printf("Menu: settings loaded from %s", g_iniPath);
        // Rewrite straight away: drops keys no longer in the list and adds new
        // ones at their defaults, without waiting for the player to change something.
        SaveSettings(CaptureSettings());
    } else {
        Log_Printf("Menu: no %s yet - defaults, written on the first change", g_iniPath);
    }
    g_savedSnapshot = Serialize(CaptureSettings());
}

bool Menu_IsDrawing()
{
    return g_drawing.load(std::memory_order_acquire);
}

void Menu_OnBeforeReset()
{
    if (g_imguiReady)
        ImGui_ImplDX9_InvalidateDeviceObjects();
}

void Menu_OnAfterReset()
{
    // Device objects are recreated lazily by the next NewFrame.
}

void Menu_OnPresent(IDirect3DDevice9* device)
{
    if (!g_hwnd || device != g_device)
        return;
    const ULONGLONG nowMs = GetTickCount64();
    if (!g_firstPresentMs)
        g_firstPresentMs = nowMs;

    // Frame time, smoothed.
    LARGE_INTEGER qpc;
    QueryPerformanceCounter(&qpc);
    if (g_lastPresentQpc.QuadPart) {
        const float ms = static_cast<float>(qpc.QuadPart - g_lastPresentQpc.QuadPart) * 1000.0f /
            static_cast<float>(g_qpcFreq.QuadPart);
        g_frameMsAvg = g_frameMsAvg > 0 ? g_frameMsAvg + (ms - g_frameMsAvg) * 0.05f : ms;
    }
    const float deltaSec = g_lastPresentQpc.QuadPart
        ? static_cast<float>(qpc.QuadPart - g_lastPresentQpc.QuadPart) / static_cast<float>(g_qpcFreq.QuadPart)
        : 1.0f / 60.0f;
    g_lastPresentQpc = qpc;

    // Start in VR, once the game has had a few seconds to get going.
    if (!g_autoVrDone && nowMs - g_firstPresentMs > 5000) {
        g_autoVrDone = true;
        if (g_prefs.checkForUpdates)
            UpdateCheck_Start();
        if (g_prefs.autoStartVr && VRBridge_IsAvailable()) {
            Log_Printf("Menu: starting VR automatically (StartInVR=1)");
            VRBridge_RequestXrMode(true);
        }
    }

    // ---- Open / close ----
    const bool wasOpen = g_open;
    static bool s_prevInsert = false;
    const bool insert = GameIsForeground() && (GetAsyncKeyState(VK_INSERT) & 0x8000) != 0;
    if (insert && !s_prevInsert)
        OpenMenu(!g_open);
    s_prevInsert = insert;

    // F10 does the same from the keyboard, for anyone playing on a monitor or
    // with the headset pushed up. The controller hold below is the one that
    // matters in VR.
    {
        static bool s_prevF10 = false;
        const bool f10 = GameIsForeground() && (GetAsyncKeyState(VK_F10) & 0x8000) != 0;
        if (f10 && !s_prevF10)
            StereoTest_ToggleTheatre();
        s_prevF10 = f10;
    }

    XINPUT_STATE pad = {};
    const bool havePad = InputBlock_ReadPad(&pad);
    const WORD buttons = havePad ? pad.Gamepad.wButtons : 0;
    {
        // Both sticks: a click toggles the menu, a one-second hold resets the
        // VR view (once per hold). Decided on release, so a hold never flashes
        // the menu open first. The click used to have to be under half a
        // second, which people reported as the shortcut simply not working
        // (2026-09-15); anything short of the recenter hold now counts.
        constexpr WORD kBoth = XINPUT_GAMEPAD_LEFT_THUMB | XINPUT_GAMEPAD_RIGHT_THUMB;
        constexpr ULONGLONG kRecenterHoldMs = 1000;
        // LEFT GRIP AND BOTH STICKS (2026-09-25, the user's own suggestion after
        // "nah thats too confusing" to three meanings on one hold - which was
        // right, nobody remembers a gesture that changes meaning by how long you
        // keep it).
        //
        // Holding the grip makes it a different chord rather than a longer one,
        // so the T-pose is not on the way past: with the grip down, this hold
        // does not open the menu and does not start the countdown. Half a second
        // is enough, because the grip already makes it deliberate.
        static ULONGLONG s_chordSinceMs = 0;
        static bool s_recentered = false;
        static bool s_theatred = false;
        const bool both = (buttons & kBoth) == kBoth;
        const bool gripped = both && XrInput_GripHeld(0);
        if (gripped && !s_theatred && s_chordSinceMs && nowMs - s_chordSinceMs >= 500) {
            s_theatred = true;
            if (g_calFiresAtMs) {
                g_calFiresAtMs = 0;
                Log_Printf("Menu: T-pose countdown dropped - the left grip was down, so that was the screen");
            }
            StereoTest_ToggleTheatre();
        }
        if (both) {
            if (!s_chordSinceMs)
                s_chordSinceMs = nowMs;
            if (!gripped && !s_theatred && !s_recentered && nowMs - s_chordSinceMs >= kRecenterHoldMs
                && CameraRigHook_IsVrActive()) {
                s_recentered = true;
                // Merged with the recentre (2026-09-19): one gesture, and the
                // recentre happens when the countdown fires rather than now, so
                // both are taken from the same pose. Holding again cancels.
                if (g_calFiresAtMs) {
                    g_calFiresAtMs = 0;
                    Log_Printf("Menu: T-pose calibration cancelled");
                } else {
                    StartTPoseCalibration();
                }
            }
        } else if (s_chordSinceMs) {
            const ULONGLONG heldMs = nowMs - s_chordSinceMs;
            const bool opens = !s_recentered && !s_theatred && heldMs < kRecenterHoldMs && g_prefs.padChord;
            Log_Printf("Menu: both sticks released after %llu ms - %s", heldMs,
                s_recentered ? "the T-pose countdown was started"
                             : (!g_prefs.padChord ? "the shortcut is off in the menu"
                                                  : (opens ? "opening/closing the menu" : "held too long, ignored")));
            if (opens)
                OpenMenu(!g_open);
            s_chordSinceMs = 0;
            s_recentered = false;
            s_theatred = false;
        }
    }
    static WORD s_prevButtons = 0;
    const WORD pressed = buttons & ~s_prevButtons;
    s_prevButtons = buttons;

    if (g_open && wasOpen && g_imguiReady) {
        // B or Escape closes, unless it is busy cancelling something in the menu.
        const bool busy = ImGui::IsAnyItemActive() || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId);
        const bool esc = GameIsForeground() && (GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0;
        static bool s_prevEsc = false;
        if (!busy && (((pressed & XINPUT_GAMEPAD_B) != 0) || (esc && !s_prevEsc)))
            g_open = false;
        s_prevEsc = esc;
        if (pressed & XINPUT_GAMEPAD_LEFT_SHOULDER)
            g_selectTab = (std::max)(0, g_currentTab - 1);
        if (pressed & XINPUT_GAMEPAD_RIGHT_SHOULDER)
            g_selectTab = g_currentTab + 1;
    }

    if (g_open != wasOpen) {
        InputBlock_SetBlocking(g_open);
        Log_Printf("Menu: %s", g_open ? "opened" : "closed");
        g_cursorX = g_cursorY = -1.0f; // recentred below once the layout is known
        g_sentX = g_sentY = g_lastRealX = g_lastRealY = -1e9f;
    }

    // ---- Save, a second after the last change or when the menu closes ----
    if (g_dirty && (!g_open || nowMs - g_lastChangeMs > 1000)) {
        g_dirty = false;
        const AllSettings now = CaptureSettings();
        const std::string snapshot = Serialize(now);
        if (snapshot != g_savedSnapshot) {
            SaveSettings(now);
            g_savedSnapshot = snapshot;
        }
    }

    // The hint's clock only runs while the game is in front and presenting
    // real frames: the first presents are black loading frames nobody sees,
    // and the first build burned the whole hint on those.
    static ULONGLONG s_hintStartMs = 0;
    if (!s_hintStartMs && deltaSec < 0.1f && GameIsForeground())
        s_hintStartMs = nowMs;
    const float hintAgeSec = s_hintStartMs ? (nowMs - s_hintStartMs) / 1000.0f : 0.0f;
    constexpr float kHintSec = 10.0f;
    const bool showHint = g_prefs.startupHint && hintAgeSec < kHintSec && !g_open;

    // Update notice: once per launch, when the Nexus check finds a newer
    // version. Same rule as the hint for when its clock starts. Opening the
    // menu ends it; the menu shows the same news with the link.
    const UpdateStatus update = UpdateCheck_GetStatus();
    static ULONGLONG s_noticeStartMs = 0;
    static bool s_noticeDone = false;
    constexpr float kNoticeSec = 15.0f;
    if (!s_noticeStartMs && !s_noticeDone && update.state == UpdateState::Available && deltaSec < 0.1f &&
        GameIsForeground()) {
        s_noticeStartMs = nowMs;
        Log_Printf("Menu: showing the update notice (Nexus has v%s)", update.latest.c_str());
    }
    const float noticeAgeSec = s_noticeStartMs ? (nowMs - s_noticeStartMs) / 1000.0f : 0.0f;
    if (s_noticeStartMs && !s_noticeDone && (g_open || noticeAgeSec >= kNoticeSec))
        s_noticeDone = true;
    const bool showNotice = s_noticeStartMs && !s_noticeDone;

    UpdateDesktopView(device);
    // Every frame until it has one - the title screen is where it shows first.
    CaptureGameCursorOnce();
#if RE5VR_MOTION_AIM_DEV
    const bool showAimTuning = XrInput_GetSettings().aimOverlay;
#else
    const bool showAimTuning = false;
#endif
    // The T-pose countdown counts as something to draw (2026-09-19). It is shown
    // with the menu CLOSED by design - you are meant to be standing in the pose,
    // not reading a menu - so leaving it out of this test meant the countdown ran
    // its full five seconds and fired with nothing ever appearing on screen.
    // The costume prompt stands until they do it, or until they have had a good
    // look at it. Starting the countdown counts as having read it.
    constexpr float kCostumeSec = 15.0f;
    const unsigned long long costumeMs = ArmIk_CostumeChangedMs();
    const float costumeAgeSec = costumeMs ? (nowMs - costumeMs) / 1000.0f : 0.0f;
    if (costumeMs && (g_calFiresAtMs || costumeAgeSec >= kCostumeSec))
        ArmIk_ClearCostumeChanged();
    const bool showCostume = costumeMs && !g_calFiresAtMs && costumeAgeSec < kCostumeSec;
    if (!g_open && !showHint && !showNotice && !showAimTuning && !g_calFiresAtMs && !showCostume)
        return;
    if (device->TestCooperativeLevel() != D3D_OK)
        return; // lost device: nothing can be drawn until the game resets it

    IDirect3DSurface9* bb = nullptr;
    D3DSURFACE_DESC desc = {};
    if (FAILED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;
    bb->GetDesc(&desc);
    bb->Release();
    if (desc.Width < 64 || desc.Height < 64)
        return;

    if (!EnsureImGui(device))
        return;
    const Layout L = ComputeLayout(desc.Width, desc.Height);
    EnsureFont(L.fontPx);

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(L.unitsW, L.unitsH);
    io.DeltaTime = (std::max)(1.0f / 1000.0f, (std::min)(deltaSec, 0.25f));
    io.MouseDrawCursor = false; // we draw our own - see DrawPointer
    if (g_cursorX < 0.0f) {
        g_cursorX = L.unitsW * 0.5f;
        g_cursorY = L.unitsH * 0.5f;
    }
    if (g_open) {
        FeedMouseAndKeys(io, L);
        FeedPad(io, havePad ? &pad : nullptr);
    } else {
        FeedPad(io, nullptr);
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
    }

    const bool openBeforeDraw = g_open;
    ImGui_ImplDX9_NewFrame();
    ImGui::NewFrame();
    if (g_open) {
        DrawMenu(L);
        DrawPointer(L);
    } else {
        if (showHint)
            DrawHint(L, (std::min)(1.0f, (kHintSec - hintAgeSec) / 1.0f), kHintSec - hintAgeSec);
        if (showNotice)
            DrawUpdateNotice(L, (std::min)(1.0f, kNoticeSec - noticeAgeSec), update.latest);
    }
#if RE5VR_MOTION_AIM_DEV
    // Over everything, menu open or not: the dials being tuned are in the menu
    // and the thing they change can only be felt with it closed.
    if (showAimTuning)
        DrawAimOverlay(L);
#endif
    // Over everything, menu open or not: you are meant to be standing up and
    // getting ready, not reading a menu.
    if (showCostume)
        DrawCostumeNotice(L, (std::min)(1.0f, (kCostumeSec - costumeAgeSec) / 1.5f));
    // Over everything, menu open or not.
    if (g_calFiresAtMs) {
        const ULONGLONG nowCal = GetTickCount64();
        if (nowCal >= g_calFiresAtMs) {
            g_calFiresAtMs = 0;
            VRBridge_RequestRecenter("T-pose calibration");
            ArmIk_Calibrate();
        } else {
            DrawTPoseOverlay(static_cast<float>(g_calFiresAtMs - nowCal) / 1000.0f);
        }
    }
    ImGui::Render();
    RenderToBackbuffer(device, L);

    // The close button on the window.
    if (openBeforeDraw && !g_open) {
        InputBlock_SetBlocking(false);
        Log_Printf("Menu: closed");
    }
}
