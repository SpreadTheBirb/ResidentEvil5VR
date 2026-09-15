#pragma once

// Phase 4: forces RE5's camera-rig structs (Horizontal/Vertical Distance,
// Distance, Horizontal/Vertical Adjustment, Angle, Field of View) to a
// fixed set of values every frame, collapsing the over-the-shoulder rig
// down toward the character's head for a rough first-person view. Two
// hooks, both confirmed working in-game (2026-07-27):
//   - "Normal" (universal non-aiming profile - RE5 has no holster
//     mechanic, so the earlier separate "Hand to Hand" hook was retired
//     in favor of treating Normal as the one profile that matters).
//   - "Aim Camera" (active whenever aim is actually held, not merely
//     from having a weapon equipped).
// Both apply the identical 7 values and share the same F4 toggle.
//
// Struct layout was found via a live Cheat Engine session (2026-07-25).
// Both hook addresses needed correction after their first guessed
// address crashed the game on level load - the Normal hook via a
// Windows Error Reporting crash dump's fault offset (one byte off), the
// Aim Camera hook by abandoning extrapolation after a second guess also
// crashed and instead reading the real disassembly field-by-field via
// Cheat Engine's "find out what writes" (which surfaced real surprises,
// like the struct silently skipping offset +0x0C, that byte-counting
// could never have predicted). See project memory for the full struct
// map, both derivations, and the crash dumps.
//
// 2026-09-11: both hooks above are superseded by ONE hook at
// re5dx9.exe+446965, after every rig copy. Each mode actually has three
// rigs (level / look-up / look-down) blended by pitch; the fix moves all six
// eyes to the head and keeps only their view directions, which removed the
// pitch "boom". See the notes in camera_rig_hook.cpp.
//
// Toggle with F4. Values are hardcoded to what was found to work
// reasonably during the original 2026-07-25 session, and kept as the
// intended final values (RE5 has no holster mechanic, so there was no
// practical way to separately re-tune Normal vs. the old Hand-to-Hand
// state). Expect visible popping/clipping of the character's own head
// mesh when looking around, since RE5 has no real first-person mode
// that hides it. Likely to be addressed later by hiding the head mesh
// outright rather than chasing a perfect camera-only fix.
void CameraRigHook_Install();

// Call once per frame from the EndScene hook to poll the F4 toggle.
void CameraRigHook_OnEndScene();

// True while the F4 near-first-person override is active. Used by
// head_hide_probe.cpp to gate its head-mesh skip so hidden parts only
// stay hidden in first-person and automatically reappear in normal
// third-person play.
bool CameraRigHook_IsEnabled();

// Copies out the struct bases a live hook hit recorded within the last
// 250ms (see the lifetime notes in camera_rig_hook.cpp). Returns the count.
// Only valid right now - never keep these across frames.
int CameraRigHook_GetLiveBases(void** outBases, bool* outAim, int maxCount);

// The camera controller currently judged to be the player's (the one whose
// eye matches the rendered camera; only decided while F4 is on). An address
// for watchpoints - never dereference it outside the camera hook.
void* CameraRigHook_GetPlayerController();

// True while the OpenXR session is delivering frames (polled from the
// render thread every 100 ms). Safe to call from any thread.
bool CameraRigHook_IsVrActive();

// The vertical culling angle VR last used (headset FOV plus the margin), and
// whether it hit the ceiling. 0 before VR has run.
float CameraRigHook_GetVrCullFovDeg(bool* atCap);

// True while head tracking is actually steering the game's camera this frame
// (VR on, F9 on, gun down). When it is, the game camera ALREADY contains your
// head rotation, so stereo_test must not rotate the eye bases by the head
// delta a second time - see the head-follow compensation there.
bool CameraRigHook_HeadFollowDrivingCamera();

// What head-follow wrote this frame, for stereo_test's direct picture turning
// (2026-09-15): the world direction of each of the player's rigs (3 normal,
// then 3 aim - the game renders from one of them) and the head forward those
// were built from. False when head-follow isn't driving or nothing recent.
struct HeadFollowTargets {
    float worldDir[7][3];      // [6]: the one view direction, blended by the game's pitch factor
    float headForward[3];
    unsigned long long poseId; // the XR pose headForward came from
    float cameraForward[3];    // what the camera was actually turned by (headForward, or doubled - mode 3)
};
bool CameraRigHook_GetHeadFollowTargets(HeadFollowTargets& out);

// Twice the yaw and pitch of head forward f (picture turning mode 3).
void CameraRigHook_DoubleHeadAim(const float f[3], float out[3]);

// TEST (2026-09-15): while aiming, turn only the MAIN camera's copy of the
// view toward the head (hook after exe+4421BC), leaving the camera
// controller's own output - and anything that reads it - on the gun. Answers
// whether the gun reads the controller (laser stays put) or the main camera
// (laser follows the head). Not saved.
void CameraRigHook_SetAimViewTest(bool on);
bool CameraRigHook_GetAimViewTest();

// Which recent head-follow update the camera now pointing along camForward
// came from: the one whose written directions match it best (newest wins a
// tie). outAge is 0 for the latest update, 1 for the one before, and so on.
// The game can draw a frame from an older camera update than the newest one,
// so tagging with the newest would still claim the frame is fresher than it is.
bool CameraRigHook_MatchHeadFollowTargets(const float camForward[3], HeadFollowTargets& out, int* outAge, float* outErrDeg);

// ---- In-game menu (ui/menu.cpp) -----------------------------------------
// Every option the old F4/F9/F11/F6/'`'/','/'.' hotkeys changed. Apply clamps,
// logs what changed, and (for first person) toggles the fade patch with it.
struct CameraRigSettings {
    bool firstPerson = false;
    bool headFollow = true;            // VR: head turns the game camera while the gun is down
    bool vrStabilise = true;           // VR: smooth the idle-animation shake out of the eye
    bool vrMatchCullFov = true;        // VR: cull with the headset's own FOV
    float vrCullMarginPct = 15.0f;     // VR: extra culling angle on top of the headset's FOV
    float vrCullTurnLookaheadMs = 0.0f; // VR: widen culling by how far the head turns in this long (developer)
    bool showHeadDuringActions = true; // head pops back in when the game's camera leaves you
    bool aimWalkCommit = false;        // developer: co-op aim-walk sync test
    float flatFovDeg = 90.0f;
    float flatEyeUp = 0.9f, flatEyeAhead = -0.4f;
    float vrEyeUp = 1.0f, vrEyeAhead = 0.2f;
};
CameraRigSettings CameraRigHook_GetSettings();
void CameraRigHook_ApplySettings(const CameraRigSettings& s);
void CameraRigHook_SetFirstPerson(bool on);

struct CameraRigStatus {
    int player = 0;             // 0 not identified yet, 1 Chris, 2 Sheva, 3 someone else
    int playerJointCount = 0;
    unsigned long long cameraHookAgeMs = ~0ull; // since the player's camera last ran through our hook
    bool headFollowDriving = false;
    unsigned long headCutaways = 0, watchdogRestores = 0, flickerLockouts = 0;
};
void CameraRigHook_GetStatus(CameraRigStatus& out);
