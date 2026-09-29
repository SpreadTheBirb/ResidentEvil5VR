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

// Where the player's character is standing, in the game's world units. Taken
// from where he STANDS rather than from any bone, so the animation's bob and
// lean are not in it. False when there is no player yet.
bool CameraRigHook_GetPlayerWorldPos(float out[3]);

// Where the camera actually rendered from last frame, decoded from the shader
// constants rather than read out of any game structure. Used to hunt for the
// field the camera object keeps its own position in: whatever tracks this is
// that field.
bool CameraRigHook_LastCameraPosition(float out[3]);

// The running lead currently being applied to the eye, in world units. It is
// part of where the eye is DRAWN and no part of where the body is, so anything
// asking "has the camera left the body" has to take it back off first.
void CameraRigHook_GetRunLead(float out[3]);

// Once per PRESENTED frame, not once per EndScene. See PinCameraToBody: this
// game calls EndScene about thirty-four times for every frame it shows, and
// the camera must be placed once, not thirty-four times at arbitrary points
// inside the game's own update.
void CameraRigHook_OnPresent();

// True while the OpenXR session is delivering frames (polled from the
// render thread every 100 ms). Safe to call from any thread.
bool CameraRigHook_IsVrActive();

// True while the game has taken the camera away from us: a cutscene, a vault,
// a stomp, a scripted action. The same signal the head watchdog runs on - the
// camera hook has gone quiet, which only happens when the game's own camera
// code is driving. Two things need it: the culling fix does not reach the
// game's own cameras, and a cutscene has to be skippable by a controller that
// has no Start button of its own.
bool CameraRigHook_InScriptedCamera();

// Throw away which joints the arms are and find them again. For when the
// numbers coming off them stop making sense, which means they are not the
// arms any more: a new character, a new mode, a reloaded level.
void CameraRigHook_ForgetArms();

// Where the eye belongs inside the character's body right now, in world
// space: the neck (or head) plus the usual up and ahead offsets, worked out
// from the skeleton rather than from the camera rig - which stops running the
// moment the game takes the camera for a kick or a vault. False when there is
// no head to use, when the setting is off, or when the game's camera is far
// enough away to be a deliberate shot rather than an action.
bool CameraRigHook_GetBodyEye(const float forward[3], const float camPos[3], float outEye[3]);

// What the aim servo is doing right now, for the tuning readout the menu draws
// over the game (developer). Aiming in VR means the menu is closed, so the
// numbers have to reach the headset some other way. False when the servo has
// not run in the last quarter second.
struct AimServoStatus {
    float controllerPitchDeg, rigPitchDeg, errorDeg;
    float wristPitchRate, wristYawRate; // deg/sec, as the servo sees them
    float stickX, stickY;               // what it is asking the pad for, -1..1
    float maxRateDeg;                   // what full stick buys at the current multiplier
    float fastestSeenDeg;               // fastest the gun has actually been turned
    float yawDebtDeg;                   // degrees the wrist has turned that the gun has not
    float gunYawRate;                   // what the gun's bearing is actually managing, deg/sec
    float pitchScale, yawScale;         // what the game's own speeds are being multiplied by
};
bool CameraRigHook_GetAimServoStatus(AimServoStatus& out);

// 3DOF aiming: the right-stick push the aim servo wants this frame, -1..1, so
// the gun's pitch catches up with where the controller points. False when it
// has nothing to ask for. Applied to the virtual pad the game reads, because
// the game re-derives its own aim pitch from stick input every frame and
// ignores anything written straight into it.
bool CameraRigHook_GetAimStick(float* x, float* y);

// 3DOF aiming through the game's own mouse: the movement the aim servo wants,
// in mouse counts, taken once and cleared. A mouse has no turn-rate ceiling,
// unlike the stick, so this is what makes pointing keep up with a wrist.
bool CameraRigHook_TakeAimMouse(long* dx, long* dy);

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
    // 0 off. Holds the GAME CAMERA against the fraction of a degree a head
    // moves while its owner is talking or breathing. What you see through the
    // lenses is untouched - see HeadSteadier in camera_rig_hook.cpp.
    float vrHeadSteady = 0.35f;        // VR: hold the camera against head micro movement
    // VR: how many game ticks of the character's own movement the eye is led
    // by when the frame is drawn, so it stays in his head at a run instead of
    // landing where his head was a tick ago. Position only, level, no lean.
    // Four, and the slider goes to eight (2026-09-25, user: "I've been setting
    // running lead to 4.0 - that's about the only solve for your body clipping
    // into the camera when sprinting").
    //
    // Four was the top of the slider, and two was the top of it before that,
    // and both times the answer was the top. A default nobody keeps is not a
    // default, so it becomes the tested value, with real headroom above it.
    //
    // Worth saying plainly though: if eight also turns out to be the answer,
    // the number is not the problem. A flat multiple of one tick of movement
    // cannot be right at every speed, and the honest fix would be to lead by
    // how fast you are actually going rather than by a constant somebody has
    // to find on a slider.
    float vrRunLead = 4.0f;
    float vrViewSteady = 0.7f;
    // The eye rides the NECK bone rather than the head (2026-09-23, user:
    // "it needs locked to his neck bone instead of the eyes, and just raised a
    // bit to match eye level"). The head bone nods, rocks when he fires and
    // leads when he runs; the neck is where a head is carried from.
    bool vrEyeOnNeck = true;
    float vrEyeAboveNeck = 20.0f; // units above the neck bone, about 23 cm
    bool vrMatchCullFov = true;        // VR: cull with the headset's own FOV
    float vrCullMarginPct = 15.0f;     // VR: extra culling angle on top of the headset's FOV
    float vrCullTurnLookaheadMs = 0.0f; // VR: widen culling by how far the head turns in this long (developer)
    // Off for now (2026-09-18): it causes more trouble than it solves while the
    // arm work is in flight, and wants debugging on its own rather than as a
    // variable in someone else's test.
    bool headLock = false;             // keep the view on the head even when the game takes the camera
    float headLockReach = 2500.0f;     // how far the camera may stray and still be brought back, game units
    float viewTurnLimitDeg = 450.0f;   // fastest the game may swing your view, 0 for no limit
    bool headLockDirection = true;     // while the game drives, face where the character faces, not where its camera does
    bool showHeadDuringActions = true; // head pops back in when the game's camera leaves you
    // HOLD YOUR VIEW WHILE THE GAME PLAYS AN ACTION (2026-09-28).
    //
    // Stomps, uppercuts, door kicks and vaults all end at the same function:
    // the game tells the camera where to be and where to look, and whatever
    // we had is gone. This keeps the position - the camera still rides your
    // body through the animation, so the move still reads - and refuses the
    // rotation, which is the part that hurts.
    //
    // "If I jump a gap, the movement from camera should be minimal, if any.
    // Enough to sell the effect of jumping, but anything intense causes
    // discomfort."
    // On, and under a NEW ini key (2026-09-28). The old HoldViewInMelee has
    // been both true and false in people\x27s settings files during a day of
    // this being wrong, and a saved value from a build that behaved
    // differently is worth nothing. RefuseMeleeCamera starts fresh.
    bool holdViewInMelee = true;
    // Holding partner locate currently swings the view onto Sheva, which in
    // a headset is somebody grabbing your head and turning it. exe+7743C8 is
    // the line that does it; zeroing what it writes keeps the locate icons
    // and leaves the camera where you left it.
    bool partnerNoCamera = true;
    // A scope in a headset points into one eye and fills the screen, which
    // is unusable. RE5 already knows how to keep a scoped weapon in third
    // person - it does it between shots with the S75 - and exe+75AF53 is the
    // check that decides. Forcing it keeps every scope out where you can
    // actually aim it. Off by default because it changes how those weapons
    // handle on a flat screen too.
    bool scopeThirdPerson = false;
    // Developer: how an aim-walk step is committed so a co-op partner sees it.
    // An AimWalkCommit, kept as an int. See the enum in camera_rig_hook.cpp -
    // this was a bool, and being a bool is why the co-op fault has gone
    // unexplained since 2026-09-11: it welded two separate actions together.
    // Defaults to 4, kCommitTidy, from 2026-09-19. It was 0 because the only
    // setting that synced the move also dumped the magazine, so shipping it on
    // was out of the question. Splitting the switch found the one that does
    // the first without the second, a co-op session confirmed it, and with
    // that there is no reason for a partner to watch you stand frozen every
    // time you aim.
    int aimWalkCommit = 4;
    // How many times a second the step may be committed. The position itself
    // is still written every frame - that is what makes walking smooth - but
    // the COMMIT is the game's own once-per-step ritual and running it at
    // frame rate is running it about five times too often.
    // ZERO, MEANING EVERY FRAME (2026-09-29, user: "0ms on the timing").
    //
    // 25 a second was a guess made while the teleporting was being chased and
    // the mode below was still wrong. With commit mode 4 - both, then put the
    // flag back - the throttle is what stops a co-op partner seeing the walk
    // at all, so it comes off. Neither of these is saved to the ini, so this
    // default is what every build runs.
    float aimWalkCommitHz = 0.0f;
    // Let walking-while-aiming work in third person too. It is first-person
    // only by default because that is the mode it exists for, but you cannot
    // judge your own legs from inside your own head: to see whether the walk
    // animation plays or the character slides, you have to watch from outside.
    bool aimWalkThirdPerson = false;
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
