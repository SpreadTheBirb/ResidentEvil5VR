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

// True while head tracking is actually steering the game's camera this frame
// (VR on, F9 on, gun down). When it is, the game camera ALREADY contains your
// head rotation, so stereo_test must not rotate the eye bases by the head
// delta a second time - see the head-follow compensation there.
bool CameraRigHook_HeadFollowDrivingCamera();
