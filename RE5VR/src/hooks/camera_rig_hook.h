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
