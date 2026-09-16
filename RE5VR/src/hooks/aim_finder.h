#pragma once

// Who writes the aim pitch (2026-09-16).
//
// 3DOF aiming needs to set the gun's pitch at its source. Writing the
// character's pitch field directly does nothing: the game re-derives it every
// frame, so the value read back is unchanged no matter what goes in. Driving
// it through the stick works but is capped by the game's own aim rate,
// measured at 86 degrees a second with sensitivity at maximum, where a wrist
// flick is more than twice that. So it has to be set where it is computed,
// and that means finding the instruction that computes it.
//
// Same method as the F5 boom finder (hooks/boom_finder.cpp): a hardware
// write watchpoint on the live address for a few seconds, recording which
// instructions wrote it, then a report in the log. Off unless re5vr.ini asks
// for it (VR.MotionAimFindWriter=1), since arming debug registers across
// every thread is not something to do on a player's machine.

// Arms a 4 byte write watchpoint for a few seconds. Safe to call every frame:
// it does nothing while a window is already running, or after one has run.
void AimFinder_Start(void* address, const char* what);

// Reports a finished window. Call once per frame from the EndScene hook.
void AimFinder_OnEndScene();
