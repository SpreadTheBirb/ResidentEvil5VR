#pragma once

#include <windows.h>

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
// includeReads asks for reads as well as writes. x86 has no read-only
// condition, so a read watch reports the writers too and the already-known
// ones have to be subtracted by hand.
void AimFinder_Start(void* address, const char* what, bool includeReads = false);

// FOUR AT ONCE (2026-09-28, user: "so each time I press end, one of them will
// fix it?").
//
// x86 has four debug registers and we were using one, which turned a single
// question into four stomps and four windows. Up to four addresses can be
// watched in the same window, and the report says which of them each
// instruction touched. One press, one melee, the whole answer.
void AimFinder_StartMany(void* const* addresses, const char* const* names, int count,
    bool includeReads = false);

// Reports a finished window. Call once per frame from the EndScene hook.
void AimFinder_OnEndScene();

// The instruction that wrote most often in the finished window, with the
// registers it held at its first hit. False until a window has finished.
// Chasing a value to its source needs these: the write that lands on the
// character is a copy, and the pointer it copied FROM is sitting in a register.
struct AimFinderHit {
    DWORD eip, eax, ecx, edx, ebx, esp, ebp, esi, edi, count;
};
bool AimFinder_BusiestHit(AimFinderHit& out);

// Throws the last window away so another can run. Without this one window per
// session is all you get, and following a chain needs several.
void AimFinder_Rearm();
