#pragma once

// Steady hands: removes the aim tremor RE5 applies while the gun is up.
//
// On a gamepad the shake is a difficulty knob. In a headset it is noise: your
// hands are not shaking, and the arm IK is trying to hold the weapon exactly
// where your controller is while the game adds a wobble on top.
//
// Four byte patches, all reversible, all verified against the expected
// original bytes before anything is written. See tremor_patch.cpp for where
// they came from and what each one does.

// Checks the four sites against this game build. Call once at startup, after
// the module is loaded.
void TremorPatch_Install();

// False when the bytes did not match, in which case SetOn does nothing and
// the menu should say so rather than offering a switch that cannot work.
bool TremorPatch_IsAvailable();

bool TremorPatch_IsOn();
void TremorPatch_SetOn(bool on);
