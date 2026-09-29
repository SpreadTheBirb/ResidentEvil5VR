#pragma once

// Forces RE5's laser sight on when aiming with the mouse (the game normally
// shows it only with a gamepad), by applying the same three code bytes as the
// community trainer's laser-sight toggle. See laser_patch.cpp. Call once the
// game's code is unpacked (from Hooks_OnDeviceCreated). Idempotent.
void LaserPatch_Install();

// TURNING IT BACK OFF (2026-09-25, user: "I installed the QOL Fixes .dll from
// a mod ... the laser is also still misaligned").
//
// This was a one-way patch, applied at startup and never reconsidered, which
// was fine while nothing else in the process had an opinion about the laser.
// Another mod with its own laser setting is a second opinion, and two mods
// writing the same three jumps is not a thing to leave undiagnosable. So the
// original bytes are kept and it goes both ways.
//
// False if the game's bytes did not match, in which case the option is dead.
bool LaserPatch_IsAvailable();
bool LaserPatch_IsForcedOn();
void LaserPatch_SetForcedOn(bool on);
