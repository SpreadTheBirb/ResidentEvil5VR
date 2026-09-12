#pragma once

// Forces RE5's laser sight on when aiming with the mouse (the game normally
// shows it only with a gamepad), by applying the same three code bytes as the
// community trainer's laser-sight toggle. See laser_patch.cpp. Call once the
// game's code is unpacked (from Hooks_OnDeviceCreated). Idempotent.
void LaserPatch_Install();
