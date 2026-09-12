#pragma once

// Diagnostic: '=' arms a snapshot that fires 3 s later (with a beep) of the
// game's global state, the heap objects it points to, and the player's
// character and camera controller, written to re5vr_state_N.bin - for
// diffing laser-sight (gamepad) against crosshair (mouse) aiming. See
// state_probe.cpp. Call once per frame from the EndScene hook.
void StateProbe_OnEndScene();
