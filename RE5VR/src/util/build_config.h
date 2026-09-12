#pragma once

// Developer diagnostics and experiment switches. 0 for anything that leaves
// this machine.
//
// At 0 the probes are compiled out entirely (the F5 watchpoint finder, the
// '=' state capture, the skeleton/constant/pixel/head-hide probes, the Phase
// 0 debug quad) and so are the experiment hotkeys - Insert, Delete and '\'.
// That matters: Delete toggles the direct-submit path, which is now the
// DEFAULT (the user confirmed it is smoother once the frame rate is stable),
// so in a release build pressing it would turn the good path off. It also
// once stuck the camera with the forward direction 180 degrees out - believed
// fixed by the per-image pose tracking, but not proven, so keep it out of
// testers' hands.
//
// Player-facing tuning keys are NOT gated by this and stay available in every
// build: F4/F7/F8/F9/F10/F11, '`', '/', ';' and '\'', '[' ']' '-',
// ',' '.', Home/End and Page Up/Down.
#define RE5VR_DIAGNOSTICS 0
