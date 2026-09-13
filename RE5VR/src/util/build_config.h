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
// Player-facing options have no hotkeys any more (2026-09-13): they live in
// the in-game menu (ui/menu.cpp, Insert or a click of both sticks). At 1 the
// menu also gets a Developer tab for the experiment switches above.
#define RE5VR_DIAGNOSTICS 0
