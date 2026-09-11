#pragma once

// F5 "boom finder" (2026-09-11): automates the Cheat Engine work of
// locating the code that pushes the camera out on pitch. See
// boom_finder.cpp for what it captures and why.
//
// Call once per frame from the EndScene hook.
void BoomFinder_OnEndScene();
