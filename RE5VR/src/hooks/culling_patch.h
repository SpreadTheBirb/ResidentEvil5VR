#pragma once

// Disables RE5's view-frustum culling test while VR is on. See
// culling_patch.cpp.

// Hooks the camera's sphere-visibility test. Call once, after MinHook is
// initialised.
void CullingPatch_Install();

// Periodic summary of how often the test ran and would have culled.
void CullingPatch_OnEndScene();
