#pragma once

// Disables RE5's view-frustum culling test while VR is on. See
// culling_patch.cpp.

// Hooks the camera's sphere-visibility test. Call once, after MinHook is
// initialised.
void CullingPatch_Install();

// Periodic summary of how often the test ran and would have culled.
void CullingPatch_OnEndScene();

// The camera the renderer asks for the player view of - which is a different
// and much larger object than the one the sphere test measures against. That
// one holds the frustum planes; this one is where the game keeps the position
// it sorts, fades and picks detail levels from.
void CullingPatch_NoteMainCamera(void* camera);

// Arm a write watchpoint on the main camera's position, to find whatever moves
// it during a melee. Press F8 just before a stomp. Each press watches the next
// of the places the position was found in, so all of them can be tried.
void CullingPatch_FindCameraWriter();

// Developer: arm a write watchpoint on a live camera's frustum planes, to find
// the routine that builds them. Every culling test in the game measures against
// those six planes, so widening them where they are WRITTEN reaches all of it,
// including whatever removes scenery during a cutscene - which is not the
// sphere test this file already answers, since that runs under once a frame.
void CullingPatch_FindPlaneWriter();
