#pragma once

#include <d3d9.h>

// Phase 2: bridges the game's rendered stereo frame (produced by
// stereo_test.cpp's scissor-split dual rendering - see its header for why
// that approach was chosen) into an actual VR headset via SteamVR/OpenVR.
//
// Toggle with F7. Turning it on:
//   1. Calls vr::VR_Init (requires SteamVR installed and reachable - it
//      will auto-launch SteamVR if needed, but a headset should already be
//      connected via Link/Air Link/Virtual Desktop/etc).
//   2. Forces stereo_test's dual-rendering on (via StereoTest_SetEnabled),
//      since there is nothing to submit to the headset otherwise.
//   3. Lazily creates a separate, hidden IDirect3DDevice9Ex ("the bridge
//      device") purely for producing shared textures OpenVR can read -
//      the game's own device stays completely untouched, matching the
//      Phase 0 finding that upgrading the game's device to Ex is fragile.
//
// Installs no D3D9 hooks itself - it's driven entirely from calls made by
// d3d9_hooks.cpp's EndScene hook. The actual submission work (readback +
// Submit) is throttled to roughly 90Hz regardless of how often EndScene
// fires - RE5 runs uncapped at 500-2000+ fps, and the per-eye
// GetRenderTargetData calls are expensive CPU<->GPU synchronization
// points; doing them every EndScene call caused a severe framerate
// regression confirmed unrelated to hardware specs.
void VRBridge_Install();

// Call once per frame from the EndScene hook, after the game's own
// rendering for that frame is completely finished (so the backbuffer
// contains the final, fully composited/tonemapped side-by-side image) but
// before anything else (like the Phase 0 debug quad) draws on top of it.
void VRBridge_OnEndScene(IDirect3DDevice9* pGameDevice);

// Phase 3: real head tracking. Returns the HMD's rotation, expressed as a
// row-major 3x3 delta relative to its orientation at the moment VR mode
// was last turned on (recentered each time F7 re-enables it), suitable for
// composing with RE5's own camera basis - see stereo_test.cpp for how it's
// applied. Returns false if no valid HMD pose has been observed yet (VR
// mode off, or no frames submitted since it was turned on), in which case
// callers should fall back to their own non-head-tracked behavior.
bool VRBridge_GetHeadDeltaRotation(float outDelta3x3[9]);

// Phase 4 follow-up: the connected HMD's actual lens-matched FOV for one
// specific eye, expressed as the same cot(halfFOV)-shaped scale factors
// RE5's own c0/c1 registers use (see stereo_test.cpp's CameraBasis) -
// queried once per eye from OpenVR's GetProjectionRaw (a fixed
// hardware/config property, not a per-frame value) rather than
// approximated with an untuned flat widen factor. Queried separately per
// eye - some headsets' left/right lenses are not perfectly mirrored, and
// reusing one eye's shape for the other was tried first and produced a
// visibly skewed/"cock-eyed" image in the eye that got the wrong data
// (2026-07-27 user report). Returns false if OpenVR isn't initialized yet,
// in which case callers should fall back to their own placeholder
// behavior.
bool VRBridge_GetRealFovScale(bool leftEye, float& outScaleX, float& outScaleY);

// Phase 4 follow-up: real per-eye "toe-in" rotation - many headsets angle
// each eye's lens slightly inward toward the nose to maximize FOV, which
// this project never accounted for before (only a flat position offset
// was ever applied for stereo separation - see stereo_test.cpp's
// g_halfSeparation). Queried once from OpenVR's GetEyeToHeadTransform (a
// fixed hardware/mount property) and returned as a row-major 3x3 rotation
// (rows = Right/Up/Forward, same convention as
// VRBridge_GetHeadDeltaRotation) expressing that eye's own view direction
// relative to the HMD's tracked head pose - compose it onto a per-eye
// CameraBasis the same way head rotation is composed. Returns false if
// OpenVR isn't initialized yet.
bool VRBridge_GetEyeToHeadRotation(bool leftEye, float outRot3x3[9]);
