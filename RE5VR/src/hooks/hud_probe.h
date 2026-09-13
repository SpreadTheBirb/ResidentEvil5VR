#pragma once

#include <d3d9.h>

// HUD draw recorder (developer diagnostics). Press K in game: records every
// draw call of one frame with the HUD on, switches the HUD off in-process
// (the community trainer's "No HUD" bytes), records another frame, switches
// it back, and writes re5vr_hud_draws.txt with the draws that disappeared -
// i.e. the HUD - and which stereo path each of them took. See hud_probe.cpp.

// Stereo paths, as decided by stereo_test.cpp's BeginStereoDraw.
enum HudStereoPath : int {
    kStereoPathOff = 0,          // stereo disabled or suppressed: one untouched draw
    kStereoPathNoMatrix = 1,     // no camera matrix cached: one untouched draw
    kStereoPathOffscreen = 2,    // render target not screen-shaped: one untouched draw
    kStereoPathScreenSpace = 3,  // pixel-space ortho matrix: squeezed into each eye's half
    kStereoPathNudged = 4,       // fallback flat nudge (no pose, or implausible matrix)
    kStereoPathCamera = 5,       // full per-eye OpenXR camera
    kStereoPathHeadSkip = 6,     // skipped entirely by the head-hide probe
    kStereoPathHudViewport = 7,  // HUD: drawn once per eye into a half-size viewport
};

void HudProbe_Install();
void HudProbe_OnPresent();
// kind: 0 DrawPrimitive, 1 DrawIndexedPrimitive, 2 DrawPrimitiveUP, 3 DrawIndexedPrimitiveUP
void HudProbe_OnDraw(IDirect3DDevice9* device, int kind, D3DPRIMITIVETYPE primType, UINT primCount, int stereoPath);

// True if the shaders about to draw are the HUD's, as learned by the last K
// press this session. Stereo then draws the call into each eye's half through
// the VIEWPORT instead of the per-eye camera path - see stereo_test.cpp.
bool HudProbe_IsHudDraw(IDirect3DDevice9* device);
