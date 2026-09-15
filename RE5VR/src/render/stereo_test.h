#pragma once

#include <d3d9.h>

// Phase 1 validation step: while toggled on (F8), every Draw* call is
// executed twice - once with the cached camera matrix (see
// hooks/constant_probe.h) shifted one way along its c0[3] translation-like
// term scissor-clipped to the left half of the current viewport, once
// shifted the other way clipped to the right half - producing a real
// side-by-side stereo image, without ever touching render targets or the
// viewport itself.
//
// Two earlier approaches were tried and rejected: splitting the *viewport*
// per eye corrupted a full-screen post-process pass elsewhere in the
// pipeline (viewport changes affect the vertex-to-clip-space transform,
// which that pass's UV math apparently depends on); redirecting rendering
// into separate full-size per-eye render targets never triggered at all,
// since RE5's Clear() calls never target the literal backbuffer directly -
// strongly suggesting it renders into an intermediate HDR/offscreen buffer
// first and reaches the backbuffer only via a separate tonemap/composite
// pass, which redirecting render targets ourselves fights with. A scissor
// rect is a pure per-pixel clip - it doesn't touch the transform, and
// draws still land wherever the game itself already intended (whatever
// that intermediate buffer is), so whatever downstream pass processes it
// should see a coherent side-by-side image.

// Installs the DrawPrimitive/DrawIndexedPrimitive/DrawPrimitiveUP/
// DrawIndexedPrimitiveUP hooks backing the test. Idempotent.
void StereoTest_Install(IDirect3DDevice9* pDevice);

// Call once per frame (from the existing EndScene hook) to poll the F8
// toggle hotkey.
void StereoTest_OnEndScene(IDirect3DDevice9* pDevice);

// Call right after every IDirect3DDevice9::Present: latches the head pose
// that every draw of the next frame will use.
void StereoTest_OnPresent();

// Call around any draw call that must never be duplicated/clipped (e.g.
// the Phase 0 debug quad) - while suppressed, stereo_test's Draw* hooks
// pass straight through to a single normal draw.
void StereoTest_SetSuppressed(bool suppressed);

// Directly forces dual-rendering on/off, bypassing the F8 hotkey. Used by
// openvr_bridge.cpp so turning on VR mode (F7) also turns on the
// underlying stereo rendering, since there's nothing to submit to the
// headset otherwise.
void StereoTest_SetEnabled(bool enabled);

// The real backbuffer aspect (width / height), or 16:9 before the first
// frame has been seen. Authoritative, unlike inferring it from whatever
// camera matrix a pass happened to leave cached - see FirstPersonVerticalFov.
float StereoTest_GetBackbufferAspect();

// The head's forward vector from the pose LATCHED for this frame - the same
// snapshot the eye matrices use. False if no fresh latch exists. Head-follow
// must use this rather than the live published pose: two different snapshots
// of the same head, sampled moments apart, make a fast turn overshoot and
// snap back as they reconverge.
bool StereoTest_GetLatchedHeadForward(float out[3]);

// The pose id of that latch (0 if none).
unsigned long long StereoTest_GetLatchedPoseId();

// Test build: 0 double (v0.4.1), 1 game camera only, 2 catch-up. See g_pictureTurnMode.
int StereoTest_GetPictureTurnMode();

// ---- In-game menu (ui/menu.cpp) -----------------------------------------
// Everything the old F8, '/', '\', '[' ']' '-' and ';' '\'' hotkeys changed,
// plus the HUD's distance and size. Apply clamps and logs what changed.
struct StereoSettings {
    bool stereoEnabled = false;        // developer: side-by-side without VR (VR turns it on itself)
    float halfSeparation = 2.75f;      // world scale: larger = the world looks smaller
    float fovWiden = 1.0f;
    bool monoSmallTargets = true;      // the light-leak fix
    bool compensateHeadFollow = false; // developer (superseded by pictureTurnMode)
    int pictureTurnMode = 3;           // test: 0 double (v0.4.1), 1 game camera only, 2 catch-up, 3 double with culling fixed
    float hudDistanceMeters = 2.0f;
    float hudScale = 0.67f;
};
StereoSettings StereoTest_GetSettings();
void StereoTest_ApplySettings(const StereoSettings& s);
bool StereoTest_IsEnabled();

// Where a flat panel straight ahead at distanceMeters lands in each eye's half
// of a side-by-side frame, using the same per-eye convergence as the HUD, plus
// how many pixels one unit of view tangent covers in that eye (x and y differ:
// an eye's half is usually narrower in pixels than the angle it covers).
// Returns false when no headset views are available (centred fallback).
struct StereoPanelEye {
    float centreX, centreY;
    float halfX0, halfWidth;
    float pxPerTanX, pxPerTanY;
};
bool StereoTest_GetPanelPlacement(float distanceMeters, UINT frameWidth, UINT frameHeight, StereoPanelEye eyes[2]);

// After a successful device Reset: the backbuffer may have changed size (full
// resolution per eye switches it), so re-read it before the next draw instead
// of up to a second later.
void StereoTest_OnDeviceReset();
