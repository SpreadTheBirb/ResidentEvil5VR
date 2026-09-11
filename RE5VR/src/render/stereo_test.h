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
