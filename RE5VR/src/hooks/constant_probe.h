#pragma once

#include <d3d9.h>

// Diagnostic-only: helps empirically identify which vertex shader constant
// register holds RE5's view/camera matrix (a prerequisite for Phase 1's
// per-eye stereo draw-call duplication). RE5 is shader-based, so the camera
// is expected to arrive via SetVertexShaderConstantF rather than the
// fixed-function SetTransform - this probe watches both to confirm.
//
// Usage: press F9 in-game (in actual gameplay, not a menu). At this game's
// uncapped framerate a single frame is too easy to miss the real 3D pass
// on, so this captures a window of many frames (~600) and aggregates:
// SetVertexShaderConstantF calls that write exactly 4 consecutive Vector4f
// registers (the shape of a 4x4 matrix upload) are tallied by register.
// Registers written only once or twice per frame across the whole window
// are camera/global-matrix candidates (logged with their actual last-seen
// values); registers written dozens/hundreds of times per frame are
// per-object (world matrices, bone palettes) and are logged as a count
// only, to keep the dump readable. Press F9 again after turning the camera
// and diff the two dumps - the view matrix's register is the one whose
// values change consistently with the turn.
//
// Press F10 to toggle a live validation nudge: while on, component [3] of
// register c0 (the row's translation-like term) gets +800 added every time
// the game uploads it. If the found register genuinely feeds the camera,
// the view should visibly shift/shear on screen while toggled on.

// Installs the SetTransform/SetVertexShaderConstantF hooks backing the
// capture. Idempotent.
void ConstantProbe_Install(IDirect3DDevice9* pDevice);

// Call once per frame from the existing EndScene hook. Polls the F9 hotkey
// (edge-triggered) to arm a new capture window, counts down while one is
// active, and dumps the aggregated tally when the window ends.
void ConstantProbe_OnEndScene();

// Returns the most recently uploaded true camera/view-projection matrix
// (register c0, 16 floats), regardless of capture-window/offset-test
// state. Returns false if none has been observed yet.
bool ConstantProbe_GetCachedCameraMatrix(float out[16]);

// The addresses the game uploads its camera matrix (c0-c3) from, most-used
// first, with upload counts since startup. Returns how many were written.
// Render thread only.
int ConstantProbe_GetCameraMatrixSources(const void** outPtrs, unsigned* outCounts, int maxCount);

// Calls the REAL (un-hooked) SetVertexShaderConstantF directly, bypassing
// this file's own capture/offset-test logic. Used by stereo_test.cpp to
// push per-eye camera overrides without corrupting the cached true camera
// matrix above.
HRESULT ConstantProbe_CallRealSetVertexShaderConstantF(IDirect3DDevice9* pDevice, UINT StartRegister, const float* pConstantData, UINT Vector4fCount);
