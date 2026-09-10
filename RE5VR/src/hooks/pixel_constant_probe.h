#pragma once

#include <d3d9.h>

// Phase 4 follow-up: hunts for the pixel shader constant that drives
// RE5's near-camera weapon fade (the held weapon goes semi-transparent
// when the render camera gets very close to it - a pre-existing RE5
// behavior, presumably meant for the normal over-the-shoulder aim zoom,
// that the near-first-person camera override triggers far more
// aggressively). Declining to fix this by backing the camera off
// (camera_rig_hook.cpp's distance values) since that would compromise
// true first-person - the fade itself needs to be found and disabled.
//
// Same technique as constant_probe.cpp's original camera-matrix hunt,
// applied to SetPixelShaderConstantF instead of the vertex-shader
// equivalent: capture a window of frames and tally which low-numbered
// registers get written with small (1-4 vector) counts, since a fade
// factor - like the camera matrix before it - is expected to be
// uploaded roughly once per relevant draw call rather than dozens of
// times (per-object world matrices, bone palettes, etc.). Unlike the
// camera hunt, a single capture likely won't be enough to identify the
// right register on its own (many small per-draw material/lighting
// constants could show similarly low counts) - capture once with the
// weapon visibly faded (camera very close) and once with it fully
// opaque (camera backed off), then diff the two dumps by hand for a
// register whose *value* changes between the two captures while its
// write pattern (register, vector count) stays the same.
//
// First attempt (whole-frame capture, diffed close-vs-opaque by hand)
// didn't cleanly isolate a candidate: every register that changed value
// between the two captures looked like a per-frame global (camera
// position, lighting, auto-exposure) shared across every draw in the
// scene, not something written specifically for the weapon or body's
// own material - confirmed 2026-07-27 when nudging the best whole-frame
// candidate (c0) blew out the *entire* scene into a black/white
// posterized look, not just the weapon.
//
// Replaced with a targeted, per-draw snapshot instead: reuses
// head_hide_probe.cpp's existing F11 (capture) / F1/F2 (browse) UI -
// already proven for finding the head-mesh parts - to point at *any*
// specific draw call (weapon, body armor, hands, etc.), not just the
// head. Delete then grabs a one-shot snapshot of every pixel shader
// constant AND the render states most likely to carry a CPU-computed
// fade alpha (TEXTUREFACTOR, blend factors) for that exact draw, so
// captures from different parts and different camera distances can be
// compared without whole-frame noise mixed in.
//
// All F1-F12 are already claimed by other probes/toggles in this
// project, so this one uses Page Up/Page Down/Delete (switched away from
// an initial Home/End binding - Home isn't present on all keyboards, and
// End may collide with one of RE5's own default binds).
//
// Usage:
//   Page Up   - (legacy whole-frame capture, superseded by Delete below -
//               kept since it's occasionally still useful for a quick
//               overview) capture a ~300-frame window of
//               SetPixelShaderConstantF calls, tallying by register.
//   Page Down - toggle a live nudge test on register c0 (component [0],
//               a large multiplicative nudge) - confirmed NOT the fade
//               (blows out the whole scene, not just the weapon), kept
//               for testing future candidates found via Delete.
//   Delete    - snapshot every pixel shader constant and the render
//               states most likely to hold a fade alpha, for the next
//               draw call matching whatever's currently selected in
//               head_hide_probe's F1/F2 browser. Workflow: F11 to
//               capture, F1/F2 to browse to the weapon or a body part
//               (watch what's skipped live, same as the head-hide
//               workflow, to confirm you've got the right one), Delete
//               to snapshot it. Repeat close to the camera (faded) and
//               backed off (opaque) for the same part, then diff.

// Installs the SetPixelShaderConstantF hook. Idempotent.
void PixelConstantProbe_Install(IDirect3DDevice9* pDevice);

// Call once per frame from the existing EndScene hook. Polls Page
// Up/Page Down/Delete (edge-triggered).
void PixelConstantProbe_OnEndScene();

// Call from each of stereo_test.cpp's Draw* hooks, before doing anything
// else (same call site as HeadHideProbe_OnDrawCall). No-ops unless a
// snapshot is armed (Delete) and this draw matches the head-hide probe's
// currently selected candidate.
void PixelConstantProbe_OnDrawCall(IDirect3DDevice9* pDevice);
