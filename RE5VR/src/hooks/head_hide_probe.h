#pragma once

#include <d3d9.h>

// Phase 4 follow-up: identifies which draw call(s) render the player
// character's own head mesh, so it can be skipped while the near-first-
// person camera override (camera_rig_hook.cpp, F4) is active - RE5 was
// never built with a real first-person mode, so the head visibly clips/
// pops when looking around close-up.
//
// DONE (2026-07-27): the head mesh's 4 sub-draws (skin, eyes/teeth, hair,
// brows or similar - one skinned model split by material) are confirmed
// found and permanently hidden. `HeadHideHook_ShouldSkip` always checks
// them first, by stable texture/vertex-buffer descriptor (not raw D3D
// pointers, which don't survive a relaunch) via `IsPermanentHeadPart` in
// the .cpp, gated only on the F4 override being on - no capture or
// bookmark needed for this part, it just works out of the box. User-
// confirmed clean in first-person with nothing else visibly missing, and
// gated so it doesn't affect normal third-person play.
//
// The interactive probe below (F11/F1-F3/F12) remains for finding
// *additional* parts later if needed - it's independent of the permanent
// mechanism and still requires its own capture/bookmark steps.
//
// First attempt (diff two captures, F4 on vs off) didn't work: F4 only
// changes the render-space camera matrix, not RE5's real internal camera
// object that drives culling (same finding as Phase 3's culling-glitch
// note), so the exact same set of draw calls happens regardless of F4
// state - nothing to diff. Replaced with an interactive cycle-and-look
// tool instead: capture once, then step through candidate draw calls
// live with the skip applied to whichever one is currently selected, and
// watch what disappears on screen. No rebuild needed between candidates.
//
// Not a standalone hook install - piggybacks on stereo_test.cpp's
// existing DrawPrimitive/DrawIndexedPrimitive/DrawPrimitiveUP/
// DrawIndexedPrimitiveUP hooks (adding a second MinHook hook on the same
// vtable slot isn't how MinHook works) via HeadHideProbe_OnDrawCall,
// which stereo_test.cpp's hooks call directly.
//
// Diagnostic probe usage (only needed to find further parts beyond the
// 4 already permanently hidden):
//   F11 - capture a ~300-frame window of unique (texture, vertex buffer)
//         draw-call signatures. Do this with F4 ON and your character's
//         head filling the screen, so the head's draw call is definitely
//         included in the window. Skipping auto-enables when the capture
//         finishes, so no separate toggle step is needed before cycling.
//   F1/F2 - step to the previous/next captured signature. Whichever one
//           is currently selected is skipped live (immediate visual
//           feedback - watch the screen as you press F2). Logs the
//           selected entry's index, texture/vertex-buffer pointers, hit
//           count, last-seen primitive count, and stable texture/vertex-
//           buffer descriptors each time you move.
//   F3 - bookmark (or unbookmark) the currently selected candidate so it
//        stays hidden even after you step past it. Multiple parts (face,
//        hair, etc.) are usually separate draw calls - step to one,
//        bookmark it, step to the next, bookmark that too, and all
//        bookmarked parts stay hidden together regardless of which one
//        is currently selected. Logs a clearly tagged BOOKMARKED/
//        UNBOOKMARKED line so the confirmed parts are easy to find in
//        the log afterward.
//   F12 - toggle skipping off/on entirely (both the live-preview
//         selection and all bookmarks), to compare against the
//         unmodified scene.

// Call once per frame from the existing EndScene hook. Polls F11/F1/F2/
// F12 (all edge-triggered).
void HeadHideProbe_OnEndScene();

// Call from each of stereo_test.cpp's Draw* hooks, before doing anything
// else. No-ops entirely outside an armed capture window - cheap
// otherwise.
void HeadHideProbe_OnDrawCall(IDirect3DDevice9* pDevice, const char* callType, UINT primCount);

// Returns true if this draw call should be skipped entirely (both eyes) -
// either because it matches one of the 4 permanently-confirmed head-mesh
// parts, or because it matches the diagnostic probe's live-preview
// selection or a bookmarked candidate with F12 skip enabled. Always false
// while the F4 override is off.
bool HeadHideHook_ShouldSkip(IDirect3DDevice9* pDevice);

// Exposes the F1/F2-selected candidate's (texture, vertex buffer) pair so
// other probes (pixel_constant_probe.cpp) can identify the same draw call
// without duplicating this probe's own capture/browse UI - lets the
// existing F11/F1/F2 workflow (already proven for finding the head parts)
// double as a general "point at any draw call" tool for other hunts, like
// the weapon/body near-camera fade. Returns false if nothing is captured
// or selected.
bool HeadHideProbe_GetSelectedSignature(void** outTexture, void** outVertexBuffer);
