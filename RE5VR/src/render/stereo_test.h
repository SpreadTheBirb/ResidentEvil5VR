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

// Hold the camera matrix to a human turning speed, in place, before anything
// downstream sees it. Called from the shader constant hook for register 0,
// which is the one point BOTH flatscreen and VR pass through - proven by the
// stomp whipping the view in flatscreen too, where none of the VR path runs.
void StereoTest_LimitCameraTurn(float m[16]);

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

// How the picture turns with the head. See g_pictureTurnMode for what each one
// does and why it exists; camera_rig_hook reads these to decide whether to
// steer the game camera at all.
enum {
    kPictureTurnModeDouble = 0,
    kPictureTurnModeCameraOnly = 1,
    kPictureTurnModeCatchUp = 2,
    kPictureTurnModeDoubleFixed = 3,
    kPictureTurnModeCompositorOnly = 4,
};
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
    int pictureTurnMode = 3;           // test: see the kPictureTurnMode values above
    float hudDistanceMeters = 2.0f;
    float hudScale = 0.67f;
    bool theatre = true;             // menus and cutscenes play on a flat screen
    bool theatreFollowsHead = false; // off: it hangs in the room and you can look away
    float theatreDistanceMeters = 2.2f;
    float theatreScale = 0.95f;
    bool headPositionTracking = false; // lean and peek: your head's real movement moves the view
    float leanScale = 1.0f;            // how far the game moves for how far you do
    float nearPlaneUnits = 0.0f;       // 0 = the game's own; larger clips your own body away
};

// How close something can get to the eye before it is clipped away, in game
// units, or 0 for the game's own. Read by the flat-screen path in
// constant_probe.cpp as well as by the stereo matrices here.
float StereoTest_GetNearPlaneUnits();

// Takes where your head is now as the new centre for leaning. Call when the
// player asks to recentre, or when leaning is switched on.
void StereoTest_RecentreLean();

// How far your head has moved from where it was taken, in the game's own
// units and axes (2026-09-23). The view already moves by this; the spine can
// move by it too, which is the difference between leaning the camera and
// leaning the man. False when leaning is off or no reference has been taken.
bool StereoTest_GetLeanWorld(float out[3]);

// How far your character still has to walk to stand where you are standing,
// in metres, along the view's own right and forward. This is what is left of
// your room movement after the spine has leaned as far as it will: step
// further than a lean can cover and he has to take a step.
bool StereoTest_GetRoomStep(float* rightMetres, float* forwardMetres);

// What the pad decided to ask his legs for this frame, so the loop above can
// tell his own steps apart from the ones your thumb asked for.
void StereoTest_SetRoomAsk(float right, float forward);

// How far the eye we are drawing from has been moved away from the camera the
// GAME thinks it is drawing from, in world units. Everything that displaces
// the eye is in it: leaning, roomscale, and the hold that keeps you in your
// body during an action. The culling needs it, because the game builds its
// frustum around its own camera and anything we see past the edge of that
// frustum has already been thrown away.
float StereoTest_EyeOffsetUnits();

// Which way the eye sits from the camera the game thinks it is drawing from,
// in game units. The culling needs the direction, not just the distance: it
// moves the frustum to where you actually are rather than making the camera's
// own frustum bigger and hoping.
void StereoTest_EyeOffset(float out[3]);
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

// Default-pool surfaces must go before Reset, not after it.
void StereoTest_OnBeforeDeviceReset();

// Moves the finished flat frame onto a screen in front of you, in each eye,
// while a menu or a cutscene is playing. Call once a frame, just before the
// real Present, on a back buffer the game has finished with.
void StereoTest_ComposeTheatre(IDirect3DDevice9* pDevice);

// Hold the screen up regardless of what the cutscene tests think, and ask
// whether it is being held. F10, or the button in the menu.
void StereoTest_ToggleTheatre();
bool StereoTest_TheatreHeld();
