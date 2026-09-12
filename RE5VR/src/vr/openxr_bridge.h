#pragma once

#include <d3d9.h>

// Phase 5: replaces the OpenVR bridge (openvr_bridge.cpp/.h, kept on disk
// unreferenced as a rollback reference - not part of the build anymore)
// with an OpenXR bridge, same overall job: get stereo_test.cpp's
// scissor-split dual-rendered frame into the headset. Driven by the user's
// direct experience that OpenXR runs meaningfully better than legacy
// OpenVR, and confirmed technically viable on this machine specifically:
// unlike the fear-vr reference project (which needed a two-process x86/x64
// split because 32-bit OpenXR was unregistered on their machine), this
// machine's registry has BOTH bitnesses registered
// (HKLM\SOFTWARE\Khronos\OpenXR\1 and ...\WOW6432Node\Khronos\OpenXR\1),
// both pointing at Virtual Desktop's own OpenXR runtime - so a 32-bit
// OpenXR client (this DLL, injected into the 32-bit re5dx9.exe) can talk
// to it directly, in-process, exactly like the OpenVR bridge did. Verify
// this assumption still holds before ever attempting the two-process split
// fear-vr needed - it's a real, larger undertaking, not a drop-in.
//
// The D3D9->D3D9Ex->D3D11 pixel pipeline (game backbuffer crop via
// StretchRect -> GetRenderTargetData into system memory -> memcpy -> a
// D3D9Ex shared texture -> opened as a real D3D11 texture) is UNCHANGED
// from the OpenVR version - that part has nothing to do with OpenVR vs
// OpenXR, it's the unavoidable classic-D3D9-device CPU readback path (see
// fear-vr's own docs/M2-D3D9-BRIDGE.md, which independently arrived at the
// identical technique for the identical reason). What changes is only the
// "hand the finished per-eye D3D11 texture to the runtime" step:
// vr::VRCompositor()->Submit() is replaced by acquiring an OpenXR
// swapchain image and CopyResource-ing into it, and WaitGetPoses is
// replaced by xrWaitFrame/xrLocateViews, which - unlike OpenVR - hands us
// each eye's real position, orientation, AND asymmetric FOV directly,
// every frame, already correctly incorporating real IPD and any lens
// toe-in. stereo_test.cpp's BeginStereoDraw now consumes this directly:
// the old manual g_rightEyeYawDegrees hack is gone (real per-eye toe-in
// replaces it), and g_fovWidenMultiplier is kept only as a live-tunable
// extra widen layered on top of the real per-eye FOV, not a replacement
// for it.
void VRBridge_Install();

// Call once per frame from the EndScene hook, after the game's own
// rendering for that frame is completely finished (so the backbuffer
// contains the final, fully composited/tonemapped side-by-side image) but
// before anything else (like the Phase 0 debug quad) draws on top of it.
void VRBridge_OnEndScene(IDirect3DDevice9* pGameDevice);

// One eye's real, runtime-provided view data from the most recent
// xrLocateViews call - replaces the OpenVR bridge's separate
// GetHeadDeltaRotation/GetRealFovScale/GetEyeToHeadRotation getters, since
// OpenXR already folds head tracking, real IPD, and any lens toe-in
// together into one authoritative per-eye pose, refreshed every frame
// (not queried once at startup like the old FOV/toe-in data was).
struct XRBridgeEyeView {
    // Position in the XR reference space, in real meters - NOT RE5
    // render-space units. stereo_test.cpp still owns the meters-to-
    // render-units conversion for its camera position offset (no more
    // confirmed conversion exists than the OpenVR version had - see
    // project memory on the render-space-is-centimeters hypothesis).
    float positionMeters[3];
    // Row-major 3x3 rotation DELTA (rows = Right/Up/Forward), already
    // recentered relative to this eye's own orientation at the moment XR
    // mode was last turned on (recentered each time, same pattern as the
    // old OpenVR bridge's head-delta) - directly composable onto RE5's own
    // camera basis via stereo_test.cpp's ApplyHeadRotation, exactly like
    // the old single shared head delta was, just now real per-eye data
    // (naturally includes any real lens toe-in the runtime reports,
    // instead of a single shared head rotation plus a hand-tuned yaw
    // hack). NOT a raw absolute quaternion - the conversion (via
    // QuaternionToMat3 in mat3.h) and recentering both already happened
    // inside the bridge, mirroring where the OpenVR version's head-delta
    // math lived.
    float rotationDelta[9];
    // Real asymmetric per-eye FOV, in radians, straight from OpenXR's
    // XrFovf (angleLeft/angleDown are negative, angleRight/angleUp
    // positive, by OpenXR convention) - the exact physical frustum, not a
    // symmetric approximation.
    float angleLeft;
    float angleRight;
    float angleUp;
    float angleDown;
};

// Returns the most recently located left/right eye views. False if no
// valid frame has been located yet (XR mode off, or no frames since it was
// turned on), in which case callers should fall back to their own
// non-head-tracked behavior.
bool VRBridge_GetEyeViews(XRBridgeEyeView& outLeft, XRBridgeEyeView& outRight);

// ---- Rendered-pose tracking (2026-09-12) -------------------------------
// A tester reported VR being janky and twitchy while the desktop split view
// was perfectly smooth. Cause: xrEndFrame was submitting the pose from the
// submit thread's OWN latest xrLocateViews, but the pixels were rendered by
// the game some frames earlier from an older pose. Claiming a newer pose
// than the image was drawn with makes the compositor reproject the wrong
// way, so every head movement lurches and settles - and it is invisible on
// the desktop mirror, which does no reprojection at all.
//
// The fix is to submit the pose the image was ACTUALLY rendered with:
//   * the render thread latches a pose id along with the eye views,
//   * at Present it tags the frame the addon just published with that id,
//   * the submit thread looks the id back up and submits that pose.
//
// Ids are opaque and monotonic; 0 means "none".
using XRBridgePoseId = unsigned long long;

// The id of the eye views most recently published by the submit thread.
// Latch this in the same breath as VRBridge_GetEyeViews.
XRBridgePoseId VRBridge_GetCurrentPoseId();

// Call from the Present hook AFTER the game has finished the frame,
// passing the pose id that frame was rendered with. Associates it with
// whichever addon slot the producer just published.
void VRBridge_NoteFramePresented(XRBridgePoseId poseId);
