#include "stereo_test.h"
#include "../hooks/camera_rig_hook.h"
#include "../hooks/constant_probe.h"
#include "../hooks/fade_probe.h"
#include "../hooks/head_hide_probe.h"
#include "../hooks/pixel_constant_probe.h"
#include "../hooks/hud_probe.h"
#include "hud_shaders.h"
#include "../vr/openxr_bridge.h"
#include "../vr/xr_input.h"
#include "../util/log.h"
#include "../util/build_config.h"
#include "mat3.h"

#include <MinHook.h>

#include <algorithm>
#include <atomic>
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

// Same stable vtable slot numbering used elsewhere in this project.
constexpr size_t kIDirect3DDevice9_DrawPrimitive = 81;
constexpr size_t kIDirect3DDevice9_DrawIndexedPrimitive = 82;
constexpr size_t kIDirect3DDevice9_DrawPrimitiveUP = 83;
constexpr size_t kIDirect3DDevice9_DrawIndexedPrimitiveUP = 84;

// Eye separation, in RE5's render-space units (not meters - no confirmed
// units-per-meter conversion exists for this game, see Phase 4 notes on
// the 100x gameplay/render scale, which is a different relationship).
// The earlier comfort-tuned value (20.0, "tolerable" but with a reported
// "shoebox" scale problem) is REPLACED here (2026-07-28) with a value
// derived from real data instead of feel: real per-eye offset from OpenVR
// (0.0318m, see VRBridge_GetEyeToHeadRotation's header/log) times an
// assumed 100 render-units-per-meter (render-space is exactly 100x
// gameplay-space per Phase 4's confirmed camera-struct scale - if
// gameplay-space is authored in meters, a common engine convention,
// render-space units are effectively centimeters). This predicts
// g_halfSeparation ~= 3.18, a large drop from 20 - direct supporting
// evidence: user reported a real, measured, quantified depth error at
// separation=20 (a deer corpse read as ~4ft away when it should be ~10ft,
// a ~2.5x compression - exactly the "toy town" symptom of an oversized
// virtual IPD relative to true scene scale).
//
// MEASURED AND REPLACED (2026-09-10): 2.75, roughly 13% tighter than the
// derived 3.18. This is the first time the value has actually been
// measured rather than predicted, and it only became possible once the
// F4 first-person camera worked - scale cannot be judged honestly from a
// floating third-person vantage point, which is why it stayed open for so
// long. The user's verdict at 2.75: "the gun, sheva, and buildings more
// or less feel like the right size... a good balance of nothing too big,
// nothing too small."
//
// The sweep behind it (179 presses, read off the log rather than assumed):
// down to the 0.00 floor, up past 7.75, then a narrowing oscillation
// inside 2.50-3.25 before settling. That final oscillation is what makes
// this a real convergence - an earlier session the same day ended at the
// 0.00 floor and was briefly, wrongly, reported as confirming 3.18.
//
// Still one session in one scene, so treat it as the best available
// measurement rather than a settled constant. Implied render-units-per-
// meter = 2.75 / 0.0318 ~= 86.5, not the assumed 100.
// Camera-trajectory tracking after an F4 toggle (see the tracking block in
// StereoTest_OnEndScene).
//
// MUST be wall-clock, not a frame count. The first attempt used 4000
// EndScene calls expecting ~20s, and got 0.4s: EndScene runs several times
// per rendered frame here, around 10,000 calls/sec, so the whole window
// elapsed during the transition into first person and never saw the
// look-down it was built to capture.
constexpr unsigned long long kRigTrackDurationMs = 20000;
constexpr unsigned long long kRigTrackSampleEveryMs = 100;

constexpr float kDefaultHalfSeparation = 2.75f;
float g_halfSeparation = kDefaultHalfSeparation;
// 0.25 rather than 0.5: with the projection/scissor bug fixed (see
// FitEyeFovToViewportHalf) the remaining stereo complaint is doubling on
// the near character, and the plausible landing zone for that is roughly
// 0.5-1.5 - a 0.5 step is too coarse to bracket a value in that range.
// Whatever value ends up comfortable also directly measures this game's
// render-units-per-meter (= g_halfSeparation / 0.0318), which is the
// constant the 100-units-per-meter assumption behind the 3.18 default
// was only ever a guess at.
constexpr float kHalfSeparationStep = 0.25f;

// Extra live-tunable FOV widen on top of the real per-eye HMD FOV scale
// reported by OpenXR (Phase 5) - kept from the Phase 4 OpenVR version in
// case the real lens-matched FOV still feels cramped; defaults to 1.0
// (no-op). scaleX/scaleY are cot(halfFOV)-shaped, so a SMALLER scale is a
// WIDER angle - dividing by this multiplier widens the FOV as the
// multiplier increases, keeping the ';'/'\'' key semantics intuitive
// (higher value = bigger FOV, matching the increase-on-the-right
// convention of '['/']'). Applied AFTER the real per-eye scale is chosen,
// and has no effect on orientation.
// Draw post-process buffers (anything meaningfully smaller than the
// backbuffer) once instead of splitting them per eye - see
// RenderTargetIsScreenShaped. This is the light-leak fix and has been ON by
// default since it landed (dc2d257, shipped in v0.3.4-alpha); '/' turns it off
// to get the old, leaking per-eye behaviour back for comparison. DO NOT
// commit it flipped: on 2026-09-12 it was set to false in a working tree
// during an experiment and left there, the leaks came back in the dev and
// tester builds, and hours went into hunting a bug that was this line.
std::atomic<bool> g_monoSmallTargets{ true };

// Don't rotate the eye bases by the head delta while F9 is already steering
// the game camera with it - see buildEyeBasis. '\' toggles.
std::atomic<bool> g_compensateHeadFollow{ false };

// How the picture turns while head-follow steers the game camera (2026-09-15).
// Measured on the user's PC: 0 (what v0.4.1 ships) turns the picture 1.84-2.06x
// the head, because the eye matrices add the head rotation to a camera that
// already has it - the source of the over-the-shoulder culling. 1 (camera
// only) is 0.92-1.03x but feels slow: the game draws from a camera update one
// to three steps old, varying frame to frame, so it can never be fresher than
// the game's own pipeline (and it jittered). 2 (catch-up) keeps the camera
// steering, so culling follows the head, and turns the picture only by the
// difference between the pose that camera was steered with and the freshest
// pose latched at draw time, per eye - Double's freshness and 3D, 1x.
// (A "direct" mode that swung every draw onto the written direction was
// tried the same night: 1.00x but "felt terrible, ton of artifacts".)
// 3 (double, culling fixed): the user preferred v0.4.1's feel ("connected to
// Chris's neck") over every 1x mode, so keep that PICTURE exactly and move only
// the cone: the camera is turned by twice the head (camera_rig_hook's
// DoubleHeadAim), which points it where v0.4.1's picture points, and each eye
// gets its fresh delta with the extra turn taken back out:
//   picture = N * aim(head used) * aim(camera used)^T * camera
// which is v0.4.1's N * camera(head used), whatever the doubling did.
// 4 (compositor only) is the head-turn jitter test (2026-09-16). Every mode
// above has TWO things turning with the head - the game camera and the eye
// matrices - on two different delays, and the runtime's reprojection assumes
// the picture carries exactly the pose we submit with it. Any disagreement is
// an error that scales with how fast the head moves, which is invisible while
// still, invisible on the stick (that rotation is painted into the pixels) and
// shakes only when the wearer physically turns: the reported symptom. This
// mode leaves the game camera entirely alone and turns the picture with the
// freshest latched pose only, so there is one rotation source, a gain of
// exactly 1, and nothing for the compositor to fight. The cost is culling: the
// game's cone no longer follows the head, so camera_rig_hook opens it to the
// widest angle the rig accepts and things can still vanish at a hard look
// behind. That is the trade being tested, not a regression.
std::atomic<int> g_pictureTurnMode{ 3 };
constexpr int kPictureTurnDouble = 0;
constexpr int kPictureTurnCameraOnly = 1;
constexpr int kPictureTurnCatchUp = 2;
constexpr int kPictureTurnDoubleFixed = 3;
constexpr int kPictureTurnCompositorOnly = 4;
constexpr int kPictureTurnModeCount = 5;

const char* PictureTurnModeName(int mode)
{
    switch (mode) {
    case kPictureTurnDouble: return "double (v0.4.1)";
    case kPictureTurnCameraOnly: return "game camera only";
    case kPictureTurnCatchUp: return "catch-up";
    case kPictureTurnDoubleFixed: return "double, culling fixed";
    case kPictureTurnCompositorOnly: return "compositor only";
    default: return "unknown";
    }
}

// Leaning and peeking: see the lean block in buildEyeBasis. The reference is
// where your head was when it was last taken, and it is retaken whenever the
// feature is switched on or the view is recentred.
// How close something can get to the eye before it is clipped away, in game
// units. 0 keeps the game's own value. See ComposeCameraMatrix.
float g_nearPlaneUnits = 0.0f;
bool g_headPositionTracking = false;
float g_leanScale = 1.0f;
float g_leanReference[3] = {};
bool g_leanReferenceSet = false;
// The same movement in the game's units and axes, for the spine.
float g_leanWorld[3] = {};
bool g_haveLeanWorld = false;
// What is left for his legs to do, in metres along the view's right and
// forward.
float g_roomStep[2] = {};
// What the pad last asked his legs for, along the view's right and forward.
float g_roomAsk[2] = {};
// The furthest either eye has been moved from the game's camera this frame.
std::atomic<float> g_eyeOffUnits{ 0.0f };
float g_eyeOffThisFrame = 0.0f;
// And WHICH WAY, not just how far. The culling needs to move the frustum to
// where the eye actually is, and a distance cannot say that.
std::atomic<float> g_eyeOffX{ 0.0f }, g_eyeOffY{ 0.0f }, g_eyeOffZ{ 0.0f };
float g_eyeOffSum[3] = {};
bool g_haveRoomStep = false;

float g_fovWidenMultiplier = 1.0f;
constexpr float kFovWidenStep = 0.1f;

bool g_enabled = false;
bool g_suppressed = false;

bool g_theatre = true;
float g_theatreDistanceMeters = 2.2f;
float g_theatreScale = 0.95f;
bool g_inTheatre = false;
// Held on by hand, whatever the tests think (2026-09-25, user: "while not a
// permanent fix, it'd be nice to have a bind to go into theatre mode").
//
// Worth more than another guess at the detection. Nothing distinguishes an
// action camera from an in-engine cutscene reliably - five tests tried, and the
// one that should have worked made the action camera worse - so the person
// wearing the headset, who can see which it is, gets a switch.
bool g_theatreForced = false;
bool g_theatreFollowsHead = false;
float g_screenFwd[3] = {};
float g_screenRight[3] = {};
bool g_haveScreenAnchor = false;
// When the rendered eye and the skeleton's eye last parted company, or 0.
unsigned long long g_cameraAwaySince = 0;
// HUD draws seen since the last Present, and when the HUD last went away.
unsigned g_hudDrawsThisFrame = 0;
unsigned g_hudDrawsLastFrame = 0;
unsigned long long g_hudGoneSince = 0;
// Hard cuts since the camera last left your body. A film is cut together; a
// chest opening, a vault and a melee are each one continuous move.
int g_cameraCuts = 0;
// The finished frame, as something a quad can be textured with.
IDirect3DTexture9* g_theatreTex = nullptr;
IDirect3DSurface9* g_theatreSurf = nullptr;
UINT g_theatreCopyW = 0, g_theatreCopyH = 0;
D3DFORMAT g_theatreCopyFmt = D3DFMT_UNKNOWN;
// Pre-transformed, with one over the depth as w so the picture stays
// perspective-correct across the quad instead of being stretched onto it.
struct TheatreVertex {
    float x, y, z, rhw;
    float u, v;
};
void ReleaseTheatreCopy()
{
    if (g_theatreSurf) {
        g_theatreSurf->Release();
        g_theatreSurf = nullptr;
    }
    if (g_theatreTex) {
        g_theatreTex->Release();
        g_theatreTex = nullptr;
    }
    g_theatreCopyW = 0;
    g_theatreCopyH = 0;
}
// Which branch BeginStereoDraw took for the draw in progress - read by the HUD
// recorder (hud_probe.cpp) right after the decision, so it can say exactly
// what VR did to each HUD draw. Game thread only.
int g_stereoPath = kStereoPathOff;

// See "2026-09-11 VR hole fixes" above BeginStereoDraw. '`' toggles them.
bool g_frameFixes = true;
// Pixel-space HUD draws are pulled in to this fraction of each eye's view -
// laid out to the screen corners, they would otherwise sit at the lens edge.
constexpr float kScreenSpaceScale = 0.8f;
// If Present stops arriving (the game presenting some other way), fall back
// to reading the pose live rather than freezing it.
constexpr unsigned long long kPresentStaleMs = 250;
XRBridgeEyeView g_frameViews[2] = {};
bool g_haveFrameViews = false;
// Which published pose g_frameViews came from, carried through to Present
// so the submit path can tell the compositor the truth about this image.
XRBridgePoseId g_frameViewsPoseId = 0;
// Game camera only: the pose head-follow steered the camera of the frame now
// being drawn with, captured at its first draw - Present tags with this
// instead of the latch (see OnRigsReady). 0 when not in that mode.
XRBridgePoseId g_frameCameraPoseId = 0;
unsigned long long g_lastPresentMs = 0;
unsigned g_presentCount = 0;
UINT g_backBufferWidth = 0;
UINT g_backBufferHeight = 0;

// What the draw hooks did with each draw, logged every 5 s while in VR.
struct DrawCensus {
    unsigned split3d, screenSpace, nudged, offscreen;
    struct Target {
        UINT w, h;
        unsigned n;
    } targets[6];
};
DrawCensus g_census = {};

void CountOffscreenTarget(UINT w, UINT h)
{
    ++g_census.offscreen;
    for (auto& t : g_census.targets) {
        if (t.n && t.w == w && t.h == h) {
            ++t.n;
            return;
        }
        if (!t.n) {
            t.w = w;
            t.h = h;
            t.n = 1;
            return;
        }
    }
}

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

// Two earlier approaches were tried and rejected:
//  - Splitting the *viewport* per eye corrupted a full-screen post-process
//    pass elsewhere in the pipeline (repeating kaleidoscope glitch) -
//    viewport changes affect the vertex-to-clip-space transform, which a
//    full-screen effect quad's UV math apparently depends on.
//  - Redirecting rendering into separate full-size per-eye render targets
//    (then compositing at end of frame) never triggered at all - the
//    game's Clear() never targets the literal backbuffer directly, which
//    strongly suggests RE5 renders its 3D scene into an intermediate
//    HDR/offscreen buffer first and only reaches the real backbuffer via a
//    separate tonemap/composite pass. Redirecting render targets ourselves
//    fights with that pipeline structure.
//
// A scissor rect avoids both problems: it's a pure per-pixel clip that
// doesn't touch the vertex/projection transform (unlike viewport) and
// doesn't require touching render targets at all (unlike the eye-texture
// approach) - draws still land wherever the game itself already intended
// (an intermediate buffer, the backbuffer, whatever), just clipped to a
// screen-space half. Whatever downstream pass processes that buffer
// (tonemap, etc.) should see a coherent side-by-side image the same way it
// would have seen the single un-split image.
struct StereoDrawContext {
    float leftMatrix[16];
    float rightMatrix[16];
};

// c0-c3 is a combined view-projection matrix (row-vector convention,
// confirmed empirically via F6 snapshots at several camera
// positions/orientations - see the matching plan/memory notes):
//   c0.xyz = Right * Sx,  c0.w = -Sx * dot(Right, CamPos)
//   c1.xyz = Up * Sy,     c1.w = -Sy * dot(Up, CamPos)
//   c2.xyz = Forward,     c2.w = -dot(Forward, CamPos) + K
//   c3.xyz = Forward,     c3.w = -dot(Forward, CamPos)
// where Right/Up/Forward is an orthonormal basis (Right/Up need
// normalizing first - they carry the horizontal/vertical FOV scale; c2/c3
// only differ by a constant depth-bias K, ~-16 in observed captures).
// Decomposing this lets per-eye views be built from a real rotated/offset
// basis instead of the old flat nudge to a single component.
struct CameraBasis {
    float right[3];
    float up[3];
    float forward[3];
    float camPos[3];
    float scaleX;
    float scaleY;
    float depthBiasK; // c2.w - c3.w, preserved as-is into the rebuilt matrix
};

float Length3(const float v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Diagnostic (2026-09-15): how far does the headset picture turn in the world
// for each degree the head turns? 1.0 is right; 2.0 means the head rotation is
// applied twice - once by head tracking turning the game camera, and again on
// the eye matrices. A tester saw culling while standing still looking over
// his shoulder, which fits the picture running ahead of the game camera the
// cone is built around. Once per frame; logged per 3 s window. (A first
// version compared the picture with the game camera directly, which only
// restated the head angle - the gap it measured is the one this code adds.)
void NoteViewVsGameCamera(const float eyeForward[3], const float rotationDelta[9])
{
    const auto yawOf = [](const float v[3]) { return std::atan2(v[0], v[2]) * 57.2957795f; };
    const auto wrap = [](float d) {
        while (d > 180.0f)
            d -= 360.0f;
        while (d < -180.0f)
            d += 360.0f;
        return d;
    };
    const float pictureYaw = yawOf(eyeForward);
    const float headYaw = yawOf(rotationDelta + 6);
    const bool compensating = g_pictureTurnMode.load(std::memory_order_relaxed) != kPictureTurnDouble;

    static bool s_havePrev = false;
    static float s_prevPicture = 0.0f, s_prevHead = 0.0f;
    static bool s_prevCompensating = false;
    static double s_pictureSum = 0.0, s_headSum = 0.0;
    static unsigned s_frames = 0;
    static ULONGLONG s_windowStartMs = 0;

    if (s_havePrev && compensating == s_prevCompensating) {
        const float dp = std::fabs(wrap(pictureYaw - s_prevPicture));
        const float dh = std::fabs(wrap(headYaw - s_prevHead));
        if (dp < 45.0f && dh < 45.0f) { // skip cuts and teleports
            s_pictureSum += dp;
            s_headSum += dh;
            ++s_frames;
        }
    }
    if (compensating != s_prevCompensating) {
        Log_Printf("StereoTest: picture turning now %s", compensating ? "single (camera only or direct)"
                                                                     : "DOUBLE (eye matrices add the head rotation too)");
        s_pictureSum = s_headSum = 0.0;
        s_frames = 0;
        s_windowStartMs = 0;
    }
    s_prevPicture = pictureYaw;
    s_prevHead = headYaw;
    s_prevCompensating = compensating;
    s_havePrev = true;

    const ULONGLONG now = GetTickCount64();
    if (!s_windowStartMs)
        s_windowStartMs = now;
    if (now - s_windowStartMs < 3000)
        return;
    if (s_headSum >= 10.0) {
        Log_Printf("StereoTest: picture turned %.0f deg in the world while the head turned %.0f deg - %.2fx (%s, %u frames)",
            s_pictureSum, s_headSum, s_pictureSum / s_headSum,
            PictureTurnModeName(g_pictureTurnMode.load(std::memory_order_relaxed)), s_frames);
    }
    s_windowStartMs = now;
    s_pictureSum = s_headSum = 0.0;
    s_frames = 0;
}

void ApplyHeadRotation(CameraBasis& basis, const Mat3& delta);

// A head orientation with its roll taken out: forward f, right level with the
// ground, in the same Right/Up/Forward rows as the eye deltas. False when f
// points straight up or down (no defined yaw).
bool NoRollAim(const float f[3], Mat3& out)
{
    const float fl = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (fl < 1e-4f)
        return false;
    const float fwd[3] = { f[0] / fl, f[1] / fl, f[2] / fl };
    float right[3] = { fwd[2], 0.0f, -fwd[0] }; // cross((0,1,0), fwd)
    const float rl = std::sqrt(right[0] * right[0] + right[2] * right[2]);
    if (rl < 1e-3f)
        return false;
    right[0] /= rl;
    right[2] /= rl;
    const float up[3] = { fwd[1] * right[2] - fwd[2] * right[1], fwd[2] * right[0] - fwd[0] * right[2],
        fwd[0] * right[1] - fwd[1] * right[0] };
    for (int i = 0; i < 3; ++i) {
        out.m[0 * 3 + i] = right[i];
        out.m[1 * 3 + i] = up[i];
        out.m[2 * 3 + i] = fwd[i];
    }
    return true;
}

float Dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// stereo_test's Draw* hooks fire for every draw call, not just 3D world
// geometry - UI/HUD elements plausibly reuse the same c0 register slot for
// a completely different (e.g. 2D/orthographic) transform that doesn't
// have this "FOV-scaled Right/Up + unit Forward" shape at all. The old
// flat nudge (c0[3] += x) was harmless even applied to the wrong kind of
// matrix - just a small additive shift. This decomposition divides by
// vector lengths and inverts a basis, so running it on a non-camera
// matrix (near-zero scale, non-unit "forward") can produce huge/garbage
// values - exactly the kind of thing that would show up as flicker on
// elements redrawn every frame. Bounds are generous (real captures showed
// Sx~1.5-1.6, Sy~2.7-2.9, |forward| always ~1.0) - this is a sanity check
// against degenerate matrices, not a tight tolerance.
bool IsPlausibleCameraMatrix(const float m[16])
{
    const float sx = Length3(&m[0]);
    const float sy = Length3(&m[4]);
    const float sf = Length3(&m[8]);
    if (sx < 0.05f || sx > 50.0f) return false;
    if (sy < 0.05f || sy > 50.0f) return false;
    if (sf < 0.9f || sf > 1.1f) return false;
    return true;
}

void DecomposeCameraMatrix(const float m[16], CameraBasis& basis)
{
    basis.scaleX = Length3(&m[0]);
    basis.scaleY = Length3(&m[4]);
    for (int i = 0; i < 3; ++i) {
        basis.right[i] = m[i] / basis.scaleX;
        basis.up[i] = m[4 + i] / basis.scaleY;
        basis.forward[i] = m[8 + i]; // already unit (c2/c3)
    }
    basis.depthBiasK = m[11] - m[15];

    const float rhs0 = -m[3] / basis.scaleX;  // dot(Right, CamPos)
    const float rhs1 = -m[7] / basis.scaleY;  // dot(Up, CamPos)
    const float rhs2 = -m[15];                // dot(Forward, CamPos)
    // Right/Up/Forward is orthonormal, so its inverse is its transpose -
    // solving R * CamPos = rhs for CamPos is just that transpose applied.
    for (int i = 0; i < 3; ++i)
        basis.camPos[i] = basis.right[i] * rhs0 + basis.up[i] * rhs1 + basis.forward[i] * rhs2;
}

// Rebuilds a 16-float c0-c3 matrix from a (possibly head-rotated/eye-
// offset) basis, using the same formulas DecomposeCameraMatrix reversed.
void ComposeCameraMatrix(const CameraBasis& basis, float outMatrix[16])
{
    const float w3 = -Dot3(basis.forward, basis.camPos);
    for (int i = 0; i < 3; ++i) {
        outMatrix[i] = basis.right[i] * basis.scaleX;
        outMatrix[4 + i] = basis.up[i] * basis.scaleY;
        outMatrix[8 + i] = basis.forward[i];
        outMatrix[12 + i] = basis.forward[i];
    }
    outMatrix[3] = -Dot3(basis.right, basis.camPos) * basis.scaleX;
    outMatrix[7] = -Dot3(basis.up, basis.camPos) * basis.scaleY;
    // The near plane, and where you stop seeing the inside of your own chest
    // (2026-09-17). z/w works out as 1 + K/d for a point d ahead of the eye, so
    // it crosses zero at d = -K: that term IS the near distance, negated. The
    // game's own value is tuned for a camera hanging behind a shoulder, and in
    // first person it lets you look down into Chris. A larger near distance
    // clips the body away before it reaches your eye.
    outMatrix[11] = w3 + (g_nearPlaneUnits > 0.0f ? -g_nearPlaneUnits : basis.depthBiasK);
    outMatrix[15] = w3;
}

// Applies a head-rotation delta (row-major 3x3, rows = Right/Up/Forward
// expressed as a "world-to-local" matrix - see openvr_bridge.h) to a
// camera basis, replacing its Right/Up/Forward with the rotated result.
// See openvr_bridge.cpp's head-tracking comment for the derivation: this
// is a LOCAL rotation delta (frame-independent relative to HMD reference),
// so it composes directly onto RE5's own basis via left-multiplication.
void ApplyHeadRotation(CameraBasis& basis, const Mat3& delta)
{
    Mat3 gameBasis{};
    for (int i = 0; i < 3; ++i) {
        gameBasis.m[0 * 3 + i] = basis.right[i];
        gameBasis.m[1 * 3 + i] = basis.up[i];
        gameBasis.m[2 * 3 + i] = basis.forward[i];
    }
    Mat3 rotated = Mat3Multiply(delta, gameBasis);
    for (int i = 0; i < 3; ++i) {
        basis.right[i] = rotated.m[0 * 3 + i];
        basis.up[i] = rotated.m[1 * 3 + i];
        basis.forward[i] = rotated.m[2 * 3 + i];
    }
}

// Remaps an eye's projection so its FULL horizontal FOV lands inside the
// half of the viewport that eye's scissor rect actually keeps.
//
// Without this the composed matrix spreads the eye's whole FOV across the
// FULL viewport width, and SetEyeState then throws away half of it - so
// the left eye ends up showing the left half of its own view and the right
// eye the right half. The two eyes are then aimed roughly half a FOV apart
// from each other, which is a large, sustained divergence: it reads as
// cross-eyed/wall-eyed and cannot be fixed by tuning separation, because
// it is an angular error, not a positional one. (This is almost certainly
// also the "eye strain, doesn't read as clearly 3D" reported all the way
// back in Phase 2, which was put down to missing head tracking at the
// time.)
//
// The correction, in clip space: we want the eye's view to occupy NDC x
// [-1,0] (left) or [0,1] (right) instead of the full [-1,1], i.e.
//   ndc_new = 0.5 * ndc_old + offset,  offset = -0.5 left, +0.5 right
// Since ndc = clip.x / clip.w, and c0 produces clip.x while c3 produces
// clip.w, that is exactly:
//   c0_new = 0.5 * c0 + offset * c3
// which keeps the eye's forward axis at the centre of its own half and
// maps the FOV edges precisely onto that half's edges.
void FitEyeFovToViewportHalf(float m[16], bool leftEye)
{
    const float offset = leftEye ? -0.5f : 0.5f;
    for (int i = 0; i < 4; ++i)
        m[i] = 0.5f * m[i] + offset * m[12 + i];
}

// Restores the OFF-CENTRE part of each eye's real OpenXR frustum.
//
// An HMD eye frustum is asymmetric: the eye sees further toward its own
// temple than toward the nose, so the frustum's centre line points
// slightly OUTWARD, not straight ahead. Building a symmetric frustum from
// the total FOV (what this code did before) throws that away and points
// both eyes straight ahead - which is equivalent to toeing the eyes IN by
// the asymmetry angle, giving the rig a fixed convergence distance it
// should not have.
//
// That produces a constant angular error theta on top of real parallax:
//     disparity(Z) ~= K * 2 * halfSeparation / Z  -  theta
// which fuses only at the single distance where the two terms cancel.
// Everything nearer is crossed-doubled, everything FARTHER is
// DIVERGENT-doubled - and divergent disparity is the kind eyes physically
// cannot fuse, so distant objects split badly however the near ones look.
//
// The observed symptoms matched this exactly: separation had to be pushed
// from 3.18 up to ~52.75 (cancelling theta at the character's distance
// rather than setting a real IPD), the character fused only at one range
// and split against a wall, distant NPCs stayed doubled, and pressing '['
// to REDUCE separation made things worse rather than better - which pure
// parallax can never do.
//
// Correction, in clip space, with l/r/u/d the frustum tangent extents:
//     ndc_x = (2x - (r+l)z) / ((r-l)z) = scaleX * x/z  +  centreX
// so with c0 producing clip.x and c3 producing clip.w (= z):
//     c0 += centreX * c3,   centreX = -(r+l)/(r-l)
// and likewise for c1/centreY vertically.
void ApplyFrustumAsymmetry(float m[16], const XRBridgeEyeView& view)
{
    const float tanL = std::tan(view.angleLeft);
    const float tanR = std::tan(view.angleRight);
    const float tanU = std::tan(view.angleUp);
    const float tanD = std::tan(view.angleDown);

    const float centreX = -(tanR + tanL) / (tanR - tanL);
    const float centreY = -(tanU + tanD) / (tanU - tanD);

    for (int i = 0; i < 4; ++i) {
        m[i] += centreX * m[12 + i];
        m[4 + i] += centreY * m[12 + i];
    }
}

// ---- 2026-09-11 VR hole fixes (A/B with '`') ---------------------------
// Headset screenshots showed dithered black patches on Sheva, foliage,
// rooftop fences and the gun - not missing objects. Three causes in this
// file's own logic, fixed below and toggled together so the difference can
// be judged in the headset:
//
//  1. The eye poses were read fresh on EVERY draw, while the XR submit
//     thread updates them on its own schedule. RE5 draws an object in
//     several passes that must agree on depth, so a pose that moved
//     between two passes of one frame made them disagree by a hair -
//     z-fighting holes. Now latched once per frame, at Present.
//  2. Every draw was split per eye, including draws into the shadow map -
//     rendering it from each eye's head-rotated view into two half-maps,
//     so shadow lookups on the real scene came back as garbage. Now only
//     screen-shaped render targets are split; shadow maps and other
//     off-screen targets draw once, untouched.
//  3. Screen-space (HUD) draws failed IsPlausibleCameraMatrix and fell back
//     to the old flat nudge, c0.w -/+ 2.75 - which for a pixel-space ortho
//     matrix (w = 1) is a 2.75 NDC shift, entirely off the screen: the HUD
//     was never visible in VR. Now squeezed into each eye's half instead.

bool IsScreenSpaceMatrix(const float m[16])
{
    // Affine (c3 = 0,0,0,1: no perspective divide) and pixel-scaled
    // (|c0| ~ 2/width). Identity stays on the camera path, as before.
    const bool affine = std::fabs(m[12]) < 1e-4f && std::fabs(m[13]) < 1e-4f && std::fabs(m[14]) < 1e-4f &&
        std::fabs(m[15] - 1.0f) < 1e-4f;
    return affine && Length3(&m[0]) < 0.05f;
}

void BuildScreenSpaceEyes(const float base[16], StereoDrawContext& ctx)
{
    for (int eye = 0; eye < 2; ++eye) {
        float* m = eye == 0 ? ctx.leftMatrix : ctx.rightMatrix;
        std::memcpy(m, base, sizeof(ctx.leftMatrix));
        for (int i = 0; i < 8; ++i) // c0 (x) and c1 (y): shrink toward the centre
            m[i] *= kScreenSpaceScale;
        FitEyeFovToViewportHalf(m, eye == 0);
    }
}

// Same aspect as the backbuffer (within 2%): full- and reduced-size scene
// buffers pass, square shadow maps don't.
bool RenderTargetIsScreenShaped(IDirect3DDevice9* pDevice)
{
    if (!g_backBufferWidth || !g_backBufferHeight)
        return true;
    IDirect3DSurface9* rt = nullptr;
    if (FAILED(pDevice->GetRenderTarget(0, &rt)) || !rt)
        return true;
    D3DSURFACE_DESC d = {};
    const bool haveDesc = SUCCEEDED(rt->GetDesc(&d));
    rt->Release();
    if (!haveDesc || !d.Width || !d.Height)
        return true;
    // Aspect alone is not enough (2026-09-12). The post-process chain -
    // bright-pass, bloom, light shafts - renders into buffers that are a
    // half or a quarter of the backbuffer but keep its ASPECT, so they
    // sailed through this test and got scissor-split per eye like real
    // scene geometry. A radial light-shaft blur reading from a source that
    // has been shifted and clipped in half produces exactly the hard-edged
    // bright wedges the user reported as "light leaks" - present with
    // culling on or off, wide FOV or not, because it was never a culling
    // problem. Anything meaningfully smaller than the backbuffer is now
    // drawn ONCE, untouched, and composited mono - user-confirmed to remove
    // the leaks, twice: when it first landed, and again on 2026-09-12 after
    // the flag had been left off in a working tree. '/' puts the per-eye
    // splitting back for comparison.
    if (g_monoSmallTargets.load(std::memory_order_relaxed)) {
        const bool fullSize = d.Width * 10 >= g_backBufferWidth * 9 && d.Height * 10 >= g_backBufferHeight * 9;
        if (!fullSize) {
            CountOffscreenTarget(d.Width, d.Height);
            return false;
        }
    }

    const float rtAspect = static_cast<float>(d.Width) / static_cast<float>(d.Height);
    const float bbAspect = static_cast<float>(g_backBufferWidth) / static_cast<float>(g_backBufferHeight);
    if (std::fabs(rtAspect - bbAspect) <= 0.02f * bbAspect)
        return true;
    CountOffscreenTarget(d.Width, d.Height);
    return false;
}

bool GetEyeViewsForDraw(XRBridgeEyeView& outLeft, XRBridgeEyeView& outRight)
{
    if (g_frameFixes && g_haveFrameViews && GetTickCount64() - g_lastPresentMs < kPresentStaleMs) {
        outLeft = g_frameViews[0];
        outRight = g_frameViews[1];
        return true;
    }
    return VRBridge_GetEyeViews(outLeft, outRight);
}

bool BeginStereoDraw(IDirect3DDevice9* pDevice, StereoDrawContext& ctx)
{
    // Flat while the film is on: one camera, one picture, no eye offset.
    if (!g_enabled || g_suppressed || g_inTheatre) {
        g_stereoPath = kStereoPathOff;
        return false;
    }

    float baseMatrix[16];
    if (!ConstantProbe_GetCachedCameraMatrix(baseMatrix)) {
        g_stereoPath = kStereoPathNoMatrix;
        return false;
    }

    XRBridgeEyeView leftView, rightView;
    const bool haveEyeViews = GetEyeViewsForDraw(leftView, rightView);
    if (g_frameFixes && haveEyeViews) {
        if (!RenderTargetIsScreenShaped(pDevice)) {
            g_stereoPath = kStereoPathOffscreen;
            return false; // shadow map etc: one untouched draw
        }
        if (IsScreenSpaceMatrix(baseMatrix)) {
            BuildScreenSpaceEyes(baseMatrix, ctx);
            ++g_census.screenSpace;
            g_stereoPath = kStereoPathScreenSpace;
            return true;
        }
    }
    if (!haveEyeViews || !IsPlausibleCameraMatrix(baseMatrix)) {
        g_stereoPath = kStereoPathNudged;
        if (haveEyeViews)
            ++g_census.nudged;
        // No HMD pose yet (VR mode off, or headset not ready), or this
        // particular draw call's matrix doesn't look like a real 3D
        // camera (likely a UI/HUD element reusing the same register for a
        // different kind of transform) - fall back to the original flat
        // nudge, which is safe even against a non-camera matrix.
        std::memcpy(ctx.leftMatrix, baseMatrix, sizeof(ctx.leftMatrix));
        ctx.leftMatrix[3] -= g_halfSeparation;
        std::memcpy(ctx.rightMatrix, baseMatrix, sizeof(ctx.rightMatrix));
        ctx.rightMatrix[3] += g_halfSeparation;
        return true;
    }

    // Real OpenXR path: rotate each eye's basis by its own recentered
    // rotationDelta (already includes real lens toe-in - see
    // openxr_bridge.h), keep the existing flat g_halfSeparation nudge for
    // position (no confirmed meters-to-render-units conversion exists yet
    // to use positionMeters directly - see g_halfSeparation's own header
    // comment), and replace the symmetric fallback FOV with the real
    // asymmetric per-eye angles OpenXR reports every frame.
    CameraBasis baseBasis;
    DecomposeCameraMatrix(baseMatrix, baseBasis);


    // Once per frame, at its first stereo draw: which head pose steered the
    // game camera this frame is drawn from (see g_pictureTurnMode).
    static ULONGLONG s_matchFrameMs = 0;
    static bool s_frameMatched = false;
    static float s_frameCamHeadForward[3] = { 0.0f, 0.0f, 1.0f };
    static float s_frameCamCameraForward[3] = { 0.0f, 0.0f, 1.0f };
    const int frameTurnMode = g_pictureTurnMode.load(std::memory_order_relaxed);
    if (g_lastPresentMs != s_matchFrameMs) {
        s_matchFrameMs = g_lastPresentMs;
        s_frameMatched = false;
        g_frameCameraPoseId = 0;
        HeadFollowTargets matched{};
        int matchAge = 0;
        float matchErr = 0.0f;
        // Compositor only has nothing to match: the camera is never steered by
        // the head, so there is no camera pose to take back out of the eyes.
        if (frameTurnMode != kPictureTurnDouble && frameTurnMode != kPictureTurnCompositorOnly &&
            CameraRigHook_MatchHeadFollowTargets(baseBasis.forward, matched, &matchAge, &matchErr) &&
            matched.poseId != 0) {
            s_frameMatched = true;
            std::memcpy(s_frameCamHeadForward, matched.headForward, sizeof(s_frameCamHeadForward));
            std::memcpy(s_frameCamCameraForward, matched.cameraForward, sizeof(s_frameCamCameraForward));
            // Camera only: the picture IS that camera, so tag the frame with its
            // pose. Catch-up: the picture is turned on to the latched pose, so the
            // latch's own tag (the default at Present) is the truth.
            if (frameTurnMode == kPictureTurnCameraOnly)
                g_frameCameraPoseId = matched.poseId;

            // Diagnostics: how old the camera's pose is against the latch, which
            // camera update the frame was drawn from, and how big the catch-up is.
            static long long s_sum = 0, s_min = 0, s_max = 0;
            static unsigned s_n = 0;
            static ULONGLONG s_windowMs = 0;
            static unsigned s_ageHist[4] = {};
            static float s_errMax = 0.0f;
            static double s_catchSum = 0.0;
            static float s_catchMax = 0.0f;
            ++s_ageHist[matchAge < 3 ? matchAge : 3];
            s_errMax = matchErr > s_errMax ? matchErr : s_errMax;
            {
                const float* a = matched.headForward;
                const float* b = leftView.rotationDelta + 6;
                const float la = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
                const float lb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
                float c = la > 1e-4f && lb > 1e-4f ? (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb) : 1.0f;
                c = c < -1.0f ? -1.0f : (c > 1.0f ? 1.0f : c);
                const float catchDeg = std::acos(c) * 57.2957795f;
                s_catchSum += catchDeg;
                s_catchMax = catchDeg > s_catchMax ? catchDeg : s_catchMax;
            }
            const long long d = static_cast<long long>(matched.poseId) - static_cast<long long>(g_frameViewsPoseId);
            if (!s_n)
                s_min = s_max = d;
            s_sum += d;
            s_min = d < s_min ? d : s_min;
            s_max = d > s_max ? d : s_max;
            ++s_n;
            const ULONGLONG nowMs = GetTickCount64();
            if (!s_windowMs)
                s_windowMs = nowMs;
            if (nowMs - s_windowMs >= 3000) {
#if RE5VR_DIAGNOSTICS
                Log_Printf("StereoTest: %s - camera pose vs latch %+.1f headset frames on average (min %+lld, max %+lld, "
                           "%u frames); drawn from camera update newest %u, 1 back %u, 2 back %u, older %u; worst "
                           "direction match %.1f deg; head moved on since the camera's pose %.1f deg avg, %.1f max",
                    frameTurnMode == kPictureTurnCatchUp ? "catch-up"
                        : (frameTurnMode == kPictureTurnDoubleFixed ? "double, culling fixed" : "camera only"),
                    static_cast<double>(s_sum) / s_n,
                    s_min, s_max, s_n, s_ageHist[0], s_ageHist[1], s_ageHist[2], s_ageHist[3], s_errMax,
                    s_catchSum / s_n, s_catchMax);
#endif
                s_windowMs = nowMs;
                s_sum = s_min = s_max = 0;
                s_n = 0;
                s_ageHist[0] = s_ageHist[1] = s_ageHist[2] = s_ageHist[3] = 0;
                s_errMax = 0.0f;
                s_catchSum = 0.0;
                s_catchMax = 0.0f;
            }
        }
    }

    // Staying in the body during an action (2026-09-23, user: "we still get
    // pulled out during actions, like kicks"). Second pass: the test used to be
    // "the camera rig hook has gone quiet", and the user still reported being
    // "definitely popped out of my camera during actions, but was actually held
    // in place during one of the cutscenes" - which is exactly what that test
    // does. A cutscene takes the camera away for seconds and trips it; a kick
    // does not stop the rig at all, so nothing ever fired.
    //
    // There is a better test right here. The eye below is decoded from the
    // matrices the renderer was handed, and the skeleton says where the eye
    // belongs. While the view is on the character those two are the same place,
    // so putting the eye "back" changes nothing; the moment anything swings the
    // camera out for a shot they part company, on the very first frame, and it
    // does not matter which camera the game used to do it.
    bool haveBodyEye = false;
    float bodyEye[3] = {};
    float awayUnits = 0.0f;
    if (CameraRigHook_GetBodyEye(baseBasis.forward, baseBasis.camPos, bodyEye)
        && bodyEye[0] == bodyEye[0] && bodyEye[1] == bodyEye[1] && bodyEye[2] == bodyEye[2]) {
        // The finite test is the belt to the braces above. Whatever else goes
        // wrong in the skeleton, a NaN place for your eye is never believed -
        // last time one reached the frustum planes and culled the world.
        haveBodyEye = true;
        // WITHOUT THE LEAD (2026-09-26, user: "but why is this pitch an issue
        // to begin with, if our camera is locked on the body - its not
        // entirely, we have to add the running lead to account for it").
        //
        // Exactly that. The eye is the neck plus a running lead of four ticks
        // of the character own travel, which exists so the view does not land
        // in Chris neck when the skeleton we read is a tick behind. It is part
        // of where the eye is DRAWN and no part of where the body IS.
        //
        // Measuring the camera against the led-forward point means a sprint
        // manufactures the gap the test is hunting for: the faster you run the
        // further ahead we put the target, and at four ticks that is most of
        // the 99 units the stairs produced. The camera had not gone anywhere.
        // We moved the thing it was being compared to.
        float lead[3];
        CameraRigHook_GetRunLead(lead);
        const float d[3] = { bodyEye[0] - lead[0] - baseBasis.camPos[0], bodyEye[1] - lead[1] - baseBasis.camPos[1],
            bodyEye[2] - lead[2] - baseBasis.camPos[2] };
        awayUnits = Length3(d);
    }
    // With hysteresis, so one frame the rig missed cannot flicker the hold on
    // and off. Sixty units is about seven tenths of a metre: further apart than
    // the two placements ever drift, nearer than any camera that has pulled out
    // to watch.
    // AND SIXTY IS SEVENTY CENTIMETRES OF YOUR HEAD (2026-09-26, user: "just
    // want to confirm, you aren't allowing the action camera to move us at
    // all, right?").
    //
    // It was. Making the hold instant fixed the ramp but not the trigger, and
    // the trigger was the bigger half: departure is NOTICED by the camera
    // having already travelled sixty units from your eye, which is about seven
    // tenths of a metre of unrequested head movement before anything engages.
    //
    // Two ways in now. The game says so directly whenever it has taken the
    // camera for something scripted, and that costs nothing to believe. And
    // the distance trigger comes down to just above ordinary play: HeadLock
    // measures the camera sitting 18 to 19 units off the eye while nothing is
    // happening, so thirty is clear of the noise and a fifth of what it was.
    // AND THIRTY WAS FAR TOO TIGHT (2026-09-26, user: "our game camera is so
    // bad with the checkerboard now that even sprinting causes it").
    //
    // My doing. Dropping the trigger from sixty units to thirty made ordinary
    // sprinting count as the camera departing, so the eye held on the body
    // while the game camera ran ahead - and the culling frustum is still built
    // for the game camera, so everything between the two got cut. That is the
    // checkerboard, now firing constantly instead of occasionally.
    //
    // Back to sixty. The game's own scripted-camera signal stays, because that
    // one is free of false positives and is what actually stops an action
    // camera moving your head - but it is worth being honest that it has the
    // same cost while it is engaged, and the real fix is for the culling to
    // follow the eye rather than the camera.
    // AND A SPIKE IS NOT A DEPARTURE (2026-09-26, user: "the camera doesn't
    // want to stay within my body, sprinting while turning doesn't spin with
    // my body anymore - although what did work was keeping the camera in place
    // during stomps").
    //
    // Both halves of that are one fault. The scripted signal is right and is
    // what holds the camera through a stomp; the DISTANCE test is what misread
    // a sprinting turn. The eye is placed on the neck with a running lead and
    // the game camera swings behind on a hard turn, so the two part company for
    // a few frames at speed - and with the take now instant, a few frames is
    // enough to snap the view into a world-locked hold and leave it there,
    // which reads exactly as the body turning without you.
    //
    // A real departure lasts seconds; this lasts frames. So the distance
    // trigger has to persist before it counts. The scripted signal keeps its
    // instant take, because that one has no false positives and is the one
    // that matters for comfort.
    //
    // A non-finite distance is never a departure either. It meant a NaN had
    // got into the eye placement, and believing it held the camera on a
    // position that was itself NaN.
    static bool s_departed = false;
    static unsigned long long s_farSince = 0;
    // And even then it has to have MOVED the camera. A scripted camera that
    // has not gone anywhere costs nothing to ignore, and ignoring it is what
    // keeps a slow frame from stealing the view.
    const bool sane = haveBodyEye && awayUnits == awayUnits && awayUnits < 1e6f;
    const bool gameSaysSo = CameraRigHook_InScriptedCamera() && (!sane || awayUnits > 40.0f);
    // NINETY, BECAUSE STAIRS ARE SEVENTY (2026-09-26, user: "the camera still
    // pops out when sprinting up stairs").
    //
    // The false takes are gone - the log used to show them at 4 units and now
    // shows nothing under 60 - but a stair sprint genuinely does put the game
    // camera 66 to 75 units from a neck mounted eye, because the camera pitches
    // and lags on the climb. Melee, which is the case worth holding for, comes
    // in at 102 and above. There is clear air between the two, so the line goes
    // in it.
    // IS IT FOLLOWING YOU (2026-09-26, the log: stairs at 99 units, melee at
    // 102).
    //
    // There is no threshold between those, and I said there was. Distance is
    // simply the wrong question: a camera trailing you up a staircase and a
    // camera swinging out for a stomp both end up about a metre away, and only
    // one of them has stopped being your camera.
    //
    // What separates them is whether it is TRACKING. On the stairs you are
    // running and the camera is running with you - the gap opens because it
    // lags, so the two are moving together and their velocities nearly match.
    // In a stomp you are planted and the camera leaves on its own, so its
    // velocity has nothing to do with yours. That difference is large, obvious,
    // and does not care how far apart they happen to be.
    //
    // The distance test stays as the gate, because a camera that has not gone
    // anywhere is not worth thinking about either way.
    static float s_wasCam[3] = {}, s_wasMan[3] = {};
    static bool s_haveWas = false;
    static float s_mismatch = 0.0f;
    static unsigned long long s_lastVelMs = 0;
    float manPos[3] = {};
    const bool haveMan = CameraRigHook_GetPlayerWorldPos(manPos);
    {
        const unsigned long long msNow = GetTickCount64();
        const float dtv = s_lastVelMs ? (msNow - s_lastVelMs) * 0.001f : 0.0f;
        if (haveMan && dtv > 0.008f) {
            if (s_haveWas) {
                float diff = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    const float vCam = (baseBasis.camPos[k] - s_wasCam[k]) / dtv;
                    const float vMan = (manPos[k] - s_wasMan[k]) / dtv;
                    const float d = vCam - vMan;
                    diff += d * d;
                }
                diff = std::sqrt(diff);
                // Smoothed, because one frame of either can be noisy at 16 fps.
                s_mismatch = s_mismatch * 0.6f + diff * 0.4f;
            }
            for (int k = 0; k < 3; ++k) {
                s_wasCam[k] = baseBasis.camPos[k];
                s_wasMan[k] = manPos[k];
            }
            s_haveWas = true;
            s_lastVelMs = msNow;
        } else if (!s_lastVelMs) {
            s_lastVelMs = msNow;
        }
    }
    // Units a second of disagreement. A sprint carries both along together and
    // leaves very little; a camera going somewhere by itself leaves a lot.
    constexpr float kNotFollowing = 200.0f;
    const bool onItsOwn = !(s_mismatch == s_mismatch) || s_mismatch > kNotFollowing;
    const bool farNow
        = sane && (s_departed ? awayUnits > 40.0f : (awayUnits > 90.0f && onItsOwn));
    const unsigned long long nowDepart = GetTickCount64();
    if (!farNow)
        s_farSince = 0;
    else if (!s_farSince)
        s_farSince = nowDepart;
    // Already held: leave at once when it comes back, as before. Not yet held:
    // the camera has to stay away for a fifth of a second first.
    const bool farForLong = farNow && (s_departed || nowDepart - s_farSince >= 200);
    bool departed = gameSaysSo || farForLong;

    // SINCE WHEN (2026-09-25, user: "any way we can also have theatre mode for
    // in game cutscenes? Not just the pre-rendered stuff").
    //
    // The theatre already catches the cutscenes where the game TAKES the camera
    // and stops calling the rig - the log has it doing so. The ones it misses
    // are the scenes the engine plays with the rig still ticking: the camera
    // cuts about and frames two people talking, and as far as the rig is
    // concerned nothing has happened at all.
    //
    // But the measurement right here sees it plainly. The eye the renderer was
    // handed and the eye the skeleton says you have are in the same place while
    // the view is on you, and part company the instant anything swings the
    // camera out for a shot. How LONG they have been apart is the whole
    // difference between a vault and a scene, so keep it and let the theatre
    // ask.
    // Measured in ONE place, and not this one (2026-09-25, user: "when I got to
    // my body, it was still in theater mode").
    //
    // Because it was measured here. This function returns at the top while the
    // theatre is on - that is what makes the frame render flat - so the clock
    // that let the theatre IN stopped the instant it got in, and could never
    // say the camera had come back. A one-way door with no handle on the
    // inside. It lives in UpdateCameraAwayClock now, which runs from Present
    // whatever else is happening.

    // A real cutscene is left alone (2026-09-23, user: "when an actual
    // cutscene happens, it's also still locking me in a spot vs playing the
    // cutscene"). A kick or a stomp takes the camera for well under a second
    // and never stops the camera rig; a cutscene takes it away and keeps it,
    // which the rig notices by simply not being called. Held past a second and
    // a half of that, it is not an action worth refusing - it is the film, and
    // the film should play.
    {
        static unsigned long long s_quietSince = 0;
        const unsigned long long nowCut = GetTickCount64();
        if (!CameraRigHook_InScriptedCamera()) {
            s_quietSince = 0;
        } else {
            if (!s_quietSince)
                s_quietSince = nowCut;
            if (nowCut - s_quietSince > 1500)
                departed = false;
            // A CUT WAS NOT ENOUGH, TESTED (2026-09-25). Releasing the hold on
            // two camera cuts made the action camera markedly WORSE at keeping
            // you in first person, so something in ordinary play is jumping the
            // camera more than a metre and a half between frames and being
            // counted as a change of shot. The counter stays - the theatre still
            // uses it, where a false positive costs a screen rather than your
            // viewpoint - but it does not get to hand your eye back.
        }
    }

    // And the view is held still while the game has it. Putting the eye back in
    // the body is only half of what makes an action jarring: the game also
    // swings the camera round for the shot, and a world that turns while your
    // head does not is the part that turns stomachs. So the heading the camera
    // had when it was taken is kept and the game's turning is refused - your
    // own head still moves the view exactly as it always does, because that is
    // applied per eye below. Yaw only: leaning the horizon in a headset is
    // worse than any camera move.
    // The heading is held for as long as the EASE lasts, not just as long as
    // the game has the camera (2026-09-23, user: "still snaps when regaining
    // control of the action camera"). The position was easing back over a fifth
    // of a second while the heading was let go in a single frame, so the whole
    // handover still landed as a snap - the eased half was simply the half
    // nobody notices. Both now come back on the same ramp, and the remembered
    // heading is only forgotten once that ramp has reached the bottom.
    static float s_heldYaw = 0.0f;
    static float s_heldForward[3] = { 0.0f, 0.0f, 1.0f };
    static bool s_holdingYaw = false;
    const float yawNow = std::atan2(baseBasis.forward[0], baseBasis.forward[2]);
    float holdYawCos = 1.0f, holdYawSin = 0.0f;
    if (departed) {
        s_holdingYaw = true;
        // The eye sits where the body was facing, not where the shot looks.
        CameraRigHook_GetBodyEye(s_heldForward, baseBasis.camPos, bodyEye);
    } else if (!s_holdingYaw) {
        // Following the game while it is behaving, and remembering where the
        // view was pointing in case it stops.
        s_heldYaw = yawNow;
        std::memcpy(s_heldForward, baseBasis.forward, sizeof(s_heldForward));
    }
    if (s_holdingYaw) {
        float back = s_heldYaw - yawNow;
        while (back > 3.14159265f)
            back -= 6.28318531f;
        while (back < -3.14159265f)
            back += 6.28318531f;
        holdYawCos = std::cos(back);
        holdYawSin = std::sin(back);
    }
#if RE5VR_DIAGNOSTICS
    if (departed != s_departed) {
        Log_Printf("StereoTest: the game %s the camera - it was %.0f units from where the body puts the eye, moving %.0f units a second differently from you",
            departed ? "took" : "gave back", awayUnits, s_mismatch);
        // WHICH OF THE TWO ACTUALLY MOVED (2026-09-26, from the pipeline read).
        //
        // Reading the camera's own position through every stage of its pipeline
        // during sprints, steps, ladders and jumps showed it sitting a steady
        // 165 to 175 units from the character the whole time - about two metres,
        // which is head height above his feet. It never went anywhere.
        //
        // But the departure detector was reporting 180 units at the same kind of
        // moment, and it measures the camera against the BODY EYE. Two things
        // can produce that, and they want opposite fixes: either the camera
        // moved, which the pipeline says it did not, or the body eye did. So
        // print both against the one reference neither of them derives from,
        // the character's own world position. A camera that stays near 170
        // while the gap is large means the eye is the thing that ran off.
        {
            float manAt[3];
            float leadNow[3];
            CameraRigHook_GetRunLead(leadNow);
            if (CameraRigHook_GetPlayerWorldPos(manAt)) {
                const float cm[3] = { baseBasis.camPos[0] - manAt[0], baseBasis.camPos[1] - manAt[1],
                    baseBasis.camPos[2] - manAt[2] };
                const float em[3] = { bodyEye[0] - manAt[0], bodyEye[1] - manAt[1], bodyEye[2] - manAt[2] };
                Log_Printf("StereoTest:   the camera is %.0f from the man, the body eye is %.0f from him, and the "
                           "lead is %.0f",
                    Length3(cm), Length3(em), Length3(leadNow));
            }
        }
    }
#endif
    s_departed = departed;

    // Handed over rather than snapped (2026-09-23, user: "it does 'snap' when
    // you regain control rather than a smooth takeover"). Taking the view is
    // quick, because whatever is happening has already started; giving it back
    // is slower, because nothing is happening any more and there is time to be
    // gentle about it. About a twentieth of a second in, a fifth of a second
    // out.
    // THE HANDOVER (2026-09-25, user: "it's not really a smooth transition back
    // to my control. It's disorienting").
    //
    // Two things wrong with what was here. It moved a share of the REMAINING
    // distance each frame, which is an exponential: it starts at full speed and
    // crawls at the end, so the eye leaves fast enough to feel like a shove and
    // then takes an age over the last centimetre. And the share was per FRAME,
    // so the whole thing ran twice as fast at 120 as at 60 - the disorientation
    // literally depended on the frame rate.
    //
    // A fixed duration with a smoothstep over it instead. Smoothstep leaves and
    // arrives at zero speed, so there is no jerk at either end, and seconds are
    // seconds whatever the machine is doing. Out is slower than in, because
    // being taken somewhere is expected and being handed back is not.
    static float s_holdT = 0.0f;
    {
        static LARGE_INTEGER s_qpf = {};
        static LARGE_INTEGER s_last = {};
        if (!s_qpf.QuadPart)
            QueryPerformanceFrequency(&s_qpf);
        LARGE_INTEGER nowQ;
        QueryPerformanceCounter(&nowQ);
        float dt = 1.0f / 90.0f;
        if (s_last.QuadPart && s_qpf.QuadPart)
            dt = static_cast<float>(nowQ.QuadPart - s_last.QuadPart) / static_cast<float>(s_qpf.QuadPart);
        s_last = nowQ;
        // A hitch must not teleport the eye across the room.
        if (!(dt > 0.0f))
            dt = 0.0f;
        if (dt > 0.05f)
            dt = 0.05f;
        // AND I HAD THE OLD CODE BACKWARDS (2026-09-25, user: "the action camera
        // was worse because I'm no longer staying in first person, I think that
        // fix was due to the one that was dealing with the camera leaving the
        // body" - correct on both counts).
        //
        // I called the old line a slow exponential. It was not. This runs once
        // per DRAW CALL, thousands of times a frame, so moving a quarter of the
        // remaining distance each time reached 1.0 inside a single frame. It was
        // effectively instant, and being instant is exactly why the camera never
        // appeared to leave your body.
        //
        // Making it honestly time-based was right; the third of a second was
        // not. That is a third of a second of the camera visibly pulling out on
        // every kick, which never used to happen. Taking the view is as close to
        // instant as a smoothstep can be, and only the handing back is gentle,
        // which was the actual complaint.
        // AND THE TAKE IS NOT A RAMP AT ALL (2026-09-26, user: "we've got to
        // stop action camera from moving your head at all - that's what causes
        // a number of issues and discomfort, I'm almost sure of it").
        //
        // Eighty milliseconds sounded instant when I wrote it and is not: at
        // 90 Hz it is seven frames during which your eye is still following a
        // camera that has started swinging away, because the hold has not
        // finished coming up yet. Seven frames of unrequested head movement is
        // exactly the thing that makes people ill, and it happens on every
        // kick, every stomp and every vault.
        //
        // There is also no reason for it to be gradual. Ramping IN means
        // easing your head part of the way somewhere it should never go; the
        // smoothness only ever mattered on the way back, which is where the
        // original complaint was. So the hold snaps on and eases off.
        constexpr float kGiveBackSec = 0.60f;
        if (departed)
            s_holdT = 1.0f;
        else
            s_holdT -= dt / kGiveBackSec;
        if (s_holdT > 1.0f)
            s_holdT = 1.0f;
        if (s_holdT < 0.0f)
            s_holdT = 0.0f;
    }
    const float s_hold = s_holdT * s_holdT * (3.0f - 2.0f * s_holdT);
    const bool inBody = s_hold > 0.0f && haveBodyEye;
    if (inBody) {
        for (int k = 0; k < 3; ++k)
            bodyEye[k] = baseBasis.camPos[k] + (bodyEye[k] - baseBasis.camPos[k]) * s_hold;
    }
    // The heading comes back on the same ramp as the position, and only when it
    // has run out is the held heading forgotten.
    {
        const float back = std::atan2(holdYawSin, holdYawCos) * s_hold;
        holdYawCos = std::cos(back);
        holdYawSin = std::sin(back);
    }
    if (s_hold <= 0.0f)
        s_holdingYaw = false;

    // Where your HEAD is, once, for the lean below (2026-09-23, user: "any idea
    // why lean and peek changes the world scale? It's unusable in its current
    // state"). The lean was being worked out per eye, from THAT eye's position,
    // against one shared reference - so the two eyes came out an IPD apart on
    // top of the IPD already applied, which is an eye separation of double and
    // a world of half the size. The stronger the lean, the smaller the world.
    // Both eyes lean by the same amount now, because a head is one thing.
    float headMeters[3] = { (leftView.positionMeters[0] + rightView.positionMeters[0]) * 0.5f,
        (leftView.positionMeters[1] + rightView.positionMeters[1]) * 0.5f,
        (leftView.positionMeters[2] + rightView.positionMeters[2]) * 0.5f };

    auto buildEyeBasis = [&](const XRBridgeEyeView& view, bool leftEye) {
        CameraBasis basis = baseBasis;
        if (inBody) {
            std::memcpy(basis.camPos, bodyEye, sizeof(bodyEye));
            // Turn the basis back about the world's up axis by whatever the
            // game has swung it since it took the camera.
            const auto unturn = [&](float v[3]) {
                const float x = v[0] * holdYawCos + v[2] * holdYawSin;
                const float z = -v[0] * holdYawSin + v[2] * holdYawCos;
                v[0] = x;
                v[2] = z;
            };
            unturn(basis.right);
            unturn(basis.up);
            unturn(basis.forward);
        }

        // Head-follow compensation (2026-09-12). When F9 is steering the
        // game's camera, that camera ALREADY contains this rotation - the
        // measurement showed it tracking the head 1:1 - so rotating the eye
        // basis by it again turns the world roughly twice as far as the
        // wearer turns. That is consistent with the user needing to "move
        // slow to get it to feel smooth". While head-follow is driving, skip
        // the delta and let the game camera carry the rotation on its own.
        // '\' toggles this off to compare.
        // Decided once per frame from the camera this frame is actually drawn
        // with (s_frameMatched), not the live head-follow flag: the flag flips
        // the moment aiming starts or stops, a frame or two before the drawn
        // camera does, and could even flip between two draws of one frame.
        const bool headFollowDriving = s_frameMatched;
        Mat3 delta{};
        std::memcpy(delta.m, view.rotationDelta, sizeof(delta.m));
        if (!headFollowDriving || frameTurnMode == kPictureTurnDouble ||
            frameTurnMode == kPictureTurnCompositorOnly) {
            ApplyHeadRotation(basis, delta);
        } else if (frameTurnMode == kPictureTurnDoubleFixed) {
            // Before the first match, fall back to the latched head and its
            // doubled aim, which is what the camera is being turned by.
            float headAim[3], cameraAim[3];
            if (s_frameMatched) {
                std::memcpy(headAim, s_frameCamHeadForward, sizeof(headAim));
                std::memcpy(cameraAim, s_frameCamCameraForward, sizeof(cameraAim));
            } else {
                std::memcpy(headAim, leftView.rotationDelta + 6, sizeof(headAim));
                CameraRigHook_DoubleHeadAim(headAim, cameraAim);
            }
            Mat3 aHead{}, aCamera{};
            if (NoRollAim(headAim, aHead) && NoRollAim(cameraAim, aCamera))
                ApplyHeadRotation(basis, Mat3Multiply(delta, Mat3Multiply(aHead, Mat3Transpose(aCamera))));
            else
                ApplyHeadRotation(basis, delta);
        } else {
            // The camera already carries a head aim. Take that aim back out of
            // this eye's delta and apply the rest:
            //  - catch-up: the aim the camera was steered with, so what remains
            //    is how far the head has moved on since, plus roll and this
            //    eye's own angle;
            //  - camera only (or catch-up before a match): the latched aim, so
            //    only roll and the eye's own angle remain. Without this the
            //    frame is tagged with a rolled pose but drawn level, and the
            //    compositor tilts the world WITH your head (user: "rolling your
            //    head is inverted").
            // What the camera was ACTUALLY steered with, which is not the same
            // as the head pose it was built from once head steadying is on
            // (2026-09-18). Taking the raw head out of a camera that was
            // turned by a steadied one would leave the difference in the
            // picture; taking the steadied one out puts that difference back
            // on the eyes, so catch-up stays 1:1 with your neck while the game
            // camera underneath it stays still. With steadying off the two are
            // identical and this changes nothing.
            const float* aimFrom = frameTurnMode == kPictureTurnCatchUp && s_frameMatched
                ? s_frameCamCameraForward
                : leftView.rotationDelta + 6;
            Mat3 aim{};
            if (NoRollAim(aimFrom, aim))
                ApplyHeadRotation(basis, Mat3Multiply(delta, Mat3Transpose(aim)));
        }

        const float offset = leftEye ? -g_halfSeparation : g_halfSeparation;
        for (int i = 0; i < 3; ++i)
            basis.camPos[i] += basis.right[i] * offset;

        // Leaning (2026-09-17). The headset's position has been published in
        // metres all along and nothing has ever used it: the eye sat wherever
        // the character's head joint was, so leaning out from behind a corner
        // moved your real head and nothing else. Only the DELTA from where you
        // were when the reference was taken is applied, so sitting differently
        // in a chair doesn't shove the camera through a wall.
        //
        // Metres become game units through the same constant the world scale
        // implies: half an IPD is 0.0318 m, and g_halfSeparation is that same
        // half IPD expressed in the game's units, so their ratio is the game's
        // units per metre. Tying it to world scale means leaning stays honest
        // when the world is made bigger or smaller.
        //
        // Applied along the BASE camera's axes, not the head-rotated eye's:
        // your head's movement is measured in the room, so lean left and you
        // go left relative to the character, whichever way you happen to be
        // looking at the time. OpenXR is +X right, +Y up, -Z forward.
        if (g_headPositionTracking) {
            // The reference has to come from a pose that means something
            // (2026-09-17, user: "lean and peek has my camera way high in the
            // air"). It was taken on the very first call, which can land before
            // tracking has produced anything, so the reference was zero and
            // every frame after it read as having stood up 1.7 metres - about
            // 150 game units straight upward. A position of exactly zero on all
            // three axes is not somebody sitting at the origin, it is no data.
            const bool poseMeansSomething = headMeters[0] != 0.0f || headMeters[1] != 0.0f
                || headMeters[2] != 0.0f;
            // And is anybody WEARING it? (2026-09-24, user: "if that setting is
            // enabled when first entering VR, the camera starts way too high and
            // you have to toggle the setting after you get into VR".)
            //
            // The space is XR_REFERENCE_SPACE_TYPE_LOCAL, whose origin is
            // wherever the headset was when the session started. If the session
            // starts while it is sitting on a desk, the reference is taken down
            // there, and putting it on reads as having stood up the best part of
            // a metre - about eighty units straight upward. Toggling the setting
            // retakes the reference while it is on your head, which is why that
            // works and why it has to be done every time.
            //
            // Rejecting a pose of exactly zero was not enough: a headset on a
            // desk reports a perfectly good position, it is just not yours. A
            // headset ON somebody moves - breathing is enough - and one lying on
            // a surface does not. So wait for a centimetre of accumulated
            // movement before believing the pose is a person.
            static float s_lastHead[3] = {};
            static bool s_haveLastHead = false;
            static float s_stirred = 0.0f;
            if (poseMeansSomething) {
                if (s_haveLastHead) {
                    const float mx = headMeters[0] - s_lastHead[0];
                    const float my = headMeters[1] - s_lastHead[1];
                    const float mz = headMeters[2] - s_lastHead[2];
                    s_stirred += std::sqrt(mx * mx + my * my + mz * mz);
                }
                std::memcpy(s_lastHead, headMeters, sizeof(s_lastHead));
                s_haveLastHead = true;
            }
            if (!g_leanReferenceSet && poseMeansSomething && s_stirred > 0.01f) {
                std::memcpy(g_leanReference, headMeters, sizeof(g_leanReference));
                g_leanReferenceSet = true;
                Log_Printf("Lean: taking your standing height from here - the headset has moved, so somebody "
                           "is wearing it");
            }
            if (!g_leanReferenceSet)
                return basis;
            float ref[3];
            std::memcpy(ref, g_leanReference, sizeof(ref));
            // And if the gap is bigger than a person, the reference is stale
            // rather than the player athletic - a runtime recentre or a new
            // session moves the origin under us. Take it again.
            const float gapX = headMeters[0] - ref[0], gapY = headMeters[1] - ref[1],
                        gapZ = headMeters[2] - ref[2];
            if (gapX * gapX + gapY * gapY + gapZ * gapZ > 2.25f) { // 1.5 m
                std::memcpy(g_leanReference, headMeters, sizeof(g_leanReference));
                std::memcpy(ref, g_leanReference, sizeof(ref));
            }
            // Height finds its own level. Belt and braces for the above, and
            // worth having on its own: nobody plays a whole session held twenty
            // centimetres off their own neutral, so a vertical offset that
            // PERSISTS is a reference that is wrong, not a player who is
            // crouching. Standing up out of a chair is the same thing and wants
            // the same answer.
            //
            // Only the vertical, and only after three quarters of a second.
            // Leaning out from behind a corner and holding it is a real thing to
            // do and must not drift out from under you; ducking for a second
            // still ducks, because the delay outlasts it.
            {
                static unsigned long long s_offSince = 0;
                static unsigned long long s_lastMs = 0;
                const unsigned long long nowY = GetTickCount64();
                const float dead = 0.18f;
                const float off = headMeters[1] - g_leanReference[1];
                if (std::fabs(off) <= dead) {
                    s_offSince = 0;
                } else {
                    if (!s_offSince) {
                        s_offSince = nowY;
                        s_lastMs = nowY;
                    }
                    if (nowY - s_offSince > 750) {
                        const float secs = (nowY - s_lastMs) / 1000.0f;
                        const float step = secs * 1.2f; // metres a second
                        const float want = off > 0.0f ? off - dead : off + dead;
                        const float move = std::fabs(want) < step ? want : (want > 0.0f ? step : -step);
                        g_leanReference[1] += move;
                        ref[1] = g_leanReference[1];
                        static unsigned long long s_toldY = 0;
                        if (nowY - s_toldY >= 2000) {
                            s_toldY = nowY;
                            Log_Printf("Lean: you have been %.0f cm off your own height for a while, so that is "
                                       "your height now",
                                off * 100.0f);
                        }
                    }
                    s_lastMs = nowY;
                }
            }
            const float unitsPerMetre = g_halfSeparation > 0.01f ? g_halfSeparation / 0.0318f : 86.0f;
            const float scale = unitsPerMetre * g_leanScale;
            const float dx = (headMeters[0] - ref[0]) * scale;
            const float dy = (headMeters[1] - ref[1]) * scale;
            const float dz = (headMeters[2] - ref[2]) * scale;
            for (int i = 0; i < 3; ++i)
                basis.camPos[i] += baseBasis.right[i] * dx + baseBasis.up[i] * dy - baseBasis.forward[i] * dz;
            if (leftEye) {
                for (int i = 0; i < 3; ++i)
                    g_leanWorld[i] = baseBasis.right[i] * dx + baseBasis.up[i] * dy - baseBasis.forward[i] * dz;
                g_haveLeanWorld = true;

                // Roomscale (2026-09-23). Second pass, and the first one was
                // wrong in two ways that the user felt immediately: "roomscale
                // completely disabled my ability to walk anymore with the left
                // stick", and "lean and peek has a huge swivel to it".
                //
                // The swivel was the worse of the two, because it happened even
                // with roomscale switched off: this whole block ran whenever
                // leaning was on, quietly moving the reference your lean is
                // measured from every time the character took a step. Walk
                // anywhere and your lean offset grew without you having moved
                // at all. It only runs when roomscale is on now.
                //
                // The stick fight was a servo that could not tell the two apart.
                // It credited the character with ALL of his movement, so pushing
                // the stick forward made him walk away from the reference, which
                // opened a gap behind him, which asked him to walk back. You
                // were fighting your own feet. He is only credited with movement
                // in the direction WE asked him to go, so a step you commanded
                // with your thumb leaves the reference exactly where it was and
                // costs nothing.
                if (XrInput_GetSettings().roomStep) {
                    static float s_hisPos[3] = {};
                    static bool s_hadHisPos = false;
                    float hisPos[3];
                    if (CameraRigHook_GetPlayerWorldPos(hisPos)) {
                        if (s_hadHisPos && unitsPerMetre > 1.0f) {
                            const float moved[3] = { hisPos[0] - s_hisPos[0], hisPos[1] - s_hisPos[1],
                                hisPos[2] - s_hisPos[2] };
                            // A cut, a load or a teleport is not a step.
                            if (Length3(moved) < 60.0f) {
                                const float alongRight = moved[0] * baseBasis.right[0]
                                    + moved[1] * baseBasis.right[1] + moved[2] * baseBasis.right[2];
                                const float alongFwd = moved[0] * baseBasis.forward[0]
                                    + moved[1] * baseBasis.forward[1] + moved[2] * baseBasis.forward[2];
                                // Only the part of it that went the way we asked.
                                const float askLen
                                    = std::sqrt(g_roomAsk[0] * g_roomAsk[0] + g_roomAsk[1] * g_roomAsk[1]);
                                if (askLen > 0.01f) {
                                    const float ux = g_roomAsk[0] / askLen, uf = g_roomAsk[1] / askLen;
                                    float ours = alongRight * ux + alongFwd * uf;
                                    if (ours > 0.0f) {
                                        g_leanReference[0] += (ux * ours) / unitsPerMetre;
                                        g_leanReference[2] -= (uf * ours) / unitsPerMetre;
                                    }
                                }
                            }
                        }
                        std::memcpy(s_hisPos, hisPos, sizeof(hisPos));
                        s_hadHisPos = true;
                    }
                }
                g_roomStep[0] = (headMeters[0] - ref[0]);
                g_roomStep[1] = -(headMeters[2] - ref[2]);
                g_haveRoomStep = true;
            }

            // Still in the air after the reference fix, so the numbers
            // themselves have to be looked at rather than reasoned about.
            if (leftEye) {
                static ULONGLONG s_ms = 0;
                const ULONGLONG now = GetTickCount64();
                if (now - s_ms > 1000) {
                    s_ms = now;
                    Log_Printf("Lean: head at (%.3f, %.3f, %.3f) m, reference (%.3f, %.3f, %.3f), so the view moves "
                               "(%.1f, %.1f, %.1f) units at %.1f units per metre",
                        headMeters[0], headMeters[1], headMeters[2], ref[0], ref[1],
                        ref[2], dx, dy, dz, unitsPerMetre);
                }
            }
        }

        // OpenXR's per-eye frustum is ASYMMETRIC (|angleLeft| != |angleRight|
        // - each eye sees further toward its own temple than toward the
        // nose). Scale from the actual tangent extents rather than from a
        // symmetric half-angle; ApplyFrustumAsymmetry below then restores
        // the off-centre part, which is the half that actually matters.
        const float tanL = std::tan(view.angleLeft);
        const float tanR = std::tan(view.angleRight);
        const float tanU = std::tan(view.angleUp);
        const float tanD = std::tan(view.angleDown);
        basis.scaleX = (2.0f / (tanR - tanL)) / g_fovWidenMultiplier;
        basis.scaleY = (2.0f / (tanU - tanD)) / g_fovWidenMultiplier;

        // How far this eye ended up from the camera the game thinks it is
        // drawing from, for the culling.
        {
            const float off[3] = { basis.camPos[0] - baseBasis.camPos[0], basis.camPos[1] - baseBasis.camPos[1],
                basis.camPos[2] - baseBasis.camPos[2] };
            const float far3 = Length3(off);
            if (far3 > g_eyeOffThisFrame)
                g_eyeOffThisFrame = far3;
            for (int k = 0; k < 3; ++k)
                g_eyeOffSum[k] += off[k];
        }
        return basis;
    };

    g_eyeOffThisFrame = 0.0f;
    g_eyeOffSum[0] = g_eyeOffSum[1] = g_eyeOffSum[2] = 0.0f;
    const CameraBasis leftBasis = buildEyeBasis(leftView, true);
    const CameraBasis rightBasis = buildEyeBasis(rightView, false);
    g_eyeOffUnits.store(g_eyeOffThisFrame, std::memory_order_relaxed);
    // The head, which is the average of the two eyes - the culling wants one
    // frustum, not one per eye, and half an IPD either side of it is far
    // smaller than the margin the planes get widened by anyway.
    // WHAT THE TWO EYES ARE ACTUALLY DOING (2026-09-26, user: "my eyes were
    // not aligning properly ... like my eyes were looking too far to the left
    // and right - straight when entering VR").
    //
    // Straight on entry and wrong later means something drifts, and guessing
    // which of half a dozen candidates it is has not worked. These are the only
    // three numbers that can describe it: how far apart the eyes are, how much
    // they are converging or diverging, and how much they are canted relative
    // to each other. A correct pair is a steady separation, near zero degrees
    // of convergence and exactly zero cant.
    {
        static unsigned long long s_saidAt = 0;
        const unsigned long long nowEye = GetTickCount64();
        if (nowEye - s_saidAt > 1000) {
            s_saidAt = nowEye;
            const float apart[3] = { rightBasis.camPos[0] - leftBasis.camPos[0],
                rightBasis.camPos[1] - leftBasis.camPos[1], rightBasis.camPos[2] - leftBasis.camPos[2] };
            const float sep = Length3(apart);
            // Convergence: the angle between the two forward vectors. Parallel
            // is zero; anything else and the eyes are pointing past or across
            // each other, which is what reads as cockeyed.
            float dotF = 0.0f, dotU = 0.0f;
            for (int k = 0; k < 3; ++k) {
                dotF += leftBasis.forward[k] * rightBasis.forward[k];
                dotU += leftBasis.up[k] * rightBasis.up[k];
            }
            if (dotF > 1.0f)
                dotF = 1.0f;
            if (dotF < -1.0f)
                dotF = -1.0f;
            if (dotU > 1.0f)
                dotU = 1.0f;
            if (dotU < -1.0f)
                dotU = -1.0f;
            const float convergeDeg = std::acos(dotF) * 57.2957795f;
            const float cantDeg = std::acos(dotU) * 57.2957795f;
            Log_Printf("EyeCheck: %.2f units apart (%.1f mm), converging %.3f deg, canted %.3f deg, each %.1f "
                       "units off the game camera",
                sep, sep / 85.8f * 1000.0f, convergeDeg, cantDeg, g_eyeOffThisFrame);
            // AND THE PROJECTION, WHICH IS WHAT IS LEFT (2026-09-26, user: "64
            // is in fact my IPD").
            //
            // Which closes the positional question for good: the eyes are
            // exactly where they should be, parallel, unrolled, and an IPD
            // apart. Nothing about where we put them can be what is wrong.
            //
            // The only other thing that differs between the two eyes is the
            // frustum. A headset lens is off centre, so OpenXR reports an
            // ASYMMETRIC one - |angleLeft| and |angleRight| are not equal, and
            // they are mirror images between the two eyes. Get that wrong, or
            // hand the left eye the right eye's angles, and the image in each
            // eye sits off to one side. Which is the complaint, in the words it
            // was made in.
            //
            // A healthy pair reads as mirrored: the left eye's centre negative
            // by as much as the right eye's is positive. Two centres with the
            // same sign means they are swapped or duplicated.
            const float leftCentreDeg = (leftView.angleLeft + leftView.angleRight) * 0.5f * 57.2957795f;
            const float rightCentreDeg = (rightView.angleLeft + rightView.angleRight) * 0.5f * 57.2957795f;
            Log_Printf("EyeCheck:   left eye %.1f to %.1f deg (centre %+.2f), right eye %.1f to %.1f deg "
                       "(centre %+.2f), widen %.2f",
                leftView.angleLeft * 57.2957795f, leftView.angleRight * 57.2957795f, leftCentreDeg,
                rightView.angleLeft * 57.2957795f, rightView.angleRight * 57.2957795f, rightCentreDeg,
                g_fovWidenMultiplier);
        }
    }

    g_eyeOffX.store(g_eyeOffSum[0] * 0.5f, std::memory_order_relaxed);
    g_eyeOffY.store(g_eyeOffSum[1] * 0.5f, std::memory_order_relaxed);
    g_eyeOffZ.store(g_eyeOffSum[2] * 0.5f, std::memory_order_relaxed);
#if RE5VR_DIAGNOSTICS
    // How far the eye is from the camera the game is culling against, while the
    // action camera has hold of it (2026-09-25, user: "I still have that weird
    // culling/checkboarding when an action happens").
    //
    // The frustum is pushed out by this plus twenty-five units, so if this reads
    // in the hundreds during an action then the margin is nowhere near enough
    // and geometry behind the real eye is being thrown away - which is what a
    // checkerboard of missing surfaces would be. It is measured here and used a
    // frame later, which is its own small problem if the number moves fast.
    if (g_eyeOffThisFrame > 5.0f) {
        static unsigned long long s_toldOff = 0;
        static float s_worstOff = 0.0f;
        if (g_eyeOffThisFrame > s_worstOff)
            s_worstOff = g_eyeOffThisFrame;
        const unsigned long long nowOff = GetTickCount64();
        if (nowOff - s_toldOff >= 1000) {
            s_toldOff = nowOff;
            Log_Printf("Culling: your eye is %.0f units from the camera being culled against (worst %.0f), so "
                       "the frustum wants %.0f of margin",
                g_eyeOffThisFrame, s_worstOff, g_eyeOffThisFrame + 25.0f);
            s_worstOff = 0.0f;
        }
    }
#endif
    {
        static ULONGLONG s_lastFrameMs = 0;
        if (g_lastPresentMs != s_lastFrameMs) {
            s_lastFrameMs = g_lastPresentMs;
            // On in release while the head-turn jitter is open (2026-09-16).
            // This is the measurement that says whether the picture turns by
            // the same amount the head does. Anything other than 1.00 is an
            // error that grows with how fast you turn, which is the symptom
            // testers describe: still is perfect, stick turning is perfect,
            // physically turning shakes.
            NoteViewVsGameCamera(leftBasis.forward, leftView.rotationDelta);
        }
    }

    ComposeCameraMatrix(leftBasis, ctx.leftMatrix);
    ComposeCameraMatrix(rightBasis, ctx.rightMatrix);

    ApplyFrustumAsymmetry(ctx.leftMatrix, leftView);
    ApplyFrustumAsymmetry(ctx.rightMatrix, rightView);

    FitEyeFovToViewportHalf(ctx.leftMatrix, true);
    FitEyeFovToViewportHalf(ctx.rightMatrix, false);

    // Log the real per-eye angles once - these have never actually been
    // looked at, and the gap between the two eyes' frustum centres is the
    // exact convergence error the correction above removes.
    static bool s_loggedFov = false;
    if (!s_loggedFov) {
        s_loggedFov = true;
        constexpr float kRadToDeg = 57.2957795f;
        const float leftCentre = (leftView.angleLeft + leftView.angleRight) * 0.5f * kRadToDeg;
        const float rightCentre = (rightView.angleLeft + rightView.angleRight) * 0.5f * kRadToDeg;
        Log_Printf("StereoTest: real per-eye FOV (deg) L[l=%.2f r=%.2f u=%.2f d=%.2f] R[l=%.2f r=%.2f u=%.2f d=%.2f]",
            leftView.angleLeft * kRadToDeg, leftView.angleRight * kRadToDeg,
            leftView.angleUp * kRadToDeg, leftView.angleDown * kRadToDeg,
            rightView.angleLeft * kRadToDeg, rightView.angleRight * kRadToDeg,
            rightView.angleUp * kRadToDeg, rightView.angleDown * kRadToDeg);
        Log_Printf("StereoTest: frustum centres L=%.2f deg R=%.2f deg -> eye-to-eye convergence error was %.2f deg (now corrected)",
            leftCentre, rightCentre, rightCentre - leftCentre);
    }
    ++g_census.split3d;
    g_stereoPath = kStereoPathCamera;
    return true;
}

// Pushes the given eye's camera matrix (via the real, un-hooked
// SetVertexShaderConstantF, so it doesn't corrupt the cached true matrix)
// and scissor-clips rendering to that eye's half of the current viewport.
void SetEyeState(IDirect3DDevice9* pDevice, const StereoDrawContext& ctx, bool leftEye)
{
    ConstantProbe_CallRealSetVertexShaderConstantF(pDevice, 0, leftEye ? ctx.leftMatrix : ctx.rightMatrix, 4);

    D3DVIEWPORT9 vp;
    pDevice->GetViewport(&vp);

    RECT r;
    r.top = static_cast<LONG>(vp.Y);
    r.bottom = static_cast<LONG>(vp.Y + vp.Height);
    if (leftEye) {
        r.left = static_cast<LONG>(vp.X);
        r.right = static_cast<LONG>(vp.X + vp.Width / 2);
    } else {
        r.left = static_cast<LONG>(vp.X + vp.Width / 2);
        r.right = static_cast<LONG>(vp.X + vp.Width);
    }

    pDevice->SetScissorRect(&r);
    pDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, TRUE);
}

void EndStereoDraw(IDirect3DDevice9* pDevice)
{
    pDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
}

// ---- HUD: one draw per eye through the viewport (2026-09-13) ------------
// The HUD recorder (hud_probe.cpp) showed why the HUD has never been visible
// in VR. Its shader does not read c0-c3, so those registers still hold the 3D
// scene's camera when it draws - which looks like a perfectly plausible camera
// - and every HUD draw took the per-eye CAMERA path: drawn twice, scissored to
// one half each time. The left eye got the left half of a full-screen HUD and
// the right eye the right half, so health and ammo, which sit at the screen
// edges, ended up in the outer corners of each eye image, outside the lenses.
//
// Nothing about the fix needs to know how the shader positions its vertices:
// D3D maps clip space into whatever viewport is set, AFTER the vertex shader.
// So each HUD draw is issued once into each eye's half, sized to keep the
// frame's aspect. No matrix writes.
//
// PLACEMENT (2026-09-13). The first version put the HUD at the same pixel spot
// in both halves, as if the centre of each half were straight ahead. It works
// - and it does not fuse: the user saw "two images next to each other, like
// going cross-eyed". Headset FOVs are ASYMMETRIC (each eye sees further out
// than in), so the two half-centres point in different directions, and the
// copies land at angles no pair of eyes can converge on.
//
// Instead the HUD is placed at a real distance straight ahead of the head,
// and each eye's copy goes where THAT point falls in THAT eye's projection,
// using the per-eye angles OpenXR reports every frame. A point at distance D
// ahead of the head centre sits ipd/2 to one side of each eye, so its tangent
// is +-(ipd/2)/D, and an asymmetric frustum maps a tangent t to
//     ndc = (2t - (tanRight + tanLeft)) / (tanRight - tanLeft).
// The two copies then converge exactly at D. Lens canting (toe-in) is ignored
// - fine for Quest-style parallel displays, may need the rotation on Index.
//
// Scissored to the eye's half, because the shifted viewport can overhang it.
// Both are menu options now (2026-09-13); these are the defaults.
// THE THEATRE (2026-09-25, user: "implement a 2d theatre mode - this would be
// applied during the main menu, and cutscenes so that you can see everything").
//
// Everything needed for this already existed, wearing a different name. Every
// draw is already sorted into one of three answers by ClassifyHudDraw: throw it
// away, put it on a flat panel hanging in front of you, or render it twice in
// stereo. The HUD has been going on that panel since the beginning, at a chosen
// distance and size, with the per-eye convergence worked out so the two copies
// fuse. A cinema screen is that panel with the whole film on it.
//
// So the theatre is not a new renderer. It is a fourth answer to the same
// question - "panel, please" - given for every draw instead of just the HUD,
// while the game is running its own camera anyway. The world arrives with the
// game's own flat framing, the HUD arrives with it, and nothing is dropped,
// which is the whole point: you can see everything.
//
// Wanted because the HUD is not fixed yet (the user: "I'd love to do a 3d main
// menu, but we haven't gone through and fixed the hud"). It earns its keep for
// cutscenes regardless - they are composed for a rectangle, and a camera that
// cuts and swings is far kinder watched on a screen than worn on your face.

float g_hudDistanceMeters = 2.0f; // how far away the HUD appears
float g_hudScale = 0.67f;         // HUD width as a fraction of one eye's half - the user's pick in the headset, 2026-09-13 (was 0.8)
constexpr float kFallbackIpdMeters = 0.063f;

float MeasuredIpd(bool haveViews, const XRBridgeEyeView* views)
{
    float ipd = kFallbackIpdMeters;
    if (haveViews) {
        const float d[3] = { views[1].positionMeters[0] - views[0].positionMeters[0],
            views[1].positionMeters[1] - views[0].positionMeters[1],
            views[1].positionMeters[2] - views[0].positionMeters[2] };
        const float measured = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (measured > 0.04f && measured < 0.09f)
            ipd = measured;
    }
    return ipd;
}

// Where a point straight ahead at distanceMeters lands in one eye's half of a
// frame at (x0, y0, w, h) - see the explanation above.
void EyeCentreForDistance(const XRBridgeEyeView* views, bool haveViews, float ipd, int eye, float distanceMeters,
    float x0, float y0, float w, float h, float& centreX, float& centreY)
{
    const float halfW = w * 0.5f;
    const float halfX0 = x0 + (eye ? halfW : 0.0f);
    centreX = halfX0 + halfW * 0.5f;
    centreY = y0 + h * 0.5f;
    if (!haveViews)
        return;
    const XRBridgeEyeView& v = views[eye];
    const float tl = std::tan(v.angleLeft), tr = std::tan(v.angleRight);
    const float tu = std::tan(v.angleUp), td = std::tan(v.angleDown);
    if (tr - tl <= 1e-3f || tu - td <= 1e-3f)
        return;
    // Left eye is ipd/2 to the left, so a point ahead of the head centre is
    // to its RIGHT (positive tangent), and vice versa.
    const float t = (eye == 0 ? 0.5f : -0.5f) * ipd / distanceMeters;
    const float ndcX = (2.0f * t - (tr + tl)) / (tr - tl);
    const float ndcY = -(tu + td) / (tu - td); // straight ahead vertically
    centreX = halfX0 + (ndcX + 1.0f) * 0.5f * halfW;
    centreY = y0 + (1.0f - ndcY) * 0.5f * h;
}

template <typename DrawFn>
HRESULT DrawHudPerEye(IDirect3DDevice9* pDevice, DrawFn draw)
{
    D3DVIEWPORT9 vp;
    if (FAILED(pDevice->GetViewport(&vp)) || vp.Width < 4 || vp.Height < 4)
        return draw();
    DWORD scissor = FALSE;
    if (FAILED(pDevice->GetRenderState(D3DRS_SCISSORTESTENABLE, &scissor)))
        scissor = FALSE;
    RECT oldScissor = {};
    pDevice->GetScissorRect(&oldScissor);

    XRBridgeEyeView views[2];
    const bool haveViews = GetEyeViewsForDraw(views[0], views[1]);
    const float ipd = MeasuredIpd(haveViews, views);

    const float halfW = vp.Width * 0.5f;
    const float hudW = halfW * g_hudScale;
    const float hudH = vp.Height * g_hudScale * 0.5f; // keeps the full frame's aspect

    HRESULT hr = D3D_OK;
    for (int eye = 0; eye < 2; ++eye) {
        const float halfX0 = vp.X + (eye ? halfW : 0.0f);
        float centreX = 0.0f, centreY = 0.0f;
        EyeCentreForDistance(views, haveViews, ipd, eye, g_hudDistanceMeters, static_cast<float>(vp.X),
            static_cast<float>(vp.Y), static_cast<float>(vp.Width), static_cast<float>(vp.Height), centreX, centreY);

        // D3D9 rejects a viewport that leaves the render target, so clamp to
        // the frame; the scissor keeps any overhang out of the other eye.
        float x = centreX - hudW * 0.5f, y = centreY - hudH * 0.5f;
        x = (std::max)(static_cast<float>(vp.X), (std::min)(x, vp.X + vp.Width - hudW));
        y = (std::max)(static_cast<float>(vp.Y), (std::min)(y, vp.Y + vp.Height - hudH));

        D3DVIEWPORT9 hud = vp;
        hud.X = static_cast<DWORD>(x + 0.5f);
        hud.Y = static_cast<DWORD>(y + 0.5f);
        hud.Width = static_cast<DWORD>(hudW + 0.5f);
        hud.Height = static_cast<DWORD>(hudH + 0.5f);
        pDevice->SetViewport(&hud);

        RECT half = { static_cast<LONG>(halfX0), static_cast<LONG>(vp.Y), static_cast<LONG>(halfX0 + halfW),
            static_cast<LONG>(vp.Y + vp.Height) };
        pDevice->SetScissorRect(&half);
        pDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, TRUE);

        const HRESULT r = draw();
        if (eye == 0)
            hr = r;
    }
    pDevice->SetViewport(&vp);
    pDevice->SetScissorRect(&oldScissor);
    pDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, scissor);
    return hr;
}

// Whether this draw should take the HUD path. Only while stereo is actually
// running - flat, the HUD draws exactly as the game intends.
// ---- The laser in the HUD (2026-09-13) ----------------------------------
// The user saw the laser sight both in the world and copied onto the flat HUD
// panel. One HUD vertex shader (1D73ACCB) is shared with a laser sprite.
//
// Ruled out: depth - every HUD-shader draw is depth-tested (LESSEQUAL, no
// write), so keeping depth-tested draws off the panel removed the whole HUD.
//
// Found by counting each kind of HUD-shader draw with the aim flag on and off:
// one kind was drawn 3722 times while aiming and 19 while not (the frames
// around raising and lowering the gun) - 4 primitives, additive blend
// (SRCALPHA/ONE), a 64x64 DXT5 texture. No other HUD-shader draw uses a 64x64
// texture. It is a flat glow sprite the game places on screen itself; the
// beam in the world is drawn separately and is unaffected, so the sprite is
// simply not drawn in stereo - on the panel it was only ever in the wrong place.
enum HudDrawClass { kNotHud, kHudPanel, kHudDrop };

bool IsLaserGlowSprite(IDirect3DDevice9* dev, UINT prims)
{
    (void)prims; // 4 in the census; not needed to tell it apart
    DWORD dst = 0;
    if (FAILED(dev->GetRenderState(D3DRS_DESTBLEND, &dst)) || dst != D3DBLEND_ONE)
        return false;
    IDirect3DBaseTexture9* base = nullptr;
    dev->GetTexture(0, &base);
    if (!base)
        return false;
    bool laser = false;
    IDirect3DTexture9* tex = nullptr;
    if (SUCCEEDED(base->QueryInterface(IID_IDirect3DTexture9, reinterpret_cast<void**>(&tex))) && tex) {
        D3DSURFACE_DESC d = {};
        laser = SUCCEEDED(tex->GetLevelDesc(0, &d)) && d.Width == 64 && d.Height == 64;
        tex->Release();
    }
    base->Release();
    return laser;
}

// Is the film on? Two occasions, and both are times the game has the camera and
// we do not want it.
//
//   * Nobody is the player. The main menu, and the loading that follows it.
//   * A cutscene. Judged exactly the way the action camera judges it, because
//     that test was already argued out once: a kick or a stomp takes the camera
//     for well under a second, a cutscene takes it and keeps it. Held past a
//     second and a half, it is the film. Anything shorter must NOT put a screen
//     in front of you - flipping to a cinema for a melee animation would be far
//     worse than the thing this fixes.
//
// Leaving waits four tenths of a second, so that the camera being handed back
// and taken again across a cut does not flicker the screen in and out.
bool TheatreNow()
{
    static unsigned long long s_askedMs = 0;
    static unsigned long long s_quietSince = 0;
    static unsigned long long s_leaveAt = 0;
    static bool s_on = false;
    const unsigned long long now = GetTickCount64();
    // Asked once every other frame at most; this runs per draw call otherwise.
    if (now - s_askedMs < 16 && s_askedMs)
        return s_on;
    s_askedMs = now;
    // NOT ON A MONITOR (2026-09-25, user: "let's not break our flatscreen
    // gamers").
    //
    // A real hole, and not a small one: the composite is called from Present
    // every frame, not from the draw path, so none of the checks that keep the
    // rest of this code away from a flat game were protecting it. On a monitor
    // it would have blacked the frame out and pasted two small copies of it
    // side by side - the whole game, unplayable, for somebody who never asked
    // for any of this.
    //
    // g_enabled is the honest question. It is what turns the side-by-side on,
    // set by the bridge when VR starts, and without it there are no eye halves
    // to paste into. Suppressed counts as off for the same reason.
    // Held on by hand until you are playing again (2026-09-25, user: "fuck the
    // automatic catch. Just a bind that turns it on", and "it would be nice to
    // have an automatic 'you're in game now' to turn it off").
    //
    // Turning it off is a far easier question than turning it on, which is why
    // this asymmetry is the right shape. "Is this a cutscene" has no reliable
    // answer - five tests tonight and the best of them made the action camera
    // worse. "Are you playing" does: the game is showing you your health and
    // ammo, and the camera is back on your body. Neither is true during anything
    // you would have put the screen up for.
    if (g_theatreForced && g_enabled && !g_suppressed) {
        static unsigned long long s_backSince = 0;
        const bool hudUp = g_hudGoneSince == 0;
        const bool onYourBody = g_cameraAwaySince == 0;
        if (hudUp && onYourBody) {
            if (!s_backSince)
                s_backSince = now;
        } else {
            s_backSince = 0;
        }
        // A second of both, so a single frame of HUD inside a scene does not
        // drop the screen in the middle of it.
        if (s_backSince && now - s_backSince > 1000) {
            s_backSince = 0;
            g_theatreForced = false;
            g_haveScreenAnchor = false;
            Log_Printf("Theatre: your HUD is back and the camera is on you, so the screen comes down");
        } else {
            if (!s_on) {
                s_on = true;
                Log_Printf("Theatre: on the screen because you asked for it");
            }
            s_leaveAt = 0;
            return true;
        }
    }
    if (!g_theatre || !g_enabled || g_suppressed) {
        s_on = false;
        s_quietSince = 0;
        g_haveScreenAnchor = false;
        return false;
    }
    if (!CameraRigHook_InScriptedCamera()) {
        // Say how long the game had the camera, every time it gives it back, so
        // the threshold above can be read off a list of real melees and real
        // scenes rather than guessed at again.
        if (s_quietSince) {
            const float heldSec = (now - s_quietSince) / 1000.0f;
            if (heldSec > 0.5f)
                Log_Printf("Theatre: the game held the camera for %.1f s - %s", heldSec,
                    heldSec > 4.0f ? "long enough to be a scene" : "too short, left alone");
        }
        s_quietSince = 0;
    } else if (!s_quietSince) {
        s_quietSince = now;
    }
    // FOUR SECONDS, AND I WAS WRONG ABOUT THIS ONE (2026-09-25).
    //
    // I told the user this test had never misfired. The log says it fired six
    // times in two minutes, and four of those lasted between one and three
    // seconds:
    //
    //   04:34:26.311 on - the game has taken the camera and kept it
    //   04:34:27.469 back in the world          (1.2 s)
    //   04:35:39.791 on
    //   04:35:40.959 back in the world          (1.2 s)
    //   04:36:09.434 on
    //   04:36:10.240 back in the world          (0.8 s)
    //
    // A cutscene is not over in eight tenths of a second. Those are the melees
    // and the window vaults, and the rig going quiet for a second and a half
    // does not tell them apart from a scene after all.
    //
    // It matters far more than a screen appearing when it should not, because
    // the arm solve stands down for a scripted camera too. Every one of those
    // flickers took the player's arms away and gave them back, which is exactly
    // what "my aim IK broke for a bit" looks like from inside a headset.
    const unsigned long long quietFor = s_quietSince ? now - s_quietSince : 0;
    const bool nobodyPlaying = CameraRigHook_GetPlayerController() == nullptr;
    // The engine's own scenes, where the rig keeps being called and the camera
    // simply leaves your body.
    //
    // The camera cannot say this on its own (2026-09-25, user: "there's a few
    // moments when theater mode kicks in and shouldn't, like longer melee
    // actions, or being revived"). Quite so: a long melee and a scene look
    // identical to a distance test, and no threshold separates them, because a
    // revive genuinely does keep the camera off you for several seconds. Any
    // number high enough to exclude a revive would start half the scenes late.
    //
    // The HUD draws the line the camera cannot. The game takes it away for a
    // scene and leaves it up for everything you do yourself - you still have
    // health and ammo to read while you are being revived, and none to read
    // during a conversation. So both have to agree: the view is off your body
    // AND the game has stopped telling you anything about yourself.
    // SIX AND FOUR (2026-09-25, user: "still kicks on at moments it shouldn't.
    // Like jumping out a window and other basic interactions. We want it to
    // fire on cutscenes only").
    //
    // Three seconds and one was not enough, and I have now guessed at these
    // numbers twice. The honest position is that this test is a safety net and
    // not the main way in - the scenes where the game TAKES the camera are
    // caught exactly by `film` above, which has never misfired. This one exists
    // for the scenes that keep the rig ticking, and being late to those is a
    // far smaller cost than turning a window vault into a cinema.
    //
    // So the thresholds go where nothing you DO can reach: six seconds with the
    // view off your body and four with nothing on screen about yourself. And
    // every episode is now logged with its real durations when it ends, so the
    // next pass at these numbers can be read off the log instead of guessed at
    // a third time.
    // Still the long stop, for the scenes that keep the rig ticking. Six
    // seconds because this signal cannot tell a scene from a revive, and a
    // revive is the longest thing you do that looks like one.
    const bool cameraGone = g_cameraAwaySince && now - g_cameraAwaySince > 6000;
    const bool hudGone = g_hudGoneSince && now - g_hudGoneSince > 1500;
    const bool acted = cameraGone && hudGone;
    // A CHEST AND A PAUSE MENU ARE NOT FILMS (2026-09-25, user: "I like the
    // in-game cutscene theater mode, but it fires like when opening a chest, or
    // pausing the game").
    //
    // Both of those look exactly like a cutscene to a camera test, and that is
    // why three passes at the thresholds never fixed it. Pause the game and
    // nothing updates, so the rig goes quiet, which is the entire definition of
    // `film`. Open a chest and the camera settles onto it and stays.
    //
    // But they have something in common that a real scene never does: the game
    // is showing you a menu. A pause screen is HUD, an inventory is HUD, a chest
    // is HUD. A cutscene has none, which is the whole point of a cutscene.
    //
    // That test was already being applied to the camera-distance path and not to
    // this one, for no better reason than the order they were written in. Both
    // want it. And with the HUD deciding it, the camera timings can come back
    // down, so a real scene reaches the screen quickly instead of playing its
    // first four seconds in 3D.
    // A chest reveal is one move with no HUD, so the HUD alone cannot refuse it.
    // A cut can. Two of them, so that the single jump into a shot does not count
    // on its own - or six seconds, because a scene long enough to be sat through
    // is a scene whether or not anybody bothered to cut it.
    // AND THE CUT CANNOT BE COMPULSORY (2026-09-25, user: "it no longer picks up
    // pre-rendered scenes. And doesn't really work to pick up in game scenes").
    //
    // My fault, and an obvious one in hindsight. Requiring a cut AND everything
    // else was an AND too far: a pre-rendered film does not move the 3D camera
    // at all, so there is nothing to detect a cut in, and the camera never
    // counts as having left your body either. Both of the things I made
    // compulsory are things a film cannot supply.
    //
    // So the cut is a reason to go EARLY, not a requirement. What every scene
    // does have is the game holding the camera with no HUD up, and the only
    // thing that shares that shape is a chest reveal, which is over in a few
    // seconds. Three seconds of it excludes the chest without excluding
    // anything else; a scene that has visibly cut between shots does not have
    // to wait that long.
    // AND NOTHING ELSE PUTS IT UP (2026-09-25, user: "fuck the automatic
    // catch").
    //
    // Fair. Five attempts: the rig going quiet, how long it stayed quiet, the
    // camera leaving your body, the HUD going away, and the camera cutting
    // between shots. Each one caught some scenes and some chests, vaults,
    // revives or pause menus, and the last one made the action camera worse. The
    // game does not distinguish a scene from an action in any way this code can
    // see, and pretending otherwise has cost a whole night.
    //
    // What is left is the one test that has never once been wrong - nobody is
    // the player, so this is the menu - and F10, pressed by somebody who can see
    // what is on the screen.
    //
    // The measurements all stay, and the watcher with them. If a pattern ever
    // does emerge in a log, it can be turned back on in one line.
    (void)quietFor;
    (void)hudGone;
    (void)acted;
    const bool want = nobodyPlaying;
#if RE5VR_DIAGNOSTICS
    // WHAT A PRE-RENDERED FILM LOOKS LIKE FROM IN HERE (2026-09-25, user: "the
    // theater mode still isn't catching pre-rendered cutscenes").
    //
    // Five theories have been spent on this behaviour tonight and three were
    // wrong, so this one gets measured instead. Whenever anything cutscene-like
    // is happening, every input the decision is made from goes in the log, once
    // a second. A film played through it will say in its own words which test it
    // fails - and the suspicion is the HUD count, because the movie is very
    // likely being drawn by a shader this code already calls HUD, which would
    // make "the HUD has gone" false for the whole film and block every path.
    if (quietFor > 1000 || g_cameraAwaySince) {
        static unsigned long long s_toldWatch = 0;
        if (now - s_toldWatch >= 1000) {
            s_toldWatch = now;
            Log_Printf("Theatre: watching - player %s, rig quiet %.1f s, %u HUD draw(s) last frame, HUD gone "
                       "%.1f s, view off your body %.1f s, %d cut(s) -> %s",
                nobodyPlaying ? "NONE" : "yes", quietFor / 1000.0f, g_hudDrawsLastFrame,
                g_hudGoneSince ? (now - g_hudGoneSince) / 1000.0f : 0.0f,
                g_cameraAwaySince ? (now - g_cameraAwaySince) / 1000.0f : 0.0f, g_cameraCuts,
                want ? "SCREEN" : "left alone");
        }
    }
#endif
    if (want) {
        s_leaveAt = 0;
        if (!s_on) {
            s_on = true;
            Log_Printf("Theatre: on the screen now - %s",
                "nobody is the player, so this is a menu or a film");
        }
    } else if (s_on) {
        if (!s_leaveAt)
            s_leaveAt = now;
        if (now - s_leaveAt > 400) {
            s_on = false;
            s_leaveAt = 0;
            // Taken again next time, so the screen is always in front of you
            // when it appears and never behind you from a cutscene ago.
            g_haveScreenAnchor = false;
            Log_Printf("Theatre: back in the world");
        }
    }
    return s_on;
}

HudDrawClass ClassifyHudDraw(IDirect3DDevice9* pDevice, UINT prims)
{
    if (!g_enabled || g_suppressed)
        return kNotHud;
    // In the theatre nothing is sorted at all: the game draws its ordinary flat
    // frame across the whole back buffer and the picture is moved onto the
    // screen afterwards, in one piece.
    //
    // Putting each draw on the panel individually was the first attempt and it
    // does not survive contact with the menu: the title and the menu text came
    // out perfectly, because those are real HUD draws that have been taking
    // this path for months, while the animated logo behind them arrived eight
    // times across the frame. Whatever those draws do with screen coordinates,
    // they do not survive being handed a different viewport, and there is no
    // reason to think they are the only ones.
    //
    // A finished frame has no such opinions. Copy it, clear the buffer, and put
    // it back twice in the right places, and it does not matter how any single
    // draw was authored.
    // Recognised by bytecode hash from launch (hud_shaders.cpp). The K capture
    // stays available in developer builds, for finding shaders not listed yet.
    const bool isHud = HudShaders_IsHudDraw(pDevice);
    // COUNTED FIRST, AND ALWAYS (2026-09-25). The theatre asks whether the HUD
    // is on screen, so this has to be answered while the theatre is running as
    // well - the same trap the camera clock fell into an hour ago, where the
    // thing that lets you in stops being measured the moment you are in.
    if (isHud)
        ++g_hudDrawsThisFrame;
    if (g_inTheatre)
        return kNotHud;
    if (isHud) {
        if (IsLaserGlowSprite(pDevice, prims)) {
            static bool logged = false;
            if (!logged) {
                logged = true;
                Log_Printf("StereoTest: laser glow sprite (HUD shader, 64x64 additive) kept off the HUD panel");
            }
            return kHudDrop;
        }
        return kHudPanel;
    }
#if RE5VR_DIAGNOSTICS
    return HudProbe_IsHudDraw(pDevice) ? kHudPanel : kNotHud;
#else
    return kNotHud;
#endif
}

typedef HRESULT(WINAPI* DrawPrimitive_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount);
DrawPrimitive_t oDrawPrimitive = nullptr;

HRESULT WINAPI hkDrawPrimitive(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount)
{
    HeadHideProbe_OnDrawCall(This, "DrawPrimitive", PrimitiveCount);
    PixelConstantProbe_OnDrawCall(This);
    FadeProbe_OnDraw(This, 0, PrimitiveType, static_cast<INT>(StartVertex), 0, 0, 0, PrimitiveCount);
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    const HudDrawClass hudClass = ClassifyHudDraw(This, PrimitiveCount);
    if (hudClass == kHudDrop)
        return D3D_OK;
    if (hudClass == kHudPanel) {
#if RE5VR_DIAGNOSTICS
        HudProbe_OnDraw(This, 0, PrimitiveType, PrimitiveCount, kStereoPathHudViewport);
#endif
        return DrawHudPerEye(This, [&] { return oDrawPrimitive(This, PrimitiveType, StartVertex, PrimitiveCount); });
    }

    StereoDrawContext ctx;
    const bool stereo = BeginStereoDraw(This, ctx);
#if RE5VR_DIAGNOSTICS
    HudProbe_OnDraw(This, 0, PrimitiveType, PrimitiveCount, g_stereoPath);
#endif
    if (!stereo)
        return oDrawPrimitive(This, PrimitiveType, StartVertex, PrimitiveCount);

    SetEyeState(This, ctx, true);
    HRESULT hr = oDrawPrimitive(This, PrimitiveType, StartVertex, PrimitiveCount);

    SetEyeState(This, ctx, false);
    oDrawPrimitive(This, PrimitiveType, StartVertex, PrimitiveCount);

    EndStereoDraw(This);
    return hr;
}

typedef HRESULT(WINAPI* DrawIndexedPrimitive_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE Type, INT BaseVertexIndex, UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount);
DrawIndexedPrimitive_t oDrawIndexedPrimitive = nullptr;

HRESULT WINAPI hkDrawIndexedPrimitive(IDirect3DDevice9* This, D3DPRIMITIVETYPE Type, INT BaseVertexIndex, UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount)
{
    HeadHideProbe_OnDrawCall(This, "DrawIndexedPrimitive", primCount);
    PixelConstantProbe_OnDrawCall(This);
    FadeProbe_OnDraw(This, 1, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    const HudDrawClass hudClass = ClassifyHudDraw(This, primCount);
    if (hudClass == kHudDrop)
        return D3D_OK;
    if (hudClass == kHudPanel) {
#if RE5VR_DIAGNOSTICS
        HudProbe_OnDraw(This, 1, Type, primCount, kStereoPathHudViewport);
#endif
        return DrawHudPerEye(This, [&] {
            return oDrawIndexedPrimitive(This, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);
        });
    }

    StereoDrawContext ctx;
    const bool stereo = BeginStereoDraw(This, ctx);
#if RE5VR_DIAGNOSTICS
    HudProbe_OnDraw(This, 1, Type, primCount, g_stereoPath);
#endif
    if (!stereo) {
        return oDrawIndexedPrimitive(This, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);
    }

    SetEyeState(This, ctx, true);
    HRESULT hr = oDrawIndexedPrimitive(This, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);

    SetEyeState(This, ctx, false);
    oDrawIndexedPrimitive(This, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);

    EndStereoDraw(This);
    return hr;
}

typedef HRESULT(WINAPI* DrawPrimitiveUP_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT PrimitiveCount, const void* pVertexStreamZeroData, UINT VertexStreamZeroStride);
DrawPrimitiveUP_t oDrawPrimitiveUP = nullptr;

HRESULT WINAPI hkDrawPrimitiveUP(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT PrimitiveCount, const void* pVertexStreamZeroData, UINT VertexStreamZeroStride)
{
    HeadHideProbe_OnDrawCall(This, "DrawPrimitiveUP", PrimitiveCount);
    PixelConstantProbe_OnDrawCall(This);
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    const HudDrawClass hudClass = ClassifyHudDraw(This, PrimitiveCount);
    if (hudClass == kHudDrop)
        return D3D_OK;
    if (hudClass == kHudPanel) {
#if RE5VR_DIAGNOSTICS
        HudProbe_OnDraw(This, 2, PrimitiveType, PrimitiveCount, kStereoPathHudViewport);
#endif
        return DrawHudPerEye(This, [&] {
            return oDrawPrimitiveUP(This, PrimitiveType, PrimitiveCount, pVertexStreamZeroData, VertexStreamZeroStride);
        });
    }

    StereoDrawContext ctx;
    const bool stereo = BeginStereoDraw(This, ctx);
#if RE5VR_DIAGNOSTICS
    HudProbe_OnDraw(This, 2, PrimitiveType, PrimitiveCount, g_stereoPath);
#endif
    if (!stereo) {
        return oDrawPrimitiveUP(This, PrimitiveType, PrimitiveCount, pVertexStreamZeroData, VertexStreamZeroStride);
    }

    SetEyeState(This, ctx, true);
    HRESULT hr = oDrawPrimitiveUP(This, PrimitiveType, PrimitiveCount, pVertexStreamZeroData, VertexStreamZeroStride);

    SetEyeState(This, ctx, false);
    oDrawPrimitiveUP(This, PrimitiveType, PrimitiveCount, pVertexStreamZeroData, VertexStreamZeroStride);

    EndStereoDraw(This);
    return hr;
}

typedef HRESULT(WINAPI* DrawIndexedPrimitiveUP_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT MinVertexIndex, UINT NumVertices, UINT PrimitiveCount, const void* pIndexData, D3DFORMAT IndexDataFormat, const void* pVertexStreamZeroData, UINT VertexStreamZeroStride);
DrawIndexedPrimitiveUP_t oDrawIndexedPrimitiveUP = nullptr;

HRESULT WINAPI hkDrawIndexedPrimitiveUP(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT MinVertexIndex, UINT NumVertices, UINT PrimitiveCount, const void* pIndexData, D3DFORMAT IndexDataFormat, const void* pVertexStreamZeroData, UINT VertexStreamZeroStride)
{
    HeadHideProbe_OnDrawCall(This, "DrawIndexedPrimitiveUP", PrimitiveCount);
    PixelConstantProbe_OnDrawCall(This);
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    const HudDrawClass hudClass = ClassifyHudDraw(This, PrimitiveCount);
    if (hudClass == kHudDrop)
        return D3D_OK;
    if (hudClass == kHudPanel) {
#if RE5VR_DIAGNOSTICS
        HudProbe_OnDraw(This, 3, PrimitiveType, PrimitiveCount, kStereoPathHudViewport);
#endif
        return DrawHudPerEye(This, [&] {
            return oDrawIndexedPrimitiveUP(This, PrimitiveType, MinVertexIndex, NumVertices, PrimitiveCount, pIndexData,
                IndexDataFormat, pVertexStreamZeroData, VertexStreamZeroStride);
        });
    }

    StereoDrawContext ctx;
    const bool stereo = BeginStereoDraw(This, ctx);
#if RE5VR_DIAGNOSTICS
    HudProbe_OnDraw(This, 3, PrimitiveType, PrimitiveCount, g_stereoPath);
#endif
    if (!stereo) {
        return oDrawIndexedPrimitiveUP(This, PrimitiveType, MinVertexIndex, NumVertices, PrimitiveCount,
            pIndexData, IndexDataFormat, pVertexStreamZeroData, VertexStreamZeroStride);
    }

    SetEyeState(This, ctx, true);
    HRESULT hr = oDrawIndexedPrimitiveUP(This, PrimitiveType, MinVertexIndex, NumVertices, PrimitiveCount,
        pIndexData, IndexDataFormat, pVertexStreamZeroData, VertexStreamZeroStride);

    SetEyeState(This, ctx, false);
    oDrawIndexedPrimitiveUP(This, PrimitiveType, MinVertexIndex, NumVertices, PrimitiveCount,
        pIndexData, IndexDataFormat, pVertexStreamZeroData, VertexStreamZeroStride);

    EndStereoDraw(This);
    return hr;
}

} // namespace

void StereoTest_Install(IDirect3DDevice9* pDevice)
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("StereoTest_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    struct HookSpec {
        size_t slot;
        void* detour;
        void** original;
        const char* name;
    };

    HookSpec specs[] = {
        { kIDirect3DDevice9_DrawPrimitive,          reinterpret_cast<void*>(&hkDrawPrimitive),          reinterpret_cast<void**>(&oDrawPrimitive),          "DrawPrimitive" },
        { kIDirect3DDevice9_DrawIndexedPrimitive,   reinterpret_cast<void*>(&hkDrawIndexedPrimitive),   reinterpret_cast<void**>(&oDrawIndexedPrimitive),   "DrawIndexedPrimitive" },
        { kIDirect3DDevice9_DrawPrimitiveUP,        reinterpret_cast<void*>(&hkDrawPrimitiveUP),        reinterpret_cast<void**>(&oDrawPrimitiveUP),        "DrawPrimitiveUP" },
        { kIDirect3DDevice9_DrawIndexedPrimitiveUP, reinterpret_cast<void*>(&hkDrawIndexedPrimitiveUP), reinterpret_cast<void**>(&oDrawIndexedPrimitiveUP), "DrawIndexedPrimitiveUP" },
    };

    for (const auto& spec : specs) {
        void* pTarget = VTableEntry(pDevice, spec.slot);
        MH_STATUS st = MH_CreateHook(pTarget, spec.detour, spec.original);
        if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
            Log_Printf("StereoTest_Install: MH_CreateHook(%s) failed -> %d", spec.name, static_cast<int>(st));
            continue;
        }
        st = MH_EnableHook(pTarget);
        Log_Printf("StereoTest_Install: %s hook enabled -> %d", spec.name, static_cast<int>(st));
    }
}

namespace {
bool g_haveBackBufferSize = false;
} // namespace

// A SCREEN IS AN OBJECT (2026-09-25, user: "as you look around, the screen
// warps. Consider the screen basically being an object that stays in one spot.
// Almost like you are in a 3d space with a 2d screen in front of you").
//
// Exactly right, and the reason it warped is that the first version was not an
// object at all. It pasted a rectangle of fixed pixel size and slid it sideways
// by the tangent of how far you had turned. A rectangle that never changes
// shape is only correct while you are looking straight at it; turn your head
// and a real screen foreshortens and keystones, and sliding a fixed rectangle
// instead reads as the picture stretching away from you.
//
// So it is four corners in space now, projected properly. The screen hangs at
// the remembered direction with a width and height in metres, each corner is
// expressed in the eye's own axes and divided through by its depth, and the
// vertices carry one over that depth as their w - which is what makes the
// texture perspective-correct across the quad rather than merely stretched to
// fit it. Off-axis it foreshortens by itself, because that falls out of the
// arithmetic instead of being imitated.
void ComposeTheatreNow(IDirect3DDevice9* dev)
{
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;
    D3DSURFACE_DESC desc = {};
    if (FAILED(bb->GetDesc(&desc)) || desc.Width < 16 || desc.Height < 16) {
        bb->Release();
        return;
    }
    if (!g_theatreTex || g_theatreCopyW != desc.Width || g_theatreCopyH != desc.Height
        || g_theatreCopyFmt != desc.Format) {
        ReleaseTheatreCopy();
        if (FAILED(dev->CreateTexture(desc.Width, desc.Height, 1, D3DUSAGE_RENDERTARGET, desc.Format,
                D3DPOOL_DEFAULT, &g_theatreTex, nullptr))
            || !g_theatreTex || FAILED(g_theatreTex->GetSurfaceLevel(0, &g_theatreSurf))) {
            ReleaseTheatreCopy();
            bb->Release();
            return;
        }
        g_theatreCopyW = desc.Width;
        g_theatreCopyH = desc.Height;
        g_theatreCopyFmt = desc.Format;
        Log_Printf("Theatre: a %ux%u screen to show the picture on", desc.Width, desc.Height);
    }
    if (FAILED(dev->StretchRect(bb, nullptr, g_theatreSurf, nullptr, D3DTEXF_NONE))) {
        bb->Release();
        return;
    }

    XRBridgeEyeView views[2];
    const bool haveViews = GetEyeViewsForDraw(views[0], views[1]);
    if (!haveViews) {
        bb->Release();
        return;
    }
    const float ipd = MeasuredIpd(haveViews, views);
    const float* R = views[0].rotationDelta; // rows: right, up, forward

    // Where it hangs. Levelled, and only the way you were facing, because a
    // cinema screen is not tilted to match somebody who was looking at the
    // floor. Taken afresh every frame if it is meant to follow you instead.
    if (!g_haveScreenAnchor || g_theatreFollowsHead) {
        const float fx = R[6], fz = R[8];
        const float flen = std::sqrt(fx * fx + fz * fz);
        const float rx = R[0], rz = R[2];
        const float rlen = std::sqrt(rx * rx + rz * rz);
        if (flen > 0.01f && rlen > 0.01f) {
            g_screenFwd[0] = fx / flen;
            g_screenFwd[1] = 0.0f;
            g_screenFwd[2] = fz / flen;
            g_screenRight[0] = rx / rlen;
            g_screenRight[1] = 0.0f;
            g_screenRight[2] = rz / rlen;
            if (!g_haveScreenAnchor) {
                g_haveScreenAnchor = true;
                Log_Printf("Theatre: the screen hangs where you were facing; look around and it stays put");
            }
        }
    }
    if (!g_haveScreenAnchor) {
        bb->Release();
        return;
    }

    // How big, in metres. The slider says what share of your view it fills, so
    // that becomes an angle, and the angle becomes a size at the set distance.
    float spanX = 2.4f;
    {
        const float sx = std::tan(views[0].angleRight) - std::tan(views[0].angleLeft);
        if (sx > 0.1f)
            spanX = sx;
    }
    const float dist = g_theatreDistanceMeters;
    const float halfWm = dist * spanX * g_theatreScale * 0.5f;
    // The picture's own shape, kept: the back buffer is the whole frame.
    const float halfHm = halfWm * (static_cast<float>(desc.Height) / static_cast<float>(desc.Width));

    // Black first. Whatever is not the screen is the dark of the room.
    dev->ColorFill(bb, nullptr, D3DCOLOR_ARGB(255, 0, 0, 0));

    IDirect3DStateBlock9* saved = nullptr;
    if (FAILED(dev->CreateStateBlock(D3DSBT_ALL, &saved)))
        saved = nullptr;
    IDirect3DSurface9* oldRt = nullptr;
    dev->GetRenderTarget(0, &oldRt);
    dev->SetRenderTarget(0, bb);

    dev->SetVertexShader(nullptr);
    dev->SetPixelShader(nullptr);
    dev->SetFVF(D3DFVF_XYZRHW | D3DFVF_TEX1);
    dev->SetTexture(0, g_theatreTex);
    dev->SetRenderState(D3DRS_LIGHTING, FALSE);
    dev->SetRenderState(D3DRS_ZENABLE, FALSE);
    dev->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    dev->SetRenderState(D3DRS_STENCILENABLE, FALSE);
    dev->SetRenderState(D3DRS_FOGENABLE, FALSE);
    dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    dev->SetRenderState(D3DRS_SRGBWRITEENABLE, FALSE);
    dev->SetRenderState(D3DRS_COLORWRITEENABLE,
        D3DCOLORWRITEENABLE_RED | D3DCOLORWRITEENABLE_GREEN | D3DCOLORWRITEENABLE_BLUE
            | D3DCOLORWRITEENABLE_ALPHA);
    dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    dev->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE);
    dev->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    dev->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);
    dev->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
    dev->SetSamplerState(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    dev->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
    dev->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);

    const float fullW = static_cast<float>(desc.Width);
    const float fullH = static_cast<float>(desc.Height);
    const float halfW = fullW * 0.5f;

    D3DVIEWPORT9 whole = {};
    whole.Width = desc.Width;
    whole.Height = desc.Height;
    whole.MaxZ = 1.0f;
    dev->SetViewport(&whole);

    for (int eye = 0; eye < 2; ++eye) {
        const XRBridgeEyeView& v = views[eye];
        const float* E = v.rotationDelta; // this eye's own axes
        const float tl = std::tan(v.angleLeft), tr = std::tan(v.angleRight);
        const float tu = std::tan(v.angleUp), td = std::tan(v.angleDown);
        if (tr - tl <= 1e-3f || tu - td <= 1e-3f)
            continue;
        const float halfX0 = eye ? halfW : 0.0f;

        // Where this eye is: half an inter-pupillary distance along its own
        // right. The screen does not move, so the two views of it differ by
        // exactly this, which is what gives it a place in the room.
        const float side = (eye == 0 ? -0.5f : 0.5f) * ipd;
        const float eyeAt[3] = { E[0] * side, E[1] * side, E[2] * side };

        TheatreVertex quad[4];
        bool ok = true;
        for (int c = 0; c < 4; ++c) {
            const float sx = (c == 0 || c == 2) ? -1.0f : 1.0f;
            const float sy = (c < 2) ? 1.0f : -1.0f;
            float p[3];
            for (int i = 0; i < 3; ++i)
                p[i] = g_screenFwd[i] * dist + g_screenRight[i] * (halfWm * sx) - eyeAt[i];
            p[1] += halfHm * sy;
            const float vx = p[0] * E[0] + p[1] * E[1] + p[2] * E[2];
            const float vy = p[0] * E[3] + p[1] * E[4] + p[2] * E[5];
            const float vz = p[0] * E[6] + p[1] * E[7] + p[2] * E[8];
            if (vz < 0.05f) { // a corner level with or behind the eye
                ok = false;
                break;
            }
            const float ndcX = (2.0f * (vx / vz) - (tr + tl)) / (tr - tl);
            const float ndcY = (2.0f * (vy / vz) - (tu + td)) / (tu - td);
            quad[c].x = halfX0 + (ndcX + 1.0f) * 0.5f * halfW;
            quad[c].y = (1.0f - ndcY) * 0.5f * fullH;
            quad[c].z = 0.5f;
            quad[c].rhw = 1.0f / vz;
            quad[c].u = (c == 0 || c == 2) ? 0.0f : 1.0f;
            quad[c].v = (c < 2) ? 0.0f : 1.0f;
        }
        if (!ok)
            continue;

        // Its own half and nothing else, so a screen at the edge of your view
        // cannot spill into the other eye.
        RECT half = { static_cast<LONG>(halfX0), 0, static_cast<LONG>(halfX0 + halfW),
            static_cast<LONG>(fullH) };
        dev->SetScissorRect(&half);
        dev->SetRenderState(D3DRS_SCISSORTESTENABLE, TRUE);
        dev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, quad, sizeof(TheatreVertex));
    }

    dev->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
    dev->SetTexture(0, nullptr);
    if (oldRt) {
        dev->SetRenderTarget(0, oldRt);
        oldRt->Release();
    }
    if (saved) {
        saved->Apply();
        saved->Release();
    }
    bb->Release();
}

// How long the rendered camera has been away from where the skeleton says your
// eye is. Run from Present, so it keeps answering while the theatre is on and
// can say when the scene has finished and given you back your body.
// The game takes the HUD away for a scene and leaves it up for everything you
// do yourself. That is the line the camera cannot draw on its own.
void UpdateHudGoneClock()
{
    const unsigned long long now = GetTickCount64();
    // ONE DRAW IS NOT A HUD (2026-09-25, user: "the theater watcher still didn't
    // grab pre-rendered cutscenes").
    //
    // The watcher earned its keep on the first run. A pre-rendered film reads:
    //
    //   rig quiet  4.8 s, 1 HUD draw(s) last frame, HUD gone 0.0 s, 0 cut(s) -> left alone
    //   rig quiet 10.8 s, 1 HUD draw(s) last frame, HUD gone 0.0 s, 0 cut(s) -> left alone
    //
    // Exactly one HUD draw a frame, for eleven seconds. That one draw IS the
    // film - a full-screen quad drawn by a shader this code already recognises
    // as HUD. So "the HUD has gone" was false for the whole picture, and since
    // every path was made to require it, every path was blocked. The reason
    // nothing I changed helped is that I kept changing the tests in front of it.
    //
    // A HUD is health and ammo and a partner's name: the same log reads 22 draws
    // six seconds later with a real one up. One or two draws is a picture, not an
    // interface, so it takes three to count as the HUD being up.
    constexpr unsigned kHudIsUp = 3;
    g_hudDrawsLastFrame = g_hudDrawsThisFrame;
    if (g_hudDrawsThisFrame >= kHudIsUp)
        g_hudGoneSince = 0;
    else if (!g_hudGoneSince)
        g_hudGoneSince = now;
    g_hudDrawsThisFrame = 0;
}

void UpdateCameraAwayClock()
{
    float m[16];
    if (!ConstantProbe_GetCachedCameraMatrix(m))
        return;
    CameraBasis basis = {};
    DecomposeCameraMatrix(m, basis);
    float bodyEye[3];
    if (!CameraRigHook_GetBodyEye(basis.forward, basis.camPos, bodyEye)) {
        g_cameraAwaySince = 0;
        return;
    }
    const float d[3] = { bodyEye[0] - basis.camPos[0], bodyEye[1] - basis.camPos[1],
        bodyEye[2] - basis.camPos[2] };
    const float away = Length3(d);
    // A CUT IS THE THING ONLY A FILM DOES (2026-09-25, user: "a chest isn't a
    // hud. It's a camera focused on the chest opening to reveal the items
    // inside").
    //
    // Which is right, and it takes the HUD test with it for that case. So this
    // asks the question the other way round: not where the camera is or how
    // long it stays, both of which a chest reveal matches perfectly, but whether
    // it has ever JUMPED.
    //
    // Cutscenes are cut together. They change shot, and a change of shot moves
    // the camera metres in a single frame. A chest opening, a vault, a melee, a
    // revive are each one continuous move from wherever you were standing - a
    // camera that travels, never one that teleports. At a hundred frames a
    // second nothing continuous covers a metre and a half between two of them.
    {
        static float s_lastCam[3] = {};
        static bool s_haveLastCam = false;
        if (s_haveLastCam) {
            const float j[3] = { basis.camPos[0] - s_lastCam[0], basis.camPos[1] - s_lastCam[1],
                basis.camPos[2] - s_lastCam[2] };
            if (Length3(j) > 150.0f && g_cameraCuts < 1000)
                ++g_cameraCuts;
        }
        std::memcpy(s_lastCam, basis.camPos, sizeof(s_lastCam));
        s_haveLastCam = true;
    }
    // The same hysteresis the action camera uses: far enough to be a shot, and
    // then held until it is clearly back.
    static bool s_away = false;
    s_away = s_away ? away > 25.0f : away > 60.0f;
    const unsigned long long now = GetTickCount64();
    if (!s_away) {
        // Say what that was, now it is over. Every vault, every melee, every
        // scene, with the two numbers the theatre judges them on - so the
        // thresholds can be set from a log of real moments rather than from
        // anybody's idea of how long a window jump takes.
        if (g_cameraAwaySince) {
            const float wasAwaySec = (now - g_cameraAwaySince) / 1000.0f;
            if (wasAwaySec > 1.0f) {
                const float hudSec = g_hudGoneSince ? (now - g_hudGoneSince) / 1000.0f : 0.0f;
                Log_Printf("Theatre: the view was off your body for %.1f s, HUD gone %.1f s, %d cut(s) - %s",
                    wasAwaySec, hudSec, g_cameraCuts, g_inTheatre ? "the screen was up" : "left alone");
            }
        }
        g_cameraAwaySince = 0;
        g_cameraCuts = 0; // back on your body: whatever that was, it is over
    } else if (!g_cameraAwaySince) {
        g_cameraAwaySince = now;
        g_cameraCuts = 0;
    }
}

void StereoTest_ComposeTheatre(IDirect3DDevice9* pDevice)
{
    if (!pDevice)
        return;
    // And belt and braces at the door, because this one is called from Present
    // rather than from a draw, so nothing upstream has already decided that VR
    // is running.
    if (!g_enabled || g_suppressed) {
        if (g_inTheatre) {
            g_inTheatre = false;
            g_haveScreenAnchor = false;
            Log_Printf("Theatre: VR is off, so the screen is too");
        }
        return;
    }
    // Composited on the state the frame was DRAWN with, then the state is
    // decided for the next one. Asking now and acting on the answer would put a
    // screen around a frame that was rendered in stereo, once, on the way in
    // and again on the way out.
    UpdateCameraAwayClock();
    UpdateHudGoneClock();
    if (g_inTheatre)
        ComposeTheatreNow(pDevice);
    g_inTheatre = TheatreNow();
}

void StereoTest_ToggleTheatre()
{
    g_theatreForced = !g_theatreForced;
    if (!g_theatreForced)
        g_haveScreenAnchor = false; // hangs where you are facing next time
    Log_Printf("Theatre: %s by hand", g_theatreForced ? "held ON" : "let go, back to deciding for itself");
}

bool StereoTest_TheatreHeld()
{
    return g_theatreForced;
}

void StereoTest_OnBeforeDeviceReset()
{
    // A default-pool render target has to be gone before Reset, or Reset fails
    // and takes the device with it.
    ReleaseTheatreCopy();
}

void StereoTest_OnDeviceReset()
{
    g_haveBackBufferSize = false;
}

void StereoTest_OnEndScene(IDirect3DDevice9* pDevice)
{
    const unsigned long long nowMs = GetTickCount64();

    // Backbuffer size, for RenderTargetIsScreenShaped. Taken once, and again
    // after a Reset, which is the only thing that changes it.
    //
    // It used to be re-read every second, and that cost a frame every second
    // (2026-09-18). GetBackBuffer takes a reference on a surface the driver is
    // using, asks it a question and hands it back, which can make the pipeline
    // wait. A tester measured a rock-steady 120 dropping to 100 on a perfect
    // one-second beat, sitting on the MENU, where the mod is otherwise doing
    // nothing - a value that only changes on a resolution switch has no business
    // being polled.
    if (!g_haveBackBufferSize) {
        IDirect3DSurface9* bb = nullptr;
        if (SUCCEEDED(pDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) {
            D3DSURFACE_DESC d = {};
            if (SUCCEEDED(bb->GetDesc(&d))) {
                g_backBufferWidth = d.Width;
                g_backBufferHeight = d.Height;
                g_haveBackBufferSize = true;
            }
            bb->Release();
        }
    }

    // Same reasoning as the patch reports: a five-second disk write on the
    // render thread, for a line only a developer reads.
    static unsigned long long s_lastCensusMs = 0;
    if (RE5VR_DIAGNOSTICS && nowMs - s_lastCensusMs >= 5000) {
        s_lastCensusMs = nowMs;
        XRBridgeEyeView l, r;
        if (g_enabled && VRBridge_GetEyeViews(l, r)) {
            char targets[256] = {};
            size_t len = 0;
            for (const auto& t : g_census.targets) {
                if (!t.n)
                    break;
                const int n = _snprintf_s(targets + len, sizeof(targets) - len, _TRUNCATE, " %ux%u x%u", t.w, t.h, t.n);
                if (n < 0)
                    break;
                len += static_cast<size_t>(n);
            }
            const bool latched = g_frameFixes && g_haveFrameViews && nowMs - g_lastPresentMs < kPresentStaleMs;
            Log_Printf("StereoTest: last 5 s [fixes %s] - %u 3D draws split per eye, %u HUD draws squeezed, %u old-nudge draws, "
                       "%u single draws into off-screen targets:%s | %u Present(s), pose %s",
                g_frameFixes ? "ON" : "OFF", g_census.split3d, g_census.screenSpace, g_census.nudged, g_census.offscreen,
                targets[0] ? targets : " none", g_presentCount, latched ? "latched per frame" : "read live per draw");
        }
        g_census = {};
        g_presentCount = 0;
    }

    // F8, '/', backslash, '[' ']' '-' and ';' quote are menu options now (ui/menu.cpp).

    // Does F4 actually move the game's camera while VR is on? The user
    // reports it doesn't reach the head position, but "where the camera
    // is" is exactly the thing that is hard to judge from inside a
    // headset - so decode it from c0-c3 and watch it across the toggle
    // instead. If camPos jumps when the override turns on, the rig hook
    // is working and the values just aren't landing where we want; if it
    // does not move at all, the override isn't reaching the camera the
    // game is actually rendering from.
    static bool prevRigEnabled = false;
    static unsigned long long rigTrackUntilMs = 0;
    static unsigned long long rigLastSampleMs = 0;
    const bool rigEnabled = CameraRigHook_IsEnabled();
    if (rigEnabled != prevRigEnabled) {
        // 2026-09-10 20:31: the camera still pops out to third person when
        // pitching down, even though the log proves ALL SIX rig structs
        // held our override the entire time (Dist=0, VDist=173, FOV=42) -
        // including at the moment the pop was on screen. So the pop is not
        // these fields being overwritten, and it is not a race we are
        // losing: we win it, and the camera moves anyway. Something else
        // positions the camera on pitch.
        //
        // Six frames at the moment of the toggle was enough to prove the
        // override reaches the camera, but it all lands before the player
        // has looked anywhere. Track the camera for ~20s instead, sampled
        // rather than per-frame, so a slow deliberate look-down produces a
        // readable trajectory: if camPos slides backwards along -forward
        // it is a boom arm extending, if it arcs it is a pivot, and either
        // way the magnitude says how far whatever-this-is thinks the
        // camera should sit.
        rigTrackUntilMs = GetTickCount64() + kRigTrackDurationMs;
        rigLastSampleMs = 0;
        Log_Printf("StereoTest: camera rig override toggled %s - tracking decoded camera position for %llu ms",
            rigEnabled ? "ON" : "OFF", kRigTrackDurationMs);
    }
    prevRigEnabled = rigEnabled;

    const unsigned long long rigNowMs = GetTickCount64();
    if (rigNowMs < rigTrackUntilMs && rigNowMs - rigLastSampleMs >= kRigTrackSampleEveryMs) {
        rigLastSampleMs = rigNowMs;
        float rigMatrix[16];
        if (ConstantProbe_GetCachedCameraMatrix(rigMatrix) && IsPlausibleCameraMatrix(rigMatrix)) {
            CameraBasis rigBasis;
            DecomposeCameraMatrix(rigMatrix, rigBasis);
            // Drop the identity-ish matrices that IsPlausibleCameraMatrix
            // still lets through - a real camera here has Sx ~1.36-1.46,
            // while UI/shadow passes decode as exactly Sx=Sy=1.000 with
            // forward=(0,0,+-1). They polluted the 20:52 trace and, more
            // importantly, the same cached matrix feeds head tracking, so
            // this is a real bug in its own right (see project memory).
            const bool identityish =
                std::fabs(rigBasis.scaleX - 1.0f) < 0.01f && std::fabs(rigBasis.scaleY - 1.0f) < 0.01f;
            if (!identityish) {
                Log_Printf("StereoTest: rig=%s camPos=(%.1f, %.1f, %.1f) forward=(%.3f, %.3f, %.3f) Sx=%.3f Sy=%.3f",
                    rigEnabled ? "ON" : "OFF",
                    rigBasis.camPos[0], rigBasis.camPos[1], rigBasis.camPos[2],
                    rigBasis.forward[0], rigBasis.forward[1], rigBasis.forward[2],
                    rigBasis.scaleX, rigBasis.scaleY);
            }
        } else {
            Log_Printf("StereoTest: rig=%s - no plausible camera matrix cached this frame",
                rigEnabled ? "ON" : "OFF");
        }
    }

#if RE5VR_DIAGNOSTICS
    // Phase 4 prep: dumps the camera's decoded world position (F5, developer
    // builds - F5 is the boom finder there too).
    static bool prevF5Down = false;
    bool f5Down = (GetAsyncKeyState(VK_F5) & 0x8000) != 0;
    if (f5Down && !prevF5Down) {
        float baseMatrix[16];
        if (ConstantProbe_GetCachedCameraMatrix(baseMatrix) && IsPlausibleCameraMatrix(baseMatrix)) {
            CameraBasis basis;
            DecomposeCameraMatrix(baseMatrix, basis);
            Log_Printf("StereoTest: F5 camPos = (%.6f, %.6f, %.6f)  forward = (%.4f, %.4f, %.4f)",
                basis.camPos[0], basis.camPos[1], basis.camPos[2],
                basis.forward[0], basis.forward[1], basis.forward[2]);
        } else {
            Log_Printf("StereoTest: F5 pressed but no plausible cached camera matrix yet");
        }
    }
    prevF5Down = f5Down;
#endif // RE5VR_DIAGNOSTICS
}

void StereoTest_SetSuppressed(bool suppressed)
{
    g_suppressed = suppressed;
}

void StereoTest_SetEnabled(bool enabled)
{
    g_enabled = enabled;
}

unsigned long long StereoTest_GetLatchedPoseId()
{
    if (!g_frameFixes || !g_haveFrameViews || GetTickCount64() - g_lastPresentMs >= kPresentStaleMs)
        return 0;
    return g_frameViewsPoseId;
}

int StereoTest_GetPictureTurnMode()
{
    return g_pictureTurnMode.load(std::memory_order_relaxed);
}

bool StereoTest_GetLatchedHeadForward(float out[3])
{
    if (!g_frameFixes || !g_haveFrameViews || GetTickCount64() - g_lastPresentMs >= kPresentStaleMs)
        return false;
    // Row 2 of the rotation delta is forward, same convention the bridge
    // publishes and camera_rig_hook consumes.
    out[0] = g_frameViews[0].rotationDelta[6];
    out[1] = g_frameViews[0].rotationDelta[7];
    out[2] = g_frameViews[0].rotationDelta[8];
    return true;
}

float StereoTest_GetBackbufferAspect()
{
    if (!g_backBufferWidth || !g_backBufferHeight)
        return 16.0f / 9.0f;
    return static_cast<float>(g_backBufferWidth) / static_cast<float>(g_backBufferHeight);
}

// ---- Holding the camera to a human turning speed (2026-09-27) ------------
//
// "The whip when stomping most certainly happens while in flatscreen." That
// one sentence rules out everything we spent the night on. Flatscreen never
// touches the per-eye path and never uses the head-locked view the way VR
// does, so if the stomp throws the view there as well, then GetViewMatrix was
// never the route and the limiter in camera_rig_hook was refusing a turn that
// was not the one reaching the screen. It explains the 102 degree ask being
// held while the view whipped anyway.
//
// This is the matrix the game hands the shader. Both modes go through it,
// nothing goes around it, and correcting it here corrects it once for both.
//
// Register 0 is written many times a frame, usually with the same matrix, so
// the correction is computed when the matrix CHANGES and replayed for every
// repeat. Otherwise the elapsed time between two writes of one frame is zero
// and the allowance collapses to nothing.
void StereoTest_LimitCameraTurn(float m[16])
{
    const float limitDeg = CameraRigHook_GetSettings().viewTurnLimitDeg;
    if (!(limitDeg > 0.0f) || !CameraRigHook_IsEnabled())
        return;
    if (!IsPlausibleCameraMatrix(m))
        return;

    static float s_lastRaw[16] = {};
    static float s_lastFixed[16] = {};
    static bool s_have = false;
    static float s_heldFwd[3] = { 0.0f, 0.0f, 1.0f };
    static unsigned long long s_changedAt = 0;

    if (s_have && std::memcmp(m, s_lastRaw, sizeof(s_lastRaw)) == 0) {
        std::memcpy(m, s_lastFixed, sizeof(s_lastFixed));
        return;
    }
    std::memcpy(s_lastRaw, m, sizeof(s_lastRaw));

    CameraBasis basis;
    DecomposeCameraMatrix(m, basis);

    const unsigned long long now = GetTickCount64();
    const float dt = s_changedAt ? static_cast<float>(now - s_changedAt) * 0.001f : 0.0f;
    s_changedAt = now;

    if (s_have && dt > 0.0f && dt < 0.25f) {
        float dot = basis.forward[0] * s_heldFwd[0] + basis.forward[1] * s_heldFwd[1]
            + basis.forward[2] * s_heldFwd[2];
        dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
        const float turned = std::acos(dot) * 57.2957795f;
        const float allowed = limitDeg * dt;
        if (turned > allowed && turned > 0.01f) {
            // Creep rather than clamp: clamping still delivers the whole turn
            // over the following frames, which is a slower whip, not none.
            constexpr float kCreepDegPerSec = 20.0f;
            const float creep = kCreepDegPerSec * dt;
            const float k = (creep < turned ? creep : turned) / turned;
            float want[3];
            for (int i = 0; i < 3; ++i)
                want[i] = s_heldFwd[i] + (basis.forward[i] - s_heldFwd[i]) * k;
            const float l = Length3(want);
            if (l > 1e-4f) {
                for (int i = 0; i < 3; ++i)
                    basis.forward[i] = want[i] / l;
                // Rebuilt about world vertical, so the horizon stays level
                // whatever the game was doing. A rolled horizon is the one
                // thing in a headset that is worse than a whip.
                const float up[3] = { 0.0f, 1.0f, 0.0f };
                float right[3] = { up[1] * basis.forward[2] - up[2] * basis.forward[1],
                    up[2] * basis.forward[0] - up[0] * basis.forward[2],
                    up[0] * basis.forward[1] - up[1] * basis.forward[0] };
                const float rl = Length3(right);
                if (rl > 1e-4f) {
                    for (int i = 0; i < 3; ++i)
                        basis.right[i] = right[i] / rl;
                    basis.up[0] = basis.forward[1] * basis.right[2] - basis.forward[2] * basis.right[1];
                    basis.up[1] = basis.forward[2] * basis.right[0] - basis.forward[0] * basis.right[2];
                    basis.up[2] = basis.forward[0] * basis.right[1] - basis.forward[1] * basis.right[0];
                    ComposeCameraMatrix(basis, m);
                }
            }
            static unsigned long long s_toldAt = 0;
            if (now - s_toldAt > 250) {
                s_toldAt = now;
                Log_Printf("StereoTest: the camera matrix turned %.0f deg in %.0f ms (%.0f deg/s) - held",
                    turned, dt * 1000.0f, turned / dt);
            }
        }
    }
    std::memcpy(s_heldFwd, basis.forward, sizeof(s_heldFwd));
    std::memcpy(s_lastFixed, m, sizeof(s_lastFixed));
    s_have = true;
}

void StereoTest_OnPresent()
{
    // The frame that was just presented is the one rendered with the pose
    // latched at the PREVIOUS Present, so hand that id over before taking a
    // new one - the bridge tags the outgoing image with it so the compositor
    // is told the pose those pixels were really drawn from.
    VRBridge_NoteFramePresented(g_frameCameraPoseId != 0 ? g_frameCameraPoseId : g_frameViewsPoseId);
    g_frameCameraPoseId = 0;

    // Views and their pose id together, in one read (2026-09-16). Asking for
    // the id separately let the submit thread publish a new pose in between,
    // so the picture was drawn with one head rotation and handed to the
    // runtime labelled as another. See VRBridge_GetEyeViews.
    XRBridgePoseId viewsPoseId = 0;
    g_haveFrameViews = VRBridge_GetEyeViews(g_frameViews[0], g_frameViews[1], &viewsPoseId);
    g_frameViewsPoseId = g_haveFrameViews ? viewsPoseId : 0;
    g_lastPresentMs = GetTickCount64();
    ++g_presentCount;
}

// ---- In-game menu (2026-09-13) -----------------------------------------
StereoSettings StereoTest_GetSettings()
{
    StereoSettings s;
    s.stereoEnabled = g_enabled;
    s.halfSeparation = g_halfSeparation;
    s.fovWiden = g_fovWidenMultiplier;
    s.monoSmallTargets = g_monoSmallTargets.load(std::memory_order_relaxed);
    s.compensateHeadFollow = g_compensateHeadFollow.load(std::memory_order_relaxed);
    s.headPositionTracking = g_headPositionTracking;
    s.leanScale = g_leanScale;
    s.nearPlaneUnits = g_nearPlaneUnits;
    s.pictureTurnMode = g_pictureTurnMode.load(std::memory_order_relaxed);
    s.hudDistanceMeters = g_hudDistanceMeters;
    s.hudScale = g_hudScale;
    s.theatre = g_theatre;
    s.theatreFollowsHead = g_theatreFollowsHead;
    s.theatreDistanceMeters = g_theatreDistanceMeters;
    s.theatreScale = g_theatreScale;
    return s;
}

void StereoTest_ApplySettings(const StereoSettings& in)
{
    StereoSettings s = in;
    const auto clamp = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    s.halfSeparation = clamp(s.halfSeparation, 0.0f, 20.0f);
    s.fovWiden = clamp(s.fovWiden, 0.5f, 2.0f);
    s.hudDistanceMeters = clamp(s.hudDistanceMeters, 0.5f, 10.0f);
    s.hudScale = clamp(s.hudScale, 0.3f, 1.0f);
    s.theatreDistanceMeters = clamp(s.theatreDistanceMeters, 0.8f, 12.0f);
    s.theatreScale = clamp(s.theatreScale, 0.3f, 1.0f);

    const StereoSettings old = StereoTest_GetSettings();
    if (s.stereoEnabled != old.stereoEnabled)
        Log_Printf("StereoTest: stereo rendering now %s", s.stereoEnabled ? "ON" : "OFF");
    g_enabled = s.stereoEnabled;
    g_monoSmallTargets.store(s.monoSmallTargets, std::memory_order_relaxed);
    if (s.monoSmallTargets != old.monoSmallTargets)
        Log_Printf("StereoTest: post-process buffers now drawn %s",
            s.monoSmallTargets ? "MONO (default, no light leaks)" : "per eye (expect light leaks)");
    g_compensateHeadFollow.store(s.compensateHeadFollow, std::memory_order_relaxed);
    s.leanScale = clamp(s.leanScale, 0.0f, 3.0f);
    if (s.headPositionTracking != old.headPositionTracking) {
        // Switching it on takes wherever you are sitting now as the centre,
        // rather than snapping the view to a reference from ten minutes ago.
        g_leanReferenceSet = false;
        Log_Printf("StereoTest: leaning is now %s", s.headPositionTracking ? "ON" : "off");
    }
    g_headPositionTracking = s.headPositionTracking;
    g_leanScale = s.leanScale;
    s.nearPlaneUnits = clamp(s.nearPlaneUnits, 0.0f, 40.0f);
    g_nearPlaneUnits = s.nearPlaneUnits;
    // Pinned (2026-09-18, user: "double, culling fixed should be the only
    // camera option, we just need to steady those micro movements"). The
    // other four were the 2026-09-15 experiments and this one won. Leaving
    // them selectable cost more than it was worth: a stale PictureTurnMode in
    // somebody's ini put them on a mode nobody was testing, and the modes do
    // not agree about who turns the picture - so head steadying is exactly
    // 1:1 on this one and deliberately not on Game camera only. One path, one
    // thing to reason about.
    s.pictureTurnMode = kPictureTurnDoubleFixed;
    if (s.pictureTurnMode != old.pictureTurnMode)
        Log_Printf("StereoTest: picture turning mode now %s", PictureTurnModeName(s.pictureTurnMode));
    g_pictureTurnMode.store(s.pictureTurnMode, std::memory_order_relaxed);
    g_halfSeparation = s.halfSeparation;
    g_fovWidenMultiplier = s.fovWiden;
    g_hudDistanceMeters = s.hudDistanceMeters;
    g_hudScale = s.hudScale;
    g_theatre = s.theatre;
    g_theatreFollowsHead = s.theatreFollowsHead;
    g_theatreDistanceMeters = s.theatreDistanceMeters;
    g_theatreScale = s.theatreScale;
    if (s.halfSeparation != old.halfSeparation || s.fovWiden != old.fovWiden ||
        s.hudDistanceMeters != old.hudDistanceMeters || s.hudScale != old.hudScale) {
        Log_Printf("StereoTest: eye half-separation %.2f, FOV widen %.2f, HUD %.2f m at scale %.2f",
            g_halfSeparation, g_fovWidenMultiplier, g_hudDistanceMeters, g_hudScale);
    }
}

float StereoTest_GetNearPlaneUnits()
{
    return g_nearPlaneUnits;
}

float StereoTest_EyeOffsetUnits()
{
    return g_eyeOffUnits.load(std::memory_order_relaxed);
}

void StereoTest_EyeOffset(float out[3])
{
    out[0] = g_eyeOffX.load(std::memory_order_relaxed);
    out[1] = g_eyeOffY.load(std::memory_order_relaxed);
    out[2] = g_eyeOffZ.load(std::memory_order_relaxed);
}

void StereoTest_SetRoomAsk(float right, float forward)
{
    g_roomAsk[0] = right;
    g_roomAsk[1] = forward;
}

bool StereoTest_GetRoomStep(float* rightMetres, float* forwardMetres)
{
    if (!g_headPositionTracking || !g_haveRoomStep)
        return false;
    if (rightMetres)
        *rightMetres = g_roomStep[0];
    if (forwardMetres)
        *forwardMetres = g_roomStep[1];
    return true;
}

bool StereoTest_GetLeanWorld(float out[3])
{
    if (!g_headPositionTracking || !g_haveLeanWorld || !out)
        return false;
    std::memcpy(out, g_leanWorld, sizeof(g_leanWorld));
    return true;
}

void StereoTest_RecentreLean()
{
    g_leanReferenceSet = false;
    g_haveLeanWorld = false;
}

bool StereoTest_IsEnabled()
{
    return g_enabled;
}

bool StereoTest_GetPanelPlacement(float distanceMeters, UINT frameWidth, UINT frameHeight, StereoPanelEye eyes[2])
{
    XRBridgeEyeView views[2];
    const bool haveViews = GetEyeViewsForDraw(views[0], views[1]);
    const float ipd = MeasuredIpd(haveViews, views);
    const float w = static_cast<float>(frameWidth), h = static_cast<float>(frameHeight);
    for (int eye = 0; eye < 2; ++eye) {
        StereoPanelEye& e = eyes[eye];
        EyeCentreForDistance(views, haveViews, ipd, eye, distanceMeters, 0.0f, 0.0f, w, h, e.centreX, e.centreY);
        e.halfX0 = eye ? w * 0.5f : 0.0f;
        e.halfWidth = w * 0.5f;
        // Pixels per unit of tangent. Without views, assume ~100 deg square.
        float spanX = 2.4f, spanY = 2.4f;
        if (haveViews) {
            spanX = std::tan(views[eye].angleRight) - std::tan(views[eye].angleLeft);
            spanY = std::tan(views[eye].angleUp) - std::tan(views[eye].angleDown);
            if (spanX < 0.1f || spanY < 0.1f)
                spanX = spanY = 2.4f;
        }
        e.pxPerTanX = e.halfWidth / spanX;
        e.pxPerTanY = h / spanY;
    }
    return haveViews;
}
