#include "stereo_test.h"
#include "../hooks/camera_rig_hook.h"
#include "../hooks/constant_probe.h"
#include "../hooks/fade_probe.h"
#include "../hooks/head_hide_probe.h"
#include "../hooks/pixel_constant_probe.h"
#include "../hooks/hud_probe.h"
#include "hud_shaders.h"
#include "../vr/openxr_bridge.h"
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

float g_fovWidenMultiplier = 1.0f;
constexpr float kFovWidenStep = 0.1f;

bool g_enabled = false;
bool g_suppressed = false;
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
    outMatrix[11] = w3 + basis.depthBiasK;
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
    if (!g_enabled || g_suppressed) {
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

    auto buildEyeBasis = [&](const XRBridgeEyeView& view, bool leftEye) {
        CameraBasis basis = baseBasis;

        // Head-follow compensation (2026-09-12). When F9 is steering the
        // game's camera, that camera ALREADY contains this rotation - the
        // measurement showed it tracking the head 1:1 - so rotating the eye
        // basis by it again turns the world roughly twice as far as the
        // wearer turns. That is consistent with the user needing to "move
        // slow to get it to feel smooth". While head-follow is driving, skip
        // the delta and let the game camera carry the rotation on its own.
        // '\' toggles this off to compare.
        const bool headFollowDriving = g_compensateHeadFollow.load(std::memory_order_relaxed) &&
            CameraRigHook_HeadFollowDrivingCamera();
        if (!headFollowDriving) {
            Mat3 delta{};
            std::memcpy(delta.m, view.rotationDelta, sizeof(delta.m));
            ApplyHeadRotation(basis, delta);
        }

        const float offset = leftEye ? -g_halfSeparation : g_halfSeparation;
        for (int i = 0; i < 3; ++i)
            basis.camPos[i] += basis.right[i] * offset;

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

        return basis;
    };

    const CameraBasis leftBasis = buildEyeBasis(leftView, true);
    const CameraBasis rightBasis = buildEyeBasis(rightView, false);

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

HudDrawClass ClassifyHudDraw(IDirect3DDevice9* pDevice, UINT prims)
{
    if (!g_enabled || g_suppressed)
        return kNotHud;
    // Recognised by bytecode hash from launch (hud_shaders.cpp). The K capture
    // stays available in developer builds, for finding shaders not listed yet.
    if (HudShaders_IsHudDraw(pDevice)) {
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

void StereoTest_OnEndScene(IDirect3DDevice9* pDevice)
{
    const unsigned long long nowMs = GetTickCount64();

    // Backbuffer size, for RenderTargetIsScreenShaped. Refreshed once a
    // second - it only changes on a resolution switch.
    static unsigned long long s_lastBackBufferMs = 0;
    if (nowMs - s_lastBackBufferMs >= 1000) {
        s_lastBackBufferMs = nowMs;
        IDirect3DSurface9* bb = nullptr;
        if (SUCCEEDED(pDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) {
            D3DSURFACE_DESC d = {};
            if (SUCCEEDED(bb->GetDesc(&d))) {
                g_backBufferWidth = d.Width;
                g_backBufferHeight = d.Height;
            }
            bb->Release();
        }
    }

    static unsigned long long s_lastCensusMs = 0;
    if (nowMs - s_lastCensusMs >= 5000) {
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

void StereoTest_OnPresent()
{
    // The frame that was just presented is the one rendered with the pose
    // latched at the PREVIOUS Present, so hand that id over before taking a
    // new one - the bridge tags the outgoing image with it so the compositor
    // is told the pose those pixels were really drawn from.
    VRBridge_NoteFramePresented(g_frameViewsPoseId);

    g_haveFrameViews = VRBridge_GetEyeViews(g_frameViews[0], g_frameViews[1]);
    g_frameViewsPoseId = g_haveFrameViews ? VRBridge_GetCurrentPoseId() : 0;
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
    s.hudDistanceMeters = g_hudDistanceMeters;
    s.hudScale = g_hudScale;
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

    const StereoSettings old = StereoTest_GetSettings();
    if (s.stereoEnabled != old.stereoEnabled)
        Log_Printf("StereoTest: stereo rendering now %s", s.stereoEnabled ? "ON" : "OFF");
    g_enabled = s.stereoEnabled;
    g_monoSmallTargets.store(s.monoSmallTargets, std::memory_order_relaxed);
    if (s.monoSmallTargets != old.monoSmallTargets)
        Log_Printf("StereoTest: post-process buffers now drawn %s",
            s.monoSmallTargets ? "MONO (default, no light leaks)" : "per eye (expect light leaks)");
    g_compensateHeadFollow.store(s.compensateHeadFollow, std::memory_order_relaxed);
    g_halfSeparation = s.halfSeparation;
    g_fovWidenMultiplier = s.fovWiden;
    g_hudDistanceMeters = s.hudDistanceMeters;
    g_hudScale = s.hudScale;
    if (s.halfSeparation != old.halfSeparation || s.fovWiden != old.fovWiden ||
        s.hudDistanceMeters != old.hudDistanceMeters || s.hudScale != old.hudScale) {
        Log_Printf("StereoTest: eye half-separation %.2f, FOV widen %.2f, HUD %.2f m at scale %.2f",
            g_halfSeparation, g_fovWidenMultiplier, g_hudDistanceMeters, g_hudScale);
    }
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
