#include "stereo_test.h"
#include "../hooks/camera_rig_hook.h"
#include "../hooks/constant_probe.h"
#include "../hooks/head_hide_probe.h"
#include "../hooks/pixel_constant_probe.h"
#include "../vr/openxr_bridge.h"
#include "../util/log.h"
#include "mat3.h"

#include <MinHook.h>
#include <windows.h>
#include <cmath>
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
float g_fovWidenMultiplier = 1.0f;
constexpr float kFovWidenStep = 0.1f;

bool g_enabled = false;
bool g_suppressed = false;

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

bool BeginStereoDraw(IDirect3DDevice9* pDevice, StereoDrawContext& ctx)
{
    (void)pDevice;
    if (!g_enabled || g_suppressed)
        return false;

    float baseMatrix[16];
    if (!ConstantProbe_GetCachedCameraMatrix(baseMatrix))
        return false;

    XRBridgeEyeView leftView, rightView;
    const bool haveEyeViews = VRBridge_GetEyeViews(leftView, rightView);
    if (!haveEyeViews || !IsPlausibleCameraMatrix(baseMatrix)) {
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

        Mat3 delta{};
        std::memcpy(delta.m, view.rotationDelta, sizeof(delta.m));
        ApplyHeadRotation(basis, delta);

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

typedef HRESULT(WINAPI* DrawPrimitive_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount);
DrawPrimitive_t oDrawPrimitive = nullptr;

HRESULT WINAPI hkDrawPrimitive(IDirect3DDevice9* This, D3DPRIMITIVETYPE PrimitiveType, UINT StartVertex, UINT PrimitiveCount)
{
    HeadHideProbe_OnDrawCall(This, "DrawPrimitive", PrimitiveCount);
    PixelConstantProbe_OnDrawCall(This);
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    StereoDrawContext ctx;
    if (!BeginStereoDraw(This, ctx))
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
    if (HeadHideHook_ShouldSkip(This))
        return D3D_OK;

    StereoDrawContext ctx;
    if (!BeginStereoDraw(This, ctx)) {
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

    StereoDrawContext ctx;
    if (!BeginStereoDraw(This, ctx)) {
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

    StereoDrawContext ctx;
    if (!BeginStereoDraw(This, ctx)) {
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
    (void)pDevice;
    static bool prevF8Down = false;
    bool f8Down = (GetAsyncKeyState(VK_F8) & 0x8000) != 0;
    if (f8Down && !prevF8Down) {
        g_enabled = !g_enabled;
        Log_Printf("StereoTest: F8 pressed, scissor-split stereo test now %s", g_enabled ? "ON" : "OFF");
    }
    prevF8Down = f8Down;

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

    // Phase 4 prep: dumps the camera's actual computed world-space
    // position (decoded from c0-c3, not a guess) so it can be searched
    // for directly in a live memory scanner (Cheat Engine/x64dbg) instead
    // of a blind "unknown initial value" scan - we already know the exact
    // float value to search for.
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

    // Live eye-separation tuning (Phase 4 follow-up) - '[' and ']' are
    // used instead of a function key (all of F1-F12 are already claimed
    // elsewhere in this project) or Numpad +/- (not present on every
    // keyboard, same concern as the earlier Home/End/Insert swaps this
    // session). Adjust in small steps while in the headset until depth
    // perception/comfort feels right, then report the final value back so
    // g_halfSeparation's starting default can be updated to match.
    static bool prevBracketDownDown = false;
    bool bracketDownDown = (GetAsyncKeyState(VK_OEM_4) & 0x8000) != 0; // '['
    if (bracketDownDown && !prevBracketDownDown) {
        g_halfSeparation = (g_halfSeparation > kHalfSeparationStep) ? g_halfSeparation - kHalfSeparationStep : 0.0f;
        Log_Printf("StereoTest: '[' pressed, eye half-separation now %.2f", g_halfSeparation);
    }
    prevBracketDownDown = bracketDownDown;

    static bool prevBracketUpDown = false;
    bool bracketUpDown = (GetAsyncKeyState(VK_OEM_6) & 0x8000) != 0; // ']'
    if (bracketUpDown && !prevBracketUpDown) {
        g_halfSeparation += kHalfSeparationStep;
        Log_Printf("StereoTest: ']' pressed, eye half-separation now %.2f", g_halfSeparation);
    }
    prevBracketUpDown = bracketUpDown;

    // Reset to the default. Needed because '[' clamps to exactly 0.00,
    // which re-bases the value onto a clean 0.25 grid - and the default
    // (3.18, derived from a real half-IPD) is NOT on that grid, so once
    // the floor has been touched it can never be returned to by tuning.
    // A tuning session that cannot get back to its own starting point
    // cannot A/B against it either, which is exactly how a full sweep
    // still left the default unverified.
    static bool prevResetDown = false;
    bool resetDown = (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000) != 0; // '-'
    if (resetDown && !prevResetDown) {
        g_halfSeparation = kDefaultHalfSeparation;
        Log_Printf("StereoTest: '-' pressed, eye half-separation RESET to default %.2f", g_halfSeparation);
    }
    prevResetDown = resetDown;

    // Live FOV widen tuning (see g_fovWidenMultiplier's declaration) -
    // ';'/'\'' continue the same increase-on-the-right key layout as
    // '['/']'. Completely separate from eye orientation (now real OpenXR
    // per-eye toe-in) and position (g_halfSeparation) - widening FOV here
    // cannot disturb either.
    static bool prevSemicolonDown = false;
    bool semicolonDown = (GetAsyncKeyState(VK_OEM_1) & 0x8000) != 0; // ';'
    if (semicolonDown && !prevSemicolonDown) {
        g_fovWidenMultiplier = (g_fovWidenMultiplier > kFovWidenStep) ? g_fovWidenMultiplier - kFovWidenStep : kFovWidenStep;
        Log_Printf("StereoTest: ';' pressed, FOV widen multiplier now %.2f", g_fovWidenMultiplier);
    }
    prevSemicolonDown = semicolonDown;

    static bool prevQuoteDown = false;
    bool quoteDown = (GetAsyncKeyState(VK_OEM_7) & 0x8000) != 0; // '\''
    if (quoteDown && !prevQuoteDown) {
        g_fovWidenMultiplier += kFovWidenStep;
        Log_Printf("StereoTest: '\\'' pressed, FOV widen multiplier now %.2f", g_fovWidenMultiplier);
    }
    prevQuoteDown = quoteDown;
}

void StereoTest_SetSuppressed(bool suppressed)
{
    g_suppressed = suppressed;
}

void StereoTest_SetEnabled(bool enabled)
{
    g_enabled = enabled;
}
