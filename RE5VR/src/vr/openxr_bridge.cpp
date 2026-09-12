#include "openxr_bridge.h"
#include "../render/stereo_test.h"
#include "../render/mat3.h"
#include "../proxy/real_d3d9.h"
#include "../util/log.h"
#include "d3d12_addon_bridge.h"

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <windows.h>
#include <d3d11_4.h> // ID3D11Multithread - the base d3d11.h doesn't declare it
#include <dxgi.h>
#include <dxgi1_4.h> // IDXGIFactory4::EnumAdapterByLuid
#include <timeapi.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

// ---- OpenXR session state -----------------------------------------------

bool g_xrModeEnabled = false;
bool g_xrInitAttempted = false;
bool g_xrInitialized = false;   // instance + system (Step A) resolved
bool g_xrSessionReady = false;  // session + swapchains built
bool g_xrSessionRunning = false; // xrBeginSession called, not yet xrEndSession'd

XrInstance g_xrInstance = XR_NULL_HANDLE;
XrSystemId g_xrSystemId = XR_NULL_SYSTEM_ID;
XrSession g_xrSession = XR_NULL_HANDLE;
XrSpace g_xrLocalSpace = XR_NULL_HANDLE;

// XR_FB_display_refresh_rate: without this, the runtime silently picks
// whatever default rate it wants for an app that never asks - suspected
// (2026-07-29, see project memory) to be why xrWaitFrame settles into a
// ~20ms/45Hz cadence on this headset even though the headset's own system
// display is confirmed set to 90Hz. Loaded via xrGetInstanceProcAddr since
// extension entry points aren't guaranteed to be statically linkable.
bool g_displayRefreshRateExtSupported = false;
PFN_xrEnumerateDisplayRefreshRatesFB g_xrEnumerateDisplayRefreshRatesFB = nullptr;
PFN_xrRequestDisplayRefreshRateFB g_xrRequestDisplayRefreshRateFB = nullptr;

// XR_EXT_performance_settings: 2026-07-29 adb logcat capture caught
// VirtualDesktop.Android (the on-device half of the VDXR runtime) explicitly
// calling xrPerfSettingsSetPerformanceLevelEXT(domain=cpu, level=sustained
// low) via this exact extension right at session start, held for the entire
// duration of a test that stayed capped at 22.22ms/45Hz - see project
// memory. This is the first direct, headset-OS-level evidence of a
// deliberate throttle, not inferred from timing. Testing whether RE5VR
// requesting a higher CPU level itself (as the OTHER connected OpenXR
// client) can override or coexist with VD's own "sustained low" default.
bool g_perfSettingsExtSupported = false;
PFN_xrPerfSettingsSetPerformanceLevelEXT g_xrPerfSettingsSetPerformanceLevelEXT = nullptr;

// What VDXR itself recommends per eye (queried via
// xrEnumerateViewConfigurationViews) - suspected 2026-07-29 to matter for
// the runtime's frame-pacing decision independent of measured performance.
// RE5VR's actual rendered content stays at g_eyeWidth/g_eyeHeight (half the
// game's fixed backbuffer - see CreateXrSessionAndSwapchains) regardless;
// only the swapchain's own allocated size changes to test this theory.
UINT g_recommendedEyeWidth = 0;
UINT g_recommendedEyeHeight = 0;

// 2026-07-29: tried XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO here as a
// diagnostic (does the runtime's 45Hz cap depend on stereo/viewCount=2?) -
// VDXR rejected it outright with XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED
// at xrBeginSession, so the session never started at all. This runtime is
// stereo-only for this form factor; mono isn't a usable diagnostic path
// here. Reverted to STEREO - do not retry PRIMARY_MONO on this runtime.
constexpr XrViewConfigurationType kDiagnosticViewConfigType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

constexpr int kEyeLeft = 0;
constexpr int kEyeRight = 1;
XrSwapchain g_xrSwapchain[2] = { XR_NULL_HANDLE, XR_NULL_HANDLE };
std::vector<XrSwapchainImageD3D11KHR> g_xrSwapchainImages[2];

// Most recently located eye views (xrLocateViews), read by
// VRBridge_GetEyeViews - see openxr_bridge.h for why this replaces the old
// OpenVR bridge's separate head-delta/FOV-scale/toe-in getters.
bool g_haveEyeViews = false;
// ---- Rendered-pose ring (2026-09-12) -----------------------------------
// See openxr_bridge.h. Each xrLocateViews publish stamps an id and keeps the
// raw XrPosef/XrFovf pair; the render thread carries that id through to
// Present, where it lands on an addon slot; the submit thread reads it back
// so the pose it hands xrEndFrame is the one the pixels were drawn with.
// A ring rather than a single slot because the game can be several frames
// ahead of, or behind, the submit thread.
struct RenderedPose {
    XrPosef pose[2];
    XrFovf fov[2];
    double publishedMs; // when this pose was located - see the age measurement
    std::atomic<XRBridgePoseId> id{ 0 }; // written last: id != 0 means the entry is complete
};
constexpr size_t kPoseRingSize = 16;
RenderedPose g_poseRing[kPoseRingSize];
std::atomic<XRBridgePoseId> g_nextPoseId{ 1 };
std::atomic<XRBridgePoseId> g_currentPoseId{ 0 };
// Pose id the game rendered the image now sitting in each addon slot with.
std::atomic<XRBridgePoseId> g_slotPoseId[2] = {};
// Pose the frame the game just finished was rendered with, set at Present.
std::atomic<XRBridgePoseId> g_poseOfPresentedFrame{ 0 };
// Diagnostics for the submit thread's pose lookup.
std::atomic<unsigned long long> g_poseHits{ 0 };
std::atomic<unsigned long long> g_poseMisses{ 0 };
// How stale the image is by the time it reaches the compositor, measured from
// when its pose was located. The user can see this directly: mouse look is 1:1
// on the desktop mirror and visibly late in the headset.
// Plain, not atomic: only the submit thread accumulates these and only the
// submit thread reports them.
double g_frameAgeSumMs = 0.0;
unsigned long long g_frameAgeCount = 0;
double g_frameAgeWorstMs = 0.0;

// Multiplies the head's rotation before it reaches anything: 1.0 is honest
// 1:1, higher means a smaller neck turn covers more world. Page Up / Page
// Down tune it live.
std::atomic<float> g_headRotationGain{ 1.0f };
constexpr float kHeadGainStep = 0.1f;
constexpr float kHeadGainMin = 0.5f;
constexpr float kHeadGainMax = 3.0f;

// Milliseconds of head-rotation prediction. Unlike gain, this scales
// VELOCITY, not displacement: it pushes the view ahead only while you are
// actually turning and contributes exactly nothing once you stop, which is
// why it can cancel drag without the overshoot-at-the-end that gain causes.
// Home / End tune it; 0 is off.
std::atomic<float> g_headPredictMs{ 0.0f };
constexpr float kHeadPredictStep = 5.0f;
constexpr float kHeadPredictMax = 60.0f;

struct HeadPredictState {
    Mat3 last;
    double lastMs;
    float omega[3]; // smoothed angular velocity, radians per millisecond
    bool valid;
};
HeadPredictState g_headPredict[2] = {};
// How hard to smooth the velocity estimate. A single-step difference is far
// too noisy to extrapolate 25+ ms from - that noise WAS the jitter that
// capped how much prediction was usable.
constexpr float kHeadOmegaSmoothing = 0.25f;

// High-resolution milliseconds. GetTickCount64 ticks every ~15.6 ms while
// poses arrive every ~11, so it quantised the step to 0 / 15.6 / 31.2 and made
// the extrapolation factor swing between half and double every publish.
double NowMillis()
{
    static LARGE_INTEGER freq = {};
    if (!freq.QuadPart)
        QueryPerformanceFrequency(&freq);
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
}

void Mat3ToAxisAngle(const Mat3& m, float axis[3], float* angle)
{
    float c = (m.m[0] + m.m[4] + m.m[8] - 1.0f) * 0.5f;
    if (c > 1.0f)
        c = 1.0f;
    if (c < -1.0f)
        c = -1.0f;
    *angle = std::acos(c);
    const float s = std::sin(*angle);
    if (*angle < 1e-5f || std::fabs(s) < 1e-6f) {
        axis[0] = axis[1] = 0.0f;
        axis[2] = 1.0f;
        *angle = 0.0f;
        return;
    }
    axis[0] = (m.m[7] - m.m[5]) / (2.0f * s);
    axis[1] = (m.m[2] - m.m[6]) / (2.0f * s);
    axis[2] = (m.m[3] - m.m[1]) / (2.0f * s);
}

Mat3 AxisAngleToMat3(const float axis[3], float angle)
{
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    const float t = 1.0f - c;
    const float x = axis[0], y = axis[1], z = axis[2];
    Mat3 r{};
    r.m[0] = c + x * x * t;
    r.m[1] = x * y * t - z * s;
    r.m[2] = x * z * t + y * s;
    r.m[3] = y * x * t + z * s;
    r.m[4] = c + y * y * t;
    r.m[5] = y * z * t - x * s;
    r.m[6] = z * x * t - y * s;
    r.m[7] = z * y * t + x * s;
    r.m[8] = c + z * z * t;
    return r;
}

// Scale a rotation by converting to axis-angle, multiplying the angle, and
// rebuilding it (Rodrigues). Scaling the matrix entries directly would not
// produce a rotation at all. Row-major, rows = right/up/forward, matching
// XRBridgeEyeView::rotationDelta.
Mat3 ScaleRotation(const Mat3& m, float gain)
{
    float c = (m.m[0] + m.m[4] + m.m[8] - 1.0f) * 0.5f;
    if (c > 1.0f)
        c = 1.0f;
    if (c < -1.0f)
        c = -1.0f;
    const float angle = std::acos(c);
    const float s = std::sin(angle);
    if (angle < 1e-4f || std::fabs(s) < 1e-6f)
        return m; // no meaningful rotation to scale, and the axis is unstable

    const float ax = (m.m[7] - m.m[5]) / (2.0f * s);
    const float ay = (m.m[2] - m.m[6]) / (2.0f * s);
    const float az = (m.m[3] - m.m[1]) / (2.0f * s);

    const float na = angle * gain;
    const float nc = std::cos(na);
    const float ns = std::sin(na);
    const float t = 1.0f - nc;

    Mat3 r{};
    r.m[0] = nc + ax * ax * t;
    r.m[1] = ax * ay * t - az * ns;
    r.m[2] = ax * az * t + ay * ns;
    r.m[3] = ay * ax * t + az * ns;
    r.m[4] = nc + ay * ay * t;
    r.m[5] = ay * az * t - ax * ns;
    r.m[6] = az * ax * t - ay * ns;
    r.m[7] = az * ay * t + ax * ns;
    r.m[8] = nc + az * az * t;
    return r;
}

XRBridgeEyeView g_eyeViews[2] = {};
// Written by the XR submit thread, read by the render thread: a pose must
// be copied in or out whole, never half of one frame and half of the next.
SRWLOCK g_eyeViewsLock = SRWLOCK_INIT;

// Per-eye orientation reference, captured once per eye the first time a
// valid view is located after XR mode is (re-)enabled - same recenter-on-
// enable pattern the old OpenVR bridge used for its single shared head
// delta (see g_haveHmdReference/g_hmdReference in the old
// openvr_bridge.cpp), just tracked per eye here since OpenXR hands back
// two independent eye orientations rather than one head pose. Using a
// PER-EYE reference (not one shared reference applied to both eyes) means
// any real per-eye toe-in the runtime reports is preserved in the
// resulting delta, rather than being cancelled out.
bool g_haveEyeReference[2] = { false, false };
Mat3 g_eyeReference[2] = { Mat3Identity(), Mat3Identity() };

// Sticky "has this XR session ever actually completed a real frame" latch.
// The first several hundred frames after xrBeginSession reliably fail
// xrEndFrame with XR_ERROR_POSE_INVALID (normal/expected, tracking not
// locked yet - see openxr_bridge.h) - and during that exact window,
// xrLocateViews has been observed to report viewStateFlags claiming BOTH
// ORIENTATION_VALID and ORIENTATION_TRACKED anyway. Neither the flags nor
// a quaternion-magnitude sanity check (both tried first, 2026-07-28)
// caught this reliably - the runtime's own signals aren't trustworthy
// this early.
//
// NOT sufficient by itself, though (confirmed 2026-07-28 via delta-angle
// diagnostic logging): the first successful xrEndFrame does NOT mean the
// orientation data has actually settled - real captured deltas right
// after the latch flipped true jumped to 136 degrees on the very next
// frame, then oscillated in the 2.5-11.5 degree range for at least
// another ~15 frames before the log window ran out. That jitter, applied
// to the render every frame, is very likely the observed "screen shake" -
// and whichever moment the per-eye reference happens to be captured
// during that unstable window becomes a permanently-wrong baseline,
// which reads as the "cockeyed eye" bug. See g_framesSincePoseLock below,
// which requires the pose to additionally sit still for a while past this
// latch before it's trusted enough to capture as a reference.
bool g_xrPoseLocked = false;

// Frames elapsed since g_xrPoseLocked first flipped true (0 until then,
// then increments once per frame forever after) - see g_xrPoseLocked's
// comment for why the latch alone isn't enough. kPoseSettleFrames is a
// generous multiple of the ~15-20 frames of observed post-lock jitter.
// Until this reaches the threshold, orientation is treated exactly like
// "no reference yet" (report identity - see the poseUsable gate below),
// so the render shows zero head rotation (flat/still) rather than the
// raw jitter during this window, instead of trying to filter/smooth it.
int g_framesSincePoseLock = 0;
constexpr int kPoseSettleFrames = 90;

// Per-eye counter of how many post-recenter deltas have been diagnostic-
// logged this XR session - see the log call inside XrSubmitThreadProc.
int g_deltaLogCount[2] = { 0, 0 };

// 2026-07-29 diagnostic (user-suggested workload-sensitivity theory): tests
// whether the runtime's predictedDisplayPeriod cap (11.11ms -> 22.22ms the
// instant real content starts flowing, see project memory) is triggered by
// the actual COST of producing a real projection layer, or merely by
// submitting ANY projection layer at all regardless of cost. For the first
// kPlaceholderModeFrames real frames after pose lock, the per-eye swapchain
// image is cleared to a solid color (ClearRenderTargetView) instead of
// receiving the real CopySubresourceRegion from game content - layerCount
// is still 1, pose/FOV are still real and current, only the one step that
// costs anything (the cross-API pixel copy) is skipped. If
// predictedDisplayPeriod stays at 11.11ms during this window, the runtime
// IS workload-sensitive and the next question is what specifically about
// our real copy reads as "expensive". If it still flips to 22.22ms with a
// blank layer, that's a clean refutation - the cap is tied to submitting a
// real projection layer at all, not its cost. Disposable diagnostic, same
// pattern as kDiagnosticViewConfigType above - revert
// kDiagnosticPlaceholderMode to false (or delete this block) once the
// result is known.
constexpr bool kDiagnosticPlaceholderMode = false;
constexpr int kPlaceholderModeFrames = 300;
int g_realContentFrameCount = 0;
bool g_placeholderModeEverActive = false;

const char* XrResultName(XrResult r)
{
    static char buf[XR_MAX_RESULT_STRING_SIZE];
    if (g_xrInstance != XR_NULL_HANDLE && XR_SUCCEEDED(xrResultToString(g_xrInstance, r, buf)))
        return buf;
    _snprintf_s(buf, sizeof(buf), _TRUNCATE, "XrResult(%d)", static_cast<int>(r));
    return buf;
}

// ---- D3D9<->D3D11 pixel pipeline (UNCHANGED from the OpenVR bridge) -----
//
// None of this is OpenVR/OpenXR-specific - it's the unavoidable classic-
// D3D9-device CPU readback path (RE5's IDirect3DDevice9 can't produce
// cross-process/cross-API shared surfaces directly; only a D3D9Ex device
// can). fear-vr's own docs/M2-D3D9-BRIDGE.md independently arrived at the
// identical technique for the identical reason - ported near-verbatim from
// the old openvr_bridge.cpp. What DOES change (below, in the OpenXR-
// specific section) is only the final step: instead of opening the shared
// D3D9Ex texture with a throwaway second D3D11 device and calling
// vr::VRCompositor()->Submit(), the SAME D3D11 device that owns the OpenXR
// session opens it, and a CopyResource lands it directly in an acquired
// swapchain image.

bool g_bridgeReady = false;
UINT g_eyeWidth = 0;
UINT g_eyeHeight = 0;
D3DFORMAT g_backbufferFormat = D3DFMT_UNKNOWN;

HWND g_bridgeWindow = nullptr;
IDirect3D9Ex* g_bridgeD3D9 = nullptr;
IDirect3DDevice9Ex* g_bridgeDevice = nullptr;

// 2026-07-29: double-buffered per eye (2 slots) instead of one shared
// texture per eye - closes a real gap vs fear-vr's own architecture (their
// AD-004: a versioned multi-slot ring specifically so the writer, this
// game's own render thread, and the reader, the XR submit path, never
// touch the same resource at the same time, and neither one blocks the
// other - a plain mutex was explicitly rejected by them for that reason,
// see docs/ARCHITECTURE.md). Slot index [0] or [1], selected by
// g_eyeFrontIndex (atomic, written by the producer after a slot is fully
// written, read by the consumer before it copies) - the producer always
// writes into the OTHER slot from whatever's currently front, so the
// consumer's in-progress read of the front slot is never touched.
IDirect3DTexture9* g_bridgeLeftTex[2] = { nullptr, nullptr };
IDirect3DTexture9* g_bridgeRightTex[2] = { nullptr, nullptr };
IDirect3DSurface9* g_bridgeLeftSurf[2] = { nullptr, nullptr };
IDirect3DSurface9* g_bridgeRightSurf[2] = { nullptr, nullptr };
HANDLE g_leftSharedHandle[2] = { nullptr, nullptr };
HANDLE g_rightSharedHandle[2] = { nullptr, nullptr };
IDirect3DSurface9* g_bridgeLeftSysMem = nullptr;
IDirect3DSurface9* g_bridgeRightSysMem = nullptr;

// Which slot is safe for the submit path to read, per eye - the producer
// (CopyHalfToEye's caller) publishes a new front index only after fully
// finishing a write into the other slot. g_eyeGeneration lets the submit
// path detect (and count, matching fear-vr's own reusedFrames_ diagnostic)
// when it's about to resubmit the same image it already sent last time,
// rather than genuinely new content.
std::atomic<int> g_eyeFrontIndex[2] = { 0, 0 };
std::atomic<UINT64> g_eyeGeneration[2] = { 0, 0 };
// The generation the submit thread has actually consumed. Producing faster
// than this is pure waste - see ShouldCopyThisCall.
std::atomic<UINT64> g_eyeConsumedGeneration[2] = { 0, 0 };
// Whether the producer waits for the submit thread to take the previous frame
// before making another. On: fewer wasted copies and no context thrash. Off:
// always keep the freshest possible image, at the cost of contention. Insert
// toggles it, because it is a prime suspect in the measured 64 ms image age.
std::atomic<bool> g_waitForConsumer{ false };

IDirect3DSurface9* g_gameLeftCrop = nullptr;
IDirect3DSurface9* g_gameRightCrop = nullptr;
IDirect3DSurface9* g_gameLeftSysMem = nullptr;
IDirect3DSurface9* g_gameRightSysMem = nullptr;

// The ONE D3D11 device for this whole bridge - created on the adapter
// OpenXR itself requires (xrGetD3D11GraphicsRequirementsKHR), and used
// both as the XR session's graphics binding AND to open the D3D9Ex shared
// eye textures. Unlike the OpenVR version, no second/throwaway D3D11
// device is needed - the swapchain images already live on this same
// device, so CopyResource works directly with no further interop.
ID3D11Device* g_d3d11Device = nullptr;
ID3D11DeviceContext* g_d3d11Context = nullptr;
ID3D11Texture2D* g_d3d11LeftTex[2] = { nullptr, nullptr };
ID3D11Texture2D* g_d3d11RightTex[2] = { nullptr, nullptr };

// 2026-09-10: when dgVoodoo2's D3D12 backend + SampleAddon.dll addon are
// active (see d3d12_addon_bridge.h/.cpp), g_d3d11LeftTex/RightTex above
// are filled directly from the addon's shared D3D12 frame via
// CopySubresourceRegion - the ENTIRE D3D9 bridge-device/shared-surface
// machinery below (g_bridgeDevice, g_bridgeLeftTex/RightTex,
// g_gameLeftCrop/RightCrop, CopyHalfToEye's whole CPU-readback chain) is
// skipped for this path (see EnsureBridgeReady/VRBridge_OnEndScene's
// branches on this flag). This exists because that D3D9-level shared-
// texture mechanism is hard-rejected by dgVoodoo2 (D3DERR_INVALIDCALL,
// see re5vr_project memory) - it only ever worked against the real
// system d3d9.dll. g_addonFrameFormat is discovered from the addon's
// actual source texture (not assumed) and used both for the OpenXR
// swapchain format search and for creating g_d3d11LeftTex/RightTex in
// the matching format in this path.
bool g_usingD3D12AddonPath = false;
DXGI_FORMAT g_addonFrameFormat = DXGI_FORMAT_UNKNOWN;

// Legacy budget, still referenced as ScopedTimer's default fallback when
// no explicit budget/bool is passed - no call site actually relies on the
// default anymore (every timer in this file now passes either an explicit
// budget pointer or the periodic bool gate - see ScopedTimer's bool
// constructor). A fixed budget was tried first for every timer added
// during this framerate investigation (CopyHalfToEye's CPU pipeline, the
// submit thread's swapchain sequence, PumpXrEvents/xrBeginFrame/
// xrLocateViews) and each one quietly went silent after its budget ran
// out within the first ~1-2 seconds of a session - meaning every
// "everything is fast" conclusion drawn from that data only covered
// session startup, never real steady-state operation. The periodic bool
// gate (log every 30th call, forever) replaced all of them for exactly
// this reason.
int g_timingLogsRemaining = 20;

struct ScopedTimer {
    const char* name;
    LARGE_INTEGER start;
    LARGE_INTEGER freq;
    bool active;
    int* budget; // nullptr = use the legacy externally-managed g_timingLogsRemaining

    explicit ScopedTimer(const char* n, int* budgetCounter = nullptr)
        : name(n), active(budgetCounter ? (*budgetCounter > 0) : (g_timingLogsRemaining > 0)), budget(budgetCounter)
    {
        if (active) {
            QueryPerformanceFrequency(&freq);
            QueryPerformanceCounter(&start);
        }
    }

    // Non-exhausting variant: caller decides per-call whether to log (e.g.
    // "every 30th iteration"), forever, instead of spending a fixed budget.
    // The budget-based constructor above was found (2026-07-28 night) to
    // have quietly stopped producing any data after the first ~1-2 seconds
    // of a session once its budget ran out - meaning every "everything is
    // fast" conclusion drawn from it only ever covered session startup, not
    // real steady-state operation deep into a test. This is the fix.
    ScopedTimer(const char* n, bool forceActive) : name(n), active(forceActive), budget(nullptr)
    {
        if (active) {
            QueryPerformanceFrequency(&freq);
            QueryPerformanceCounter(&start);
        }
    }

    ~ScopedTimer()
    {
        if (!active)
            return;
        LARGE_INTEGER end;
        QueryPerformanceCounter(&end);
        const double ms = static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
        Log_Printf("XRBridge: TIMING %s: %.2f ms", name, ms);
        if (budget && *budget > 0)
            --(*budget);
    }
};

HWND CreateHiddenWindow()
{
    static bool classRegistered = false;
    static const char* kClassName = "RE5VR_BridgeWindowClass";

    if (!classRegistered) {
        WNDCLASSA wc = {};
        wc.lpfnWndProc = DefWindowProcA;
        wc.hInstance = GetModuleHandleA(nullptr);
        wc.lpszClassName = kClassName;
        RegisterClassA(&wc);
        classRegistered = true;
    }

    return CreateWindowExA(0, kClassName, "RE5VR Bridge", WS_POPUP,
        0, 0, 4, 4, nullptr, nullptr, GetModuleHandleA(nullptr), nullptr);
}

bool CreateBridgeDevice(HWND hwnd)
{
    if (!g_RealDirect3DCreate9Ex) {
        Log_Printf("XRBridge: CreateBridgeDevice: real Direct3DCreate9Ex not resolved yet");
        return false;
    }

    HRESULT hr = g_RealDirect3DCreate9Ex(D3D_SDK_VERSION, &g_bridgeD3D9);
    if (FAILED(hr) || !g_bridgeD3D9) {
        Log_Printf("XRBridge: CreateBridgeDevice: Direct3DCreate9Ex failed (hr=0x%08lX)", hr);
        return false;
    }

    D3DPRESENT_PARAMETERS pp = {};
    pp.Windowed = TRUE;
    pp.SwapEffect = D3DSWAPEFFECT_DISCARD;
    pp.BackBufferWidth = 4;
    pp.BackBufferHeight = 4;
    pp.BackBufferFormat = D3DFMT_UNKNOWN;
    pp.hDeviceWindow = hwnd;

    hr = g_bridgeD3D9->CreateDeviceEx(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd,
        D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_MULTITHREADED,
        &pp, nullptr, &g_bridgeDevice);
    if (FAILED(hr) || !g_bridgeDevice) {
        Log_Printf("XRBridge: CreateBridgeDevice: CreateDeviceEx failed (hr=0x%08lX)", hr);
        return false;
    }

    return true;
}

// Deliberately always D3DFMT_A8R8G8B8, regardless of what the game's own
// backbuffer format actually is (e.g. this session it was D3DFMT_X8R8G8B8,
// no alpha) - a real bug found and fixed 2026-07-28: this bridge texture
// was originally created with g_backbufferFormat directly, which meant
// when the D3D9Ex-shared texture got opened as a D3D11 resource, it came
// back as DXGI_FORMAT_B8G8R8X8_UNORM, not the DXGI_FORMAT_B8G8R8A8_UNORM
// the OpenXR swapchain was created with - CopyResource between mismatched
// DXGI formats fails SILENTLY (no error, no exception, the copy just
// doesn't happen), which is exactly what produced a black headset view
// despite every log line reporting success. Confirmed against fear-vr's
// own shipped implementation (src/host64/ipc_bridge.cpp), which explicitly
// REJECTS any shared texture that isn't exactly DXGI_FORMAT_B8G8R8A8_UNORM
// - i.e. they deliberately target a fixed alpha-having format for their
// own bridge texture too, not whatever the game's real backbuffer is.
// Safe: D3DFMT_X8R8G8B8 and D3DFMT_A8R8G8B8 are byte-identical (32bpp
// BGRX vs BGRA - only whether the top byte is read as "unused" or
// "alpha"), and CopyHalfToEye's memcpy step is already a raw byte copy
// that doesn't care about the format label either side carries.
constexpr D3DFORMAT kBridgeFormat = D3DFMT_A8R8G8B8;

bool CreateSharedEyeTexture(IDirect3DTexture9** outTex, IDirect3DSurface9** outSurf, HANDLE* outHandle)
{
    *outHandle = nullptr;
    HRESULT hr = g_bridgeDevice->CreateTexture(g_eyeWidth, g_eyeHeight, 1, D3DUSAGE_RENDERTARGET,
        kBridgeFormat, D3DPOOL_DEFAULT, outTex, outHandle);
    if (FAILED(hr) || !*outTex) {
        Log_Printf("XRBridge: CreateSharedEyeTexture failed (hr=0x%08lX)", hr);
        return false;
    }
    (*outTex)->GetSurfaceLevel(0, outSurf);
    return true;
}

// Unlike the OpenVR version, this only builds the D3D9-side half of the
// bridge (shared textures + sysmem surfaces) - the D3D11 device and the
// OpenSharedResource calls now happen earlier, in
// CreateXrSessionAndSwapchains, since OpenXR needs that same device to
// exist BEFORE xrCreateSession (it's passed in the graphics binding).
// xrCreateSession behind a structured-exception guard. Deliberately tiny and
// free of anything needing unwinding, which is what lets __try live in a C++
// translation unit compiled without /EHa. Catching an access violation and
// carrying on is normally a bad idea - the faulting component's state is
// unknowable afterwards - but the alternative here is the process dying
// outright, and we never touch that session again on this path.
XrResult CreateSessionGuarded(XrInstance instance, const XrSessionCreateInfo* info, XrSession* out, DWORD* outSehCode)
{
    __try {
        return xrCreateSession(instance, info, out);
    } __except (*outSehCode = GetExceptionCode(), EXCEPTION_EXECUTE_HANDLER) {
        return XR_ERROR_RUNTIME_FAILURE;
    }
}

bool EnsureBridgeReady(IDirect3DDevice9* pGameDevice)
{
    if (g_bridgeReady)
        return true;
    if (!g_d3d11Device) {
        Log_Printf("XRBridge: EnsureBridgeReady: no D3D11 device yet (session not built)");
        return false;
    }

    IDirect3DSurface9* backbuffer = nullptr;
    if (FAILED(pGameDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &backbuffer)) || !backbuffer) {
        Log_Printf("XRBridge: EnsureBridgeReady: GetBackBuffer failed");
        return false;
    }
    D3DSURFACE_DESC desc = {};
    backbuffer->GetDesc(&desc);
    backbuffer->Release();

    g_backbufferFormat = desc.Format;
    g_eyeWidth = desc.Width / 2;
    g_eyeHeight = desc.Height;

    // 2026-09-10: D3D12 addon bridge path - skip the entire D3D9 bridge-
    // device/shared-surface machinery below (it's what CRASHES against
    // dgVoodoo2, see re5vr_project memory) and just create plain D3D11
    // textures directly, in the addon's own real format, for
    // D3D12AddonBridge_CopyToEyeSlots to write into via
    // CopySubresourceRegion. No D3D9 involvement, no shared handles on
    // this side at all - the addon already handled that on its side.
    if (g_usingD3D12AddonPath) {
        if (g_d3d11LeftTex[0] && g_d3d11RightTex[0]) {
            g_bridgeReady = true;
            return true;
        }

        D3D11_TEXTURE2D_DESC texDesc = {};
        texDesc.Width = g_eyeWidth;
        texDesc.Height = g_eyeHeight;
        texDesc.MipLevels = 1;
        texDesc.ArraySize = 1;
        texDesc.Format = g_addonFrameFormat;
        texDesc.SampleDesc.Count = 1;
        texDesc.Usage = D3D11_USAGE_DEFAULT;
        texDesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;

        for (int slot = 0; slot < 2; ++slot) {
            HRESULT texHr = g_d3d11Device->CreateTexture2D(&texDesc, nullptr, &g_d3d11LeftTex[slot]);
            if (FAILED(texHr)) {
                Log_Printf("XRBridge: EnsureBridgeReady (D3D12 addon path): CreateTexture2D (left, slot=%d) failed (hr=0x%08lX)", slot, texHr);
                return false;
            }
            texHr = g_d3d11Device->CreateTexture2D(&texDesc, nullptr, &g_d3d11RightTex[slot]);
            if (FAILED(texHr)) {
                Log_Printf("XRBridge: EnsureBridgeReady (D3D12 addon path): CreateTexture2D (right, slot=%d) failed (hr=0x%08lX)", slot, texHr);
                return false;
            }
        }

        Log_Printf("XRBridge: EnsureBridgeReady (D3D12 addon path): ready (eye %ux%u, format=%d)",
            g_eyeWidth, g_eyeHeight, static_cast<int>(g_addonFrameFormat));
        g_bridgeReady = true;
        return true;
    }

    g_bridgeWindow = CreateHiddenWindow();
    if (!g_bridgeWindow) {
        Log_Printf("XRBridge: hidden window creation failed");
        return false;
    }

    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateBridgeDevice");
    if (!CreateBridgeDevice(g_bridgeWindow))
        return false;
    Log_Printf("XRBridge: EnsureBridgeReady: CreateBridgeDevice OK");

    for (int slot = 0; slot < 2; ++slot) {
        Log_Printf("XRBridge: EnsureBridgeReady: about to CreateSharedEyeTexture (left, slot=%d)", slot);
        if (!CreateSharedEyeTexture(&g_bridgeLeftTex[slot], &g_bridgeLeftSurf[slot], &g_leftSharedHandle[slot]))
            return false;
        Log_Printf("XRBridge: EnsureBridgeReady: CreateSharedEyeTexture (left, slot=%d) OK, handle=%p", slot, g_leftSharedHandle[slot]);
        Log_Printf("XRBridge: EnsureBridgeReady: about to CreateSharedEyeTexture (right, slot=%d)", slot);
        if (!CreateSharedEyeTexture(&g_bridgeRightTex[slot], &g_bridgeRightSurf[slot], &g_rightSharedHandle[slot]))
            return false;
        Log_Printf("XRBridge: EnsureBridgeReady: CreateSharedEyeTexture (right, slot=%d) OK, handle=%p", slot, g_rightSharedHandle[slot]);
    }

    // kBridgeFormat here too, matching bridgeTarget (g_bridgeLeftSurf/
    // RightSurf) - UpdateSurface requires identical formats on both sides,
    // same reasoning as CreateSharedEyeTexture above.
    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateOffscreenPlainSurface (bridge left sysmem)");
    HRESULT hr = g_bridgeDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, kBridgeFormat,
        D3DPOOL_SYSTEMMEM, &g_bridgeLeftSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: bridge left sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }
    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateOffscreenPlainSurface (bridge right sysmem)");
    hr = g_bridgeDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, kBridgeFormat,
        D3DPOOL_SYSTEMMEM, &g_bridgeRightSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: bridge right sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }

    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateRenderTarget (game left crop)");
    hr = pGameDevice->CreateRenderTarget(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DMULTISAMPLE_NONE, 0, FALSE, &g_gameLeftCrop, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: game left crop render target failed (hr=0x%08lX)", hr);
        return false;
    }
    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateRenderTarget (game right crop)");
    hr = pGameDevice->CreateRenderTarget(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DMULTISAMPLE_NONE, 0, FALSE, &g_gameRightCrop, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: game right crop render target failed (hr=0x%08lX)", hr);
        return false;
    }

    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateOffscreenPlainSurface (game left sysmem)");
    hr = pGameDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_gameLeftSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: game left sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }
    Log_Printf("XRBridge: EnsureBridgeReady: about to CreateOffscreenPlainSurface (game right sysmem)");
    hr = pGameDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_gameRightSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: game right sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }

    for (int slot = 0; slot < 2; ++slot) {
        Log_Printf("XRBridge: EnsureBridgeReady: about to OpenSharedResource (left, slot=%d, handle=%p)", slot, g_leftSharedHandle[slot]);
        hr = g_d3d11Device->OpenSharedResource(g_leftSharedHandle[slot], __uuidof(ID3D11Texture2D),
            reinterpret_cast<void**>(&g_d3d11LeftTex[slot]));
        if (FAILED(hr)) {
            Log_Printf("XRBridge: OpenSharedResource (left, slot=%d) failed (hr=0x%08lX)", slot, hr);
            return false;
        }
        Log_Printf("XRBridge: EnsureBridgeReady: OpenSharedResource (left, slot=%d) OK", slot);
        Log_Printf("XRBridge: EnsureBridgeReady: about to OpenSharedResource (right, slot=%d, handle=%p)", slot, g_rightSharedHandle[slot]);
        hr = g_d3d11Device->OpenSharedResource(g_rightSharedHandle[slot], __uuidof(ID3D11Texture2D),
            reinterpret_cast<void**>(&g_d3d11RightTex[slot]));
        if (FAILED(hr)) {
            Log_Printf("XRBridge: OpenSharedResource (right, slot=%d) failed (hr=0x%08lX)", slot, hr);
            return false;
        }
        Log_Printf("XRBridge: EnsureBridgeReady: OpenSharedResource (right, slot=%d) OK", slot);
    }

    // Diagnostic (2026-07-28, see kBridgeFormat's comment for why this
    // matters): confirm the opened D3D11 resource's real DXGI format
    // actually matches what CreateXrSessionAndSwapchains hardcoded for the
    // swapchain (DXGI_FORMAT_B8G8R8A8_UNORM = 87) - a mismatch here means
    // CopyResource will silently no-op every frame with no other symptom
    // than a black headset view, exactly as fear-vr's own bridge code
    // explicitly guards against (src/host64/ipc_bridge.cpp).
    D3D11_TEXTURE2D_DESC openedDesc = {};
    g_d3d11LeftTex[0]->GetDesc(&openedDesc);
    constexpr UINT kExpectedDxgiFormat = 87; // DXGI_FORMAT_B8G8R8A8_UNORM
    if (openedDesc.Format != kExpectedDxgiFormat) {
        Log_Printf("XRBridge: WARNING - opened shared texture format=%u, expected %u (B8G8R8A8_UNORM) - CopyResource into the swapchain will silently fail",
            openedDesc.Format, kExpectedDxgiFormat);
    } else {
        Log_Printf("XRBridge: opened shared texture format confirmed DXGI_FORMAT_B8G8R8A8_UNORM, matches swapchain");
    }

    Log_Printf("XRBridge: bridge ready (eye %ux%u, format=%d)", g_eyeWidth, g_eyeHeight, g_backbufferFormat);
    g_bridgeReady = true;
    return true;
}

bool CopyHalfToEye(IDirect3DDevice9* pGameDevice, IDirect3DSurface9* gameBackbuffer, const RECT& srcRect,
    IDirect3DSurface9* gameCrop, IDirect3DSurface9* gameSysMem, IDirect3DSurface9* bridgeSysMem, IDirect3DSurface9* bridgeTarget,
    const char* eyeName)
{
    char label[64];

    // Periodic (non-exhausting) logging - see ScopedTimer's bool
    // constructor and g_copyTimingLogsRemaining's old budget-based
    // approach, which quietly stopped producing data once its fixed
    // budget ran out.
    static UINT64 s_copyCallCount = 0;
    ++s_copyCallCount;
    const bool logThisCall = (s_copyCallCount % 30 == 0);

    HRESULT hr;
    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "StretchRect(%s)", eyeName);
        ScopedTimer t(label, logThisCall);
        hr = pGameDevice->StretchRect(gameBackbuffer, &srcRect, gameCrop, nullptr, D3DTEXF_NONE);
    }
    if (FAILED(hr)) {
        Log_Printf("XRBridge: CopyHalfToEye: crop StretchRect failed (hr=0x%08lX)", hr);
        return false;
    }

    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "GetRenderTargetData(%s)", eyeName);
        ScopedTimer t(label, logThisCall);
        hr = pGameDevice->GetRenderTargetData(gameCrop, gameSysMem);
    }
    if (FAILED(hr)) {
        Log_Printf("XRBridge: CopyHalfToEye: GetRenderTargetData failed (hr=0x%08lX)", hr);
        return false;
    }

    D3DLOCKED_RECT srcLocked = {};
    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "LockRect-src(%s)", eyeName);
        ScopedTimer t(label, logThisCall);
        hr = gameSysMem->LockRect(&srcLocked, nullptr, D3DLOCK_READONLY);
    }
    if (FAILED(hr)) {
        Log_Printf("XRBridge: CopyHalfToEye: LockRect(src) failed (hr=0x%08lX)", hr);
        return false;
    }

    D3DLOCKED_RECT dstLocked = {};
    hr = bridgeSysMem->LockRect(&dstLocked, nullptr, 0);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: CopyHalfToEye: LockRect(dst) failed (hr=0x%08lX)", hr);
        gameSysMem->UnlockRect();
        return false;
    }

    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "memcpy(%s)", eyeName);
        ScopedTimer t(label, logThisCall);
        const BYTE* srcBytes = static_cast<const BYTE*>(srcLocked.pBits);
        BYTE* dstBytes = static_cast<BYTE*>(dstLocked.pBits);
        const UINT rowBytes = g_eyeWidth * 4; // A8R8G8B8/X8R8G8B8 = 4 bytes/pixel
        for (UINT y = 0; y < g_eyeHeight; ++y)
            std::memcpy(dstBytes + static_cast<size_t>(y) * dstLocked.Pitch,
                srcBytes + static_cast<size_t>(y) * srcLocked.Pitch, rowBytes);
    }

    gameSysMem->UnlockRect();
    bridgeSysMem->UnlockRect();

    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "UpdateSurface(%s)", eyeName);
        ScopedTimer t(label, logThisCall);
        hr = g_bridgeDevice->UpdateSurface(bridgeSysMem, nullptr, bridgeTarget, nullptr);
    }
    if (FAILED(hr)) {
        Log_Printf("XRBridge: CopyHalfToEye: UpdateSurface failed (hr=0x%08lX)", hr);
        return false;
    }

    return true;
}

// ---- OpenXR-specific: instance/system/session/swapchains ----------------

// Which runtime are we about to talk to? Normally you'd ask the instance
// after creating it, but the extension list has to be decided BEFORE that,
// and on Meta's runtime the extensions we ask for are the suspect - see
// InitOpenXRInstanceAndSystem. The active runtime is a manifest path in the
// registry; this process is 32-bit, so HKLM\SOFTWARE is redirected to
// WOW6432Node for us and we get the 32-bit runtime's manifest, which is the
// one that will actually be loaded.
bool ActiveRuntimeLooksLikeOculus()
{
    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SOFTWARE\\Khronos\\OpenXR\\1", 0, KEY_QUERY_VALUE, &key) != ERROR_SUCCESS)
        return false;

    char path[MAX_PATH * 2] = {};
    DWORD size = sizeof(path) - 1;
    DWORD type = 0;
    const LSTATUS st = RegQueryValueExA(key, "ActiveRuntime", nullptr, &type, reinterpret_cast<BYTE*>(path), &size);
    RegCloseKey(key);
    if (st != ERROR_SUCCESS || (type != REG_SZ && type != REG_EXPAND_SZ))
        return false;

    for (char* p = path; *p; ++p)
        *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
    const bool oculus = std::strstr(path, "oculus") != nullptr || std::strstr(path, "meta horizon") != nullptr;
    Log_Printf("XRBridge: active OpenXR runtime manifest is \"%s\"%s", path, oculus ? " (Meta)" : "");
    return oculus;
}

bool InitOpenXRInstanceAndSystem()
{
    // Check whether this runtime supports XR_FB_display_refresh_rate before
    // asking for it - xrCreateInstance fails outright if we request an
    // unsupported extension.
    uint32_t extPropCount = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &extPropCount, nullptr);
    std::vector<XrExtensionProperties> extProps(extPropCount, { XR_TYPE_EXTENSION_PROPERTIES });
    xrEnumerateInstanceExtensionProperties(nullptr, extPropCount, &extPropCount, extProps.data());
    bool haveDisplayRefreshRateExt = false;
    bool havePerfSettingsExt = false;
    for (const auto& ext : extProps) {
        if (std::strcmp(ext.extensionName, XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME) == 0) {
            haveDisplayRefreshRateExt = true;
        } else if (std::strcmp(ext.extensionName, XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME) == 0) {
            havePerfSettingsExt = true;
        }
    }
    Log_Printf("XRBridge: %s supported by runtime",
        haveDisplayRefreshRateExt ? XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME
                                   : (XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME " NOT"));
    Log_Printf("XRBridge: %s supported by runtime",
        havePerfSettingsExt ? XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME
                             : (XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME " NOT"));

    // Logged because knowing which runtime is loaded explains most VR
    // reports at a glance. It used to gate an experiment: on Meta's runtime,
    // xrCreateSession dies with a null write inside their service IPC client
    // (RuntimeIPCServiceClient_32.dll+0x28561), and since
    // XR_FB_display_refresh_rate is a Meta extension carried over that same
    // IPC, requesting nothing optional seemed worth a try. Tested
    // 2026-09-12: identical crash, same address, same stack. The fault is in
    // their baseline session path, so the extensions are back on - there is
    // nothing to gain by shipping Meta users a degraded instance.
    ActiveRuntimeLooksLikeOculus();

    std::vector<const char*> extensions = { XR_KHR_D3D11_ENABLE_EXTENSION_NAME };
    if (haveDisplayRefreshRateExt)
        extensions.push_back(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
    if (havePerfSettingsExt)
        extensions.push_back(XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME);

    XrInstanceCreateInfo instanceInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
    instanceInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    instanceInfo.enabledExtensionNames = extensions.data();
    strncpy_s(instanceInfo.applicationInfo.applicationName, "RE5VR", XR_MAX_APPLICATION_NAME_SIZE - 1);
    instanceInfo.applicationInfo.applicationVersion = 1;
    instanceInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    XrResult r = xrCreateInstance(&instanceInfo, &g_xrInstance);
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrCreateInstance failed -> %d", static_cast<int>(r));
        g_xrInstance = XR_NULL_HANDLE;
        return false;
    }

    if (haveDisplayRefreshRateExt) {
        xrGetInstanceProcAddr(g_xrInstance, "xrEnumerateDisplayRefreshRatesFB",
            reinterpret_cast<PFN_xrVoidFunction*>(&g_xrEnumerateDisplayRefreshRatesFB));
        xrGetInstanceProcAddr(g_xrInstance, "xrRequestDisplayRefreshRateFB",
            reinterpret_cast<PFN_xrVoidFunction*>(&g_xrRequestDisplayRefreshRateFB));
        g_displayRefreshRateExtSupported = g_xrEnumerateDisplayRefreshRatesFB && g_xrRequestDisplayRefreshRateFB;
        Log_Printf("XRBridge: display refresh rate function pointers resolved -> %s",
            g_displayRefreshRateExtSupported ? "OK" : "FAILED");
    }

    if (havePerfSettingsExt) {
        xrGetInstanceProcAddr(g_xrInstance, "xrPerfSettingsSetPerformanceLevelEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&g_xrPerfSettingsSetPerformanceLevelEXT));
        g_perfSettingsExtSupported = g_xrPerfSettingsSetPerformanceLevelEXT != nullptr;
        Log_Printf("XRBridge: perf settings function pointer resolved -> %s",
            g_perfSettingsExtSupported ? "OK" : "FAILED");
    }

    XrInstanceProperties instanceProps{ XR_TYPE_INSTANCE_PROPERTIES };
    if (XR_SUCCEEDED(xrGetInstanceProperties(g_xrInstance, &instanceProps))) {
        Log_Printf("XRBridge: xrCreateInstance OK - runtime \"%s\" version 0x%llX",
            instanceProps.runtimeName, static_cast<unsigned long long>(instanceProps.runtimeVersion));

        // 2026-09-12: a tester on Meta Link (Quest 3, "Oculus" runtime) lost
        // the whole process inside xrCreateSession - no exception for the
        // crash reporter to catch, so the runtime killed it outright. The
        // same machine, same build, works when SteamVR is made the active
        // OpenXR runtime. Everything up to here succeeds, so the log looks
        // healthy right until the game vanishes; say so plainly instead.
        if (std::strstr(instanceProps.runtimeName, "Oculus") != nullptr) {
            Log_Printf("XRBridge: WARNING - this is Meta's own OpenXR runtime. It has been seen to kill this "
                       "(32-bit) game during xrCreateSession. If the game disappears now, set SteamVR as the "
                       "active OpenXR runtime and try again - same headset, same Link connection, it just "
                       "routes through a runtime that works.");
        }
    }

    XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    r = xrGetSystem(g_xrInstance, &systemInfo, &g_xrSystemId);
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrGetSystem failed -> %s", XrResultName(r));
        xrDestroyInstance(g_xrInstance);
        g_xrInstance = XR_NULL_HANDLE;
        return false;
    }

    XrSystemProperties systemProps{ XR_TYPE_SYSTEM_PROPERTIES };
    if (XR_SUCCEEDED(xrGetSystemProperties(g_xrInstance, g_xrSystemId, &systemProps))) {
        Log_Printf("XRBridge: xrGetSystem OK - system \"%s\" (vendorId=%u)", systemProps.systemName, systemProps.vendorId);
    }

    uint32_t viewCount = 0;
    xrEnumerateViewConfigurationViews(g_xrInstance, g_xrSystemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        0, &viewCount, nullptr);
    std::vector<XrViewConfigurationView> viewConfigs(viewCount, { XR_TYPE_VIEW_CONFIGURATION_VIEW });
    r = xrEnumerateViewConfigurationViews(g_xrInstance, g_xrSystemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        viewCount, &viewCount, viewConfigs.data());
    if (XR_SUCCEEDED(r) && viewCount > 0) {
        g_recommendedEyeWidth = viewConfigs[0].recommendedImageRectWidth;
        g_recommendedEyeHeight = viewConfigs[0].recommendedImageRectHeight;
        Log_Printf("XRBridge: runtime recommends %ux%u per eye (max %ux%u)",
            g_recommendedEyeWidth, g_recommendedEyeHeight,
            viewConfigs[0].maxImageRectWidth, viewConfigs[0].maxImageRectHeight);
    } else {
        Log_Printf("XRBridge: xrEnumerateViewConfigurationViews failed -> %s", XrResultName(r));
    }

    return true;
}

// Creates the ONE D3D11 device this whole bridge uses (see its
// declaration), on the exact adapter OpenXR requires, then xrCreateSession
// with it as the graphics binding, a LOCAL reference space, and one
// swapchain per eye at our own chosen resolution (half the game's
// backbuffer, matching CopyResource's "identical dimensions" requirement
// against the bridge's D3D9Ex-origin eye textures - not the runtime's
// merely-recommended size).
bool CreateXrSessionAndSwapchains(UINT eyeWidth, UINT eyeHeight, D3DFORMAT d3d9Format)
{
    auto xrGetD3D11GraphicsRequirementsKHR = reinterpret_cast<PFN_xrGetD3D11GraphicsRequirementsKHR>(
        [] {
            PFN_xrVoidFunction fn = nullptr;
            xrGetInstanceProcAddr(g_xrInstance, "xrGetD3D11GraphicsRequirementsKHR", &fn);
            return fn;
        }());
    if (!xrGetD3D11GraphicsRequirementsKHR) {
        Log_Printf("XRBridge: xrGetInstanceProcAddr(xrGetD3D11GraphicsRequirementsKHR) failed");
        return false;
    }

    XrGraphicsRequirementsD3D11KHR gfxReq{ XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR };
    XrResult r = xrGetD3D11GraphicsRequirementsKHR(g_xrInstance, g_xrSystemId, &gfxReq);
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrGetD3D11GraphicsRequirementsKHR failed -> %s", XrResultName(r));
        return false;
    }

    IDXGIFactory1* dxgiFactory = nullptr;
    if (FAILED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory)))) {
        Log_Printf("XRBridge: CreateDXGIFactory1 failed");
        return false;
    }
    IDXGIAdapter1* chosenAdapter = nullptr;

    // 2026-09-10: prefer IDXGIFactory4::EnumAdapterByLuid over a manual
    // EnumAdapters1 loop - a real prior-art report (ValveSoftware/openvr
    // #539, an unrelated project hitting this exact same
    // OpenSharedResource E_INVALIDARG symptom on a dual-GPU laptop) found
    // that even switching from the legacy IDXGIFactory::EnumAdapters to
    // IDXGIFactory1::EnumAdapters1 changed the outcome for otherwise-
    // identical adapter selection - suggesting the exact DXGI call used
    // to resolve an adapter can matter for cross-API shared-resource
    // behavior beyond just "same LUID" (which this project already
    // confirmed matches - see re5vr_project memory). EnumAdapterByLuid
    // asks DXGI directly for the adapter with this specific LUID, rather
    // than us re-deriving it via a loop-and-compare - the most direct/
    // canonical resolution available, worth trying given that precedent.
    IDXGIFactory4* dxgiFactory4 = nullptr;
    if (SUCCEEDED(dxgiFactory->QueryInterface(__uuidof(IDXGIFactory4), reinterpret_cast<void**>(&dxgiFactory4)))) {
        HRESULT luidHr = dxgiFactory4->EnumAdapterByLuid(gfxReq.adapterLuid, __uuidof(IDXGIAdapter1), reinterpret_cast<void**>(&chosenAdapter));
        Log_Printf("XRBridge: IDXGIFactory4::EnumAdapterByLuid -> %s (hr=0x%08lX)", SUCCEEDED(luidHr) ? "OK" : "FAILED", luidHr);
        dxgiFactory4->Release();
    } else {
        Log_Printf("XRBridge: QueryInterface(IDXGIFactory4) failed - falling back to manual EnumAdapters1 loop");
    }

    if (!chosenAdapter) {
        for (UINT i = 0; ; ++i) {
            IDXGIAdapter1* adapter = nullptr;
            if (dxgiFactory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND)
                break;
            DXGI_ADAPTER_DESC1 desc;
            adapter->GetDesc1(&desc);
            if (desc.AdapterLuid.LowPart == gfxReq.adapterLuid.LowPart && desc.AdapterLuid.HighPart == gfxReq.adapterLuid.HighPart) {
                chosenAdapter = adapter;
                break;
            }
            adapter->Release();
        }
    }
    dxgiFactory->Release();
    if (!chosenAdapter) {
        Log_Printf("XRBridge: could not find the DXGI adapter matching OpenXR's required LUID (%08lX:%08lX)",
            gfxReq.adapterLuid.HighPart, gfxReq.adapterLuid.LowPart);
        return false;
    }

    // 2026-09-10: try the D3D11 debug/validation layer first - every
    // plausible cause for the D3D12 addon bridge's OpenSharedResource1
    // E_INVALIDARG has been ruled out with direct evidence (stale
    // handle, format, adapter mismatch, resource state,
    // SharedResourceCompatibilityTier - see re5vr_project memory), so a
    // bare HRESULT isn't enough anymore. The debug layer emits specific,
    // human-readable validation messages via OutputDebugString
    // (catchable with DebugView, same tool already used earlier this
    // session) that should explain exactly what it doesn't like about
    // the shared-resource open, instead of continuing to guess. Falls
    // back to the plain device if the debug layer isn't installed
    // (DXGI_ERROR_SDK_COMPONENT_MISSING) - this machine has Visual
    // Studio installed so it likely is, but don't hard-fail the whole
    // bridge if not.
    D3D_FEATURE_LEVEL achievedLevel;
    HRESULT hr = D3D11CreateDevice(chosenAdapter, D3D_DRIVER_TYPE_UNKNOWN, nullptr, D3D11_CREATE_DEVICE_DEBUG, nullptr, 0,
        D3D11_SDK_VERSION, &g_d3d11Device, &achievedLevel, &g_d3d11Context);
    if (FAILED(hr)) {
        Log_Printf("XRBridge: D3D11CreateDevice with D3D11_CREATE_DEVICE_DEBUG failed (hr=0x%08lX) - debug layer probably not installed, falling back to non-debug device", hr);
        hr = D3D11CreateDevice(chosenAdapter, D3D_DRIVER_TYPE_UNKNOWN, nullptr, 0, nullptr, 0,
            D3D11_SDK_VERSION, &g_d3d11Device, &achievedLevel, &g_d3d11Context);
    }
    chosenAdapter->Release();
    if (FAILED(hr)) {
        Log_Printf("XRBridge: D3D11CreateDevice (on OpenXR's required adapter) failed (hr=0x%08lX)", hr);
        return false;
    }
    Log_Printf("XRBridge: D3D11 device created, achieved feature level = 0x%04X", static_cast<unsigned>(achievedLevel));

    // An ID3D11DeviceContext is NOT thread-safe, and as of the D3D12
    // addon path there are genuinely two threads using this one:
    // XrSubmitOneFrame (submit thread) copies eye textures into the
    // OpenXR swapchain images, while VRBridge_OnEndScene (the GAME's
    // main thread) copies the addon's shared frame into those same eye
    // textures via D3D12AddonBridge_CopyToEyeSlots.
    //
    // This is almost certainly the freeze that has survived every
    // producer-side fix so far: it appeared the moment the addon path
    // started actually delivering frames, which is precisely when the
    // main thread began touching this context at all. Before that,
    // CopyToEyeSlots never ran and the submit thread had the context to
    // itself - which is why the old D3D9 path never hit this.
    //
    // SetMultithreadProtected makes the runtime take its own lock around
    // context access, which is exactly what this situation is for.
    ID3D11Multithread* multithread = nullptr;
    if (SUCCEEDED(g_d3d11Context->QueryInterface(__uuidof(ID3D11Multithread), reinterpret_cast<void**>(&multithread)))) {
        const BOOL wasProtected = multithread->SetMultithreadProtected(TRUE);
        Log_Printf("XRBridge: ID3D11Multithread::SetMultithreadProtected(TRUE) -> OK (was %s)",
            wasProtected ? "already protected" : "unprotected");
        multithread->Release();
    } else {
        Log_Printf("XRBridge: WARNING - QueryInterface(ID3D11Multithread) failed; the submit thread and the game's main thread will share an unprotected D3D11 context");
    }

    // 2026-09-12: the addon bridge init USED to run here, before
    // xrCreateSession, because the swapchain format below needs to match
    // the addon's real source format. It only has to happen before that
    // choice, though - not before the session - and doing it first meant we
    // handed the runtime a D3D11 device that already had two foreign D3D12
    // shared textures opened on it, which no ordinary OpenXR app does. A
    // tester's process was killed outright inside xrCreateSession on Meta's
    // runtime, so it now runs AFTER the session exists, leaving the device
    // in the state a runtime expects at binding time. Nothing else about it
    // changes; see the format-selection block below, which is still the
    // first thing that reads g_usingD3D12AddonPath.
    XrGraphicsBindingD3D11KHR binding{ XR_TYPE_GRAPHICS_BINDING_D3D11_KHR };
    binding.device = g_d3d11Device;

    XrSessionCreateInfo sessionInfo{ XR_TYPE_SESSION_CREATE_INFO };
    sessionInfo.next = &binding;
    sessionInfo.systemId = g_xrSystemId;
    // Guarded because Meta's own runtime faults in here. Confirmed
    // 2026-09-12 from Windows Error Reporting on two machines: an access
    // violation at RuntimeIPCServiceClient_32.dll+0x28561 (Meta Horizon
    // 208.0.47.535), inside their 32-bit runtime's service IPC client, which
    // took the whole game down with it. Nothing on our side can prevent
    // their bug - but losing the player's session to it is avoidable, so
    // catch it, leave XR off, and let them keep playing flat.
    Log_Printf("XRBridge: calling xrCreateSession (D3D11 device=%p)", g_d3d11Device);
    DWORD sehCode = 0;
    r = CreateSessionGuarded(g_xrInstance, &sessionInfo, &g_xrSession, &sehCode);
    if (sehCode != 0) {
        Log_Printf("XRBridge: xrCreateSession CRASHED inside the runtime (SEH code 0x%08lX) - VR not started, "
                   "the game keeps running. This is a bug in the OpenXR runtime itself; on Meta's runtime the "
                   "known fault is RuntimeIPCServiceClient_32.dll. Switching the active OpenXR runtime to "
                   "SteamVR avoids it.",
            sehCode);
        return false;
    }
    Log_Printf("XRBridge: xrCreateSession returned %s", XrResultName(r));
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrCreateSession failed -> %s", XrResultName(r));
        return false;
    }

    XrReferenceSpaceCreateInfo spaceInfo{ XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    spaceInfo.poseInReferenceSpace.orientation = { 0, 0, 0, 1 };
    r = xrCreateReferenceSpace(g_xrSession, &spaceInfo, &g_xrLocalSpace);
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrCreateReferenceSpace failed -> %s", XrResultName(r));
        return false;
    }

    // Now that the runtime has the device, open the dgVoodoo2 addon's shared
    // D3D12 frame on it - see the note above xrCreateSession. Must still
    // happen before the swapchain format is chosen, since that format has to
    // match the addon's real source format family rather than the D3D9
    // backbuffer's.
    D3D12AddonBridgeInfo addonInfo;
    g_usingD3D12AddonPath = D3D12AddonBridge_TryInit(g_d3d11Device, &addonInfo);
    if (g_usingD3D12AddonPath)
        g_addonFrameFormat = addonInfo.format;

    // Pick a swapchain format matching our D3D9 backbuffer's byte layout
    // (D3DFMT_A8R8G8B8 is BGRA) AND its gamma encoding. Prefer the sRGB
    // variant (DXGI_FORMAT_B8G8R8A8_UNORM_SRGB = 91), not plain UNORM (87)
    // - a real bug found 2026-07-28: the first working build (plain UNORM)
    // delivered real pixels (format-mismatch/black-screen bug already
    // fixed) but looked washed-out/wrong-color in the headset, desktop
    // fine. Real game rendering is always sRGB-gamma-encoded; a compositor
    // fed a plain UNORM swapchain either skips or wrongly re-applies gamma
    // correction, producing exactly that washed-out look. Confirmed via
    // fear-vr's own working M1 log (docs/M1-OPENXR-HOST.md /
    // ENVIRONMENT.md): their swapchain format is
    // "DXGI_FORMAT_R8G8B8A8_UNORM_SRGB" - they deliberately chose the sRGB
    // variant too. Safe to still CopyResource from our UNORM-formatted
    // bridge texture (kBridgeFormat) into an SRGB-formatted swapchain
    // image - D3D11 explicitly allows CopyResource between a format and
    // its _SRGB counterpart (same underlying TYPELESS format, only the
    // sampling/blending interpretation differs, which doesn't apply to a
    // raw resource-to-resource copy) - only the *interpretation* changes,
    // not the bytes, which is exactly the fix. Falls back to plain UNORM,
    // then whatever the runtime lists first, if the SRGB variant isn't
    // offered.
    uint32_t formatCount = 0;
    xrEnumerateSwapchainFormats(g_xrSession, 0, &formatCount, nullptr);
    std::vector<int64_t> formats(formatCount);
    xrEnumerateSwapchainFormats(g_xrSession, formatCount, &formatCount, formats.data());
    int64_t chosenFormat = formats.empty() ? 0 : formats[0];

    // 2026-09-10: when the D3D12 addon bridge is active, its real source
    // format (queried from the actual ID3D12Resource, not assumed) may be
    // a different DXGI family than the D3D9 path's BGRA - most likely
    // R8G8B8A8 (28) rather than B8G8R8A8 (87), since that's what
    // dgVoodoo2's own D3D12 backend used in the first working test. Only
    // these two families are handled explicitly (the ones actually
    // observed/plausible here) - anything else falls through to using
    // the addon's exact reported format with no _SRGB upgrade, logged so
    // it's visible rather than silently guessed at.
    constexpr int64_t kDxgiFormatB8G8R8A8Unorm = 87;
    constexpr int64_t kDxgiFormatB8G8R8A8UnormSrgb = 91;
    constexpr int64_t kDxgiFormatR8G8B8A8Unorm = 28;
    constexpr int64_t kDxgiFormatR8G8B8A8UnormSrgb = 29;

    int64_t preferredUnorm = kDxgiFormatB8G8R8A8Unorm;
    int64_t preferredSrgb = kDxgiFormatB8G8R8A8UnormSrgb;
    if (g_usingD3D12AddonPath) {
        const int64_t addonFormat = static_cast<int64_t>(g_addonFrameFormat);
        if (addonFormat == kDxgiFormatR8G8B8A8Unorm) {
            preferredUnorm = kDxgiFormatR8G8B8A8Unorm;
            preferredSrgb = kDxgiFormatR8G8B8A8UnormSrgb;
        } else if (addonFormat == kDxgiFormatB8G8R8A8Unorm) {
            preferredUnorm = kDxgiFormatB8G8R8A8Unorm;
            preferredSrgb = kDxgiFormatB8G8R8A8UnormSrgb;
        } else {
            preferredUnorm = addonFormat;
            preferredSrgb = addonFormat; // no known _SRGB counterpart - use as-is
            Log_Printf("XRBridge: D3D12 addon bridge reports an unrecognized format (%lld) - using it exactly with no _SRGB upgrade",
                static_cast<long long>(addonFormat));
        }
    }

    bool foundSrgb = false;
    for (int64_t f : formats) {
        if (f == preferredSrgb) {
            chosenFormat = f;
            foundSrgb = true;
            break;
        }
    }
    if (!foundSrgb) {
        for (int64_t f : formats) {
            if (f == preferredUnorm) {
                chosenFormat = f;
                break;
            }
        }
        Log_Printf("XRBridge: WARNING - runtime doesn't list the preferred sRGB format (%lld), using format %lld instead - colors may be washed out or off",
            static_cast<long long>(preferredSrgb), static_cast<long long>(chosenFormat));
    }

    (void)d3d9Format;
    // 2026-07-29: allocate the swapchain at the runtime's OWN recommended
    // size (queried in InitOpenXRInstanceAndSystem), not just our rendered
    // content size - suspected the runtime treats a far-below-recommended
    // swapchain as an atypical/legacy client and caps frame rate as a
    // result (see project memory). Our actual rendered content still only
    // fills a g_eyeWidth x g_eyeHeight sub-rectangle of it (copied via
    // CopySubresourceRegion below, and the composition layer's imageRect
    // stays exactly that size) - this tests swapchain size in isolation
    // without changing render resolution/cost at all.
    const UINT swapchainW = (std::max)(eyeWidth, g_recommendedEyeWidth);
    const UINT swapchainH = (std::max)(eyeHeight, g_recommendedEyeHeight);
    for (int eye = 0; eye < 2; ++eye) {
        XrSwapchainCreateInfo scInfo{ XR_TYPE_SWAPCHAIN_CREATE_INFO };
        scInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        scInfo.format = chosenFormat;
        scInfo.sampleCount = 1;
        scInfo.width = swapchainW;
        scInfo.height = swapchainH;
        scInfo.faceCount = 1;
        scInfo.arraySize = 1;
        scInfo.mipCount = 1;
        r = xrCreateSwapchain(g_xrSession, &scInfo, &g_xrSwapchain[eye]);
        if (XR_FAILED(r)) {
            Log_Printf("XRBridge: xrCreateSwapchain(%s) failed -> %s", eye == 0 ? "left" : "right", XrResultName(r));
            return false;
        }

        uint32_t imageCount = 0;
        xrEnumerateSwapchainImages(g_xrSwapchain[eye], 0, &imageCount, nullptr);
        g_xrSwapchainImages[eye].resize(imageCount, XrSwapchainImageD3D11KHR{ XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR });
        r = xrEnumerateSwapchainImages(g_xrSwapchain[eye], imageCount, &imageCount,
            reinterpret_cast<XrSwapchainImageBaseHeader*>(g_xrSwapchainImages[eye].data()));
        if (XR_FAILED(r)) {
            Log_Printf("XRBridge: xrEnumerateSwapchainImages(%s) failed -> %s", eye == 0 ? "left" : "right", XrResultName(r));
            return false;
        }
        Log_Printf("XRBridge: swapchain %s ready - %u images at %ux%u (content %ux%u), format=%lld",
            eye == 0 ? "left" : "right", imageCount, swapchainW, swapchainH, eyeWidth, eyeHeight,
            static_cast<long long>(chosenFormat));
    }

    Log_Printf("XRBridge: session + swapchains ready");
    return true;
}

// ---- Frame submission (synchronous, from the game's own EndScene) -------

// 2026-07-29: NOT the display rate (1000/90) on purpose - see
// ShouldCopyThisCall's comment. Oversampling the copy relative to the
// independent ~90Hz submit thread avoids the two free-running clocks
// beating against each other and reusing stale eye textures.
// Back to 400 Hz, and this time it is measured rather than assumed
// (2026-09-12). Three configurations, from the "image age at submit" line:
//   waiting for the consumer, 400 Hz cap : 59 ms avg, 118 worst
//   no wait, 400 Hz cap                  : 36 ms avg,  57 worst
//   no wait, 120 Hz cap                  : 44-47 ms avg, and MICROSTUTTER
// The 120 Hz cap looked principled - why produce faster than the headset
// consumes - but it recreated exactly what the original oversampling existed
// to avoid: two free-running clocks beating against each other. Oversampling
// costs some redundant copies and buys steady, fresh frames. Keep it.
double g_targetFrameIntervalMs = 1000.0 / 400.0;
bool g_xrThreadStarted = false;
HANDLE g_xrThread = nullptr; // 2026-07-29: dedicated submit thread, see EnsureXrThreadStarted

void PumpXrEvents()
{
    XrEventDataBuffer event{ XR_TYPE_EVENT_DATA_BUFFER };
    while (xrPollEvent(g_xrInstance, &event) == XR_SUCCESS) {
        if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            const auto& stateEvent = reinterpret_cast<const XrEventDataSessionStateChanged&>(event);
            Log_Printf("XRBridge: session state -> %d", static_cast<int>(stateEvent.state));
            if (stateEvent.state == XR_SESSION_STATE_READY && !g_xrSessionRunning) {
                XrSessionBeginInfo beginInfo{ XR_TYPE_SESSION_BEGIN_INFO };
                beginInfo.primaryViewConfigurationType = kDiagnosticViewConfigType;
                XrResult r = xrBeginSession(g_xrSession, &beginInfo);
                Log_Printf("XRBridge: xrBeginSession -> %s", XR_SUCCEEDED(r) ? "OK" : XrResultName(r));
                g_xrSessionRunning = XR_SUCCEEDED(r);

                if (g_xrSessionRunning && g_displayRefreshRateExtSupported) {
                    uint32_t rateCount = 0;
                    g_xrEnumerateDisplayRefreshRatesFB(g_xrSession, 0, &rateCount, nullptr);
                    std::vector<float> rates(rateCount);
                    g_xrEnumerateDisplayRefreshRatesFB(g_xrSession, rateCount, &rateCount, rates.data());
                    float maxRate = 0.0f;
                    std::string rateList;
                    char rateBuf[32];
                    for (float rate : rates) {
                        _snprintf_s(rateBuf, sizeof(rateBuf), _TRUNCATE, "%.1f ", rate);
                        rateList += rateBuf;
                        if (rate > maxRate) maxRate = rate;
                    }
                    Log_Printf("XRBridge: available display refresh rates: %s", rateList.c_str());

                    if (maxRate > 0.0f) {
                        XrResult rr = g_xrRequestDisplayRefreshRateFB(g_xrSession, maxRate);
                        Log_Printf("XRBridge: xrRequestDisplayRefreshRateFB(%.1f) -> %s",
                            maxRate, XR_SUCCEEDED(rr) ? "OK" : XrResultName(rr));
                    }
                }

                // 2026-07-29: adb logcat caught VirtualDesktop.Android (the
                // on-device half of the runtime) explicitly setting its own
                // CPU perf level to "sustained low" via this exact extension
                // right at session start, held for an entire test that
                // stayed capped at 22.22ms/45Hz - see project memory. Ask
                // for BOOST on both domains ourselves, as the other
                // connected OpenXR client, to see if it overrides or merges
                // with VD's own default.
                if (g_xrSessionRunning && g_perfSettingsExtSupported) {
                    XrResult rrCpu = g_xrPerfSettingsSetPerformanceLevelEXT(
                        g_xrSession, XR_PERF_SETTINGS_DOMAIN_CPU_EXT, XR_PERF_SETTINGS_LEVEL_BOOST_EXT);
                    Log_Printf("XRBridge: xrPerfSettingsSetPerformanceLevelEXT(cpu, boost) -> %s",
                        XR_SUCCEEDED(rrCpu) ? "OK" : XrResultName(rrCpu));
                    XrResult rrGpu = g_xrPerfSettingsSetPerformanceLevelEXT(
                        g_xrSession, XR_PERF_SETTINGS_DOMAIN_GPU_EXT, XR_PERF_SETTINGS_LEVEL_BOOST_EXT);
                    Log_Printf("XRBridge: xrPerfSettingsSetPerformanceLevelEXT(gpu, boost) -> %s",
                        XR_SUCCEEDED(rrGpu) ? "OK" : XrResultName(rrGpu));
                }
            } else if (stateEvent.state == XR_SESSION_STATE_STOPPING && g_xrSessionRunning) {
                xrEndSession(g_xrSession);
                g_xrSessionRunning = false;
                Log_Printf("XRBridge: xrEndSession called (STOPPING)");
            }
        }
        event = { XR_TYPE_EVENT_DATA_BUFFER };
    }
}

// 2026-07-29: was briefly made synchronous (called directly from the
// game's own EndScene, one call per real game frame) to test whether the
// old ~45Hz predictedDisplayPeriod cap was tied to running on a decoupled
// thread, matching REFramework's pattern. That theory was refuted (see
// project memory) and the cap later disappeared for unrelated reasons -
// but the synchronous design turned out to have its own real cost: since
// xrWaitFrame's per-call block duration is genuinely jittery frame to
// frame (observed bouncing between ~0ms and 5-10ms against an 11.11ms
// target, even in otherwise-healthy sessions), and the whole submission
// sequence ran directly inside the game's render thread, that jitter
// became the game's OWN frame pacing in real time - with nothing to
// smooth it out. User-reported symptom: visible stutter/judder (not lag),
// consistent with frame-to-frame timing variance rather than a throughput
// problem. Restoring a dedicated thread (see XrSubmitThreadProc below) so
// the game can render at its own pace into the double-buffer and a
// separate thread submits to the compositor on its own cadence - jitter
// in the runtime's own wake-up timing no longer propagates directly into
// what the game (and therefore the headset) feels frame to frame.
void XrSubmitOneFrame()
{
    static UINT64 iterCount = 0;

    // Periodic (non-exhausting) logging gate for this whole call - see
    // ScopedTimer's bool constructor. Every ScopedTimer below shares this
    // same flag so one logged call shows the full breakdown together, and
    // it keeps firing every 30th call for the entire session lifetime
    // instead of going silent after a fixed budget runs out.
    ++iterCount;
    const bool logThisIteration = (iterCount % 30 == 0);

    {
        ScopedTimer t("PumpXrEvents", logThisIteration);
        PumpXrEvents();
    }
    if (!g_xrSessionRunning)
        return;

    XrFrameWaitInfo waitInfo{ XR_TYPE_FRAME_WAIT_INFO };
    XrFrameState frameState{ XR_TYPE_FRAME_STATE };
    XrResult r;
    {
        ScopedTimer t("xrWaitFrame", logThisIteration);
        r = xrWaitFrame(g_xrSession, &waitInfo, &frameState);
    }
    if (XR_FAILED(r)) {
        Log_Printf("XRBridge: xrWaitFrame failed -> %s", XrResultName(r));
        return;
    }
        if (frameState.shouldRender)
            ++g_realContentFrameCount;
        const bool placeholderMode = kDiagnosticPlaceholderMode &&
            g_realContentFrameCount > 0 && g_realContentFrameCount <= kPlaceholderModeFrames;
        if (placeholderMode && !g_placeholderModeEverActive) {
            g_placeholderModeEverActive = true;
            Log_Printf("XRBridge: placeholder diagnostic window STARTED (frame %d/%d) - submitting blank layers, watch predictedDisplayPeriod",
                g_realContentFrameCount, kPlaceholderModeFrames);
        } else if (!placeholderMode && g_placeholderModeEverActive && g_realContentFrameCount == kPlaceholderModeFrames + 1) {
            Log_Printf("XRBridge: placeholder diagnostic window ENDED - back to real content, watch predictedDisplayPeriod for any change");
        }
        // Dense (every-frame) logging through the whole diagnostic window
        // plus a short tail after it ends - the periodic 30-frame gate
        // (logThisIteration) is fine for general steady-state monitoring but
        // would miss the exact transition moments that matter most here.
        const bool densePlaceholderLogging = kDiagnosticPlaceholderMode &&
            g_realContentFrameCount > 0 && g_realContentFrameCount <= (kPlaceholderModeFrames + 30);
        if (logThisIteration || densePlaceholderLogging) {
            Log_Printf("XRBridge: frameState predictedDisplayPeriod=%.2fms shouldRender=%d placeholderMode=%d frameNum=%d",
                static_cast<double>(frameState.predictedDisplayPeriod) / 1000000.0,
                frameState.shouldRender, placeholderMode ? 1 : 0, g_realContentFrameCount);
        }

        XrFrameBeginInfo beginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
        {
            ScopedTimer t("xrBeginFrame", logThisIteration);
            r = xrBeginFrame(g_xrSession, &beginInfo);
        }
        if (XR_FAILED(r)) {
            Log_Printf("XRBridge: xrBeginFrame failed -> %s", XrResultName(r));
            return;
        }

        XrView views[2] = { { XR_TYPE_VIEW }, { XR_TYPE_VIEW } };
        XrCompositionLayerProjectionView projViews[2] = {};
        uint32_t viewCount = 0;
        bool haveViews = false;
        if (frameState.shouldRender) {
            XrViewLocateInfo locateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
            locateInfo.viewConfigurationType = kDiagnosticViewConfigType;
            locateInfo.displayTime = frameState.predictedDisplayTime;
            locateInfo.space = g_xrLocalSpace;
            XrViewState viewState{ XR_TYPE_VIEW_STATE };
            {
                ScopedTimer t("xrLocateViews", logThisIteration);
                r = xrLocateViews(g_xrSession, &locateInfo, &viewState, 2, &viewCount, views);
            }
            haveViews = XR_SUCCEEDED(r) && viewCount >= 1;
            // xrLocateViews can SUCCEED with orientation not yet actually
            // tracked (this is exactly the normal ~400+ frame window right
            // after xrBeginSession where xrEndFrame itself returns
            // XR_ERROR_POSE_INVALID - see openxr_bridge.h). Without this
            // check, the recenter-on-enable logic below could capture that
            // untracked placeholder pose as an eye's permanent reference
            // orientation - and since the runtime's placeholder isn't
            // guaranteed identical between the two eyes, this produced a
            // real observed bug: one eye landing dozens of degrees
            // "cockeyed" while the other looked fine, baked in permanently
            // for the rest of that XR session. Requiring ORIENTATION_VALID
            // alone (first attempt, 2026-07-28) did NOT fix this in
            // practice - the runtime evidently reports VALID (a
            // well-defined, non-NaN value) well before TRACKED (that value
            // being real, live tracking rather than a default/predicted
            // placeholder), so also require TRACKED before trusting the
            // pose enough to capture as a reference.
            const bool orientationValid = haveViews &&
                (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0 &&
                (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_TRACKED_BIT) != 0;

            if (haveViews) {
                // Built locally and published in one go under the lock: the
                // render thread reads these on its own schedule and must
                // never see half of one pose and half of the next. (This
                // thread is the only writer, so reading them here is safe.)
                XRBridgeEyeView next[2];
                std::memcpy(next, g_eyeViews, sizeof(next));
                for (int eye = 0; eye < static_cast<int>(viewCount); ++eye) {
                    next[eye].positionMeters[0] = views[eye].pose.position.x;
                    next[eye].positionMeters[1] = views[eye].pose.position.y;
                    next[eye].positionMeters[2] = views[eye].pose.position.z;

                    const XrQuaternionf& q = views[eye].pose.orientation;
                    // Belt-and-suspenders on top of orientationValid: this
                    // runtime (VirtualDesktopXR) has been observed to
                    // report viewStateFlags claiming BOTH
                    // ORIENTATION_VALID and ORIENTATION_TRACKED on the very
                    // first xrLocateViews call after xrBeginSession while
                    // still handing back an all-zero quaternion (0,0,0,0) -
                    // not a real rotation at all (a genuine unit quaternion
                    // always has length 1; QuaternionToMat3 of an all-zero
                    // input produces a REFLECTION matrix, determinant -1,
                    // not a rotation). Capturing that as a reference
                    // permanently corrupts every subsequent frame's delta
                    // for that eye - confirmed as the real root cause of
                    // the observed "one eye cockeyed" bug (requiring the
                    // TRACKED bit alone, tried first, did NOT catch this -
                    // the flags themselves lie in this specific case).
                    // Don't trust the flags alone; sanity-check the data.
                    const float qLenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
                    const bool poseUsable = orientationValid && qLenSq > 0.5f && g_framesSincePoseLock >= kPoseSettleFrames;

                    if (poseUsable) {
                        // Recenter-on-enable, per eye - see
                        // g_haveEyeReference's declaration. Mirrors the old
                        // OpenVR bridge's single shared head-delta
                        // computation (hmdNow * transpose(reference)), just
                        // done twice, once per eye, so any real per-eye
                        // toe-in the runtime reports survives into the
                        // delta instead of being cancelled out by a shared
                        // reference.
                        const Mat3 nowMat = QuaternionToMat3(q.x, q.y, q.z, q.w);
                        if (!g_haveEyeReference[eye]) {
                            g_eyeReference[eye] = nowMat;
                            g_haveEyeReference[eye] = true;
                            // NOTE: XrViewStateFlags is a 64-bit XrFlags64 -
                            // must NOT be passed to a 32-bit %X on this
                            // Win32 build (cdecl varargs are pushed by
                            // exact type size; a 64-bit arg where a 32-bit
                            // one is expected misaligns every following
                            // vararg on the stack). This bug was
                            // responsible for an earlier misdiagnosis - the
                            // quat values this line printed before the
                            // cast was added were garbage read from the
                            // wrong stack offset, not the real quaternion.
                            Log_Printf("XRBridge: eye %d reference orientation captured (flags=0x%016llX, quat=%.4f,%.4f,%.4f,%.4f)",
                                eye, static_cast<unsigned long long>(viewState.viewStateFlags), q.x, q.y, q.z, q.w);
                        }
                        Mat3 delta = Mat3Multiply(nowMat, Mat3Transpose(g_eyeReference[eye]));
                        // Head-rotation gain (2026-09-12). The user wants a
                        // smaller neck turn to cover more world - "what if we
                        // add more to it". Applied HERE, at the one place the
                        // delta is published, so the eye matrices and F9's
                        // head-follow scale together and can never disagree.
                        const float gain = g_headRotationGain.load(std::memory_order_relaxed);
                        if (std::fabs(gain - 1.0f) > 1e-3f)
                            delta = ScaleRotation(delta, gain);

                        // Latency prediction (2026-09-12). Unifying the pose
                        // snapshots killed the overshoot-and-snap, but the
                        // view still drags at normal turning speed: the frame
                        // you see was rendered from where your head WAS, and
                        // this pipeline is deep - game, dgVoodoo, D3D12 addon,
                        // shared texture, submit thread, compositor. So take
                        // the rotation since the last publish as the current
                        // angular velocity and extrapolate it forward by
                        // g_headPredictMs. Home / End tune it live; 0 is off.
                        const float predictMs = g_headPredictMs.load(std::memory_order_relaxed);
                        if (predictMs > 0.1f) {
                            const double nowMs = NowMillis();
                            HeadPredictState& hp = g_headPredict[eye];
                            const double stepMs = nowMs - hp.lastMs;
                            if (hp.valid && stepMs > 0.2 && stepMs < 100.0) {
                                // Angular velocity of the head, as an
                                // axis-angle vector in radians per ms, then
                                // smoothed - one raw step is far too noisy to
                                // extrapolate tens of milliseconds from.
                                const Mat3 increment = Mat3Multiply(delta, Mat3Transpose(hp.last));
                                float axis[3];
                                float angle = 0.0f;
                                Mat3ToAxisAngle(increment, axis, &angle);
                                const float rate = angle / static_cast<float>(stepMs);
                                for (int i = 0; i < 3; ++i) {
                                    const float sample = axis[i] * rate;
                                    hp.omega[i] += (sample - hp.omega[i]) * kHeadOmegaSmoothing;
                                }

                                const float speed = std::sqrt(hp.omega[0] * hp.omega[0] +
                                    hp.omega[1] * hp.omega[1] + hp.omega[2] * hp.omega[2]);
                                if (speed > 1e-6f) {
                                    const float dir[3] = { hp.omega[0] / speed, hp.omega[1] / speed, hp.omega[2] / speed };
                                    const Mat3 ahead = AxisAngleToMat3(dir, speed * predictMs);
                                    hp.last = delta;
                                    hp.lastMs = nowMs;
                                    delta = Mat3Multiply(ahead, delta);
                                } else {
                                    hp.last = delta;
                                    hp.lastMs = nowMs;
                                }
                            } else {
                                hp.last = delta;
                                hp.lastMs = nowMs;
                                hp.valid = true;
                            }
                        }
                        std::memcpy(next[eye].rotationDelta, delta.m, sizeof(delta.m));

                        // DIAGNOSTIC (2026-07-28): the 90-frame settle delay
                        // (see g_framesSincePoseLock) eliminated the huge
                        // one-frame spike seen before (136 degrees), but a
                        // real several-degree jitter (2.5-9 degrees/frame)
                        // persisted for the entire first 20 logged samples
                        // (~500ms) after settling - open question: does this
                        // decay toward 0 given more time, or is it a
                        // continuous characteristic of how this runtime
                        // delivers poses at the already-degraded ~40Hz
                        // frame rate? The first 20 log every single frame
                        // for fine detail near the start; after that, log
                        // every 30th frame indefinitely so a longer test
                        // run shows the trend instead of just the opening
                        // burst.
                        ++g_deltaLogCount[eye];
                        if (g_deltaLogCount[eye] <= 20 || g_deltaLogCount[eye] % 30 == 0) {
                            float trace = delta.m[0] + delta.m[4] + delta.m[8];
                            if (trace > 3.0f) trace = 3.0f;
                            if (trace < -1.0f) trace = -1.0f;
                            const float angleDeg = std::acos((trace - 1.0f) * 0.5f) * (180.0f / 3.14159265f);
                            Log_Printf("XRBridge: eye %d delta #%d angle=%.2fdeg row0=(%.3f,%.3f,%.3f) row1=(%.3f,%.3f,%.3f) row2=(%.3f,%.3f,%.3f)",
                                eye, g_deltaLogCount[eye], angleDeg,
                                delta.m[0], delta.m[1], delta.m[2],
                                delta.m[3], delta.m[4], delta.m[5],
                                delta.m[6], delta.m[7], delta.m[8]);
                        }
                    } else if (!g_haveEyeReference[eye]) {
                        // No reference yet and this frame's orientation
                        // isn't trustworthy either - report identity rather
                        // than risk it. If we DO already have a reference,
                        // this is just a brief mid-session tracking loss -
                        // leave rotationDelta as whatever it was last frame
                        // instead of overwriting with an untrustworthy one.
                        const Mat3 identity = Mat3Identity();
                        std::memcpy(next[eye].rotationDelta, identity.m, sizeof(identity.m));
                    }

                    next[eye].angleLeft = views[eye].fov.angleLeft;
                    next[eye].angleRight = views[eye].fov.angleRight;
                    next[eye].angleUp = views[eye].fov.angleUp;
                    next[eye].angleDown = views[eye].fov.angleDown;
                }
                // Stamp this locate's raw poses into the ring before
                // publishing the derived views, so the id the render thread
                // is about to read always resolves to a complete entry.
                const XRBridgePoseId poseId = g_nextPoseId.fetch_add(1, std::memory_order_relaxed);
                RenderedPose& entry = g_poseRing[poseId % kPoseRingSize];
                entry.id.store(0, std::memory_order_relaxed); // mark incomplete while writing
                for (int eye = 0; eye < 2; ++eye) {
                    entry.pose[eye] = views[eye].pose;
                    entry.fov[eye] = views[eye].fov;
                }
                entry.publishedMs = NowMillis();
                entry.id.store(poseId, std::memory_order_release);

                AcquireSRWLockExclusive(&g_eyeViewsLock);
                std::memcpy(g_eyeViews, next, sizeof(g_eyeViews));
                g_haveEyeViews = true;
                ReleaseSRWLockExclusive(&g_eyeViewsLock);
                g_currentPoseId.store(poseId, std::memory_order_release);
            }

            // Read whichever slot the producer most recently published as
            // front - never the slot it might currently be writing into
            // (see g_eyeFrontIndex's comment). Track generation per eye to
            // detect and count resubmitting the same image again, matching
            // fear-vr's own reusedFrames_ diagnostic (their docs: this is
            // NOT an error, it's expected whenever the XR compositor's own
            // cadence outruns how often the game produces a genuinely new
            // frame - see project memory on their TESTING.md numbers).
            const int leftFrontSlot = g_eyeFrontIndex[kEyeLeft].load(std::memory_order_acquire);
            const int rightFrontSlot = g_eyeFrontIndex[kEyeRight].load(std::memory_order_acquire);
            ID3D11Texture2D* srcTex[2] = { g_d3d11LeftTex[leftFrontSlot], g_d3d11RightTex[rightFrontSlot] };

            // Which pose were these pixels actually drawn with? Submitting
            // the freshly-located pose instead is what made the image lurch
            // on every head movement (see openxr_bridge.h). Fall back to the
            // fresh pose only when the tag is missing or has already been
            // overwritten in the ring - i.e. the game is more than
            // kPoseRingSize frames behind, at which point nothing is going
            // to look good anyway.
            XrPosef submitPose[2] = { views[0].pose, views[1].pose };
            XrFovf submitFov[2] = { views[0].fov, views[1].fov };
            const XRBridgePoseId taggedId = g_slotPoseId[leftFrontSlot].load(std::memory_order_acquire);
            if (taggedId != 0) {
                const RenderedPose& entry = g_poseRing[taggedId % kPoseRingSize];
                if (entry.id.load(std::memory_order_acquire) == taggedId) {
                    for (int eye = 0; eye < 2; ++eye) {
                        submitPose[eye] = entry.pose[eye];
                        submitFov[eye] = entry.fov[eye];
                    }
                    // End-to-end age: how long ago the pose this image was
                    // rendered with was located. That is the delay the user
                    // sees as the headset trailing the desktop mirror, and it
                    // is the number to beat if we start removing pipeline
                    // hops (addon slot -> eye texture -> swapchain).
                    const double ageMs = NowMillis() - entry.publishedMs;
                    if (ageMs >= 0.0 && ageMs < 1000.0) {
                        g_frameAgeSumMs += ageMs;
                        ++g_frameAgeCount;
                        if (ageMs > g_frameAgeWorstMs)
                            g_frameAgeWorstMs = ageMs;
                    }
                    g_poseHits.fetch_add(1, std::memory_order_relaxed);
                } else {
                    g_poseMisses.fetch_add(1, std::memory_order_relaxed);
                }
            } else {
                g_poseMisses.fetch_add(1, std::memory_order_relaxed);
            }
            const UINT64 curGeneration[2] = { g_eyeGeneration[kEyeLeft].load(std::memory_order_acquire), g_eyeGeneration[kEyeRight].load(std::memory_order_acquire) };
            static UINT64 s_lastSubmittedGeneration[2] = { 0, 0 };
            static UINT64 s_reusedFrameCount[2] = { 0, 0 };
            static UINT64 s_submittedFrameCount[2] = { 0, 0 };
            const char* eyeNames[2] = { "left", "right" };
            for (int eye = 0; eye < static_cast<int>(viewCount) && haveViews && srcTex[eye]; ++eye) {
                ++s_submittedFrameCount[eye];
                if (curGeneration[eye] != 0 && curGeneration[eye] == s_lastSubmittedGeneration[eye])
                    ++s_reusedFrameCount[eye];
                s_lastSubmittedGeneration[eye] = curGeneration[eye];
                // Tell the producer this frame has been taken - see
                // ShouldCopyThisCall. Until it changes, the game thread has
                // no reason to touch the shared D3D11 context again.
                g_eyeConsumedGeneration[eye].store(curGeneration[eye], std::memory_order_release);
                if (logThisIteration) {
                    Log_Printf("XRBridge: eye %d reused=%llu/%llu submitted frames", eye,
                        s_reusedFrameCount[eye], s_submittedFrameCount[eye]);
                }
                char label[64];
                XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
                uint32_t imageIndex = 0;
                XrResult acquireResult;
                {
                    _snprintf_s(label, sizeof(label), _TRUNCATE, "xrAcquireSwapchainImage(%s)", eyeNames[eye]);
                    ScopedTimer t(label, logThisIteration);
                    acquireResult = xrAcquireSwapchainImage(g_xrSwapchain[eye], &acquireInfo, &imageIndex);
                }
                if (XR_FAILED(acquireResult))
                    continue;
                XrSwapchainImageWaitInfo waitImgInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
                waitImgInfo.timeout = XR_INFINITE_DURATION;
                XrResult waitResult;
                {
                    _snprintf_s(label, sizeof(label), _TRUNCATE, "xrWaitSwapchainImage(%s)", eyeNames[eye]);
                    ScopedTimer t(label, logThisIteration);
                    waitResult = xrWaitSwapchainImage(g_xrSwapchain[eye], &waitImgInfo);
                }
                if (XR_SUCCEEDED(waitResult)) {
                    if (placeholderMode) {
                        // Diagnostic placeholder path (see
                        // kDiagnosticPlaceholderMode) - skip the real
                        // cross-API copy entirely and just clear the
                        // swapchain image to a solid color, so this frame's
                        // "workload" is as close to zero as a real
                        // layerCount=1 submission can be.
                        _snprintf_s(label, sizeof(label), _TRUNCATE, "ClearPlaceholder(%s)", eyeNames[eye]);
                        ScopedTimer t(label, logThisIteration);
                        ID3D11RenderTargetView* placeholderRtv = nullptr;
                        if (SUCCEEDED(g_d3d11Device->CreateRenderTargetView(
                                g_xrSwapchainImages[eye][imageIndex].texture, nullptr, &placeholderRtv))) {
                            const float kPlaceholderColor[4] = { 0.0f, 0.4f, 1.0f, 1.0f };
                            g_d3d11Context->ClearRenderTargetView(placeholderRtv, kPlaceholderColor);
                            placeholderRtv->Release();
                        }
                    } else {
                        _snprintf_s(label, sizeof(label), _TRUNCATE, "CopyResource(%s)", eyeNames[eye]);
                        ScopedTimer t(label, logThisIteration);
                        // CopySubresourceRegion (not CopyResource, which requires
                        // identical src/dst dimensions) - the swapchain image may
                        // now be allocated larger than our content texture (see
                        // CreateXrSessionAndSwapchains), so this places our full
                        // g_eyeWidth x g_eyeHeight content in the swapchain's
                        // top-left corner, matching the composition layer's
                        // imageRect below.
                        D3D11_BOX srcBox{ 0, 0, 0, g_eyeWidth, g_eyeHeight, 1 };
                        g_d3d11Context->CopySubresourceRegion(g_xrSwapchainImages[eye][imageIndex].texture, 0, 0, 0, 0,
                            srcTex[eye], 0, &srcBox);
                    }
                }
                XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
                {
                    _snprintf_s(label, sizeof(label), _TRUNCATE, "xrReleaseSwapchainImage(%s)", eyeNames[eye]);
                    ScopedTimer t(label, logThisIteration);
                    xrReleaseSwapchainImage(g_xrSwapchain[eye], &releaseInfo);
                }

                projViews[eye].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
                projViews[eye].pose = submitPose[eye];
                projViews[eye].fov = submitFov[eye];
                projViews[eye].subImage.swapchain = g_xrSwapchain[eye];
                projViews[eye].subImage.imageRect.offset = { 0, 0 };
                projViews[eye].subImage.imageRect.extent = { static_cast<int32_t>(g_eyeWidth), static_cast<int32_t>(g_eyeHeight) };
            }
        }

        XrCompositionLayerProjection projLayer{ XR_TYPE_COMPOSITION_LAYER_PROJECTION };
        projLayer.space = g_xrLocalSpace;
        projLayer.viewCount = viewCount;
        projLayer.views = projViews;
        const XrCompositionLayerBaseHeader* layers[1] = { reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projLayer) };

        XrFrameEndInfo endInfo{ XR_TYPE_FRAME_END_INFO };
        endInfo.displayTime = frameState.predictedDisplayTime;
        endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        endInfo.layerCount = (frameState.shouldRender && haveViews) ? 1 : 0;
        endInfo.layers = layers;
        {
            ScopedTimer t("xrEndFrame", logThisIteration);
            r = xrEndFrame(g_xrSession, &endInfo);
        }
        static UINT64 g_frameCount = 0;
        ++g_frameCount;
        if (g_frameCount % 600 == 0) {
            // Should be almost all hits. A high miss count means the tag
            // isn't reaching the submit thread and we are back to submitting
            // a pose the image was never rendered with.
            const unsigned long long ageCount = g_frameAgeCount;
            const double ageSum = g_frameAgeSumMs;
            const double ageWorst = g_frameAgeWorstMs;
            g_frameAgeCount = 0;
            g_frameAgeSumMs = 0.0;
            g_frameAgeWorstMs = 0.0;
            Log_Printf("XRBridge: submitted pose came from the rendered frame %llu time(s), fell back %llu time(s) "
                       "| image age at submit: avg %.1f ms, worst %.1f ms",
                g_poseHits.load(std::memory_order_relaxed), g_poseMisses.load(std::memory_order_relaxed),
                ageCount ? ageSum / static_cast<double>(ageCount) : 0.0, ageWorst);
        }
        if (XR_FAILED(r)) {
            Log_Printf("XRBridge: xrEndFrame failed (count=%llu) -> %s", g_frameCount, XrResultName(r));
        } else {
            if (!g_xrPoseLocked) {
                g_xrPoseLocked = true;
                Log_Printf("XRBridge: pose locked (first successful xrEndFrame, count=%llu)", g_frameCount);
            }
            ++g_framesSincePoseLock;
            if (g_framesSincePoseLock == kPoseSettleFrames) {
                Log_Printf("XRBridge: pose settled (%d frames since lock) - orientation reference capture now allowed", g_framesSincePoseLock);
            }
        }
        if (XR_SUCCEEDED(r) && g_frameCount % 300 == 0) {
            Log_Printf("XRBridge: frame heartbeat count=%llu", g_frameCount);
        }
}

// 2026-07-29: runs XrSubmitOneFrame in a continuous loop on its own
// thread, decoupled from the game's own EndScene cadence - see
// XrSubmitOneFrame's comment for why this replaced the synchronous
// call-from-EndScene design. xrWaitFrame's own block naturally paces this
// loop once the session is running; the only case that needs an explicit
// guard is the brief pre-session window, where every call would otherwise
// return near-instantly (PumpXrEvents with nothing to do yet) and spin a
// core at 100%.
DWORD WINAPI XrSubmitThreadProc(LPVOID)
{
    while (true) {
        XrSubmitOneFrame();
        if (!g_xrSessionRunning)
            Sleep(5);
    }
}

// 2026-07-29: starts the dedicated submit thread (see XrSubmitThreadProc)
// plus the one-time high-res timer request. THREAD_PRIORITY_HIGHEST to
// minimize this thread's own OS-scheduling jitter, since the whole point
// of decoupling it from the game's render thread is to give the
// runtime's frame-pacing signal a clean, low-jitter path to the
// compositor instead of inheriting the game's own scheduling variance.
void EnsureXrThreadStarted()
{
    if (g_xrThreadStarted)
        return;
    g_xrThreadStarted = true;
    MMRESULT tpResult = timeBeginPeriod(1);
    Log_Printf("XRBridge: timeBeginPeriod(1) -> %d", static_cast<int>(tpResult));
    g_xrThread = CreateThread(nullptr, 0, XrSubmitThreadProc, nullptr, 0, nullptr);
    if (g_xrThread) {
        SetThreadPriority(g_xrThread, THREAD_PRIORITY_HIGHEST);
        Log_Printf("XRBridge: submit thread started");
    } else {
        Log_Printf("XRBridge: CreateThread for submit thread FAILED (err=%lu)", GetLastError());
    }
}

// Paces the (cheap) main-thread copy so the game doesn't spend CPU/GPU
// time re-copying the backbuffer on every single EndScene call when it's
// running far faster than the XR submission rate (previously a real
// concern at >1000fps unthrottled) - mirrors the old OpenVR bridge's
// ShouldSubmitThisCall. Reinstated 2026-07-29 alongside the dedicated
// submit thread; was dead code during the brief synchronous-submission
// period since xrWaitFrame itself paced everything then.
bool ShouldCopyThisCall()
{
    static LARGE_INTEGER freq = {};
    static LARGE_INTEGER last = {};
    static bool freqInit = false;
    if (!freqInit) {
        QueryPerformanceFrequency(&freq);
        freqInit = true;
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    if (last.QuadPart == 0) {
        last = now;
        return true;
    }
    const double elapsedMs = static_cast<double>(now.QuadPart - last.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
    if (elapsedMs < g_targetFrameIntervalMs)
        return false;

    // 2026-09-12: the rate gate above is no longer the real limiter. It was
    // set to 400 Hz deliberately, to oversample against a free-running
    // submit thread so the two clocks couldn't beat and leave stale eye
    // textures - but the game thread and the submit thread share ONE
    // ID3D11DeviceContext, which is serialized between them. At 100+ fps the
    // game was taking that lock hundreds of times a second for frames the
    // headset would never display, starving the thread that actually feeds
    // the compositor. That is why calling up the SteamVR overlay made VR
    // smoother: it throttled the game and handed the context back.
    //
    // Now the producer waits until the submit thread has consumed what it
    // last published. Copies land at the headset's real rate, whatever that
    // is, with no magic number - and frames are no longer stale, because
    // each one carries the pose it was rendered with (see the pose ring).
    // The deadline is a safety valve: if VR stalls or the submit thread is
    // gone, keep refreshing rather than freezing the last image forever.
    constexpr double kUnconsumedDeadlineMs = 100.0;
    if (g_waitForConsumer.load(std::memory_order_relaxed)) {
        const bool consumed = g_eyeConsumedGeneration[kEyeLeft].load(std::memory_order_acquire) >=
            g_eyeGeneration[kEyeLeft].load(std::memory_order_acquire);
        if (!consumed && elapsedMs < kUnconsumedDeadlineMs)
            return false;
    }

    last = now;
    return true;
}

} // namespace

void VRBridge_Install()
{
    Log_Printf("XRBridge: ready (press F7 in-game to enable)");
}

void VRBridge_OnEndScene(IDirect3DDevice9* pGameDevice)
{
    // Page Up / Page Down: head-rotation gain (see g_headRotationGain).
    {
        static bool prevUp = false, prevDown = false;
        const bool up = (GetAsyncKeyState(VK_PRIOR) & 0x8000) != 0;
        const bool down = (GetAsyncKeyState(VK_NEXT) & 0x8000) != 0;
        if ((up && !prevUp) || (down && !prevDown)) {
            float g = g_headRotationGain.load(std::memory_order_relaxed) + (up && !prevUp ? kHeadGainStep : -kHeadGainStep);
            if (g < kHeadGainMin)
                g = kHeadGainMin;
            if (g > kHeadGainMax)
                g = kHeadGainMax;
            g_headRotationGain.store(g, std::memory_order_relaxed);
            Log_Printf("XRBridge: head rotation gain now %.1fx (1.0 = 1:1 with your neck)", g);
        }
        prevUp = up;
        prevDown = down;
    }

    // Home / End: head-rotation prediction in ms (see g_headPredictMs).
    {
        static bool prevHome = false, prevEnd = false;
        const bool home = (GetAsyncKeyState(VK_HOME) & 0x8000) != 0;
        const bool end = (GetAsyncKeyState(VK_END) & 0x8000) != 0;
        if ((home && !prevHome) || (end && !prevEnd)) {
            float ms = g_headPredictMs.load(std::memory_order_relaxed) +
                (home && !prevHome ? kHeadPredictStep : -kHeadPredictStep);
            if (ms < 0.0f)
                ms = 0.0f;
            if (ms > kHeadPredictMax)
                ms = kHeadPredictMax;
            g_headPredictMs.store(ms, std::memory_order_relaxed);
            Log_Printf("XRBridge: head rotation prediction now %.0f ms (0 = off; only acts while you are turning)", ms);
        }
        prevHome = home;
        prevEnd = end;
    }

    // Insert: producer waits for the consumer, or always keeps the newest
    // frame (see g_waitForConsumer). Watch the "image age at submit" line.
    {
        static bool prevInsert = false;
        const bool insert = (GetAsyncKeyState(VK_INSERT) & 0x8000) != 0;
        if (insert && !prevInsert) {
            const bool on = !g_waitForConsumer.load(std::memory_order_relaxed);
            g_waitForConsumer.store(on, std::memory_order_relaxed);
            Log_Printf("XRBridge: Insert pressed, producer %s",
                on ? "WAITS for the submit thread (fewer copies)" : "always refreshes (freshest image)");
        }
        prevInsert = insert;
    }

    static bool prevF7Down = false;
    bool f7Down = (GetAsyncKeyState(VK_F7) & 0x8000) != 0;
    if (f7Down && !prevF7Down) {
        g_xrModeEnabled = !g_xrModeEnabled;
        Log_Printf("XRBridge: F7 pressed, XR mode now %s", g_xrModeEnabled ? "ON" : "OFF");

        if (g_xrModeEnabled) {
            g_haveEyeViews = false;
            g_haveEyeReference[0] = false;
            g_haveEyeReference[1] = false;
            g_xrPoseLocked = false;
            g_framesSincePoseLock = 0;
            g_deltaLogCount[0] = 0;
            g_deltaLogCount[1] = 0;
            g_realContentFrameCount = 0;
            g_placeholderModeEverActive = false;
            if (!g_xrInitAttempted) {
                g_xrInitAttempted = true;
                g_xrInitialized = InitOpenXRInstanceAndSystem();
            }
            if (!g_xrInitialized) {
                Log_Printf("XRBridge: OpenXR instance/system init failed (no headset or no OpenXR "
                           "runtime?), turning XR mode back off");
                g_xrModeEnabled = false;
            } else {
                // Only split the screen once we know there is somewhere to
                // send the two halves. Turning it on before this point left
                // anyone without a headset staring at a side-by-side image
                // with no obvious way back - F7 again wouldn't undo it.
                StereoTest_SetEnabled(true);
            }
        } else {
            StereoTest_SetEnabled(false);
        }
    }
    prevF7Down = f7Down;

    if (!g_xrModeEnabled || !g_xrInitialized)
        return;

    if (!g_xrSessionReady) {
        IDirect3DSurface9* backbuffer = nullptr;
        UINT w = 1280, h = 720;
        if (SUCCEEDED(pGameDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &backbuffer)) && backbuffer) {
            D3DSURFACE_DESC desc = {};
            backbuffer->GetDesc(&desc);
            backbuffer->Release();
            w = desc.Width;
            h = desc.Height;
        }
        if (!CreateXrSessionAndSwapchains(w / 2, h, D3DFMT_A8R8G8B8)) {
            Log_Printf("XRBridge: session/swapchain setup failed, turning XR mode back off");
            g_xrModeEnabled = false;
            StereoTest_SetEnabled(false); // don't strand them in split-screen
            return;
        }
        g_xrSessionReady = true;
        EnsureXrThreadStarted();
    }

    if (!EnsureBridgeReady(pGameDevice))
        return;

    if (!ShouldCopyThisCall())
        return;

    // Always write into the slot that ISN'T currently front (i.e. not the
    // one the submit path might be mid-read of) - see g_eyeFrontIndex's
    // comment. Only publish the new front index (making this write visible
    // to the reader) after the write fully succeeds.
    const int leftBackSlot = 1 - g_eyeFrontIndex[kEyeLeft].load(std::memory_order_acquire);
    const int rightBackSlot = 1 - g_eyeFrontIndex[kEyeRight].load(std::memory_order_acquire);

    bool okLeft = false;
    bool okRight = false;

    if (g_usingD3D12AddonPath) {
        // No D3D9 backbuffer/CPU readback at all in this path - straight
        // GPU-side CopySubresourceRegion from the addon's shared D3D12
        // frame (already opened as a D3D11 texture) into both eye slots.
        okLeft = okRight = D3D12AddonBridge_CopyToEyeSlots(g_d3d11Context,
            g_d3d11LeftTex[leftBackSlot], g_d3d11RightTex[rightBackSlot], g_eyeWidth, g_eyeHeight);
    } else {
        IDirect3DSurface9* backbuffer = nullptr;
        if (FAILED(pGameDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &backbuffer)) || !backbuffer)
            return;

        RECT leftRect = { 0, 0, static_cast<LONG>(g_eyeWidth), static_cast<LONG>(g_eyeHeight) };
        RECT rightRect = { static_cast<LONG>(g_eyeWidth), 0, static_cast<LONG>(g_eyeWidth) * 2, static_cast<LONG>(g_eyeHeight) };

        okLeft = CopyHalfToEye(pGameDevice, backbuffer, leftRect, g_gameLeftCrop, g_gameLeftSysMem, g_bridgeLeftSysMem, g_bridgeLeftSurf[leftBackSlot], "left");
        okRight = CopyHalfToEye(pGameDevice, backbuffer, rightRect, g_gameRightCrop, g_gameRightSysMem, g_bridgeRightSysMem, g_bridgeRightSurf[rightBackSlot], "right");

        backbuffer->Release();
    }

    if (okLeft) {
        // Tag the slot with the pose the game actually rendered these
        // pixels with, before publishing it - the submit thread reads the
        // tag only after seeing the new front index, so it can never pick
        // up a slot whose pose hasn't been written yet.
        g_slotPoseId[leftBackSlot].store(g_poseOfPresentedFrame.load(std::memory_order_acquire),
            std::memory_order_release);
        g_eyeFrontIndex[kEyeLeft].store(leftBackSlot, std::memory_order_release);
        g_eyeGeneration[kEyeLeft].fetch_add(1, std::memory_order_release);
    }
    if (okRight) {
        g_eyeFrontIndex[kEyeRight].store(rightBackSlot, std::memory_order_release);
        g_eyeGeneration[kEyeRight].fetch_add(1, std::memory_order_release);
    }

    static bool loggedFirstCopy = false;
    if (!loggedFirstCopy && okLeft && okRight) {
        loggedFirstCopy = true;
        Log_Printf("XRBridge: first successful eye copy (submit thread handles headset delivery)");
    }
    static UINT64 g_copyCount = 0;
    ++g_copyCount;
    if (!okLeft || !okRight) {
        Log_Printf("XRBridge: eye copy FAILED (count=%llu) okLeft=%d okRight=%d", g_copyCount, okLeft, okRight);
    } else if (g_copyCount % 300 == 0) {
        Log_Printf("XRBridge: copy heartbeat count=%llu", g_copyCount);
    }
}

XRBridgePoseId VRBridge_GetCurrentPoseId()
{
    return g_currentPoseId.load(std::memory_order_acquire);
}

void VRBridge_NoteFramePresented(XRBridgePoseId poseId)
{
    g_poseOfPresentedFrame.store(poseId, std::memory_order_release);
}

bool VRBridge_GetEyeViews(XRBridgeEyeView& outLeft, XRBridgeEyeView& outRight)
{
    AcquireSRWLockShared(&g_eyeViewsLock);
    const bool have = g_haveEyeViews;
    if (have) {
        outLeft = g_eyeViews[kEyeLeft];
        outRight = g_eyeViews[kEyeRight];
    }
    ReleaseSRWLockShared(&g_eyeViewsLock);
    return have;
}
