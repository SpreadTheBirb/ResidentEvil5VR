#include "openvr_bridge.h"
#include "../render/stereo_test.h"
#include "../render/mat3.h"
#include "../proxy/real_d3d9.h"
#include "../util/log.h"

#include <openvr.h>
#include <d3d11.h>
#include <windows.h>
#include <timeapi.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

// ---- OpenVR session state -----------------------------------------------

bool g_vrModeEnabled = false;
bool g_vrInitAttempted = false;
bool g_vrInitialized = false;
vr::IVRSystem* g_vrSystem = nullptr;
vr::IVRCompositor* g_vrCompositor = nullptr;

// ---- Head tracking (Phase 3) ---------------------------------------------
//
// Extracted from the HMD pose returned by WaitGetPoses (previously
// discarded). Stored the same way stereo_test.cpp treats RE5's own camera
// matrix - a 3x3 whose rows are (Right, Up, Forward) unit vectors, i.e. a
// "world-to-local" rotation matrix. g_hmdReference is captured once each
// time VR mode turns on (a recenter); g_hmdDeltaLocal is the HMD's current
// orientation expressed as a pure local rotation relative to that
// reference (frame-independent, so it can be composed directly onto RE5's
// own camera basis regardless of how OpenVR's room space happens to be
// oriented relative to the game world - see stereo_test.cpp).
bool g_haveHmdReference = false;
Mat3 g_hmdReference = Mat3Identity();
bool g_haveHmdDelta = false;
Mat3 g_hmdDeltaLocal = Mat3Identity();

// ---- Real per-eye FOV scale + toe-in rotation (Phase 4 follow-up:
// replaces the untuned flat widen factor, and the never-implemented
// per-eye orientation offset) ---------------------------------------------
//
// Both queried once right after a successful VR_Init, since
// GetProjectionRaw/GetEyeToHeadTransform describe the headset's fixed
// lens/mount geometry, not per-frame values. Indexed [0]=left, [1]=right
// throughout - queried and stored separately per eye after an earlier
// version that reused the left eye's FOV shape for both eyes produced a
// visibly skewed "cock-eyed" right-eye image (2026-07-27 user report) -
// don't go back to sharing one eye's data across both without new
// evidence it's actually safe on this headset.
bool g_haveRealEyeGeometry = false;
float g_realFovScaleX[2] = { 0.0f, 0.0f };
float g_realFovScaleY[2] = { 0.0f, 0.0f };
Mat3 g_eyeToHeadRotation[2] = { Mat3Identity(), Mat3Identity() };

void QueryRealEyeGeometry()
{
    if (!g_vrSystem)
        return;

    const vr::EVREye eyes[2] = { vr::Eye_Left, vr::Eye_Right };
    const char* eyeNames[2] = { "left", "right" };
    bool anyFovOk = false;

    for (int e = 0; e < 2; ++e) {
        // These are tangents of the half-angles from the eye's forward
        // axis to each frustum edge (left/top conventionally negative,
        // right/bottom positive) - total FOV along an axis is the angular
        // span between the two edges, not simply double one of them,
        // since consumer HMDs are frequently NOT symmetric about center
        // (canted lenses). We still only have one shared symmetric scale
        // to give this eye's half of the scissor-split render (see header
        // comment), so the best available fit is a symmetric FOV of the
        // same total angular span.
        float tanLeft = 0.0f, tanRight = 0.0f, tanTop = 0.0f, tanBottom = 0.0f;
        g_vrSystem->GetProjectionRaw(eyes[e], &tanLeft, &tanRight, &tanTop, &tanBottom);
        const float totalFovX = std::fabs(std::atan(tanRight) - std::atan(tanLeft));
        const float totalFovY = std::fabs(std::atan(tanBottom) - std::atan(tanTop));
        if (totalFovX > 0.01f && totalFovY > 0.01f) {
            g_realFovScaleX[e] = 1.0f / std::tan(totalFovX * 0.5f);
            g_realFovScaleY[e] = 1.0f / std::tan(totalFovY * 0.5f);
            anyFovOk = true;
            Log_Printf("VRBridge: real FOV scale (%s eye): scaleX=%.4f scaleY=%.4f (totalFovX=%.1fdeg totalFovY=%.1fdeg)",
                eyeNames[e], g_realFovScaleX[e], g_realFovScaleY[e],
                totalFovX * (180.0f / 3.14159265f), totalFovY * (180.0f / 3.14159265f));
        } else {
            Log_Printf("VRBridge: QueryRealEyeGeometry: degenerate GetProjectionRaw for %s eye (L=%.4f R=%.4f T=%.4f B=%.4f)",
                eyeNames[e], tanLeft, tanRight, tanTop, tanBottom);
        }

        // Rotation part of head->eye transform, decoded the same way
        // VRSubmitThreadProc decodes the HMD's own tracking pose (columns
        // 0/1/2 of the row-major m[3][4] are the local Right/Up/-Forward
        // axes, expressed here in HEAD space rather than tracking space) -
        // this is exactly the per-eye toe-in delta to compose onto a
        // head-tracked CameraBasis, the same way ApplyHeadRotation
        // composes the head's own delta.
        const vr::HmdMatrix34_t eyeToHead = g_vrSystem->GetEyeToHeadTransform(eyes[e]);
        const float(&hm)[3][4] = eyeToHead.m;
        Mat3 rot{};
        rot.m[0] = hm[0][0]; rot.m[1] = hm[1][0]; rot.m[2] = hm[2][0]; // Right
        rot.m[3] = hm[0][1]; rot.m[4] = hm[1][1]; rot.m[5] = hm[2][1]; // Up
        rot.m[6] = -hm[0][2]; rot.m[7] = -hm[1][2]; rot.m[8] = -hm[2][2]; // Forward
        g_eyeToHeadRotation[e] = rot;
        // Full matrix logged (not just the offset) purely as a diagnostic -
        // applying this rotation to a live render was tried and reverted
        // 2026-07-27 after it badly broke the image, which shouldn't
        // happen for genuine lens toe-in (normally a few degrees at most,
        // i.e. this matrix should look close to identity). Read these
        // values back before ever re-attempting to apply it.
        Log_Printf("VRBridge: eye-to-head offset (%s): (%.4f, %.4f, %.4f) m", eyeNames[e], hm[0][3], hm[1][3], hm[2][3]);
        Log_Printf("VRBridge: eye-to-head rotation (%s): right=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f) forward=(%.4f,%.4f,%.4f)",
            eyeNames[e], rot.m[0], rot.m[1], rot.m[2], rot.m[3], rot.m[4], rot.m[5], rot.m[6], rot.m[7], rot.m[8]);
    }

    g_haveRealEyeGeometry = anyFovOk;
}

// RE5 fires EndScene anywhere from ~500 to 2000+ times per second
// (uncapped framerate on modern hardware). Each VR submission does two
// GetRenderTargetData calls - a notoriously expensive CPU<->GPU
// synchronization point that stalls the pipeline - plus WaitGetPoses,
// which is itself designed to block until it's time for the next HMD
// frame. Running that on every EndScene call (rather than once per real
// displayed frame) caused a severe framerate regression reported on
// hardware far more powerful than this 2009 game needs, confirming it's a
// synchronization/call-frequency bug, not a rendering cost problem. Gate
// the whole submission path to roughly the headset's own cadence instead.
//
// Was hardcoded to a flat 90Hz assumption everywhere in this file - a
// real, provably wrong assumption for this specific headset, discovered
// 2026-07-28 when the user's SteamVR settings screenshot showed
// "Refresh Rate: 72 Hz" while running over the Link cable (may differ
// again over wireless - the whole point of querying it live instead of
// hardcoding either number). g_targetFrameIntervalMs replaces the old
// constexpr kTargetSubmitIntervalMs/kVRFrameCapIntervalMs/
// kSubmitThreadIntervalMs (all three were the same hardcoded value,
// unified into one variable here) - starts at the old 90Hz-based value as
// a fallback, overwritten with the real queried refresh rate by
// QueryRealDisplayFrequency() once OpenVR is initialized.
double g_targetFrameIntervalMs = 1000.0 / 90.0; // ~11.1ms (90Hz fallback)

void QueryRealDisplayFrequency()
{
    if (!g_vrSystem)
        return;
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    const float hz = g_vrSystem->GetFloatTrackedDeviceProperty(
        vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float, &err);
    if (err != vr::TrackedProp_Success || hz < 1.0f || hz > 1000.0f) {
        Log_Printf("VRBridge: QueryRealDisplayFrequency: failed (err=%d hz=%.2f), keeping 90Hz fallback",
            static_cast<int>(err), hz);
        return;
    }
    g_targetFrameIntervalMs = 1000.0 / static_cast<double>(hz);
    Log_Printf("VRBridge: real display frequency from HMD: %.2f Hz (target interval %.3f ms)", hz, g_targetFrameIntervalMs);
}
LARGE_INTEGER g_perfFrequency = {};
LARGE_INTEGER g_lastSubmitTime = {};
bool g_perfFrequencyInit = false;

bool ShouldSubmitThisCall()
{
    if (!g_perfFrequencyInit) {
        QueryPerformanceFrequency(&g_perfFrequency);
        g_perfFrequencyInit = true;
    }

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    if (g_lastSubmitTime.QuadPart == 0) {
        g_lastSubmitTime = now;
        return true;
    }

    const double elapsedMs = static_cast<double>(now.QuadPart - g_lastSubmitTime.QuadPart) * 1000.0
        / static_cast<double>(g_perfFrequency.QuadPart);
    if (elapsedMs < g_targetFrameIntervalMs)
        return false;

    g_lastSubmitTime = now;
    return true;
}

// ---- Frame rate cap while VR mode is active -----------------------------
//
// RE5 renders completely uncapped (2000-3000+ fps) regardless of VR mode -
// it has no idea anything is different, so the GPU keeps presenting
// thousands of frames per second that nobody watching the monitor can
// even see, WHILE our own bridge thread is trying to get its own texture
// copy + compositor handoff through on the same GPU/driver queue. User
// confirmed the wireless link itself is fine (another game runs smoothly
// over the same Virtual Desktop connection), and a much earlier 30fps-cap
// test predates the fix that moved WaitGetPoses/Submit off the main
// thread, so it doesn't actually rule this out. Capping the game's own
// Present rate here (only while VR mode is on, applied directly rather
// than relying on the game's own limiter option) is a controlled way to
// test whether GPU contention from that uncapped monitor-only rendering
// is why the headset stutters despite our own pipeline reporting healthy
// numbers throughout.
LARGE_INTEGER g_frameCapFreq = {};
LARGE_INTEGER g_lastFrameCapTime = {};
bool g_frameCapFreqInit = false;

void CapFrameRateWhileVRActive()
{
    if (!g_frameCapFreqInit) {
        QueryPerformanceFrequency(&g_frameCapFreq);
        g_frameCapFreqInit = true;
    }

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    if (g_lastFrameCapTime.QuadPart != 0) {
        double elapsedMs = static_cast<double>(now.QuadPart - g_lastFrameCapTime.QuadPart) * 1000.0
            / static_cast<double>(g_frameCapFreq.QuadPart);
        while (elapsedMs < g_targetFrameIntervalMs) {
            Sleep(1);
            QueryPerformanceCounter(&now);
            elapsedMs = static_cast<double>(now.QuadPart - g_lastFrameCapTime.QuadPart) * 1000.0
                / static_cast<double>(g_frameCapFreq.QuadPart);
        }
    }

    g_lastFrameCapTime = now;
}

// Diagnostic timing: logs how long each named step took, but only for the
// first several submissions (to find where a stall actually is without
// spamming the log forever at high frequency). Shared across the main
// thread (copy steps) and the VR submit thread (WaitGetPoses/Submit) - a
// benign race on this counter at worst logs a couple extra/fewer lines.
int g_timingLogsRemaining = 20;

struct ScopedTimer {
    const char* name;
    LARGE_INTEGER start;
    LARGE_INTEGER freq;
    bool active;

    explicit ScopedTimer(const char* n) : name(n), active(g_timingLogsRemaining > 0)
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
        Log_Printf("VRBridge: TIMING %s: %.2f ms", name, ms);
    }
};

bool InitOpenVR()
{
    vr::EVRInitError eError = vr::VRInitError_None;
    g_vrSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);
    if (eError != vr::VRInitError_None) {
        Log_Printf("VRBridge: VR_Init failed: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        g_vrSystem = nullptr;
        return false;
    }

    g_vrCompositor = vr::VRCompositor();
    if (!g_vrCompositor) {
        Log_Printf("VRBridge: VRCompositor() returned null");
        vr::VR_Shutdown();
        g_vrSystem = nullptr;
        return false;
    }

    Log_Printf("VRBridge: OpenVR initialized successfully");
    QueryRealEyeGeometry();
    QueryRealDisplayFrequency();
    return true;
}

// ---- Bridge device (separate, hidden, Ex - see header comment) ---------

bool g_bridgeReady = false;
UINT g_eyeWidth = 0;
UINT g_eyeHeight = 0;
D3DFORMAT g_backbufferFormat = D3DFMT_UNKNOWN;

HWND g_bridgeWindow = nullptr;
IDirect3D9Ex* g_bridgeD3D9 = nullptr;
IDirect3DDevice9Ex* g_bridgeDevice = nullptr;

IDirect3DTexture9* g_bridgeLeftTex = nullptr;
IDirect3DTexture9* g_bridgeRightTex = nullptr;
IDirect3DSurface9* g_bridgeLeftSurf = nullptr;
IDirect3DSurface9* g_bridgeRightSurf = nullptr;
HANDLE g_leftSharedHandle = nullptr;
HANDLE g_rightSharedHandle = nullptr;
IDirect3DSurface9* g_bridgeLeftSysMem = nullptr;
IDirect3DSurface9* g_bridgeRightSysMem = nullptr;

IDirect3DSurface9* g_gameLeftCrop = nullptr;
IDirect3DSurface9* g_gameRightCrop = nullptr;
IDirect3DSurface9* g_gameLeftSysMem = nullptr;
IDirect3DSurface9* g_gameRightSysMem = nullptr;

ID3D11Device* g_d3d11Device = nullptr;
ID3D11DeviceContext* g_d3d11Context = nullptr;
ID3D11Texture2D* g_d3d11LeftTex = nullptr;
ID3D11Texture2D* g_d3d11RightTex = nullptr;

// ---- VR submit thread ----------------------------------------------------
//
// WaitGetPoses measured 10-34ms per call in practice (likely inflated by
// the wireless Link/Air Link/Virtual Desktop round-trip), and Submit can
// spike too (127ms on the very first call). Both were being called from
// VRBridge_OnEndScene, i.e. the game's own main render thread inside its
// EndScene hook - so every submission stalled the entire game (render,
// physics, logic, everything) for that duration. That's what turned into
// the reported "slideshow": the game's own frame counter dropped from
// ~3000fps to ~51fps the moment VR mode turned on, exactly matching the
// measured per-submission cost. Fix: run WaitGetPoses+Submit on their own
// thread, paced independently by WaitGetPoses itself, and never touch the
// game's thread for them again. The main thread still does the (cheap,
// <10ms measured) backbuffer->bridge-texture copy; the submit thread just
// keeps resubmitting whatever's currently in the shared textures - OpenVR
// permits WaitGetPoses/Submit from any thread as long as only one thread
// ever calls WaitGetPoses.
HANDLE g_vrSubmitThread = nullptr;
bool g_vrSubmitThreadStarted = false;

// A "Fatal Application Exit: ERR09: Unsupported function" from the game
// itself showed up ~7s into otherwise-clean steady-state VR operation once
// this thread had no pacing of its own (confirmed via the Windows
// Application-Popup event log entry, not a crash dump - the game calls its
// own fatal-exit path deliberately). This is the exact same message RE5
// showed back when upgrading the game's OWN device to CreateDeviceEx broke
// PresentEx/ResetEx - i.e. it's the game's generic "something in my D3D9
// layer just returned a fatal HRESULT" handler, not something specific to
// that old approach. WaitGetPoses is *supposed* to block for roughly one
// HMD frame interval, but isn't guaranteed to - if it ever returns early
// (compositor catching up, wireless-link hiccup), this loop would hammer
// Submit() far faster than the compositor expects, which can exhaust
// shared-resource/driver state on the adapter - and since the game's own
// device shares that same GPU/driver, a resulting fault can surface there
// too even though we never touch that device directly. Explicit rate cap
// added below so this thread never exceeds ~90Hz regardless of how fast
// WaitGetPoses actually returns.
LARGE_INTEGER g_submitThreadFreq = {};
LARGE_INTEGER g_lastSubmitThreadTime = {};
bool g_submitThreadFreqInit = false;

DWORD WINAPI VRSubmitThreadProc(LPVOID)
{
    while (true) {
        if (!g_vrModeEnabled || !g_vrCompositor || !g_d3d11LeftTex || !g_d3d11RightTex) {
            Sleep(10);
            continue;
        }

        if (!g_submitThreadFreqInit) {
            QueryPerformanceFrequency(&g_submitThreadFreq);
            g_submitThreadFreqInit = true;
        }
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        if (g_lastSubmitThreadTime.QuadPart != 0) {
            const double elapsedMs = static_cast<double>(now.QuadPart - g_lastSubmitThreadTime.QuadPart) * 1000.0
                / static_cast<double>(g_submitThreadFreq.QuadPart);
            if (elapsedMs < g_targetFrameIntervalMs) {
                Sleep(1);
                continue;
            }
        }
        g_lastSubmitThreadTime = now;

        static vr::TrackedDevicePose_t renderPoses[vr::k_unMaxTrackedDeviceCount];
        {
            ScopedTimer t("WaitGetPoses");
            g_vrCompositor->WaitGetPoses(renderPoses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
        }

        const vr::TrackedDevicePose_t& hmdPose = renderPoses[vr::k_unTrackedDeviceIndex_Hmd];
        if (hmdPose.bPoseIsValid) {
            // OpenVR's HmdMatrix34_t is row-major m[3][4]; columns 0/1/2 of
            // the rotation part are the device's local Right/Up/-Forward
            // axes expressed in tracking space (standard convention - the
            // device looks down its own -Z). Extracted here as ROWS (same
            // layout stereo_test.cpp uses for RE5's own camera basis) so
            // both matrices compose the same way.
            const float(&hm)[3][4] = hmdPose.mDeviceToAbsoluteTracking.m;
            Mat3 hmdNow{};
            hmdNow.m[0] = hm[0][0]; hmdNow.m[1] = hm[1][0]; hmdNow.m[2] = hm[2][0]; // Right
            hmdNow.m[3] = hm[0][1]; hmdNow.m[4] = hm[1][1]; hmdNow.m[5] = hm[2][1]; // Up
            hmdNow.m[6] = -hm[0][2]; hmdNow.m[7] = -hm[1][2]; hmdNow.m[8] = -hm[2][2]; // Forward

            if (!g_haveHmdReference) {
                g_hmdReference = hmdNow;
                g_haveHmdReference = true;
            }

            // Local (frame-independent) delta: how much the head has
            // rotated relative to its reference orientation, expressed in
            // the reference's own local coordinates - see stereo_test.cpp
            // for why this is what can be composed onto RE5's own camera
            // basis instead of OpenVR's room-space axes directly.
            g_hmdDeltaLocal = Mat3Multiply(hmdNow, Mat3Transpose(g_hmdReference));
            g_haveHmdDelta = true;
        }

        vr::EVRCompositorError errLeft, errRight;
        {
            ScopedTimer t("Submit(left)");
            vr::Texture_t tex = { g_d3d11LeftTex, vr::TextureType_DirectX, vr::ColorSpace_Auto };
            errLeft = g_vrCompositor->Submit(vr::Eye_Left, &tex);
        }
        {
            ScopedTimer t("Submit(right)");
            vr::Texture_t tex = { g_d3d11RightTex, vr::TextureType_DirectX, vr::ColorSpace_Auto };
            errRight = g_vrCompositor->Submit(vr::Eye_Right, &tex);
        }

        // Submit()'s return was never checked before - if it's silently
        // failing on most calls, the game and the rest of our pipeline
        // would look perfectly healthy in the log while the headset barely
        // updates, which is exactly the symptom reported ("great on the
        // monitor, still a slideshow in the headset"). Log every failure,
        // plus a low-noise heartbeat periodically so a whole play session
        // is visible, not just the first 20 submissions.
        static UINT64 g_submitCount = 0;
        ++g_submitCount;
        if (errLeft != vr::VRCompositorError_None || errRight != vr::VRCompositorError_None) {
            Log_Printf("VRBridge: Submit FAILED (count=%llu) left=%d right=%d",
                g_submitCount, static_cast<int>(errLeft), static_cast<int>(errRight));
        } else if (g_submitCount % 300 == 0) {
            Log_Printf("VRBridge: submit heartbeat count=%llu", g_submitCount);
        }

        if (g_timingLogsRemaining > 0)
            --g_timingLogsRemaining;
    }
    return 0;
}

void EnsureVRSubmitThreadStarted()
{
    if (g_vrSubmitThreadStarted)
        return;
    g_vrSubmitThreadStarted = true;

    // Windows' default timer resolution is ~15.6ms, not 1ms - a Sleep(1)
    // call (used below to pace this thread to ~90Hz) can actually sleep
    // for up to ~15ms without this, which lines up suspiciously well with
    // the measured ~15-20ms WaitGetPoses/loop cadence (should be ~11.1ms
    // for a real 90Hz app) behind the reported in-headset stutter. This is
    // standard practice for any latency-sensitive Windows app (games,
    // audio, VR) - raises the whole process's timer resolution so Sleep()
    // calls actually approximate what's asked for.
    MMRESULT tpResult = timeBeginPeriod(1);
    Log_Printf("VRBridge: timeBeginPeriod(1) -> %d", static_cast<int>(tpResult));

    g_vrSubmitThread = CreateThread(nullptr, 0, &VRSubmitThreadProc, nullptr, 0, nullptr);
    Log_Printf("VRBridge: submit thread started -> %p", g_vrSubmitThread);

    // The measured WaitGetPoses variability (9-34ms, not cleanly locked to
    // any single rate) fits CPU scheduling contention better than
    // compositor pacing: RE5's main thread runs completely uncapped
    // (2000-3000+ fps), and even though SteamVR signals this thread ready
    // on time internally, Windows may not actually get around to
    // scheduling it promptly while the game's thread is spinning that
    // hard. Standard fix for a latency-sensitive thread like this is to
    // raise its priority directly, rather than trying to indirectly starve
    // the competition (which is likely why capping the game's own frame
    // rate made things worse, not better, in an earlier test).
    if (g_vrSubmitThread) {
        BOOL prioOk = SetThreadPriority(g_vrSubmitThread, THREAD_PRIORITY_TIME_CRITICAL);
        Log_Printf("VRBridge: SetThreadPriority(TIME_CRITICAL) -> %d", prioOk);
    }
}

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

    // Never shown (no WS_VISIBLE, no ShowWindow call) - purely a focus
    // window D3D9Ex requires, not meant to be seen.
    return CreateWindowExA(0, kClassName, "RE5VR Bridge", WS_POPUP,
        0, 0, 4, 4, nullptr, nullptr, GetModuleHandleA(nullptr), nullptr);
}

bool CreateBridgeDevice(HWND hwnd)
{
    if (!g_RealDirect3DCreate9Ex) {
        Log_Printf("VRBridge: CreateBridgeDevice: real Direct3DCreate9Ex not resolved yet");
        return false;
    }

    HRESULT hr = g_RealDirect3DCreate9Ex(D3D_SDK_VERSION, &g_bridgeD3D9);
    if (FAILED(hr) || !g_bridgeD3D9) {
        Log_Printf("VRBridge: CreateBridgeDevice: Direct3DCreate9Ex failed (hr=0x%08lX)", hr);
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
        Log_Printf("VRBridge: CreateBridgeDevice: CreateDeviceEx failed (hr=0x%08lX)", hr);
        return false;
    }

    return true;
}

bool CreateSharedEyeTexture(IDirect3DTexture9** outTex, IDirect3DSurface9** outSurf, HANDLE* outHandle)
{
    *outHandle = nullptr;
    HRESULT hr = g_bridgeDevice->CreateTexture(g_eyeWidth, g_eyeHeight, 1, D3DUSAGE_RENDERTARGET,
        g_backbufferFormat, D3DPOOL_DEFAULT, outTex, outHandle);
    if (FAILED(hr) || !*outTex) {
        Log_Printf("VRBridge: CreateSharedEyeTexture failed (hr=0x%08lX)", hr);
        return false;
    }
    (*outTex)->GetSurfaceLevel(0, outSurf);
    return true;
}

bool EnsureBridgeReady(IDirect3DDevice9* pGameDevice)
{
    if (g_bridgeReady)
        return true;

    IDirect3DSurface9* backbuffer = nullptr;
    if (FAILED(pGameDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &backbuffer)) || !backbuffer) {
        Log_Printf("VRBridge: EnsureBridgeReady: GetBackBuffer failed");
        return false;
    }
    D3DSURFACE_DESC desc = {};
    backbuffer->GetDesc(&desc);
    backbuffer->Release();

    g_backbufferFormat = desc.Format;
    g_eyeWidth = desc.Width / 2;
    g_eyeHeight = desc.Height;

    g_bridgeWindow = CreateHiddenWindow();
    if (!g_bridgeWindow) {
        Log_Printf("VRBridge: hidden window creation failed");
        return false;
    }

    if (!CreateBridgeDevice(g_bridgeWindow))
        return false;

    if (!CreateSharedEyeTexture(&g_bridgeLeftTex, &g_bridgeLeftSurf, &g_leftSharedHandle))
        return false;
    if (!CreateSharedEyeTexture(&g_bridgeRightTex, &g_bridgeRightSurf, &g_rightSharedHandle))
        return false;

    HRESULT hr = g_bridgeDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_bridgeLeftSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: bridge left sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }
    hr = g_bridgeDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_bridgeRightSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: bridge right sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }

    hr = pGameDevice->CreateRenderTarget(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DMULTISAMPLE_NONE, 0, FALSE, &g_gameLeftCrop, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: game left crop render target failed (hr=0x%08lX)", hr);
        return false;
    }
    hr = pGameDevice->CreateRenderTarget(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DMULTISAMPLE_NONE, 0, FALSE, &g_gameRightCrop, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: game right crop render target failed (hr=0x%08lX)", hr);
        return false;
    }

    hr = pGameDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_gameLeftSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: game left sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }
    hr = pGameDevice->CreateOffscreenPlainSurface(g_eyeWidth, g_eyeHeight, g_backbufferFormat,
        D3DPOOL_SYSTEMMEM, &g_gameRightSysMem, nullptr);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: game right sysmem surface failed (hr=0x%08lX)", hr);
        return false;
    }

    D3D_FEATURE_LEVEL achievedLevel;
    hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
        D3D11_SDK_VERSION, &g_d3d11Device, &achievedLevel, &g_d3d11Context);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: D3D11CreateDevice failed (hr=0x%08lX)", hr);
        return false;
    }

    hr = g_d3d11Device->OpenSharedResource(g_leftSharedHandle, __uuidof(ID3D11Texture2D),
        reinterpret_cast<void**>(&g_d3d11LeftTex));
    if (FAILED(hr)) {
        Log_Printf("VRBridge: OpenSharedResource (left) failed (hr=0x%08lX)", hr);
        return false;
    }
    hr = g_d3d11Device->OpenSharedResource(g_rightSharedHandle, __uuidof(ID3D11Texture2D),
        reinterpret_cast<void**>(&g_d3d11RightTex));
    if (FAILED(hr)) {
        Log_Printf("VRBridge: OpenSharedResource (right) failed (hr=0x%08lX)", hr);
        return false;
    }

    Log_Printf("VRBridge: bridge ready (eye %ux%u, format=%d)", g_eyeWidth, g_eyeHeight, g_backbufferFormat);
    g_bridgeReady = true;
    EnsureVRSubmitThreadStarted();
    return true;
}

// Copies one half of the game's finished backbuffer across to the
// matching bridge-device shared texture. Three hops, since a plain (non-Ex)
// device can't produce shared handles and DX9 has no cross-device GPU
// copy: (1) StretchRect the half-region into an eye-sized render target on
// the GAME device (GetRenderTargetData can't take a source rect, only
// whole-surface copies, hence needing this crop step first); (2)
// GetRenderTargetData that crop into a GAME-device system-memory surface
// (GPU -> CPU-visible memory); (3) a raw CPU memcpy into a BRIDGE-device
// system-memory surface (UpdateSurface requires source and dest to belong
// to the same device, so the two system-memory surfaces can't be the same
// object); (4) UpdateSurface into the bridge device's shared GPU texture.
bool CopyHalfToEye(IDirect3DDevice9* pGameDevice, IDirect3DSurface9* gameBackbuffer, const RECT& srcRect,
    IDirect3DSurface9* gameCrop, IDirect3DSurface9* gameSysMem, IDirect3DSurface9* bridgeSysMem, IDirect3DSurface9* bridgeTarget,
    const char* eyeName)
{
    char label[64];

    HRESULT hr;
    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "StretchRect(%s)", eyeName);
        ScopedTimer t(label);
        hr = pGameDevice->StretchRect(gameBackbuffer, &srcRect, gameCrop, nullptr, D3DTEXF_NONE);
    }
    if (FAILED(hr)) {
        Log_Printf("VRBridge: CopyHalfToEye: crop StretchRect failed (hr=0x%08lX)", hr);
        return false;
    }

    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "GetRenderTargetData(%s)", eyeName);
        ScopedTimer t(label);
        hr = pGameDevice->GetRenderTargetData(gameCrop, gameSysMem);
    }
    if (FAILED(hr)) {
        Log_Printf("VRBridge: CopyHalfToEye: GetRenderTargetData failed (hr=0x%08lX)", hr);
        return false;
    }

    D3DLOCKED_RECT srcLocked = {};
    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "LockRect-src(%s)", eyeName);
        ScopedTimer t(label);
        hr = gameSysMem->LockRect(&srcLocked, nullptr, D3DLOCK_READONLY);
    }
    if (FAILED(hr)) {
        Log_Printf("VRBridge: CopyHalfToEye: LockRect(src) failed (hr=0x%08lX)", hr);
        return false;
    }

    D3DLOCKED_RECT dstLocked = {};
    hr = bridgeSysMem->LockRect(&dstLocked, nullptr, 0);
    if (FAILED(hr)) {
        Log_Printf("VRBridge: CopyHalfToEye: LockRect(dst) failed (hr=0x%08lX)", hr);
        gameSysMem->UnlockRect();
        return false;
    }

    {
        _snprintf_s(label, sizeof(label), _TRUNCATE, "memcpy(%s)", eyeName);
        ScopedTimer t(label);
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
        ScopedTimer t(label);
        hr = g_bridgeDevice->UpdateSurface(bridgeSysMem, nullptr, bridgeTarget, nullptr);
    }
    if (FAILED(hr)) {
        Log_Printf("VRBridge: CopyHalfToEye: UpdateSurface failed (hr=0x%08lX)", hr);
        return false;
    }

    return true;
}

} // namespace

void VRBridge_Install()
{
    Log_Printf("VRBridge: ready (press F7 in-game to enable)");
}

void VRBridge_OnEndScene(IDirect3DDevice9* pGameDevice)
{
    static bool prevF7Down = false;
    bool f7Down = (GetAsyncKeyState(VK_F7) & 0x8000) != 0;
    if (f7Down && !prevF7Down) {
        g_vrModeEnabled = !g_vrModeEnabled;
        Log_Printf("VRBridge: F7 pressed, VR mode now %s", g_vrModeEnabled ? "ON" : "OFF");

        if (g_vrModeEnabled) {
            StereoTest_SetEnabled(true);
            // Recenter head tracking on every enable, not just the first
            // one - otherwise turning VR off and back on would keep using
            // a stale reference orientation from whenever it was first
            // turned on this session.
            g_haveHmdReference = false;
            g_haveHmdDelta = false;
            if (!g_vrInitAttempted) {
                g_vrInitAttempted = true;
                g_vrInitialized = InitOpenVR();
            }
            if (!g_vrInitialized) {
                Log_Printf("VRBridge: OpenVR not initialized, turning VR mode back off");
                g_vrModeEnabled = false;
            }
        }
    }
    prevF7Down = f7Down;

    if (!g_vrModeEnabled || !g_vrInitialized)
        return;

    if (!EnsureBridgeReady(pGameDevice)) {
        Log_Printf("VRBridge: bridge setup failed, disabling VR mode");
        g_vrModeEnabled = false;
        return;
    }

    // Only paces the (cheap) copy below. WaitGetPoses/Submit run on their
    // own thread now (see VRSubmitThreadProc) - they used to be called
    // right here on the game's own main thread, which is what caused the
    // slideshow (measured 10-34ms/call for WaitGetPoses alone, blocking
    // the entire game every submission).
    if (!ShouldSubmitThisCall())
        return;

    IDirect3DSurface9* backbuffer = nullptr;
    if (FAILED(pGameDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &backbuffer)) || !backbuffer)
        return;

    RECT leftRect = { 0, 0, static_cast<LONG>(g_eyeWidth), static_cast<LONG>(g_eyeHeight) };
    RECT rightRect = { static_cast<LONG>(g_eyeWidth), 0, static_cast<LONG>(g_eyeWidth) * 2, static_cast<LONG>(g_eyeHeight) };

    bool okLeft = CopyHalfToEye(pGameDevice, backbuffer, leftRect, g_gameLeftCrop, g_gameLeftSysMem, g_bridgeLeftSysMem, g_bridgeLeftSurf, "left");
    bool okRight = CopyHalfToEye(pGameDevice, backbuffer, rightRect, g_gameRightCrop, g_gameRightSysMem, g_bridgeRightSysMem, g_bridgeRightSurf, "right");

    backbuffer->Release();

    static bool loggedFirstCopy = false;
    if (!loggedFirstCopy && okLeft && okRight) {
        loggedFirstCopy = true;
        Log_Printf("VRBridge: first successful eye copy (submit thread handles headset delivery)");
    }

    static UINT64 g_copyCount = 0;
    ++g_copyCount;
    if (!okLeft || !okRight) {
        Log_Printf("VRBridge: eye copy FAILED (count=%llu) okLeft=%d okRight=%d", g_copyCount, okLeft, okRight);
    } else if (g_copyCount % 300 == 0) {
        Log_Printf("VRBridge: copy heartbeat count=%llu", g_copyCount);
    }
}

bool VRBridge_GetHeadDeltaRotation(float outDelta3x3[9])
{
    if (!g_haveHmdDelta)
        return false;
    std::memcpy(outDelta3x3, g_hmdDeltaLocal.m, sizeof(g_hmdDeltaLocal.m));
    return true;
}

bool VRBridge_GetRealFovScale(bool leftEye, float& outScaleX, float& outScaleY)
{
    if (!g_haveRealEyeGeometry)
        return false;
    const int e = leftEye ? 0 : 1;
    outScaleX = g_realFovScaleX[e];
    outScaleY = g_realFovScaleY[e];
    return true;
}

bool VRBridge_GetEyeToHeadRotation(bool leftEye, float outRot3x3[9])
{
    if (!g_haveRealEyeGeometry)
        return false;
    const int e = leftEye ? 0 : 1;
    std::memcpy(outRot3x3, g_eyeToHeadRotation[e].m, sizeof(g_eyeToHeadRotation[e].m));
    return true;
}
