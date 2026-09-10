#include "constant_probe.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <array>
#include <cstring>

namespace {

// Same stable vtable slot numbering used in d3d9_hooks.cpp.
constexpr size_t kIDirect3DDevice9_SetTransform = 44;
constexpr size_t kIDirect3DDevice9_SetVertexShaderConstantF = 94;

// At this game's uncapped framerate (500-1000+ fps observed), a single
// captured frame is too easy to land on a trivial/HUD-only pass or a frame
// where the engine skipped re-uploading unchanged constants. Capture a
// whole window of frames instead and aggregate.
constexpr int kCaptureWindowFrames = 600;

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

struct RegisterEntry {
    UINT count = 0;
    float lastValues[16] = {};
};

std::array<RegisterEntry, 256> g_tallyF;
int g_framesRemainingInCapture = 0;

// Empirical validation: c0 (the register found via capture, believed to be
// RE5's per-frame view/view-projection matrix) has its component [3] (the
// translation-like term of that row) nudged by a large, obvious amount
// while this is toggled on. If the camera visibly shifts/shears on screen,
// that confirms both that this register genuinely feeds the render output
// and roughly what a positional nudge to it does - without first needing
// to fully decompose the matrix's exact mathematical convention.
bool g_offsetTestEnabled = false;
constexpr float kOffsetTestAmount = 800.0f;

// The most recently observed *true* value of c0 (register 0, 4 vectors),
// cached unconditionally regardless of capture-window/offset-test state,
// so stereo_test.cpp can build per-eye variants from a known-good baseline
// instead of drifting off of its own previous override.
float g_cachedCameraMatrix[16] = {};
bool g_haveCachedCameraMatrix = false;

typedef HRESULT(WINAPI* SetTransform_t)(IDirect3DDevice9* This, D3DTRANSFORMSTATETYPE State, const D3DMATRIX* pMatrix);
SetTransform_t oSetTransform = nullptr;

HRESULT WINAPI hkSetTransform(IDirect3DDevice9* This, D3DTRANSFORMSTATETYPE State, const D3DMATRIX* pMatrix)
{
    if (g_framesRemainingInCapture > 0 && pMatrix) {
        const float* m = &pMatrix->_11;
        Log_Printf("ConstantProbe: SetTransform State=%d [%.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f]",
            static_cast<int>(State), m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
    }
    return oSetTransform(This, State, pMatrix);
}

typedef HRESULT(WINAPI* SetVertexShaderConstantF_t)(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount);
SetVertexShaderConstantF_t oSetVertexShaderConstantF = nullptr;

HRESULT WINAPI hkSetVertexShaderConstantF(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount)
{
    if (StartRegister == 0 && Vector4fCount == 4 && pConstantData) {
        std::memcpy(g_cachedCameraMatrix, pConstantData, sizeof(g_cachedCameraMatrix));
        g_haveCachedCameraMatrix = true;
    }

    if (g_framesRemainingInCapture > 0 && Vector4fCount == 4 && StartRegister < g_tallyF.size() && pConstantData) {
        RegisterEntry& e = g_tallyF[StartRegister];
        e.count++;
        std::memcpy(e.lastValues, pConstantData, sizeof(e.lastValues));
    }

    if (g_offsetTestEnabled && StartRegister == 0 && Vector4fCount == 4 && pConstantData) {
        float modified[16];
        std::memcpy(modified, pConstantData, sizeof(modified));
        modified[3] += kOffsetTestAmount;
        return oSetVertexShaderConstantF(This, StartRegister, modified, Vector4fCount);
    }

    return oSetVertexShaderConstantF(This, StartRegister, pConstantData, Vector4fCount);
}

void DumpCapture()
{
    // A register written roughly once (or a couple times) per frame across
    // the whole window is a global/camera candidate; one written dozens of
    // times per frame (per-object world matrices, bone palettes) will have
    // accumulated a much larger count over the same window.
    const UINT candidateThreshold = kCaptureWindowFrames * 2;

    for (size_t reg = 0; reg < g_tallyF.size(); ++reg) {
        const RegisterEntry& e = g_tallyF[reg];
        if (e.count == 0)
            continue;

        if (e.count <= candidateThreshold) {
            const float* m = e.lastValues;
            Log_Printf("ConstantProbe: c%zu count=%u [%.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f]",
                reg, e.count, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
        } else {
            Log_Printf("ConstantProbe: c%zu count=%u (skipping values - likely per-object)", reg, e.count);
        }
    }
    Log_Printf("ConstantProbe: === capture complete ===");
}

} // namespace

void ConstantProbe_Install(IDirect3DDevice9* pDevice)
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("ConstantProbe_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    struct HookSpec {
        size_t slot;
        void* detour;
        void** original;
        const char* name;
    };

    HookSpec specs[] = {
        { kIDirect3DDevice9_SetTransform,            reinterpret_cast<void*>(&hkSetTransform),            reinterpret_cast<void**>(&oSetTransform),            "SetTransform" },
        { kIDirect3DDevice9_SetVertexShaderConstantF, reinterpret_cast<void*>(&hkSetVertexShaderConstantF), reinterpret_cast<void**>(&oSetVertexShaderConstantF), "SetVertexShaderConstantF" },
    };

    for (const auto& spec : specs) {
        void* pTarget = VTableEntry(pDevice, spec.slot);
        MH_STATUS st = MH_CreateHook(pTarget, spec.detour, spec.original);
        if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
            Log_Printf("ConstantProbe_Install: MH_CreateHook(%s) failed -> %d", spec.name, static_cast<int>(st));
            continue;
        }
        st = MH_EnableHook(pTarget);
        Log_Printf("ConstantProbe_Install: %s hook enabled -> %d", spec.name, static_cast<int>(st));
    }
}

void ConstantProbe_OnEndScene()
{
    static bool prevF9Down = false;
    bool f9Down = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
    if (f9Down && !prevF9Down && g_framesRemainingInCapture == 0) {
        g_framesRemainingInCapture = kCaptureWindowFrames;
        for (auto& e : g_tallyF)
            e.count = 0;
        Log_Printf("ConstantProbe: F9 pressed, capturing next %d frames", kCaptureWindowFrames);
    }
    prevF9Down = f9Down;

    if (g_framesRemainingInCapture > 0) {
        --g_framesRemainingInCapture;
        if (g_framesRemainingInCapture == 0)
            DumpCapture();
    }

    static bool prevF10Down = false;
    bool f10Down = (GetAsyncKeyState(VK_F10) & 0x8000) != 0;
    if (f10Down && !prevF10Down) {
        g_offsetTestEnabled = !g_offsetTestEnabled;
        Log_Printf("ConstantProbe: F10 pressed, offset test now %s", g_offsetTestEnabled ? "ON" : "OFF");
    }
    prevF10Down = f10Down;

    // F6: dump the current cached c0 matrix (16 floats) on demand, so it
    // can be captured at different known camera orientations/positions and
    // diffed by hand. Needed to figure out c0's actual layout (row-major
    // vs column-major, which slots are rotation basis vectors vs
    // translation) before real head-tracked rotation can be implemented -
    // we currently only know component [3] acts as a translation-like
    // term when nudged, which isn't enough to compose a rotation into it.
    static bool prevF6Down = false;
    bool f6Down = (GetAsyncKeyState(VK_F6) & 0x8000) != 0;
    if (f6Down && !prevF6Down) {
        if (g_haveCachedCameraMatrix) {
            const float* m = g_cachedCameraMatrix;
            Log_Printf("ConstantProbe: F6 snapshot c0 [%.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f]",
                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
        } else {
            Log_Printf("ConstantProbe: F6 pressed but no cached camera matrix yet");
        }
    }
    prevF6Down = f6Down;
}

bool ConstantProbe_GetCachedCameraMatrix(float out[16])
{
    if (!g_haveCachedCameraMatrix)
        return false;
    std::memcpy(out, g_cachedCameraMatrix, sizeof(g_cachedCameraMatrix));
    return true;
}

HRESULT ConstantProbe_CallRealSetVertexShaderConstantF(IDirect3DDevice9* pDevice, UINT StartRegister, const float* pConstantData, UINT Vector4fCount)
{
    if (!oSetVertexShaderConstantF)
        return E_FAIL;
    return oSetVertexShaderConstantF(pDevice, StartRegister, pConstantData, Vector4fCount);
}
