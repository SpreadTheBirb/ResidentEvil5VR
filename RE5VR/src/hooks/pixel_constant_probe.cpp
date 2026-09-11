#include "pixel_constant_probe.h"
#include "fade_probe.h"
#include "head_hide_probe.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <array>
#include <cstring>

namespace {

// Same stable vtable slot numbering used elsewhere in this project.
// SetPixelShaderConstantF is the pixel-shader counterpart to
// SetVertexShaderConstantF (slot 94, already used by constant_probe.cpp).
constexpr size_t kIDirect3DDevice9_SetPixelShaderConstantF = 109;

constexpr int kCaptureWindowFrames = 300;

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

struct RegisterEntry {
    UINT count = 0;
    float lastValues[4] = {};
    UINT lastVector4fCount = 0;
};

std::array<RegisterEntry, 256> g_tally;
int g_framesRemainingInCapture = 0;

// Nudge test: once a candidate register is suspected, force component
// [0] of that register to a large fixed value on every upload. If the
// weapon's opacity visibly locks to one state regardless of camera
// distance while this is on, that confirms the register controls the
// fade (same validation approach as constant_probe's original c0 nudge
// for the camera matrix).
bool g_nudgeEnabled = false;
constexpr UINT kNudgeRegister = 0; // adjust after the first capture dump
constexpr float kNudgeValue = 1000.0f;

// Always-current mirror of every register's last-uploaded value,
// independent of the capture window above - lets a one-shot snapshot
// (Insert) grab "whatever is live right now" for a specific draw call
// rather than a whole-window aggregate. The whole-frame capture above
// mixed together every object's constants with no way to tell which
// draw a given value belonged to; this snapshot is scoped to one
// specific, user-identified draw instead (see PixelConstantProbe_OnDrawCall).
struct LiveRegister {
    bool everSet = false;
    float value[4] = {};
};
std::array<LiveRegister, 256> g_live;

// One-shot: armed by Insert, consumed by the next draw call whose bound
// (texture, vertex buffer) matches whatever's currently selected in the
// head-hide probe's F1/F2 browser - reuses that proven capture/browse UI
// instead of building a second one, so the same technique that found the
// head parts can point at the weapon or any body part too.
bool g_snapshotArmed = false;

typedef HRESULT(WINAPI* SetPixelShaderConstantF_t)(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount);
SetPixelShaderConstantF_t oSetPixelShaderConstantF = nullptr;

HRESULT WINAPI hkSetPixelShaderConstantF(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount)
{
    FadeProbe_OnSetPixelShaderConstantF(StartRegister, pConstantData, Vector4fCount);

    if (Vector4fCount >= 1 && StartRegister < g_live.size() && pConstantData) {
        LiveRegister& live = g_live[StartRegister];
        live.everSet = true;
        std::memcpy(live.value, pConstantData, sizeof(live.value));
    }

    if (g_framesRemainingInCapture > 0 && Vector4fCount <= 4 && StartRegister < g_tally.size() && pConstantData) {
        RegisterEntry& e = g_tally[StartRegister];
        e.count++;
        e.lastVector4fCount = Vector4fCount;
        // Only the first vector4 is logged (fade factors are expected to
        // be a single float or Vector4, not a multi-vector block).
        std::memcpy(e.lastValues, pConstantData, sizeof(e.lastValues));
    }

    if (g_nudgeEnabled && StartRegister == kNudgeRegister && pConstantData) {
        float modified[4];
        std::memcpy(modified, pConstantData, sizeof(modified));
        modified[0] = kNudgeValue;
        return oSetPixelShaderConstantF(This, StartRegister, modified, Vector4fCount);
    }

    return oSetPixelShaderConstantF(This, StartRegister, pConstantData, Vector4fCount);
}

// Render states most likely to carry a CPU-computed fade alpha if the
// fade turns out not to be a shader constant at all - a distance-based
// alpha is just as commonly applied via TEXTUREFACTOR or a blend factor
// as via an explicit constant register.
struct RenderStateOfInterest {
    D3DRENDERSTATETYPE state;
    const char* name;
};
constexpr RenderStateOfInterest kRenderStatesOfInterest[] = {
    { D3DRS_ALPHABLENDENABLE, "ALPHABLENDENABLE" },
    { D3DRS_SRCBLEND, "SRCBLEND" },
    { D3DRS_DESTBLEND, "DESTBLEND" },
    { D3DRS_TEXTUREFACTOR, "TEXTUREFACTOR" },
    { D3DRS_BLENDOP, "BLENDOP" },
    { D3DRS_SEPARATEALPHABLENDENABLE, "SEPARATEALPHABLENDENABLE" },
    { D3DRS_SRCBLENDALPHA, "SRCBLENDALPHA" },
    { D3DRS_DESTBLENDALPHA, "DESTBLENDALPHA" },
};

void DumpDrawSnapshot(IDirect3DDevice9* pDevice)
{
    Log_Printf("PixelConstantProbe: === draw snapshot (selected head-hide-probe candidate) ===");
    for (size_t reg = 0; reg < g_live.size(); ++reg) {
        const LiveRegister& live = g_live[reg];
        if (!live.everSet)
            continue;
        Log_Printf("PixelConstantProbe: c%zu = [%.4f %.4f %.4f %.4f]",
            reg, live.value[0], live.value[1], live.value[2], live.value[3]);
    }
    for (const auto& rs : kRenderStatesOfInterest) {
        DWORD value = 0;
        pDevice->GetRenderState(rs.state, &value);
        Log_Printf("PixelConstantProbe: renderstate %s = %lu (0x%08lX)", rs.name, value, value);
    }
    Log_Printf("PixelConstantProbe: === end draw snapshot ===");
}

void DumpCapture()
{
    // Unlike the vertex-shader camera hunt, a low count alone doesn't
    // isolate a single candidate here - pixel shaders commonly bind
    // several small per-draw material/lighting constants. Dump every
    // register seen with a low (<=4) vector count; the fade candidate is
    // identified by comparing VALUES between a "faded" and "opaque"
    // capture, not by count alone.
    for (size_t reg = 0; reg < g_tally.size(); ++reg) {
        const RegisterEntry& e = g_tally[reg];
        if (e.count == 0)
            continue;
        const float* v = e.lastValues;
        Log_Printf("PixelConstantProbe: c%zu count=%u vecCount=%u last=[%.4f %.4f %.4f %.4f]",
            reg, e.count, e.lastVector4fCount, v[0], v[1], v[2], v[3]);
    }
    Log_Printf("PixelConstantProbe: === capture complete ===");
}

} // namespace

void PixelConstantProbe_Install(IDirect3DDevice9* pDevice)
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("PixelConstantProbe_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    void* pTarget = VTableEntry(pDevice, kIDirect3DDevice9_SetPixelShaderConstantF);
    MH_STATUS st = MH_CreateHook(pTarget, reinterpret_cast<void*>(&hkSetPixelShaderConstantF),
        reinterpret_cast<void**>(&oSetPixelShaderConstantF));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("PixelConstantProbe_Install: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(pTarget);
    Log_Printf("PixelConstantProbe_Install: SetPixelShaderConstantF hook enabled -> %d", static_cast<int>(st));
}

void PixelConstantProbe_OnEndScene()
{
    // VK_HOME isn't present on all keyboards (laptop/compact layouts) -
    // VK_PRIOR (Page Up) is more universally available and unclaimed by
    // any other probe/toggle in this project.
    static bool prevPageUpDown = false;
    bool pageUpDown = (GetAsyncKeyState(VK_PRIOR) & 0x8000) != 0;
    if (pageUpDown && !prevPageUpDown && g_framesRemainingInCapture == 0) {
        g_framesRemainingInCapture = kCaptureWindowFrames;
        for (auto& e : g_tally)
            e.count = 0;
        Log_Printf("PixelConstantProbe: Page Up pressed, capturing next %d frames", kCaptureWindowFrames);
    }
    prevPageUpDown = pageUpDown;

    if (g_framesRemainingInCapture > 0) {
        --g_framesRemainingInCapture;
        if (g_framesRemainingInCapture == 0)
            DumpCapture();
    }

    // Switched from VK_END after the first real test (2026-07-27): user
    // saw a broad renderer-wide change on the first press that might have
    // been RE5's own End keybind firing at the same time as our nudge,
    // not necessarily (only) our c0 nudge - VK_NEXT (Page Down) avoids
    // that ambiguity.
    static bool prevPageDownDown = false;
    bool pageDownDown = (GetAsyncKeyState(VK_NEXT) & 0x8000) != 0;
    if (pageDownDown && !prevPageDownDown) {
        g_nudgeEnabled = !g_nudgeEnabled;
        Log_Printf("PixelConstantProbe: Page Down pressed, nudge test on c%u now %s",
            kNudgeRegister, g_nudgeEnabled ? "ON" : "OFF");
    }
    prevPageDownDown = pageDownDown;

    // VK_INSERT isn't present on all keyboards either (same issue as the
    // earlier Home swap) - VK_DELETE is confirmed present.
    static bool prevDeleteDown = false;
    bool deleteDown = (GetAsyncKeyState(VK_DELETE) & 0x8000) != 0;
    if (deleteDown && !prevDeleteDown) {
        void* selTex = nullptr;
        void* selVb = nullptr;
        if (HeadHideProbe_GetSelectedSignature(&selTex, &selVb)) {
            g_snapshotArmed = true;
            Log_Printf("PixelConstantProbe: Delete pressed, will snapshot the next draw matching the head-hide probe's selected candidate (tex=%p vb=%p)", selTex, selVb);
        } else {
            Log_Printf("PixelConstantProbe: Delete pressed but head-hide probe has no candidate selected - capture (F11) and select one (F1/F2) first");
        }
    }
    prevDeleteDown = deleteDown;
}

void PixelConstantProbe_OnDrawCall(IDirect3DDevice9* pDevice)
{
    if (!g_snapshotArmed)
        return;

    void* selTex = nullptr;
    void* selVb = nullptr;
    if (!HeadHideProbe_GetSelectedSignature(&selTex, &selVb))
        return;

    IDirect3DBaseTexture9* tex = nullptr;
    pDevice->GetTexture(0, &tex);
    if (tex)
        tex->Release();

    IDirect3DVertexBuffer9* vb = nullptr;
    UINT offset = 0, stride = 0;
    pDevice->GetStreamSource(0, &vb, &offset, &stride);
    if (vb)
        vb->Release();

    if (tex != selTex || vb != selVb)
        return;

    DumpDrawSnapshot(pDevice);
    g_snapshotArmed = false;
}
