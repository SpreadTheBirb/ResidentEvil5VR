#include "d3d9_hooks.h"
#include "constant_probe.h"
#include "pixel_constant_probe.h"
#include "head_hide_probe.h"
#include "camera_rig_hook.h"
#include "boom_finder.h"
#include "fade_probe.h"
#include "fade_patch.h"
#include "culling_patch.h"
#include "query_probe.h"
#include "skeleton_probe.h"
#include "state_probe.h"
#include "laser_patch.h"
#include "filter_patch.h"
#include "hud_probe.h"
#include "../render/stereo_test.h"
#include "../render/hud_shaders.h"
#include "../render/render_size.h"
#include "../vr/openxr_bridge.h"
#include "../ui/input_block.h"
#include "../ui/menu.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <cstdio>
#include <cstring>

// Developer diagnostics: the probes and captures used to find things in the
// game, plus the Phase 0 blinking quad. Off in released builds - players hit
// F-keys by accident, and some of these visibly break the camera or write
// large files into the game folder. Now shared, so the experiment hotkeys in
// the VR bridge and stereo path are gated by the same switch.
#include "../util/build_config.h"

namespace {

// Stable, widely-documented Direct3D9 COM vtable slot indices (unchanged
// since Windows XP/Vista; 0 = QueryInterface). Verified against the
// IDirect3D9 / IDirect3DDevice9 declaration order in d3d9.h.
constexpr size_t kIDirect3D9_CreateDevice = 16;
constexpr size_t kIDirect3DDevice9_EndScene = 42;

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

bool g_minHookInitialized = false;
bool g_deviceHooksInstalled = false;

void EnsureMinHookInitialized()
{
    if (g_minHookInitialized)
        return;
    MH_STATUS st = MH_Initialize();
    g_minHookInitialized = (st == MH_OK || st == MH_ERROR_ALREADY_INITIALIZED);
    Log_Printf("MH_Initialize -> %d", static_cast<int>(st));
}


// ---- Resolution ------------------------------------------------------------
// Each VR eye used to be half the backbuffer. Forcing a bigger backbuffer here
// (re5vr_res.txt, 2026-09-12 to 2026-09-14) turned out to be the wrong lever:
// RE5 sizes its scene targets from its own render size, not the backbuffer,
// and just copied the old-size image into the corner of the bigger frame. Full
// resolution per eye now asks RE5 itself for the size - render/render_size.cpp.

// Logs what the backbuffer ACTUALLY ended up as, which is the only proof that
// an override stuck: D3D9 is free to hand back something else, and dgVoodoo
// sits in the middle with opinions of its own.
void LogActualBackbuffer(IDirect3DDevice9* pDevice, const char* when)
{
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(pDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb) {
        Log_Printf("LogActualBackbuffer(%s): GetBackBuffer failed", when);
        return;
    }
    D3DSURFACE_DESC d = {};
    const HRESULT hr = bb->GetDesc(&d);
    bb->Release();
    if (FAILED(hr)) {
        Log_Printf("LogActualBackbuffer(%s): GetDesc failed (0x%08lX)", when, hr);
        return;
    }
    Log_Printf("LogActualBackbuffer(%s): backbuffer is %ux%u (format %d, multisample %d) - each VR eye gets %ux%u",
        when, d.Width, d.Height, static_cast<int>(d.Format), static_cast<int>(d.MultiSampleType),
        d.Width / 2, d.Height);
}

// ---- IDirect3D9::CreateDevice -------------------------------------------
//
// Phase 0 only needs to prove this proxy's hooks run inside the game's real
// D3D9 pipeline (log + on-screen debug quad) - it doesn't need an Ex
// device. Upgrading the game's own device to Ex (via CreateDeviceEx, for
// Phase 2's shared-surface textures) turned out to be fragile on this
// driver's windowed<->exclusive-fullscreen transition even after fixing
// every documented Ex restriction (SwapEffect, desktop BackBufferFormat,
// D3DCREATE_PUREDEVICE, pFullscreenDisplayMode) - PresentEx/ResetEx kept
// returning D3DERR_INVALIDCALL. Rather than keep fighting that, Phase 0
// leaves the game's device completely untouched (plain CreateDevice) and
// only hooks EndScene. Shared-surface textures for the VR bridge will come
// from a separate, dedicated hidden Ex device created later (Phase 2),
// decoupled from the game's own device/window lifecycle entirely.

typedef HRESULT(WINAPI* CreateDevice_t)(
    IDirect3D9* This,
    UINT Adapter,
    D3DDEVTYPE DeviceType,
    HWND hFocusWindow,
    DWORD BehaviorFlags,
    D3DPRESENT_PARAMETERS* pPresentationParameters,
    IDirect3DDevice9** ppReturnedDeviceInterface);

CreateDevice_t oCreateDevice = nullptr;

HRESULT WINAPI hkCreateDevice(
    IDirect3D9* This,
    UINT Adapter,
    D3DDEVTYPE DeviceType,
    HWND hFocusWindow,
    DWORD BehaviorFlags,
    D3DPRESENT_PARAMETERS* pPresentationParameters,
    IDirect3DDevice9** ppReturnedDeviceInterface)
{
    Log_Printf("hkCreateDevice: Adapter=%u DeviceType=%d BehaviorFlags=0x%08lX Windowed=%d %ux%u",
        Adapter, DeviceType, BehaviorFlags,
        pPresentationParameters ? pPresentationParameters->Windowed : -1,
        pPresentationParameters ? pPresentationParameters->BackBufferWidth : 0,
        pPresentationParameters ? pPresentationParameters->BackBufferHeight : 0);

    HRESULT hr = oCreateDevice(This, Adapter, DeviceType, hFocusWindow, BehaviorFlags,
        pPresentationParameters, ppReturnedDeviceInterface);

    if (SUCCEEDED(hr) && ppReturnedDeviceInterface && *ppReturnedDeviceInterface) {
        Log_Printf("hkCreateDevice: real CreateDevice succeeded");
        LogActualBackbuffer(*ppReturnedDeviceInterface, "after CreateDevice");
        Hooks_OnDeviceCreated(*ppReturnedDeviceInterface);
    } else {
        Log_Printf("hkCreateDevice: real CreateDevice failed (hr=0x%08lX)", hr);
    }

    return hr;
}

// ---- IDirect3DDevice9::EndScene: frame counter + Phase 0 debug quad ----

typedef HRESULT(WINAPI* EndScene_t)(IDirect3DDevice9* This);

EndScene_t oEndScene = nullptr;
UINT64 g_frameCounter = 0;

#if RE5VR_DIAGNOSTICS
struct DebugVertex {
    float x, y, z, rhw;
    DWORD color;
};

void DrawDebugQuad(IDirect3DDevice9* pDevice)
{
    const DWORD color = ((g_frameCounter / 30) % 2) ? 0xFFFF0000 : 0xFF00FF00;
    const float left = 10.0f, top = 10.0f, size = 40.0f;

    DebugVertex verts[4] = {
        { left,        top,        0.0f, 1.0f, color },
        { left + size, top,        0.0f, 1.0f, color },
        { left,        top + size, 0.0f, 1.0f, color },
        { left + size, top + size, 0.0f, 1.0f, color },
    };

    pDevice->SetTexture(0, nullptr);
    pDevice->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    pDevice->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    pDevice->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    pDevice->SetRenderState(D3DRS_ZENABLE, FALSE);
    pDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    pDevice->SetRenderState(D3DRS_LIGHTING, FALSE);
    pDevice->SetFVF(D3DFVF_XYZRHW | D3DFVF_DIFFUSE);

    // Never let this get caught up in stereo_test's draw duplication/
    // render-target redirection - it must always land on the real,
    // currently-visible backbuffer.
    StereoTest_SetSuppressed(true);
    pDevice->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, verts, sizeof(DebugVertex));
    StereoTest_SetSuppressed(false);
}
#endif


// ---- IDirect3DDevice9::Reset --------------------------------------------
// The game resets the device whenever display settings change, on some
// alt-tab paths, and when full resolution per eye switches its render size.
constexpr size_t kIDirect3DDevice9_Reset = 16;

typedef HRESULT(WINAPI* Reset_t)(IDirect3DDevice9* This, D3DPRESENT_PARAMETERS* pPresentationParameters);
Reset_t oReset = nullptr;

HRESULT WINAPI hkReset(IDirect3DDevice9* This, D3DPRESENT_PARAMETERS* pPresentationParameters)
{
    Log_Printf("hkReset: game asked for %ux%u (windowed=%d)",
        pPresentationParameters ? pPresentationParameters->BackBufferWidth : 0,
        pPresentationParameters ? pPresentationParameters->BackBufferHeight : 0,
        pPresentationParameters ? pPresentationParameters->Windowed : -1);
    RenderSize_OnBeforeReset(pPresentationParameters);
    Menu_OnBeforeReset(); // the menu's D3DPOOL_DEFAULT textures and buffers must go first
    const HRESULT hr = oReset(This, pPresentationParameters);
    Menu_OnAfterReset();
    if (SUCCEEDED(hr)) {
        LogActualBackbuffer(This, "after Reset");
        StereoTest_OnDeviceReset();
    } else {
        Log_Printf("hkReset: Reset failed (hr=0x%08lX)", hr);
    }
    RenderSize_OnReset(hr);
    return hr;
}
// ---- IDirect3DDevice9::Present: the one true frame boundary ------------
// EndScene fires several times per rendered frame here (once per pass), so
// it can't mark "a new frame starts now". Present can: stereo_test latches
// the head pose there so every pass of the next frame uses the same one.
constexpr size_t kIDirect3DDevice9_Present = 17;

typedef HRESULT(WINAPI* Present_t)(IDirect3DDevice9* This, const RECT* pSourceRect, const RECT* pDestRect,
    HWND hDestWindowOverride, const RGNDATA* pDirtyRegion);
Present_t oPresent = nullptr;

HRESULT WINAPI hkPresent(IDirect3DDevice9* This, const RECT* pSourceRect, const RECT* pDestRect,
    HWND hDestWindowOverride, const RGNDATA* pDirtyRegion)
{
    // Before the real Present, so the menu is in the frame that reaches the
    // monitor and the headset.
    Menu_OnPresent(This);
    const HRESULT hr = oPresent(This, pSourceRect, pDestRect, hDestWindowOverride, pDirtyRegion);
    StereoTest_OnPresent();
    RenderSize_OnPresent(This);
#if RE5VR_DIAGNOSTICS
    HudProbe_OnPresent(); // K: record the HUD's draw calls
#endif
    return hr;
}

HRESULT WINAPI hkEndScene(IDirect3DDevice9* This)
{
    // The menu's own scene: none of the per-frame work below belongs to it.
    if (Menu_IsDrawing())
        return oEndScene(This);
    ++g_frameCounter;

    // Poll the F8 stereo-test toggle first.
    StereoTest_OnEndScene(This);
    CameraRigHook_OnEndScene();
#if RE5VR_DIAGNOSTICS
    BoomFinder_OnEndScene();   // F5: hardware-watchpoint finder
    StateProbe_OnEndScene();   // "=": game-state capture
    SkeletonProbe_OnEndScene();
#endif
    FadePatch_OnEndScene();
    CullingPatch_OnEndScene();
    QueryProbe_OnEndScene();

    // At this point the game's own rendering for this frame is completely
    // finished (backbuffer holds the final, fully composited/tonemapped
    // image - side-by-side if stereo_test is on) - the right moment to
    // grab it and submit to the headset, before anything else (like the
    // debug quad below) draws on top of it.
    VRBridge_OnEndScene(This);

#if RE5VR_DIAGNOSTICS
    DrawDebugQuad(This);
    ConstantProbe_OnEndScene();      // F6/F9/F10 (F10 visibly shears the camera)
    PixelConstantProbe_OnEndScene(); // Page Up/Down, Delete
    HeadHideProbe_OnEndScene();      // F1-F3, F11, F12 (F12 is Steam's screenshot key)
#endif

    if (g_frameCounter % 300 == 0)
        Log_Printf("hkEndScene: frame %llu", g_frameCounter);

    return oEndScene(This);
}

} // namespace

void Hooks_OnD3D9ExCreated(IDirect3D9Ex* pD3D9Ex)
{
    EnsureMinHookInitialized();
    if (!g_minHookInitialized)
        return;

    void* pTarget = VTableEntry(pD3D9Ex, kIDirect3D9_CreateDevice);

    MH_STATUS st = MH_CreateHook(pTarget, reinterpret_cast<void*>(&hkCreateDevice),
        reinterpret_cast<void**>(&oCreateDevice));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("Hooks_OnD3D9ExCreated: MH_CreateHook(CreateDevice) failed -> %d", static_cast<int>(st));
        return;
    }

    st = MH_EnableHook(pTarget);
    Log_Printf("Hooks_OnD3D9ExCreated: CreateDevice hook enabled -> %d", static_cast<int>(st));
}

void Hooks_OnDeviceCreated(IDirect3DDevice9* pDevice)
{
    if (g_deviceHooksInstalled)
        return;
    g_deviceHooksInstalled = true;

    EnsureMinHookInitialized();
    if (!g_minHookInitialized)
        return;

    void* pTarget = VTableEntry(pDevice, kIDirect3DDevice9_EndScene);
    MH_STATUS st = MH_CreateHook(pTarget, reinterpret_cast<void*>(&hkEndScene),
        reinterpret_cast<void**>(&oEndScene));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("Hooks_OnDeviceCreated: MH_CreateHook(EndScene) failed -> %d", static_cast<int>(st));
        return;
    }

    st = MH_EnableHook(pTarget);
    Log_Printf("Hooks_OnDeviceCreated: EndScene hook enabled -> %d", static_cast<int>(st));

    void* pPresent = VTableEntry(pDevice, kIDirect3DDevice9_Present);
    st = MH_CreateHook(pPresent, reinterpret_cast<void*>(&hkPresent), reinterpret_cast<void**>(&oPresent));
    if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
        st = MH_EnableHook(pPresent);
    Log_Printf("Hooks_OnDeviceCreated: Present hook enabled -> %d", static_cast<int>(st));

    // Reset: the menu has to release its D3DPOOL_DEFAULT resources around it.
    void* pReset = VTableEntry(pDevice, kIDirect3DDevice9_Reset);
    st = MH_CreateHook(pReset, reinterpret_cast<void*>(&hkReset), reinterpret_cast<void**>(&oReset));
    if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
        st = MH_EnableHook(pReset);
    Log_Printf("Hooks_OnDeviceCreated: Reset hook enabled -> %d", static_cast<int>(st));

    ConstantProbe_Install(pDevice);
    PixelConstantProbe_Install(pDevice);
    QueryProbe_Install(pDevice);
    // Before the game loads its shaders, so every one of them is hashed at creation.
    HudShaders_Install(pDevice);
    StereoTest_Install(pDevice);
    VRBridge_Install();
    CameraRigHook_Install();
    FadePatch_Install();
    CullingPatch_Install();
    LaserPatch_Install();
    FilterPatch_Install();
#if RE5VR_DIAGNOSTICS
    HudProbe_Install();
#endif
    // Last: loading re5vr.ini applies settings through everything above.
    InputBlock_Install();
    Menu_Install(pDevice);
}
