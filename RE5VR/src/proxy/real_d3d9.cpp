#include "real_d3d9.h"
#include "blind_forwards.h"
#include "../hooks/d3d9_hooks.h"
#include "../util/log.h"

#include <windows.h>
#include <cstdio>
#include <cstring>

PFN_Direct3DCreate9Ex g_RealDirect3DCreate9Ex = nullptr;

namespace {
HMODULE g_hRealD3D9 = nullptr;
}

bool RealD3D9_Init()
{
    if (g_RealDirect3DCreate9Ex)
        return true;

    // 2026-09-10 dgVoodoo2 experiment: chain to dgVoodoo2's own D3D9->D3D11
    // wrapper (dropped in the game folder as "d3d9_dgvoodoo.dll", renamed to
    // avoid colliding with this proxy's own "d3d9.dll") instead of the real
    // system d3d9.dll, using the exact same chaining mechanism this file
    // already used for the system DLL - it exports the same
    // Direct3DCreate9/Ex ABI, confirmed via dumpbin. Revert to the
    // GetSystemDirectoryA block below (still intact in version control) to
    // go back to the plain system d3d9.dll.
    char moduleDir[MAX_PATH] = {};
    HMODULE hSelf = nullptr;
    GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
        reinterpret_cast<LPCSTR>(&RealD3D9_Init),
        &hSelf);
    GetModuleFileNameA(hSelf, moduleDir, MAX_PATH);
    char* lastSlash = strrchr(moduleDir, '\\');
    if (lastSlash)
        *(lastSlash + 1) = '\0';

    char realPath[MAX_PATH] = {};
    _snprintf_s(realPath, sizeof(realPath), _TRUNCATE, "%sd3d9_dgvoodoo.dll", moduleDir);

    g_hRealD3D9 = LoadLibraryA(realPath);
    if (!g_hRealD3D9) {
        Log_Printf("RealD3D9_Init: LoadLibraryA(%s) failed (err=%lu)", realPath, GetLastError());
        return false;
    }

    g_RealDirect3DCreate9Ex = reinterpret_cast<PFN_Direct3DCreate9Ex>(
        GetProcAddress(g_hRealD3D9, "Direct3DCreate9Ex"));

    if (!g_RealDirect3DCreate9Ex) {
        Log_Printf("RealD3D9_Init: GetProcAddress(Direct3DCreate9Ex) failed (err=%lu)", GetLastError());
        return false;
    }

    ResolveBlindForwards(g_hRealD3D9);

    Log_Printf("RealD3D9_Init: loaded real d3d9.dll from %s (module=%p)", realPath, g_hRealD3D9);

    // Bootstrap the CreateDevice hook ourselves, independent of whether the
    // game ever calls this proxy's own Direct3DCreate9/Ex exports. RE5
    // resolves Direct3DCreate9 some other way (its repeated GetProcAddress
    // lookups against a separately-loaded d3d9.dll instance are never
    // actually invoked - likely a watchdog/anti-tamper check, with the real
    // resolution done via manual export-table parsing that bypasses
    // GetProcAddress entirely). CreateDevice's code lives at a fixed
    // address in the shared module no matter how a caller obtained its
    // interface pointer, so hooking it via our own throwaway probe object
    // patches the code itself and intercepts every caller.
    IDirect3D9Ex* pProbe = nullptr;
    HRESULT probeHr = g_RealDirect3DCreate9Ex(D3D_SDK_VERSION, &pProbe);
    if (SUCCEEDED(probeHr) && pProbe) {
        Log_Printf("RealD3D9_Init: bootstrap probe IDirect3D9Ex created, installing CreateDevice hook");
        Hooks_OnD3D9ExCreated(pProbe);
        pProbe->Release();
    } else {
        Log_Printf("RealD3D9_Init: bootstrap Direct3DCreate9Ex failed (hr=0x%08lX)", probeHr);
    }

    return true;
}

extern "C" IDirect3D9* WINAPI Direct3DCreate9(UINT SDKVersion)
{
    Log_Printf("Direct3DCreate9(SDKVersion=%u) called", SDKVersion);

    if (!RealD3D9_Init())
        return nullptr;

    IDirect3D9Ex* pD3D9Ex = nullptr;
    HRESULT hr = g_RealDirect3DCreate9Ex(SDKVersion, &pD3D9Ex);
    if (FAILED(hr) || !pD3D9Ex) {
        Log_Printf("Direct3DCreate9: real Direct3DCreate9Ex failed (hr=0x%08lX)", hr);
        return nullptr;
    }

    Hooks_OnD3D9ExCreated(pD3D9Ex);

    // IDirect3D9Ex publicly derives from IDirect3D9 with an identical base
    // vtable layout, so this pointer is safe to hand back as IDirect3D9*.
    return static_cast<IDirect3D9*>(pD3D9Ex);
}

extern "C" HRESULT WINAPI Direct3DCreate9Ex(UINT SDKVersion, IDirect3D9Ex** ppD3D)
{
    Log_Printf("Direct3DCreate9Ex(SDKVersion=%u) called", SDKVersion);

    if (!RealD3D9_Init())
        return E_FAIL;

    HRESULT hr = g_RealDirect3DCreate9Ex(SDKVersion, ppD3D);
    if (SUCCEEDED(hr) && ppD3D && *ppD3D)
        Hooks_OnD3D9ExCreated(*ppD3D);

    return hr;
}
