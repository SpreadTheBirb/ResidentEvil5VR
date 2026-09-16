#include "real_d3d9.h"
#include "blind_forwards.h"
#include "../hooks/d3d9_hooks.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>
#include <tlhelp32.h>
#include <cstdio>
#include <cstring>

PFN_Direct3DCreate9Ex g_RealDirect3DCreate9Ex = nullptr;

namespace {
HMODULE g_hRealD3D9 = nullptr;

// Did we chain to dgVoodoo2, or fall back to the system d3d9.dll? VR needs
// dgVoodoo2's D3D12 frames, so this is also the answer to "can this install
// do VR at all" - see RealD3D9_UsingDgVoodoo.
bool g_usingDgVoodoo = false;
// The CreateDevice-bootstrap probe (dgVoodoo2 route), kept for the whole run -
// see RealD3D9_Init.
IDirect3D9Ex* g_bootstrapProbe = nullptr;

// One creation at a time (2026-09-15). dgVoodoo2 crashed on a tester's PC when
// something else in the process entered its Direct3DCreate9Ex while the game's
// own call was still inside it starting D3D12 up. The GetProcAddress hook no
// longer hands our exports to anything but the game, and this is the belt to
// that pair of braces: a DLL bound to this proxy by its import table reaches
// these exports without asking GetProcAddress at all.
SRWLOCK g_createLock = SRWLOCK_INIT;
volatile LONG g_createOwner = 0; // thread already inside, so we don't self-deadlock

class CreateGuard {
public:
    CreateGuard()
    {
        const LONG self = static_cast<LONG>(GetCurrentThreadId());
        held_ = InterlockedCompareExchange(&g_createOwner, 0, 0) != self;
        if (held_) {
            AcquireSRWLockExclusive(&g_createLock);
            InterlockedExchange(&g_createOwner, self);
        }
    }
    ~CreateGuard()
    {
        if (held_) {
            InterlockedExchange(&g_createOwner, 0);
            ReleaseSRWLockExclusive(&g_createLock);
        }
    }

private:
    bool held_ = false;
};

// System-d3d9 route (DXVK under Proton): the game never called our
// Direct3DCreate9 export there (2026-09-11 Linux log: no call, so no device
// hooks and a dead F4), so hook the system DLL's own entry points instead.
// However the game reaches them, the object it gets back passes through here
// and its CreateDevice gets hooked. Pure code patching - safe from DllMain.
typedef IDirect3D9*(WINAPI* PFN_Direct3DCreate9)(UINT SDKVersion);
PFN_Direct3DCreate9 g_origSystemCreate9 = nullptr;
PFN_Direct3DCreate9Ex g_origSystemCreate9Ex = nullptr;

// What else is in the process when D3D is created (2026-09-15). Overlays and
// other d3d9 wrappers are the usual suspects when a launch crashes inside
// dgVoodoo2 on one PC and nowhere else, and their names are the only way to
// tell from a log. Names only, no paths, and only once.
void LogLoadedModulesOnce()
{
    static bool logged = false;
    if (logged)
        return;
    logged = true;

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
    if (snap == INVALID_HANDLE_VALUE) {
        Log_Printf("LoadedModules: snapshot failed (err=%lu)", GetLastError());
        return;
    }

    char line[1024] = {};
    size_t used = 0;
    MODULEENTRY32 me = {};
    me.dwSize = sizeof(me);
    for (BOOL ok = Module32First(snap, &me); ok; ok = Module32Next(snap, &me)) {
        const size_t len = strlen(me.szModule);
        if (used + len + 2 >= sizeof(line)) {
            Log_Printf("LoadedModules: %s ...", line);
            used = 0;
            line[0] = '\0';
        }
        if (used) {
            line[used++] = ' ';
            line[used] = '\0';
        }
        memcpy(line + used, me.szModule, len + 1);
        used += len;
    }
    CloseHandle(snap);
    if (used)
        Log_Printf("LoadedModules: %s", line);
}

IDirect3D9* WINAPI hkSystemDirect3DCreate9(UINT SDKVersion)
{
    IDirect3D9* d3d = g_origSystemCreate9(SDKVersion);
    Log_Printf("system Direct3DCreate9(SDKVersion=%u) -> %p", SDKVersion, d3d);
    // Only CreateDevice's vtable slot is used, and it's the same in both.
    if (d3d)
        Hooks_OnD3D9ExCreated(reinterpret_cast<IDirect3D9Ex*>(d3d));
    return d3d;
}

HRESULT WINAPI hkSystemDirect3DCreate9Ex(UINT SDKVersion, IDirect3D9Ex** ppD3D)
{
    const HRESULT hr = g_origSystemCreate9Ex(SDKVersion, ppD3D);
    Log_Printf("system Direct3DCreate9Ex(SDKVersion=%u) -> hr=0x%08lX", SDKVersion, hr);
    if (SUCCEEDED(hr) && ppD3D && *ppD3D)
        Hooks_OnD3D9ExCreated(*ppD3D);
    return hr;
}

void HookSystemEntry(const char* name, void* detour, void** original)
{
    void* target = reinterpret_cast<void*>(GetProcAddress(g_hRealD3D9, name));
    if (!target) {
        Log_Printf("RealD3D9_Init: system d3d9.dll has no %s - not hooked", name);
        return;
    }
    MH_STATUS st = MH_CreateHook(target, detour, original);
    if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
        st = MH_EnableHook(target);
    Log_Printf("RealD3D9_Init: system %s hook enabled -> %d", name, static_cast<int>(st));
}

void HookSystemCreateFunctions()
{
    const MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("RealD3D9_Init: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }
    HookSystemEntry("Direct3DCreate9", reinterpret_cast<void*>(&hkSystemDirect3DCreate9),
        reinterpret_cast<void**>(&g_origSystemCreate9));
    HookSystemEntry("Direct3DCreate9Ex", reinterpret_cast<void*>(&hkSystemDirect3DCreate9Ex),
        reinterpret_cast<void**>(&g_origSystemCreate9Ex));
}
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
    const bool usingDgVoodoo = g_hRealD3D9 != nullptr;
    g_usingDgVoodoo = usingDgVoodoo;
    if (!usingDgVoodoo) {
        // No dgVoodoo2 next to us: fall back to the system d3d9.dll - Windows'
        // own on Windows, DXVK under Proton. dgVoodoo2's author doesn't
        // support Wine/Proton at all, so that's the Linux route: install
        // without d3d9_dgvoodoo.dll and SampleAddon.dll, and get flat play,
        // first person included, on DXVK. VR needs dgVoodoo2's D3D12 frames.
        Log_Printf("RealD3D9_Init: %s not loaded (err=%lu) - falling back to the system d3d9.dll",
            realPath, GetLastError());
        char systemDir[MAX_PATH] = {};
        GetSystemDirectoryA(systemDir, MAX_PATH);
        _snprintf_s(realPath, sizeof(realPath), _TRUNCATE, "%s\\d3d9.dll", systemDir);
        g_hRealD3D9 = LoadLibraryA(realPath);
        if (!g_hRealD3D9) {
            Log_Printf("RealD3D9_Init: LoadLibraryA(%s) failed (err=%lu)", realPath, GetLastError());
            return false;
        }
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
    // The probe is only made for dgVoodoo2. On the system-d3d9 route the
    // system DLL's own entry points are hooked instead (see
    // HookSystemCreateFunctions), so no D3D object is created under the
    // loader lock at all.
    if (!usingDgVoodoo) {
        HookSystemCreateFunctions();
        return true;
    }

    IDirect3D9Ex* pProbe = nullptr;
    HRESULT probeHr = g_RealDirect3DCreate9Ex(D3D_SDK_VERSION, &pProbe);
    if (SUCCEEDED(probeHr) && pProbe) {
        Log_Printf("RealD3D9_Init: bootstrap probe IDirect3D9Ex created, installing CreateDevice hook");
        Hooks_OnD3D9ExCreated(pProbe);
        // Deliberately never released (Linux/Proton, 2026-09-11). There,
        // dgVoodoo creates its D3D12 device while making this probe, and the
        // game's own IDirect3D9 shares it:
        //  - releasing it here, inside DllMain, tears that device down while
        //    vkd3d-proton's worker threads wait on the loader lock this
        //    thread holds - the game hung at launch;
        //  - releasing it later, just before the game's CreateDevice, left
        //    dgVoodoo reading freed memory inside CreateDevice - a crash.
        // Keeping it costs one small object. On Windows this doesn't come up:
        // the probe fails outright there (E_FAIL, 11:24 log) and the hook is
        // installed when the game calls our Direct3DCreate9 export instead.
        g_bootstrapProbe = pProbe;
    } else {
        Log_Printf("RealD3D9_Init: bootstrap Direct3DCreate9Ex failed (hr=0x%08lX)", probeHr);
    }

    return true;
}

extern "C" IDirect3D9* WINAPI Direct3DCreate9(UINT SDKVersion)
{
    Log_Printf("Direct3DCreate9(SDKVersion=%u) called", SDKVersion);

    CreateGuard guard;

    LogLoadedModulesOnce();

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

    CreateGuard guard;

    LogLoadedModulesOnce();

    if (!RealD3D9_Init())
        return E_FAIL;

    HRESULT hr = g_RealDirect3DCreate9Ex(SDKVersion, ppD3D);
    if (SUCCEEDED(hr) && ppD3D && *ppD3D)
        Hooks_OnD3D9ExCreated(*ppD3D);

    return hr;
}

bool RealD3D9_UsingDgVoodoo()
{
    return g_usingDgVoodoo;
}
