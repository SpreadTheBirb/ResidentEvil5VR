#include "query_probe.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

// ---- Why (2026-09-11) --------------------------------------------------
// VR shows holes where geometry is missing - level geometry and even Sheva
// - as the head turns. Forcing the model sphere test (+435A90, see
// culling_patch.cpp) visible didn't help, a live watch showed the main
// camera's frustum planes are read by nothing else, and a memory scan found
// only dead copies of them (no reader in 10 s). What this probe found
// instead (07:48): the game creates 64 D3DQUERYTYPE_OCCLUSION queries at
// level load - hardware OCCLUSION CULLING. It draws cheap proxies inside a
// query, asks the GPU how many pixels passed, and skips objects that came
// back hidden. The game camera's judgement of "hidden" is wrong for what
// the eyes see once the head turns, which punches exactly these holes.
//
// The fix: while VR is on, every occlusion query result reads as "plenty
// of pixels visible". Hooked at IDirect3DQuery9::GetData (vtable slot 7)
// of the query class, taken from the first occlusion query the game
// creates; other query types (the game also creates 3 EVENT queries) and
// flat screen pass through untouched. Possible side effect: if any of the
// queries drive lens flares or sun glare, those could show through walls
// in VR.

namespace {

// Stable IDirect3DDevice9 vtable slot (last method of the interface).
constexpr size_t kIDirect3DDevice9_CreateQuery = 118;
// IDirect3DQuery9: QueryInterface, AddRef, Release, GetDevice, GetType,
// GetDataSize, Issue, GetData.
constexpr size_t kIDirect3DQuery9_GetData = 7;
constexpr int kMaxQueryType = 16;
constexpr DWORD kForcedVisiblePixels = 0x00100000; // far above any "is it visible" threshold

typedef HRESULT(WINAPI* CreateQuery_t)(IDirect3DDevice9* This, D3DQUERYTYPE Type, IDirect3DQuery9** ppQuery);
CreateQuery_t g_origCreateQuery = nullptr;

typedef HRESULT(WINAPI* GetData_t)(IDirect3DQuery9* This, void* pData, DWORD dwSize, DWORD dwGetDataFlags);
GetData_t g_origGetData = nullptr;
volatile LONG g_getDataHooked = 0;

volatile LONG g_created[kMaxQueryType];
volatile LONG g_supportChecks = 0; // CreateQuery with ppQuery == nullptr (asks "is this type supported?")
LONG g_reported[kMaxQueryType];
LONG g_reportedChecks = 0;
unsigned long long g_lastCheckMs = 0;

volatile LONG g_occlusionReads = 0;
volatile LONG g_occlusionForced = 0; // reads that came back hidden (0 pixels) or not ready
unsigned long long g_lastOverrideReportMs = 0;

const char* QueryTypeName(int type)
{
    switch (type) {
    case D3DQUERYTYPE_VCACHE: return "VCACHE";
    case D3DQUERYTYPE_RESOURCEMANAGER: return "RESOURCEMANAGER";
    case D3DQUERYTYPE_VERTEXSTATS: return "VERTEXSTATS";
    case D3DQUERYTYPE_EVENT: return "EVENT";
    case D3DQUERYTYPE_OCCLUSION: return "OCCLUSION";
    case D3DQUERYTYPE_TIMESTAMP: return "TIMESTAMP";
    case D3DQUERYTYPE_TIMESTAMPDISJOINT: return "TIMESTAMPDISJOINT";
    case D3DQUERYTYPE_TIMESTAMPFREQ: return "TIMESTAMPFREQ";
    default: return "other";
    }
}

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

HRESULT WINAPI hkGetData(IDirect3DQuery9* This, void* pData, DWORD dwSize, DWORD dwGetDataFlags)
{
    const HRESULT hr = g_origGetData(This, pData, dwSize, dwGetDataFlags);
    if (!CameraRigHook_IsVrActive() || This->GetType() != D3DQUERYTYPE_OCCLUSION || !pData || dwSize < sizeof(DWORD))
        return hr;
    InterlockedIncrement(&g_occlusionReads);
    DWORD* pixels = static_cast<DWORD*>(pData);
    if (hr != S_OK || *pixels == 0)
        InterlockedIncrement(&g_occlusionForced);
    *pixels = kForcedVisiblePixels;
    return S_OK;
}

// Hooks GetData of the query implementation class, once, from the first
// occlusion query the game creates.
void HookGetDataFrom(IDirect3DQuery9* query)
{
    if (InterlockedCompareExchange(&g_getDataHooked, 1, 0) != 0)
        return;
    void* target = VTableEntry(query, kIDirect3DQuery9_GetData);
    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&hkGetData), reinterpret_cast<void**>(&g_origGetData));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("QueryProbe: MH_CreateHook(IDirect3DQuery9::GetData) failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("QueryProbe: occlusion GetData hook enabled -> %d (target=%p) - occlusion culling off while VR is on",
        static_cast<int>(st), target);
}

HRESULT WINAPI hkCreateQuery(IDirect3DDevice9* This, D3DQUERYTYPE Type, IDirect3DQuery9** ppQuery)
{
    const HRESULT hr = g_origCreateQuery(This, Type, ppQuery);
    const int type = static_cast<int>(Type);
    if (!ppQuery)
        InterlockedIncrement(&g_supportChecks);
    else if (type >= 0 && type < kMaxQueryType)
        InterlockedIncrement(&g_created[type]);
    if (SUCCEEDED(hr) && ppQuery && *ppQuery && Type == D3DQUERYTYPE_OCCLUSION)
        HookGetDataFrom(*ppQuery);
    return hr;
}

} // namespace

void QueryProbe_Install(IDirect3DDevice9* pDevice)
{
    void* target = VTableEntry(pDevice, kIDirect3DDevice9_CreateQuery);
    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&hkCreateQuery), reinterpret_cast<void**>(&g_origCreateQuery));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("QueryProbe_Install: MH_CreateHook(CreateQuery) failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("QueryProbe_Install: CreateQuery hook enabled -> %d", static_cast<int>(st));
}

void QueryProbe_OnEndScene()
{
    const unsigned long long now = GetTickCount64();

    if (now - g_lastOverrideReportMs >= 5000) {
        g_lastOverrideReportMs = now;
        const LONG reads = InterlockedExchange(&g_occlusionReads, 0);
        const LONG forced = InterlockedExchange(&g_occlusionForced, 0);
        if (reads)
            Log_Printf("QueryProbe: last 5 s - %ld occlusion result(s) read in VR, %ld said hidden/not ready, all reported visible",
                reads, forced);
    }

    if (now - g_lastCheckMs < 10000)
        return;
    g_lastCheckMs = now;

    bool changed = g_supportChecks != g_reportedChecks;
    for (int i = 0; i < kMaxQueryType; ++i)
        changed |= g_created[i] != g_reported[i];
    if (!changed)
        return;

    g_reportedChecks = g_supportChecks;
    Log_Printf("QueryProbe: CreateQuery totals so far (%ld support check(s)):", g_reportedChecks);
    for (int i = 0; i < kMaxQueryType; ++i) {
        g_reported[i] = g_created[i];
        if (g_reported[i])
            Log_Printf("QueryProbe:   type %d %s: %ld created", i, QueryTypeName(i), g_reported[i]);
    }
}
