#include "hud_shaders.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <vector>

// ---- Which shaders are the HUD (2026-09-13) -----------------------------
// The HUD draws through a shader that ignores c0-c3, so stereo mistook it for
// the 3D scene and cut it in half per eye; drawing those draws through a
// per-eye viewport instead fixes the HUD, the inventory, the pause screen and
// most menus (stereo_test.cpp, DrawHudPerEye). What was missing was knowing
// WHICH draws, without the developer K capture.
//
// Shader pointers change every launch; their bytecode does not. The K capture
// (hud_probe.cpp) logged FNV-1a hashes of the bytecode for the shader pairs
// that vanish when the HUD is switched off. Those hashes are listed below.
// Each shader is hashed exactly once, when the game CREATES it, and tagged if
// it matches - so a draw only costs two pointer comparisons, and a pointer
// that is freed and reused for a different shader is re-tagged correctly,
// because creating that shader runs the check again.
//
// Measured 2026-09-13 (one session, then confirmed working by pointer): the
// main HUD is vs 18A5AA2B + ps 781175EA (16 draws -> 0 with the HUD off); a
// second HUD vertex shader 1D73ACCB shares that pixel shader (5 -> 2). If a
// future capture finds more 2D shaders - menu layers, say - they go here.

namespace {

constexpr unsigned long kHudVertexShaderHashes[] = { 0x18A5AA2Bul, 0x1D73ACCBul };
constexpr unsigned long kHudPixelShaderHashes[] = { 0x781175EAul };

constexpr size_t kIDirect3DDevice9_CreateVertexShader = 91;
constexpr size_t kIDirect3DDevice9_CreatePixelShader = 106;

std::vector<void*> g_hudVs, g_hudPs;
SRWLOCK g_lock = SRWLOCK_INIT; // shaders can be created off the render thread
unsigned g_vsSeen = 0, g_psSeen = 0;

// Same FNV-1a over GetFunction's bytes as the K capture, so the numbers agree.
template <typename Shader>
unsigned long HashShader(Shader* s)
{
    UINT size = 0;
    if (!s || FAILED(s->GetFunction(nullptr, &size)) || !size || size > (1u << 20))
        return 0;
    std::vector<BYTE> code(size);
    if (FAILED(s->GetFunction(code.data(), &size)))
        return 0;
    unsigned long h = 2166136261ul;
    for (BYTE b : code) {
        h ^= b;
        h *= 16777619ul;
    }
    return h;
}

bool IsListed(unsigned long hash, const unsigned long* list, size_t n)
{
    for (size_t i = 0; i < n; ++i)
        if (list[i] == hash)
            return true;
    return false;
}

// Tags or un-tags a freshly created shader pointer.
void Note(std::vector<void*>& set, void* shader, bool isHud)
{
    AcquireSRWLockExclusive(&g_lock);
    for (size_t i = 0; i < set.size(); ++i) {
        if (set[i] == shader) {
            if (!isHud)
                set.erase(set.begin() + i); // pointer reused by a different shader
            ReleaseSRWLockExclusive(&g_lock);
            return;
        }
    }
    if (isHud)
        set.push_back(shader);
    ReleaseSRWLockExclusive(&g_lock);
}

bool Contains(const std::vector<void*>& set, void* shader)
{
    for (void* p : set)
        if (p == shader)
            return true;
    return false;
}

typedef HRESULT(WINAPI* CreateVertexShader_t)(IDirect3DDevice9*, const DWORD*, IDirect3DVertexShader9**);
typedef HRESULT(WINAPI* CreatePixelShader_t)(IDirect3DDevice9*, const DWORD*, IDirect3DPixelShader9**);
CreateVertexShader_t oCreateVertexShader = nullptr;
CreatePixelShader_t oCreatePixelShader = nullptr;

HRESULT WINAPI hkCreateVertexShader(IDirect3DDevice9* This, const DWORD* pFunction, IDirect3DVertexShader9** ppShader)
{
    const HRESULT hr = oCreateVertexShader(This, pFunction, ppShader);
    if (SUCCEEDED(hr) && ppShader && *ppShader) {
        ++g_vsSeen;
        const unsigned long h = HashShader(*ppShader);
        const bool hud = IsListed(h, kHudVertexShaderHashes, _countof(kHudVertexShaderHashes));
        Note(g_hudVs, *ppShader, hud);
        if (hud)
            Log_Printf("HudShaders: HUD vertex shader recognised (hash %08lX) -> %p", h, *ppShader);
    }
    return hr;
}

HRESULT WINAPI hkCreatePixelShader(IDirect3DDevice9* This, const DWORD* pFunction, IDirect3DPixelShader9** ppShader)
{
    const HRESULT hr = oCreatePixelShader(This, pFunction, ppShader);
    if (SUCCEEDED(hr) && ppShader && *ppShader) {
        ++g_psSeen;
        const unsigned long h = HashShader(*ppShader);
        const bool hud = IsListed(h, kHudPixelShaderHashes, _countof(kHudPixelShaderHashes));
        Note(g_hudPs, *ppShader, hud);
        if (hud)
            Log_Printf("HudShaders: HUD pixel shader recognised (hash %08lX) -> %p", h, *ppShader);
    }
    return hr;
}

void* VTableEntry(void* pInterface, size_t index)
{
    return (*reinterpret_cast<void***>(pInterface))[index];
}

} // namespace

void HudShaders_Install(IDirect3DDevice9* device)
{
    MH_STATUS st = MH_CreateHook(VTableEntry(device, kIDirect3DDevice9_CreateVertexShader),
        reinterpret_cast<void*>(&hkCreateVertexShader), reinterpret_cast<void**>(&oCreateVertexShader));
    if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
        st = MH_EnableHook(VTableEntry(device, kIDirect3DDevice9_CreateVertexShader));
    Log_Printf("HudShaders_Install: CreateVertexShader hook -> %d", static_cast<int>(st));

    st = MH_CreateHook(VTableEntry(device, kIDirect3DDevice9_CreatePixelShader),
        reinterpret_cast<void*>(&hkCreatePixelShader), reinterpret_cast<void**>(&oCreatePixelShader));
    if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
        st = MH_EnableHook(VTableEntry(device, kIDirect3DDevice9_CreatePixelShader));
    Log_Printf("HudShaders_Install: CreatePixelShader hook -> %d", static_cast<int>(st));
}

bool HudShaders_IsHudDraw(IDirect3DDevice9* device)
{
    // Cheap exit before any COM calls: nothing recognised yet (early startup,
    // or a game build whose shaders do not match).
    AcquireSRWLockShared(&g_lock);
    const bool anyKnown = !g_hudVs.empty() && !g_hudPs.empty();
    ReleaseSRWLockShared(&g_lock);
    if (!anyKnown)
        return false;

    IDirect3DVertexShader9* vs = nullptr;
    IDirect3DPixelShader9* ps = nullptr;
    device->GetVertexShader(&vs);
    device->GetPixelShader(&ps);
    if (vs)
        vs->Release();
    if (ps)
        ps->Release();
    if (!vs || !ps)
        return false;

    AcquireSRWLockShared(&g_lock);
    const bool hud = Contains(g_hudVs, vs) && Contains(g_hudPs, ps);
    ReleaseSRWLockShared(&g_lock);
    return hud;
}

void HudShaders_GetCounts(int* hudVertexShaders, int* hudPixelShaders, unsigned* shadersSeen)
{
    AcquireSRWLockShared(&g_lock);
    *hudVertexShaders = static_cast<int>(g_hudVs.size());
    *hudPixelShaders = static_cast<int>(g_hudPs.size());
    *shadersSeen = g_vsSeen + g_psSeen;
    ReleaseSRWLockShared(&g_lock);
}
