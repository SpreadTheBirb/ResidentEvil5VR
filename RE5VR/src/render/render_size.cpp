#include "render_size.h"
#include "../util/log.h"

#include <windows.h>
#include <dxgi1_4.h>

#include <algorithm>
#include <cmath>
#include <cstring>

// ---- How RE5 is told to render at another size -------------------------------
// Found 2026-09-14 by disassembly, with the addresses confirmed live:
//
// Forcing only the backbuffer does nothing useful: RE5 builds every scene
// target from its own render size and copies the result into the corner of a
// bigger frame. Its resolution lives in the render object behind the global
// at exe+E345D4 (0x012345D4):
//
//   +40/+44/+48/+4C  where the image sits in the frame (left, top, right,
//                    bottom). The letterbox pass (0x45A190) clears around it,
//                    and the final copy to the frame is placed by it.
//   +50/+54          render size. Every scene target is created from it
//                    (0x458A30).
//   +17500/+17504    frame size, used for the present parameters on Reset.
//   +196CC           flags; bit 4 means "display mode changed, reset next
//                    frame".
//
// RE5's own resolution setter (0x454CB8) writes +50/+54 and +17500/+17504
// and sets bit 4; the game then resets the device on its next frame and
// rebuilds everything at the new size by itself. Doing exactly the same from
// here - plus the image rectangle, which the setter leaves to other code -
// switches it at runtime with no restart and no config edit. All writes are
// in-process.
//
// The player's own values are captured before the first write and put back
// when VR turns off. Nothing is written unless the code at the setter matches
// byte for byte and the object's sizes agree with the real backbuffer.

namespace {

constexpr DWORD kRenderGlobal = 0x012345D4;
constexpr DWORD kSetterAddr = 0x00454CBC;
// or [esi+196CC],4 / mov [esi+17500],eax / mov eax,[esp+1C] / mov [esi+50],ebx
// mov [esi+54],edx / mov [esi+17504],eax
constexpr BYTE kSetterBytes[] = { 0x83, 0x8E, 0xCC, 0x96, 0x01, 0x00, 0x04, 0x89, 0x86, 0x00, 0x75, 0x01, 0x00, 0x8B,
    0x44, 0x24, 0x1C, 0x89, 0x5E, 0x50, 0x89, 0x56, 0x54, 0x89, 0x86, 0x04, 0x75, 0x01, 0x00 };

constexpr DWORD kOffImageRect = 0x40;
constexpr DWORD kOffRenderSize = 0x50;
constexpr DWORD kOffFrameSize = 0x17500;
constexpr DWORD kOffFlags = 0x196CC;
constexpr DWORD kFlagModeChanged = 4;

constexpr unsigned long long kSwitchTimeoutMs = 5000;

struct Sizes {
    int imageRect[4];
    int render[2];
    int frame[2];
};

enum class State { Idle, Pending, Active };

RenderSizeSettings g_settings;
State g_state = State::Idle;
bool g_checked = false, g_available = false;
bool g_failed = false;     // RE5 refused the size, or a write failed: not again this session
UINT g_targetW = 0, g_targetH = 0;           // whole frame while in VR
bool g_capped = false;
Sizes g_game = {};                           // the player's own values
bool g_haveGame = false;
unsigned long long g_requestMs = 0;
unsigned long long g_lastGuardMs = 0;

bool SafeRead(DWORD addr, void* dst, size_t n)
{
    __try {
        memcpy(dst, reinterpret_cast<void*>(static_cast<ULONG_PTR>(addr)), n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool SafeWrite(DWORD addr, const void* src, size_t n)
{
    __try {
        memcpy(reinterpret_cast<void*>(static_cast<ULONG_PTR>(addr)), src, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

DWORD RenderObject()
{
    DWORD obj = 0;
    return SafeRead(kRenderGlobal, &obj, 4) ? obj : 0;
}

bool ReadSizes(DWORD obj, Sizes& s)
{
    return SafeRead(obj + kOffImageRect, s.imageRect, sizeof(s.imageRect)) &&
        SafeRead(obj + kOffRenderSize, s.render, sizeof(s.render)) &&
        SafeRead(obj + kOffFrameSize, s.frame, sizeof(s.frame));
}

// Writes the sizes and raises the "mode changed" flag, the way RE5's setter does.
bool WriteSizes(DWORD obj, const Sizes& s)
{
    DWORD flags = 0;
    if (!SafeRead(obj + kOffFlags, &flags, 4))
        return false;
    flags |= kFlagModeChanged;
    return SafeWrite(obj + kOffFrameSize, s.frame, sizeof(s.frame)) &&
        SafeWrite(obj + kOffRenderSize, s.render, sizeof(s.render)) &&
        SafeWrite(obj + kOffImageRect, s.imageRect, sizeof(s.imageRect)) && SafeWrite(obj + kOffFlags, &flags, 4);
}

Sizes VrSizes()
{
    const int w = static_cast<int>(g_targetW), h = static_cast<int>(g_targetH);
    return Sizes{ { 0, 0, w, h }, { w, h }, { w, h } };
}

bool BackbufferSize(IDirect3DDevice9* device, UINT* w, UINT* h)
{
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return false;
    D3DSURFACE_DESC d = {};
    const bool ok = SUCCEEDED(bb->GetDesc(&d));
    bb->Release();
    *w = d.Width;
    *h = d.Height;
    return ok;
}

bool CheckAvailable(IDirect3DDevice9* device)
{
    if (g_checked)
        return g_available;
    g_checked = true;
    BYTE code[sizeof(kSetterBytes)] = {};
    if (!SafeRead(kSetterAddr, code, sizeof(code)) || memcmp(code, kSetterBytes, sizeof(code)) != 0) {
        Log_Printf("RenderSize: RE5's resolution setter isn't where this build expects - full resolution per eye "
                   "is off (each eye gets half the game's frame)");
        return false;
    }
    const DWORD obj = RenderObject();
    Sizes s = {};
    UINT bbW = 0, bbH = 0;
    if (!obj || !ReadSizes(obj, s) || !BackbufferSize(device, &bbW, &bbH) || s.render[0] != static_cast<int>(bbW) ||
        s.render[1] != static_cast<int>(bbH) || s.frame[0] != static_cast<int>(bbW) ||
        s.frame[1] != static_cast<int>(bbH)) {
        Log_Printf("RenderSize: render object %08lX doesn't agree with the %ux%u backbuffer (render %dx%d, frame %dx%d) - "
                   "full resolution per eye is off",
            obj, bbW, bbH, s.render[0], s.render[1], s.frame[0], s.frame[1]);
        return false;
    }
    g_available = true;
    return true;
}

// ---- Cutscene aspect crash (from RE5Fix by Lyall, MIT) -----------------------
// RE5Fix found that cutscenes crash above about 1.8:1, and fixes it by writing
// the frame's height/width over RE5's 9:16 constant (pattern 00 00 10 3F AC).
// Two eyes side by side is 1.8:1 or wider for most headsets, so the same fix
// applies while the VR size is active and is undone after.
BYTE* g_cutsceneAspect = nullptr;
bool g_cutsceneAspectSearched = false;
constexpr float kNativeInvAspect = 0.5625f;

BYTE* FindCutsceneAspect()
{
    const HMODULE exe = GetModuleHandleA(nullptr);
    const auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(exe);
    const auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(reinterpret_cast<BYTE*>(exe) + dos->e_lfanew);
    BYTE* p = reinterpret_cast<BYTE*>(exe);
    const BYTE* end = p + nt->OptionalHeader.SizeOfImage;
    constexpr BYTE kPattern[] = { 0x00, 0x00, 0x10, 0x3F, 0xAC };
    while (p < end) {
        MEMORY_BASIC_INFORMATION mbi = {};
        if (!VirtualQuery(p, &mbi, sizeof(mbi)))
            break;
        BYTE* rb = static_cast<BYTE*>(mbi.BaseAddress);
        BYTE* re = (std::min)(rb + mbi.RegionSize, const_cast<BYTE*>(end));
        const bool readable = mbi.State == MEM_COMMIT && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS));
        if (readable) {
            for (BYTE* q = (std::max)(rb, p); q + sizeof(kPattern) <= re; ++q)
                if (memcmp(q, kPattern, sizeof(kPattern)) == 0)
                    return q;
        }
        p = re;
    }
    return nullptr;
}

void SetCutsceneInvAspect(float value)
{
    if (!g_cutsceneAspectSearched) {
        g_cutsceneAspectSearched = true;
        g_cutsceneAspect = FindCutsceneAspect();
        Log_Printf("RenderSize: cutscene aspect constant %s", g_cutsceneAspect ? "found" : "not found (fix skipped)");
    }
    if (!g_cutsceneAspect)
        return;
    float cur = 0;
    memcpy(&cur, g_cutsceneAspect, 4);
    if (cur == value)
        return;
    DWORD old = 0;
    if (VirtualProtect(g_cutsceneAspect, 4, PAGE_READWRITE, &old)) {
        memcpy(g_cutsceneAspect, &value, 4);
        VirtualProtect(g_cutsceneAspect, 4, old, &old);
        Log_Printf("RenderSize: cutscene aspect %.4f -> %.4f", cur, value);
    }
}

void ApplyCutsceneFix(bool vrSize)
{
    const float aspect = g_targetH ? static_cast<float>(g_targetW) / static_cast<float>(g_targetH) : 0.0f;
    if (vrSize && aspect > 1.8f)
        SetCutsceneInvAspect(1.0f / aspect);
    else if (g_cutsceneAspectSearched)
        SetCutsceneInvAspect(kNativeInvAspect);
}

void Restore(const char* why)
{
    const DWORD obj = RenderObject();
    if (obj && g_haveGame) {
        const bool ok = WriteSizes(obj, g_game);
        Log_Printf("RenderSize: %s - back to the game's own %dx%d (%s)", why, g_game.render[0], g_game.render[1],
            ok ? "requested" : "WRITE FAILED");
    }
    ApplyCutsceneFix(false);
    g_state = State::Idle;
}

} // namespace

RenderSizeSettings RenderSize_GetSettings()
{
    return g_settings;
}

void RenderSize_ApplySettings(const RenderSizeSettings& in)
{
    if (in.fullResPerEye != g_settings.fullResPerEye)
        Log_Printf("RenderSize: full resolution per eye %s (applies the next time VR is switched on)",
            in.fullResPerEye ? "ON" : "OFF");
    g_settings = in;
}

namespace {

// ---- Memory cap ---------------------------------------------------------------
// The limit is dgVoodoo's, not the GPU's. dgVoodoo presents RE5 with a virtual
// card whose memory is its config's [DirectX] VRAM, and refuses allocations
// past it; RE5 then stops with "ERR08 : Memory overrun" (its name for a D3D9
// out-of-memory result). The VR package shipped with VRAM = 256, which is
// why every size over ~5 MP died while the real GPU and the address space
// were nowhere near full. Measured 2026-09-14 (Quadro M2200, VRAM = 2048):
//   1280x720  - 1950 MB of texture memory still available
//   6144x3264 - 1377 MB still available, and it played
// i.e. about 30 bytes of dgVoodoo memory per extra pixel. With 256 MB only
// ~158 MB is free at 1280x720, room for ~5 MP - exactly where it failed.
//
// So the cap is read from dgVoodoo itself: the texture memory RE5's device
// reports free at the game's own size, less a margin, at 32 bytes a pixel.
// A player still on a 256 MB config gets a safe ~4 MP; 2048 MB covers every
// current headset preset. Without the 4GB patch the address space is the
// tighter risk, so that stays at 5 MP whatever dgVoodoo allows.
bool ExeIsLargeAddressAware();

constexpr double kBytesPerPixel = 32.0;     // measured ~30
constexpr double kTextureMemoryMargin = 0.8; // keep a fifth free for level textures
constexpr double kMaxPixelsFallback = 5.0e6; // before dgVoodoo's memory has been read
constexpr double kMaxPixelsMinimum = 2.0e6;
constexpr double kMaxPixelsPlain = 5.0e6;    // no 4GB patch

// Texture memory free at the game's OWN size, refreshed while VR is off (at
// the VR size it would already include the big targets).
UINT g_freeTextureMemMb = 0;
unsigned long long g_lastFreeMemReadMs = 0;

void ReadFreeTextureMemory(IDirect3DDevice9* device)
{
    const UINT mb = device->GetAvailableTextureMem() / 1048576u;
    if (mb != g_freeTextureMemMb)
        Log_Printf("RenderSize: dgVoodoo texture memory free at the game's own size: %u MB (VR limit %.1f MP)", mb,
            (std::max)(mb * 1048576.0 * kTextureMemoryMargin / kBytesPerPixel, kMaxPixelsMinimum) / 1.0e6);
    g_freeTextureMemMb = mb;
    g_lastFreeMemReadMs = GetTickCount64();
}

// Video memory this process is using on the biggest adapter, for the log.
double GpuMemoryUsedGb()
{
    double used = 0.0, bestBudget = 0.0;
    IDXGIFactory1* factory = nullptr;
    if (FAILED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&factory))) || !factory)
        return 0.0;
    for (UINT i = 0;; ++i) {
        IDXGIAdapter1* adapter = nullptr;
        if (factory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND)
            break;
        IDXGIAdapter3* adapter3 = nullptr;
        if (SUCCEEDED(adapter->QueryInterface(__uuidof(IDXGIAdapter3), reinterpret_cast<void**>(&adapter3)))) {
            DXGI_QUERY_VIDEO_MEMORY_INFO info = {};
            DXGI_QUERY_VIDEO_MEMORY_INFO nonLocal = {};
            const bool discrete =
                SUCCEEDED(adapter3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &nonLocal)) &&
                nonLocal.Budget > 0;
            if (discrete && SUCCEEDED(adapter3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &info)) &&
                static_cast<double>(info.Budget) > bestBudget) {
                bestBudget = static_cast<double>(info.Budget);
                used = static_cast<double>(info.CurrentUsage) / 1073741824.0;
            }
            adapter3->Release();
        }
        adapter->Release();
    }
    factory->Release();
    return used;
}

double MaxPixels()
{
    double limit = kMaxPixelsFallback;
    if (g_freeTextureMemMb)
        limit = (std::max)(g_freeTextureMemMb * 1048576.0 * kTextureMemoryMargin / kBytesPerPixel, kMaxPixelsMinimum);
    if (!ExeIsLargeAddressAware())
        limit = (std::min)(limit, kMaxPixelsPlain);
    return limit;
}

bool ExeIsLargeAddressAware()
{
    const auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(GetModuleHandleA(nullptr));
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(reinterpret_cast<const BYTE*>(dos) + dos->e_lfanew);
    return (nt->FileHeader.Characteristics & IMAGE_FILE_LARGE_ADDRESS_AWARE) != 0;
}

double AddressSpaceUsedGb()
{
    MEMORYSTATUSEX mem = {};
    mem.dwLength = sizeof(mem);
    if (!GlobalMemoryStatusEx(&mem))
        return 0.0;
    return static_cast<double>(mem.ullTotalVirtual - mem.ullAvailVirtual) / 1073741824.0;
}

} // namespace

bool RenderSize_EyeSizeFor(UINT recW, UINT recH, UINT* outW, UINT* outH)
{
    const double maxPixels = MaxPixels();
    const double pixels = 2.0 * recW * recH;
    double k = pixels > maxPixels ? std::sqrt(maxPixels / pixels) : 1.0;
    // Largest texture side a D3D9-era device is expected to take; the two eyes
    // side by side make the frame twice as wide. 6144x3264 is confirmed to work.
    constexpr double kMaxFrameSide = 8192.0;
    k = (std::min)(k, kMaxFrameSide / (2.0 * recW));
    k = (std::min)(k, kMaxFrameSide / recH);
    // Eye width a multiple of 4 and height of 8: RE5 builds quarter-size
    // targets, and whole numbers keep the two halves exactly even. Rounded
    // down, so the cap really is a cap.
    UINT w = static_cast<UINT>(recW * k / 4.0) * 4;
    UINT h = static_cast<UINT>(recH * k / 8.0) * 8;
    w = (std::min)((std::max)(w, 320u), 4096u);
    h = (std::min)((std::max)(h, 320u), 8192u);
    *outW = w;
    *outH = h;
    return k < 1.0;
}

bool RenderSize_EnterVR(IDirect3DDevice9* device, UINT recW, UINT recH)
{
    if (g_state == State::Active)
        return true;
    if (!g_settings.fullResPerEye)
        return true;
    if (g_failed || !recW || !recH || !CheckAvailable(device))
        return true;

    const unsigned long long now = GetTickCount64();
    const DWORD obj = RenderObject();
    if (!obj)
        return true;

    if (g_state == State::Idle) {
        ReadFreeTextureMemory(device); // still at the game's own size here
        UINT eyeW = 0, eyeH = 0;
        g_capped = RenderSize_EyeSizeFor(recW, recH, &eyeW, &eyeH);
        g_targetW = eyeW * 2;
        g_targetH = eyeH;

        Sizes cur = {};
        if (!ReadSizes(obj, cur))
            return true;
        if (cur.render[0] == static_cast<int>(g_targetW) && cur.render[1] == static_cast<int>(g_targetH)) {
            g_state = State::Active;
            ApplyCutsceneFix(true);
            return true;
        }
        g_game = cur;
        g_haveGame = true;
        if (!WriteSizes(obj, VrSizes())) {
            Log_Printf("RenderSize: write failed - staying at the game's own size");
            g_failed = true;
            return true;
        }
        Log_Printf("RenderSize: VR on - asked RE5 for %ux%u (each eye %ux%u; the runtime recommends %ux%u%s); "
                   "the game's own is %dx%d; limit %.1f MP; address space in use %.2f GB, video memory %.2f GB",
            g_targetW, g_targetH, eyeW, eyeH, recW, recH, g_capped ? ", capped for memory" : "", cur.render[0],
            cur.render[1], MaxPixels() / 1.0e6, AddressSpaceUsedGb(), GpuMemoryUsedGb());
        Log_Printf("RenderSize: dgVoodoo reports %u MB of texture memory still available", device->GetAvailableTextureMem() / 1048576u);
        g_state = State::Pending;
        g_requestMs = now;
        return false;
    }

    // Pending: done once the real backbuffer has the new size.
    UINT bbW = 0, bbH = 0;
    if (BackbufferSize(device, &bbW, &bbH) && bbW == g_targetW && bbH == g_targetH) {
        g_state = State::Active;
        ApplyCutsceneFix(true);
        Log_Printf("RenderSize: RE5 is rendering at %ux%u after %llu ms; address space in use %.2f GB, video "
                   "memory %.2f GB",
            bbW, bbH, now - g_requestMs, AddressSpaceUsedGb(), GpuMemoryUsedGb());
        Log_Printf("RenderSize: dgVoodoo reports %u MB of texture memory still available", device->GetAvailableTextureMem() / 1048576u);
        return true;
    }
    if (now - g_requestMs > kSwitchTimeoutMs) {
        Log_Printf("RenderSize: RE5 didn't switch to %ux%u within %llu ms (backbuffer %ux%u) - giving up for this "
                   "session",
            g_targetW, g_targetH, kSwitchTimeoutMs, bbW, bbH);
        g_failed = true;
        Restore("gave up");
        return true;
    }
    return false;
}

void RenderSize_ExitVR()
{
    if (g_state != State::Idle)
        Restore("VR off");
}

void RenderSize_OnPresent(IDirect3DDevice9* device)
{
    if (g_state == State::Idle && GetTickCount64() - g_lastFreeMemReadMs > 5000)
        ReadFreeTextureMemory(device);
    if (g_state != State::Active)
        return;
    // Once, a few seconds in (after RE5 has reloaded what it needs at the new
    // size): how much of dgVoodoo's texture memory is left - what sizes the cap.
    static unsigned long long s_activeSinceMs = 0;
    static bool s_loggedSettled = false;
    if (!s_activeSinceMs)
        s_activeSinceMs = GetTickCount64();
    if (!s_loggedSettled && GetTickCount64() - s_activeSinceMs > 10000) {
        s_loggedSettled = true;
        Log_Printf("RenderSize: 10 s at %ux%u - dgVoodoo texture memory still available %u MB, video memory %.2f GB, "
                   "address space %.2f GB",
            g_targetW, g_targetH, device->GetAvailableTextureMem() / 1048576u, GpuMemoryUsedGb(), AddressSpaceUsedGb());
    }
    // RE5 can change its resolution by itself (its display options, a device
    // loss). Its new choice becomes the one to restore later, and the VR size
    // goes back in. Checked twice a second - it's a few memory reads.
    const unsigned long long now = GetTickCount64();
    if (now - g_lastGuardMs < 500)
        return;
    g_lastGuardMs = now;
    const DWORD obj = RenderObject();
    Sizes cur = {};
    if (!obj || !ReadSizes(obj, cur))
        return;
    if (cur.render[0] == static_cast<int>(g_targetW) && cur.render[1] == static_cast<int>(g_targetH) &&
        cur.frame[0] == static_cast<int>(g_targetW) && cur.frame[1] == static_cast<int>(g_targetH))
        return;
    g_game = cur;
    WriteSizes(obj, VrSizes());
    g_state = State::Pending;
    g_requestMs = now;
    Log_Printf("RenderSize: RE5 changed its resolution to %dx%d by itself - asking for %ux%u again", cur.render[0],
        cur.render[1], g_targetW, g_targetH);
}

void RenderSize_OnBeforeReset(D3DPRESENT_PARAMETERS* pp)
{
    // Exclusive fullscreen only takes frame sizes that are real display modes,
    // and a VR size never is: a tester in fullscreen got D3DERR_INVALIDCALL on
    // the switch and RE5 died (2026-09-14). A windowed frame can be any size,
    // and it is only the headset that sees it. The game's own fullscreen comes
    // back with its own size when VR turns off.
    if (!pp || pp->Windowed || g_state == State::Idle)
        return;
    if (pp->BackBufferWidth != g_targetW || pp->BackBufferHeight != g_targetH)
        return;
    pp->Windowed = TRUE;
    pp->FullScreen_RefreshRateInHz = 0;
    Log_Printf("RenderSize: RE5 is in exclusive fullscreen - the %ux%u VR frame is presented windowed instead",
        g_targetW, g_targetH);
}

void RenderSize_OnReset(HRESULT hr)
{
    if (SUCCEEDED(hr) || g_state == State::Idle)
        return;
    Log_Printf("RenderSize: Reset failed (hr=0x%08lX) while at or switching to %ux%u - putting the game's own size "
               "back and not trying again this session",
        hr, g_targetW, g_targetH);
    g_failed = true;
    Restore("Reset failed");
}

void RenderSize_GetStatus(RenderSizeStatus& out)
{
    out = RenderSizeStatus{};
    out.available = !g_checked || g_available;
    out.active = g_state == State::Active;
    out.pending = g_state == State::Pending;
    out.failed = g_failed;
    out.vrWidth = g_targetW;
    out.vrHeight = g_targetH;
    out.gameWidth = g_haveGame ? static_cast<UINT>(g_game.render[0]) : 0;
    out.gameHeight = g_haveGame ? static_cast<UINT>(g_game.render[1]) : 0;
    out.capped = g_capped;
}
