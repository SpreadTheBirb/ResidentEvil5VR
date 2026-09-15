#include "d3d12_addon_bridge.h"
#include "../util/log.h"

#include <d3d11_1.h> // ID3D11Device1::OpenSharedResource1 - needed for the D3D12 NT shared handle
#include <dxgi.h> // IDXGIDevice/IDXGIAdapter - adapter LUID diagnostic

namespace {

// Matches addon/src/addon_main.cpp's exported signatures exactly (plain
// C types on both sides - both DLLs are built with the same MSVC
// toolset/platform, so the ABI lines up without sharing headers).
using PFN_GetFrameInfo = bool(__cdecl*)(unsigned int* outWidth, unsigned int* outHeight, int* outDxgiFormat,
    void** outHandleSlot0, void** outHandleSlot1);
using PFN_GetFrontSlot = int(__cdecl*)();
using PFN_ReadSlot = void(__cdecl*)(int slot);
using PFN_IsSlotReady = bool(__cdecl*)(int slot);
using PFN_GetSharedGeneration = unsigned(__cdecl*)();

HMODULE g_addonModule = nullptr;
PFN_GetFrameInfo g_pGetFrameInfo = nullptr;
PFN_GetFrontSlot g_pGetFrontSlot = nullptr;
PFN_ReadSlot g_pBeginReadSlot = nullptr;
PFN_ReadSlot g_pEndReadSlot = nullptr;
PFN_IsSlotReady g_pIsSlotReady = nullptr;
PFN_GetSharedGeneration g_pGetSharedGeneration = nullptr; // optional: older addons lack it
unsigned g_openedGeneration = 0;

ID3D11Texture2D* g_fullFrameTex[2] = { nullptr, nullptr };
bool g_active = false;
UINT g_frameWidth = 0;
UINT g_frameHeight = 0;

// Tracks our own in-flight read of one addon slot across frames, so the
// slot can be released once the GPU has finished with it WITHOUT ever
// blocking to find out. g_pendingSlot is the slot still marked as being
// read; g_syncQuery is how we ask whether that read has landed.
ID3D11Query* g_syncQuery = nullptr;
int g_pendingSlot = -1;
bool g_readPending = false;

void ReleaseTextures()
{
    for (auto& tex : g_fullFrameTex) {
        if (tex) {
            tex->Release();
            tex = nullptr;
        }
    }
}

bool OpenBothSlots(ID3D11Device* d3d11Device, void* handle0, void* handle1)
{
    ID3D11Device1* d3d11Device1 = nullptr;
    HRESULT hr = d3d11Device->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&d3d11Device1));
    if (FAILED(hr)) {
        Log_Printf("D3D12AddonBridge: QueryInterface(ID3D11Device1) failed (hr=0x%08lX) - needed to open the D3D12 NT shared handle", hr);
        return false;
    }
    HANDLE handles[2] = { static_cast<HANDLE>(handle0), static_cast<HANDLE>(handle1) };
    for (int i = 0; i < 2; ++i) {
        hr = d3d11Device1->OpenSharedResource1(handles[i], __uuidof(ID3D11Texture2D),
            reinterpret_cast<void**>(&g_fullFrameTex[i]));
        if (FAILED(hr)) {
            Log_Printf("D3D12AddonBridge: OpenSharedResource1 (slot=%d, handle=%p) failed (hr=0x%08lX)", i, handles[i], hr);
            d3d11Device1->Release();
            ReleaseTextures();
            return false;
        }
    }
    d3d11Device1->Release();
    return true;
}

// ---- The game frame changing size mid-session (2026-09-14) -----------------
// Full resolution per eye (render/render_size.cpp) switches RE5's frame size
// when VR starts and stops, and the addon then recreates its shared textures
// with new handles. The textures opened here would otherwise keep showing the
// last frame of the old set forever. The addon counts its recreations; when
// the count moves, reopen.
bool RefreshSharedTextures(ID3D11DeviceContext* d3d11Context)
{
    if (!g_pGetSharedGeneration)
        return true;
    const unsigned gen = g_pGetSharedGeneration();
    if (gen == g_openedGeneration)
        return true;

    unsigned width = 0, height = 0;
    int dxgiFormat = 0;
    void* handle0 = nullptr;
    void* handle1 = nullptr;
    // Not ready, or recreated again while we asked: try on a later frame.
    if (!g_pGetFrameInfo(&width, &height, &dxgiFormat, &handle0, &handle1) || g_pGetSharedGeneration() != gen)
        return false;

    // Hand back our read of the old set; the copy holds its own references,
    // so it finishes safely either way.
    if (g_readPending && g_pEndReadSlot && g_pendingSlot >= 0)
        g_pEndReadSlot(g_pendingSlot);
    g_pendingSlot = -1;
    g_readPending = false;
    ReleaseTextures();

    ID3D11Device* device = nullptr;
    d3d11Context->GetDevice(&device);
    const bool ok = device && OpenBothSlots(device, handle0, handle1);
    if (device)
        device->Release();
    if (!ok) {
        static ULONGLONG s_lastFailMs = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_lastFailMs > 2000) {
            s_lastFailMs = now;
            Log_Printf("D3D12AddonBridge: reopening the addon's recreated textures (generation %u, %ux%u) failed", gen,
                width, height);
        }
        return false;
    }
    g_frameWidth = width;
    g_frameHeight = height;
    g_openedGeneration = gen;
    Log_Printf("D3D12AddonBridge: addon textures recreated - reopened at %ux%u (generation %u)", width, height, gen);
    return true;
}

} // namespace

bool D3D12AddonBridge_TryInit(ID3D11Device* d3d11Device, D3D12AddonBridgeInfo* outInfo)
{
    g_active = false;
    ReleaseTextures();

    // Don't LoadLibrary it ourselves - if dgVoodoo never loaded it (e.g.
    // OutputAPI isn't a D3D12 variant this run, or the addon simply
    // isn't present), there's nothing to bridge to and no reason to poll.
    g_addonModule = GetModuleHandleA("SampleAddon.dll");
    if (!g_addonModule) {
        Log_Printf("D3D12AddonBridge: SampleAddon.dll not loaded in this process - D3D9 CPU-readback path will be used instead");
        return false;
    }

    g_pGetFrameInfo = reinterpret_cast<PFN_GetFrameInfo>(GetProcAddress(g_addonModule, "RE5VRAddon_GetFrameInfo"));
    g_pGetFrontSlot = reinterpret_cast<PFN_GetFrontSlot>(GetProcAddress(g_addonModule, "RE5VRAddon_GetFrontSlot"));
    g_pBeginReadSlot = reinterpret_cast<PFN_ReadSlot>(GetProcAddress(g_addonModule, "RE5VRAddon_BeginReadSlot"));
    g_pEndReadSlot = reinterpret_cast<PFN_ReadSlot>(GetProcAddress(g_addonModule, "RE5VRAddon_EndReadSlot"));
    g_pIsSlotReady = reinterpret_cast<PFN_IsSlotReady>(GetProcAddress(g_addonModule, "RE5VRAddon_IsSlotReady"));
    g_pGetSharedGeneration =
        reinterpret_cast<PFN_GetSharedGeneration>(GetProcAddress(g_addonModule, "RE5VRAddon_GetSharedGeneration"));
    if (!g_pGetFrameInfo || !g_pGetFrontSlot || !g_pBeginReadSlot || !g_pEndReadSlot || !g_pIsSlotReady) {
        Log_Printf("D3D12AddonBridge: SampleAddon.dll is loaded but missing expected exports (GetFrameInfo=%p, GetFrontSlot=%p, BeginReadSlot=%p, EndReadSlot=%p, IsSlotReady=%p)",
            g_pGetFrameInfo, g_pGetFrontSlot, g_pBeginReadSlot, g_pEndReadSlot, g_pIsSlotReady);
        return false;
    }

    // The addon only has real frame info once it's copied at least one
    // frame - poll briefly rather than failing on the very first check,
    // since this runs right as XR mode is being enabled and the addon
    // may only just now be receiving its first PresentBegin calls.
    unsigned int width = 0, height = 0;
    int dxgiFormat = 0;
    void* handle0 = nullptr;
    void* handle1 = nullptr;
    bool ready = false;
    unsigned generationBefore = 0; // read before the info, so a race only costs a reopen
    for (int attempt = 0; attempt < 30 && !ready; ++attempt) {
        generationBefore = g_pGetSharedGeneration ? g_pGetSharedGeneration() : 0;
        ready = g_pGetFrameInfo(&width, &height, &dxgiFormat, &handle0, &handle1);
        if (!ready)
            Sleep(100);
    }
    if (!ready) {
        Log_Printf("D3D12AddonBridge: SampleAddon.dll loaded but never reported a ready frame after ~3s - falling back to D3D9 path");
        return false;
    }

    // Diagnostic: a shared-handle open across two different physical
    // adapters fails with exactly the E_INVALIDARG symptom seen here -
    // log this D3D11 device's own adapter LUID (compare against the
    // addon's D3D12BeginUsingAdapter log line) to confirm or rule this
    // out rather than guessing further.
    IDXGIDevice* dxgiDevice = nullptr;
    if (SUCCEEDED(d3d11Device->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice)))) {
        IDXGIAdapter* dxgiAdapter = nullptr;
        if (SUCCEEDED(dxgiDevice->GetAdapter(&dxgiAdapter))) {
            DXGI_ADAPTER_DESC adapterDesc = {};
            dxgiAdapter->GetDesc(&adapterDesc);
            Log_Printf("D3D12AddonBridge: this D3D11 device's adapter LUID = %08lX:%08lX (%ls)",
                adapterDesc.AdapterLuid.HighPart, adapterDesc.AdapterLuid.LowPart, adapterDesc.Description);
            dxgiAdapter->Release();
        }
        dxgiDevice->Release();
    }

    if (!OpenBothSlots(d3d11Device, handle0, handle1))
        return false;
    // If the addon recreated its textures while these were being opened, the
    // generation check in CopyToEyeSlots reopens them on the next frame.
    g_openedGeneration = generationBefore;

    g_frameWidth = width;
    g_frameHeight = height;
    g_active = true;

    if (outInfo) {
        outInfo->width = width;
        outInfo->height = height;
        outInfo->format = static_cast<DXGI_FORMAT>(dxgiFormat);
    }

    Log_Printf("D3D12AddonBridge: active - %ux%u format=%d, opened both shared textures", width, height, dxgiFormat);
    return true;
}

bool D3D12AddonBridge_IsActive()
{
    return g_active;
}

int D3D12AddonBridge_GetFrontSlot()
{
    // Copied first: the session teardown on the submit thread can clear the
    // pointer between the check and the call (the addon itself stays loaded).
    const PFN_GetFrontSlot getFrontSlot = g_pGetFrontSlot;
    if (!g_active || !getFrontSlot)
        return -1;
    const int slot = getFrontSlot();
    return (slot < 0 || slot > 1) ? -1 : slot;
}

namespace {

// Wall-clock milliseconds, high resolution.
double AddonNowMs()
{
    static LARGE_INTEGER freq = {};
    if (!freq.QuadPart)
        QueryPerformanceFrequency(&freq);
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
}

// Spin on a predicate until it comes true or the budget runs out. Yields the
// rest of the timeslice between polls rather than burning the core, and
// always returns - the deadline is the whole point.
template <typename Predicate>
bool SpinUntil(Predicate ready, double maxWaitMs)
{
    if (ready())
        return true;
    if (maxWaitMs <= 0.0)
        return false;
    const double deadline = AddonNowMs() + maxWaitMs;
    while (AddonNowMs() < deadline) {
        SwitchToThread();
        if (ready())
            return true;
    }
    return false;
}

} // namespace

bool D3D12AddonBridge_CopyToEyeSlots(ID3D11DeviceContext* d3d11Context,
    ID3D11Texture2D* leftDst, ID3D11Texture2D* rightDst, UINT eyeWidth, UINT eyeHeight,
    int* outSlot, double maxWaitMs, bool* outCopied)
{
    if (outCopied)
        *outCopied = false;
    if (!g_active || !g_pGetFrontSlot)
        return false;
    if (!RefreshSharedTextures(d3d11Context))
        return false;
    // The headset images keep their size for the whole session; a frame of
    // any other size (VR just switched off, or RE5 mid-switch) must not be
    // cut into eye boxes that don't fit it.
    if (eyeWidth * 2 > g_frameWidth || eyeHeight > g_frameHeight) {
        static ULONGLONG s_lastLogMs = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_lastLogMs > 5000) {
            s_lastLogMs = now;
            Log_Printf("D3D12AddonBridge_CopyToEyeSlots: frame is %ux%u, too small for two %ux%u eyes - skipping",
                g_frameWidth, g_frameHeight, eyeWidth, eyeHeight);
        }
        return false;
    }

    int frontSlot = g_pGetFrontSlot();
    if (frontSlot < 0 || frontSlot > 1)
        return false; // addon hasn't published a frame yet
    if (outSlot)
        *outSlot = frontSlot;

    ID3D11Texture2D* src = g_fullFrameTex[frontSlot];
    if (!src)
        return false;

    static UINT s_copyCount = 0;
    ++s_copyCount;
    const bool verboseLog = (s_copyCount <= 10 || (s_copyCount % 300) == 0);

    if (!g_syncQuery) {
        ID3D11Device* device = nullptr;
        d3d11Context->GetDevice(&device);
        if (device) {
            D3D11_QUERY_DESC qd = {};
            qd.Query = D3D11_QUERY_EVENT;
            const HRESULT qhr = device->CreateQuery(&qd, &g_syncQuery);
            if (FAILED(qhr))
                Log_Printf("D3D12AddonBridge: CreateQuery(EVENT) failed (hr=0x%08lX) - slots will be released immediately instead", qhr);
            device->Release();
        }
    }

    // --- Step 1: retire the PREVIOUS frame's read, without ever waiting.
    //
    // We told the addon "I'm reading slot N" last frame and must only
    // take that back once our own GPU-side copy out of it has genuinely
    // finished - a D3D11 immediate-context CopySubresourceRegion only
    // QUEUES work and returns long before the GPU is done. The previous
    // version of this code spin-waited right here on the game's main
    // thread until the query said done, which is precisely the kind of
    // unbounded blocking wait that the freeze investigation pointed at.
    // So: poll once, and if the GPU isn't finished yet, just leave the
    // slot marked and try again next frame. Nothing blocks, ever.
    if (g_readPending) {
        BOOL done = FALSE;
        // Deliberately NOT D3D11_ASYNC_GETDATA_DONOTFLUSH: that flag says
        // "answer without flushing", but our copies are sitting in an
        // unsubmitted command batch, so nothing would ever cause them to
        // execute and the query would essentially never report done. The
        // first version of this used DONOTFLUSH and the logs showed the
        // consequence plainly - ~300 of the first 333 frames skipped with
        // "previous read not finished", which in turn made the producer
        // skip almost every copy because the slot stayed locked.
        // Passing 0 lets GetData flush and then answer; it still returns
        // immediately either way, so this is a single poll, not a wait.
        HRESULT hr = S_OK; // no query available - treat as done, see step 3
        if (g_syncQuery) {
            SpinUntil(
                [&]() {
                    hr = d3d11Context->GetData(g_syncQuery, &done, sizeof(done), 0);
                    return hr == S_OK && done;
                },
                maxWaitMs);
        }

        if (hr == S_OK && done) {
            if (g_pEndReadSlot && g_pendingSlot >= 0)
                g_pEndReadSlot(g_pendingSlot);
            g_pendingSlot = -1;
            g_readPending = false;
        } else {
            // Still reading last frame's slot. Skip this update entirely;
            // the eye textures keep their previous (valid) contents.
            static UINT s_notDoneCount = 0;
            ++s_notDoneCount;
            if (s_notDoneCount <= 10 || (s_notDoneCount % 300) == 0)
                Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: previous read of slot=%d not finished on GPU yet, reusing last frame (skip #%u)",
                    s_copyCount, g_pendingSlot, s_notDoneCount);
            // Returns TRUE deliberately, even though nothing was copied. The
            // staged caller treats this as "the frame you already have is
            // still good", which is true - its eye textures keep valid
            // contents - and it goes on publishing and pose-tagging them.
            // Returning false here (tried 2026-09-12 for direct submit)
            // silently killed the tagging on the staged path: tags stopped
            // being written, the 16-entry pose ring wrapped past the last
            // one, and every submit fell back to a freshly located pose.
            // Visible in the cloud-PC log as hits frozen at 4845 while
            // fallbacks climbed 555 -> 1155 -> 1755, with image age 0.0.
            // This fires on roughly every other call, so it matters.
            return true;
        }
    }

    // --- Step 2: has the addon's own copy INTO this slot finished?
    //
    // The addon publishes a slot as soon as it SUBMITS the copy, not once
    // it completes, so this is the matching half of that contract: a
    // cheap fence GetCompletedValue() comparison on the addon's side. If
    // it isn't ready we reuse the previous frame rather than wait.
    // If the newest slot's GPU copy still hasn't landed, the OTHER slot holds
    // the previous frame and is complete by definition - one frame old beats
    // no frame at all. This matters enormously for the direct-submit caller:
    // there, "skip" means submitting a swapchain image nothing was written
    // into, and swapchain images cycle, so the headset shows a frame from two
    // or three displays ago. That backwards jump is exactly the hitching the
    // user felt. The staged caller never noticed, because its eye textures
    // always still held the previous frame.
    if (g_pIsSlotReady && !SpinUntil([&]() { return g_pIsSlotReady(frontSlot) != 0; }, maxWaitMs)) {
        const int otherSlot = 1 - frontSlot;
        if (g_fullFrameTex[otherSlot] && g_pIsSlotReady(otherSlot)) {
            static UINT s_otherSlotCount = 0;
            ++s_otherSlotCount;
            if (s_otherSlotCount <= 5 || (s_otherSlotCount % 300) == 0)
                Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: slot=%d not ready, using slot=%d (previous frame) "
                           "instead of skipping (#%u)",
                    s_copyCount, frontSlot, otherSlot, s_otherSlotCount);
            frontSlot = otherSlot;
            src = g_fullFrameTex[otherSlot];
            if (outSlot)
                *outSlot = otherSlot;
        } else {
            // Same reasoning as the skip above: true means "keep using what
            // you have", which is correct for the staged caller and keeps its
            // pose tagging alive. Only the direct-submit path needs to know
            // the difference, and that path is off.
            static UINT s_notReadyCount = 0;
            ++s_notReadyCount;
            if (s_notReadyCount <= 10 || (s_notReadyCount % 300) == 0)
                Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: slot=%d published but its GPU copy hasn't completed yet, reusing last frame (skip #%u)",
                    s_copyCount, frontSlot, s_notReadyCount);
            return true;
        }
    }

    // --- Step 3: the slot is genuinely ours to read. Mark and copy.
    if (g_pBeginReadSlot)
        g_pBeginReadSlot(frontSlot);

    D3D11_BOX leftBox{ 0, 0, 0, eyeWidth, eyeHeight, 1 };
    D3D11_BOX rightBox{ eyeWidth, 0, 0, eyeWidth * 2, eyeHeight, 1 };

    if (verboseLog) {
        Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: about to copy left (frontSlot=%d, src=%p, leftDst=%p, box=%u,%u,%u..%u,%u,%u)",
            s_copyCount, frontSlot, src, leftDst, leftBox.left, leftBox.top, leftBox.front, leftBox.right, leftBox.bottom, leftBox.back);
    }
    d3d11Context->CopySubresourceRegion(leftDst, 0, 0, 0, 0, src, 0, &leftBox);
    if (verboseLog)
        Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: left copy returned OK", s_copyCount);

    if (verboseLog) {
        Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: about to copy right (frontSlot=%d, src=%p, rightDst=%p, box=%u,%u,%u..%u,%u,%u)",
            s_copyCount, frontSlot, src, rightDst, rightBox.left, rightBox.top, rightBox.front, rightBox.right, rightBox.bottom, rightBox.back);
    }
    d3d11Context->CopySubresourceRegion(rightDst, 0, 0, 0, 0, src, 0, &rightBox);
    if (verboseLog)
        Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: right copy returned OK", s_copyCount);
    if (outCopied)
        *outCopied = true; // real pixels went into both destinations

    // Mark where the GPU is in our copies so step 1 can retire this read
    // next frame without blocking. If the query couldn't be created we
    // still hand the slot straight back rather than holding it forever -
    // the addon would otherwise skip every future frame's copy.
    if (g_syncQuery) {
        d3d11Context->End(g_syncQuery);
        g_pendingSlot = frontSlot;
        g_readPending = true;
    } else {
        if (verboseLog)
            Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: no sync query available - releasing slot=%d immediately (small overwrite race, but never a stall)",
                s_copyCount, frontSlot);
        if (g_pEndReadSlot)
            g_pEndReadSlot(frontSlot);
    }

    // A GPU fault is silent from D3D11's per-call return values - copies
    // keep "succeeding" while nothing actually completes. Ask the device
    // directly; this is the consumer-side half of the same check the
    // addon does, and between them they cover both APIs.
    if (verboseLog) {
        ID3D11Device* device = nullptr;
        d3d11Context->GetDevice(&device);
        if (device) {
            const HRESULT removed = device->GetDeviceRemovedReason();
            if (removed != S_OK)
                Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: *** D3D11 DEVICE REMOVED *** reason=0x%08lX (0x887A0006=HUNG, 0x887A0005=REMOVED, 0x887A0007=RESET)",
                    s_copyCount, removed);
            device->Release();
        }
    }

    return true;
}

void D3D12AddonBridge_Shutdown()
{
    // Hand back any slot we still hold, or the addon would skip copying
    // into it forever if the bridge is ever brought back up.
    if (g_readPending && g_pEndReadSlot && g_pendingSlot >= 0)
        g_pEndReadSlot(g_pendingSlot);
    g_pendingSlot = -1;
    g_readPending = false;

    if (g_syncQuery) {
        g_syncQuery->Release();
        g_syncQuery = nullptr;
    }

    ReleaseTextures();
    g_active = false;
    g_pGetFrameInfo = nullptr;
    g_pGetFrontSlot = nullptr;
    g_pBeginReadSlot = nullptr;
    g_pEndReadSlot = nullptr;
    g_pIsSlotReady = nullptr;
    g_pGetSharedGeneration = nullptr;
    g_openedGeneration = 0;
    g_addonModule = nullptr;
}

// ---- Desktop view (2026-09-13) --------------------------------------------
// See RE5VRAddon_SetDesktopView in addon_main.cpp. Looked up on its own,
// not in TryInit: the desktop view matters from the moment stereo is on,
// and TryInit only runs once the VR session starts.
void D3D12AddonBridge_SetDesktopView(bool enabled, int x, int y, int w, int h)
{
    using PFN_SetDesktopView = void(__cdecl*)(bool, int, int, int, int);
    static PFN_SetDesktopView s_fn = nullptr;
    static ULONGLONG s_lastLookupMs = 0;
    static bool s_reportedMissing = false;
    if (!s_fn) {
        const ULONGLONG now = GetTickCount64();
        if (now - s_lastLookupMs < 2000)
            return;
        s_lastLookupMs = now;
        if (HMODULE addon = GetModuleHandleA("SampleAddon.dll"))
            s_fn = reinterpret_cast<PFN_SetDesktopView>(GetProcAddress(addon, "RE5VRAddon_SetDesktopView"));
        if (!s_fn) {
            if (!s_reportedMissing && enabled) {
                s_reportedMissing = true;
                Log_Printf("D3D12AddonBridge: SampleAddon.dll has no RE5VRAddon_SetDesktopView (missing or an older "
                           "addon) - the desktop keeps showing the side-by-side image");
            }
            return;
        }
        Log_Printf("D3D12AddonBridge: desktop view control found in SampleAddon.dll");
    }
    s_fn(enabled, x, y, w, h);
}

bool D3D12AddonBridge_WarmUp()
{
    HMODULE addon = GetModuleHandleA("SampleAddon.dll");
    const auto getFrameInfo =
        addon ? reinterpret_cast<PFN_GetFrameInfo>(GetProcAddress(addon, "RE5VRAddon_GetFrameInfo")) : nullptr;
    if (!getFrameInfo)
        return true;

    static ULONGLONG s_startMs = 0, s_lastCallMs = 0;
    const ULONGLONG now = GetTickCount64();
    if (!s_startMs || now - s_lastCallMs > 1000)
        s_startMs = now; // a new attempt, not the tail of an old one
    s_lastCallMs = now;

    // Any consumer call wakes the copy. Textures from an earlier session can
    // already be there, so give the copy a moment to put a fresh frame in them.
    const bool ready = getFrameInfo(nullptr, nullptr, nullptr, nullptr, nullptr);
    const ULONGLONG waited = now - s_startMs;
    if ((ready && waited >= 250) || waited >= 3000) {
        Log_Printf("D3D12AddonBridge: addon warmed up for VR in %llu ms (%s)", waited,
            ready ? "frame ready" : "no frame yet, going ahead");
        s_startMs = 0;
        return true;
    }
    return false;
}

void D3D12AddonBridge_NotifyReset(bool resetting)
{
    using PFN_SetResetting = void(__cdecl*)(bool);
    static PFN_SetResetting s_fn = nullptr;
    if (!s_fn) {
        if (HMODULE addon = GetModuleHandleA("SampleAddon.dll"))
            s_fn = reinterpret_cast<PFN_SetResetting>(GetProcAddress(addon, "RE5VRAddon_SetResetting"));
        if (!s_fn)
            return;
    }
    s_fn(resetting);
}
