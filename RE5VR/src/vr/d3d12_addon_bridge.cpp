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

HMODULE g_addonModule = nullptr;
PFN_GetFrameInfo g_pGetFrameInfo = nullptr;
PFN_GetFrontSlot g_pGetFrontSlot = nullptr;
PFN_ReadSlot g_pBeginReadSlot = nullptr;
PFN_ReadSlot g_pEndReadSlot = nullptr;
PFN_IsSlotReady g_pIsSlotReady = nullptr;

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
    for (int attempt = 0; attempt < 30 && !ready; ++attempt) {
        ready = g_pGetFrameInfo(&width, &height, &dxgiFormat, &handle0, &handle1);
        if (!ready)
            Sleep(100);
    }
    if (!ready) {
        Log_Printf("D3D12AddonBridge: SampleAddon.dll loaded but never reported a ready frame after ~3s - falling back to D3D9 path");
        return false;
    }

    ID3D11Device1* d3d11Device1 = nullptr;
    HRESULT hr = d3d11Device->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&d3d11Device1));
    if (FAILED(hr)) {
        Log_Printf("D3D12AddonBridge: QueryInterface(ID3D11Device1) failed (hr=0x%08lX) - needed to open the D3D12 NT shared handle", hr);
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
    if (!g_active || !g_pGetFrontSlot)
        return -1;
    const int slot = g_pGetFrontSlot();
    return (slot < 0 || slot > 1) ? -1 : slot;
}

bool D3D12AddonBridge_CopyToEyeSlots(ID3D11DeviceContext* d3d11Context,
    ID3D11Texture2D* leftDst, ID3D11Texture2D* rightDst, UINT eyeWidth, UINT eyeHeight,
    int* outSlot)
{
    if (!g_active || !g_pGetFrontSlot)
        return false;

    const int frontSlot = g_pGetFrontSlot();
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
        const HRESULT hr = g_syncQuery
            ? d3d11Context->GetData(g_syncQuery, &done, sizeof(done), 0)
            : S_OK; // no query available - treat as done, see step 3

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
            return true;
        }
    }

    // --- Step 2: has the addon's own copy INTO this slot finished?
    //
    // The addon publishes a slot as soon as it SUBMITS the copy, not once
    // it completes, so this is the matching half of that contract: a
    // cheap fence GetCompletedValue() comparison on the addon's side. If
    // it isn't ready we reuse the previous frame rather than wait.
    if (g_pIsSlotReady && !g_pIsSlotReady(frontSlot)) {
        static UINT s_notReadyCount = 0;
        ++s_notReadyCount;
        if (s_notReadyCount <= 10 || (s_notReadyCount % 300) == 0)
            Log_Printf("D3D12AddonBridge_CopyToEyeSlots #%u: slot=%d published but its GPU copy hasn't completed yet, reusing last frame (skip #%u)",
                s_copyCount, frontSlot, s_notReadyCount);
        return true;
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
    g_addonModule = nullptr;
}
