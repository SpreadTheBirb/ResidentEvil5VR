// dgVoodoo2 "Addon" DLL (must be named exactly SampleAddon.dll - see the
// header comment further down for why). This is the producer half of
// RE5VR's D3D12 interop bridge: it observes dgVoodoo2's D3D12 backend,
// copies the real rendered frame (the same side-by-side stereo image the
// old D3D9 CPU-readback bridge used to crop out of the game's backbuffer)
// into one of two shared D3D12 textures every frame, and exposes those
// textures - plus which one is currently safe to read - to RE5VR's own
// proxy DLL (d3d9.dll) via a small polling API. See src/vr/
// d3d12_addon_bridge.cpp in the main project for the consumer half.
//
// Hard constraint, confirmed from dgVoodoo's own Addon docs
// (Addons/Overview/AddonMain.html, only readable after decompiling
// dgVoodooAPI.chm - not covered by the public web docs at all): as of
// the dgVoodoo2 version in use, it only ever looks for a DLL named
// EXACTLY "SampleAddon.dll" sitting next to its own D3D9.dll - this is
// not a naming convention we chose, it's hardcoded on dgVoodoo's side.
// The project's TargetName must stay "SampleAddon" or dgVoodoo will
// never find this DLL at all (no error, just silently never loaded).
//
// Export names (AddOnInit / AddOnExit, capital "O") were verified
// against the real compiled sample binary via dumpbin /exports, not
// just the prose docs (which use different casing - the docs are wrong,
// the sample binary is ground truth).
//
// Synchronization design (rewritten 2026-09-10 - see below for what it
// replaced and why). Nothing here blocks the CPU, on either side:
//
//   AFlushLock()            <- dgVoodoo may not flush mid-sequence now
//     ResourceBarrier(src -> COPY_SOURCE, dst COMMON -> COPY_DEST)
//     CopyResource(dst, src)
//     ResourceBarrier(back to original states)
//   value = GetFenceValue() <- what this batch will signal when DONE
//   AFlushUnlock(true)      <- submit; GPU starts working, we do NOT wait
//   publish {slot, value}
//
// The consumer then asks RE5VRAddon_IsSlotReady(slot), which is just
// AGetFence()->GetCompletedValue() >= the value recorded for that slot -
// a cheap, non-blocking read that answers exactly the right question
// ("has the GPU actually finished writing this slot?"). If the answer is
// no, the consumer simply reuses the previous frame rather than waiting.
//
// This is the idiom the official "D3D12 Addon" sample itself uses
// (Samples/D3D12 Addon/Presenter.cpp: AFlushLock/AFlushUnlock around a
// recorded block, and AGetFence()+GetFenceValue() handed to the
// descriptor ring buffer so it knows when an allocation is safe to
// recycle). The sample never calls AFlush(true) anywhere.
//
// What this replaced, and why: the first working version called
// AFlush(true) - a host-blocking wait for the copy to finish - right
// here in the present callback, because it looked like the only way to
// know the copy was done (GetFenceValue/AGetFence were misread as being
// only for dgVoodoo's own internal bookkeeping). That version delivered
// a real image but then reliably froze the game a few seconds later,
// identically across every variant tried (unthrottled, rate-capped, and
// with a full CPU-side reader/writer handshake). Two things were wrong
// with it, both fixed above:
//   1. AFlush(true) has no timeout, and ran on whatever thread dgVoodoo
//      calls PresentBegin from. If that wait ever fails to complete, the
//      process simply stops, which is exactly what the logs showed
//      (frame counter stops advancing, no error, no shutdown message).
//   2. The barrier/copy/barrier block was recorded with NO AFlushLock,
//      so dgVoodoo's own auto-flush could split it - submitting the
//      barriers and the copy in separate batches. A copy executing
//      against a resource that isn't in the state its barrier promised
//      is a genuine GPU fault, which would surface as a device
//      removal/reset rather than a clean error, and matches the
//      "everything stops at once" symptom just as well.
// Both are now gone rather than diagnosed further, since the correct
// design costs less than the broken one did.

#define _APIDLL
#include "Addon/AddonDefs.hpp"
#include "Addon/IAddonMainCallback.hpp"
#include "Addon/ID3D12RootObserver.hpp"

#include "addon_log.h"

#include <windows.h>
#include <d3d12.h>
#include <d3dcompiler.h>
#include <atomic>
#include <cstring>

using namespace dgVoodoo;

namespace {

IAddonMainCallback* g_addonMainCB = nullptr;
ID3D12Root* g_d3d12Root = nullptr;
ID3D12Device* g_d3d12Device = nullptr;

// --- Shared double-buffer state, published to the consumer (d3d9.dll) ---
// via SampleAddon.dll's own exported query functions further down.

constexpr int kNumSlots = 2;

bool g_texturesReady = false;
UINT32 g_frameWidth = 0;
UINT32 g_frameHeight = 0;
DXGI_FORMAT g_frameFormat = DXGI_FORMAT_UNKNOWN; // the SHARED texture's own (concrete) format, reported to the consumer
DXGI_FORMAT g_lastSourceFormat = DXGI_FORMAT_UNKNOWN; // the raw source's format (possibly typeless) - for change detection only

ID3D12Resource* g_sharedTex[kNumSlots] = { nullptr, nullptr };
HANDLE g_sharedHandle[kNumSlots] = { nullptr, nullptr };

// Which slot was most recently written (the freshest one for the
// consumer to read). -1 until the first copy publishes one. Same
// front-index pattern as the old D3D9 bridge, with one difference: the
// slot is published as soon as the copy is SUBMITTED, not once it has
// finished, so "is it actually finished?" is a separate question the
// consumer asks via RE5VRAddon_IsSlotReady (g_slotFenceValue below).
// Publishing early like this is what lets both sides stay non-blocking.
std::atomic<int> g_frontSlot{ -1 };
// Bumped every time the shared textures are recreated (the game frame changed
// size or format, e.g. full resolution per eye switching on or off). The
// consumer compares it to reopen the new handles; handle values alone can be
// reused by Windows after CloseHandle.
std::atomic<unsigned> g_sharedGeneration{ 0 };

// dgVoodoo's own auto-flush fence, and the value each slot's copy will
// have signalled once the GPU has genuinely finished writing it.
// Reading GetCompletedValue() off this fence is the entire cross-API
// synchronization mechanism - see this file's header comment.
ID3D12Fence* g_flushFence = nullptr;
std::atomic<UINT64> g_slotFenceValue[kNumSlots] = {};

// The other half of the handshake, guarding the opposite direction from
// g_slotFenceValue: that one tells the consumer when the PRODUCER is
// done writing a slot, this one tells the producer when the CONSUMER is
// done reading one. The consumer marks a slot via
// RE5VRAddon_BeginReadSlot/EndReadSlot and the producer skips any slot
// with an active reader rather than overwriting it mid-read.
//
// Importantly the consumer only calls EndReadSlot once its own D3D11
// read has actually completed on the GPU, which it determines by
// polling a query WITHOUT blocking (it just keeps the slot marked and
// tries again next frame). Neither side ever waits on the other; the
// worst case is one side skipping a frame, never stalling on it.
std::atomic<int> g_slotReaders[kNumSlots] = {};

UInt32 g_presentBeginCount = 0;
constexpr UInt32 kPresentLogInterval = 300; // ~once every few seconds at typical framerates

// D3D12 happily creates/copies TYPELESS resources, but D3D11's
// OpenSharedResource1 flatly rejects opening a shared handle to one
// (E_INVALIDARG, confirmed empirically 2026-09-10 - see re5vr_project
// memory) - it needs a concrete format. CopyResource explicitly allows
// copying between a typeless resource and a concrete-format one in the
// same family (raw bytes, no reinterpretation), so the fix is simply to
// create the SHARED texture with the concrete counterpart while still
// copying from whatever the real (possibly typeless) source format is.
// Only the two families actually observed/plausible here are mapped;
// anything else passes through unchanged (and will very likely fail the
// same way if it's also typeless - not silently swallowed, logged by
// EnsureSharedTextures either way since the reported format will look
// exactly like whatever was passed through).
DXGI_FORMAT ToConcreteFormat(DXGI_FORMAT format)
{
    switch (format) {
        case DXGI_FORMAT_R8G8B8A8_TYPELESS: return DXGI_FORMAT_R8G8B8A8_UNORM;
        case DXGI_FORMAT_B8G8R8A8_TYPELESS: return DXGI_FORMAT_B8G8R8A8_UNORM;
        default: return format;
    }
}

// Releases any previously-created shared textures/handles - called both
// on swapchain size/format changes (re-create at the new size) and on
// D3D12RootReleased (full teardown).
void ReleaseSharedTextures()
{
    for (int i = 0; i < kNumSlots; ++i) {
        if (g_sharedHandle[i]) {
            CloseHandle(g_sharedHandle[i]);
            g_sharedHandle[i] = nullptr;
        }
        if (g_sharedTex[i]) {
            g_sharedTex[i]->Release();
            g_sharedTex[i] = nullptr;
        }
    }
    g_texturesReady = false;
    g_lastSourceFormat = DXGI_FORMAT_UNKNOWN;
    g_frontSlot.store(-1, std::memory_order_release);
    for (int i = 0; i < kNumSlots; ++i)
        g_slotFenceValue[i].store(0, std::memory_order_release);
}

// Lazily (re)creates the two shared textures the first time we see a
// real source texture in PresentBegin, or whenever its format/size
// changes from what we last allocated for. Deliberately keyed off the
// real ID3D12Resource's own GetDesc() rather than the swapchain's
// reported format/size (D3D12SwapchainChanged's SwapchainData) - those
// are not guaranteed to be the exact same object, and guessing wrong
// here would silently corrupt every downstream copy (same class of bug
// documented elsewhere in this project: CopyResource-style operations
// between mismatched formats/sizes fail or produce garbage with no
// obvious symptom until someone notices the image is wrong).
bool EnsureSharedTextures(ID3D12Resource* pSrcTexture)
{
    D3D12_RESOURCE_DESC srcDesc = pSrcTexture->GetDesc();

    if (g_texturesReady &&
        g_frameWidth == static_cast<UINT32>(srcDesc.Width) &&
        g_frameHeight == srcDesc.Height &&
        g_lastSourceFormat == srcDesc.Format) {
        return true;
    }

    AddonLog_Printf("EnsureSharedTextures: (re)creating shared textures for %ux%u format=%d (was %ux%u format=%d)",
        static_cast<UINT32>(srcDesc.Width), srcDesc.Height, static_cast<int>(srcDesc.Format),
        g_frameWidth, g_frameHeight, static_cast<int>(g_lastSourceFormat));

    ReleaseSharedTextures();

    D3D12_HEAP_PROPERTIES heapProps = {};
    heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

    const DXGI_FORMAT concreteFormat = ToConcreteFormat(srcDesc.Format);
    if (concreteFormat != srcDesc.Format) {
        AddonLog_Printf("EnsureSharedTextures: source format %d is typeless - creating the shared texture as concrete format %d instead (CopyResource still works between them)",
            static_cast<int>(srcDesc.Format), static_cast<int>(concreteFormat));
    }

    D3D12_RESOURCE_DESC texDesc = {};
    texDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    texDesc.Width = srcDesc.Width;
    texDesc.Height = srcDesc.Height;
    texDesc.DepthOrArraySize = 1;
    texDesc.MipLevels = 1;
    texDesc.Format = concreteFormat;
    texDesc.SampleDesc.Count = 1;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    // 2026-09-10: was D3D12_RESOURCE_FLAG_NONE (no declared capability at
    // all) - reproduced the identical OpenSharedResource1 E_INVALIDARG on
    // two completely different GPUs/driver branches (laptop Quadro M2200
    // AND desktop RTX A4500), ruling out a driver quirk and pointing back
    // at the resource description itself. D3D12's own OpenSharedHandle
    // doesn't seem to mind a flagless resource, but D3D11 has to
    // construct a fully valid resource description across the API
    // boundary, and a texture declaring zero capabilities may not be
    // something it accepts as legitimate even though we only ever use it
    // as a plain copy source/destination. ALLOW_RENDER_TARGET costs
    // nothing here (never actually rendered to) but gives D3D11 a real
    // capability to hang a valid resource description on.
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

    for (int i = 0; i < kNumSlots; ++i) {
        // D3D12_RESOURCE_STATE_COMMON (not COPY_DEST) matches Microsoft's
        // own documented D3D11/D3D12 shared-resource pattern exactly -
        // trying this after COPY_DEST's OpenSharedResource1 kept failing
        // with E_INVALIDARG on an otherwise-valid, stable handle (same
        // adapter confirmed via LUID - see re5vr_project memory).
        HRESULT hr = g_d3d12Device->CreateCommittedResource(
            &heapProps, D3D12_HEAP_FLAG_SHARED, &texDesc,
            D3D12_RESOURCE_STATE_COMMON, nullptr,
            IID_PPV_ARGS(&g_sharedTex[i]));
        if (FAILED(hr)) {
            AddonLog_Printf("EnsureSharedTextures: CreateCommittedResource (slot=%d) failed (hr=0x%08lX)", i, hr);
            ReleaseSharedTextures();
            return false;
        }

        hr = g_d3d12Device->CreateSharedHandle(g_sharedTex[i], nullptr, GENERIC_ALL, nullptr, &g_sharedHandle[i]);
        if (FAILED(hr)) {
            AddonLog_Printf("EnsureSharedTextures: CreateSharedHandle (slot=%d) failed (hr=0x%08lX)", i, hr);
            ReleaseSharedTextures();
            return false;
        }

        // Diagnostic self-test: confirm the handle is valid at the D3D12
        // level itself (same device re-opening its own handle) before
        // ever handing it to the D3D11 consumer - narrows down whether a
        // future OpenSharedResource1 failure is D3D11-interop-specific
        // or the handle/heap itself being unsound from the start.
        ID3D12Resource* selfTestResource = nullptr;
        HRESULT selfTestHr = g_d3d12Device->OpenSharedHandle(g_sharedHandle[i], IID_PPV_ARGS(&selfTestResource));
        AddonLog_Printf("EnsureSharedTextures: self-test OpenSharedHandle (slot=%d, D3D12 side) -> %s (hr=0x%08lX)",
            i, SUCCEEDED(selfTestHr) ? "OK" : "FAILED", selfTestHr);
        if (selfTestResource)
            selfTestResource->Release();
    }

    g_frameWidth = static_cast<UINT32>(srcDesc.Width);
    g_frameHeight = srcDesc.Height;
    g_frameFormat = concreteFormat; // the SHARED texture's own format, not necessarily the source's
    g_lastSourceFormat = srcDesc.Format;
    g_texturesReady = true;
    g_sharedGeneration.fetch_add(1, std::memory_order_acq_rel);

    AddonLog_Printf("EnsureSharedTextures: ready (%ux%u, format=%d, handles=%p/%p)",
        g_frameWidth, g_frameHeight, static_cast<int>(g_frameFormat), g_sharedHandle[0], g_sharedHandle[1]);
    return true;
}

// ---- Desktop view (2026-09-13) --------------------------------------------
// In VR the game's frame is both eyes side by side, and that is what the game
// window showed - no good for streaming or recording. dgVoodoo lets an addon
// replace the image its presenter puts in the window (PresentBeginContextOutput).
//
// First attempt: copy one eye's region into our own texture of that size and
// hand it over. The docs say the output "may not have the same size", but in
// practice the presenter still read the ORIGINAL source rect (0,0-1280,720)
// out of our 640x375 texture: the eye sat unscaled in the top-left corner with
// junk around it (user screenshot). So the scaling is done here instead, the
// way dgVoodoo's own sample addon draws: a textured quad into one of the
// swapchain's "proxy" textures (always swapchain-sized), covering the source
// rect, sampling just the chosen eye region with a bilinear filter.
//
// Which region is decided by d3d9.dll (it knows the per-eye projection and
// the window's aspect) and pushed every frame through RE5VRAddon_SetDesktopView.
// Disabled means the presenter's input is left alone.
std::atomic<bool> g_dvEnabled{ false };
std::atomic<Int32> g_dvX{ 0 }, g_dvY{ 0 }, g_dvW{ 0 }, g_dvH{ 0 };

ID3D12RootSignature* g_dvRootSig = nullptr;
ID3DBlob* g_dvVS = nullptr;
ID3DBlob* g_dvPS = nullptr;
ID3D12PipelineState* g_dvPSO = nullptr;
DXGI_FORMAT g_dvPSOFormat = DXGI_FORMAT_UNKNOWN;
bool g_dvFailed = false; // setup failed once: stay out of the presenter's way for good

// A full-screen triangle from the vertex id alone (no vertex buffer), with the
// eye region's UV offset and scale in four root constants.
constexpr const char kDesktopViewVS[] = R"(
cbuffer Crop : register(b0) { float2 uvOffset; float2 uvScale; };
struct V2P { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };
V2P main(uint id : SV_VertexID)
{
    V2P o;
    float2 t = float2((id << 1) & 2, id & 2);
    o.pos = float4(t * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    o.uv = uvOffset + t * uvScale;
    return o;
}
)";
constexpr const char kDesktopViewPS[] = R"(
Texture2D<float4> src : register(t0);
SamplerState bilinear : register(s0);
struct V2P { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };
float4 main(V2P i) : SV_TARGET0
{
    return float4(src.Sample(bilinear, i.uv).rgb, 1.0);
}
)";

ID3DBlob* CompileDesktopShader(const char* source, size_t length, const char* target)
{
    ID3DBlob* code = nullptr;
    ID3DBlob* errors = nullptr;
    const HRESULT hr = D3DCompile(source, length, "RE5VRDesktopView", nullptr, nullptr, "main", target,
        D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors);
    if (FAILED(hr)) {
        AddonLog_Printf("DesktopView: %s compile failed (hr=0x%08lX) %s", target, hr,
            errors ? static_cast<const char*>(errors->GetBufferPointer()) : "");
        if (code)
            code->Release();
        code = nullptr;
    }
    if (errors)
        errors->Release();
    return code;
}

bool EnsureDesktopPipeline(UInt32 adapterID, DXGI_FORMAT rtvFormat)
{
    if (g_dvFailed)
        return false;
    if (!g_dvVS) {
        g_dvVS = CompileDesktopShader(kDesktopViewVS, sizeof(kDesktopViewVS) - 1, "vs_5_0");
        g_dvPS = CompileDesktopShader(kDesktopViewPS, sizeof(kDesktopViewPS) - 1, "ps_5_0");
        if (!g_dvVS || !g_dvPS) {
            g_dvFailed = true;
            return false;
        }
    }
    if (!g_dvRootSig) {
        static const D3D12_DESCRIPTOR_RANGE srvRange = { D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0 };
        D3D12_ROOT_PARAMETER params[2] = {};
        params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        params[0].DescriptorTable.NumDescriptorRanges = 1;
        params[0].DescriptorTable.pDescriptorRanges = &srvRange;
        params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        params[1].Constants.ShaderRegister = 0;
        params[1].Constants.RegisterSpace = 0;
        params[1].Constants.Num32BitValues = 4;
        params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
        D3D12_STATIC_SAMPLER_DESC sampler = {};
        sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
        sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
        sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_ALWAYS;
        sampler.MaxLOD = D3D12_FLOAT32_MAX;
        sampler.ShaderRegister = 0;
        sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        D3D12_ROOT_SIGNATURE_DESC desc = {};
        desc.NumParameters = 2;
        desc.pParameters = params;
        desc.NumStaticSamplers = 1;
        desc.pStaticSamplers = &sampler;
        desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS |
            D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS;
        g_dvRootSig = g_d3d12Root->SerializeAndCreateRootSignature(adapterID, D3D_ROOT_SIGNATURE_VERSION_1, &desc, nullptr);
        if (!g_dvRootSig) {
            AddonLog_Printf("DesktopView: root signature creation failed");
            g_dvFailed = true;
            return false;
        }
    }
    if (!g_dvPSO || g_dvPSOFormat != rtvFormat) {
        static const D3D12_BLEND_DESC blend = { FALSE, FALSE,
            { { FALSE, FALSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_BLEND_OP_ADD, D3D12_BLEND_ONE, D3D12_BLEND_ZERO,
                D3D12_BLEND_OP_ADD, D3D12_LOGIC_OP_NOOP, D3D12_COLOR_WRITE_ENABLE_ALL } } };
        static const D3D12_RASTERIZER_DESC raster = { D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, FALSE, 0, 0.0f, 0.0f,
            TRUE, FALSE, FALSE, 0, D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF };
        static const D3D12_DEPTH_STENCIL_DESC depth = { FALSE, D3D12_DEPTH_WRITE_MASK_ZERO, D3D12_COMPARISON_FUNC_ALWAYS, FALSE,
            0xFF, 0xFF, { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS },
            { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS } };
        static D3D12_INPUT_LAYOUT_DESC noInputLayout = { nullptr, 0 };
        ID3D12Root::GraphicsPLDesc pl = {};
        pl.pRootSignature = g_dvRootSig;
        pl.pVS = g_dvVS;
        pl.pPS = g_dvPS;
        pl.pBlendState = g_d3d12Root->PLCacheGetBlend4Desc(adapterID, blend);
        pl.SampleMask = 0xFFFFFFFF;
        pl.pRasterizerState = g_d3d12Root->PLCacheGetRasterizerDesc(adapterID, raster);
        pl.pDepthStencilState = g_d3d12Root->PLCacheGetDepthStencilDesc(adapterID, depth);
        pl.pInputLayout = &noInputLayout;
        pl.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED;
        pl.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        pl.NumRenderTargets = 1;
        pl.RTVFormats[0] = rtvFormat;
        pl.DSVFormat = DXGI_FORMAT_UNKNOWN;
        pl.SampleDesc.Count = 1;
        g_dvPSO = g_d3d12Root->PLCacheGetGraphicsPipeline(adapterID, pl);
        g_dvPSOFormat = rtvFormat;
        if (!g_dvPSO) {
            AddonLog_Printf("DesktopView: pipeline creation failed (rtv format %d)", static_cast<int>(rtvFormat));
            g_dvFailed = true;
            return false;
        }
        AddonLog_Printf("DesktopView: pipeline ready (rtv format %d)", static_cast<int>(rtvFormat));
    }
    return true;
}

void ReleaseDesktopView()
{
    // The pipeline comes from dgVoodoo's cache and dies with it. Tell the
    // cache about our root signature before letting it go, as the sample does.
    if (g_dvRootSig && g_d3d12Root)
        g_d3d12Root->GPLRootSignatureReleased(0, g_dvRootSig);
    if (g_dvRootSig)
        g_dvRootSig->Release();
    g_dvRootSig = nullptr;
    g_dvPSO = nullptr;
    g_dvPSOFormat = DXGI_FORMAT_UNKNOWN;
}

class RE5VRAddonObserver : public ID3D12RootObserver
{
public:
    bool D3D12RootCreated(HMODULE hD3D12Dll, ID3D12Root* pD3D12Root) override
    {
        AddonLog_Printf("D3D12RootCreated (hD3D12Dll=%p, pD3D12Root=%p)", hD3D12Dll, pD3D12Root);
        g_d3d12Root = pD3D12Root;
        return true;
    }

    void D3D12RootReleased(const ID3D12Root* pD3D12Root) override
    {
        AddonLog_Printf("D3D12RootReleased (pD3D12Root=%p)", pD3D12Root);
        ReleaseSharedTextures();
        ReleaseDesktopView();
        g_flushFence = nullptr; // dgVoodoo owns it; it dies with the root
        g_d3d12Root = nullptr;
        g_d3d12Device = nullptr;
    }

    bool D3D12BeginUsingAdapter(UInt32 adapterID) override
    {
        AddonLog_Printf("D3D12BeginUsingAdapter (adapterID=%u)", adapterID);
        if (g_d3d12Root && !g_d3d12Device) {
            g_d3d12Device = g_d3d12Root->GetDevice(adapterID);
            AddonLog_Printf("D3D12BeginUsingAdapter: ID3D12Device* = %p", g_d3d12Device);
            if (g_d3d12Device) {
                LUID luid = g_d3d12Device->GetAdapterLuid();
                AddonLog_Printf("D3D12BeginUsingAdapter: adapter LUID = %08lX:%08lX", luid.HighPart, luid.LowPart);

                // D3D12_FEATURE_D3D12_OPTIONS4::SharedResourceCompatibilityTier
                // is the GPU/driver's explicit declaration of whether (and how
                // well) it supports opening a D3D12 resource from D3D11 at all -
                // some older/lower-tier GPUs report no support for this
                // regardless of anything the calling code does. Checking this
                // directly instead of continuing to guess, now that the
                // self-test above proved the resource/handle itself is sound
                // and the failure is specifically in the D3D11-side open.
                D3D12_FEATURE_DATA_D3D12_OPTIONS4 options4 = {};
                HRESULT featHr = g_d3d12Device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS4, &options4, sizeof(options4));
                if (SUCCEEDED(featHr)) {
                    AddonLog_Printf("D3D12BeginUsingAdapter: SharedResourceCompatibilityTier = %d",
                        static_cast<int>(options4.SharedResourceCompatibilityTier));
                } else {
                    AddonLog_Printf("D3D12BeginUsingAdapter: CheckFeatureSupport(OPTIONS4) failed (hr=0x%08lX) - driver may not even know about this feature", featHr);
                }
            }
        }
        return true;
    }

    void D3D12EndUsingAdapter(UInt32 adapterID) override
    {
        AddonLog_Printf("D3D12EndUsingAdapter (adapterID=%u)", adapterID);
        ReleaseSharedTextures();
        ReleaseDesktopView();
        g_d3d12Device = nullptr;
    }

    bool D3D12CreateSwapchainHook(UInt32 adapterID, IDXGIFactory1* pDxgiFactory, IUnknown* pCommandQueue,
        const DXGI_SWAP_CHAIN_DESC& desc, IDXGISwapChain** ppSwapChain) override
    {
        // Not hooking swapchain creation itself - let dgVoodoo create it
        // as normal.
        return false;
    }

    void D3D12SwapchainCreated(UInt32 adapterID, ID3D12Swapchain* pSwapchain, const ID3D12Root::SwapchainData& swapchainData) override
    {
        AddonLog_Printf("D3D12SwapchainCreated (adapterID=%u, pSwapchain=%p, imageSize=%ldx%ld, format=%d)",
            adapterID, pSwapchain,
            static_cast<long>(swapchainData.imageSize.cx), static_cast<long>(swapchainData.imageSize.cy),
            static_cast<int>(swapchainData.format));
    }

    void D3D12SwapchainChanged(UInt32 adapterID, ID3D12Swapchain* pSwapchain, const ID3D12Root::SwapchainData& swapchainData) override
    {
        AddonLog_Printf("D3D12SwapchainChanged (adapterID=%u, pSwapchain=%p, imageSize=%ldx%ld, format=%d)",
            adapterID, pSwapchain,
            static_cast<long>(swapchainData.imageSize.cx), static_cast<long>(swapchainData.imageSize.cy),
            static_cast<int>(swapchainData.format));
    }

    void D3D12SwapchainReleased(UInt32 adapterID, ID3D12Swapchain* pSwapchain) override
    {
        AddonLog_Printf("D3D12SwapchainReleased (adapterID=%u, pSwapchain=%p)", adapterID, pSwapchain);
    }

    bool D3D12SwapchainPresentBegin(UInt32 adapterID, const PresentBeginContextInput& iCtx, PresentBeginContextOutput& oCtx) override
    {
        CopyForHeadset(adapterID, iCtx);
        return OverrideDesktopView(adapterID, iCtx, oCtx);
    }

    // After the headset has its copy: draw one eye, scaled to fill, into a
    // proxy texture and present that instead of the side-by-side frame.
    bool OverrideDesktopView(UInt32 adapterID, const PresentBeginContextInput& iCtx, PresentBeginContextOutput& oCtx)
    {
        if (!g_dvEnabled.load(std::memory_order_acquire) || !g_d3d12Root || !g_d3d12Device || !iCtx.pSrcTexture ||
            !iCtx.pSwapchain)
            return false;

        const LONG frameW = iCtx.srcRect.right - iCtx.srcRect.left;
        const LONG frameH = iCtx.srcRect.bottom - iCtx.srcRect.top;
        LONG x = g_dvX.load(std::memory_order_acquire), y = g_dvY.load(std::memory_order_acquire);
        LONG w = g_dvW.load(std::memory_order_acquire), h = g_dvH.load(std::memory_order_acquire);
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x + w > frameW) w = frameW - x;
        if (y + h > frameH) h = frameH - y;
        if (w < 16 || h < 16 || frameW < 16 || frameH < 16)
            return false;

        // A proxy texture that isn't the incoming one (the source can itself be a proxy).
        ID3D12Root::SwapchainProxyTextureData proxy = {};
        bool haveProxy = false;
        for (UInt32 i = 0; i < 2 && !haveProxy; ++i) {
            if (g_d3d12Root->GetProxyTexture(iCtx.pSwapchain, i, &proxy) && proxy.pTexture && proxy.pTexture != iCtx.pSrcTexture)
                haveProxy = true;
        }
        if (!haveProxy || !EnsureDesktopPipeline(adapterID, proxy.rtvFormat))
            return false;

        ID3D12GraphicsCommandListAuto* cmdList = g_d3d12Root->GetGraphicsCommandListAuto(adapterID);
        ID3D12ResourceDescRingBuffer* ring = g_d3d12Root->GetCBV_SRV_UAV_RingBuffer(adapterID);
        if (!cmdList || !ring)
            return false;
        ID3D12ResourceDescRingBuffer::AllocData srv = {};
        if (!ring->Alloc(1, cmdList->AGetFence(), cmdList->GetFenceValue(), srv))
            return false;
        g_d3d12Device->CopyDescriptorsSimple(1, srv.cpuDescHandle, iCtx.srvCPUHandle, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

        const D3D12_RESOURCE_DESC srcDesc = iCtx.pSrcTexture->GetDesc();
        const float texW = static_cast<float>(srcDesc.Width), texH = static_cast<float>(srcDesc.Height);
        const float crop[4] = { (iCtx.srcRect.left + x) / texW, (iCtx.srcRect.top + y) / texH, w / texW, h / texH };

        cmdList->ChangeId(&g_dvEnabled); // we change the list's state; whoever writes next must reset theirs
        cmdList->AFlushLock();           // no early return until the unlock below
        ID3D12GraphicsCommandList* list = cmdList->GetCommandListInterface();

        D3D12_RESOURCE_BARRIER b[2] = {};
        UINT nb = 0;
        const bool srcToSrv = (iCtx.srcTextureState & D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE) == 0;
        if (srcToSrv) {
            b[nb].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b[nb].Transition.pResource = iCtx.pSrcTexture;
            b[nb].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b[nb].Transition.StateBefore = static_cast<D3D12_RESOURCE_STATES>(iCtx.srcTextureState);
            b[nb].Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            ++nb;
        }
        if ((proxy.texState & D3D12_RESOURCE_STATE_RENDER_TARGET) == 0) {
            b[nb].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b[nb].Transition.pResource = proxy.pTexture;
            b[nb].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b[nb].Transition.StateBefore = static_cast<D3D12_RESOURCE_STATES>(proxy.texState);
            b[nb].Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
            ++nb;
        }
        if (nb)
            list->ResourceBarrier(nb, b);

        list->SetGraphicsRootSignature(g_dvRootSig);
        list->SetPipelineState(g_dvPSO);
        list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        list->SetDescriptorHeaps(1, &srv.pHeap);
        list->SetGraphicsRootDescriptorTable(0, srv.gpuDescHandle);
        list->SetGraphicsRoot32BitConstants(1, 4, crop, 0);
        list->OMSetRenderTargets(1, &proxy.rtvHandle, TRUE, nullptr);
        const RECT dst = iCtx.srcRect; // dgVoodoo presents only this rect of the proxy
        list->RSSetScissorRects(1, &dst);
        const D3D12_VIEWPORT vp = { static_cast<FLOAT>(dst.left), static_cast<FLOAT>(dst.top), static_cast<FLOAT>(frameW),
            static_cast<FLOAT>(frameH), 0.0f, 1.0f };
        list->RSSetViewports(1, &vp);
        list->DrawInstanced(3, 1, 0, 0);

        // The source goes back to the state dgVoodoo handed it over in.
        if (srcToSrv) {
            b[0].Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            b[0].Transition.StateAfter = static_cast<D3D12_RESOURCE_STATES>(iCtx.srcTextureState);
            list->ResourceBarrier(1, &b[0]);
        }

        cmdList->AFlushUnlock(); // the presenter carries on writing into the same list

        oCtx.pOutputTexture = proxy.pTexture;
        oCtx.outputTexSRVCPUHandle = proxy.srvHandle;
        oCtx.outputTextureExpectedState = static_cast<UINT>(-1); // a proxy: dgVoodoo tracks its state

        static UInt32 s_logged = 0;
        if (s_logged < 3 || (g_presentBeginCount % kPresentLogInterval) == 0) {
            ++s_logged;
            AddonLog_Printf("DesktopView: drew frame region %ld,%ld %ldx%ld scaled into proxy %p (srcRect %ld,%ld-%ld,%ld, "
                            "proxy state 0x%X, source state 0x%X)",
                x, y, w, h, proxy.pTexture, iCtx.srcRect.left, iCtx.srcRect.top, iCtx.srcRect.right, iCtx.srcRect.bottom,
                proxy.texState, iCtx.srcTextureState);
        }
        return true;
    }

    void CopyForHeadset(UInt32 adapterID, const PresentBeginContextInput& iCtx)
    {
        ++g_presentBeginCount;
        const bool verboseLog = (g_presentBeginCount <= 5 || (g_presentBeginCount % kPresentLogInterval) == 0);

        if (!g_d3d12Root || !g_d3d12Device || !iCtx.pSrcTexture) {
            if (verboseLog)
                AddonLog_Printf("D3D12SwapchainPresentBegin #%u: missing root/device/srcTexture, skipping copy", g_presentBeginCount);
            return;
        }

        // 2026-09-10: throttle the producer instead of copying on every
        // single present (potentially thousands/sec, unthrottled). The
        // consumer reads at its own, much slower, independent rate - with
        // only 2 buffer slots, an unthrottled producer can "lap" the
        // consumer and start overwriting a slot before the consumer's own
        // GPU-side read of it has actually finished, even though
        // AFlush(true) already proved OUR OWN copy INTO it was complete.
        // This is the real explanation for the intermittent
        // KERNELBASE.dll crash that hit after ~300+ otherwise-successful
        // frames on both test machines (a real image was even seen in
        // the headset first) - the signature of an occasional race, not
        // a deterministic bug. The actual fix is a proper producer/
        // consumer handshake (not yet built); this bounded rate is a
        // cheap, well-motivated interim mitigation, matching the same
        // oversampling philosophy the old D3D9 bridge already uses
        // (openxr_bridge.cpp's g_targetFrameIntervalMs).
        static LARGE_INTEGER s_lastCopyTime = {};
        static LARGE_INTEGER s_qpcFreq = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return f; }();
        constexpr double kTargetIntervalMs = 1000.0 / 250.0; // 250Hz cap
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        if (s_lastCopyTime.QuadPart != 0) {
            const double elapsedMs = static_cast<double>(now.QuadPart - s_lastCopyTime.QuadPart) * 1000.0 / static_cast<double>(s_qpcFreq.QuadPart);
            if (elapsedMs < kTargetIntervalMs)
                return;
        }
        s_lastCopyTime = now;

        if (!EnsureSharedTextures(iCtx.pSrcTexture))
            return;

        // Write into whichever slot ISN'T currently published as front,
        // so the consumer never reads a slot we're mid-write into.
        const int frontNow = g_frontSlot.load(std::memory_order_acquire);
        const int backSlot = (frontNow == 0) ? 1 : 0;

        // Always leave the consumer one FINISHED frame. The copy into a slot
        // lands when the GPU gets to it, which is a frame or more behind once
        // the GPU is busy. Overwriting the back slot before the front slot's
        // copy has landed leaves both slots unfinished, and the consumer finds
        // nothing it may read, frame after frame: the headset froze while the
        // game played on (2026-09-14, full resolution per eye at 2960x1616 on
        // a Quadro M2200, ~45 skips a second). Waiting for the front copy
        // costs nothing - the consumer could not have used a newer frame yet.
        if (frontNow >= 0 && g_flushFence) {
            const UINT64 frontRequired = g_slotFenceValue[frontNow].load(std::memory_order_acquire);
            if (frontRequired != 0 && g_flushFence->GetCompletedValue() < frontRequired) {
                static UInt32 s_behindCount = 0;
                ++s_behindCount;
                if (s_behindCount <= 5 || (s_behindCount % 600) == 0)
                    AddonLog_Printf("D3D12SwapchainPresentBegin #%u: slot=%d's copy hasn't landed on the GPU yet, "
                                    "keeping it and skipping this frame's copy (#%u)",
                        g_presentBeginCount, frontNow, s_behindCount);
                return;
            }
        }

        // The real fix (see this file's g_slotReaders comment above): if
        // the consumer has marked this slot as actively being read, skip
        // this frame's copy entirely rather than overwrite it. This is
        // the actual race-closing check - the rate throttle above only
        // reduces how often this situation arises, it doesn't prevent it.
        if (g_slotReaders[backSlot].load(std::memory_order_acquire) > 0) {
            // Own counter and its own logging cadence, independent of
            // verboseLog - a skip could land on any present call number,
            // and gating this on the same "first 5 / every 300th" window
            // as normal success logging could hide it entirely if a skip
            // never happens to coincide with a verbose call. Always log
            // the first 20 skips, then periodically, so we can actually
            // tell whether this path is engaging at all.
            static UInt32 s_skipCount = 0;
            ++s_skipCount;
            if (s_skipCount <= 20 || (s_skipCount % 100) == 0) {
                AddonLog_Printf("D3D12SwapchainPresentBegin #%u: slot=%d has an active reader, skipping this frame's copy (skip #%u)",
                    g_presentBeginCount, backSlot, s_skipCount);
            }
            return;
        }

        ID3D12GraphicsCommandListAuto* cmdList = g_d3d12Root->GetGraphicsCommandListAuto(adapterID);
        if (!cmdList) {
            if (verboseLog)
                AddonLog_Printf("D3D12SwapchainPresentBegin #%u: GetGraphicsCommandListAuto failed", g_presentBeginCount);
            return;
        }
        ID3D12GraphicsCommandList* list = cmdList->GetCommandListInterface();

        // Everything from here to AFlushUnlock must be recorded as one
        // uninterrupted block - without this lock dgVoodoo's own
        // auto-flush can land between our barriers and our copy and
        // submit them separately, which is a real GPU fault waiting to
        // happen (see this file's header comment). There must be no
        // early return between the lock and the matching unlock below.
        cmdList->AFlushLock();

        D3D12_RESOURCE_BARRIER barriers[4] = {};

        barriers[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barriers[0].Transition.pResource = iCtx.pSrcTexture;
        barriers[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barriers[0].Transition.StateBefore = static_cast<D3D12_RESOURCE_STATES>(iCtx.srcTextureState);
        barriers[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;

        // Our own shared texture sits in COMMON at rest (the correct
        // resting state for a resource another API/device will read via
        // a shared handle) - transition it to COPY_DEST just for the
        // copy, then straight back to COMMON before publishing it.
        barriers[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barriers[1].Transition.pResource = g_sharedTex[backSlot];
        barriers[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barriers[1].Transition.StateBefore = D3D12_RESOURCE_STATE_COMMON;
        barriers[1].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;

        list->ResourceBarrier(2, &barriers[0]);

        list->CopyResource(g_sharedTex[backSlot], iCtx.pSrcTexture);

        // Leave the source texture exactly as dgVoodoo handed it to us -
        // its own presentation pipeline still needs to use it right
        // after this callback returns.
        barriers[2].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barriers[2].Transition.pResource = iCtx.pSrcTexture;
        barriers[2].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barriers[2].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
        barriers[2].Transition.StateAfter = static_cast<D3D12_RESOURCE_STATES>(iCtx.srcTextureState);

        barriers[3].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barriers[3].Transition.pResource = g_sharedTex[backSlot];
        barriers[3].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barriers[3].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        barriers[3].Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;

        list->ResourceBarrier(2, &barriers[2]);

        // Grab the fence + the value this batch will signal BEFORE
        // releasing the flush lock. Read after the unlock instead and
        // the command list may already have rolled on to the next batch,
        // giving a value that covers work we don't care about.
        g_flushFence = cmdList->AGetFence();
        const UINT64 pendingValue = cmdList->GetFenceValue();

        // Submit. The GPU starts working here; we deliberately do NOT
        // wait for it (the old AFlush(true) did, and that is the single
        // biggest suspect for the freeze this replaces).
        const bool flushed = cmdList->AFlushUnlock(true);

        // Publish slot + the value that proves it finished, in that
        // order, so a consumer that sees the new front slot always sees
        // a fence value it can trust for it.
        g_slotFenceValue[backSlot].store(pendingValue, std::memory_order_release);
        g_frontSlot.store(backSlot, std::memory_order_release);

        // A GPU fault (device removed/reset/hung) is the other live
        // explanation for the freeze, and it is completely invisible
        // from a D3D12 command list - nothing returns an error, work
        // just silently stops completing. Ask the device directly, and
        // log loudly and once, since after this everything downstream is
        // meaningless anyway.
        const HRESULT removedReason = g_d3d12Device->GetDeviceRemovedReason();
        if (removedReason != S_OK) {
            static bool s_reportedRemoval = false;
            if (!s_reportedRemoval) {
                s_reportedRemoval = true;
                AddonLog_Printf("D3D12SwapchainPresentBegin #%u: *** D3D12 DEVICE REMOVED *** GetDeviceRemovedReason=0x%08lX "
                    "(0x887A0006=HUNG, 0x887A0005=REMOVED, 0x887A0007=RESET) - the GPU faulted, everything after this is meaningless",
                    g_presentBeginCount, removedReason);
            }
        }

        if (verboseLog) {
            AddonLog_Printf("D3D12SwapchainPresentBegin #%u: submitted copy into slot=%d (flushUnlock=%d, fenceValue=%llu, completed=%llu, %ux%u format=%d)",
                g_presentBeginCount, backSlot, flushed ? 1 : 0,
                static_cast<unsigned long long>(pendingValue),
                static_cast<unsigned long long>(g_flushFence ? g_flushFence->GetCompletedValue() : 0),
                g_frameWidth, g_frameHeight, static_cast<int>(g_frameFormat));
        }

    }

    void D3D12SwapchainPresentEnd(UInt32 adapterID, const PresentEndContextInput& iCtx) override
    {
        // Nothing to do here for this bridge.
    }
};

RE5VRAddonObserver g_observer;

} // namespace

// --- Consumer-facing query API (called from RE5VR's own d3d9.dll via ---
// --- GetModuleHandle("SampleAddon.dll") + GetProcAddress - see       ---
// --- src/vr/d3d12_addon_bridge.cpp in the main project).             ---
//
// Deliberately NOT added to d3d9.dll's own export table for this - that
// table is carefully hand-matched to a real d3d9.dll's exports (see
// RE5VR.def's own header comment / real_d3d9.cpp's Phase 0 findings on
// RE5's anti-tamper export-table probing) and adding an arbitrary new
// export there risks breaking that property. Exporting from this DLL
// instead costs nothing - dgVoodoo only ever calls AddOnInit/AddOnExit
// on it, nothing else inspects its export table shape.
extern "C" {

bool API_EXPORT RE5VRAddon_GetFrameInfo(UInt32* outWidth, UInt32* outHeight, Int32* outDxgiFormat,
    void** outHandleSlot0, void** outHandleSlot1)
{
    if (!g_texturesReady)
        return false;
    if (outWidth) *outWidth = g_frameWidth;
    if (outHeight) *outHeight = g_frameHeight;
    if (outDxgiFormat) *outDxgiFormat = static_cast<Int32>(g_frameFormat);
    if (outHandleSlot0) *outHandleSlot0 = g_sharedHandle[0];
    if (outHandleSlot1) *outHandleSlot1 = g_sharedHandle[1];
    return true;
}

// See g_sharedGeneration. 0 until the first set of shared textures exists.
UInt32 API_EXPORT RE5VRAddon_GetSharedGeneration()
{
    return g_sharedGeneration.load(std::memory_order_acquire);
}

Int32 API_EXPORT RE5VRAddon_GetFrontSlot()
{
    return g_frontSlot.load(std::memory_order_acquire);
}

// Consumer calls this right after reading RE5VRAddon_GetFrontSlot, before
// touching the slot's texture, and must call RE5VRAddon_EndReadSlot with
// the SAME slot value once it's done (both eye copies issued) - see
// g_slotReaders' comment for why this exists.
void API_EXPORT RE5VRAddon_BeginReadSlot(Int32 slot)
{
    if (slot >= 0 && slot < kNumSlots)
        g_slotReaders[slot].fetch_add(1, std::memory_order_acq_rel);
}

void API_EXPORT RE5VRAddon_EndReadSlot(Int32 slot)
{
    if (slot >= 0 && slot < kNumSlots)
        g_slotReaders[slot].fetch_sub(1, std::memory_order_acq_rel);
}

// Has the GPU actually finished writing this slot? A slot is published
// as front the moment its copy is SUBMITTED, so the consumer must ask
// this before reading it. Non-blocking by design: a false answer means
// "not yet, come back next frame", never "wait here".
bool API_EXPORT RE5VRAddon_IsSlotReady(Int32 slot)
{
    if (slot < 0 || slot >= kNumSlots || !g_flushFence)
        return false;

    const UINT64 required = g_slotFenceValue[slot].load(std::memory_order_acquire);
    if (required == 0)
        return false; // nothing published into this slot yet

    return g_flushFence->GetCompletedValue() >= required;
}

// d3d9.dll, every frame at Present: show this region of the game's frame in
// the window instead of the whole frame (enabled), or leave it alone. See
// "Desktop view" above.
void API_EXPORT RE5VRAddon_SetDesktopView(bool enabled, Int32 x, Int32 y, Int32 w, Int32 h)
{
    g_dvX.store(x, std::memory_order_release);
    g_dvY.store(y, std::memory_order_release);
    g_dvW.store(w, std::memory_order_release);
    g_dvH.store(h, std::memory_order_release);
    g_dvEnabled.store(enabled, std::memory_order_release);
}

bool API_EXPORT AddOnInit(IAddonMainCallback* pAddonMainCB)
{
    AddonLog_Init();
    AddonLog_Printf("AddOnInit called (pAddonMainCB=%p)", pAddonMainCB);

    g_addonMainCB = pAddonMainCB;

    UInt32 version = pAddonMainCB->GetVersion();
    AddonLog_Printf("dgVoodoo addon API version: %u.%u%u",
        (version >> 8) & 0xF, (version >> 4) & 0xF, (version >> 0) & 0xF);

    bool registered = pAddonMainCB->RegisterForCallback(IID_D3D12RootObserver, &g_observer);
    AddonLog_Printf("RegisterForCallback(IID_D3D12RootObserver) -> %s", registered ? "true" : "false");

    return true;
}

void API_EXPORT AddOnExit()
{
    AddonLog_Printf("AddOnExit called");
    ReleaseSharedTextures();
    if (g_addonMainCB)
        g_addonMainCB->UnregisterForCallback(IID_D3D12RootObserver, &g_observer);
    g_addonMainCB = nullptr;
    g_d3d12Root = nullptr;
    g_d3d12Device = nullptr;
}

} // extern "C"
