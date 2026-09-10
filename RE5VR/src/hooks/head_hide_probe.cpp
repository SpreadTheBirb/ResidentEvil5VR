#include "head_hide_probe.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <windows.h>
#include <array>

namespace {

constexpr int kCaptureWindowFrames = 300;

// Real per-frame unique draw-call signature counts easily exceeded the
// original 64-slot table (1500+ overflow misses observed) - RE5 submits
// on the order of a few hundred unique (texture, vertex buffer) pairs
// per frame between environment, characters, and effects. Sized
// generously so a capture is complete rather than an arbitrary early
// subset.
constexpr int kMaxSignatures = 2048;

struct DrawSignature {
    void* texture = nullptr;
    void* vertexBuffer = nullptr;
    UINT count = 0;
    UINT lastPrimCount = 0;
    UINT lastStride = 0;
    // Stable-across-launches identifying info, queried once when a new
    // signature is first recorded. Raw texture/vertex-buffer pointers
    // are only valid for one game session (fresh D3D resource addresses
    // every launch) - these describe the asset itself instead, so a
    // confirmed candidate can be re-matched on a future launch by shape
    // rather than by address.
    UINT texWidth = 0;
    UINT texHeight = 0;
    D3DFORMAT texFormat = D3DFMT_UNKNOWN;
    UINT vbSize = 0;
    DWORD vbFVF = 0;
    bool bookmarked = false;
};

// Queried once per newly-recorded signature, not per draw call - GetType/
// QueryInterface/GetLevelDesc/GetDesc are too costly to run on every draw
// during a capture window. Relies on the texture/vertex buffer still
// being alive via the device's own current binding or the game's asset
// ownership, same assumption HeadHideHook_ShouldSkip already makes when
// comparing raw pointers after their probe-side reference was released.
void QueryStableDescriptors(void* texture, void* vertexBuffer, DrawSignature& sig)
{
    if (texture) {
        IDirect3DBaseTexture9* baseTex = static_cast<IDirect3DBaseTexture9*>(texture);
        if (baseTex->GetType() == D3DRTYPE_TEXTURE) {
            IDirect3DTexture9* tex2D = nullptr;
            if (SUCCEEDED(baseTex->QueryInterface(__uuidof(IDirect3DTexture9), reinterpret_cast<void**>(&tex2D))) && tex2D) {
                D3DSURFACE_DESC desc;
                if (SUCCEEDED(tex2D->GetLevelDesc(0, &desc))) {
                    sig.texWidth = desc.Width;
                    sig.texHeight = desc.Height;
                    sig.texFormat = desc.Format;
                }
                tex2D->Release();
            }
        }
    }
    if (vertexBuffer) {
        IDirect3DVertexBuffer9* vb = static_cast<IDirect3DVertexBuffer9*>(vertexBuffer);
        D3DVERTEXBUFFER_DESC vbDesc;
        if (SUCCEEDED(vb->GetDesc(&vbDesc))) {
            sig.vbSize = vbDesc.Size;
            sig.vbFVF = vbDesc.FVF;
        }
    }
}

std::array<DrawSignature, kMaxSignatures> g_signatures;
int g_filledCount = 0;
int g_signatureOverflow = 0;
int g_framesRemainingInCapture = 0;

int g_selectedIndex = -1;
bool g_skipEnabled = false;

// Permanent, stable-across-launches identification of draw calls to hide
// in first-person. The first four entries are the head mesh's sub-draws,
// found via the F11/F1-F3 probe workflow (2026-07-27) and confirmed clean
// in first-person with nothing else visibly missing. All four share the
// same vertex buffer size (one skinned head model, split into
// per-material sub-draws - most likely skin, eyes/teeth, hair, and brows
// or similar) but were captured as separate signatures since each binds a
// different texture. Matched by texture dimensions/format and vertex
// buffer byte size rather than raw D3D resource pointers, which are only
// valid for one game session - texFormat values are legitimate D3DFORMAT
// FourCC codes (827611204 = DXT1, 894720068 = DXT5), not arbitrary noise.
//
// REVERTED (2026-07-27): a 5th entry for the untextured "black mesh"
// (tex=0, vbSize=985312) was added and deployed, but turned out to be the
// same draw call as the character's own HAND geometry - hiding it left
// the held pistol floating with no hands gripping it. The (0,0,unknown,
// 985312) descriptor is too broad / actually identifies the hands, not a
// standalone artifact - do not re-add without a more specific match (e.g.
// primitive/vertex count, or distinguishing it by draw order relative to
// a hand-specific texture) if this needs revisiting.
struct StableHeadPartMatch {
    UINT texWidth;
    UINT texHeight;
    D3DFORMAT texFormat;
    UINT vbSize;
};

constexpr StableHeadPartMatch kHeadParts[] = {
    { 256, 256, static_cast<D3DFORMAT>(827611204), 985312 }, // candidate 91
    { 128, 128, static_cast<D3DFORMAT>(894720068), 985312 }, // candidate 93
    { 64,  128, static_cast<D3DFORMAT>(827611204), 985312 }, // candidate 96
    { 128, 64,  static_cast<D3DFORMAT>(827611204), 985312 }, // candidate 100
};

bool MatchesKnownHeadPart(UINT texW, UINT texH, D3DFORMAT fmt, UINT vbSize)
{
    for (const auto& part : kHeadParts) {
        if (part.texWidth == texW && part.texHeight == texH &&
            part.texFormat == fmt && part.vbSize == vbSize)
            return true;
    }
    return false;
}

// Per-session cache keyed by raw (texture, vertex buffer) pointers, which
// - unlike the stable descriptors above - ARE valid for comparison within
// one game session. Avoids re-running GetType/QueryInterface/
// GetLevelDesc/GetDesc on every single draw call once a given pair has
// been resolved once; this path runs unconditionally whenever the F4
// override is on; a raw per-draw descriptor query would be real overhead
// across a few hundred draws/frame.
struct PermanentCacheEntry {
    void* texture = nullptr;
    void* vertexBuffer = nullptr;
    bool matches = false;
};
constexpr int kPermanentCacheSize = 256;
std::array<PermanentCacheEntry, kPermanentCacheSize> g_permanentCache;
int g_permanentCacheCount = 0;

bool IsPermanentHeadPart(void* texture, void* vertexBuffer)
{
    for (int i = 0; i < g_permanentCacheCount; ++i) {
        if (g_permanentCache[i].texture == texture && g_permanentCache[i].vertexBuffer == vertexBuffer)
            return g_permanentCache[i].matches;
    }

    DrawSignature descriptors;
    QueryStableDescriptors(texture, vertexBuffer, descriptors);
    const bool matches = MatchesKnownHeadPart(descriptors.texWidth, descriptors.texHeight,
        descriptors.texFormat, descriptors.vbSize);

    if (g_permanentCacheCount < kPermanentCacheSize) {
        PermanentCacheEntry& entry = g_permanentCache[g_permanentCacheCount++];
        entry.texture = texture;
        entry.vertexBuffer = vertexBuffer;
        entry.matches = matches;
    }
    return matches;
}

void LogSelected()
{
    if (g_selectedIndex < 0 || g_filledCount == 0) {
        Log_Printf("HeadHideProbe: no candidate selected (capture with F11 first)");
        return;
    }
    const DrawSignature& sig = g_signatures[g_selectedIndex];
    Log_Printf("HeadHideProbe: candidate %d/%d%s  tex=%p vb=%p count=%u lastPrimCount=%u lastStride=%u  texWH=%ux%u texFmt=%d vbSize=%u vbFVF=0x%08lX",
        g_selectedIndex + 1, g_filledCount, sig.bookmarked ? " [BOOKMARKED]" : "",
        sig.texture, sig.vertexBuffer,
        sig.count, sig.lastPrimCount, sig.lastStride,
        sig.texWidth, sig.texHeight, static_cast<int>(sig.texFormat), sig.vbSize, sig.vbFVF);
}

// Called from F3 - separate from LogSelected's routine per-step logging
// so bookmark events are easy to find and re-read afterward (searching
// the log for "BOOKMARK" gives just the parts of interest, not every
// candidate stepped past along the way).
void LogBookmarkChange(const DrawSignature& sig, int index)
{
    Log_Printf("HeadHideProbe: %s candidate %d/%d  tex=%p vb=%p texWH=%ux%u texFmt=%d vbSize=%u vbFVF=0x%08lX",
        sig.bookmarked ? "BOOKMARKED" : "UNBOOKMARKED", index + 1, g_filledCount,
        sig.texture, sig.vertexBuffer,
        sig.texWidth, sig.texHeight, static_cast<int>(sig.texFormat), sig.vbSize, sig.vbFVF);
}

void RecordSignature(void* texture, void* vertexBuffer, UINT primCount, UINT stride)
{
    for (int i = 0; i < g_filledCount; ++i) {
        DrawSignature& sig = g_signatures[i];
        if (sig.texture == texture && sig.vertexBuffer == vertexBuffer) {
            sig.count++;
            sig.lastPrimCount = primCount;
            sig.lastStride = stride;
            return;
        }
    }
    if (g_filledCount < kMaxSignatures) {
        DrawSignature& sig = g_signatures[g_filledCount++];
        sig.texture = texture;
        sig.vertexBuffer = vertexBuffer;
        sig.count = 1;
        sig.lastPrimCount = primCount;
        sig.lastStride = stride;
        QueryStableDescriptors(texture, vertexBuffer, sig);
        return;
    }
    g_signatureOverflow++;
}

} // namespace

void HeadHideProbe_OnEndScene()
{
    static bool prevF11Down = false;
    bool f11Down = (GetAsyncKeyState(VK_F11) & 0x8000) != 0;
    if (f11Down && !prevF11Down && g_framesRemainingInCapture == 0) {
        g_framesRemainingInCapture = kCaptureWindowFrames;
        g_signatureOverflow = 0;
        g_filledCount = 0;
        g_selectedIndex = -1;
        Log_Printf("HeadHideProbe: F11 pressed, capturing next %d frames", kCaptureWindowFrames);
    }
    prevF11Down = f11Down;

    if (g_framesRemainingInCapture > 0) {
        --g_framesRemainingInCapture;
        if (g_framesRemainingInCapture == 0) {
            Log_Printf("HeadHideProbe: === capture complete (%d unique signatures, %d overflowed) ===",
                g_filledCount, g_signatureOverflow);
            g_selectedIndex = g_filledCount > 0 ? 0 : -1;
            g_skipEnabled = true;
            LogSelected();
            Log_Printf("HeadHideProbe: skip auto-enabled - F1/F2 step candidates and you'll see live effects immediately, F3 bookmarks the selected one (face, hair, etc. - hide multiple at once), F12 toggles skipping off/on");
        }
    }

    static bool prevF1Down = false;
    bool f1Down = (GetAsyncKeyState(VK_F1) & 0x8000) != 0;
    if (f1Down && !prevF1Down) {
        if (g_filledCount > 0) {
            g_selectedIndex = (g_selectedIndex - 1 + g_filledCount) % g_filledCount;
            LogSelected();
        } else {
            Log_Printf("HeadHideProbe: F1 pressed but there's no capture yet - press F11 first");
        }
    }
    prevF1Down = f1Down;

    static bool prevF2Down = false;
    bool f2Down = (GetAsyncKeyState(VK_F2) & 0x8000) != 0;
    if (f2Down && !prevF2Down) {
        if (g_filledCount > 0) {
            g_selectedIndex = (g_selectedIndex + 1) % g_filledCount;
            LogSelected();
        } else {
            Log_Printf("HeadHideProbe: F2 pressed but there's no capture yet - press F11 first");
        }
    }
    prevF2Down = f2Down;

    static bool prevF3Down = false;
    bool f3Down = (GetAsyncKeyState(VK_F3) & 0x8000) != 0;
    if (f3Down && !prevF3Down) {
        if (g_selectedIndex >= 0 && g_selectedIndex < g_filledCount) {
            DrawSignature& sig = g_signatures[g_selectedIndex];
            sig.bookmarked = !sig.bookmarked;
            LogBookmarkChange(sig, g_selectedIndex);
        } else {
            Log_Printf("HeadHideProbe: F3 pressed but there's no capture yet - press F11 first");
        }
    }
    prevF3Down = f3Down;

    static bool prevF12Down = false;
    bool f12Down = (GetAsyncKeyState(VK_F12) & 0x8000) != 0;
    if (f12Down && !prevF12Down) {
        g_skipEnabled = !g_skipEnabled;
        int bookmarkCount = 0;
        for (int i = 0; i < g_filledCount; ++i)
            if (g_signatures[i].bookmarked)
                bookmarkCount++;
        Log_Printf("HeadHideProbe: F12 pressed, skip now %s (%d candidate(s) captured, %d bookmarked)%s",
            g_skipEnabled ? "ON" : "OFF", g_filledCount, bookmarkCount,
            g_filledCount == 0 ? " - no capture yet, this won't do anything until you press F11" : "");
    }
    prevF12Down = f12Down;
}

void HeadHideProbe_OnDrawCall(IDirect3DDevice9* pDevice, const char* callType, UINT primCount)
{
    (void)callType;
    if (g_framesRemainingInCapture <= 0)
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

    RecordSignature(tex, vb, primCount, stride);
}

bool HeadHideHook_ShouldSkip(IDirect3DDevice9* pDevice)
{
    // Gated on the F4 near-first-person override so hidden parts (face,
    // hair, etc.) automatically reappear the instant you leave first-
    // person, instead of staying invisible in normal third-person play -
    // confirmed necessary 2026-07-27 (user saw a black/missing head
    // persist in third-person before this check was added).
    if (!CameraRigHook_IsEnabled())
        return false;

    IDirect3DBaseTexture9* tex = nullptr;
    pDevice->GetTexture(0, &tex);
    if (tex)
        tex->Release();

    IDirect3DVertexBuffer9* vb = nullptr;
    UINT offset = 0, stride = 0;
    pDevice->GetStreamSource(0, &vb, &offset, &stride);
    if (vb)
        vb->Release();

    // Permanent: the 4 confirmed head-mesh sub-draws, matched by stable
    // descriptor and cached by pointer for this session - always hidden
    // in first-person regardless of the diagnostic probe's state below.
    if (IsPermanentHeadPart(tex, vb))
        return true;

    if (!g_skipEnabled)
        return false;

    // Live preview: whatever's currently selected for browsing with
    // F1/F2, so stepping through candidates shows an immediate visual
    // result without needing to bookmark it first.
    if (g_selectedIndex >= 0 && g_selectedIndex < g_filledCount) {
        const DrawSignature& sel = g_signatures[g_selectedIndex];
        if (tex == sel.texture && vb == sel.vertexBuffer)
            return true;
    }

    // Accumulated: every part confirmed with F3 so far, so multiple
    // parts (face, hair, etc.) can be hidden together regardless of
    // which one is currently selected.
    for (int i = 0; i < g_filledCount; ++i) {
        const DrawSignature& sig = g_signatures[i];
        if (sig.bookmarked && tex == sig.texture && vb == sig.vertexBuffer)
            return true;
    }
    return false;
}

bool HeadHideProbe_GetSelectedSignature(void** outTexture, void** outVertexBuffer)
{
    if (g_selectedIndex < 0 || g_selectedIndex >= g_filledCount)
        return false;
    const DrawSignature& sel = g_signatures[g_selectedIndex];
    *outTexture = sel.texture;
    *outVertexBuffer = sel.vertexBuffer;
    return true;
}
