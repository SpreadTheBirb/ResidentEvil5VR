#include "fade_probe.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <windows.h>

#include <cstdio>
#include <cstring>

// ---- Why this exists ---------------------------------------------------
// RE5 fades any character that gets close to the camera: in first person
// Chris's body and gun go slightly transparent, and Sheva fades out too when
// she walks into the camera's space (user, 2026-09-11). So it is a generic
// per-character proximity fade, not something specific to the player.
//
// The July hunt (pixel_constant_probe.cpp) stalled for two reasons: it
// captured one hand-picked draw at a time through the F11/F1/F2 browser, and
// it never had an opaque-vs-faded pair to compare. F4 now provides the pair
// for free - third person, Chris is opaque; first person, he is faded - so
// this captures EVERYTHING about every draw in both states and leaves the
// comparison to an offline diff:
//   - bound vertex/pixel shader (a fade may swap in a dither/alpha variant)
//   - textures on all 8 stages (a dither pattern may be bound only when faded)
//   - alpha-test/blend/depth/colour-write render states
//   - the complete vertex and pixel shader constant files at draw time
//
// FIRST RESULT (2026-09-11 04:33, captures 1-3): the fade is a switch to an
// alpha-BLENDED pass - SRCBLEND ONE->SRCALPHA, DESTBLEND ZERO->INVSRCALPHA,
// ALPHAREF 8->0 - on every draw of the faded character (Chris's VB 985312
// bytes in capture 2; Sheva's 984160 in capture 3, with Chris's unchanged
// in all 104 of his draws as the control). July's "ALPHABLENDENABLE is 0"
// was a capture of a different pass.
//
// That run also exposed a flaw: de-duplicating by geometry alone kept only
// each mesh's FIRST draw of the frame, but characters are drawn several
// times (depth/shadow, then colour), so opaque and faded captures compared
// different passes. The key now includes shaders, render target and the
// blend/write states, so every pass is kept.
//
// Each capture also snapshots the character objects the camera controllers
// follow ([controller+0x140], see camera_rig_hook.cpp / the disassembly at
// re5dx9.exe+447116), so the per-character opacity the renderer acts on can
// be found as a field that drops from 1.0 when faded.

namespace {

constexpr UINT kVsRegs = 256;
constexpr UINT kPsRegs = 224; // ps_3_0 float constant count
constexpr unsigned long long kCaptureWindowMs = 150; // several frames at ~90fps
constexpr int kSeenSlots = 32768;                     // power of two

constexpr DWORD kOffControllerTarget = 0x140; // camera controller -> followed character
constexpr DWORD kOffNormalMiddleRig = 0x4B0;  // live-list NORMAL base = controller + this
constexpr DWORD kObjSnapBytes = 0x3000;

float g_vs[kVsRegs][4];
float g_ps[kPsRegs][4];

constexpr D3DRENDERSTATETYPE kStates[] = {
    D3DRS_ALPHABLENDENABLE, D3DRS_SRCBLEND, D3DRS_DESTBLEND, D3DRS_BLENDOP,
    D3DRS_SEPARATEALPHABLENDENABLE, D3DRS_SRCBLENDALPHA, D3DRS_DESTBLENDALPHA, D3DRS_BLENDOPALPHA,
    D3DRS_BLENDFACTOR, D3DRS_ALPHATESTENABLE, D3DRS_ALPHAREF, D3DRS_ALPHAFUNC,
    D3DRS_ZENABLE, D3DRS_ZWRITEENABLE, D3DRS_ZFUNC, D3DRS_CULLMODE,
    D3DRS_COLORWRITEENABLE, D3DRS_TEXTUREFACTOR, D3DRS_STENCILENABLE, D3DRS_STENCILFUNC,
    D3DRS_STENCILREF, D3DRS_SRGBWRITEENABLE, D3DRS_FOGENABLE, D3DRS_MULTISAMPLEMASK,
};
constexpr int kNumStates = sizeof(kStates) / sizeof(kStates[0]);
// Indices into kStates that distinguish passes of the same mesh.
constexpr int kStAlphaBlend = 0, kStSrcBlend = 1, kStDestBlend = 2, kStZWrite = 13, kStColorWrite = 16;

#pragma pack(push, 1)
struct DrawRecord {
    char magic[4]; // "DRAW"
    DWORD order;   // position among this capture's unique draws
    DWORD kind, primType, baseVertex, minIndex, numVertices, startIndex, primCount;
    DWORD vb, vbOffset, vbStride, vbSize, ib;
    DWORD tex[8];
    DWORD tex0Width, tex0Height, tex0Format;
    DWORD vs, ps, rt0;
    DWORD states[kNumStates];
    float vsConst[kVsRegs][4];
    float psConst[kPsRegs][4];
};

struct ObjHeader {
    char magic[4]; // "OBJ "
    DWORD controller;
    DWORD object;
    DWORD size;
};
#pragma pack(pop)

DrawRecord g_rec;
BYTE g_objBuf[kObjSnapBytes];
unsigned long long g_seen[kSeenSlots];
HANDLE g_capFile = INVALID_HANDLE_VALUE;
unsigned long long g_capUntilMs = 0;
int g_capIndex = 0;
DWORD g_capWritten = 0;

void GameFolderPath(const char* fileName, char* out, size_t outSize)
{
    char exePath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    char drive[_MAX_DRIVE], dir[_MAX_DIR];
    _splitpath_s(exePath, drive, sizeof(drive), dir, sizeof(dir), nullptr, 0, nullptr, 0);
    _snprintf_s(out, outSize, _TRUNCATE, "%s%s%s", drive, dir, fileName);
}

DWORD Ptr(const void* p)
{
    return static_cast<DWORD>(reinterpret_cast<uintptr_t>(p));
}

// True the first time a signature is seen in this capture.
bool MarkSeen(const DWORD* key, int n)
{
    unsigned long long h = 1469598103934665603ull;
    for (int i = 0; i < n; ++i) {
        h ^= key[i];
        h *= 1099511628211ull;
    }
    if (h == 0)
        h = 1;
    for (int probe = 0; probe < kSeenSlots; ++probe) {
        unsigned long long& slot = g_seen[(h + probe) & (kSeenSlots - 1)];
        if (slot == h)
            return false;
        if (slot == 0) {
            slot = h;
            return true;
        }
    }
    return false; // table full - stop recording rather than duplicate
}

void CopyConstants(float (*shadow)[4], UINT regs, UINT start, const float* data, UINT count)
{
    if (!data || start >= regs)
        return;
    if (count > regs - start)
        count = regs - start;
    std::memcpy(shadow[start], data, count * 4 * sizeof(float));
}

// Game heap we don't own - copy under SEH, a page at a time, zeroing
// anything that faults. No C++ objects here: MSVC forbids unwinding
// across __try.
bool TryCopy(void* dst, const void* src, size_t n)
{
    __try {
        std::memcpy(dst, src, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void SafeCopyRegion(BYTE* dst, uintptr_t src, size_t size)
{
    size_t done = 0;
    while (done < size) {
        const uintptr_t at = src + done;
        size_t chunk = 4096 - (at & 4095);
        if (chunk > size - done)
            chunk = size - done;
        if (!TryCopy(dst + done, reinterpret_cast<const void*>(at), chunk))
            std::memset(dst + done, 0, chunk);
        done += chunk;
    }
}

void SnapshotCharacters(int index)
{
    void* bases[32];
    bool aim[32];
    const int n = CameraRigHook_GetLiveBases(bases, aim, 32);

    char name[64];
    _snprintf_s(name, sizeof(name), _TRUNCATE, "re5vr_fadeobj_%d.bin", index);
    char path[MAX_PATH];
    GameFolderPath(name, path, sizeof(path));
    HANDLE f = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE)
        return;

    int written = 0;
    for (int i = 0; i < n; ++i) {
        if (aim[i])
            continue;
        const uintptr_t controller = reinterpret_cast<uintptr_t>(bases[i]) - kOffNormalMiddleRig;
        DWORD object = 0;
        if (!TryCopy(&object, reinterpret_cast<const void*>(controller + kOffControllerTarget), sizeof(object)) || !object)
            continue;
        SafeCopyRegion(g_objBuf, object, kObjSnapBytes);
        ObjHeader h = {};
        std::memcpy(h.magic, "OBJ ", 4);
        h.controller = static_cast<DWORD>(controller);
        h.object = object;
        h.size = kObjSnapBytes;
        DWORD w = 0;
        WriteFile(f, &h, sizeof(h), &w, nullptr);
        WriteFile(f, g_objBuf, sizeof(g_objBuf), &w, nullptr);
        Log_Printf("FadeProbe: character snapshot controller=%08lX object=%08lX", h.controller, h.object);
        ++written;
    }
    CloseHandle(f);
    Log_Printf("FadeProbe: %d character object(s) -> %s", written, name);
}

} // namespace

void FadeProbe_OnSetVertexShaderConstantF(UINT startRegister, const float* data, UINT vector4fCount)
{
    CopyConstants(g_vs, kVsRegs, startRegister, data, vector4fCount);
}

void FadeProbe_OnSetPixelShaderConstantF(UINT startRegister, const float* data, UINT vector4fCount)
{
    CopyConstants(g_ps, kPsRegs, startRegister, data, vector4fCount);
}

void FadeProbe_OnDraw(IDirect3DDevice9* pDevice, DWORD kind, D3DPRIMITIVETYPE primType, INT baseVertex,
    UINT minIndex, UINT numVertices, UINT startIndex, UINT primCount)
{
    if (g_capFile == INVALID_HANDLE_VALUE || GetTickCount64() >= g_capUntilMs)
        return;

    // Everything except the constant files goes into the header first: the
    // pass-distinguishing fields are needed for the de-dup key.
    DrawRecord& r = g_rec;
    std::memcpy(r.magic, "DRAW", 4);
    r.kind = kind;
    r.primType = static_cast<DWORD>(primType);
    r.baseVertex = static_cast<DWORD>(baseVertex);
    r.minIndex = minIndex;
    r.numVertices = numVertices;
    r.startIndex = startIndex;
    r.primCount = primCount;

    IDirect3DVertexBuffer9* vb = nullptr;
    UINT vbOffset = 0, vbStride = 0;
    pDevice->GetStreamSource(0, &vb, &vbOffset, &vbStride);
    r.vb = Ptr(vb);
    r.vbOffset = vbOffset;
    r.vbStride = vbStride;
    r.vbSize = 0;
    if (vb) {
        D3DVERTEXBUFFER_DESC d = {};
        if (SUCCEEDED(vb->GetDesc(&d)))
            r.vbSize = d.Size;
        vb->Release();
    }
    IDirect3DIndexBuffer9* ib = nullptr;
    pDevice->GetIndices(&ib);
    r.ib = Ptr(ib);
    if (ib)
        ib->Release();

    r.tex0Width = r.tex0Height = r.tex0Format = 0;
    for (DWORD s = 0; s < 8; ++s) {
        IDirect3DBaseTexture9* t = nullptr;
        pDevice->GetTexture(s, &t);
        r.tex[s] = Ptr(t);
        if (t) {
            if (s == 0 && t->GetType() == D3DRTYPE_TEXTURE) {
                D3DSURFACE_DESC d = {};
                if (SUCCEEDED(static_cast<IDirect3DTexture9*>(t)->GetLevelDesc(0, &d))) {
                    r.tex0Width = d.Width;
                    r.tex0Height = d.Height;
                    r.tex0Format = static_cast<DWORD>(d.Format);
                }
            }
            t->Release();
        }
    }

    IDirect3DVertexShader9* vs = nullptr;
    pDevice->GetVertexShader(&vs);
    r.vs = Ptr(vs);
    if (vs)
        vs->Release();
    IDirect3DPixelShader9* ps = nullptr;
    pDevice->GetPixelShader(&ps);
    r.ps = Ptr(ps);
    if (ps)
        ps->Release();
    IDirect3DSurface9* rt = nullptr;
    pDevice->GetRenderTarget(0, &rt);
    r.rt0 = Ptr(rt);
    if (rt)
        rt->Release();

    for (int i = 0; i < kNumStates; ++i) {
        DWORD v = 0;
        pDevice->GetRenderState(kStates[i], &v);
        r.states[i] = v;
    }

    const DWORD key[] = { r.kind, r.primType, r.baseVertex, r.minIndex, r.numVertices, r.startIndex,
        r.primCount, r.vb, r.vbOffset, r.ib, r.tex[0], r.vs, r.ps, r.rt0,
        r.states[kStAlphaBlend], r.states[kStSrcBlend], r.states[kStDestBlend],
        r.states[kStZWrite], r.states[kStColorWrite] };
    if (!MarkSeen(key, sizeof(key) / sizeof(key[0])))
        return;

    r.order = g_capWritten;
    std::memcpy(r.vsConst, g_vs, sizeof(g_vs));
    std::memcpy(r.psConst, g_ps, sizeof(g_ps));
    DWORD written = 0;
    WriteFile(g_capFile, &r, sizeof(r), &written, nullptr);
    ++g_capWritten;
}

void FadeProbe_OnEndScene()
{
    static bool prevDown = false;
    const bool down = (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000) != 0; // '='
    if (down && !prevDown && g_capFile == INVALID_HANDLE_VALUE) {
        ++g_capIndex;
        char name[64];
        _snprintf_s(name, sizeof(name), _TRUNCATE, "re5vr_fadecap_%d.bin", g_capIndex);
        char path[MAX_PATH];
        GameFolderPath(name, path, sizeof(path));
        SnapshotCharacters(g_capIndex);
        g_capFile = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (g_capFile == INVALID_HANDLE_VALUE) {
            Log_Printf("FadeProbe: could not create %s (error %lu)", name, GetLastError());
        } else {
            std::memset(g_seen, 0, sizeof(g_seen));
            g_capWritten = 0;
            g_capUntilMs = GetTickCount64() + kCaptureWindowMs;
            Log_Printf("FadeProbe: '=' pressed - capture %d started (%s, %llu ms, record %u bytes)",
                g_capIndex, name, kCaptureWindowMs, static_cast<unsigned>(sizeof(DrawRecord)));
        }
    }
    prevDown = down;

    if (g_capFile != INVALID_HANDLE_VALUE && GetTickCount64() >= g_capUntilMs) {
        CloseHandle(g_capFile);
        g_capFile = INVALID_HANDLE_VALUE;
        Log_Printf("FadeProbe: capture %d complete - %lu unique draw(s)", g_capIndex, g_capWritten);
    }
}
