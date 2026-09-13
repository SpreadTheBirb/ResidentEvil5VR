#include "hud_probe.h"
#include "constant_probe.h"
#include "../util/log.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

// ---- Why this exists (2026-09-13) ---------------------------------------
// The HUD is fine flat and missing in VR. The first theory - HUD draws use a
// pixel-space ortho matrix, so squeeze them into each eye - matched ZERO draws
// in practice, which means nobody actually knows which draws are the HUD.
//
// The community trainer's "No HUD" option answers that. A code capture with
// it off and on (statediff, 2026-09-13) showed exactly three bytes, all in the
// HUD's update logic around a thiscall at exe+5F8CF0:
//   exe+5F8D19  cmp byte [esi+1ABh], 1   ->  cmp ..., 0   (gates a large block)
//   exe+5F8EB0  je                       ->  jmp          (skips one element)
//   exe+5F9125  mov byte [esi+344h], 1   ->  mov ..., 0   (a "show" flag)
//
// So the HUD can be switched off in-process, the same way the laser and colour
// filter patches work, and whatever draw calls vanish when it is off ARE the
// HUD - identified by what the draws are, not by a guess about their matrices.
// Recording which stereo path each of those draws takes then says directly
// what VR is doing to them.
//
// One K press runs the whole sequence, so there is nothing to get wrong:
//   wait a few frames -> record one frame (HUD on) -> patch HUD off ->
//   wait for the HUD to go -> record one frame (HUD off) -> restore -> report.
// Draws are recorded once per game draw, before stereo duplicates them.

namespace {

struct PatchSite {
    DWORD rva;
    BYTE original;
    BYTE hudOff;
    const char* what;
};
constexpr PatchSite kSites[] = {
    { 0x5F8D19, 0x01, 0x00, "cmp byte [esi+1ABh], 1 -> 0" },
    { 0x5F8EB0, 0x74, 0xEB, "je -> jmp" },
    { 0x5F9125, 0x01, 0x00, "mov byte [esi+344h], 1 -> 0" },
};

constexpr int kSettleFramesOn = 5;   // before the HUD-on frame
constexpr int kSettleFramesOff = 45; // after patching, for the HUD to actually go
constexpr size_t kMaxDrawsPerFrame = 8192;

struct DrawRec {
    int kind;
    int primType;
    int path;
    UINT primCount;
    void* rt;
    UINT rtW, rtH;
    bool rtIsBackbuffer;
    void* vs;
    void* ps;
    void* tex0;
    void* decl;
    DWORD fvf;
    D3DVIEWPORT9 vp;
    DWORD scissorOn, zOn, blendOn; // 0xFFFFFFFF = could not read
    RECT scissor;
    bool haveMatrix;
    float c0len, c3[4];
};

enum class State { Idle, SettleOn, RecordOn, SettleOff, RecordOff };

// The HUD's shader pairs, learned from the last K press. Pointers are only
// valid for this launch - the bytecode hash logged next to them is what a
// permanent version has to match on.
constexpr int kMaxHudShaders = 8;
void* g_hudVs[kMaxHudShaders] = {};
void* g_hudPs[kMaxHudShaders] = {};
int g_hudShaderCount = 0;

bool g_verified = false;
State g_state = State::Idle;
int g_frames = 0;
bool g_recording = false;
std::vector<DrawRec> g_current, g_on, g_off;
bool g_prevKey = false;

BYTE* SiteAddr(const PatchSite& s)
{
    return reinterpret_cast<BYTE*>(GetModuleHandleA(nullptr)) + s.rva;
}

void WriteByte(BYTE* at, BYTE value)
{
    DWORD oldProtect = 0;
    VirtualProtect(at, 1, PAGE_EXECUTE_READWRITE, &oldProtect);
    *at = value;
    VirtualProtect(at, 1, oldProtect, &oldProtect);
    FlushInstructionCache(GetCurrentProcess(), at, 1);
}

void SetHudOff(bool off)
{
    for (const PatchSite& s : kSites)
        WriteByte(SiteAddr(s), off ? s.hudOff : s.original);
    Log_Printf("HudProbe: HUD patched %s", off ? "OFF" : "back ON (original bytes)");
}

const char* KindName(int k)
{
    static const char* names[] = { "DP", "DIP", "DPUP", "DIPUP" };
    return k >= 0 && k < 4 ? names[k] : "?";
}

const char* PathName(int p)
{
    switch (p) {
    case kStereoPathOff: return "stereo-off (one untouched draw)";
    case kStereoPathNoMatrix: return "no-camera-matrix (one untouched draw)";
    case kStereoPathOffscreen: return "offscreen-target (one untouched draw)";
    case kStereoPathScreenSpace: return "screen-space squeeze (per eye)";
    case kStereoPathNudged: return "fallback nudge (per eye)";
    case kStereoPathCamera: return "per-eye camera";
    case kStereoPathHeadSkip: return "skipped by head-hide";
    case kStereoPathHudViewport: return "HUD half-viewport (per eye)";
    default: return "?";
    }
}

// Identity of a draw for diffing: what it is, not how often it happens.
struct Key {
    int kind, primType, path;
    void* rt, *vs, *ps, *tex0, *decl;
    DWORD fvf;
    bool operator<(const Key& o) const
    {
        return std::memcmp(this, &o, sizeof(Key)) < 0;
    }
    bool operator==(const Key& o) const { return std::memcmp(this, &o, sizeof(Key)) == 0; }
};

Key KeyOf(const DrawRec& r)
{
    Key k;
    std::memset(&k, 0, sizeof(k));
    k.kind = r.kind;
    k.primType = r.primType;
    k.path = r.path;
    k.rt = r.rt;
    k.vs = r.vs;
    k.ps = r.ps;
    k.tex0 = r.tex0;
    k.decl = r.decl;
    k.fvf = r.fvf;
    return k;
}

void WriteReport()
{
    char path[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    char* slash = strrchr(path, 0x5C);
    if (!slash)
        return;
    strcpy_s(slash + 1, MAX_PATH - (slash + 1 - path), "re5vr_hud_draws.txt");
    FILE* f = nullptr;
    if (fopen_s(&f, path, "w") != 0 || !f) {
        Log_Printf("HudProbe: could not write %s", path);
        return;
    }

    struct Agg {
        Key key;
        int on = 0, off = 0;
        const DrawRec* sample = nullptr;
        UINT minPrims = 0xFFFFFFFF, maxPrims = 0;
    };
    std::vector<Agg> aggs;
    auto find = [&](const Key& k) -> Agg& {
        for (Agg& a : aggs)
            if (a.key == k)
                return a;
        aggs.push_back(Agg{});
        aggs.back().key = k;
        return aggs.back();
    };
    for (const DrawRec& r : g_on) {
        Agg& a = find(KeyOf(r));
        ++a.on;
        if (!a.sample)
            a.sample = &r;
        a.minPrims = (std::min)(a.minPrims, r.primCount);
        a.maxPrims = (std::max)(a.maxPrims, r.primCount);
    }
    for (const DrawRec& r : g_off)
        ++find(KeyOf(r)).off;

    std::vector<Agg*> gone;
    for (Agg& a : aggs)
        if (a.on > a.off)
            gone.push_back(&a);
    std::sort(gone.begin(), gone.end(), [](const Agg* x, const Agg* y) {
        return (x->on - x->off) > (y->on - y->off);
    });

    int pathCounts[8] = {};
    int goneDraws = 0;
    for (const Agg* a : gone) {
        goneDraws += a->on - a->off;
        if (a->key.path >= 0 && a->key.path < 8)
            pathCounts[a->key.path] += a->on - a->off;
    }

    fprintf(f, "RE5VR HUD draw recorder\n=======================\n\n");
    fprintf(f, "Frame with HUD on:  %zu draws\n", g_on.size());
    fprintf(f, "Frame with HUD off: %zu draws\n", g_off.size());
    fprintf(f, "Draw kinds that disappeared (the HUD): %zu kinds, %d draws\n\n", gone.size(), goneDraws);
    fprintf(f, "What stereo did to the HUD draws:\n");
    for (int p = 0; p < 8; ++p)
        if (pathCounts[p])
            fprintf(f, "  %4d  %s\n", pathCounts[p], PathName(p));
    fprintf(f, "\n");

    for (const Agg* a : gone) {
        const DrawRec& r = *a->sample;
        fprintf(f, "---- %d on / %d off  %s prims %u-%u  type %d\n", a->on, a->off, KindName(r.kind),
            a->minPrims, a->maxPrims, r.primType);
        fprintf(f, "  stereo path: %s\n", PathName(r.path));
        fprintf(f, "  target: %p %ux%u%s\n", r.rt, r.rtW, r.rtH, r.rtIsBackbuffer ? " (BACKBUFFER)" : "");
        fprintf(f, "  vs %p  ps %p  tex0 %p  decl %p  fvf 0x%08lX%s\n", r.vs, r.ps, r.tex0, r.decl, r.fvf,
            (r.fvf & D3DFVF_POSITION_MASK) == D3DFVF_XYZRHW ? " (XYZRHW - pretransformed)" : "");
        fprintf(f, "  viewport %lu,%lu %lux%lu  scissor %s %ld,%ld-%ld,%ld  z %s  blend %s\n", r.vp.X, r.vp.Y,
            r.vp.Width, r.vp.Height,
            r.scissorOn == 0xFFFFFFFF ? "?" : (r.scissorOn ? "ON" : "off"), r.scissor.left, r.scissor.top,
            r.scissor.right, r.scissor.bottom,
            r.zOn == 0xFFFFFFFF ? "?" : (r.zOn ? "on" : "off"),
            r.blendOn == 0xFFFFFFFF ? "?" : (r.blendOn ? "on" : "off"));
        if (r.haveMatrix)
            fprintf(f, "  c0-c3 at draw: |c0| %.5f  c3 (%.4f %.4f %.4f %.4f)\n", r.c0len, r.c3[0], r.c3[1], r.c3[2],
                r.c3[3]);
        else
            fprintf(f, "  c0-c3 at draw: none cached\n");
    }

    fprintf(f, "\n\nFor reference - the HUD-on frame in draw order (first 400):\n");
    size_t n = 0;
    for (const DrawRec& r : g_on) {
        if (n++ >= 400)
            break;
        fprintf(f, "  %4zu %-5s p%-5u rt %p %ux%u%s  vs %p ps %p tex %p fvf %08lX  %s\n", n, KindName(r.kind),
            r.primCount, r.rt, r.rtW, r.rtH, r.rtIsBackbuffer ? "*" : " ", r.vs, r.ps, r.tex0, r.fvf,
            PathName(r.path));
    }
    fclose(f);

    Log_Printf("HudProbe: report -> %s (HUD on %zu draws, off %zu, %zu kind(s) / %d draw(s) disappeared)", path,
        g_on.size(), g_off.size(), gone.size(), goneDraws);
    for (int p = 0; p < 8; ++p)
        if (pathCounts[p])
            Log_Printf("HudProbe:   %d HUD draw(s) went through %s", pathCounts[p], PathName(p));
}

// FNV-1a over a shader's bytecode: stable across launches, unlike its pointer.
template <typename Shader>
unsigned long ShaderHash(void* p, UINT* outSize)
{
    *outSize = 0;
    if (!p)
        return 0;
    Shader* s = static_cast<Shader*>(p);
    UINT size = 0;
    if (FAILED(s->GetFunction(nullptr, &size)) || !size || size > (1u << 20))
        return 0;
    std::vector<BYTE> code(size);
    if (FAILED(s->GetFunction(code.data(), &size)))
        return 0;
    unsigned long h = 2166136261ul;
    for (BYTE b : code) {
        h ^= b;
        h *= 16777619ul;
    }
    *outSize = size;
    return h;
}

// Which of the draws that vanished are really the HUD: the ones stereo was
// DUPLICATING. The rest of the diff is frame-to-frame jitter that happens to
// land on one side - the luminance downsample chain (off-screen, drawn once)
// and our own diagnostics quad (stereo suppressed) - neither of which the
// HUD fix should touch.
bool IsDuplicatedPath(int path)
{
    return path == kStereoPathCamera || path == kStereoPathScreenSpace || path == kStereoPathNudged ||
        path == kStereoPathHudViewport;
}

// Judged per SHADER PAIR over the whole frame, not per draw type. The first
// version learned any draw type that drew more with the HUD on than off, and
// ordinary frame-to-frame jitter qualified: 8 pairs learned, 7 of them wrong,
// including a 1914-triangle character mesh that "drew once more" in the HUD-on
// frame - its skin got moved into the HUD panel and Chris and Sheva rendered
// as black silhouettes (user, 2026-09-13). The real HUD pair went 16 -> 0.
//
// So a pair is the HUD only if:
//  - it essentially stops drawing when the HUD is off (off <= on / 2, and at
//    least kHudMinDrop fewer): a scene shader draws dozens of times and wobbles
//    by one;
//  - every draw it makes is small (kHudMaxPrims): HUD pieces are quads and
//    short strips, meshes are thousands of triangles;
//  - stereo was duplicating it (otherwise there is nothing to fix).
// Each K press replaces the learned set rather than adding to it, so a bad
// capture can be corrected by pressing K again.
constexpr int kHudMinDrop = 3;
constexpr UINT kHudMaxPrims = 200;

void LearnHudShaders(const std::vector<DrawRec>& on, const std::vector<DrawRec>& off, FILE* report)
{
    struct Pair {
        void* vs;
        void* ps;
        int on = 0, off = 0;
        UINT maxPrims = 0;
        bool duplicated = false;
    };
    std::vector<Pair> pairs;
    auto find = [&](void* vs, void* ps) -> Pair& {
        for (Pair& p : pairs)
            if (p.vs == vs && p.ps == ps)
                return p;
        pairs.push_back(Pair{});
        pairs.back().vs = vs;
        pairs.back().ps = ps;
        return pairs.back();
    };
    for (const DrawRec& r : on) {
        if (!r.vs || !r.ps)
            continue;
        Pair& p = find(r.vs, r.ps);
        ++p.on;
        p.maxPrims = (std::max)(p.maxPrims, r.primCount);
        if (IsDuplicatedPath(r.path))
            p.duplicated = true;
    }
    for (const DrawRec& r : off) {
        if (r.vs && r.ps)
            ++find(r.vs, r.ps).off;
    }

    g_hudShaderCount = 0;
    int nearMisses = 0;
    for (const Pair& p : pairs) {
        const bool dropped = p.on - p.off >= kHudMinDrop && p.off * 2 <= p.on;
        if (!dropped || !p.duplicated)
            continue;
        if (p.maxPrims > kHudMaxPrims) {
            ++nearMisses;
            Log_Printf("HudProbe: rejected vs %p / ps %p - it dropped %d -> %d but draws up to %u triangles, "
                       "which is a mesh, not HUD",
                p.vs, p.ps, p.on, p.off, p.maxPrims);
            continue;
        }
        if (g_hudShaderCount >= kMaxHudShaders)
            break;
        g_hudVs[g_hudShaderCount] = p.vs;
        g_hudPs[g_hudShaderCount] = p.ps;
        ++g_hudShaderCount;

        UINT vsSize = 0, psSize = 0;
        const unsigned long vsHash = ShaderHash<IDirect3DVertexShader9>(p.vs, &vsSize);
        const unsigned long psHash = ShaderHash<IDirect3DPixelShader9>(p.ps, &psSize);
        Log_Printf("HudProbe: HUD shader pair learned - %d draws -> %d with the HUD off, up to %u triangles - "
                   "vs %p (hash %08lX, %u bytes)  ps %p (hash %08lX, %u bytes)",
            p.on, p.off, p.maxPrims, p.vs, vsHash, vsSize, p.ps, psHash, psSize);
        if (report)
            fprintf(report, "HUD shader pair: vs %p hash %08lX (%u bytes)   ps %p hash %08lX (%u bytes)\n", p.vs,
                vsHash, vsSize, p.ps, psHash, psSize);
    }
    if (g_hudShaderCount)
        Log_Printf("HudProbe: %d HUD shader pair(s) now drawn per eye (replacing anything learned before)",
            g_hudShaderCount);
    else
        Log_Printf("HudProbe: no shader pair stopped drawing with the HUD off (%d rejected as meshes) - "
                   "nothing learned; is VR on?",
            nearMisses);
}
DWORD ReadState(IDirect3DDevice9* d, D3DRENDERSTATETYPE s)
{
    DWORD v = 0;
    return SUCCEEDED(d->GetRenderState(s, &v)) ? v : 0xFFFFFFFF;
}

} // namespace

void HudProbe_Install()
{
    g_verified = true;
    for (const PatchSite& s : kSites) {
        const BYTE have = *SiteAddr(s);
        if (have != s.original) {
            Log_Printf("HudProbe: exe+%06lX holds 0x%02X, expected 0x%02X (%s) - wrong game build or already "
                       "patched (trainer?); HUD recorder disabled",
                s.rva, have, s.original, s.what);
            g_verified = false;
        }
    }
    if (g_verified)
        Log_Printf("HudProbe: HUD bytes verified at exe+5F8D19/+5F8EB0/+5F9125 - press K to record the HUD");
}

void HudProbe_OnPresent()
{
    const bool key = (GetAsyncKeyState('K') & 0x8000) != 0;
    const bool pressed = key && !g_prevKey;
    g_prevKey = key;

    switch (g_state) {
    case State::Idle:
        if (!pressed)
            return;
        if (!g_verified) {
            Log_Printf("HudProbe: K pressed but the HUD bytes did not verify at startup - nothing to do");
            return;
        }
        Log_Printf("HudProbe: K pressed - recording a HUD-on frame, then a HUD-off frame (~1 s, keep still)");
        g_on.clear();
        g_off.clear();
        g_frames = 0;
        g_state = State::SettleOn;
        return;
    case State::SettleOn:
        if (++g_frames >= kSettleFramesOn) {
            g_current.clear();
            g_recording = true;
            g_state = State::RecordOn;
        }
        return;
    case State::RecordOn:
        // Present ends the recorded frame.
        g_recording = false;
        g_on.swap(g_current);
        SetHudOff(true);
        g_frames = 0;
        g_state = State::SettleOff;
        return;
    case State::SettleOff:
        if (++g_frames >= kSettleFramesOff) {
            g_current.clear();
            g_recording = true;
            g_state = State::RecordOff;
        }
        return;
    case State::RecordOff:
        g_recording = false;
        g_off.swap(g_current);
        SetHudOff(false);
        WriteReport();
        LearnHudShaders(g_on, g_off, nullptr); // hashes go to re5vr.log
        g_state = State::Idle;
        return;
    }
}

void HudProbe_OnDraw(IDirect3DDevice9* device, int kind, D3DPRIMITIVETYPE primType, UINT primCount, int stereoPath)
{
    if (!g_recording || g_current.size() >= kMaxDrawsPerFrame)
        return;

    DrawRec r;
    std::memset(&r, 0, sizeof(r));
    r.kind = kind;
    r.primType = static_cast<int>(primType);
    r.path = stereoPath;
    r.primCount = primCount;

    IDirect3DSurface9* rt = nullptr;
    if (SUCCEEDED(device->GetRenderTarget(0, &rt)) && rt) {
        r.rt = rt;
        D3DSURFACE_DESC d = {};
        if (SUCCEEDED(rt->GetDesc(&d))) {
            r.rtW = d.Width;
            r.rtH = d.Height;
        }
        IDirect3DSurface9* bb = nullptr;
        if (SUCCEEDED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) {
            r.rtIsBackbuffer = bb == rt;
            bb->Release();
        }
        rt->Release();
    }
    IDirect3DVertexShader9* vs = nullptr;
    if (SUCCEEDED(device->GetVertexShader(&vs)) && vs) {
        r.vs = vs;
        vs->Release();
    }
    IDirect3DPixelShader9* ps = nullptr;
    if (SUCCEEDED(device->GetPixelShader(&ps)) && ps) {
        r.ps = ps;
        ps->Release();
    }
    IDirect3DBaseTexture9* tex = nullptr;
    if (SUCCEEDED(device->GetTexture(0, &tex)) && tex) {
        r.tex0 = tex;
        tex->Release();
    }
    IDirect3DVertexDeclaration9* decl = nullptr;
    if (SUCCEEDED(device->GetVertexDeclaration(&decl)) && decl) {
        r.decl = decl;
        decl->Release();
    }
    device->GetFVF(&r.fvf);
    device->GetViewport(&r.vp);
    r.scissorOn = ReadState(device, D3DRS_SCISSORTESTENABLE);
    device->GetScissorRect(&r.scissor);
    r.zOn = ReadState(device, D3DRS_ZENABLE);
    r.blendOn = ReadState(device, D3DRS_ALPHABLENDENABLE);

    float m[16];
    if (ConstantProbe_GetCachedCameraMatrix(m)) {
        r.haveMatrix = true;
        r.c0len = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        std::memcpy(r.c3, &m[12], sizeof(r.c3));
    }
    g_current.push_back(r);
}

bool HudProbe_IsHudDraw(IDirect3DDevice9* device)
{
    if (!g_hudShaderCount)
        return false;
    IDirect3DVertexShader9* vs = nullptr;
    IDirect3DPixelShader9* ps = nullptr;
    device->GetVertexShader(&vs);
    device->GetPixelShader(&ps);
    if (vs)
        vs->Release();
    if (ps)
        ps->Release();
    for (int i = 0; i < g_hudShaderCount; ++i)
        if (g_hudVs[i] == vs && g_hudPs[i] == ps)
            return true;
    return false;
}
