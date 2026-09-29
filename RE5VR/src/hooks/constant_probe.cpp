#include "constant_probe.h"
#include "camera_rig_hook.h"
#include "fade_probe.h"
#include "../render/stereo_test.h"
#include "../render/bone_palette.h"
#include "../render/bone_palette.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <array>
#include <cstring>

namespace {

// Same stable vtable slot numbering used in d3d9_hooks.cpp.
constexpr size_t kIDirect3DDevice9_SetTransform = 44;
constexpr size_t kIDirect3DDevice9_SetVertexShaderConstantF = 94;
constexpr size_t kIDirect3DDevice9_DrawIndexedPrimitive = 82;

// At this game's uncapped framerate (500-1000+ fps observed), a single
// captured frame is too easy to land on a trivial/HUD-only pass or a frame
// where the engine skipped re-uploading unchanged constants. Capture a
// whole window of frames instead and aggregate.
constexpr int kCaptureWindowFrames = 600;

void* VTableEntry(void* pInterface, size_t index)
{
    void** vtable = *reinterpret_cast<void***>(pInterface);
    return vtable[index];
}

struct RegisterEntry {
    UINT count = 0;
    float lastValues[16] = {};
};

std::array<RegisterEntry, 256> g_tallyF;
int g_framesRemainingInCapture = 0;

// Empirical validation: c0 (the register found via capture, believed to be
// RE5's per-frame view/view-projection matrix) has its component [3] (the
// translation-like term of that row) nudged by a large, obvious amount
// while this is toggled on. If the camera visibly shifts/shears on screen,
// that confirms both that this register genuinely feeds the render output
// and roughly what a positional nudge to it does - without first needing
// to fully decompose the matrix's exact mathematical convention.
bool g_offsetTestEnabled = false;
constexpr float kOffsetTestAmount = 800.0f;

// The most recently observed *true* value of c0 (register 0, 4 vectors),
// cached unconditionally regardless of capture-window/offset-test state,
// so stereo_test.cpp can build per-eye variants from a known-good baseline
// instead of drifting off of its own previous override.
float g_cachedCameraMatrix[16] = {};
bool g_haveCachedCameraMatrix = false;

// Where the camera matrix (c0-c3) is uploaded FROM (2026-09-11, culling
// hunt). Forcing the model sphere test and every occlusion query visible
// still left VR holes in level geometry, and nothing but the sphere test
// reads the main camera's frustum planes - so whatever culls the level must
// work from the renderer's own view-projection. Its address is the source
// pointer of these uploads; the F5 watch then finds its readers. Render
// thread only (all D3D9 calls), so no locking.
struct MatrixSource {
    const void* ptr;
    unsigned count;
};
constexpr int kMaxMatrixSources = 16;
MatrixSource g_matrixSources[kMaxMatrixSources];

void RecordCameraMatrixSource(const void* p)
{
    int freeSlot = -1, minSlot = 0;
    for (int i = 0; i < kMaxMatrixSources; ++i) {
        if (g_matrixSources[i].ptr == p) {
            ++g_matrixSources[i].count;
            return;
        }
        if (!g_matrixSources[i].ptr && freeSlot < 0)
            freeSlot = i;
        if (g_matrixSources[i].count < g_matrixSources[minSlot].count)
            minSlot = i;
    }
    const int slot = freeSlot >= 0 ? freeSlot : minSlot;
    g_matrixSources[slot].ptr = p;
    g_matrixSources[slot].count = 1;
}

typedef HRESULT(WINAPI* SetTransform_t)(IDirect3DDevice9* This, D3DTRANSFORMSTATETYPE State, const D3DMATRIX* pMatrix);
SetTransform_t oSetTransform = nullptr;

HRESULT WINAPI hkSetTransform(IDirect3DDevice9* This, D3DTRANSFORMSTATETYPE State, const D3DMATRIX* pMatrix)
{
    if (g_framesRemainingInCapture > 0 && pMatrix) {
        const float* m = &pMatrix->_11;
        Log_Printf("ConstantProbe: SetTransform State=%d [%.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f]",
            static_cast<int>(State), m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
    }
    return oSetTransform(This, State, pMatrix);
}

typedef HRESULT(WINAPI* SetVertexShaderConstantF_t)(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount);

// HOW THE CHARACTER IS SPLIT UP (2026-09-26, the roadmap: "measure the
// player's draw calls").
//
// Hiding parts of the body has now failed four ways, and every one of them
// failed because it was writing something. The lever we have never pulled is
// simply not drawing what we do not want - nothing written anywhere, nothing
// to leak into the eye or the partner, nothing to restore because the next
// frame just draws it again.
//
// Whether that can work at all comes down to one measurement: how many draws
// the character is made of, and what each one covers. If the torso and the
// arms are separate draws it is the clean answer to Arms and Hands both. If
// the whole character is one draw it cannot separate them and we keep the
// careful approach.
//
// So this counts, and says nothing about which draw is whose. A bone table is
// uploaded immediately before the draw that uses it, so the pairing is just
// "whatever was uploaded last".
typedef HRESULT(WINAPI* DrawIndexedPrimitive_t)(IDirect3DDevice9* This, D3DPRIMITIVETYPE Type, INT BaseVertexIndex,
    UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount);
DrawIndexedPrimitive_t oDrawIndexedPrimitive = nullptr;

unsigned g_tableJustUploaded = 0; // bones in the table set for the next draw
bool g_tableHadNaN = false;       // and whether any of it was a hidden bone
unsigned long g_drawsSkipped = 0;

// SKIP IT RATHER THAN SEND IT (2026-09-26).
//
// Hiding puts NaN into the skeleton, the game builds its bone table from that,
// and the NaN arrives here on its way to the shader. It works - the hardware
// throws away any triangle holding one - but it means every frame we hand the
// driver a pile of arithmetic that cannot produce a number, and the log has a
// read fault inside dgVoodoo in the middle of a draw.
//
// A table with a NaN in it can only belong to a part of the model we are
// hiding, because the only NaN in the skeleton is the one we put there. So the
// draw can simply not happen. Same result on screen, nothing undefined handed
// to the driver, and it is the first half of doing this properly: eventually
// the decision moves here entirely and nothing is written to the skeleton at
// all.
bool LooksHidden(const float* data, unsigned count)
{
    const unsigned floats = count * 4;
    for (unsigned i = 0; i < floats; ++i)
        if (data[i] != data[i])
            return true;
    return false;
}

struct DrawKind {
    unsigned bones;
    unsigned verts;
    unsigned prims;
    unsigned long seen;
};
constexpr int kMaxDrawKinds = 40;
DrawKind g_kinds[kMaxDrawKinds];
int g_kindCount = 0;
unsigned long long g_kindsToldAt = 0;
unsigned long g_skinnedDraws = 0;

void NoteSkinnedDraw(unsigned bones, unsigned verts, unsigned prims)
{
    ++g_skinnedDraws;
    for (int i = 0; i < g_kindCount; ++i) {
        if (g_kinds[i].bones == bones && g_kinds[i].verts == verts && g_kinds[i].prims == prims) {
            ++g_kinds[i].seen;
            return;
        }
    }
    if (g_kindCount >= kMaxDrawKinds)
        return;
    g_kinds[g_kindCount].bones = bones;
    g_kinds[g_kindCount].verts = verts;
    g_kinds[g_kindCount].prims = prims;
    g_kinds[g_kindCount].seen = 1;
    ++g_kindCount;
}
SetVertexShaderConstantF_t oSetVertexShaderConstantF = nullptr;

// ---- Finding the bone palette (2026-09-26) ----------------------------
// See constant_probe.h. Two halves: a tally of every large upload, so the
// table can be found rather than guessed at, and a register range that gets
// zeroed on its way to the shader, so what each part of the table draws can
// be seen by making it vanish.
struct BigUpload {
    UINT start, count;
    unsigned long seen;
};
BigUpload g_big[16];
int g_bigCount = 0;
unsigned long long g_bigToldAt = 0;

volatile LONG g_pokeOn = 0;
volatile LONG g_pokeFrom = 0;
volatile LONG g_pokeTo = 0;

void NoteBigUpload(UINT start, UINT count)
{
    for (int i = 0; i < g_bigCount; ++i) {
        if (g_big[i].start == start && g_big[i].count == count) {
            ++g_big[i].seen;
            return;
        }
    }
    if (g_bigCount < 16) {
        g_big[g_bigCount].start = start;
        g_big[g_bigCount].count = count;
        g_big[g_bigCount].seen = 1;
        ++g_bigCount;
    }
}

HRESULT WINAPI hkSetVertexShaderConstantF(IDirect3DDevice9* This, UINT StartRegister, const float* pConstantData, UINT Vector4fCount)
{
    FadeProbe_OnSetVertexShaderConstantF(StartRegister, pConstantData, Vector4fCount);

    if (StartRegister == 23 && pConstantData && Vector4fCount >= 3 && Vector4fCount % 3 == 0) {
        g_tableJustUploaded = Vector4fCount / 3;
        g_tableHadNaN = LooksHidden(pConstantData, Vector4fCount);

        // AND OUT AGAIN, WITH PROOF (2026-09-28).
        //
        // Wiring BonePalette_Filter in for one build was worth it for the
        // one line it printed:
        //
        //   a table of 28 bone(s). Your joints run 5189 157 -3891 ...
        //     entry 0 translation 2321.8 -71.1 -3392.1
        //
        // Nearly three thousand units apart. The palette holds skinning
        // matrices - inverse bind times world - so their translations are
        // not world positions and cannot be matched against joints at all.
        // It hid 676 entries out of 47,000 by coincidence, which is exactly
        // what "arms isn\x27t collapsing where we want" looks like.
        //
        // That is why the module was never called. It is not a wiring
        // oversight, it is an approach that cannot work, and the hiding
        // belongs back in the skeleton where the hierarchy is known. See
        // hooks/arm_ik.cpp: collapse, not NaN.
    }

    // Anything big enough to be a skinning table. A camera matrix is four
    // registers; a palette is dozens.
    if (Vector4fCount >= 12 && pConstantData) {
        NoteBigUpload(StartRegister, Vector4fCount);
        const unsigned long long nowBig = GetTickCount64();
        if (nowBig - g_bigToldAt > 5000) {
            g_bigToldAt = nowBig;
            for (int i = 0; i < g_bigCount; ++i) {
                Log_Printf("BoneTable: uploads of %u register(s) from c%u - %lu time(s)", g_big[i].count,
                    g_big[i].start, g_big[i].seen);
                g_big[i].seen = 0;
            }
        }
    }

    // The poke. Zeroes a range of registers on their way to the shader, on
    // every model that uses them - so a bone hidden this way goes missing on
    // your partner too, which is exactly what makes it obvious.
    if (g_pokeOn && pConstantData && Vector4fCount <= 256) {
        const int from = g_pokeFrom, to = g_pokeTo;
        const int first = static_cast<int>(StartRegister);
        const int last = first + static_cast<int>(Vector4fCount) - 1;
        if (to >= first && from <= last) {
            static float s_poked[256 * 4];
            std::memcpy(s_poked, pConstantData, sizeof(float) * 4 * Vector4fCount);
            for (int r = from > first ? from : first; r <= (to < last ? to : last); ++r)
                std::memset(s_poked + (r - first) * 4, 0, sizeof(float) * 4);
            return oSetVertexShaderConstantF(This, StartRegister, s_poked, Vector4fCount);
        }
    }

    // NOT HERE (2026-09-27, user: "the camera constantly flickers, this was not
    // a good change - and the whip still happens anyways").
    //
    // Register 0 is not the player camera. It is whatever matrix the pass being
    // drawn needs - shadow, reflection, UI - so holding it against one
    // remembered direction limits every pass against the player facing, and
    // poisons that remembered direction several times a frame in the process.
    // Hence the flicker, and very likely why the whip survived too.
    //
    // Both modes passing through a point does not make it one camera, which is
    // the thing I did not check before building on it.
    if (StartRegister == 0 && Vector4fCount == 4 && pConstantData) {
        std::memcpy(g_cachedCameraMatrix, pConstantData, sizeof(g_cachedCameraMatrix));
        g_haveCachedCameraMatrix = true;
        RecordCameraMatrixSource(pConstantData);
    }

    if (g_framesRemainingInCapture > 0 && Vector4fCount == 4 && StartRegister < g_tallyF.size() && pConstantData) {
        RegisterEntry& e = g_tallyF[StartRegister];
        e.count++;
        std::memcpy(e.lastValues, pConstantData, sizeof(e.lastValues));
    }

    // The near plane, for flat screen as well as VR (2026-09-17). It was only
    // being applied where stereo_test rebuilds the per-eye matrices, so it did
    // nothing without a headset - and a flat screen player in first person can
    // look down into their own chest just as easily. This is the same matrix,
    // caught on its way to the shader for every draw: z/w is 1 + K/d for a
    // point d ahead, so the near distance is that term negated. stereo_test
    // overwrites register 0 for its own eyes afterwards and carries the same
    // value, so the two never fight.
    const float nearUnits = StereoTest_GetNearPlaneUnits();
    if (nearUnits > 0.0f && !StereoTest_IsEnabled() && StartRegister == 0 && Vector4fCount == 4 && pConstantData
        && CameraRigHook_IsEnabled()) {
        float modified[16];
        std::memcpy(modified, pConstantData, sizeof(modified));
        modified[11] = modified[15] - nearUnits;
        return oSetVertexShaderConstantF(This, StartRegister, modified, Vector4fCount);
    }

    if (g_offsetTestEnabled && StartRegister == 0 && Vector4fCount == 4 && pConstantData) {
        float modified[16];
        std::memcpy(modified, pConstantData, sizeof(modified));
        modified[3] += kOffsetTestAmount;
        return oSetVertexShaderConstantF(This, StartRegister, modified, Vector4fCount);
    }

    // And the body IS hidden here after all (2026-09-26). Hiding it in the
    // skeleton black-screened the game: NaN in a joint world matrix is read
    // back by everything in this mod that uses one. Here it cannot be, because
    // this data goes to the GPU and nowhere else.
    //
    // The identification problem that sank five earlier attempts is gone too.
    // Nothing here has to know WHICH bone a table entry is - arm_ik works the
    // keep set out from the hierarchy, and each entry carries its own world
    // position, so matching one to the nearest of your joints is all that is
    // left to do.
    // AND BACK OUT AGAIN (2026-09-26, user: "I think we just manually assign
    // to the bones like we did before, like why can't we just snap off the
    // elbows and the wrists").
    //
    // Right, and the diversion was mine. The keep set was never what was
    // wrong; writing it into the skeleton was, and coming here was an attempt
    // to dodge that rather than fix it. What it cost is that these matrices
    // are not world matrices - the log has your torso missing by tens to
    // hundreds of units while the weapon, parented at your hand, matched
    // perfectly - so a position test can name a bone near the model origin and
    // nothing else.
    //
    // The bones know which bone they are. Hiding is back on the skeleton, in
    // arm_ik.cpp, with the leak plugged instead of avoided.

    // The old note, kept because the reasoning still holds for the skeleton
    // route: editing the shader's bone table
    // needed to know which entry was which bone, and five attempts at that
    // failed; hiding is done to the skeleton's world matrices instead, late
    // in the frame, in arm_ik.cpp. Nothing in this file has to know anything
    // about bodies any more.

    return oSetVertexShaderConstantF(This, StartRegister, pConstantData, Vector4fCount);
}

HRESULT WINAPI hkDrawIndexedPrimitive(IDirect3DDevice9* This, D3DPRIMITIVETYPE Type, INT BaseVertexIndex,
    UINT MinVertexIndex, UINT NumVertices, UINT startIndex, UINT primCount)
{
    if (g_tableJustUploaded) {
        NoteSkinnedDraw(g_tableJustUploaded, NumVertices, primCount);
        g_tableJustUploaded = 0;
        if (g_tableHadNaN) {
            g_tableHadNaN = false;
            ++g_drawsSkipped;
            return D3D_OK;
        }
    }
    return oDrawIndexedPrimitive(This, Type, BaseVertexIndex, MinVertexIndex, NumVertices, startIndex, primCount);
}

void DumpCapture()
{
    // A register written roughly once (or a couple times) per frame across
    // the whole window is a global/camera candidate; one written dozens of
    // times per frame (per-object world matrices, bone palettes) will have
    // accumulated a much larger count over the same window.
    const UINT candidateThreshold = kCaptureWindowFrames * 2;

    for (size_t reg = 0; reg < g_tallyF.size(); ++reg) {
        const RegisterEntry& e = g_tallyF[reg];
        if (e.count == 0)
            continue;

        if (e.count <= candidateThreshold) {
            const float* m = e.lastValues;
            Log_Printf("ConstantProbe: c%zu count=%u [%.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f]",
                reg, e.count, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
        } else {
            Log_Printf("ConstantProbe: c%zu count=%u (skipping values - likely per-object)", reg, e.count);
        }
    }
    Log_Printf("ConstantProbe: === capture complete ===");
}

} // namespace

void ConstantProbe_Install(IDirect3DDevice9* pDevice)
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("ConstantProbe_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    struct HookSpec {
        size_t slot;
        void* detour;
        void** original;
        const char* name;
    };

    HookSpec specs[] = {
        { kIDirect3DDevice9_SetTransform,            reinterpret_cast<void*>(&hkSetTransform),            reinterpret_cast<void**>(&oSetTransform),            "SetTransform" },
        { kIDirect3DDevice9_SetVertexShaderConstantF, reinterpret_cast<void*>(&hkSetVertexShaderConstantF), reinterpret_cast<void**>(&oSetVertexShaderConstantF), "SetVertexShaderConstantF" },
    };

    for (const auto& spec : specs) {
        void* pTarget = VTableEntry(pDevice, spec.slot);
        MH_STATUS st = MH_CreateHook(pTarget, spec.detour, spec.original);
        if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
            Log_Printf("ConstantProbe_Install: MH_CreateHook(%s) failed -> %d", spec.name, static_cast<int>(st));
            continue;
        }
        st = MH_EnableHook(pTarget);
        Log_Printf("ConstantProbe_Install: %s hook enabled -> %d", spec.name, static_cast<int>(st));
    }
}

void ConstantProbe_OnEndScene()
{
    // The draw census. Sorted biggest first, because the character's body is
    // the heaviest skinned thing on screen and whatever sits at the top of this
    // list is almost certainly him.
    {
        const unsigned long long now = GetTickCount64();
        if (g_kindCount && now - g_kindsToldAt > 5000) {
            g_kindsToldAt = now;
            Log_Printf("DrawCensus: %lu skinned draw(s) in the last 5 s, in %d distinct shape(s), %lu skipped as "
                       "hidden. Biggest first:",
                g_skinnedDraws, g_kindCount, g_drawsSkipped);
            g_drawsSkipped = 0;
            // A small selection sort over at most forty entries, once every
            // five seconds. Not worth being clever about.
            for (int shown = 0; shown < g_kindCount && shown < 14; ++shown) {
                int best = -1;
                for (int i = shown; i < g_kindCount; ++i)
                    if (best < 0 || g_kinds[i].verts > g_kinds[best].verts)
                        best = i;
                const DrawKind swap = g_kinds[shown];
                g_kinds[shown] = g_kinds[best];
                g_kinds[best] = swap;
                Log_Printf("DrawCensus:   %u bone(s), %u vertices, %u triangles - drawn %lu time(s)",
                    g_kinds[shown].bones, g_kinds[shown].verts, g_kinds[shown].prims, g_kinds[shown].seen);
            }
            g_kindCount = 0;
            g_skinnedDraws = 0;
        }
    }

    static bool prevF9Down = false;
    bool f9Down = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
    if (f9Down && !prevF9Down && g_framesRemainingInCapture == 0) {
        g_framesRemainingInCapture = kCaptureWindowFrames;
        for (auto& e : g_tallyF)
            e.count = 0;
        Log_Printf("ConstantProbe: F9 pressed, capturing next %d frames", kCaptureWindowFrames);
    }
    prevF9Down = f9Down;

    if (g_framesRemainingInCapture > 0) {
        --g_framesRemainingInCapture;
        if (g_framesRemainingInCapture == 0)
            DumpCapture();
    }

    static bool prevF10Down = false;
    bool f10Down = (GetAsyncKeyState(VK_F10) & 0x8000) != 0;
    if (f10Down && !prevF10Down) {
        g_offsetTestEnabled = !g_offsetTestEnabled;
        Log_Printf("ConstantProbe: F10 pressed, offset test now %s", g_offsetTestEnabled ? "ON" : "OFF");
    }
    prevF10Down = f10Down;

    // F6: dump the current cached c0 matrix (16 floats) on demand, so it
    // can be captured at different known camera orientations/positions and
    // diffed by hand. Needed to figure out c0's actual layout (row-major
    // vs column-major, which slots are rotation basis vectors vs
    // translation) before real head-tracked rotation can be implemented -
    // we currently only know component [3] acts as a translation-like
    // term when nudged, which isn't enough to compose a rotation into it.
    static bool prevF6Down = false;
    bool f6Down = (GetAsyncKeyState(VK_F6) & 0x8000) != 0;
    if (f6Down && !prevF6Down) {
        if (g_haveCachedCameraMatrix) {
            const float* m = g_cachedCameraMatrix;
            Log_Printf("ConstantProbe: F6 snapshot c0 [%.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f]",
                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
        } else {
            Log_Printf("ConstantProbe: F6 pressed but no cached camera matrix yet");
        }
    }
    prevF6Down = f6Down;
}

bool ConstantProbe_GetCachedCameraMatrix(float out[16])
{
    if (!g_haveCachedCameraMatrix)
        return false;
    std::memcpy(out, g_cachedCameraMatrix, sizeof(g_cachedCameraMatrix));
    return true;
}

void ConstantProbe_SetVertexPoke(bool on, int fromRegister, int toRegister)
{
    InterlockedExchange(&g_pokeFrom, fromRegister);
    InterlockedExchange(&g_pokeTo, toRegister);
    InterlockedExchange(&g_pokeOn, on ? 1 : 0);
    Log_Printf("BoneTable: poke %s, registers c%d to c%d", on ? "on" : "off", fromRegister, toRegister);
}

void ConstantProbe_GetVertexPoke(bool& on, int& fromRegister, int& toRegister)
{
    on = g_pokeOn != 0;
    fromRegister = g_pokeFrom;
    toRegister = g_pokeTo;
}

HRESULT ConstantProbe_CallRealSetVertexShaderConstantF(IDirect3DDevice9* pDevice, UINT StartRegister, const float* pConstantData, UINT Vector4fCount)
{
    if (!oSetVertexShaderConstantF)
        return E_FAIL;
    return oSetVertexShaderConstantF(pDevice, StartRegister, pConstantData, Vector4fCount);
}

int ConstantProbe_GetCameraMatrixSources(const void** outPtrs, unsigned* outCounts, int maxCount)
{
    bool taken[kMaxMatrixSources] = {};
    int n = 0;
    while (n < maxCount) {
        int best = -1;
        for (int i = 0; i < kMaxMatrixSources; ++i) {
            if (!taken[i] && g_matrixSources[i].ptr && (best < 0 || g_matrixSources[i].count > g_matrixSources[best].count))
                best = i;
        }
        if (best < 0)
            break;
        taken[best] = true;
        outPtrs[n] = g_matrixSources[best].ptr;
        outCounts[n] = g_matrixSources[best].count;
        ++n;
    }
    return n;
}
