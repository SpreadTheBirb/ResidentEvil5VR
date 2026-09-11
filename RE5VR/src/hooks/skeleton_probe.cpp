#include "skeleton_probe.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstring>

// ---- Why this exists ---------------------------------------------------
// First-person head hiding (head_hide_probe.cpp, July) skips whole draw
// calls recognised by texture/vertex-buffer descriptors. Every part of Chris
// shares one vertex buffer, so it is guesswork: in practice it hides his
// head, hair AND hands, and leaves a black inner mesh behind. The user only
// ever wanted the head gone - the hands are needed for arm IK later.
//
// The robust way, and what that IK will need anyway, is the skeleton:
// collapse the head joint and everything weighted to it disappears, hands
// untouched. It also gives the real eye position (the fixed 173-unit eye
// height looks too high) and Chris's height in game units.
//
// '\' finds it without any hand-searching. From each character object the
// camera controllers follow ([controller+0x140]) and that character's world
// position (the controller's transform, translation at controller+0x260):
// follow every pointer in the object and keep any pointed-to block that
// contains several 4x4 matrices positioned on the character - world-space
// (near the character) or model-space (near the origin). A joint array is
// dozens of those at a regular stride. Everything goes to re5vr_skel.bin for
// offline analysis (scratchpad skeleton.ps1).

namespace {

constexpr DWORD kOffNormalMiddleRig = 0x4B0;  // live NORMAL base = controller + this
constexpr DWORD kOffControllerTarget = 0x140; // controller -> followed character
constexpr DWORD kOffControllerCharPos = 0x260; // translation row of the controller's normal transform
constexpr DWORD kObjBytes = 0x8000;
constexpr DWORD kProbeBytes = 0x4000; // how far into a pointed-to block to look for matrices
constexpr DWORD kDumpBytes = 0x8000;  // how much of a hit block to save
constexpr int kMinMatrices = 8;

BYTE g_obj[kObjBytes];
BYTE g_blk[kDumpBytes];

#pragma pack(push, 1)
struct SkelHeader {
    char magic[4];  // "OBJ " or "BLK "
    DWORD object;   // character object this belongs to
    DWORD address;  // OBJ: object itself; BLK: the pointed-to block
    DWORD fieldOff; // BLK: offset in the object where the pointer was found
    DWORD worldHits, modelHits;
    float charPos[3];
    DWORD size;
};
#pragma pack(pop)

void GameFolderPath(const char* fileName, char* out, size_t outSize)
{
    char exePath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    char drive[_MAX_DRIVE], dir[_MAX_DIR];
    _splitpath_s(exePath, drive, sizeof(drive), dir, sizeof(dir), nullptr, 0, nullptr, 0);
    _snprintf_s(out, outSize, _TRUNCATE, "%s%s%s", drive, dir, fileName);
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

// Cheap pre-check so garbage "pointers" don't each cost a fault.
bool Readable(uintptr_t p)
{
    MEMORY_BASIC_INFORMATION mbi = {};
    if (!VirtualQuery(reinterpret_cast<const void*>(p), &mbi, sizeof(mbi)))
        return false;
    return mbi.State == MEM_COMMIT && mbi.Protect != 0 && !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS));
}

float Len3(const float* v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Row-major D3D-style affine matrix: rows 0-2 basis, row 3 translation.
bool IsAffine(const float* m)
{
    if (std::fabs(m[15] - 1.0f) > 1e-3f)
        return false;
    if (std::fabs(m[3]) > 1e-3f || std::fabs(m[7]) > 1e-3f || std::fabs(m[11]) > 1e-3f)
        return false;
    for (int r = 0; r < 3; ++r) {
        const float l = Len3(&m[r * 4]);
        if (!(l > 0.01f && l < 10.0f))
            return false;
    }
    return true;
}

void CountMatrices(const BYTE* blk, DWORD size, const float* charPos, DWORD* world, DWORD* model)
{
    *world = *model = 0;
    for (DWORD off = 0; off + 64 <= size; off += 4) {
        const float* m = reinterpret_cast<const float*>(blk + off);
        if (!IsAffine(m))
            continue;
        const float dx = m[12] - charPos[0], dy = m[13] - charPos[1], dz = m[14] - charPos[2];
        if (std::fabs(dx) < 250.0f && std::fabs(dz) < 250.0f && dy > -50.0f && dy < 300.0f)
            ++*world;
        else if (std::fabs(m[12]) < 150.0f && std::fabs(m[14]) < 150.0f && m[13] > -50.0f && m[13] < 300.0f &&
                 Len3(&m[12]) > 1.0f)
            ++*model;
    }
}

void Capture()
{
    void* bases[32];
    bool aim[32];
    const int n = CameraRigHook_GetLiveBases(bases, aim, 32);

    char path[MAX_PATH];
    GameFolderPath("re5vr_skel.bin", path, sizeof(path));
    HANDLE f = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) {
        Log_Printf("SkeletonProbe: could not create re5vr_skel.bin (error %lu)", GetLastError());
        return;
    }

    DWORD done[8] = {};
    int doneCount = 0;
    for (int i = 0; i < n && doneCount < 8; ++i) {
        if (aim[i])
            continue;
        const uintptr_t controller = reinterpret_cast<uintptr_t>(bases[i]) - kOffNormalMiddleRig;
        DWORD object = 0;
        float charPos[3] = {};
        if (!TryCopy(&object, reinterpret_cast<const void*>(controller + kOffControllerTarget), sizeof(object)) || !object)
            continue;
        if (!TryCopy(charPos, reinterpret_cast<const void*>(controller + kOffControllerCharPos), sizeof(charPos)))
            continue;
        bool dup = false;
        for (int d = 0; d < doneCount; ++d)
            dup |= done[d] == object;
        if (dup)
            continue;
        done[doneCount++] = object;

        SafeCopyRegion(g_obj, object, kObjBytes);
        SkelHeader h = {};
        std::memcpy(h.magic, "OBJ ", 4);
        h.object = h.address = object;
        std::memcpy(h.charPos, charPos, sizeof(charPos));
        h.size = kObjBytes;
        DWORD w = 0;
        WriteFile(f, &h, sizeof(h), &w, nullptr);
        WriteFile(f, g_obj, kObjBytes, &w, nullptr);

        int blocks = 0;
        DWORD bestWorld = 0, bestAt = 0, bestOff = 0;
        for (DWORD off = 0; off + 4 <= kObjBytes; off += 4) {
            DWORD p;
            std::memcpy(&p, g_obj + off, sizeof(p));
            if (p < 0x00400000 || p >= 0x7FFF0000 || (p & 3) || p == object || !Readable(p))
                continue;
            SafeCopyRegion(g_blk, p, kProbeBytes);
            DWORD world = 0, model = 0;
            CountMatrices(g_blk, kProbeBytes, charPos, &world, &model);
            if (world < kMinMatrices && model < kMinMatrices)
                continue;
            SafeCopyRegion(g_blk, p, kDumpBytes);
            SkelHeader b = {};
            std::memcpy(b.magic, "BLK ", 4);
            b.object = object;
            b.address = p;
            b.fieldOff = off;
            b.worldHits = world;
            b.modelHits = model;
            std::memcpy(b.charPos, charPos, sizeof(charPos));
            b.size = kDumpBytes;
            WriteFile(f, &b, sizeof(b), &w, nullptr);
            WriteFile(f, g_blk, kDumpBytes, &w, nullptr);
            ++blocks;
            if (world > bestWorld) {
                bestWorld = world;
                bestAt = p;
                bestOff = off;
            }
        }
        Log_Printf("SkeletonProbe: character %08lX at (%.1f, %.1f, %.1f): %d matrix block(s); most world-space "
                   "matrices: %lu in block %08lX (object+0x%lX)",
            object, charPos[0], charPos[1], charPos[2], blocks, bestWorld, bestAt, bestOff);
    }
    CloseHandle(f);
    Log_Printf("SkeletonProbe: capture complete - %d character(s) -> re5vr_skel.bin", doneCount);
}

} // namespace

void SkeletonProbe_OnEndScene()
{
    static bool prevDown = false;
    const bool down = (GetAsyncKeyState(VK_OEM_5) & 0x8000) != 0; // '\'
    if (down && !prevDown) {
        Log_Printf("SkeletonProbe: '\\' pressed - scanning character objects for joint arrays");
        Capture();
    }
    prevDown = down;
}
