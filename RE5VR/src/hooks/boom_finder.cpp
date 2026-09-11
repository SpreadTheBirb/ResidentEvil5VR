#include "boom_finder.h"
#include "camera_rig_hook.h"
#include "constant_probe.h"
#include "../util/log.h"

#include <windows.h>
#include <tlhelp32.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

// ---- Why this exists ---------------------------------------------------
// The first-person camera still pops out to third person when pitching.
// Measured 2026-09-10: a translation along a fixed world direction 19.6 deg
// above horizontal, magnitude growing with pitch - computed somewhere none
// of our rig-struct writes reach (all ruled out, see project memory).
// Finding that code by hand in Cheat Engine proved impractical, and the
// game's code can't be read from disk either: re5dx9.exe is wrapped by
// Steam's DRM (.bind section), so its code only exists decrypted in memory.
//
// One F5 press captures three things, so everything after it can be done
// offline without further game runs:
//
//  1. re5vr_codedump.bin - the exe's full in-memory image, decrypted, so
//     the camera code can be disassembled at leisure.
//
//  2. Hardware read/write watchpoints (the same debug-register mechanism
//     behind Cheat Engine's "find out what accesses this address") on the
//     live NORMAL rig struct's Distance/Vertical Distance fields for a few
//     seconds. The code that READS those is the camera-position calculation
//     the boom lives in. Logged as unique instruction addresses with hit
//     counts and the registers at first hit.
//
//  3. re5vr_camsnap.bin - the memory around each live NORMAL rig struct,
//     sampled every 100ms alongside the game's actual camera matrix. The
//     rig structs are embedded in a larger object (the Aim struct sits at
//     esi+0x420 in its hook stub, and NORMAL/AIM pairs are 0x90 apart), so
//     this is the camera controller. Correlating its fields against the real
//     camera pitch offline finds the pitch variable and the boom term.
//
// Lifetime rule from camera_rig_hook.cpp still applies: bases come only from
// the live list (hit within the last 250ms) and are never held for later.

namespace {

constexpr unsigned long long kSnapshotWindowMs = 12000;
constexpr unsigned long long kSnapshotEveryMs = 100;
constexpr DWORD kWatchWindowMs = 10000;
constexpr int kSnapBefore = 0x800;
constexpr int kSnapAfter = 0x1800;
constexpr int kMaxLive = 32;

// Same offsets as camera_rig_hook.cpp.
constexpr DWORD kOffVerticalDistance = 0x04;
constexpr DWORD kOffDistance = 0x08;
constexpr DWORD kOffVerticalAdjust = 0x14;
constexpr DWORD kOffAngle = 0x18;

// ---- Fade-flag watch mode (2026-09-11) ---------------------------------
// The pitch "boom" is solved, so F5's watchpoints now point at the near-
// camera fade instead. fade_probe.cpp's character-object snapshots (three
// captures: Chris opaque / Chris faded / Sheva faded) left exactly two
// fields that flip with the fade on BOTH characters and on nothing else:
//   character +0x304: 0.0 solid, 1.0 faded
//   character +0x308: 1.0 solid, 0.0 faded
// Watching them R/W finds the code that decides a character is too close
// to the camera (the writer) and the renderer that acts on it (readers).
// Characters are reached as [controller+0x140], the object the camera
// controller follows (disassembly at re5dx9.exe+447116).
constexpr DWORD kOffNormalMiddleRig = 0x4B0; // live NORMAL base = controller + this
constexpr DWORD kOffControllerTarget = 0x140;
constexpr DWORD kOffFadeFlagA = 0x304;
constexpr DWORD kOffFadeFlagB = 0x308;

// ---- Character-position watch mode (2026-09-11) ------------------------
// Move-while-aiming. RE5 roots the player while aiming; no working mod
// exists (animations are the usual blocker - irrelevant in first person,
// where the aim pose can stay and the body slides). The plan is to feed a
// walk displacement into the game's own collision-aware movement while
// aiming, so step one is finding that movement code: the instructions that
// WRITE the character's position. The character object keeps its position
// at +0x30 (x) / +0x38 (z) - found by matching the camera controller's copy
// against the object snapshot, same offsets on Chris and Sheva - plus many
// derived copies. Write-only watchpoints: everything reads a position,
// only movement/collision writes it.
constexpr DWORD kOffCharacterPosX = 0x30;
constexpr DWORD kOffCharacterPosZ = 0x38;

// ---- Camera-output watch mode (2026-09-11) -----------------------------
// Culling hunt. VR culling is bad and a wide game FOV can't fix head turns
// past +-85 deg (tried, removed), so the culling test itself has to go. The
// camera controller's own update (re5dx9.exe+449EF0) calls the camera
// function (+446500) with its outputs pointing into the controller: final
// eye at +0x170, target +0x190, up +0x180, FOV +0x1A0. Whatever builds the
// render camera - and so the frustum the culling tests against - must READ
// those, so these are read/write watchpoints on the player's controller.
constexpr DWORD kOffCamEye = 0x170;
constexpr DWORD kOffCamUp = 0x180;
constexpr DWORD kOffCamTarget = 0x190;
constexpr DWORD kOffCamFov = 0x1A0;
// Round 1 (07:13) found the only outside reader: a MAIN CAMERA object that
// copies those four from its active controller ([main+0x1B0]) into itself
// at the same offsets, with the player's controller embedded at +0xE00. The
// frustum must be built from the main camera's copy, so round 2 watches
// that: main = player controller - 0xE00 (relationship checked + logged).
constexpr DWORD kOffMainCameraPlayerController = 0xE00;
constexpr DWORD kOffMainCameraActiveController = 0x1B0;
// Round 2 (07:18) disassembly of the main camera's update (+4425ED..
// +4428D8): the copied output goes through two effect layers (+0x380,
// +0x610) and ends up in the camera's CANONICAL fields - eye +0x30, up
// +0x40, target +0x50, FOV +0x24 - which is what the renderer must read to
// build view, projection and frustum. Round 3 watches those.
constexpr DWORD kOffMainEye = 0x30;
constexpr DWORD kOffMainUp = 0x40;
constexpr DWORD kOffMainTarget = 0x50;
constexpr DWORD kOffMainFov = 0x24;

// Round 4: the culling test (+435A90) reads six planes stored in the camera
// at +0xE0..+0x13C. Other culling routines (level geometry may not use the
// sphere test at all) and the code that BUILDS the planes must touch the
// same addresses - a static search for those offsets drowned in 1600
// unrelated hits, so watch them live instead: plane 0 (normal, d) and plane
// 5 (normal, d) on the player's main camera.
constexpr DWORD kOffMainPlane0 = 0xE0;
constexpr DWORD kOffMainPlane0D = 0xEC;
constexpr DWORD kOffMainPlane5 = 0x130;
constexpr DWORD kOffMainPlane5D = 0x13C;

// Round 5: round 4 (07:42) showed the main camera's planes have exactly two
// users - their builder (+437793..+437C10) and the sphere test already
// hooked - yet VR still loses level geometry and even Sheva. So the other
// culling uses a DIFFERENT copy of the frustum. This mode reads the main
// camera's plane 0 and plane 5, scans all writable memory for other copies
// (exact within 1e-3, or negated), and watches the first few it finds.
// Round 6: the round-5 VR run showed occlusion queries never report
// "hidden" (not the cause) and forcing the sphere test visible isn't enough,
// so level geometry is culled by something that doesn't use the main
// camera's planes. It must use the renderer's view-projection - the matrix
// uploaded to c0-c3 - so watch that matrix at the address it is uploaded
// from (recorded by constant_probe.cpp), one float per row.
enum class WatchTarget { RigFields, FadeFlags, CharacterPosition, CameraOutput, MainCameraPlanes, FrustumCopies, ViewProjSource };
constexpr WatchTarget kWatchTarget = WatchTarget::ViewProjSource;

// The search pattern lives in a global rather than on the stack, so the
// scan can skip it instead of "finding" its own copy.
float g_planePattern[2][4];

bool NearlyEqual(float a, float b)
{
    const float scale = std::fabs(b) > 1.0f ? std::fabs(b) : 1.0f;
    return std::fabs(a - b) <= 1e-3f * scale;
}

// SEH-guarded scan of one region; appends matches. No C++ objects here:
// MSVC forbids unwinding across __try.
int ScanRegionForPlanes(uintptr_t begin, uintptr_t end, DWORD* outAddr, int* outWhich, bool* outNeg, int maxOut, int found)
{
    __try {
        for (uintptr_t a = begin; a + 16 <= end && found < maxOut; a += 4) {
            const float* f = reinterpret_cast<const float*>(a);
            for (int p = 0; p < 2 && found < maxOut; ++p) {
                const float* q = g_planePattern[p];
                const bool same = NearlyEqual(f[0], q[0]) && NearlyEqual(f[1], q[1]) && NearlyEqual(f[2], q[2]) && NearlyEqual(f[3], q[3]);
                const bool neg = !same && NearlyEqual(f[0], -q[0]) && NearlyEqual(f[1], -q[1]) && NearlyEqual(f[2], -q[2]) &&
                    NearlyEqual(f[3], -q[3]);
                if (same || neg) {
                    outAddr[found] = static_cast<DWORD>(a);
                    outWhich[found] = p;
                    outNeg[found] = neg;
                    ++found;
                }
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return found;
}

int FindPlaneCopies(DWORD skipA, DWORD skipB, DWORD* outAddr, int* outWhich, bool* outNeg, int maxOut)
{
    int found = 0;
    const DWORD pattern = static_cast<DWORD>(reinterpret_cast<uintptr_t>(g_planePattern));
    MEMORY_BASIC_INFORMATION mbi = {};
    for (uintptr_t addr = 0x10000; addr < 0x7FFF0000 && found < maxOut;
         addr = reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize) {
        if (!VirtualQuery(reinterpret_cast<const void*>(addr), &mbi, sizeof(mbi)))
            break;
        const DWORD prot = mbi.Protect & 0xFF;
        const bool writable = prot == PAGE_READWRITE || prot == PAGE_EXECUTE_READWRITE || prot == PAGE_WRITECOPY ||
            prot == PAGE_EXECUTE_WRITECOPY;
        if (mbi.State != MEM_COMMIT || !writable || (mbi.Protect & PAGE_GUARD))
            continue;
        const uintptr_t begin = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
        found = ScanRegionForPlanes(begin, begin + mbi.RegionSize, outAddr, outWhich, outNeg, maxOut, found);
    }
    // Drop the known originals and our own pattern.
    int kept = 0;
    for (int i = 0; i < found; ++i) {
        if (outAddr[i] == skipA || outAddr[i] == skipB || (outAddr[i] >= pattern && outAddr[i] < pattern + sizeof(g_planePattern)))
            continue;
        outAddr[kept] = outAddr[i];
        outWhich[kept] = outWhich[i];
        outNeg[kept] = outNeg[i];
        ++kept;
    }
    return kept;
}

// Dr7 R/W bits: 01 = break on write only, 11 = read or write.
constexpr DWORD kWatchRwBits = kWatchTarget == WatchTarget::CharacterPosition ? 1u : 3u;

// ---- Watchpoint hit table ---------------------------------------------
// Written from the exception handler, which can run on any game thread at
// any moment - including one that holds the log mutex or the heap lock. So
// the handler must never log, allocate, or take a lock: fixed array,
// interlocked ops only. Everything is reported later from EndScene.
constexpr int kMaxHits = 128;

struct HitEntry {
    volatile LONG eip;       // 0 = free slot
    volatile LONG count;
    volatile LONG drMask;    // which of Dr0-Dr3 fired for this instruction
    volatile LONG regsValid;
    DWORD eax, ecx, edx, ebx, esp, ebp, esi, edi; // at first hit
};
HitEntry g_hits[kMaxHits];
volatile LONG g_hitOverflow = 0;
volatile LONG g_recording = 0;
volatile LONG g_everArmed = 0;

constexpr DWORD kTrapFlag = 0x100; // EFlags.TF: a debugger single-step, not ours

LONG CALLBACK WatchpointHandler(EXCEPTION_POINTERS* ep)
{
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP)
        return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* ctx = ep->ContextRecord;
    const LONG fired = static_cast<LONG>(ctx->Dr6 & 0xF);
    if (!fired) {
        // A trap already in flight while the watch was being cleared can
        // arrive with its Dr6 status gone (crash 2026-09-11 08:13: our own
        // c0 hook read the watched matrix right as the window closed). If
        // we ever armed and it isn't a trace step, it's one of ours.
        if (g_everArmed && !(ctx->EFlags & kTrapFlag))
            return EXCEPTION_CONTINUE_EXECUTION;
        return EXCEPTION_CONTINUE_SEARCH;
    }

    // Data breakpoints are traps: ExceptionAddress is the instruction AFTER
    // the one that touched the watched address.
    if (g_recording) {
        const LONG eip = static_cast<LONG>(reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress));
        bool recorded = false;
        for (int i = 0; i < kMaxHits && !recorded; ++i) {
            LONG cur = g_hits[i].eip;
            if (cur == 0 && InterlockedCompareExchange(&g_hits[i].eip, eip, 0) == 0) {
                HitEntry& h = g_hits[i];
                h.eax = ctx->Eax; h.ecx = ctx->Ecx; h.edx = ctx->Edx; h.ebx = ctx->Ebx;
                h.esp = ctx->Esp; h.ebp = ctx->Ebp; h.esi = ctx->Esi; h.edi = ctx->Edi;
                InterlockedExchange(&h.regsValid, 1);
                cur = eip;
            } else {
                cur = g_hits[i].eip;
            }
            if (cur == eip) {
                InterlockedIncrement(&g_hits[i].count);
                InterlockedOr(&g_hits[i].drMask, fired);
                recorded = true;
            }
        }
        if (!recorded)
            InterlockedIncrement(&g_hitOverflow);
    }

    // Handled even when not recording: a trap already in flight when the
    // window closes must not reach the game as an unhandled exception.
    ctx->Dr6 = 0;
    return EXCEPTION_CONTINUE_EXECUTION;
}

// Sets (count > 0) or clears (count == 0) the watchpoints on every thread
// in the process except the caller. Must run on the worker thread - a
// thread cannot suspend itself. Nothing between SuspendThread and
// ResumeThread may take a lock (no logging, no allocation): the suspended
// thread could be holding it.
void ApplyWatchToAllThreads(const DWORD* addrs, int count, int* outArmed, int* outFailed)
{
    *outArmed = 0;
    *outFailed = 0;

    DWORD dr7 = 0;
    for (int i = 0; i < count; ++i) {
        dr7 |= 1u << (i * 2);          // L0-L3: local enable
        dr7 |= kWatchRwBits << (16 + i * 4); // R/W: see kWatchRwBits
        dr7 |= 3u << (18 + i * 4);     // LEN = 11: 4 bytes
    }

    const DWORD self = GetCurrentThreadId();
    const DWORD pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE)
        return;

    THREADENTRY32 te = {};
    te.dwSize = sizeof(te);
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self)
            continue;
        HANDLE t = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
        if (!t) {
            ++*outFailed;
            continue;
        }
        bool armed = false;
        if (SuspendThread(t) != static_cast<DWORD>(-1)) {
            CONTEXT c = {};
            c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
            if (GetThreadContext(t, &c)) {
                c.Dr0 = count > 0 ? addrs[0] : 0;
                c.Dr1 = count > 1 ? addrs[1] : 0;
                c.Dr2 = count > 2 ? addrs[2] : 0;
                c.Dr3 = count > 3 ? addrs[3] : 0;
                // Dr6 left alone: wiping it under a trap that is mid-delivery
                // hands the handler an exception it can't recognise.
                c.Dr7 = dr7;
                armed = SetThreadContext(t, &c) != FALSE;
            }
            ResumeThread(t);
        }
        if (armed)
            ++*outArmed;
        else
            ++*outFailed;
        CloseHandle(t);
    }
    CloseHandle(snap);
}

void GameFolderPath(const char* fileName, char* out, size_t outSize)
{
    char exePath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    char drive[_MAX_DRIVE], dir[_MAX_DIR];
    _splitpath_s(exePath, drive, sizeof(drive), dir, sizeof(dir), nullptr, 0, nullptr, 0);
    _snprintf_s(out, outSize, _TRUNCATE, "%s%s%s", drive, dir, fileName);
}

// Writes the exe's whole in-memory image, page-aligned from the module base,
// so file offset == RVA. Uncommitted or unreadable pages are zero-filled to
// keep that mapping exact.
bool DumpModuleImage(DWORD* outBytes)
{
    *outBytes = 0;
    BYTE* exe = reinterpret_cast<BYTE*>(GetModuleHandleA(nullptr));
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(exe);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS32*>(exe + dos->e_lfanew);
    const DWORD imageSize = nt->OptionalHeader.SizeOfImage;

    char path[MAX_PATH];
    GameFolderPath("re5vr_codedump.bin", path, sizeof(path));
    HANDLE f = CreateFileA(path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE)
        return false;

    static const BYTE kZeroPage[4096] = {};
    DWORD off = 0;
    while (off < imageSize) {
        MEMORY_BASIC_INFORMATION mbi = {};
        if (!VirtualQuery(exe + off, &mbi, sizeof(mbi)))
            break;
        DWORD regionEnd = static_cast<DWORD>(static_cast<BYTE*>(mbi.BaseAddress) + mbi.RegionSize - exe);
        if (regionEnd > imageSize)
            regionEnd = imageSize;
        const bool readable = mbi.State == MEM_COMMIT && mbi.Protect != 0 &&
            !(mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS));
        for (; off < regionEnd; off += 4096) {
            DWORD written = 0;
            WriteFile(f, readable ? exe + off : kZeroPage, 4096, &written, nullptr);
            *outBytes += written;
        }
    }
    CloseHandle(f);
    return *outBytes == imageSize;
}

// ---- Worker: dump, arm, wait, disarm ------------------------------------
DWORD g_watchAddrs[4] = {};
int g_watchCount = 0;
std::atomic<int> g_workerState{0}; // 0 idle, 1 running, 2 finished (report pending)
bool g_codeDumped = false;

DWORD WINAPI WatchWorker(LPVOID)
{
    if (!g_codeDumped) {
        // The exe's code never changes (fixed image base, and the only
        // patches in it are ours), so one dump serves every session.
        char existing[MAX_PATH];
        GameFolderPath("re5vr_codedump.bin", existing, sizeof(existing));
        if (GetFileAttributesA(existing) != INVALID_FILE_ATTRIBUTES) {
            g_codeDumped = true;
            Log_Printf("BoomFinder: re5vr_codedump.bin already exists - not re-dumping");
        } else {
            DWORD bytes = 0;
            g_codeDumped = DumpModuleImage(&bytes);
            Log_Printf("BoomFinder: code dump %s (%lu bytes) -> re5vr_codedump.bin",
                g_codeDumped ? "written" : "FAILED", bytes);
        }
    }

    std::memset(const_cast<HitEntry*>(g_hits), 0, sizeof(g_hits));
    g_hitOverflow = 0;
    InterlockedExchange(&g_everArmed, 1);
    InterlockedExchange(&g_recording, 1);

    int armed = 0, failed = 0;
    ApplyWatchToAllThreads(g_watchAddrs, g_watchCount, &armed, &failed);
    Log_Printf("BoomFinder: watchpoints armed on %d thread(s) (%d failed) for %lu ms",
        armed, failed, kWatchWindowMs);

    Sleep(kWatchWindowMs);

    InterlockedExchange(&g_recording, 0);
    ApplyWatchToAllThreads(nullptr, 0, &armed, &failed);
    Log_Printf("BoomFinder: watchpoints cleared on %d thread(s) (%d failed)", armed, failed);

    g_workerState.store(2);
    return 0;
}

void DescribeAddress(DWORD addr, char* out, size_t outSize)
{
    HMODULE mod = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(static_cast<uintptr_t>(addr)), &mod) && mod) {
        char path[MAX_PATH] = {};
        GetModuleFileNameA(mod, path, MAX_PATH);
        const char* name = std::strrchr(path, '\\');
        name = name ? name + 1 : path;
        _snprintf_s(out, outSize, _TRUNCATE, "%s+%lX", name,
            addr - static_cast<DWORD>(reinterpret_cast<uintptr_t>(mod)));
    } else {
        _snprintf_s(out, outSize, _TRUNCATE, "%08lX (no module)", addr);
    }
}

void ReportHits()
{
    int used = 0;
    for (int i = 0; i < kMaxHits; ++i) {
        if (g_hits[i].eip)
            ++used;
    }
    Log_Printf("BoomFinder: %d unique instruction(s) touched the watched fields (overflow=%ld). "
               "Addresses are the instruction AFTER the access. dr bits: 1=Dr0 2=Dr1 4=Dr2 8=Dr3",
        used, g_hitOverflow);
    for (int i = 0; i < kMaxHits; ++i) {
        const HitEntry& h = g_hits[i];
        if (!h.eip)
            continue;
        char where[MAX_PATH + 32];
        DescribeAddress(static_cast<DWORD>(h.eip), where, sizeof(where));
        Log_Printf("BoomFinder: HIT %s count=%ld dr=0x%lX eax=%08lX ecx=%08lX edx=%08lX ebx=%08lX "
                   "esp=%08lX ebp=%08lX esi=%08lX edi=%08lX",
            where, h.count, h.drMask, h.eax, h.ecx, h.edx, h.ebx, h.esp, h.ebp, h.esi, h.edi);
    }
}

// ---- Snapshots ---------------------------------------------------------
#pragma pack(push, 1)
struct SnapHeader {
    char magic[4];       // "SNAP"
    DWORD msSinceStart;
    DWORD structBase;    // the NORMAL rig struct this region is centred on
    DWORD regionStart;
    DWORD regionSize;
    DWORD haveMatrix;
    float matrix[16];    // c0-c3 as last uploaded (may be a UI pass - filter offline)
};
#pragma pack(pop)

HANDLE g_snapFile = INVALID_HANDLE_VALUE;
unsigned long long g_startMs = 0;
unsigned long long g_lastSnapMs = 0;
int g_snapCount = 0;
BYTE g_snapBuf[kSnapBefore + kSnapAfter];

// The region reaches past the rig struct into neighbouring heap memory we
// don't own, so copy it a page at a time under SEH and zero anything that
// faults. No C++ objects in here - MSVC forbids unwinding across __try.
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

void WriteSnapshots(unsigned long long now)
{
    void* bases[kMaxLive];
    bool aim[kMaxLive];
    const int n = CameraRigHook_GetLiveBases(bases, aim, kMaxLive);

    SnapHeader hdr = {};
    std::memcpy(hdr.magic, "SNAP", 4);
    hdr.msSinceStart = static_cast<DWORD>(now - g_startMs);
    hdr.haveMatrix = ConstantProbe_GetCachedCameraMatrix(hdr.matrix) ? 1 : 0;

    for (int i = 0; i < n; ++i) {
        if (aim[i])
            continue;
        const uintptr_t base = reinterpret_cast<uintptr_t>(bases[i]);
        hdr.structBase = static_cast<DWORD>(base);
        hdr.regionStart = static_cast<DWORD>(base - kSnapBefore);
        hdr.regionSize = sizeof(g_snapBuf);
        SafeCopyRegion(g_snapBuf, base - kSnapBefore, sizeof(g_snapBuf));
        DWORD written = 0;
        WriteFile(g_snapFile, &hdr, sizeof(hdr), &written, nullptr);
        WriteFile(g_snapFile, g_snapBuf, sizeof(g_snapBuf), &written, nullptr);
        ++g_snapCount;
    }
}

void Start()
{
    void* bases[kMaxLive];
    bool aim[kMaxLive];
    const int n = CameraRigHook_GetLiveBases(bases, aim, kMaxLive);

    // Watch up to two NORMAL structs (Chris and Sheva both have one, and we
    // can't tell which is whose). Two fields each; with only one struct,
    // spend the spare debug registers on two more of its fields instead.
    DWORD normal[2] = {};
    int normalCount = 0;
    for (int i = 0; i < n && normalCount < 2; ++i) {
        if (!aim[i])
            normal[normalCount++] = static_cast<DWORD>(reinterpret_cast<uintptr_t>(bases[i]));
    }
    if (normalCount == 0) {
        Log_Printf("BoomFinder: F5 pressed but no live NORMAL camera struct - be in gameplay, not aiming, pistol equipped");
        return;
    }

    if (kWatchTarget == WatchTarget::ViewProjSource) {
        const void* sources[16];
        unsigned counts[16];
        const int n = ConstantProbe_GetCameraMatrixSources(sources, counts, 16);
        if (n == 0) {
            Log_Printf("BoomFinder: no camera-matrix uploads recorded yet");
            return;
        }
        // A source on this (render) thread's stack is a temporary copy -
        // watching it would catch whatever else reuses that stack slot.
        const NT_TIB* tib = reinterpret_cast<const NT_TIB*>(NtCurrentTeb());
        const uintptr_t stackLo = reinterpret_cast<uintptr_t>(tib->StackLimit);
        const uintptr_t stackHi = reinterpret_cast<uintptr_t>(tib->StackBase);
        int pick = -1;
        for (int i = 0; i < n; ++i) {
            const uintptr_t p = reinterpret_cast<uintptr_t>(sources[i]);
            const bool onStack = p >= stackLo && p < stackHi;
            char where[MAX_PATH + 32];
            DescribeAddress(static_cast<DWORD>(p), where, sizeof(where));
            Log_Printf("BoomFinder: c0-c3 source #%d %08lX (%s%s): %u upload(s)", i, static_cast<DWORD>(p), where,
                onStack ? ", RENDER-THREAD STACK" : "", counts[i]);
            if (pick < 0 && !onStack)
                pick = i;
        }
        if (pick < 0) {
            Log_Printf("BoomFinder: every camera-matrix source is a stack temporary - nothing stable to watch");
            return;
        }
        const DWORD src = static_cast<DWORD>(reinterpret_cast<uintptr_t>(sources[pick]));
        g_watchAddrs[0] = src + 0x00;
        g_watchAddrs[1] = src + 0x10;
        g_watchAddrs[2] = src + 0x20;
        g_watchAddrs[3] = src + 0x30;
        g_watchCount = 4;
        Log_Printf("BoomFinder: view-projection watch on source #%d %08lX - Dr0 row0, Dr1 row1, Dr2 row2, Dr3 row3", pick, src);
    } else if (kWatchTarget == WatchTarget::CameraOutput || kWatchTarget == WatchTarget::MainCameraPlanes ||
        kWatchTarget == WatchTarget::FrustumCopies) {
        const uintptr_t controller = reinterpret_cast<uintptr_t>(CameraRigHook_GetPlayerController());
        if (!controller) {
            Log_Printf("BoomFinder: F5 pressed but the player's camera controller isn't known yet - turn F4 on first");
            return;
        }
        const uintptr_t mainCamera = controller - kOffMainCameraPlayerController;
        DWORD active = 0;
        const bool linked = TryCopy(&active, reinterpret_cast<const void*>(mainCamera + kOffMainCameraActiveController),
                                sizeof(active)) &&
            active == static_cast<DWORD>(controller);
        if (kWatchTarget == WatchTarget::FrustumCopies) {
            if (!TryCopy(g_planePattern[0], reinterpret_cast<const void*>(mainCamera + kOffMainPlane0), sizeof(g_planePattern[0])) ||
                !TryCopy(g_planePattern[1], reinterpret_cast<const void*>(mainCamera + kOffMainPlane5), sizeof(g_planePattern[1]))) {
                Log_Printf("BoomFinder: could not read the main camera's frustum planes");
                return;
            }
            Log_Printf("BoomFinder: main camera %08lX (%s): plane0 (%.4f %.4f %.4f %.2f) plane5 (%.4f %.4f %.4f %.2f) - "
                       "scanning all writable memory for copies",
                static_cast<DWORD>(mainCamera), linked ? "linked" : "NOT linked",
                g_planePattern[0][0], g_planePattern[0][1], g_planePattern[0][2], g_planePattern[0][3],
                g_planePattern[1][0], g_planePattern[1][1], g_planePattern[1][2], g_planePattern[1][3]);
            const unsigned long long scanStart = GetTickCount64();
            DWORD copies[32];
            int which[32];
            bool negated[32];
            const int n = FindPlaneCopies(static_cast<DWORD>(mainCamera + kOffMainPlane0),
                static_cast<DWORD>(mainCamera + kOffMainPlane5), copies, which, negated, 32);
            Log_Printf("BoomFinder: plane scan took %llu ms - %d other cop%s", GetTickCount64() - scanStart, n,
                n == 1 ? "y" : "ies");
            for (int i = 0; i < n; ++i) {
                char where[MAX_PATH + 32];
                DescribeAddress(copies[i], where, sizeof(where));
                Log_Printf("BoomFinder:   copy #%d at %08lX (%s): plane %d%s", i, copies[i], where, which[i] == 0 ? 0 : 5,
                    negated[i] ? " NEGATED" : "");
            }
            if (n == 0) {
                Log_Printf("BoomFinder: no other copy of the frustum planes anywhere - nothing to watch");
                return;
            }
            g_watchCount = n < 4 ? n : 4;
            for (int i = 0; i < 4; ++i)
                g_watchAddrs[i] = i < g_watchCount ? copies[i] : 0;
        } else {
        const bool planes = kWatchTarget == WatchTarget::MainCameraPlanes;
        g_watchAddrs[0] = static_cast<DWORD>(mainCamera + (planes ? kOffMainPlane0 : kOffMainEye));
        g_watchAddrs[1] = static_cast<DWORD>(mainCamera + (planes ? kOffMainPlane0D : kOffMainTarget));
        g_watchAddrs[2] = static_cast<DWORD>(mainCamera + (planes ? kOffMainPlane5 : kOffMainFov));
        g_watchAddrs[3] = static_cast<DWORD>(mainCamera + (planes ? kOffMainPlane5D : kOffMainUp));
        g_watchCount = 4;
        Log_Printf("BoomFinder: main-camera %s watch on %08lX (player controller %08lX; [main+0x1B0] = %08lX, %s) - %s",
            planes ? "FRUSTUM-PLANE" : "CANONICAL-field", static_cast<DWORD>(mainCamera), static_cast<DWORD>(controller),
            active, linked ? "linked as expected" : "NOT linked - layout assumption wrong",
            planes ? "Dr0 plane0 n +0xE0, Dr1 plane0 d +0xEC, Dr2 plane5 n +0x130, Dr3 plane5 d +0x13C"
                   : "Dr0 eye +0x30, Dr1 target +0x50, Dr2 FOV +0x24, Dr3 up +0x40");
        }
    } else if (kWatchTarget != WatchTarget::RigFields) {
        // Two fields on each followed character - two characters fill all
        // four debug registers.
        const bool position = kWatchTarget == WatchTarget::CharacterPosition;
        const DWORD offA = position ? kOffCharacterPosX : kOffFadeFlagA;
        const DWORD offB = position ? kOffCharacterPosZ : kOffFadeFlagB;
        g_watchCount = 0;
        for (int i = 0; i < normalCount; ++i) {
            const uintptr_t controller = normal[i] - kOffNormalMiddleRig;
            DWORD object = 0;
            if (!TryCopy(&object, reinterpret_cast<const void*>(controller + kOffControllerTarget), sizeof(object)) || !object)
                continue;
            Log_Printf("BoomFinder: %s watch - controller %08lX follows character object %08lX",
                position ? "position (write-only)" : "fade", static_cast<DWORD>(controller), object);
            g_watchAddrs[g_watchCount++] = object + offA;
            g_watchAddrs[g_watchCount++] = object + offB;
        }
        if (g_watchCount == 0) {
            Log_Printf("BoomFinder: F5 pressed but no character object readable from the live controllers");
            return;
        }
        for (int i = g_watchCount; i < 4; ++i)
            g_watchAddrs[i] = 0;
    } else {
        g_watchAddrs[0] = normal[0] + kOffDistance;
        g_watchAddrs[1] = normal[0] + kOffVerticalDistance;
        if (normalCount == 2) {
            g_watchAddrs[2] = normal[1] + kOffDistance;
            g_watchAddrs[3] = normal[1] + kOffVerticalDistance;
        } else {
            g_watchAddrs[2] = normal[0] + kOffAngle;
            g_watchAddrs[3] = normal[0] + kOffVerticalAdjust;
        }
        g_watchCount = 4;
    }

    for (int i = 0; i < n; ++i)
        Log_Printf("BoomFinder: live struct #%d base=%p (%s)", i, bases[i], aim[i] ? "AIM" : "NORMAL");
    Log_Printf("BoomFinder: watching Dr0=%08lX Dr1=%08lX Dr2=%08lX Dr3=%08lX",
        g_watchAddrs[0], g_watchAddrs[1], g_watchAddrs[2], g_watchAddrs[3]);

    static bool handlerInstalled = false;
    if (!handlerInstalled) {
        // First in the chain, and never removed: see WatchpointHandler.
        handlerInstalled = AddVectoredExceptionHandler(1, WatchpointHandler) != nullptr;
        if (!handlerInstalled) {
            Log_Printf("BoomFinder: AddVectoredExceptionHandler failed - not arming");
            return;
        }
    }

    g_startMs = GetTickCount64();
    g_lastSnapMs = 0;
    g_snapCount = 0;
    if (kWatchTarget == WatchTarget::RigFields) {
        // Controller snapshots only mean something for the camera hunt.
        char path[MAX_PATH];
        GameFolderPath("re5vr_camsnap.bin", path, sizeof(path));
        g_snapFile = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (g_snapFile == INVALID_HANDLE_VALUE)
            Log_Printf("BoomFinder: could not create re5vr_camsnap.bin (error %lu)", GetLastError());
    }

    g_workerState.store(1);
    HANDLE worker = CreateThread(nullptr, 0, WatchWorker, nullptr, 0, nullptr);
    if (worker) {
        CloseHandle(worker);
    } else {
        g_workerState.store(0);
        Log_Printf("BoomFinder: CreateThread failed (error %lu)", GetLastError());
    }

    if (kWatchTarget == WatchTarget::ViewProjSource)
        Log_Printf("BoomFinder: F5 - watching who reads the renderer's view-projection for %lu ms. Look and walk around.",
            kWatchWindowMs);
    else if (kWatchTarget == WatchTarget::FrustumCopies)
        Log_Printf("BoomFinder: F5 - watching the frustum-plane copies for %lu ms. Look and walk around.", kWatchWindowMs);
    else if (kWatchTarget == WatchTarget::MainCameraPlanes)
        Log_Printf("BoomFinder: F5 - watching who builds/reads the frustum planes for %lu ms. Look and walk around.",
            kWatchWindowMs);
    else if (kWatchTarget == WatchTarget::CameraOutput)
        Log_Printf("BoomFinder: F5 - watching who reads the camera output for %lu ms. Look and walk around.",
            kWatchWindowMs);
    else if (kWatchTarget == WatchTarget::CharacterPosition)
        Log_Printf("BoomFinder: F5 - watching character position writes for %lu ms. Walk around, then aim and try to walk.",
            kWatchWindowMs);
    else if (kWatchTarget == WatchTarget::FadeFlags)
        Log_Printf("BoomFinder: F5 - watching the fade flags for %lu ms. Toggle F4 off and on a couple of times.",
            kWatchWindowMs);
    else
        Log_Printf("BoomFinder: F5 - capturing for %llu ms. Look slowly all the way UP, hold, all the way DOWN, hold, repeat.",
            kSnapshotWindowMs);
}

} // namespace

void BoomFinder_OnEndScene()
{
    static bool prevF5Down = false;
    const bool f5Down = (GetAsyncKeyState(VK_F5) & 0x8000) != 0;
    const bool idle = g_workerState.load() == 0 && g_snapFile == INVALID_HANDLE_VALUE;
    if (f5Down && !prevF5Down) {
        if (idle)
            Start();
        else
            Log_Printf("BoomFinder: F5 ignored - a capture is still running");
    }
    prevF5Down = f5Down;

    if (g_snapFile != INVALID_HANDLE_VALUE) {
        const unsigned long long now = GetTickCount64();
        if (now - g_startMs >= kSnapshotWindowMs) {
            CloseHandle(g_snapFile);
            g_snapFile = INVALID_HANDLE_VALUE;
            Log_Printf("BoomFinder: %d snapshot(s) written to re5vr_camsnap.bin - capture complete, you can quit", g_snapCount);
        } else if (now - g_lastSnapMs >= kSnapshotEveryMs) {
            g_lastSnapMs = now;
            WriteSnapshots(now);
        }
    }

    if (g_workerState.load() == 2) {
        ReportHits();
        g_workerState.store(0);
    }
}
