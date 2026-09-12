#include "state_probe.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <windows.h>

#include <cstdio>
#include <cstring>

// ---- Why this exists (2026-09-11) ---------------------------------------
// RE5 shows a laser sight when you aim with a gamepad and a crosshair when
// you aim with the mouse. The user wants the laser forced on - in VR there
// is no HUD, so the laser is the only aiming aid - which means finding what
// the game keys that choice on: most likely an input-device flag in some
// manager object, possibly something on the player or the weapon.
//
// Same method that found the fade flags: capture the game's state in both
// modes and diff offline. Each '=' press arms one capture that fires 3 s
// later, with a beep, so the user can get back on the pad before it fires -
// the keyboard press itself may switch the game to mouse mode. A capture
// holds:
//  - the data sections of re5dx9.exe (its globals, including the pointers
//    to its singleton objects);
//  - kGlobalDerefBytes at every distinct heap address those globals point
//    to (the singletons themselves - input managers and the like);
//  - the player's camera controller and character object, plus
//    kObjectDerefBytes at every new heap address the character points to
//    (its weapon, among others).
// Written to re5vr_state_N.bin as blocks (BlockHeader + bytes); diffed with
// a scratchpad tool.

namespace {

constexpr unsigned long long kCaptureDelayMs = 3000;
constexpr DWORD kGlobalDerefBytes = 0x200;
constexpr DWORD kObjectDerefBytes = 0x400;
constexpr DWORD kCharacterBytes = 0x10000;
constexpr DWORD kControllerBytes = 0x3000;
constexpr DWORD kOffControllerCharacter = 0x140;
constexpr int kMaxTargets = 80000;
constexpr int kTargetSlots = 1 << 18; // power of two, well over 2 * kMaxTargets
constexpr int kMaxRegions = 16384;

// re5dx9.exe's data sections, as RVA ranges from `dumpbin /headers` of the
// fixed dump. The Steam DRM strips section names and marks everything RWX,
// so they can't be picked by flags. Section 1 (code), .rsrc and .bind are
// left out.
struct Range {
    DWORD rva;
    DWORD size;
};
constexpr Range kCodeSection = { 0x1000, 0xB49000 };
constexpr Range kDataSections[] = {
    { 0xB4A000, 0x180000 }, { 0xCCA000, 0x1E9000 }, { 0xEB3000, 0xBA000 },
    { 0xF7F000, 0x764000 }, { 0x16E3000, 0x21C000 },
};

#pragma pack(push, 1)
struct BlockHeader {
    char magic[4]; // "SECT", "GREF", "CTRL", "OBJ ", "OREF"
    DWORD address;
    DWORD size;
};
#pragma pack(pop)

struct Region {
    uintptr_t base;
    uintptr_t end;
};

Region g_regions[kMaxRegions];
int g_regionCount = 0;
DWORD g_targetSet[kTargetSlots];
DWORD g_targetList[kMaxTargets];
int g_targetCount = 0;
BYTE g_buf[0x10000];
HANDLE g_file = INVALID_HANDLE_VALUE;
DWORD g_bytesWritten = 0;
int g_captureIndex = 0;
unsigned long long g_captureAtMs = 0; // 0 = nothing armed

// Game memory we don't own - copy under SEH, a page at a time, zeroing
// anything that faults. No C++ objects here: MSVC forbids unwinding across
// __try.
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

void GameFolderPath(const char* fileName, char* out, size_t outSize)
{
    char exePath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    char drive[_MAX_DRIVE], dir[_MAX_DIR];
    _splitpath_s(exePath, drive, sizeof(drive), dir, sizeof(dir), nullptr, 0, nullptr, 0);
    _snprintf_s(out, outSize, _TRUNCATE, "%s%s%s", drive, dir, fileName);
}

// Committed, read-write, private memory - the heaps - in ascending order.
void CollectHeapRegions()
{
    g_regionCount = 0;
    MEMORY_BASIC_INFORMATION mbi = {};
    uintptr_t addr = 0x10000;
    while (g_regionCount < kMaxRegions && VirtualQuery(reinterpret_cast<const void*>(addr), &mbi, sizeof(mbi))) {
        const uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
        const uintptr_t end = base + mbi.RegionSize;
        if (mbi.State == MEM_COMMIT && mbi.Type == MEM_PRIVATE && (mbi.Protect & 0xFF) == PAGE_READWRITE &&
            !(mbi.Protect & PAGE_GUARD))
            g_regions[g_regionCount++] = { base, end };
        if (end <= addr)
            break; // wrapped past the top of the address space
        addr = end;
    }
}

bool IsHeapPointer(DWORD v)
{
    if (v & 3)
        return false;
    int lo = 0, hi = g_regionCount - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (v < g_regions[mid].base)
            hi = mid - 1;
        else if (v >= g_regions[mid].end)
            lo = mid + 1;
        else
            return true;
    }
    return false;
}

// True if v wasn't seen yet this capture (and there's room to record it).
bool AddTarget(DWORD v)
{
    if (g_targetCount >= kMaxTargets)
        return false;
    DWORD h = v * 2654435761u;
    for (int probe = 0; probe < kTargetSlots; ++probe) {
        DWORD& slot = g_targetSet[(h + probe) & (kTargetSlots - 1)];
        if (slot == v)
            return false;
        if (slot == 0) {
            slot = v;
            g_targetList[g_targetCount++] = v;
            return true;
        }
    }
    return false;
}

void HarvestPointers(const BYTE* data, DWORD size)
{
    for (DWORD off = 0; off + 4 <= size; off += 4) {
        DWORD v;
        std::memcpy(&v, data + off, sizeof(v));
        if (v && IsHeapPointer(v))
            AddTarget(v);
    }
}

void WriteBlock(const char* magic, DWORD address, const BYTE* data, DWORD size)
{
    BlockHeader h;
    std::memcpy(h.magic, magic, 4);
    h.address = address;
    h.size = size;
    DWORD w = 0;
    WriteFile(g_file, &h, sizeof(h), &w, nullptr);
    g_bytesWritten += w;
    WriteFile(g_file, data, size, &w, nullptr);
    g_bytesWritten += w;
}

void Capture(int index)
{
    char name[64];
    _snprintf_s(name, sizeof(name), _TRUNCATE, "re5vr_state_%d.bin", index);
    char path[MAX_PATH];
    GameFolderPath(name, path, sizeof(path));
    g_file = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (g_file == INVALID_HANDLE_VALUE) {
        Log_Printf("StateProbe: could not create %s (error %lu)", name, GetLastError());
        return;
    }
    const unsigned long long startMs = GetTickCount64();
    g_bytesWritten = 0;
    CollectHeapRegions();
    std::memset(g_targetSet, 0, sizeof(g_targetSet));
    g_targetCount = 0;

    const uintptr_t exeBase = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));

    // 0. The exe's code. It only changes when something patches it - like the
    // community trainer's laser-sight toggle (user, 2026-09-11) - so a
    // capture with the toggle off and one with it on show its patch exactly.
    for (DWORD off = 0; off < kCodeSection.size; off += sizeof(g_buf)) {
        const DWORD chunk = kCodeSection.size - off < sizeof(g_buf) ? kCodeSection.size - off : static_cast<DWORD>(sizeof(g_buf));
        const uintptr_t at = exeBase + kCodeSection.rva + off;
        SafeCopyRegion(g_buf, at, chunk);
        WriteBlock("CODE", static_cast<DWORD>(at), g_buf, chunk);
    }

    // 1. The exe's globals, harvesting heap pointers as we go.
    for (const Range& s : kDataSections) {
        for (DWORD off = 0; off < s.size; off += sizeof(g_buf)) {
            const DWORD chunk = s.size - off < sizeof(g_buf) ? s.size - off : static_cast<DWORD>(sizeof(g_buf));
            const uintptr_t at = exeBase + s.rva + off;
            SafeCopyRegion(g_buf, at, chunk);
            WriteBlock("SECT", static_cast<DWORD>(at), g_buf, chunk);
            HarvestPointers(g_buf, chunk);
        }
    }
    const int globalTargets = g_targetCount;

    // 2. What the globals point to.
    for (int i = 0; i < globalTargets; ++i) {
        SafeCopyRegion(g_buf, g_targetList[i], kGlobalDerefBytes);
        WriteBlock("GREF", g_targetList[i], g_buf, kGlobalDerefBytes);
    }

    // 3. The player's controller and character, and what the character
    // points to that the globals didn't already cover.
    unsigned char* controller = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    DWORD character = 0;
    if (controller) {
        const uintptr_t c = reinterpret_cast<uintptr_t>(controller);
        SafeCopyRegion(g_buf, c, kControllerBytes);
        WriteBlock("CTRL", static_cast<DWORD>(c), g_buf, kControllerBytes);
        if (TryCopy(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
            SafeCopyRegion(g_buf, character, kCharacterBytes);
            WriteBlock("OBJ ", character, g_buf, kCharacterBytes);
            HarvestPointers(g_buf, kCharacterBytes);
            for (int i = globalTargets; i < g_targetCount; ++i) {
                SafeCopyRegion(g_buf, g_targetList[i], kObjectDerefBytes);
                WriteBlock("OREF", g_targetList[i], g_buf, kObjectDerefBytes);
            }
        }
    }

    CloseHandle(g_file);
    g_file = INVALID_HANDLE_VALUE;
    Log_Printf("StateProbe: capture %d -> %s: %d global target(s), %d from the character (controller %p, character %08lX), "
               "%lu bytes, %llu ms",
        index, name, globalTargets, g_targetCount - globalTargets, controller, character, g_bytesWritten,
        GetTickCount64() - startMs);
    MessageBeep(MB_ICONASTERISK);
}

} // namespace

void StateProbe_OnEndScene()
{
    const unsigned long long now = GetTickCount64();
    static bool prevDown = false;
    const bool down = (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000) != 0; // '='
    if (down && !prevDown && !g_captureAtMs) {
        ++g_captureIndex;
        g_captureAtMs = now + kCaptureDelayMs;
        Log_Printf("StateProbe: '=' pressed - capture %d fires in %llu ms, aim now", g_captureIndex, kCaptureDelayMs);
    }
    prevDown = down;

    if (g_captureAtMs && now >= g_captureAtMs) {
        g_captureAtMs = 0;
        Capture(g_captureIndex);
    }
}
