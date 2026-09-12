#include "filter_patch.h"
#include "../util/log.h"

#include <windows.h>

#include <cstring>

// ---- Why (2026-09-12) --------------------------------------------------
// A tester asked for RE5's colour filter - the heavy yellow grade - to be
// removable. The community trainer has that option, so the same trick as the
// laser sight found it: capture the game's code with the option off and on
// (state_probe.cpp) and diff. It changes exactly one byte.
//
// The site copies a block of effect settings and decides the effect's own
// "enabled" flag:
//     cmp  byte ptr [edx+44h], 0
//     je   skip                     <- exe+3C2FF7, this jump
//     cmp  dword ptr [edx+48h], 0
//     je   skip
//     mov  ecx, 1
//     jmp  store
//   skip:
//     xor  ecx, ecx
//   store:
//     mov  byte ptr [eax+64h], cl
// Forcing the jump makes that flag always 0, so the filter never turns on.
// Unlike the laser patch this one is reversible at runtime: the game re-runs
// this copy as settings change, so the picture follows the toggle. Applied at
// startup - the tester who asked for it wanted the grade gone - and F10 puts
// the original look back for anyone who prefers it.

namespace {

constexpr DWORD kRva = 0x3C2FF3;                              // "cmp byte ptr [edx+44h],0 / je"
constexpr BYTE kOriginal[6] = { 0x80, 0x7A, 0x44, 0x00, 0x74, 0x0D };
constexpr int kJumpOffset = 4;                                // the je opcode within kOriginal
constexpr BYTE kJmpShort = 0xEB;

bool g_verified = false;
bool g_filterOff = true; // default: filter removed (user, 2026-09-12); F10 puts the original look back

BYTE* Site()
{
    return reinterpret_cast<BYTE*>(GetModuleHandleA(nullptr)) + kRva;
}

void WriteJumpOpcode(BYTE value)
{
    BYTE* at = Site() + kJumpOffset;
    DWORD oldProtect = 0;
    VirtualProtect(at, 1, PAGE_EXECUTE_READWRITE, &oldProtect);
    *at = value;
    VirtualProtect(at, 1, oldProtect, &oldProtect);
    FlushInstructionCache(GetCurrentProcess(), at, 1);
}

} // namespace

void FilterPatch_Install()
{
    const BYTE* at = Site();
    BYTE patched[sizeof(kOriginal)];
    std::memcpy(patched, kOriginal, sizeof(patched));
    patched[kJumpOffset] = kJmpShort;
    g_verified = std::memcmp(at, kOriginal, sizeof(kOriginal)) == 0 ||
        std::memcmp(at, patched, sizeof(patched)) == 0;
    if (!g_verified) {
        Log_Printf("FilterPatch: exe+%lX holds %02X %02X %02X %02X %02X %02X, not the expected instructions - "
                   "F10 will do nothing",
            kRva, at[0], at[1], at[2], at[3], at[4], at[5]);
        return;
    }
    WriteJumpOpcode(kJmpShort);
    Log_Printf("FilterPatch: RE5's colour filter removed (exe+%lX) - F10 puts the original look back", kRva + kJumpOffset);
}

void FilterPatch_OnEndScene()
{
    static bool prevDown = false;
    const bool down = (GetAsyncKeyState(VK_F10) & 0x8000) != 0;
    if (down && !prevDown && g_verified) {
        g_filterOff = !g_filterOff;
        WriteJumpOpcode(g_filterOff ? kJmpShort : kOriginal[kJumpOffset]);
        Log_Printf("FilterPatch: F10 pressed, RE5's colour filter now %s", g_filterOff ? "OFF" : "ON");
    }
    prevDown = down;
}
