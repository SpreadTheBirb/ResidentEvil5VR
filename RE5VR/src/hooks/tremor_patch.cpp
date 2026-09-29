#include "tremor_patch.h"

#include "../util/log.h"

#include <windows.h>

#include <cstring>

// ---- Steady hands (2026-09-27) -------------------------------------------
//
// RE5 shakes the character's hands while aiming. On a gamepad that is a
// deliberate difficulty knob: it makes the crosshair drift so holding a shot
// costs something. In a headset it is simply wrong. Your hands are not
// shaking, the arm IK is trying to put the weapon exactly where your
// controller is, and the game is adding a wobble on top of that - so the gun
// will not sit still no matter how steady you are, and the IK spends its
// effort fighting it.
//
// The four sites came from the trainer, found without reading a line of its
// code. It is packed and has no readable strings, so instead the game's own
// code section was recorded with the option off, the option was switched on,
// and the two were compared from inside the process. Eleven megabytes of
// code, four differences:
//
//   exe+20CB47   B0 01                    -> 32 C0
//                mov al,1                 -> xor al,al
//                A predicate that says tremors are on, forced to false.
//
//   exe+76D78D   8B 90 28 01 00 00        -> 31 D2 EB 02 90 90
//                mov edx,[eax+128]        -> xor edx,edx / jmp +2 / nop nop
//   exe+76D80E   8B B0 28 01 00 00        -> 31 F6 EB 02 90 90
//                mov esi,[eax+128]        -> xor esi,esi / jmp +2 / nop nop
//                Two loads of the same field at +0x128, both zeroed.
//
//   exe+786227   F3 0F 10 83 AC 00 00 00  -> 0F 57 C0 EB 03 90 90 90
//                movss xmm0,[ebx+AC]      -> xorps xmm0,xmm0 / jmp +3 / nops
//                A float load of +0xAC, replaced with zero.
//
// Each replacement is exactly as long as what it replaces, with a short jump
// over the padding, so nothing downstream shifts and every one of them is
// reversible byte for byte. The originals are verified before anything is
// written: if the bytes are not what we expect this is a different build of
// the game and we do nothing at all.

namespace {

struct Site {
    uintptr_t rva;
    unsigned char was[8];
    unsigned char now[8];
    int len;
};

// Verified before use, so a different game build simply turns the feature off
// rather than corrupting anything.
// PLUS THE SECTION BASE (2026-09-28, "steady the hands said not available").
//
// The differ that found these reported offsets into the code SECTION and
// labelled them "exe+", and the first executable section of this game starts
// at 0x1000 - so every address was short by exactly that. The bytes it found
// were real, they were simply a page adrift from where I went looking.
const Site kSites[] = {
    { 0x20DB47, { 0xB0, 0x01 }, { 0x32, 0xC0 }, 2 },
    { 0x76E78D, { 0x8B, 0x90, 0x28, 0x01, 0x00, 0x00 }, { 0x31, 0xD2, 0xEB, 0x02, 0x90, 0x90 }, 6 },
    { 0x76E80E, { 0x8B, 0xB0, 0x28, 0x01, 0x00, 0x00 }, { 0x31, 0xF6, 0xEB, 0x02, 0x90, 0x90 }, 6 },
    { 0x787227, { 0xF3, 0x0F, 0x10, 0x83, 0xAC, 0x00, 0x00, 0x00 },
        { 0x0F, 0x57, 0xC0, 0xEB, 0x03, 0x90, 0x90, 0x90 }, 8 },
};
constexpr int kSiteCount = sizeof(kSites) / sizeof(kSites[0]);

uintptr_t g_base = 0;
bool g_available = false;
bool g_on = false;

bool WriteBytes(unsigned char* at, const unsigned char* what, int len)
{
    DWORD old = 0;
    if (!VirtualProtect(at, len, PAGE_EXECUTE_READWRITE, &old))
        return false;
    std::memcpy(at, what, len);
    DWORD back = 0;
    VirtualProtect(at, len, old, &back);
    FlushInstructionCache(GetCurrentProcess(), at, len);
    return true;
}

} // namespace

void TremorPatch_Install()
{
    g_base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    if (!g_base)
        return;
    int matched = 0;
    for (int i = 0; i < kSiteCount; ++i) {
        const unsigned char* at = reinterpret_cast<const unsigned char*>(g_base + kSites[i].rva);
        unsigned char seen[8] = {};
        __try {
            std::memcpy(seen, at, kSites[i].len);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            continue;
        }
        // Already patched counts as matched: the trainer may have got there
        // first, and in that case we simply agree with it.
        if (std::memcmp(seen, kSites[i].was, kSites[i].len) == 0
            || std::memcmp(seen, kSites[i].now, kSites[i].len) == 0)
            ++matched;
        else
            Log_Printf("TremorPatch: exe+%X holds %02X %02X %02X %02X, not what this game build should have",
                static_cast<unsigned>(kSites[i].rva), seen[0], seen[1], seen[2], seen[3]);
    }
    g_available = matched == kSiteCount;
    Log_Printf("TremorPatch: %d of %d sites recognised - steady hands %s", matched, kSiteCount,
        g_available ? "available" : "unavailable on this build");
}

bool TremorPatch_IsAvailable()
{
    return g_available;
}

bool TremorPatch_IsOn()
{
    return g_on;
}

void TremorPatch_SetOn(bool on)
{
    if (!g_available || on == g_on)
        return;
    int done = 0;
    for (int i = 0; i < kSiteCount; ++i) {
        unsigned char* at = reinterpret_cast<unsigned char*>(g_base + kSites[i].rva);
        if (WriteBytes(at, on ? kSites[i].now : kSites[i].was, kSites[i].len))
            ++done;
    }
    g_on = on;
    Log_Printf("TremorPatch: hands %s (%d site(s) written)", on ? "steadied" : "back to the game's shake", done);
}
