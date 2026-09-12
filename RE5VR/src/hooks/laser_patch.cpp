#include "laser_patch.h"
#include "../util/log.h"

#include <windows.h>

#include <cstring>

// ---- Why (2026-09-11) --------------------------------------------------
// RE5 draws a laser sight when you aim with a gamepad and a crosshair when you
// aim with the mouse. In VR there is no HUD, so the laser is the only aiming
// aid - and the user wants it with the mouse on a monitor too.
//
// Found by diffing the game's code with the community trainer's laser-sight
// toggle off and on (state_probe.cpp captures): it changes exactly three
// conditional jumps (je, 74) into unconditional ones (jmp, EB). Two of them
// follow a call to re5dx9.exe+75C4F0, which the laser code asks before
// drawing and which answers "yes" in mouse mode; the third skips a call that
// sets [obj+0x2918] - by the look of it, the crosshair. This applies the same
// three bytes, after checking the original instructions are all there.
//
// (The first attempt looked for a data flag instead, and forcing candidate
// bytes from an outside process blue-screened the PC - see project memory.)

namespace {

struct Patch {
    DWORD rva;          // "test r8,r8 / je rel8"; the je opcode is at +2
    BYTE original[4];
};

constexpr Patch kPatches[] = {
    { 0x769A8F, { 0x84, 0xDB, 0x74, 0x17 } },
    { 0x776DBF, { 0x84, 0xC0, 0x74, 0x23 } },
    { 0x77714F, { 0x84, 0xC0, 0x74, 0x0B } },
};
constexpr int kJumpOpcode = 2;
constexpr BYTE kJmpShort = 0xEB;

bool g_applied = false;

// Original bytes, or already patched (the trainer's toggle is on).
bool LooksRight(const BYTE* at, const Patch& p)
{
    if (std::memcmp(at, p.original, sizeof(p.original)) == 0)
        return true;
    BYTE patched[4];
    std::memcpy(patched, p.original, sizeof(patched));
    patched[kJumpOpcode] = kJmpShort;
    return std::memcmp(at, patched, sizeof(patched)) == 0;
}

} // namespace

void LaserPatch_Install()
{
    if (g_applied)
        return;
    BYTE* exe = reinterpret_cast<BYTE*>(GetModuleHandleA(nullptr));

    // All or nothing: one unexpected site means a different game build.
    for (const Patch& p : kPatches) {
        const BYTE* at = exe + p.rva;
        if (!LooksRight(at, p)) {
            Log_Printf("LaserPatch: exe+%lX holds %02X %02X %02X %02X, not the expected instructions - laser left as the game has it",
                p.rva, at[0], at[1], at[2], at[3]);
            return;
        }
    }
    for (const Patch& p : kPatches) {
        BYTE* jump = exe + p.rva + kJumpOpcode;
        DWORD oldProtect = 0;
        VirtualProtect(jump, 1, PAGE_EXECUTE_READWRITE, &oldProtect);
        *jump = kJmpShort;
        VirtualProtect(jump, 1, oldProtect, &oldProtect);
        FlushInstructionCache(GetCurrentProcess(), jump, 1);
    }
    g_applied = true;
    Log_Printf("LaserPatch: laser sight forced on (exe+769A91, +776DC1, +777151: je -> jmp, same as the trainer's toggle)");
}
