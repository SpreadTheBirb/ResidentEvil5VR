#include "camera_rig_hook.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <atomic>

namespace {

bool g_enabled = false;

// Both Aim Camera hook address guesses crashed (see AimCameraRigHook_Stub
// below) before the real address was found by directly reading Cheat
// Engine's disassembly field-by-field instead of extrapolating further -
// now confirmed via real bytes, re-enabled.
constexpr bool kInstallAimCameraHook = true;

// Confirmed struct offsets (see project memory for the live CE session
// that found these - each one was independently value-matched and most
// were also verified via the "find out what writes + NOP the write"
// technique). Base pointer is whatever EDX holds at the hook site below -
// heap-allocated, different every game launch, but we never need to know
// it ourselves: the game hands it to us fresh every frame via EDX.
constexpr int kOffHorizontalDistance = 0x00;
constexpr int kOffVerticalDistance = 0x04;
constexpr int kOffDistance = 0x08;
constexpr int kOffHorizontalAdjust = 0x10;
constexpr int kOffVerticalAdjust = 0x14;
constexpr int kOffAngle = 0x18;
constexpr int kOffFOV = 0x24;

// Not among the trainer's 7 UI fields, and never written by us. Logged
// because of the 2026-09-10 pop-out: pitching the view swings the camera
// out to reveal Chris even with Distance=0, which means a boom length is
// coming from somewhere other than the seven fields we override. +0x0C is
// skipped entirely by the game's own copy sequence; +0x20 and +0x30 are
// written in the same block and were previously dismissed as internal
// damping/smoothing state. One of them is a candidate.
constexpr int kOffUnknown0C = 0x0C;
constexpr int kOffUnknown20 = 0x20;
constexpr int kOffUnknown30 = 0x30;

// Empirically-found "near first-person" values (found via the Hand to
// Hand/unarmed state, 2026-07-25 live session, but kept as the intended
// final values per user decision 2026-07-27 - RE5 has no holster
// mechanic so Hand to Hand isn't independently reachable to re-tune
// against; these are applied through the new hook below instead).
float g_targetHorizontalDistance = 0.0f;
float g_targetVerticalDistance = 173.0f;
float g_targetDistance = 0.0f;
float g_targetHorizontalAdjust = 0.0f;
float g_targetVerticalAdjust = 135.0f;
// TESTED AND RULED OUT (2026-09-10 21:09): Angle does NOT aim the boom.
// Hypothesis was that Angle is tenths of a degree and sets the boom
// elevation (208 -> 20.8 deg, close to the 19.6 deg measured at 21:00,
// and it would have made sense of the trainer's 110/148/175/220/224 as
// 11.0-22.4 deg). Built a test with Angle=400, predicting a ~40 deg boom.
// The measured elevation came back 19.6 deg, unchanged:
//
//   Angle=208 run:  direction (0.100, 0.335, 0.936) -> 19.6 deg
//   Angle=400 run:  direction (0.409, 0.335, 0.849) -> 19.6 deg
//
// Identical y-component to three decimals. The horizontal part differs
// only because the player was facing a different way. Don't re-try this.
float g_targetAngle = 208.0f;
float g_targetFOV = 42.0f;

void* g_trampoline = nullptr;
void* g_aimTrampoline = nullptr;

// ---- F4 diagnostic (2026-09-10) ---------------------------------------
// The 2026-09-10 VR test toggled F4 eleven times over 17 seconds and the
// decoded render camera was bit-identical on every single sample -
// camPos=(10516.6, 198.1, 7960.4) forward=(0.178, -0.297, -0.938)
// Sx=1.743 Sy=3.099 - with no difference at all between rig=ON and
// rig=OFF. Since the override writes FOV=42, a working override would
// have to move Sx. The user separately confirmed F4 does nothing on a
// flat screen either, so this is NOT VR-specific.
//
// Both hooks report MH_OK at install, so "installed" is not the question.
// The open question is whether the stubs actually EXECUTE in this camera
// state, and if so whether the struct they write is the one the camera
// is built from. These counters answer exactly that, and nothing else:
//
//   hits stay 0            -> the hook site never runs in this state.
//                             The +446895 block is the "Normal" camera
//                             write sequence; if the game is using some
//                             other camera path during normal gameplay,
//                             we are patching a road nobody drives on.
//   hits climb, before-values look like plausible rig parameters
//                          -> we are writing the right struct, and the
//                             game recomputes or overrides the camera
//                             downstream of our write.
//   hits climb, before-values are garbage
//                          -> the struct base is wrong (EDX at this site
//                             isn't what it was when it was found).
std::atomic<unsigned long long> g_normalHits{0};
std::atomic<unsigned long long> g_aimHits{0};
std::atomic<int> g_diagSamplesRemaining{0};

// ---- Does our write survive to the end of the frame? ------------------
// The 19:39 test answered the first question: both stubs fire constantly
// (NORMAL=854 / AIM=2536 in ~8s), and the structs hold real camera
// parameters - four distinct instances, two profiles (Angle=148 FOV=45
// and Angle=220 FOV=33), almost certainly Chris and Sheva x normal/aim.
// So the hook sites and offsets are right, and we override all four every
// frame, yet the rendered camera never moves.
//
// Note the "before-write" values are the game's own every time, but that
// proves nothing by itself - the game rewrites this struct each frame
// before our hook runs, so seeing its values there is expected.
//
// The real question is what happens AFTER we write. We remember every
// struct base we've seen and re-read all of them at EndScene, once the
// frame's rendering is finished:
//
//   values at EndScene are OURS      -> the write sticks and survives the
//                                       whole frame, so nothing overwrites
//                                       it - the camera simply isn't built
//                                       from these structs. Wrong target.
//   values at EndScene are the GAME'S -> something re-copies over us later
//                                       in the frame; we need a hook after
//                                       that copy, not after this one.
void ApplyOverride(void* structBase); // defined below

// ---- Lifetime, the hard way (2026-09-10 crash) ------------------------
// An earlier version of this kept every struct base it had ever seen and
// re-read/re-wrote them all at EndScene, guarded only by a 2-second
// "recently hit" staleness check on the write (and NO guard at all on the
// read). That crashed the game, and the log showed exactly why:
//
//   END-OF-FRAME #0 base=09AA23C0 (AIM, enabled=1 ageMs=19453 hits=3676)
//      HDist=-10.00 VDist=500.00 Dist=204.00 VAdj=45.00 FOV=167.00
//
// FOV=167 and VDist=500 are not camera values. That allocation had been
// freed 19 seconds earlier and the memory reused by something else; the
// dump walked the list into a page that was gone.
//
// The model behind that design was wrong. These are NOT a fixed set of
// per-weapon profile structs sitting at stable addresses. They are
// short-lived allocations the game creates and destroys continuously -
// 16 distinct bases appeared in about a minute (hitting the old array
// cap), always in AIM/NORMAL pairs 0x90 apart, and in that run every one
// of them carried identical values. Holding a pointer across frames was
// never safe.
//
// So the rule now: only ever touch a base that a live hook hit recorded
// during the current EndScene window. That is the only real proof the
// allocation still exists. The list is collected by the stubs, used once
// at EndScene, and cleared - it never survives into a later frame.
constexpr int kMaxLiveBases = 32;

struct LiveBase {
    void* base;
    bool aim;
    unsigned long long lastHitMs;
};
LiveBase g_liveBases[kMaxLiveBases] = {};
std::atomic<int> g_liveCount{0};
std::atomic<int> g_endSceneDumpsRemaining{0};

// ---- Watch mode: what takes over when the camera pops out? ------------
// 2026-09-10 20:17, first VR run with first-person working: "the fps
// camera doesn't currently lock in place, so when moving the mouse, the
// camera still pops out to reveal chris". The override holds until the
// player looks around, then something restores a third-person distance.
//
// The trainer's Normal tab has "Freeze cam 1", "Freeze cam 2" and
// "Freeze melee camera" checkboxes alongside the Normal/Aim panels -
// i.e. the game has more camera slots than the two this file hooks.
// Mouse-look is probably handing control to one of those.
//
// Dumping every frame would be thousands of lines, so instead log only
// when the SET of live struct bases changes from the previous EndScene.
// A base appearing exactly when the camera pops out is the culprit, and
// its values identify which trainer tab/slot it belongs to.
std::atomic<int> g_watchLogsRemaining{0};
uintptr_t g_prevLiveSignature = 0;

// How long a base stays eligible for the freeze after its last live hook
// hit. The 20:25 trace showed why this cannot be "only this EndScene
// call": EndScene runs several times per frame, and passes with no hook
// hits logged "0 live struct(s) this frame", so the freeze wrote nothing
// at all on those - which is exactly the gap the game's other write path
// slipped through, reverting a struct to its third-person values and
// popping the camera out.
//
// 250ms is chosen against measured behaviour, not guessed: the stubs fire
// ~220 times/sec, so any genuinely live struct is re-hit within ~5ms. A
// base silent for 250ms is 50x past that and is not in active use. This
// is also two orders of magnitude tighter than the 2000ms window whose
// stale pointers crashed the game at 20:01 (that log showed a base
// ageMs=19453 - freed 19 seconds earlier and its memory reused).
constexpr unsigned long long kLiveBaseExpiryMs = 250;

void RememberLiveBase(void* structBase, bool aim)
{
    const unsigned long long now = GetTickCount64();
    const int n = g_liveCount.load(std::memory_order_relaxed);
    for (int i = 0; i < n; ++i) {
        if (g_liveBases[i].base == structBase) {
            g_liveBases[i].lastHitMs = now;
            return;
        }
    }
    if (n < kMaxLiveBases) {
        g_liveBases[n].base = structBase;
        g_liveBases[n].aim = aim;
        g_liveBases[n].lastHitMs = now;
        g_liveCount.store(n + 1, std::memory_order_relaxed);
    }
}

// Drop entries whose last live hit is older than the expiry window, so the
// list can never accumulate a pointer the game has since freed.
void ExpireLiveBases()
{
    const unsigned long long now = GetTickCount64();
    const int n = g_liveCount.load(std::memory_order_relaxed);
    int out = 0;
    for (int i = 0; i < n; ++i) {
        if (g_liveBases[i].base && now - g_liveBases[i].lastHitMs <= kLiveBaseExpiryMs) {
            if (out != i)
                g_liveBases[out] = g_liveBases[i];
            ++out;
        }
    }
    g_liveCount.store(out, std::memory_order_relaxed);
}

// The freeze: re-apply the override to every base seen alive recently,
// after the game's whole frame of updates has run, making us the last
// writer.
void FreezeLiveBases()
{
    const int n = g_liveCount.load(std::memory_order_relaxed);
    for (int i = 0; i < n; ++i) {
        if (g_liveBases[i].base)
            ApplyOverride(g_liveBases[i].base);
    }
}

// Bounded to a dozen lines per F4 press: this runs on the game thread
// inside a naked stub, mid-function, so it must not log every frame.
void RecordHit(void* structBase, bool aim)
{
    if (aim)
        g_aimHits.fetch_add(1, std::memory_order_relaxed);
    else
        g_normalHits.fetch_add(1, std::memory_order_relaxed);

    if (structBase)
        RememberLiveBase(structBase, aim);

    const int remaining = g_diagSamplesRemaining.load(std::memory_order_relaxed);
    if (remaining <= 0 || !structBase)
        return;
    g_diagSamplesRemaining.store(remaining - 1, std::memory_order_relaxed);

    const unsigned char* base = static_cast<const unsigned char*>(structBase);
    const auto readF = [base](int off) {
        return *reinterpret_cast<const float*>(base + off);
    };
    Log_Printf("CameraRigHook: %s stub HIT (base=%p, enabled=%d) before-write "
               "HDist=%.2f VDist=%.2f Dist=%.2f HAdj=%.2f VAdj=%.2f Angle=%.2f FOV=%.2f",
        aim ? "AIM" : "NORMAL", structBase, g_enabled ? 1 : 0,
        readF(kOffHorizontalDistance), readF(kOffVerticalDistance),
        readF(kOffDistance), readF(kOffHorizontalAdjust),
        readF(kOffVerticalAdjust), readF(kOffAngle), readF(kOffFOV));
}

void ApplyOverride(void* structBase)
{
    if (!g_enabled || !structBase)
        return;
    unsigned char* base = static_cast<unsigned char*>(structBase);
    *reinterpret_cast<float*>(base + kOffHorizontalDistance) = g_targetHorizontalDistance;
    *reinterpret_cast<float*>(base + kOffVerticalDistance) = g_targetVerticalDistance;
    *reinterpret_cast<float*>(base + kOffDistance) = g_targetDistance;
    *reinterpret_cast<float*>(base + kOffHorizontalAdjust) = g_targetHorizontalAdjust;
    *reinterpret_cast<float*>(base + kOffVerticalAdjust) = g_targetVerticalAdjust;
    *reinterpret_cast<float*>(base + kOffAngle) = g_targetAngle;
    *reinterpret_cast<float*>(base + kOffFOV) = g_targetFOV;
}

// Called from the naked stub below with EDX (the struct base) pushed as
// its one argument. Plain __cdecl - the stub cleans up the pushed arg.
extern "C" void CameraRigHook_OnWriteComplete(void* edxValue)
{
    RecordHit(edxValue, /*aim=*/false);
    ApplyOverride(edxValue);
}

// Same thing for the Aim Camera stub, split out only so the diagnostic can
// tell the two hook sites apart - which one fires during normal play is
// exactly what we don't know yet.
extern "C" void CameraRigHook_OnAimWriteComplete(void* structBase)
{
    RecordHit(structBase, /*aim=*/true);
    ApplyOverride(structBase);
}

// Naked hook stub, installed at re5dx9.exe+446895 - the "Normal" camera
// state's write-sequence block (2026-07-27), a different code location
// from the old Hand-to-Hand block this used to hook (+4467D1, now
// retired - RE5 has no holster mechanic, so Hand to Hand isn't reachable
// during normal play and doesn't need its own hook; "Normal" is treated
// as the universal profile per user decision).
//
// UNLIKE the old block, this one is a tight back-to-back struct copy
// (fld [eax+N] / fstp [edx+N] per field, no interleaved computation).
// +446895 was reached in two steps: extrapolating the confirmed
// +0x00/+0x04 pair spacing forward predicted +446894 (one byte early),
// which crashed the game on level load - Windows Error Reporting showed
// exception 0xc0000096 (STATUS_PRIVILEGED_INSTRUCTION) at fault offset
// +446895, meaning some other branch in the function jumps directly into
// that byte and our 5-byte JMP patch (starting at +446894) corrupted it.
// Shifting the hook to +446895 (the address the crash dump itself
// pointed to) confirmed working in-game 2026-07-27: clean level load,
// F4 toggled the override on/off repeatedly with correct visible results,
// no crashes. Same reasoning as the old block still applies: this is the
// instruction *after* the last field write (FOV), never mid-sequence, so
// nothing is left to clobber our override on the same frame.
//
// This is NOT a normal call boundary - execution just falls through into
// this address sequentially as part of the surrounding function, so the
// stub must preserve every register/flag it touches.
__declspec(naked) void CameraRigHook_Stub()
{
    __asm {
        pushad
        pushfd
        push edx
        call CameraRigHook_OnWriteComplete
        add esp, 4
        popfd
        popad
        jmp g_trampoline
    }
}

// Aim Camera naked hook stub, installed at re5dx9.exe+4466E9 (2026-07-27) -
// the Aim Camera state's write-sequence block, found live via Cheat Engine
// the same way as the Normal block above (trainer's Aim Camera tab,
// exact-value scan while holding aim, "find out what writes"). Same
// relative field layout as Normal/the original Hand-to-Hand struct
// (confirmed: the write to +0x00 landed at esi+0x420, and the very next
// field write in the same trace landed at esi+0x424, matching the known
// +0x04 = Vertical Distance offset) - but this struct is embedded inside
// a larger object at a large, disp32-encoded offset (esi+0x420) rather
// than being its own small allocation based directly in a register like
// the Normal block's EDX. So the struct base ApplyOverride() needs is
// ESI+0x420, not ESI itself.
//
// The hook address was first calculated the same way as the Normal
// block's - extrapolating the confirmed +0x00/+0x04 pair spacing (here
// 9 bytes/field: 3-byte fld + 6-byte fstp, since esi+0x420 and beyond
// all need disp32 encoding) through to the end of the FOV (+0x24) pair.
// Two extrapolated guesses (+4466E9, then +4466ED corrected from the
// first crash's WER fault offset) both crashed the game on level load -
// the second crash (0xc0000005 ACCESS_VIOLATION at fault offset
// +4466E8, not matching the clean "corrupted JMP byte" pattern the first
// crash and the Normal block's crash both showed) didn't point at an
// obvious next guess, so extrapolation was abandoned in favor of reading
// Cheat Engine's disassembly directly, field by field via repeated
// "find out what writes" captures. That surfaced a real surprise no
// extrapolation could have predicted: the write sequence **skips +0x0C
// entirely** (goes straight from +0x08 Distance's write to +0x10
// Horizontal Adjustment's, no pair between them - consistent with the
// Normal/original struct, which also never showed a +0x0C write), and
// there's an extra `mov eax,[esi+0x2B8]` instruction reloading the
// source pointer immediately before the FOV field's `fld`, something no
// byte-counting could have anticipated. The confirmed FOV write
// completes at 008466D7 (`fstp [esi+0x444]`, 6 bytes), so the real
// hook point - read directly off the disassembly, not calculated - is
// the very next instruction, 008466DD (`re5dx9.exe+4466DD`). Confirmed
// working in-game 2026-07-27: clean level load, F4 toggled the override
// correctly while aiming, no crashes.
__declspec(naked) void AimCameraRigHook_Stub()
{
    __asm {
        pushad
        pushfd
        lea eax, [esi + 0x420]
        push eax
        call CameraRigHook_OnAimWriteComplete
        add esp, 4
        popfd
        popad
        jmp g_aimTrampoline
    }
}

} // namespace

void CameraRigHook_Install()
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("CameraRigHook_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    uintptr_t moduleBase = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    void* target = reinterpret_cast<void*>(moduleBase + 0x446895);

    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&CameraRigHook_Stub), &g_trampoline);
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("CameraRigHook_Install: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("CameraRigHook_Install: camera rig hook enabled -> %d (target=%p)", static_cast<int>(st), target);

    if (!kInstallAimCameraHook) {
        Log_Printf("CameraRigHook_Install: aim camera hook skipped (kInstallAimCameraHook=false, address not yet confirmed)");
        return;
    }

    void* aimTarget = reinterpret_cast<void*>(moduleBase + 0x4466DD);
    MH_STATUS aimSt = MH_CreateHook(aimTarget, reinterpret_cast<void*>(&AimCameraRigHook_Stub), &g_aimTrampoline);
    if (aimSt != MH_OK && aimSt != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("CameraRigHook_Install: aim camera MH_CreateHook failed -> %d", static_cast<int>(aimSt));
        return;
    }
    aimSt = MH_EnableHook(aimTarget);
    Log_Printf("CameraRigHook_Install: aim camera rig hook enabled -> %d (target=%p)", static_cast<int>(aimSt), aimTarget);
}

void CameraRigHook_OnEndScene()
{
    static bool prevF4Down = false;
    bool f4Down = (GetAsyncKeyState(VK_F4) & 0x8000) != 0;
    if (f4Down && !prevF4Down) {
        g_enabled = !g_enabled;
        Log_Printf("CameraRigHook: F4 pressed, first-person camera override now %s", g_enabled ? "ON" : "OFF");
        // Stub hit counts since process start. If these stay at 0 across a
        // whole session, the hook sites are never executed and no amount of
        // tuning the written values will ever do anything.
        Log_Printf("CameraRigHook: stub hits so far - NORMAL=%llu AIM=%llu",
            g_normalHits.load(std::memory_order_relaxed),
            g_aimHits.load(std::memory_order_relaxed));
        g_diagSamplesRemaining.store(12, std::memory_order_relaxed);
        g_endSceneDumpsRemaining.store(3, std::memory_order_relaxed);
        g_watchLogsRemaining.store(g_enabled ? 120 : 0, std::memory_order_relaxed);
        g_prevLiveSignature = 0;
    }

    // Retire anything that has gone quiet before touching the list at all:
    // a pointer the game may already have freed must never be read or
    // written - that is exactly what crashed the 20:01 run.
    ExpireLiveBases();
    const int liveNow = g_liveCount.load(std::memory_order_relaxed);
    const int dumps = g_endSceneDumpsRemaining.load(std::memory_order_relaxed);
    if (dumps > 0) {
        g_endSceneDumpsRemaining.store(dumps - 1, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: %d live struct(s) this frame (enabled=%d)",
            liveNow, g_enabled ? 1 : 0);
        for (int i = 0; i < liveNow; ++i) {
            const unsigned char* base = static_cast<const unsigned char*>(g_liveBases[i].base);
            if (!base)
                continue;
            const auto readF = [base](int off) {
                return *reinterpret_cast<const float*>(base + off);
            };
            Log_Printf("CameraRigHook: END-OF-FRAME #%d base=%p (%s) "
                       "HDist=%.2f VDist=%.2f Dist=%.2f HAdj=%.2f VAdj=%.2f Angle=%.2f FOV=%.2f "
                       "| unknown +0x0C=%.2f +0x20=%.2f +0x30=%.2f",
                i, g_liveBases[i].base, g_liveBases[i].aim ? "AIM" : "NORMAL",
                readF(kOffHorizontalDistance), readF(kOffVerticalDistance),
                readF(kOffDistance), readF(kOffHorizontalAdjust),
                readF(kOffVerticalAdjust), readF(kOffAngle), readF(kOffFOV),
                readF(kOffUnknown0C), readF(kOffUnknown20), readF(kOffUnknown30));
        }
    }

    // Watch mode: log only when the set of live bases changes, so looking
    // around produces a readable trace instead of thousands of lines.
    const int watch = g_watchLogsRemaining.load(std::memory_order_relaxed);
    if (watch > 0 && liveNow > 0) {
        uintptr_t sig = static_cast<uintptr_t>(liveNow);
        for (int i = 0; i < liveNow; ++i)
            sig ^= reinterpret_cast<uintptr_t>(g_liveBases[i].base) * 31u + i;

        if (sig != g_prevLiveSignature) {
            g_prevLiveSignature = sig;
            g_watchLogsRemaining.store(watch - 1, std::memory_order_relaxed);
            for (int i = 0; i < liveNow; ++i) {
                const unsigned char* base = static_cast<const unsigned char*>(g_liveBases[i].base);
                if (!base)
                    continue;
                const auto readF = [base](int off) {
                    return *reinterpret_cast<const float*>(base + off);
                };
                Log_Printf("CameraRigHook: WATCH set-change (%d live) #%d base=%p (%s) "
                           "HDist=%.2f VDist=%.2f Dist=%.2f HAdj=%.2f VAdj=%.2f Angle=%.2f FOV=%.2f "
                           "| unknown +0x0C=%.2f +0x20=%.2f +0x30=%.2f",
                    liveNow, i, g_liveBases[i].base, g_liveBases[i].aim ? "AIM" : "NORMAL",
                    readF(kOffHorizontalDistance), readF(kOffVerticalDistance),
                    readF(kOffDistance), readF(kOffHorizontalAdjust),
                    readF(kOffVerticalAdjust), readF(kOffAngle), readF(kOffFOV),
                    readF(kOffUnknown0C), readF(kOffUnknown20), readF(kOffUnknown30));
            }
        }
    }

    // The freeze runs AFTER the dump above, so the logged values still show
    // what the game's own frame left behind rather than what we just wrote.
    FreezeLiveBases();
    prevF4Down = f4Down;
}

bool CameraRigHook_IsEnabled()
{
    return g_enabled;
}
