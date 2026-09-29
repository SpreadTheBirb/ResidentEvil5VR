#include "camera_rig_hook.h"
#include "culling_patch.h"

#include "arm_ik.h"
#include "../net/ik_sync.h"
#include "aim_finder.h"
#include "constant_probe.h"
#include "fade_patch.h"
#include "../render/stereo_test.h"
#include "../vr/openxr_bridge.h"
#include "../vr/xr_input.h"
#include "../ui/input_block.h"
#include "../util/log.h"
#include "../util/build_config.h"

#include <MinHook.h>
#include <windows.h>
#include <intrin.h>
#include <Xinput.h>
#include <cstdlib>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>

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

// ---- What these structs actually are (2026-09-11) ----------------------
// Read from the disassembly of the camera function at re5dx9.exe+446500
// (taken from an in-memory code dump - the exe is Steam-DRM wrapped on disk,
// see boom_finder.cpp). The trainer's names are misleading: each struct is
// two points in the character's yaw space plus a FOV -
//   +0x00/+0x04/+0x08  EYE    x, y, z  (trainer: H Dist, V Dist, Distance)
//   +0x10/+0x14/+0x18  TARGET x, y, z  (trainer: H Adj, V Adj, Angle)
//   +0x24              FOV
// and they come in THREES, 0x30 apart, per mode: normal at esi+0x480/+0x4B0/
// +0x4E0, aim at esi+0x3F0/+0x420/+0x450. The game has no camera pitch as
// such. A pitch-driven blend factor at esi+0x1D0 (-1..+1) interpolates eye
// and target from the middle rig toward the "look up" rig (first, factor >
// 0) or the "look down" rig (third, factor < 0).
//
// That is the "boom". The old hooks only ever wrote the MIDDLE rig, so
// looking up or down slid the camera in a straight line toward an untouched
// third-person rig. Checked against the 2026-09-10 measurement: Chris's
// look-down rig eye is (-40, 242, -190); from our eye (0, 173, 0) that is a
// direction 19.56 deg above horizontal. The measured boom was 19.6 deg.
constexpr int kRigStride = 0x30;
constexpr int kOffNormalRigs = 0x480;   // three rigs from here, kRigStride apart
constexpr int kOffAimRigs = 0x3F0;
constexpr int kOffNormalMiddle = 0x4B0; // == the legacy NORMAL hook's EDX
constexpr int kOffAimMiddle = 0x420;    // == the legacy AIM hook's esi+0x420

// The legacy per-mode write hooks (+446895 / +4466DD, below) are superseded
// by the single rigs-ready hook. Kept for reference only: they must NOT run
// alongside it, because the new hook reads each rig's original view
// direction and they would have already flattened the middle ones.
constexpr bool kInstallLegacyWriteHooks = false;

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
void* g_rigsReadyTrampoline = nullptr;

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
bool GameIsDrivingTheCamera();

void KeepTheRigCollapsed()
{
    if (!g_enabled)
        return;
    void* bases[8] = {};
    bool aim[8] = {};
    const int n = CameraRigHook_GetLiveBases(bases, aim, 8);
    for (int i = 0; i < n; ++i) {
        __try {
            ApplyOverride(bases[i]);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
        }
    }

    // THE BLIND RIG SWEEP IS OUT (2026-09-28: "collapsed 8 rig-shaped
    // block(s)", forty crashes, and nothing held).
    //
    // Restricting it to the rig grid cut the writes from twenty four to eight,
    // and eight is still almost every slot on that grid - so the test is
    // matching structures that are not rigs and we were corrupting them for no
    // benefit. Blind writing on a shape test does not work here at any width,
    // and two rounds of it cost more than they found.
    //
    // What survives: the melee camera is not a fourth entry in the rig array.
    static unsigned long long s_toldAt = 0;
    const unsigned long long now = GetTickCount64();
    if (n && now - s_toldAt > 5000) {
        s_toldAt = now;
        Log_Printf("CameraRig: holding %d rig base(s) collapsed before every view, not only when the game "
                   "builds them",
            n);
    }
}


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
// CALLED FROM ASM, SO IT GOES THROUGH A POINTER (2026-09-29, user: "how
// wouldn't this work on a release build but works fine on dev").
//
// Release builds this project with WholeProgramOptimization - /GL - and MSVC
// is then free to give a function a calling convention of its own choosing,
// because it believes it can see every call site and fix them all up. Inline
// asm call sites are invisible to it. So a function the asm passes an
// argument to on the stack can be compiled to expect it in a register, and
// what arrives is whatever happened to be there.
//
// That is exactly what broke 6DOF in release and nowhere else: the arm pose
// hook fired, PoseComplete ran, and the joint pointer it received was
// rubbish - so every joint failed the "is this one of ours" test and not a
// single rotation was ever written back. The arms stayed in the animation.
//
// A call through a pointer of declared type cannot be rewritten that way.
// The two stubs added most recently already did this, which is why they were
// the only ones that kept working.
typedef void(__cdecl* CallOnWriteComplete_t)(void*);
CallOnWriteComplete_t g_callOnWriteComplete = &CameraRigHook_OnWriteComplete;

__declspec(naked) void CameraRigHook_Stub()
{
    __asm {
        pushad
        pushfd
        push edx
        call g_callOnWriteComplete
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
typedef void(__cdecl* CallOnAimWriteComplete_t)(void*);
CallOnAimWriteComplete_t g_callOnAimWriteComplete = &CameraRigHook_OnAimWriteComplete;

__declspec(naked) void AimCameraRigHook_Stub()
{
    __asm {
        pushad
        pushfd
        lea eax, [esi + 0x420]
        push eax
        call g_callOnAimWriteComplete
        add esp, 4
        popfd
        popad
        jmp g_aimTrampoline
    }
}

// ---- The fix: one hook after every rig copy (2026-09-11) ---------------
// re5dx9.exe+446965 (`movss xmm1,[esi+630h]`) is where all the copy paths
// merge. Both mode groups have been refreshed from their profile sources by
// then - including a SECOND copy of the normal rigs, taken when [esi+1B3]
// is set, which ran after the legacy +446895 hook and silently overwrote it
// (the "profile C" structs that never took our values on 2026-09-10).
// Nothing between here and the blend rewrites the rigs. It is a jump target
// (je at +4468C3), but only at its first byte, which is where our JMP goes.
//
// Rather than stamping one fixed rig over all six, keep each rig's own view
// DIRECTION and move only its eye to the head: eye = the first-person eye,
// target = eye + the rig's original (target - eye), sideways part dropped.
// The blend then only swings the target, so pitching rotates the view about
// a fixed eye - the game's own pitch range for whatever profile is active,
// with no boom. FOV is flattened too so pitching doesn't zoom.
//
// Only x87/integer code runs in this function before this point, so no XMM
// register is live across the hook and the callback's SSE math is safe
// under the stub's pushad/pushfd.
constexpr float kFirstPersonTargetDistance = 208.0f;

// User decision 2026-09-11, after the first working test: the aim presets
// reach much further (Chris: ~48 deg up / ~50 deg down, vs ~29 deg for the
// normal ones) and that range "feels more natural in first person", so the
// normal presets borrow the aim presets' view directions. Both modes share
// the one blend factor at esi+0x1D0, so this also means entering or leaving
// aim no longer shifts the view vertically. (Irrelevant once VR drives pitch
// from the headset; this is for first-person flat-screen comfort.)
constexpr bool kNormalUsesAimPitchRange = true;

// TRIED AND REVERTED (2026-09-11): tilting the aim presets' view below
// their directions, to raise the pistol on screen when aiming level (it
// sits very low, flat-screen only). The aim itself follows the view, so it
// just made Chris aim toward the floor instead of where you look. If the
// gun height is ever revisited, move the ARMS (shoulder joints 45/75),
// never the camera.

// A rig's view direction in the character's vertical plane, unit length,
// sideways part dropped. False for a degenerate rig (eye == target), e.g. a
// profile that doesn't populate its aim presets.
bool RigViewDirection(const unsigned char* rig, float* outDy, float* outDz)
{
    const float* f = reinterpret_cast<const float*>(rig); // f[0..2] eye, f[4..6] target
    const float dy = f[5] - f[1];
    const float dz = f[6] - f[2];
    const float len = std::sqrt(dy * dy + dz * dz);
    if (len < 1e-3f)
        return false;
    *outDy = dy / len;
    *outDz = dz / len;
    return true;
}

// User request 2026-09-11: FOV 90 in first person, in the usual shooter
// sense (horizontal). The rig's FOV field is VERTICAL degrees - at the old
// 42 the projection's vertical scale was exactly cot(21 deg) = 2.605 - so
// convert through the screen aspect, read off the game's own projection
// (Sy/Sx) in LastCameraPosition. 90 horizontal at 16:9 is ~58.7 vertical.
// In VR this does not change the headset image at all: stereo_test.cpp
// builds each eye's projection from OpenXR's own FOV. There it only widens
// the frustum the game culls with, which helps.
float g_flatFovDeg = 90.0f; // horizontal; Ctrl + ',' / '.' tune it live
constexpr float kPi = 3.14159265358979f;

float FirstPersonVerticalFov()
{
    // Aspect comes from the BACKBUFFER, not from whatever camera matrix a
    // pass happened to leave cached (2026-09-12). The old way read sy/sx off
    // the cached matrix, and the game runs plenty of passes whose projection
    // isn't the screen's shape - catch a squarish one at the wrong moment and
    // the aspect lands near 1.0, which turns a 90 deg horizontal FOV into a
    // 90 deg VERTICAL one and fisheyes the view. That is the "FOV bug" the
    // tester reported as FOV fighting, reproduced by the user on 2026-09-12
    // by pressing F4 in flat mode; entering VR hid it because VR uses the
    // fixed cull FOV constant instead of this function.
    const float aspect = StereoTest_GetBackbufferAspect();
    const float halfH = g_flatFovDeg * 0.5f * kPi / 180.0f;
    return 2.0f * std::atan(std::tan(halfH) / aspect) * 180.0f / kPi;
}

// ---- VR-only tuning (2026-09-11) ----------------------------------------
// Set from the render thread in CameraRigHook_OnEndScene (which is where the
// VR bridge is used), read by the camera hook on the game thread.
std::atomic<bool> g_vrActive{false};

// Wide game FOV in VR, for culling only (2026-09-11, second attempt). The
// headset image never uses the game FOV - stereo_test.cpp builds each eye
// from OpenXR's own - so this only widens the frustum the game culls with.
//
// The first attempt was written off as a dead end, but that test couldn't
// tell: its test object was a fence over the shoulder (past +-85 deg, out of
// reach of ANY frustum around the game's forward), and the black-patch bugs
// in stereo_test.cpp - fixed since - were being read as culling. The clean
// case now: looking straight at Sheva, her legs are culled. They sit below
// the game camera's ~59 deg vertical view but inside the headset's, and the
// model sphere test (culling_patch.cpp) only sees ~2 objects a frame, so
// whatever culls them is something else working from the game camera.
// 150 deg vertical also covers looking down with the head, which the game
// camera doesn't follow. '`' toggles it for A/B in the headset.
// ---- VR camera stabilisation (F11, default ON, 2026-09-12) -------------
// The rig sits on the character's head JOINT, which breathes, sways and
// settles under idle animation. On a monitor that reads as life; in a
// headset it is the view shaking by itself, and it also breaks reprojection,
// because none of that motion is in the pose we hand OpenXR - the compositor
// is told the head didn't move while the image says otherwise. The user
// confirmed it 2026-09-12: standing still, no stick, no mouse, head-only -
// still jittery, and only F9 (which overrides where the camera points) made
// it better.
//
// So in VR the eye position is low-passed: slow motion (walking, crouching,
// genuinely leaning) passes through, animation shake does not. A time
// constant rather than a hard freeze, so the camera still follows the
// character instead of detaching from him.
std::atomic<bool> g_vrStabiliseEye{true};
constexpr float kEyeSmoothTimeConstantSec = 0.12f;

std::atomic<bool> g_vrWideFov{true};
// 150 is a ceiling, not a preference. 179 was tried on 2026-09-12 to get
// "cull nothing" and the headset image went visibly FISHEYE - so the claim
// this comment used to make, that the game's FOV never reaches the VR image
// because stereo_test builds each eye from OpenXR's own FOV, is WRONG. The
// game's projection still feeds the matrices we transform. Widening the cull
// frustum therefore costs image distortion, and 150 is about where that
// stops being noticeable. Fixing culling properly means decoupling the two,
// not turning this number up.
constexpr float kVrCullVerticalFovDeg = 150.0f; // fallback only, see VrCullVerticalFov

// Derive the culling FOV from the headset's REAL frustum instead of a magic
// number (2026-09-12). 90 was too narrow - the Quest 3 shows ~100-110 deg per
// eye, so a partner's legs sat outside the game's cone unless you looked
// straight at them, with F9 on or off. 150 covered everything but cost LOD on
// most of the scene, because LOD is picked from projected screen size. The
// headset already tells us its exact per-eye angles every frame; the right
// answer is "everything the wearer can physically see, plus a margin", which
// is far narrower than 150 and never too narrow.
// A player option since 2026-09-14: a tester's clip showed things popping in
// when looking over the shoulder. More margin draws them before they reach
// the edge of your view, at the cost of LOD and draw calls (see above).
std::atomic<float> g_vrCullMarginPct{ 15.0f };
std::atomic<float> g_vrHeadSteady{ 0.35f };
std::atomic<float> g_vrRunLead{ 4.0f };
// How much of the idle animation to keep out of your head, 0 to 1.
std::atomic<float> g_vrViewSteady{ 0.7f };
// Where the running lead put the eye this frame (2026-09-23). Anything else
// that works out where the eye belongs - the body eye used while the game has
// the camera - has to land in the same place, or the two disagree by up to a
// metre at a run and every test between them reads as the camera running off.
// Set from anywhere: the cached arm joints are no longer the arms.
std::atomic<bool> g_forgetArms{ false };
std::atomic<float> g_leadX{ 0.0f };
std::atomic<float> g_leadZ{ 0.0f };

// The camera the renderer asks for the player view of, and the routine that
// puts it back on your head each frame. Both defined further down, beside the
// disassembly that explains them; declared here because the view hook stores
// the pointer and EndScene calls the routine, and both come first in the file.
std::atomic<void*> g_theCamera{ nullptr };

// The camera position as the GAME left it, captured just before the pin
// overwrites it. See the note in BuildHeadLockedView: the view direction is
// measured from this, and measuring it from the pinned value instead is what
// the walking stutter turned out to be.
float g_gameEye[3] = {};
bool g_haveGameEye = false;
void* g_gameEyeFrom = nullptr; // which camera it was read from

// Where the pin last decided your eye is, for the look-at hook below to reuse
// rather than working it out a second time and disagreeing by a hair.
float g_pinnedEye[3] = {};
bool g_havePinnedEye = false;
void* g_theCameraRaw = nullptr; // the same pointer, for the asm stub to compare
std::atomic<bool> g_vrEyeOnNeck{ true };
std::atomic<float> g_vrEyeAboveNeck{ 20.0f };
constexpr float kVrCullMarginMaxPct = 100.0f;
constexpr float kVrCullFovMinDeg = 90.0f;
// 178, not 170 (2026-09-15): the tester's +60% on a Quest 3 already reached
// 158 and still asked for more ("80% or even higher"). 180 is a flat plane
// and can't be a frustum; 178 is as close as it's sensible to go.
constexpr float kVrCullFovMaxDeg = 178.0f;
// What VrCullVerticalFov returns with the head still (no turn widening), for
// the menu. 0 until VR has run it.
std::atomic<float> g_vrCullFovDegShown{ 0.0f };

// Widen while turning (2026-09-15). With head tracking the cone already turns
// with the head (the tester's log: head 148 deg, camera 149 deg), but the game
// picks what to draw from where its camera pointed a moment earlier, so on a
// fast turn your view runs ahead of the cone and meets its edge - worst on a
// hard look behind, the fastest turn there is. Rather than a huge margin all
// the time (lower detail everywhere), add the angle the head covers in this
// much time at its current turn speed, on every side, and ease back after the
// turn. Only the culling angle changes; where the camera points and the
// headset image do not.
std::atomic<float> g_vrCullTurnLookaheadMs{ 0.0f }; // off since the view split fixed culling at its root
constexpr float kVrCullTurnLookaheadMaxMs = 300.0f;
constexpr float kTurnSpeedReleaseSec = 0.35f; // how long the widening takes to fade after a turn
constexpr float kTurnGapMaxDeg = 60.0f;

// Head turn speed in deg/sec from the pose latched each frame: rises at once,
// fades over kTurnSpeedReleaseSec, since the camera is still catching up for a
// moment after the head stops.
float HeadTurnSpeedEnvelope()
{
    static float s_prev[3] = { 0.0f, 0.0f, 1.0f };
    static bool s_havePrev = false;
    static LARGE_INTEGER s_prevTime = {};
    static float s_envelope = 0.0f;
    static LARGE_INTEGER s_freq = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return f; }();

    float fwd[3];
    if (!StereoTest_GetLatchedHeadForward(fwd)) {
        s_havePrev = false;
        s_envelope = 0.0f;
        return 0.0f;
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    const float dt = s_havePrev
        ? static_cast<float>(now.QuadPart - s_prevTime.QuadPart) / static_cast<float>(s_freq.QuadPart)
        : 0.0f;
    // The latched pose only changes once a frame while this runs several
    // times a frame; let a few milliseconds pass between samples.
    if (s_havePrev && dt < 0.004f)
        return s_envelope;
    if (s_havePrev && dt > 0.0f) {
        const float len = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
        const float plen = std::sqrt(s_prev[0] * s_prev[0] + s_prev[1] * s_prev[1] + s_prev[2] * s_prev[2]);
        float cosA = len > 1e-4f && plen > 1e-4f
            ? (fwd[0] * s_prev[0] + fwd[1] * s_prev[1] + fwd[2] * s_prev[2]) / (len * plen)
            : 1.0f;
        cosA = std::fmax(-1.0f, std::fmin(1.0f, cosA));
        // A long gap is a pause (menu, loading), not a turn.
        const float speed = dt < 0.25f ? std::acos(cosA) * 180.0f / kPi / dt : 0.0f;
        const float decayed = s_envelope * std::exp(-dt / kTurnSpeedReleaseSec);
        s_envelope = std::fmax(speed, decayed);
    }
    std::memcpy(s_prev, fwd, sizeof(s_prev));
    s_prevTime = now;
    s_havePrev = true;
    return s_envelope;
}

float VrCullVerticalFov()
{
    XRBridgeEyeView l, r;
    if (!VRBridge_GetEyeViews(l, r))
        return kVrCullVerticalFovDeg;

    // OpenXR angles are signed from the view axis: left/down negative, so the
    // union across both eyes is the widest excursion on each side.
    const float up = std::fmax(l.angleUp, r.angleUp);
    const float down = std::fmax(-l.angleDown, -r.angleDown);
    const float left = std::fmax(-l.angleLeft, -r.angleLeft);
    const float right = std::fmax(l.angleRight, r.angleRight);
    if (up + down <= 0.0f || left + right <= 0.0f)
        return kVrCullVerticalFovDeg;

    // The rig's FOV field is VERTICAL, and the game widens it horizontally by
    // the screen aspect - so covering the headset's horizontal reach needs a
    // vertical value big enough that aspect stretches it far enough.
    const float aspect = StereoTest_GetBackbufferAspect();
    const float vertical = up + down;
    const float horizontal = left + right;
    const float verticalForHorizontal = 2.0f * std::atan(std::tan(horizontal * 0.5f) / aspect);

    const float margin = 1.0f + g_vrCullMarginPct.load(std::memory_order_relaxed) / 100.0f;
    const auto clampFov = [](float d) { return std::fmax(kVrCullFovMinDeg, std::fmin(kVrCullFovMaxDeg, d)); };
    // Compositor only: the cone is stuck facing the game camera while the head
    // looks wherever it likes, so widen it - but NOT to the 178 cap. At 178 the
    // game's own projection is within a couple of degrees of a flat plane, and
    // the picture came back as radial smears (user, 2026-09-16): every eye
    // basis here is decomposed from that matrix, and a frustum that wide leaves
    // the scale terms too small to recover a direction from. 150 is the angle
    // the first-person path has always fallen back to, and it renders.
    float baseDeg = clampFov(std::fmax(vertical, verticalForHorizontal) * 180.0f / kPi * margin);
    if (StereoTest_GetPictureTurnMode() == kPictureTurnModeCompositorOnly)
        baseDeg = std::fmax(baseDeg, kVrCullVerticalFovDeg);
    g_vrCullFovDegShown.store(baseDeg, std::memory_order_relaxed);

    // The same, with the angle the head covers during the lookahead added on
    // every side of the headset's view.
    const float turnSpeed = HeadTurnSpeedEnvelope();
    const float gapDeg = std::fmin(kTurnGapMaxDeg,
        turnSpeed * g_vrCullTurnLookaheadMs.load(std::memory_order_relaxed) / 1000.0f);
    float deg = baseDeg;
    if (gapDeg > 0.5f) {
        const float gap = gapDeg * kPi / 180.0f;
        const float halfH = std::fmin(horizontal * 0.5f + gap, 89.0f * kPi / 180.0f);
        const float verticalTurning = vertical + 2.0f * gap;
        const float verticalForHorizontalTurning = 2.0f * std::atan(std::tan(halfH) / aspect);
        deg = std::fmax(baseDeg,
            clampFov(std::fmax(verticalTurning, verticalForHorizontalTurning) * 180.0f / kPi * margin));
    }

    // How far turning widened it, summarised every few seconds rather than
    // logged on every change.
    static float s_windowMaxDeg = 0.0f, s_windowMaxSpeed = 0.0f;
    static ULONGLONG s_windowStartMs = 0;
    s_windowMaxDeg = std::fmax(s_windowMaxDeg, deg - baseDeg);
    s_windowMaxSpeed = std::fmax(s_windowMaxSpeed, turnSpeed);
    const ULONGLONG nowMs = GetTickCount64();
    if (!s_windowStartMs)
        s_windowStartMs = nowMs;
    if (nowMs - s_windowStartMs >= 3000) {
        if (s_windowMaxDeg >= 1.0f)
            Log_Printf("CameraRigHook: culling widened while turning - up to +%.0f deg (fastest turn %.0f deg/sec, "
                       "lookahead %.0f ms)",
                s_windowMaxDeg, s_windowMaxSpeed, g_vrCullTurnLookaheadMs.load(std::memory_order_relaxed));
        s_windowStartMs = nowMs;
        s_windowMaxDeg = 0.0f;
        s_windowMaxSpeed = 0.0f;
    }

    static float s_lastLogged = 0.0f;
    if (std::fabs(baseDeg - s_lastLogged) > 1.0f) {
        s_lastLogged = baseDeg;
        Log_Printf("CameraRigHook: VR culling FOV from the headset - %.0f deg vertical (headset %.0f v / %.0f h, "
                   "+%.0f%% margin)",
            baseDeg, vertical * 180.0f / kPi, horizontal * 180.0f / kPi, (margin - 1.0f) * 100.0f);
    }
    return deg;
}

// Eye placement. The flat-screen eye (15 ahead of the head pivot for Chris)
// felt too far forward in VR, where head tracking moves the view on top of
// it. This scales the forward offset in VR only; ',' / '.' tune it live.
// 0.2: the user settled at 0.0 on 2026-09-11 but said "probably a bit more
// forward" - and 0.0 puts the eye right above the neck pivot, inside the
// collar/neck geometry that the head collapse leaves (it belongs to the
// chest joint), a likely source of that session's black flicker.
// Two independent eye positions, as multipliers of the skeleton-derived
// offsets: one for flat first person, one for VR - where head tracking moves
// the view on top of the offset, so the eye wants to sit further back. ',' /
// '.' move the active mode's eye back / forward; Shift with them moves it
// down / up. VR forward starts at 0.2: the user settled at 0.0 on 2026-09-11
// but said "probably a bit more forward", and 0.0 puts the eye right above
// the neck pivot, inside the collar geometry the head collapse leaves.
struct EyeOffsetScale {
    float up;
    float ahead;
};
// Flat default: up 0.9, fwd -0.4. The height is where the user settled on
// 2026-09-11; the forward offset was retuned on 2026-09-12 flat-screen -
// they ran it to the -1.0 clamp, then stepped it forward six times and
// stopped, calling that the default flat view. A little back and a hair down
// from the skeleton eye, which frames the gun well. 1.0/1.0 is the
// anatomically right 6-foot view, a few presses away.
EyeOffsetScale g_flatEye = { 0.9f, -0.4f };
EyeOffsetScale g_vrEye = { 1.0f, 0.2f };
constexpr float kEyeScaleStep = 0.1f;
constexpr float kEyeScaleMax = 2.0f;   // forward
constexpr float kEyeUpMax = 3.0f;      // height: the user wants to reach Chris's full 6 feet
constexpr float kFlatFovStep = 5.0f;
constexpr float kFlatFovMin = 50.0f;
constexpr float kFlatFovMax = 120.0f;
constexpr float kEyeAheadMin = -1.0f; // forward may go behind the head pivot: the user hit the old 0.0 floor and wanted to pull back further

// ---- Head tracking drives the game camera (F9, default ON) -------------
// In VR the game culls to where ITS camera points, so anything over your
// shoulder never draws. Turning the game camera with your head fixes that.
// It started as an experiment because RE5 also AIMS where the camera points,
// which the forced laser makes very visible - but that objection is gone:
// head-follow now applies only while the gun is down (see the aim gate in
// OnRigsReady), so aiming is untouched. Tested 2026-09-12, the user asked for
// it on by default: "felt a lot better, especially with F9 on". F9 still
// toggles it. Known rough edge: the camera does not track 1:1 with a fast
// head turn - something downstream eases toward the direction we write, and
// finding it is the next job.
std::atomic<bool> g_headFollow{ true };

// Mirrors the gated head-follow decision for stereo_test - see
// CameraRigHook_HeadFollowDrivingCamera.
std::atomic<bool> g_headFollowDriving{ false };

// The head's forward direction in the game camera's own frame (right, up,
// forward), published by the render thread. Double-buffered: a torn read
// here would be a jump in the camera - the same bug that punched holes in
// the VR image before the pose was published whole.
float g_headForward[2][3] = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f } };
std::atomic<int> g_headForwardFront{ 0 };

void PublishHeadForward(const float rotationDelta[9])
{
    const int back = 1 - g_headForwardFront.load(std::memory_order_relaxed);
    g_headForward[back][0] = rotationDelta[6];
    g_headForward[back][1] = rotationDelta[7];
    g_headForward[back][2] = rotationDelta[8];
    g_headForwardFront.store(back, std::memory_order_release);
}

// Turns a rig direction (pitch only, in rig space) by the head rotation.
// The rig's own basis is forward = (0, dy, dz), up = (0, dz, -dy) and
// right = (1, 0, 0); the head's forward arrives in exactly those terms.
// The directions head-follow last wrote, in world space - see
// CameraRigHook_GetHeadFollowTargets. Double-buffered like the head forward.
HeadFollowTargets g_hfTargets[2] = {};
std::atomic<int> g_hfTargetsFront{ -1 };
std::atomic<unsigned long long> g_hfTargetsMs{ 0 };

// The last few published updates, newest at g_hfHistoryHead - see
// CameraRigHook_MatchHeadFollowTargets. Guarded by a lock: written a few dozen
// times a second, read once a frame.
constexpr int kHfHistory = 8;
HeadFollowTargets g_hfHistory[kHfHistory] = {};
int g_hfHistoryHead = -1;
int g_hfHistoryCount = 0;
SRWLOCK g_hfHistoryLock = SRWLOCK_INIT;

// ---- Aim-view test (2026-09-15) - see CameraRigHook_SetAimViewTest ------
// The view split (v0.4.2): the head turns only what is drawn and culled - see
// GetViewMatrixHook - and never the game's own camera, which aiming and
// walking follow. Found 2026-09-15: turning the game camera (or either stage
// of the main camera's copy of it, +0x170/+0x190 or +0x30/+0x50) turned the gun
// with it, and made Chris turn toward where you looked whenever you pressed aim.
// Off only for comparison (developer).
std::atomic<bool> g_aimViewTest{ true };
// Published by the rigs-ready hook while aiming: each aim rig's world
// direction as the game has it (gunDir) and as head-follow would turn it
// (viewDir). The main-camera hook maps whichever the camera is on.
struct AimViewTargets {
    float gunDir[3][3];
    float viewDir[3][3];
};
AimViewTargets g_aimView[2] = {};
std::atomic<int> g_aimViewFront{ -1 };
std::atomic<unsigned long long> g_aimViewMs{ 0 };
std::atomic<unsigned long long> g_aimViewApplied{ 0 };
std::atomic<unsigned long long> g_aimViewSkipped{ 0 };

// Rig-space direction to world through one of the controller's transforms
// (rotation only).
void RigDirToWorld(const unsigned char* controller, DWORD transformOff, float dx, float dy, float dz, float out[3])
{
    const float* m = reinterpret_cast<const float*>(controller + transformOff);
    for (int i = 0; i < 3; ++i)
        out[i] = dx * m[i] + dy * m[4 + i] + dz * m[8 + i];
}

// Picture turning mode 3 ("double, culling fixed", 2026-09-15): the game
// camera is turned by TWICE the head's yaw and pitch, so the cone it culls
// with points where v0.4.1's picture points - see g_pictureTurnMode in
// stereo_test.cpp. Pitch is held short of straight up/down, where the game's
// look-at has no defined sideways axis.
void DoubleHeadAim(const float f[3], float out[3])
{
    const float len = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (len < 1e-4f) {
        std::memcpy(out, f, 3 * sizeof(float));
        return;
    }
    const float yaw = std::atan2(f[0], f[2]);
    float s = f[1] / len;
    s = s < -1.0f ? -1.0f : (s > 1.0f ? 1.0f : s);
    const float pitch = std::asin(s);
    const float kMaxPitch = 80.0f * kPi / 180.0f;
    const float yaw2 = 2.0f * yaw;
    float pitch2 = 2.0f * pitch;
    pitch2 = pitch2 < -kMaxPitch ? -kMaxPitch : (pitch2 > kMaxPitch ? kMaxPitch : pitch2);
    out[0] = std::cos(pitch2) * std::sin(yaw2);
    out[1] = std::sin(pitch2);
    out[2] = std::cos(pitch2) * std::cos(yaw2);
}

// Turns a rig direction by the head forward f - sampled ONCE per camera
// update by the caller (see the snapshot in OnRigsReady), so all six rigs
// agree.
void ApplyHeadFollow(const float f[3], float* dx, float* dy, float* dz)
{
    const float fx = f[0], fy = f[1], fz = f[2];
    const float rigDy = *dy, rigDz = *dz;
    // Negated 2026-09-12: tested in the headset, looking left swung the game
    // camera right and vice versa. The rig's sideways axis runs opposite to
    // OpenXR's +X, which no amount of staring at the basis was going to
    // settle - the headset did.
    const float nx = -fx;
    const float ny = fy * rigDz + fz * rigDy;
    const float nz = -fy * rigDy + fz * rigDz;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-3f)
        return;
    *dx = nx / len;
    *dy = ny / len;
    *dz = nz / len;
}

void PlaceRigAtEye(unsigned char* rig, const float eye[3], float dx, float dy, float dz, float fovDeg)
{
    float* f = reinterpret_cast<float*>(rig); // f[0..2] eye, f[4..6] target, f[9] FOV (+0x24)
    f[0] = eye[0];
    f[1] = eye[1];
    f[2] = eye[2];
    f[4] = eye[0] + dx * kFirstPersonTargetDistance;
    f[5] = eye[1] + dy * kFirstPersonTargetDistance;
    f[6] = eye[2] + dz * kFirstPersonTargetDistance;
    f[9] = fovDeg;
}

// ---- Eye on the head, head collapsed (2026-09-11) ----------------------
// skeleton_probe.cpp found the character's joint array at character+0x318:
// 130 joints, 0x90 apart, each with its parent index at +0x05, local scale
// at +0x30 and world matrix at +0x50. Joint 4 is the head - the eyes, the
// top of the head and the face joints are all its children - pivoting at
// ~157 units, with the eyes ~11 above and ~15 ahead of it.
//
// That replaces two approximations at once:
//  - The eye was a fixed point, 173 up at the character's root, which the
//    skeleton showed is ~5 too high and ~20 BEHIND the real eyes - hence the
//    shoulders-from-above view, and the aim-down lean leaving it behind.
//    It now follows the head pivot every frame.
//  - The head was hidden by skipping draw calls matched on texture/vertex
//    buffer descriptors (head_hide_probe.cpp), which also took the hands
//    and missed a black inner mesh. Scaling joint 4 to zero collapses
//    everything weighted to the head instead, and leaves the arms alone -
//    and joint access is what the planned arm IK needs anyway.
//
// All of this runs here, inside the camera update, where the controller -
// and so the character it follows - is provably alive; every read goes
// through SEH, and nothing is written until the joint layout checks out.
// Only YOUR character's head is collapsed: the one nearest last frame's
// rendered camera, so Sheva walking into you keeps hers.
constexpr DWORD kOffControllerCharacter = 0x140;
constexpr DWORD kOffNormalTransform = 0x230; // rig space -> world, normal mode
constexpr DWORD kOffAimTransform = 0x1E0;    // rig space -> world, aim mode
// The body's facing: a quaternion at +0x40, with two matrices built from it
// every frame at +0x60 and +0xA0. Declared here because the arm search needs
// the character's right axis; see TurnBodyBy for how it was found.
constexpr DWORD kOffBodyTransform[2] = { 0x60, 0xA0 };
constexpr DWORD kOffBodyQuaternion = 0x40;
constexpr DWORD kOffCharacterJoints = 0x318;
constexpr int kJointStride = 0x90;
constexpr int kMaxJoints = 200;
constexpr int kOffJointLinks = 0x04;       // byte 1 = parent index, byte 3 = joint id
constexpr int kOffJointBindOffset = 0x10;  // rest-pose offset from the parent, parent space
constexpr int kOffJointScale = 0x30;
constexpr int kOffJointWorldPos = 0x80;    // row 3 of the world matrix at +0x50
constexpr int kOffJointWorldMatrix = 0x50; // row 0 of the world matrix (rows are 0x10 apart)

// Joints are found by ID, never by position in the array: Sheva's array is
// ordered differently from Chris's (her head is index 23, his index 4, and
// her index 4 is a foot), but the IDs are shared - both use the same
// animations. Checked on both skeletons, 2026-09-11.
constexpr unsigned char kHeadJointId = 4;    // parent must be kChestJointId
constexpr unsigned char kChestJointId = 3;

// Eye offset from the head pivot. Measured and approved on Chris
// (2026-09-11): 11 up, 15 ahead. Not every high ID is shared - Chris's eye
// joints (118/120) don't exist on Sheva, whose ID 120 is a chest joint -
// but 22 face joints are, with the same layout: IDs 56-60 and 180-196, all
// children of the head. Their mean rest-pose offset is the face's size
// (Chris 12.00, Sheva 9.54), so each character's eye offset is Chris's
// scaled by it. That puts Sheva's eye at 155.5 against Chris's 168
// (x0.926, matching her body: head pivot 146.7 vs 157.0, x0.934).
constexpr float kEyeAbovePivot = 11.0f;
constexpr float kEyeAheadOfPivot = 15.0f;
constexpr float kChrisFaceSize = 12.0f;
constexpr int kMinFaceJoints = 10;

bool IsSharedFaceJoint(unsigned char id)
{
    return (id >= 56 && id <= 60) || (id >= 180 && id <= 196);
}
constexpr float kPlayerHeadMaxDistance = 50.0f;
constexpr unsigned long long kHeadTrackExpiryMs = 250;

// ---- Which skeleton is this? (2026-09-12) -------------------------------
// Proximity alone decides who the player is, and in co-op it gets it wrong:
// Sheva keeps losing her head (user, recurring). Tightening the distances
// helps but cannot fix the shape of the rule - it re-decides every frame from
// a noisy measurement, so it only ever takes one bad moment.
//
// The user's framing is the right one: work out WHOSE skeleton this is, and
// if it is not the player's, never touch its head. Chris and Sheva are
// trivially distinguishable - their joint arrays are ordered differently (his
// head is index 4, hers 23) and their faces are different sizes (12.00
// against 9.54, already measured for the eye offset). So the player's
// skeleton gets identified once, latched, and every head after that is
// matched by identity rather than by who happens to be near the camera.
//
// This works whichever character is being played: nothing here knows or cares
// which one is Chris. It latches whoever the camera is genuinely inside.
struct SkeletonId
{
    bool valid = false;
    int jointCount = 0;
    int headIndex = 0;
    int faceSize10 = 0; // mean face-joint offset x10, 0 when too few were found
};

bool SkeletonMatches(const SkeletonId& a, const SkeletonId& b)
{
    if (!a.valid || !b.valid)
        return false;
    if (a.jointCount != b.jointCount || a.headIndex != b.headIndex)
        return false;
    // Face size is a float average, so allow a little slack; 0 means it could
    // not be measured on one side, in which case the first two already agree.
    if (!a.faceSize10 || !b.faceSize10)
        return true;
    const int d = a.faceSize10 - b.faceSize10;
    return (d < 0 ? -d : d) <= 3;
}


struct HeadTrack {
    unsigned char* controller;
    unsigned char* joints; // the followed character's joint array; a change means a different character
    unsigned char* head;
    float eyeUp, eyeAhead; // eye offset from the head pivot, character space
    float distance;        // head pivot to last frame's camera
    unsigned long long ms;
    bool collapsed;
    bool isPlayer;         // this frame: the character nearest the rendered camera
    bool direct;           // active controller embedded in a main camera (a candidate)
    SkeletonId skel;       // who this character is, for the player latch
    bool headShown;        // the game has the camera, so the head is drawn even in first person
    unsigned long long headNearSinceMs; // since when the camera has been back near the eye (0 = it is not)
    float prevRoot[3];     // where the character stood last frame, for the running lead
    bool haveRoot;
    float lastLead;        // how far the lead moved the eye, for the log
    float prevDistance;    // last measured camera-to-eye distance, for the one-frame jump
    bool havePrevDistance;
    float histDistance;    // distance as of histMs, for "is it closing?"
    unsigned long long histMs;
    float camWorld[3], eyeWorld[3], headPivotWorld[3]; // trace only - the endpoints behind track->distance
};
HeadTrack g_heads[8] = {};
std::atomic<unsigned long> g_headCollapseFrames{0};
std::atomic<unsigned long> g_headScaleResets{0};
// How often the game took the camera away and got the head back, and the
// furthest the camera got from the eye while the head was still fully
// collapsed - i.e. what first person really costs, which is the number the
// bottom of the ramp band has to clear.
std::atomic<unsigned long> g_headShownEvents{0};
std::atomic<long> g_headNearMaxDistance{0};
std::atomic<long> g_headNearMaxJump{0};
std::atomic<int> g_eyeLogsRemaining{0};

// Sprint is Shift on the keyboard. The user's idea: tag head events with it,
// so a false pop while sprinting is identifiable in the log instead of being
// inferred from the timing. Says nothing about a controller sprint.
bool ShiftHeld()
{
    return (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
}

// A window opened by pressing F (see CameraRigHook_OnEndScene), during which
// every frame's camera-to-eye distance is logged. ~2 s covers a punch, a
// vault or a grab from start to finish.
constexpr unsigned long long kActionTraceMs = 2000;
unsigned long long g_actionTraceUntilMs = 0;

// TWO TESTS, TWO KEYS (2026-09-27, user: "so hold page down for 2 seconds,
// then stomp in flatscreen?").
//
// They were on one key, which would have spoiled both: the window that
// RECORDS what the camera does during a stomp was also FORCING the view due
// north, so the recording would have been of the forcing.
//
//   Page Down - record only. Press it, then stomp. Nothing changes in the
//               game; one line per frame goes into the log.
//   End       - force the view due north for two seconds, and change nothing
//               else. Whether the picture moves is the whole answer.
unsigned long long g_northTestUntilMs = 0;

// Set while the turn limiter is holding, so the view can be levelled for as
// long as it lasts plus a moment after. See "LEVEL WHILE HELD".

// WHICH PATH SUPPLIED THE VIEW (2026-09-27, after End proved the view is ours).
//
// So the whip does come through something we control - and yet the limiter
// refused a 102 degree turn while it still arrived. There is one route that
// explains both: GetViewMatrix tries BuildAimRenderView FIRST and returns it
// when it succeeds, so on any frame that takes the aim path, the head lock and
// its limiter never run at all. No refusal is logged either, because the
// function was never called, which is exactly the silence we have been
// reading as "nothing happened".
//
// Counted per frame and printed with the Page Down trace.
unsigned long g_viewFromAim = 0;
unsigned long g_viewFromHeadLock = 0;
unsigned long g_viewFromGame = 0;
// The head is hidden by one write of 0 to the joint scale, so it should
// vanish in a single frame - but the user sees it "shrink down like someone
// drained the air out of his head". Either the game interpolates the value we
// write, or something writes it back. Every hide therefore opens a short
// trace of its own: the scale read-back and the world-matrix scale, frame by
// frame, for as long as the shrink appears to take.
constexpr unsigned long long kHideTraceMs = 400;

// All of the head tracing is developer instrumentation: the per-frame lines
// below run for 400 ms after every hide, and EndScene fires ~20 times a
// frame, so one hide is several hundred lines in re5vr.log. Useful here,
// unacceptable in a build a tester runs - so it compiles out with the rest of
// the diagnostics. The head on/off and watchdog lines are NOT gated: they are
// one line per event and they are what makes a bug report readable.
void HideTrace_Open(unsigned long long now)
{
#if RE5VR_DIAGNOSTICS
    g_actionTraceUntilMs = now + kHideTraceMs;
#else
    (void)now;
#endif
}

// ---- The camera hook stops running during scripted actions ---------------
// Measured 2026-09-12, and it invalidates every threshold above as an
// explanation for "the head never comes back during a vault". Tracing an edge
// jump frame by frame left a 1370 ms HOLE: no trace lines at all through the
// fall, then one at 88.2 the moment Chris stood up. The trace only runs while
// our controller is the player, so for the whole action our camera hook was
// not deciding anything for it - the head scale simply stayed at the 0.00 it
// had from first person, because nothing was there to write 1.0.
//
// That is also why the head "appears as he stands": that is the first frame
// the hook runs again, not the detector catching a cut.
//
// The fix has to live somewhere that keeps running when the game's camera
// code does not. CameraRigHook_OnEndScene is called from the D3D9 EndScene
// hook every single frame, so it can watch for the camera hook going quiet
// and give the head back on its own. Writing to a joint we have not seen for
// a while is the risk, so the pointer is re-validated (class word, joint id,
// parent id) before any write - the same checks FindHeadJoint uses.
constexpr unsigned long long kHookQuietMs = 100;  // no camera-hook update for this long -> watchdog takes over
constexpr float kWatchdogHideDistance = 35.0f;   // camera this close to the head pivot -> hide it again
unsigned char* g_lastPlayerHead = nullptr;       // head joint of whoever was last the player
unsigned char* g_lastPlayerJoints = nullptr;     // and the array it came from, for validation
unsigned long long g_lastPlayerHeadMs = 0;
bool g_watchdogRestored = false;                 // the watchdog, not UpdateHead, owns the head now
std::atomic<unsigned long> g_watchdogRestores{0};

bool TryRead(void* dst, const void* src, size_t n)
{
    __try {
        std::memcpy(dst, src, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// The other direction, for the few places the mod sets one of the game's own
// values rather than reading it. In-process only, and guarded the same way:
// a character can be freed between the read that found it and the write.
bool TryWrite(void* dst, const void* src, size_t n)
{
    __try {
        std::memcpy(dst, src, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

float Length3(const float* v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// ---- The main camera (2026-09-11) --------------------------------------
// Found by the F5 camera-output watch: the render camera the game builds its
// view from is a "main camera" object that calls its ACTIVE controller's
// update, then copies eye/target/up/FOV from it into itself at the same
// offsets (re5dx9.exe+44211B..+4421BC). It points at the active controller
// from +0x1B0 - and the player's controller is EMBEDDED in it at +0xE00
// (main 0A006B00, player controller 0A007900 in that run; other embedded
// controllers at +0x1480 and +0x2DE0). So a controller is the one being
// rendered from exactly when [controller - 0xE00 + 0x1B0] points back at
// it. For any other controller (Sheva's is a separate heap object) that
// address is unrelated memory and practically never matches.
constexpr DWORD kOffMainCameraPlayerController = 0xE00;
constexpr DWORD kOffMainCameraActiveController = 0x1B0;

bool IsActiveMainCameraController(unsigned char* controller)
{
    unsigned char* active = nullptr;
    return TryRead(&active, controller - kOffMainCameraPlayerController + kOffMainCameraActiveController,
               sizeof(active)) &&
        active == controller;
}

// The followed character's joint array, or null.
unsigned char* JointArray(unsigned char* controller)
{
    unsigned char* character = nullptr;
    if (!TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) || !character)
        return nullptr;
    unsigned char* joints = nullptr;
    if (!TryRead(&joints, character + kOffCharacterJoints, sizeof(joints)) || !joints)
        return nullptr;
    return joints;
}

// The head joint (ID 4, child of ID 3), and the eye offset from it - or null
// if this isn't the skeleton layout found on 2026-09-11 (e.g. a
// non-character target). The array ends where the joint class changes.
// outId, when given, receives this skeleton's identity - see SkeletonId.
unsigned char* FindHeadJoint(unsigned char* joints, float* eyeUp, float* eyeAhead, SkeletonId* outId = nullptr)
{
    DWORD rootClass = 0;
    if (!TryRead(&rootClass, joints, sizeof(rootClass)))
        return nullptr;
    unsigned char ids[kMaxJoints], parents[kMaxJoints];
    int count = 0;
    for (; count < kMaxJoints; ++count) {
        const unsigned char* j = joints + count * kJointStride;
        DWORD cls = 0;
        unsigned char links[4] = {};
        if (!TryRead(&cls, j, sizeof(cls)) || cls != rootClass || !TryRead(links, j + kOffJointLinks, sizeof(links)))
            break;
        parents[count] = links[1];
        ids[count] = links[3];
    }
    int head = -1;
    for (int i = 0; i < count && head < 0; ++i) {
        if (ids[i] == kHeadJointId && parents[i] < count && ids[parents[i]] == kChestJointId)
            head = i;
    }
    if (head < 0)
        return nullptr;

    // Face size -> eye offset (see kChrisFaceSize). Falls back to Chris's
    // numbers if too few shared face joints are found.
    *eyeUp = kEyeAbovePivot;
    *eyeAhead = kEyeAheadOfPivot;
    float total = 0.0f;
    int faceJoints = 0;
    for (int i = 0; i < count; ++i) {
        if (parents[i] != head || !IsSharedFaceJoint(ids[i]))
            continue;
        float off[3];
        if (TryRead(off, joints + i * kJointStride + kOffJointBindOffset, sizeof(off))) {
            total += Length3(off);
            ++faceJoints;
        }
    }
    if (faceJoints >= kMinFaceJoints) {
        const float ratio = (total / faceJoints) / kChrisFaceSize;
        if (ratio > 0.5f && ratio < 1.5f) {
            *eyeUp = kEyeAbovePivot * ratio;
            *eyeAhead = kEyeAheadOfPivot * ratio;
        }
    }
    if (outId) {
        outId->valid = true;
        outId->jointCount = count;
        outId->headIndex = head;
        outId->faceSize10 = faceJoints >= kMinFaceJoints
            ? static_cast<int>((total / faceJoints) * 10.0f + 0.5f)
            : 0;
    }
    return joints + head * kJointStride;
}

// ---- Finding the arms (2026-09-17) --------------------------------------
// Arm IK needs three joints a side: the shoulder it pivots from, the elbow it
// bends at, and the wrist the hand hangs off. Nothing in the data is named, so
// they are found by shape instead, which also means this works on Sheva, on the
// Majini, and on whatever else has a skeleton, rather than on a table of
// indices that only fits Chris.
//
// The chest is already known (it is how the head is found). Its children that
// are not the head are the two collarbones; from each, following the longest
// chain of single-child joints walks down the arm. Left and right are told
// apart by which side of the chest the shoulder sits on, measured along the
// character's own right axis rather than the world's, so it holds however the
// character is turned.
struct ArmChain {
    int shoulder = -1, elbow = -1, wrist = -1;
    bool valid = false;
};
struct ArmChains {
    ArmChain left, right;
    int count = 0, head = -1, chest = -1;
    bool valid = false;
};

bool JointWorldPos(const unsigned char* joints, int index, float out[3])
{
    return TryRead(out, joints + index * kJointStride + kOffJointWorldPos, sizeof(float) * 3);
}

// The deepest chain from start, following whichever child leads furthest.
int WalkArm(const unsigned char* parents, int count, int start, int out[3])
{
    int chain[8];
    int depth = 0;
    int at = start;
    while (depth < 8) {
        chain[depth++] = at;
        int best = -1, bestDepth = -1;
        for (int i = 0; i < count; ++i) {
            if (parents[i] != at)
                continue;
            // How far this child's own line runs, so a finger stub does not
            // win over the forearm.
            int run = 0, cursor = i;
            while (run < 8) {
                int next = -1;
                for (int k = 0; k < count && next < 0; ++k) {
                    if (parents[k] == cursor)
                        next = k;
                }
                if (next < 0)
                    break;
                cursor = next;
                ++run;
            }
            if (run > bestDepth) {
                bestDepth = run;
                best = i;
            }
        }
        if (best < 0)
            break;
        at = best;
    }
    // Shoulder, elbow, wrist: the collarbone is chain[0], so the arm proper
    // starts one down.
    if (depth < 4)
        return 0;
    out[0] = chain[1];
    out[1] = chain[2];
    out[2] = chain[3];
    return 3;
}

// Which skeleton the bone table has already been printed for. Cleared
// whenever the arms are forgotten, so ticking the setting prints it again.
unsigned char* g_boneTableTold = nullptr;

ArmChains FindArms(unsigned char* joints, unsigned char* character)
{
    ArmChains arms;
    DWORD rootClass = 0;
    if (!TryRead(&rootClass, joints, sizeof(rootClass)))
        return arms;
    unsigned char ids[kMaxJoints], parents[kMaxJoints];
    int count = 0;
    for (; count < kMaxJoints; ++count) {
        const unsigned char* j = joints + count * kJointStride;
        DWORD cls = 0;
        unsigned char links[4] = {};
        if (!TryRead(&cls, j, sizeof(cls)) || cls != rootClass || !TryRead(links, j + kOffJointLinks, sizeof(links)))
            break;
        parents[count] = links[1];
        ids[count] = links[3];
    }
    int chest = -1, head = -1;
    for (int i = 0; i < count; ++i) {
        if (ids[i] == kHeadJointId && parents[i] < count && ids[parents[i]] == kChestJointId) {
            head = i;
            chest = parents[i];
        }
    }
    if (chest < 0)
        return arms;
    arms.count = count;
    arms.head = head;
    arms.chest = chest;

    // Hands, by counting children (2026-09-17). The first version assumed the
    // arms hang off the chest, and the log was blunt about it: the chest has
    // exactly ONE child, the head. So the arms are parented somewhere else
    // entirely and walking down from the chest was never going to find them.
    //
    // A hand does not need to be found by where it hangs, though. It is the
    // one joint in a skeleton with a fistful of children, because every finger
    // starts there: three or more children is a hand and almost nothing else.
    // From a hand, the elbow is its parent and the shoulder its grandparent,
    // no walking required.
    int childCount[kMaxJoints] = {};
    for (int i = 0; i < count; ++i) {
        if (parents[i] < count)
            ++childCount[parents[i]];
    }
    // Children alone are not enough (2026-09-17): it picked joints 2 and 9,
    // which are spine, because a spine branches too - into two arms and a neck.
    // What separates them is how FAR each branch runs. A finger is three joints
    // and stops; an arm off the spine runs eight or more. So a hand is a joint
    // whose children all peter out quickly.
    int chainDepth[kMaxJoints] = {};
    for (int pass = 0; pass < 12; ++pass) {
        for (int i = 0; i < count; ++i) {
            if (parents[i] >= count || parents[i] == i)
                continue;
            const int want = chainDepth[i] + 1;
            if (want > chainDepth[parents[i]])
                chainDepth[parents[i]] = want;
        }
    }

    // The character's own right, from the body transform the absolute yaw work
    // found: row 0 is [sin, 0, cos] of the facing, so right is across it.
    float row0[4] = {};
    if (!TryRead(row0, character + kOffBodyTransform[0], sizeof(row0)))
        return arms;
    // Row 0 lies ACROSS the body, not along it (2026-09-17). The body's facing
    // measured as the view's bearing plus ninety, which is the same statement.
    // Using it as forward made "which side" mean "how far in front", so both
    // hands came out on the same side and the second was discarded: "joint 54
    // ... calling it a hand" and then nothing.
    //
    // And it points LEFT. Which way across only started to matter once the
    // hands had sides to get wrong: taken as right, this called the left hand
    // the right one, and arm IK then drove it from the right controller and
    // towards the left - the user saw both halves of that at once. See the
    // same correction in arm_ik.cpp.
    const float rightX = -row0[0], rightZ = -row0[2];

    float chestPos[3];
    if (!JointWorldPos(joints, chest, chestPos))
        return arms;

    // What the chest actually has hanging off it, when the search comes up
    // empty. Guessing at a skeleton from the outside goes wrong quietly
    // otherwise: too few children, chains too short, everything on one side.
    if (XrInput_GetSettings().findArms) {
        if (g_boneTableTold != joints) {
            g_boneTableTold = joints;
            Log_Printf("Arms: %d joint(s), chest is %d, head is %d", count, chest, head);
            // The whole table, once per skeleton (2026-09-24, user: "we're
            // pretty confident what skeleton is chris, and what is sheva. so
            // we should be easily tell what based off of that what bones to
            // assign"). Quite right, and the engine already hands us the
            // means: every joint carries an id, and the head and the chest are
            // found by theirs rather than by shape. Everything else in this
            // search guesses from shape instead, which is correct on Chris and
            // picks a finger on Sheva.
            //
            // Naming the arm bones outright needs the table for both
            // characters, and nothing has ever printed it. One run as each is
            // all it takes, and then this stops being a guess forever.
            for (int i = 0; i < count; ++i) {
                float p[3] = {};
                if (!JointWorldPos(joints, i, p))
                    continue;
                const float up = p[1] - chestPos[1];
                const float side = (p[0] - chestPos[0]) * rightX + (p[2] - chestPos[2]) * rightZ;
                float fromParent = 0.0f;
                if (parents[i] < count && parents[i] != i) {
                    float q[3] = {};
                    if (JointWorldPos(joints, parents[i], q)) {
                        const float d3[3] = { p[0] - q[0], p[1] - q[1], p[2] - q[2] };
                        fromParent = Length3(d3);
                    }
                }
                Log_Printf("BoneTable: %3d  id %3u  parent %3d  %d child(ren)  %.1f long  %6.1f %s of the "
                           "chest, %6.1f up",
                    i, static_cast<unsigned>(ids[i]), static_cast<int>(parents[i]), childCount[i], fromParent,
                    side < 0.0f ? -side : side, side < 0.0f ? "left " : "right", up);
            }
            // Every joint with enough children to be a hand, whether or not it
            // passes the rest of the tests.
            for (int i = 0; i < count; ++i) {
                if (childCount[i] < 3)
                    continue;
                float p[3] = {};
                JointWorldPos(joints, i, p);
                const float side = (p[0] - chestPos[0]) * rightX + (p[2] - chestPos[2]) * rightZ;
                Log_Printf("Arms:   joint %d (id %u, parent %u) has %d children running %d deep, %.1f to the %s and "
                           "%.1f above the chest",
                    i, static_cast<unsigned>(ids[i]), static_cast<unsigned>(parents[i]), childCount[i],
                    chainDepth[i], side < 0.0f ? -side : side, side >= 0.0f ? "right" : "left",
                    p[1] - chestPos[1]);
            }
        }
    }

    ArmChain candidates[4];
    float candidateSide[4] = {};
    int found = 0;
    for (int i = 0; i < count; ++i) {
        // Four or more branches, none of them running further than a finger
        // does, and not on the spine we already know about.
        if (childCount[i] < 4 || chainDepth[i] > 4 || i == chest || i == head)
            continue;
        // Past the twist bones (2026-09-17). The first pick reported an upper
        // arm of 26.8 units and a forearm of 0.0: the hand's parent sits at the
        // same point as the hand, which is a roll joint, not an elbow. Riggers
        // put one or two along a forearm to spread the wrist's twist. So walk
        // up until the bone has real length.
        const auto realParent = [&](int of) {
            int up = parents[of];
            float here[3], there[3];
            if (!JointWorldPos(joints, of, here))
                return up;
            for (int guard = 0; guard < 6 && up < count && up != of; ++guard) {
                if (!JointWorldPos(joints, up, there))
                    break;
                const float d[3] = { here[0] - there[0], here[1] - there[1], here[2] - there[2] };
                if (Length3(d) > 1.0f)
                    break;
                up = parents[up];
            }
            return up;
        };
        const int elbow = realParent(i);
        if (elbow >= count)
            continue;
        const int shoulder = realParent(elbow);
        if (shoulder >= count || shoulder == elbow)
            continue;
        float handPos[3];
        if (!JointWorldPos(joints, i, handPos))
            continue;
        // An arm is a certain size, and a finger is not (2026-09-24). Every
        // test above this one is about SHAPE - how many children a joint has,
        // how far its branches run, which side of the chest it is on - and a
        // finger passes all of them, because a finger is a little joint with
        // little branches off to one side. Driving somebody else's character,
        // this search settled on joints 140, 141 and 142 with a total length
        // of 9.1 units, and because a finger really is rigid, everything
        // downstream accepted it and drove their whole arm from it.
        //
        // Nothing here knew how big an arm ought to be. The chest to the head
        // is a neck, and it is the one measurement always to hand: on this rig
        // an arm runs several necks long and each of its two bones is longer
        // than one. A finger is a fraction of it and is turned away.
        {
            float elbowPos[3], shoulderPos[3], headPos[3];
            if (!JointWorldPos(joints, elbow, elbowPos) || !JointWorldPos(joints, shoulder, shoulderPos))
                continue;
            const float uv[3] = { elbowPos[0] - shoulderPos[0], elbowPos[1] - shoulderPos[1],
                elbowPos[2] - shoulderPos[2] };
            const float fv[3] = { handPos[0] - elbowPos[0], handPos[1] - elbowPos[1],
                handPos[2] - elbowPos[2] };
            const float upperLen = Length3(uv), foreLen = Length3(fv);
            float neck = 0.0f;
            if (head >= 0 && JointWorldPos(joints, head, headPos)) {
                const float nv[3] = { headPos[0] - chestPos[0], headPos[1] - chestPos[1],
                    headPos[2] - chestPos[2] };
                neck = Length3(nv);
            }
            if (neck < 1.0f)
                neck = 10.0f; // no head to measure against: fall back on something sane
            // Loose on purpose (2026-09-24). This same search finds YOUR arms
            // as well as a partner's, on Chris and on Sheva, so a threshold
            // set too high does not merely fail to find a partner - it takes
            // your own arms away in single player. Chris measures 53.2 units
            // across two bones and the neck on that rig has never been
            // measured, so the first pass at this asked for two and a half
            // necks and would have thrown him away outright if that neck is
            // twenty units rather than twelve.
            //
            // Half a neck a bone and a neck and a half in total is nowhere
            // near a real arm and miles above a finger, which measured 6.2 and
            // 2.8. A filter only has to separate the two things in front of
            // it, and being generous costs nothing when the gap is that wide.
            const bool tooSmall
                = upperLen < neck * 0.5f || foreLen < neck * 0.5f || upperLen + foreLen < neck * 1.5f;
            const float ratio2 = (upperLen > foreLen ? upperLen : foreLen)
                / ((upperLen < foreLen ? upperLen : foreLen) + 0.001f);
            if (XrInput_GetSettings().findArms) {
                Log_Printf("Arms:   joint %d as a hand - upper %.1f, forearm %.1f, neck %.1f, one bone against "
                           "the other %.1f -> %s",
                    i, upperLen, foreLen, neck, ratio2,
                    tooSmall ? "too small for an arm" : (ratio2 > 3.0f ? "too lopsided for an arm" : "kept"));
            }
            if (tooSmall || ratio2 > 3.0f)
                continue;
        }
        // Arms HANG (2026-09-17). The pair that is obviously right in the log -
        // four children each, four deep each, 15 and 17 units out on opposite
        // sides - sits 59 units BELOW the chest, and a filter that wanted hands
        // above chest height minus forty threw both of them away. Feet are
        // further down again, so the line just needs to be drawn lower.
        if (handPos[1] < chestPos[1] - 120.0f)
            continue;
        // And not on the face: the head carries a pile of little joints, one of
        // which had five children and was duly called a hand.
        bool underHead = false;
        for (int up = parents[i], guard = 0; up < count && guard < 16; up = parents[up], ++guard) {
            if (up == head) {
                underHead = true;
                break;
            }
            if (parents[up] == up)
                break;
        }
        if (underHead)
            continue;
        // A hand is out to the side, not on the centre line.
        // Which hand is which is decided between them, not against the chest
        // (2026-09-17). Both hands came out "to the left" in one run and the
        // second was thrown away as a duplicate - because the character was
        // holding a gun, with both hands together off to one side. Relative to
        // each other they are still unambiguous: whichever is further along the
        // body's right IS the right hand, wherever the pair happens to be.
        // Measured at the SHOULDER, not the hand (2026-09-17). Hands end up in
        // the same place as each other whenever a weapon is held two-handed, so
        // which of them is further right came down to noise, and the answer
        // changed between runs - one launch called joint 54 the right wrist and
        // the next called it 79, its opposite number. Shoulders are never
        // ambiguous: they are a body's width apart whatever the arms are doing.
        float shoulderPos[3];
        if (!JointWorldPos(joints, shoulder, shoulderPos))
            continue;
        const float side
            = (shoulderPos[0] - chestPos[0]) * rightX + (shoulderPos[2] - chestPos[2]) * rightZ;
        if (found < 4) {
            candidates[found].shoulder = shoulder;
            candidates[found].elbow = elbow;
            candidates[found].wrist = i;
            candidates[found].valid = true;
            candidateSide[found] = side;
            ++found;
        }
        if (XrInput_GetSettings().findArms) {
            Log_Printf("Arms: joint %d has %d children, on the arm whose shoulder sits %.1f to the %s "
                       "of the chest - a hand",
                i, childCount[i], side < 0.0f ? -side : side, side >= 0.0f ? "right" : "left");
        }
    }
    if (found >= 2) {
        int rightMost = 0, leftMost = 0;
        for (int i = 1; i < found; ++i) {
            if (candidateSide[i] > candidateSide[rightMost])
                rightMost = i;
            if (candidateSide[i] < candidateSide[leftMost])
                leftMost = i;
        }
        if (rightMost != leftMost) {
            arms.right = candidates[rightMost];
            arms.left = candidates[leftMost];
        }
    }
    arms.valid = arms.left.valid && arms.right.valid;
    return arms;
}

// World position -> rig space through one of the controller's transforms
// (row-major, orthonormal basis rows, translation in row 3).
void WorldToRig(const unsigned char* controller, DWORD transformOff, const float world[3], float out[3])
{
    const float* m = reinterpret_cast<const float*>(controller + transformOff);
    const float d[3] = { world[0] - m[12], world[1] - m[13], world[2] - m[14] };
    for (int k = 0; k < 3; ++k)
        out[k] = d[0] * m[k * 4] + d[1] * m[k * 4 + 1] + d[2] * m[k * 4 + 2];
}

// The inverse: rig space -> world.
void RigToWorld(const unsigned char* controller, DWORD transformOff, const float rig[3], float out[3])
{
    const float* m = reinterpret_cast<const float*>(controller + transformOff);
    for (int i = 0; i < 3; ++i)
        out[i] = m[12 + i] + rig[0] * m[i] + rig[1] * m[4 + i] + rig[2] * m[8 + i];
}

// Which camera controller is the player's - see "Which character is YOU" in
// UpdateHead. Held once chosen: it only moves to another controller when the
// current one expires, or its eye measures clearly away from the rendered
// camera (kPlayerKeepDistance) for kPlayerSwitchDelayMs - what a real change
// of character looks like, and what Sheva standing close never does.
const unsigned char* g_playerController = nullptr;
// Both numbers were guesses made before anything was measured, and one of
// them is why Sheva keeps losing her head in co-op (user, recurring). The
// steal is gated on the CURRENT player looking "clearly away" from the camera
// for a second - but sprinting puts the player's own eye 85-111 from the
// rendered camera for half a second to nearly two (measured 2026-09-12), so
// a sprint past 75 starts that clock, and if Sheva happens to be near the
// camera when it expires she takes the pick and her head collapses instead.
// So "clearly away" now means past 130, above anything sprinting produces,
// and a challenger has to be within 15 rather than 25 - in first person the
// real player measures 0-14, so 15 is all a genuine character change needs.
constexpr float kPlayerEyeMaxDistance = 15.0f;
constexpr float kPlayerKeepDistance = 130.0f;
constexpr unsigned long long kPlayerSwitchDelayMs = 1000;
// The player's identity, once established: every head decision after that is
// made by matching this, not by measuring distances. Latched only from a
// camera that is genuinely INSIDE a character's head (kPlayerLatchDistance,
// far tighter than the pick's own threshold), because a latch on the wrong
// character would be sticky in exactly the way this is designed to be.
// Cleared if nothing matches it for a while - a chapter change, or the game
// handing the player a different character.
SkeletonId g_playerSkeleton;
constexpr float kPlayerLatchDistance = 12.0f;
constexpr unsigned long long kPlayerSkeletonGoneMs = 5000;
unsigned long long g_playerSkeletonSeenMs = 0;

// ---- Give the head back when the game takes the camera (2026-09-12) -----
// Melee attacks, window vaults and cutscenes pull the view out to a scripted
// camera. First person can't follow those (the animation swings the skull,
// which is nauseating in a headset), so the view leaves the body - and what
// it finds there is a headless character, because the head joint is still
// collapsed.
//
// The trigger is deliberately NOT a list of actions. Enumerating them means
// missing the ones nobody thought of, and every miss ships a decapitated
// Chris. The rendered camera's own position answers it directly: it is
// decoded from the vertex constants every frame (LastCameraPosition), so it
// is where the game actually drew from, whatever code put it there. In first
// person the eye IS the camera - the measured distance is ~0 - so anything
// beyond arm's reach means the camera is no longer ours.
//
// Measured over two sessions on 2026-09-12, and the numbers are decisive:
//
//   normal first person   <= 14 from the eye
//   sprinting             one- and two-frame spikes to 15.6-19.1 (10-30 ms)
//   a real action camera  88, 124, 134, 155, 156, 194, 238
//
// The spikes are an artefact, not the camera moving: the measurement is LAST
// frame's rendered camera against THIS frame's eye, so running fast opens a
// gap that was never there. They are also brief, where an action camera
// stays out for hundreds of milliseconds. Either way there is a clean empty
// gap between 19 and 88 to put the threshold in.
//
// It pops, it does not fade in. A ramp was tried first, on the theory that a
// smooth pan would hide a smooth grow - but the head is hidden by scaling the
// joint, so a partial value is a SHRUNKEN head, and watching Chris's head
// swell as he throws a punch is worse than any pop ("comical, but not ideal",
// user). A correctly-sized head appearing while the camera is already moving
// is close to invisible; a wrong-sized one is not.
//
// Asymmetric, because the action camera does not go far and stay there - it
// pans out past 88, then settles about a third of a metre off the eye (33-40
// measured) and looks at Chris for the rest of the animation. Hiding again at
// anything near that distance takes the head away mid-punch, which is the
// first version's bug. So: show high, and only hide again below 25 once it
// has stayed there long enough to be real.
//
// DISTANCE ALONE CANNOT DO THIS (measured 2026-09-12, fourth run). Sprinting
// puts the rendered camera 70.5, 104.9 and 111.4 from the eye, and it STAYS
// there for half a second to nearly two. A door kick puts it at 124.5. Those
// ranges touch, so every level-based threshold tried here has either popped
// the head while running or missed the kick - both, at 40.
//
// The SHAPE separates them completely. Traced frame by frame, a door kick
// reads 1.8, 1.2, 124.5: a hundred and twenty units in one frame, which is a
// camera being cut to somewhere else, not a camera moving. Sprint climbs
// there gradually, because that is the game's own camera trailing a running
// character. Nothing that trails can teleport.
//
// So the trigger is the JUMP, with a level floor to keep ordinary jitter out.
// Getting hit by a zombie - traced start to finish - never leaves 0.3-7.8, so
// it stays first person, which is what it should do.
// How far the run lead has pushed the eye ahead of the body this frame. Our
// own displacement, which the head-show test below must not mistake for the
// game snatching the camera away.
std::atomic<float> g_leadUnits{ 0.0f };
constexpr float kHeadShowDistance = 60.0f;             // never on a camera nearer than this...
constexpr float kHeadShowJump = 50.0f;                 // ...and only when it got there in one frame
// Menu option "Show head during action cameras" (default on). Off keeps the
// head hidden no matter where the game puts the camera - no pop, no deflate.
std::atomic<bool> g_showHeadDuringActions{ true };
// Read from the game thread inside SetLookAtHook, so an atomic rather than a
// trip through the settings struct.
std::atomic<bool> g_holdViewInMelee{ true };
std::atomic<bool> g_partnerNoCamera{ true };
std::atomic<bool> g_scopeThirdPerson{ false };
constexpr float kHeadHideDistance = 25.0f;             // and back inside this...
constexpr unsigned long long kHeadHideDwellMs = 100;   // ...for this long -> hide it again

// That pair hides FAR too late on the way back in. Measured 2026-09-12: every
// hide in a session landed at an eye distance of 2.7 to 9.4, and the user's
// screenshot at the moment it should have gone shows the camera already
// through the back of the skull, looking at hair - "it hid after I make it
// into the head and see teeth". The camera covers that last stretch in a few
// frames, so a 100 ms dwell spends the whole of it inside his head.
//
// Hiding early is only dangerous for a camera that PARKS near the head, which
// is what an action camera does (33-40, measured) - hide on distance alone at
// that range and the head vanishes mid-punch again. A camera on its way back
// to the eye is different in a way that has nothing to do with where it is:
// it is closing, fast. So the early hide asks for both - inside 45, and at
// least 10 units closer than it was 150 ms ago. A parked camera drifts a unit
// or two in that time and never qualifies; the return leg closes 2-4 units
// per frame, which is 20-40.
// 45 -> 60 (user: "could be slightly faster"). The camera closes 2-4 units a
// frame on the way in, so starting 15 earlier buys about 4-7 frames. Only the
// closing test makes this safe at 60 - a parked action camera at 33-40 is
// well inside this distance and must still keep the head.
constexpr float kHeadHideCloseDistance = 60.0f;        // or inside this...
constexpr float kHeadHideClosingDrop = 10.0f;          // ...while closing by at least this much...
constexpr unsigned long long kHeadHideClosingMs = 150; // ...over this long -> hide it now

// The 150 ms test alone is EVALUATED only once per 150 ms, and the return leg
// covers the whole distance inside one such window: measured 2026-09-12, the
// hides fired at 2.2-3.9 from the eye having "closed 146-190 in 150 ms", i.e.
// the camera was already home before the check next ran. That lateness is the
// hair-clipping. So the per-frame closing rate decides it too: 8 units in a
// single frame is a camera travelling home (it was doing ~21), while a parked
// action camera drifts a unit or two. This fires on the first frame inside
// kHeadHideCloseDistance instead of up to 150 ms later.
constexpr float kHeadHideClosingRate = 8.0f;           // ...or this much closer in ONE frame

// THE HEAD DOES NOT VANISH WHEN WE COLLAPSE IT - it deflates like a balloon
// over several frames, anchored at the neck (user screenshots, 2026-09-12).
// Both read-backs, taken at opposite ends of the frame and across every pass,
// only ever show 1.0 or 0.0, so the renderer is not skinning from the joint
// scale at +0x30 or the world matrix at +0x50: the animation system almost
// certainly blends the scale channel toward our target over its normal blend
// time and feeds a separate matrix palette. Writing that palette means
// finding it first.
//
// Until then, the deflate is not prevented but moved. It takes roughly the
// same time wherever it happens, so the trick is to start it while the camera
// is still far away, where a shrinking head is small, off to one side, and
// mostly behind you - instead of at half a metre, where it fills the view. A
// camera rushing home does ~20 units a frame, so requiring 15 in one frame
// keeps this off everything else: an action camera drifting near the head
// never qualifies, and neither does sprint (measured 2-4 a frame).
constexpr float kHeadHideEarlyDistance = 160.0f;       // this far out is fine to hide...
constexpr float kHeadHideEarlyRate = 15.0f;            // ...but only for a camera racing home
unsigned long long g_playerFarSinceMs = 0; // 0 = the player's eye is near the camera (or unmeasured)

// ---- Flicker guard: split screen, and anything else like it -------------
// Split screen renders two viewports per frame with two different cameras,
// and "the camera" here is whatever matrix was last left in the vertex
// constants - so the measured distance alternates between the two players'
// views and the head toggles with it. Tested 2026-09-12: Chris's head
// flickers, Sheva's is untouched (the identity latch protects her).
//
// Detecting split screen specifically would mean finding the game's local
// player count. This does not bother: a head that changes state several times
// a second is wrong whatever the cause, so when that happens the head is
// locked hidden for a few seconds. Hidden is the safe state - it is what the
// mod did for months before any of this - and the lockout expires on its own,
// so a one-off burst costs nothing and a permanent condition just keeps
// renewing it.
constexpr int kFlickerTransitions = 4;
constexpr unsigned long long kFlickerWindowMs = 1000;
constexpr unsigned long long kFlickerLockoutMs = 5000;
unsigned long long g_headFlickerFirstMs = 0;
int g_headFlickerCount = 0;
unsigned long long g_headLockoutUntilMs = 0;
std::atomic<unsigned long> g_headLockouts{0};

// Called on every show and every hide. Returns nothing - it only decides
// whether things are changing too fast to be real.
void NoteHeadTransition(unsigned long long now)
{
    if (!g_headFlickerFirstMs || now - g_headFlickerFirstMs > kFlickerWindowMs) {
        g_headFlickerFirstMs = now;
        g_headFlickerCount = 1;
        return;
    }
    if (++g_headFlickerCount < kFlickerTransitions)
        return;
    g_headLockoutUntilMs = now + kFlickerLockoutMs;
    g_headFlickerFirstMs = 0;
    g_headFlickerCount = 0;
    g_headLockouts.fetch_add(1, std::memory_order_relaxed);
    Log_Printf("CameraRigHook: head changed state %d times in under %llu ms - something is feeding us two cameras "
               "(split screen?); holding it hidden for %llu ms",
        kFlickerTransitions, kFlickerWindowMs, kFlickerLockoutMs);
}

bool HeadLockedHidden(unsigned long long now)
{
    return g_headLockoutUntilMs && now < g_headLockoutUntilMs;
}
unsigned long long g_lastHoldLogMs = 0;

// Last frame's rendered camera basis in world space (screen-right and
// forward), refreshed by LastCameraPosition. Used by AimWalk so WASD moves
// relative to the view without assuming the game's axis conventions.
float g_camRight[3] = { 1.0f, 0.0f, 0.0f };
float g_camForward[3] = { 0.0f, 0.0f, 1.0f };

// Last frame's rendered camera position, decoded from c0-c3 - the same
// maths as stereo_test.cpp's DecomposeCameraMatrix. Holds the last good
// value across UI/shadow passes.
bool LastCameraPosition(float out[3])
{
    static float s_pos[3];
    static bool s_have = false;
    float m[16];
    if (ConstantProbe_GetCachedCameraMatrix(m)) {
        const float sx = Length3(&m[0]), sy = Length3(&m[4]), sf = Length3(&m[8]);
        const bool identityish = std::fabs(sx - 1.0f) < 0.01f && std::fabs(sy - 1.0f) < 0.01f;
        if (sx > 0.05f && sy > 0.05f && sf > 0.9f && sf < 1.1f && !identityish) {
            const float rhs0 = -m[3] / sx, rhs1 = -m[7] / sy, rhs2 = -m[15];
            for (int i = 0; i < 3; ++i) {
                s_pos[i] = (m[i] / sx) * rhs0 + (m[4 + i] / sy) * rhs1 + m[8 + i] * rhs2;
                g_camRight[i] = m[i] / sx;
                g_camForward[i] = m[8 + i];
            }
            s_have = true;
            // Deliberately NOT deriving the screen aspect here any more. It
            // looked sound - Sx = Sy / aspect whatever the FOV - but "the
            // matrix cached this instant" is not always the main scene's, and
            // one squarish pass was enough to fisheye first person. The
            // backbuffer knows its own shape; see FirstPersonVerticalFov.
        }
    }
    if (s_have)
        std::memcpy(out, s_pos, sizeof(s_pos));
    return s_have;
}

HeadTrack* TrackFor(unsigned char* controller, unsigned long long now)
{
    HeadTrack* freeSlot = nullptr;
    for (HeadTrack& t : g_heads) {
        if (t.controller == controller)
            return &t;
        if (!freeSlot && (!t.controller || now - t.ms > kHeadTrackExpiryMs))
            freeSlot = &t;
    }
    if (freeSlot)
        *freeSlot = HeadTrack{ controller, nullptr, nullptr, kEyeAbovePivot, kEyeAheadOfPivot, 1e9f, now, false, false, false };
    return freeSlot;
}

void SetHeadScale(unsigned char* head, float s)
{
    float* scale = reinterpret_cast<float*>(head + kOffJointScale);
    scale[0] = scale[1] = scale[2] = s;
}

// ---- Does the game move our camera when Sheva is close? (2026-09-13) ----
// The user found the jitter's trigger by playing: "When standing near sheva,
// it jitters. Running away from her and moving my head, smooth ... as soon as
// she gets close to me, jitter" - with head-follow off as well. The log
// agrees in its own way: in those windows the rendered camera turned 2-4x
// further than the head, and the camera hook ran more often per frame.
//
// The suspicion is RE5's partner avoidance: in third person the camera is
// nudged so it does not clip through Sheva, and that correction runs AFTER
// our rig write (we hook the merge point before the blend). In a headset,
// being shoved every frame is jitter.
//
// Before hunting for that code, prove it and say what it does. Every frame
// for the player this records how far the RENDERED camera is from the eye we
// wrote, and how fast the rendered camera turns, split by whether any other
// character's head is within kStrayNearDistance. Near and far side by side,
// every 3 s: if only "near" strays, the avoidance is confirmed, and position
// versus turn says which part of the camera it touches.
constexpr float kStrayNearDistance = 150.0f; // ~1.5 m in render units
constexpr unsigned long long kStrayWindowMs = 3000;

struct StrayBucket {
    unsigned frames = 0;
    float maxOff = 0.0f;
    double sumOff = 0.0;
    double turnDeg = 0.0;
};
StrayBucket g_strayNear, g_strayFar;
unsigned long long g_strayWindowStartMs = 0;
unsigned long long g_strayLastMs = 0;
float g_strayPrevCamYaw = 0.0f;
bool g_strayHavePrev = false;
float g_strayNearest = 1e9f;

void NoteCameraStray(const HeadTrack& player, unsigned long long now)
{
    if (player.distance >= 1e8f)
        return;

    // Nearest other character's head to our eye.
    float nearest = 1e9f;
    for (const HeadTrack& t : g_heads) {
        if (&t == &player || !t.controller || !t.head || now - t.ms > kHeadTrackExpiryMs)
            continue;
        const float d[3] = { t.headPivotWorld[0] - player.eyeWorld[0], t.headPivotWorld[1] - player.eyeWorld[1],
            t.headPivotWorld[2] - player.eyeWorld[2] };
        const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len > 1.0f && len < nearest) // 0 means that track was never measured
            nearest = len;
    }

    const float camYaw = std::atan2(g_camForward[0], g_camForward[2]);
    float turn = 0.0f;
    if (g_strayHavePrev) {
        float a = camYaw - g_strayPrevCamYaw;
        while (a > kPi)
            a -= 2.0f * kPi;
        while (a < -kPi)
            a += 2.0f * kPi;
        turn = std::fabs(a) * 180.0f / kPi;
    }
    g_strayPrevCamYaw = camYaw;
    g_strayHavePrev = true;

    StrayBucket& b = nearest < kStrayNearDistance ? g_strayNear : g_strayFar;
    ++b.frames;
    b.maxOff = (std::max)(b.maxOff, player.distance);
    b.sumOff += player.distance;
    b.turnDeg += turn;
    g_strayNearest = (std::min)(g_strayNearest, nearest);

    if (!g_strayWindowStartMs)
        g_strayWindowStartMs = now;
    if (now - g_strayWindowStartMs < kStrayWindowMs)
        return;
    const double secs = (now - g_strayWindowStartMs) * 0.001;
    auto describe = [](const StrayBucket& s, char* out, size_t n) {
        if (!s.frames) {
            _snprintf_s(out, n, _TRUNCATE, "no frames");
            return;
        }
        _snprintf_s(out, n, _TRUNCATE, "%u calls, camera off our eye by max %.1f / avg %.1f, turned %.0f deg total",
            s.frames, s.maxOff, s.sumOff / s.frames, s.turnDeg);
    };
    char nearText[160], farText[160];
    describe(g_strayNear, nearText, sizeof(nearText));
    describe(g_strayFar, farText, sizeof(farText));
    Log_Printf("CameraRigHook: camera vs our pose, last %.1f s (nearest other head %.0f) - NEAR (<%.0f): %s | "
               "FAR: %s",
        secs, g_strayNearest, kStrayNearDistance, nearText, farText);
    g_strayNear = StrayBucket{};
    g_strayFar = StrayBucket{};
    g_strayNearest = 1e9f;
    g_strayWindowStartMs = now;
}

// Collapses/restores this controller's character's head, and when first
// person is on, moves both eyes onto it. Leaves the eyes untouched (the
// fixed fallback) if the skeleton isn't there.
void UpdateHead(unsigned char* controller, bool vrActive, float eyeNormal[3], float eyeAim[3])
{
    const unsigned long long now = GetTickCount64();
    HeadTrack* track = TrackFor(controller, now);
    if (!track)
        return;
    track->isPlayer = false;
    unsigned char* joints = JointArray(controller);
    if (joints != track->joints) {
        // A different character (or none): the old one is not ours to touch
        // any more. The head search walks the whole array, so it only runs
        // when the character changes.
        track->joints = joints;
        track->skel = SkeletonId{};
        track->head = joints ? FindHeadJoint(joints, &track->eyeUp, &track->eyeAhead, &track->skel) : nullptr;
        track->collapsed = false;
        track->headShown = false;
        track->headNearSinceMs = 0;
        track->havePrevDistance = false;
        track->histMs = 0;
        if (track->head)
            Log_Printf("CameraRigHook: controller %p follows a skeleton with its head at joint index %d; eye %.1f above, "
                       "%.1f ahead of the pivot (skeleton: %d joints, head %d, face %.1f)",
                controller, static_cast<int>((track->head - joints) / kJointStride), track->eyeUp, track->eyeAhead,
                track->skel.jointCount, track->skel.headIndex, track->skel.faceSize10 / 10.0f);
    }
    unsigned char* head = track->head;
    track->ms = now;
    if (!head)
        return;

    float headWorld[3];
    if (!TryRead(headWorld, head + kOffJointWorldPos, sizeof(headWorld)))
        return;

    if (!g_enabled) {
        if (track->collapsed) {
            SetHeadScale(head, 1.0f);
            track->collapsed = false;
        }
        track->headShown = false;
        track->headNearSinceMs = 0;
        track->havePrevDistance = false;
        track->histMs = 0;
        return;
    }

    // THIS is where the VR eye is placed (2026-09-23). The head lock at render
    // time turned out to run only while VR is off - one line in the log, right
    // before XR mode came on, and never again - so the neck, the eye height and
    // the running lead all went into a path the headset never uses. They live
    // here now, on the world position that becomes the eye.
    float pivotWorld[3];
    std::memcpy(pivotWorld, headWorld, sizeof(pivotWorld));
    float eyeUpUnits = -1.0f; // -1: use the skeleton's own measured eye height
    if (vrActive && g_vrEyeOnNeck.load(std::memory_order_relaxed) && joints) {
        // The neck: the head's parent. A head nods, rocks on every shot and
        // swings on a run; a neck does far less of all three.
        unsigned char links[4] = {};
        if (TryRead(links, head + kOffJointLinks, sizeof(links))) {
            const int headIndex = static_cast<int>((head - joints) / kJointStride);
            const int neckIndex = links[1];
            float neckWorld[3];
            if (neckIndex != headIndex && neckIndex < 200
                && TryRead(neckWorld, joints + neckIndex * kJointStride + kOffJointWorldPos, sizeof(neckWorld))) {
                const float gap[3] = { headWorld[0] - neckWorld[0], headWorld[1] - neckWorld[1],
                    headWorld[2] - neckWorld[2] };
                if (Length3(gap) < 60.0f) {
                    std::memcpy(pivotWorld, neckWorld, sizeof(neckWorld));
                    eyeUpUnits = g_vrEyeAboveNeck.load(std::memory_order_relaxed);
                }
            }
        }
    }
    // Running lead: one tick of the character's own travel, so the eye lands
    // where his head is when the frame is drawn rather than where it was. The
    // root joint is the character himself; horizontal only, never a step that
    // no run could make.
    if (vrActive && joints) {
        const float lead = g_vrRunLead.load(std::memory_order_relaxed);
        float rootNow[3];
        if (lead > 0.0f && TryRead(rootNow, joints + kOffJointWorldPos, sizeof(rootNow))) {
            if (track->haveRoot) {
                const float step[3] = { rootNow[0] - track->prevRoot[0], 0.0f, rootNow[2] - track->prevRoot[2] };
                const float flat = std::sqrt(step[0] * step[0] + step[2] * step[2]);
                if (flat > 0.02f && flat < 30.0f) {
                    pivotWorld[0] += step[0] * lead;
                    pivotWorld[2] += step[2] * lead;
                    track->lastLead = flat * lead;
                }
            }
            std::memcpy(track->prevRoot, rootNow, sizeof(rootNow));
            track->haveRoot = true;
        }
    }

    // Eye in rig space. VR gets its own forward offset (g_vrEyeAheadScale).
    float pivotNormal[3], pivotAim[3];
    WorldToRig(controller, kOffNormalTransform, pivotWorld, pivotNormal);
    WorldToRig(controller, kOffAimTransform, pivotWorld, pivotAim);
    // Plausibility: a head pivot is roughly above the root, at head height.
    const bool plausible = pivotNormal[1] >= 80.0f && pivotNormal[1] <= 250.0f &&
        std::fabs(pivotNormal[0]) <= 100.0f && std::fabs(pivotNormal[2]) <= 100.0f;
    const EyeOffsetScale& eyeScale = vrActive ? g_vrEye : g_flatEye;
    const float ahead = track->eyeAhead * eyeScale.ahead;
    const float up = eyeUpUnits >= 0.0f ? eyeUpUnits : track->eyeUp * eyeScale.up;
    const float eyeRigNormal[3] = { pivotNormal[0], pivotNormal[1] + up, pivotNormal[2] + ahead };
#if RE5VR_DIAGNOSTICS
    if (vrActive && track->isPlayer) {
        static unsigned long long s_toldMs = 0;
        if (now - s_toldMs >= 3000) {
            s_toldMs = now;
            Log_Printf("VrEye: on the %s, %.1f units up and %.1f ahead; the running lead moved it up to %.1f units",
                eyeUpUnits >= 0.0f ? "neck" : "head", up, ahead, track->lastLead);
            track->lastLead = 0.0f;
        }
    }
#endif
    const float eyeRigAim[3] = { pivotAim[0], pivotAim[1] + up, pivotAim[2] + ahead };

    // Which character is YOU: the one whose EYE is where last frame's camera
    // was. The first version compared head PIVOTS to the camera - but your
    // own pivot is ~19 units from your eye, so whenever Sheva's head came
    // within ~19 units of the camera she won and her head collapsed instead
    // (user, flat and VR). Your eye is essentially at the camera; hers
    // practically never is. Held once chosen (see g_playerController): the
    // old instant switch still let Sheva steal it whenever her eye came
    // within 25 of the camera while yours measured further or not at all -
    // online, "certain times when we got close" (user, 2026-09-11).
    track->distance = 1e9f;
    float cam[3];
    if (plausible && LastCameraPosition(cam)) {
        float eyeWorld[3];
        RigToWorld(controller, kOffNormalTransform, eyeRigNormal, eyeWorld);
        const float d[3] = { eyeWorld[0] - cam[0], eyeWorld[1] - cam[1], eyeWorld[2] - cam[2] };
        track->distance = Length3(d);
        // Kept for the trace. A distance says two points are apart; it does
        // not say whether either of them is where it should be, and this
        // whole investigation has been reasoning about a scalar without ever
        // looking at its endpoints.
        std::memcpy(track->camWorld, cam, sizeof(cam));
        std::memcpy(track->eyeWorld, eyeWorld, sizeof(eyeWorld));
        std::memcpy(track->headPivotWorld, headWorld, sizeof(headWorld));
    }
    // Candidates are controllers that are embedded in a main camera and
    // active in it (IsActiveMainCameraController). That alone is NOT unique:
    // Sheva has her own main camera (co-op), so both pass - in the 07:18 run
    // the pick flipped to her controller and collapsed her head. Among the
    // candidates, the one being rendered is the one whose eye is where the
    // rendered camera was (Chris's ~0 away, Sheva's 242-290 in that run).
    // With no candidates at all, every character is considered.
    track->direct = IsActiveMainCameraController(controller);
    bool anyDirect = false;
    for (const HeadTrack& t : g_heads) {
        if (t.controller && t.head && now - t.ms <= kHeadTrackExpiryMs && t.direct)
            anyDirect = true;
    }
    // Identity first. Once the player's skeleton is known, a controller that
    // is not following it cannot become the player and cannot lose its head,
    // no matter where the camera happens to be. This is the rule that keeps
    // Sheva's head on: not a better distance, just never asking the question
    // about her in the first place. It is symmetric - if the player IS Sheva,
    // the same rule protects Chris.
    if (g_playerSkeleton.valid) {
        bool anyMatch = false;
        for (const HeadTrack& t : g_heads) {
            if (t.controller && t.head && now - t.ms <= kHeadTrackExpiryMs && SkeletonMatches(t.skel, g_playerSkeleton))
                anyMatch = true;
        }
        if (anyMatch) {
            g_playerSkeletonSeenMs = now;
        } else if (g_playerSkeletonSeenMs && now - g_playerSkeletonSeenMs >= kPlayerSkeletonGoneMs) {
            Log_Printf("CameraRigHook: the player's skeleton (%d joints, head %d, face %.1f) has been gone for %llu ms "
                       "- unlatching, will re-identify",
                g_playerSkeleton.jointCount, g_playerSkeleton.headIndex, g_playerSkeleton.faceSize10 / 10.0f,
                now - g_playerSkeletonSeenMs);
            g_playerSkeleton = SkeletonId{};
            g_playerSkeletonSeenMs = 0;
        }
    }

    const HeadTrack* best = nullptr;
    for (const HeadTrack& t : g_heads) {
        if (!t.controller || !t.head || now - t.ms > kHeadTrackExpiryMs || (anyDirect && !t.direct))
            continue;
        if (g_playerSkeleton.valid && !SkeletonMatches(t.skel, g_playerSkeleton))
            continue;
        if (!best || t.distance < best->distance)
            best = &t;
    }

    // Latching. Only from a camera essentially inside the head - being the
    // nearest candidate is not enough, because "nearest" is whoever is left
    // when the real player is not measurable, and a wrong latch here would
    // stick.
    if (!g_playerSkeleton.valid && track->skel.valid && track->distance < kPlayerLatchDistance) {
        g_playerSkeleton = track->skel;
        g_playerSkeletonSeenMs = now;
        Log_Printf("CameraRigHook: the player is the skeleton with %d joints, head at index %d, face %.1f "
                   "(camera %.1f from its eye) - no other character's head will be touched",
            track->skel.jointCount, track->skel.headIndex, track->skel.faceSize10 / 10.0f, track->distance);
    }
    // Hold the current player (see g_playerController). Its "far" clock only
    // runs on a real measurement: an unmeasured frame (an odd pose failing
    // the plausibility check) is no evidence either way.
    if (controller == g_playerController && track->distance < 1e8f) {
        if (track->distance <= kPlayerKeepDistance)
            g_playerFarSinceMs = 0;
        else if (!g_playerFarSinceMs)
            g_playerFarSinceMs = now;
    }
    const HeadTrack* current = nullptr;
    for (const HeadTrack& t : g_heads) {
        if (t.controller && t.controller == g_playerController && t.head && now - t.ms <= kHeadTrackExpiryMs)
            current = &t;
    }
    const bool currentHolds = current && !(g_playerFarSinceMs && now - g_playerFarSinceMs >= kPlayerSwitchDelayMs);
    if (best && best->distance < kPlayerEyeMaxDistance && best->controller != g_playerController) {
        if (currentHolds) {
            if (now - g_lastHoldLogMs >= 5000) {
                g_lastHoldLogMs = now;
                Log_Printf("CameraRigHook: kept player controller %p - %p's eye came within %.1f of the camera "
                           "(the player's: %.1f)",
                    g_playerController, best->controller, best->distance, current->distance);
            }
        } else {
            Log_Printf("CameraRigHook: player controller now %p (%s, eye %.1f from the rendered camera)",
                best->controller, best->direct ? "main-camera candidate" : "fallback", best->distance);
            g_playerController = best->controller;
            g_playerFarSinceMs = 0;
        }
    }
    // Belt and braces: even if the pick still points here, a mismatched
    // skeleton is not the player. Costs nothing and means one stale pointer
    // can never take a partner's head off.
    const bool nearest = controller == g_playerController &&
        (!g_playerSkeleton.valid || SkeletonMatches(track->skel, g_playerSkeleton));

    track->isPlayer = nearest;
#if RE5VR_DIAGNOSTICS
    // Near/far camera stray log, every 3 s. It ruled out partner avoidance: the
    // near-Sheva jitter was the game dropping to ~55 fps against a 45 Hz
    // headset on the laptop, not the camera being moved. Developer builds only.
    if (nearest && g_enabled)
        NoteCameraStray(*track, now);
#endif

    // Coming back from a scripted action: the watchdog has been deciding the
    // head while this hook was not running, so adopt whatever it left on
    // screen rather than rediscovering it. Two things would otherwise go
    // wrong. The stale previous distance makes a huge phantom jump on the
    // resume frame, which fires the "the game cut the camera" path for an
    // action that is already ending; and the state machine would disagree with
    // what is actually rendered, so the hide timer would never start. Both
    // read to the player as the head hanging around too long afterwards.
    if (nearest && g_lastPlayerHeadMs && now - g_lastPlayerHeadMs >= kHookQuietMs) {
        track->headShown = g_watchdogRestored;
        track->histDistance = track->distance;
        track->histMs = now;
        track->havePrevDistance = false;
        track->headNearSinceMs = 0;
    }

    // Head on or off - never in between. Only decide on a real measurement:
    // an unmeasured frame (1e9 sentinel - an implausible pose, or no camera
    // matrix cached yet) is no evidence either way, and treating it as "far"
    // would flash a head through every UI pass. Unmeasured frames hold.

    if (nearest && track->distance < 1e8f) {
        // One frame's worth of movement. Only meaningful against the previous
        // MEASURED frame, so unmeasured frames don't manufacture a jump.
        const float jump = track->havePrevDistance ? track->distance - track->prevDistance : 0.0f;
        track->prevDistance = track->distance;
        track->havePrevDistance = true;

        if (!track->headShown) {
            // OUR OWN LEAD IS NOT THE CAMERA RUNNING AWAY (2026-09-25, user:
            // "with the run lead set to 4, I don't think the head is hiding").
            //
            // The lead pushes the eye ahead of the body by four ticks of
            // movement, and a sprint step runs to thirty units, so it can hold
            // the eye more than a hundred units in front - past the sixty this
            // test calls "the camera has been taken away", and breaking into a
            // run applies it fast enough to look like the fifty-unit jump too.
            // So sprinting showed the head, which is the one thing first person
            // must never do.
            //
            // It was written when the lead was two and a sprint could not quite
            // reach the threshold. Both numbers now allow for whatever we are
            // doing ourselves, so the test means what it says at any lead.
            const float lead = g_leadUnits.load(std::memory_order_relaxed);
            if (track->distance > kHeadShowDistance + lead && jump > kHeadShowJump + lead
                && !HeadLockedHidden(now) && g_showHeadDuringActions.load(std::memory_order_relaxed)) {
                track->headShown = true;
                track->histDistance = track->distance;
                track->histMs = now;
                track->headNearSinceMs = 0;
                g_headShownEvents.fetch_add(1, std::memory_order_relaxed);
                NoteHeadTransition(now);
                Log_Printf("CameraRigHook: the game cut the camera away (%.1f from the eye, +%.1f in one frame)%s"
                           " - head on",
                    track->distance, jump, ShiftHeld() ? " WHILE SHIFT IS DOWN (sprint?)" : "");
            } else {
                // Two separate things worth knowing when this misfires: how
                // far the camera got without the head coming on, and the
                // biggest single-frame jump that did NOT qualify. A level
                // that climbs to 111 with no jump is a trailing camera; a
                // jump near the threshold is a cut we nearly missed.
                const long d = static_cast<long>(track->distance);
                long seen = g_headNearMaxDistance.load(std::memory_order_relaxed);
                while (d > seen && !g_headNearMaxDistance.compare_exchange_weak(seen, d, std::memory_order_relaxed)) {
                }
                const long j = static_cast<long>(jump);
                long seenJump = g_headNearMaxJump.load(std::memory_order_relaxed);
                while (j > seenJump &&
                       !g_headNearMaxJump.compare_exchange_weak(seenJump, j, std::memory_order_relaxed)) {
                }
            }
        } else {
            // Closing test first: a camera returning to the eye should give
            // the head back before it reaches the skull, not after.
            const bool haveHist = track->histMs && now - track->histMs >= kHeadHideClosingMs;
            const float closed = haveHist ? track->histDistance - track->distance : 0.0f;
            if (haveHist) {
                track->histDistance = track->distance;
                track->histMs = now;
            }
            // -jump is this frame's closing rate: the show branch computes it
            // as distance - prevDistance, so a camera coming home is negative.
            const float closingNow = -jump;
            const bool comingHome = closed >= kHeadHideClosingDrop || closingNow >= kHeadHideClosingRate;
            // The early path: much further out, but only for a camera that is
            // unmistakably racing back, so the deflate happens at distance.
            const bool racingHome = track->distance < kHeadHideEarlyDistance &&
                closingNow >= kHeadHideEarlyRate;
            if (racingHome || (track->distance < kHeadHideCloseDistance && comingHome)) {
                track->headShown = false;
                track->headNearSinceMs = 0;
                NoteHeadTransition(now);
                HideTrace_Open(now);
                Log_Printf("CameraRigHook: camera coming back to the eye (%.1f, closing %.1f this frame, %.1f in "
                           "%llu ms)%s - head off",
                    track->distance, closingNow, closed, kHeadHideClosingMs,
                    ShiftHeld() ? " WHILE SHIFT IS DOWN (sprint?)" : "");
            } else if (track->distance < kHeadHideDistance) {
                // Backstop for a camera that arrives slowly enough not to
                // count as closing: it just has to stay near the eye.
                if (!track->headNearSinceMs)
                    track->headNearSinceMs = now;
                else if (now - track->headNearSinceMs >= kHeadHideDwellMs) {
                    track->headShown = false;
                    track->headNearSinceMs = 0;
                    NoteHeadTransition(now);
                    HideTrace_Open(now);
                    Log_Printf("CameraRigHook: camera back on the eye (%.1f for %llu ms)%s - head off",
                        track->distance, kHeadHideDwellMs,
                        ShiftHeld() ? " WHILE SHIFT IS DOWN (sprint?)" : "");
                }
            } else {
                track->headNearSinceMs = 0;
            }
        }
    }

    if (nearest) {
        // The watchdog needs to know who to fall back to, and that this frame
        // happened at all - its whole job is noticing that these stop.
        g_lastPlayerHead = head;
        g_lastPlayerJoints = track->joints;
        g_lastPlayerHeadMs = now;
        g_watchdogRestored = false;
        const float* scale = reinterpret_cast<const float*>(head + kOffJointScale);
        if (track->collapsed && scale[0] != 0.0f)
            g_headScaleResets.fetch_add(1, std::memory_order_relaxed);
        SetHeadScale(head, track->headShown ? 1.0f : 0.0f);
        track->collapsed = !track->headShown;
        if (!track->headShown)
            g_headCollapseFrames.fetch_add(1, std::memory_order_relaxed);
    } else if (track->collapsed) {
        SetHeadScale(head, 1.0f);
        track->collapsed = false;
        track->headShown = true;
    }

    // Trace, AFTER the write, so the scale reported is the one now sitting in
    // the joint rather than the one intended. The user reports the head is
    // not visible while the action camera is out even on frames where this
    // code has un-collapsed it, so what we asked for and what the model does
    // are not the same thing, and only a read-back can tell them apart.
    // Positions are here for the same reason: "100 units apart" says nothing
    // about whether the camera is behind Chris or in another room.
#if RE5VR_DIAGNOSTICS
    if (nearest && g_actionTraceUntilMs && now <= g_actionTraceUntilMs) {
        // The renderer reads the WORLD matrix, not the local scale we write, and
        // the game rebuilds it from the animation. If a scripted animation stops
        // rebuilding this joint, our scale sits in memory doing nothing - which
        // is what "head appears as Chris stands up" looks like. Row 0's length is
        // the world scale: ~0 collapsed, ~1 restored.
        float row0[3] = { 0.0f, 0.0f, 0.0f };
        TryRead(row0, head + kOffJointWorldMatrix, sizeof(row0));
        const float worldScale = Length3(row0);
        float scaleNow[3] = { -1.0f, -1.0f, -1.0f };
        TryRead(scaleNow, head + kOffJointScale, sizeof(scaleNow));
        // The head joint collapses in one frame - proven by the read-back
        // above - and the user still sees it "deflate" three times out of
        // three. So look at what hangs OFF the head: face and hair joints are
        // its children, and hair in particular is usually driven by its own
        // dynamics, which would chase the collapsed head over several frames
        // instead of snapping with it. Count how many children still have a
        // non-zero world scale, and how big the largest one is.
        int childrenAlive = 0;
        float biggestChild = 0.0f;
        if (track->joints && track->skel.valid) {
            for (int i = 0; i < track->skel.jointCount && i < kMaxJoints; ++i) {
                unsigned char* j = track->joints + i * kJointStride;
                unsigned char links[4] = {};
                if (!TryRead(links, j + kOffJointLinks, sizeof(links)) || links[1] != track->skel.headIndex)
                    continue;
                float childRow[3] = { 0.0f, 0.0f, 0.0f };
                if (!TryRead(childRow, j + kOffJointWorldMatrix, sizeof(childRow)))
                    continue;
                const float s = Length3(childRow);
                if (s > 0.05f)
                    ++childrenAlive;
                if (s > biggestChild)
                    biggestChild = s;
            }
        }
        if (track->distance < 1e8f) {
            Log_Printf("CameraRigHook: trace - dist %.1f, head %s (joint scale %.2f, world %.2f, %d child joints alive, "
                       "biggest %.2f), cam (%.0f %.0f %.0f) "
                       "eye (%.0f %.0f %.0f) pivot (%.0f %.0f %.0f)",
                track->distance, track->headShown ? "ON" : "off", scaleNow[0], worldScale, childrenAlive, biggestChild,
                track->camWorld[0], track->camWorld[1], track->camWorld[2],
                track->eyeWorld[0], track->eyeWorld[1], track->eyeWorld[2],
                track->headPivotWorld[0], track->headPivotWorld[1], track->headPivotWorld[2]);
        } else {
            Log_Printf("CameraRigHook: trace - no measurement, head %s (joint scale %.2f, world %.2f)",
                track->headShown ? "ON" : "off", scaleNow[0], worldScale);
        }
    } else if (g_actionTraceUntilMs && now > g_actionTraceUntilMs) {
        g_actionTraceUntilMs = 0;
        Log_Printf("CameraRigHook: trace ended");
    }
#endif // RE5VR_DIAGNOSTICS

    if (!plausible)
        return;
    std::memcpy(eyeNormal, eyeRigNormal, sizeof(eyeRigNormal));
    std::memcpy(eyeAim, eyeRigAim, sizeof(eyeRigAim));

    const int logs = g_eyeLogsRemaining.load(std::memory_order_relaxed);
    if (nearest && logs > 0) {
        g_eyeLogsRemaining.store(logs - 1, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: head joint %p pivot rig-space (%.1f, %.1f, %.1f) -> eye (%.1f, %.1f, %.1f), "
                   "%.1f from last camera, collapsed",
            head, pivotNormal[0], pivotNormal[1], pivotNormal[2], eyeNormal[0], eyeNormal[1], eyeNormal[2],
            track->distance);
    }
}

bool IsPlayerController(const unsigned char* controller)
{
    for (const HeadTrack& t : g_heads) {
        if (t.controller == controller)
            return t.isPlayer;
    }
    return false;
}

// ---- Move while aiming (2026-09-11, prototype) -------------------------
// RE5 roots the player while aiming, and no working mod exists - the usual
// blocker is animation (there's no walk-and-aim set). In first person the
// legs aren't seen, so the aim pose stays and the body slides.
//
// The F5 write-watch on the character's position (+0x30) found how the game
// moves the player: action code (+87A69B, +87ACF8, +88976E) adds the
// animation's root-motion delta to the position - newPos = oldPos + R *
// motion - at roughly 25 Hz. The aim animation has no root motion, so the
// delta is zero. This prototype adds a WASD displacement to the position
// directly while aiming. Open question it answers: whether collision
// applies to any position change (walls stop you) or only inside the
// game's own movement code (you'd pass through - then the displacement has
// to go through the collision routine instead).
//
// Directions come from the rendered camera's own basis (screen-right and
// forward, flattened), so A/D can't come out mirrored. Keyboard (WASD) or a
// gamepad's left stick - analog, so half a push walks at half speed (added
// 2026-09-11 after v0.3.0 shipped keyboard-only). First person (F4) only,
// player character only, aim flag at controller+0x1B1 (the camera
// function's aim-group switch).
constexpr bool kMoveWhileAiming = true;
constexpr DWORD kOffControllerAimFlag = 0x1B1;

// THE ACTION CAMERA FLAG (2026-09-28, found by elimination on End).
//
// Narrowed from 174 differing bytes to exactly one over four alternating
// rounds of ordinary play and melee:
//
//   round 3 leaves 2 - +183(BD vs 3D), +634(01 vs 00)
//   round 4 leaves 1 - +634(01 vs 01)   ordinary
//   round 5 leaves 1 - +634(01 vs 00)   melee
//   round 6 leaves 1 - +634(01 vs 01)   ordinary
//
// One in ordinary play, zero while the game is running an action camera.
// This is the thing every fix for the past day has been improvising around -
// rate thresholds, mouse twitches, servo timestamps, hold expiry - all of it
// was guessing at a state the game keeps in a single byte.
constexpr DWORD kOffControllerActionCam = 0x634;

bool GameIsDrivingTheCamera()
{
    unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    unsigned char flag = 1;
    if (!pc || !TryRead(&flag, pc + kOffControllerActionCam, sizeof(flag)))
        return false; // unknown: assume it is yours, which is the safe default
    if (flag != 0)
        return false;
    // EXCEPT AIMING (2026-09-28, user: "something is locking my camera in
    // place while aiming").
    //
    // So +0x634 is not "an action camera is playing". It is closer to "you are
    // not on the normal camera", and raising the gun counts, because RE5
    // switches to its aim camera. The anchor was grabbing you the instant you
    // aimed, which is both wrong and the most noticeable possible place to be
    // wrong.
    //
    // Aiming has its own flag, found long before this one, so the two together
    // say what we actually mean: the game has taken the camera, and not merely
    // because you are looking down the sights.
    unsigned char aim = 0;
    if (TryRead(&aim, pc + kOffControllerAimFlag, sizeof(aim)) && aim == 1)
        return false;
    return true;
}
constexpr DWORD kOffCharacterPos = 0x30;
constexpr float kAimWalkSpeed = 100.0f; // render units per second - first guess, tune by feel
// After each step, do what the game's movement code does (see the end of
// AimWalk): set the character's "moved" bit and run its step handler.
// Multiplayer test switch, default OFF. While on, each aim-walk step is
// committed the way the game's own movement code does: the character's
// "moved" flag plus its step handler. That may be what makes the move reach a
// co-op partner - but the weapon path consumes the same flag, so the gun
// fires several rounds per trigger pull while it is on (user, 2026-09-11).
//
// That note has sat here since September and names the suspect without ever
// testing it, because the switch was a BOOL: it did two things at once, so a
// session could only ever report that the pair of them syncs the move and the
// pair of them dumps the magazine. Which of the two does which was never
// asked. So it is a mode now, and one co-op session can answer it:
//
//   Off      nothing. The partner sees you frozen while you aim.
//   Flag     set the moved bit and stop. If this alone syncs, the handler
//            call was never needed and the fault goes with it.
//   Handler  run the step handler without ever setting the bit. If THIS alone
//            syncs, the weapon path never sees anything it should not.
//   Both     what the old switch did, kept so the fault can be reproduced
//            on purpose rather than only stumbled into.
//   Tidy     both, and then put the bit back exactly as it was found. If the
//            handler is what syncs but the bit has to be up while it runs,
//            this is the one that works AND leaves nothing behind for the
//            weapon to read later in the frame. It is the candidate fix.
//
// Firing without ammo is almost certainly the same fault seen from another
// angle: a shot that goes down a path which never reaches the ammo count.
enum AimWalkCommit { kCommitOff = 0, kCommitFlag, kCommitHandler, kCommitBoth, kCommitTidy };
std::atomic<int> g_aimWalkCommit{ kCommitOff };
constexpr DWORD kOffCharacterStepFlags = 0x2D7C;
constexpr DWORD kStepMovedFlag = 0x10000000;
constexpr DWORD kOffCharacterStepHandler = 0x2F30;
constexpr DWORD kRvaStepHandlerUpdate = 0x864B40;
// How often the commit may run (2026-09-19). With it on every frame the user
// reported "occasionally teleporting happens, and when you shoot you kind of
// continuously teleport" - which is what an increment applied five times per
// step looks like, and why shooting makes it worse rather than better: a
// recoil impulse gets applied once per call instead of once.
//
// The game does this after a root-motion step, at roughly 25 Hz. At 120 fps we
// were doing it nearly five times as often. The position write stays on every
// frame, because that is what makes the walk smooth; only the ritual is paced.
std::atomic<float> g_aimWalkCommitHz{ 25.0f };
std::atomic<bool> g_aimWalkThirdPerson{ false };

// XInput is loaded at runtime rather than linked, so a missing DLL only
// means no gamepad walking. 1_4 ships with Windows 8+, 1_3 with the DirectX
// runtime RE5 installs, 9_1_0 with Vista/7.
typedef DWORD(WINAPI* XInputGetState_t)(DWORD userIndex, XINPUT_STATE* state);
XInputGetState_t g_xinputGetState = nullptr;

void LoadXInput()
{
    static const char* const kDlls[] = { "xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll" };
    for (const char* name : kDlls) {
        HMODULE m = LoadLibraryA(name);
        if (!m)
            continue;
        g_xinputGetState = reinterpret_cast<XInputGetState_t>(GetProcAddress(m, "XInputGetState"));
        if (g_xinputGetState) {
            Log_Printf("CameraRigHook: gamepad walk-while-aiming via %s", name);
            return;
        }
    }
    Log_Printf("CameraRigHook: no XInput DLL found - walk-while-aiming is keyboard only");
}

// Left stick of the first connected pad, deadzone removed, as forward/strafe
// with length 0..1. Polling an empty XInput slot is slow, so while no pad is
// connected the slots are only rescanned every 2 s.
// The first connected pad's whole state. Shared by walk-while-aiming and the
// recenter combo, so both see the same pad and the slot scan happens once.
bool ReadPadState(XINPUT_STATE* out)
{
    static bool s_loaded = false;
    if (!s_loaded) {
        s_loaded = true;
        LoadXInput();
    }
    if (!g_xinputGetState)
        return false;

    static int s_pad = -1;
    static ULONGLONG s_lastScanMs = 0;
    XINPUT_STATE st = {};
    if (s_pad >= 0 && g_xinputGetState(static_cast<DWORD>(s_pad), &st) != ERROR_SUCCESS)
        s_pad = -1;
    if (s_pad < 0) {
        const ULONGLONG nowMs = GetTickCount64();
        if (nowMs - s_lastScanMs < 2000)
            return false;
        s_lastScanMs = nowMs;
        for (DWORD i = 0; i < XUSER_MAX_COUNT && s_pad < 0; ++i) {
            if (g_xinputGetState(i, &st) == ERROR_SUCCESS)
                s_pad = static_cast<int>(i);
        }
        if (s_pad < 0)
            return false;
        Log_Printf("CameraRigHook: gamepad found in XInput slot %d", s_pad);
    }
    *out = st;
    return true;
}

// The pad, motion controllers included (v0.4.3). Walking while aiming is the
// mod's own doing - RE5 roots you the moment the gun comes up - so it reads
// the stick here rather than through the game, and this reader knew only
// about real pads. Hidden while the mod menu is open, exactly like the game's
// own pad reads, so steering the menu doesn't walk Chris around.
bool ReadPadStateWithMotion(XINPUT_STATE* out)
{
    const bool real = ReadPadState(out);
    if (InputBlock_IsBlocking())
        return real;

    XINPUT_GAMEPAD motion = {};
    if (!XrInput_GetPad(&motion))
        return real;
    if (!real)
        *out = XINPUT_STATE{};

    XINPUT_GAMEPAD& g = out->Gamepad;
    g.wButtons |= motion.wButtons;
    if (motion.bLeftTrigger > g.bLeftTrigger)
        g.bLeftTrigger = motion.bLeftTrigger;
    if (motion.bRightTrigger > g.bRightTrigger)
        g.bRightTrigger = motion.bRightTrigger;
    const auto add = [](SHORT a, SHORT b) {
        const long sum = static_cast<long>(a) + static_cast<long>(b);
        return static_cast<SHORT>(sum < -32767 ? -32767 : (sum > 32767 ? 32767 : sum));
    };
    g.sThumbLX = add(g.sThumbLX, motion.sThumbLX);
    g.sThumbLY = add(g.sThumbLY, motion.sThumbLY);
    g.sThumbRX = add(g.sThumbRX, motion.sThumbRX);
    g.sThumbRY = add(g.sThumbRY, motion.sThumbRY);
    return true;
}

bool ReadLeftStick(float* forward, float* strafe)
{
    XINPUT_STATE st = {};
    if (!ReadPadStateWithMotion(&st))
        return false;

    const float x = st.Gamepad.sThumbLX, y = st.Gamepad.sThumbLY;
    const float len = std::sqrt(x * x + y * y);
    const float dead = static_cast<float>(XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
    if (len <= dead)
        return false;
    float mag = (len - dead) / (32767.0f - dead);
    if (mag > 1.0f)
        mag = 1.0f;
    *strafe = x / len * mag;
    *forward = y / len * mag;
    return true;
}

// ---- 3DOF aiming: the gun's pitch follows the controller ----------------
// See the call site in OnRigsReady for what the probe run established. The
// ends of the game's own range, from that log: about 85 degrees up and 74
// down, which is further than a person usually points.
// 3DOF aiming is unfinished and switched off for release: no checkbox, no
// ini keys, and no game code patched for it. Set to true to resume the work.
constexpr bool kInstallAimHooks = false;

constexpr float kAimPitchUpDeg = 85.0f;
constexpr float kAimPitchDownDeg = -74.0f;
// A field step this big is the most one frame may ask for, so a lost target
// or a weapon change can't snap the gun.
constexpr float kAimFieldMaxStep = 0.35f;

// The pitch the rig actually came out at, from eye (+0x170) to target
// (+0x190). This is the feedback the loop closes on.
// The direction the rig points, in world degrees around the vertical. Needed to
// see what the yaw axis actually achieves: without it, yaw is driven blind and
// any cap on it is invisible.
bool RigYawDegrees(unsigned char* controller, float* outDeg)
{
    float eye[3] = {}, target[3] = {};
    if (!TryRead(eye, controller + 0x170, sizeof(eye)) || !TryRead(target, controller + 0x190, sizeof(target)))
        return false;
    const float dx = target[0] - eye[0], dz = target[2] - eye[2];
    if (dx * dx + dz * dz < 1e-6f)
        return false;
    *outDeg = std::atan2(dx, dz) * 180.0f / 3.14159265358979f;
    return true;
}

bool RigPitchDegrees(unsigned char* controller, float* outDeg)
{
    float eye[3] = {}, target[3] = {};
    if (!TryRead(eye, controller + 0x170, sizeof(eye)) || !TryRead(target, controller + 0x190, sizeof(target)))
        return false;
    const float dx = target[0] - eye[0], dy = target[1] - eye[1], dz = target[2] - eye[2];
    const float flat = std::sqrt(dx * dx + dz * dz);
    if (flat < 1e-3f)
        return false;
    *outDeg = std::atan2(dy, flat) * 180.0f / 3.14159265358979f;
    return true;
}

// ---- The game's own aim speed (2026-09-17) ------------------------------
// The chase ended here. exe+7879B2 is the aim integrator, and it reads its
// speeds straight out of a settings block:
//
//   00B877FC: mov   ecx,[011B27B4]   ; the block
//   00B87802: movss xmm1,[ecx+2D0h]  ; speed, one axis
//   00B8780A: movss xmm2,[ecx+2D4h]  ; speed, the other
//   00B87820: movss xmm0,[ecx+2D8h]  ; a further multiplier, one mode only
//   ...      the stick is multiplied by those, then by the frame time at
//            [[011B209Ch]+28h], then subtracted from the angle at [esi+4B4h]
//            (yaw) and [esi+4B8h] (pitch), clamped, and stored.
//
// So the 86 degrees a second is not a hard limit anywhere - it is these
// numbers times full stick. Scaling them raises the ceiling with no code
// patch at all, which is as close to a free fix as this has come.
//
// The originals are captured before anything is written and put back the
// moment the multiplier goes to 1, so a player's own sensitivity setting is
// never left changed behind them. Nothing is saved: this block is the live
// copy, not the one on disk.
// What full stick buys with the game's own speeds: measured on the user's
// machine with sensitivity at maximum.
constexpr float kGameTurnRateDeg = 86.0f;
constexpr DWORD kAimSpeedPointerRva = 0xDB27B4; // 011B27B4 at the 0x400000 base
constexpr DWORD kOffAimSpeed[3] = { 0x2D0, 0x2D4, 0x2D8 };
float g_aimSpeedOriginal[3] = {};
float g_aimSpeedWritten[3] = {};
bool g_haveAimSpeedOriginal = false;

unsigned char* AimSpeedBlock()
{
    const uintptr_t base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    unsigned char* block = nullptr;
    if (!TryRead(&block, reinterpret_cast<void*>(base + kAimSpeedPointerRva), sizeof(block)))
        return nullptr;
    return block;
}

// Writes three target speeds, keeping track of what the game's own values were.
// Call every frame: the game rewrites the block whenever the options change, so
// anything found there that we did not put there is treated as the new
// original.
// mayLearn: only true when nothing of ours is in the block, so the values
// there are certainly the game's own.
//
// Getting this wrong ate the aim entirely (2026-09-17, user: "still can't move
// the gun at all"). The old version re-learned the "original" whenever the
// block held something it did not remember writing, and its memory was stale
// by design: it skipped the write when a value already matched, and skipped
// updating the memory with it. So a frame later the block looked unfamiliar,
// it learned OUR value as the game's own, multiplied that by a small scale,
// wrote something smaller, learned that, and spiralled down to zero in a
// second. The log caught it mid-fall: "the game's own aim speeds are 0.0000,
// 0.0000" repeating every frame. Zero there is an aim that cannot move, by any
// input, which is exactly what it felt like.
void AimSpeed_WriteTargets(const float target[3], bool mayLearn)
{
    unsigned char* block = AimSpeedBlock();
    if (!block)
        return;

    float now[3] = {};
    for (int i = 0; i < 3; ++i) {
        if (!TryRead(&now[i], block + kOffAimSpeed[i], sizeof(float)))
            return;
    }

    if (mayLearn) {
        bool ours = g_haveAimSpeedOriginal;
        for (int i = 0; i < 3 && ours; ++i)
            ours = now[i] == g_aimSpeedWritten[i];
        // And never learn a speed that could not be a real setting. The game
        // ships 57.9 here; anything near zero is either our own mistake or a
        // moment when the value means nothing, and taking it as the truth is
        // what makes the aim unrecoverable.
        const bool plausible = now[0] > 0.01f && now[1] > 0.01f;
        if (!ours && plausible) {
            std::memcpy(g_aimSpeedOriginal, now, sizeof(now));
            g_haveAimSpeedOriginal = true;
            Log_Printf("AimSpeed: the game's own aim speeds are %.4f, %.4f, %.4f", now[0], now[1], now[2]);
        }
    }
    if (!g_haveAimSpeedOriginal)
        return; // nothing to scale against yet

    for (int i = 0; i < 3; ++i) {
        if (now[i] != target[i])
            TryWrite(block + kOffAimSpeed[i], &target[i], sizeof(target[i]));
        // Remembered whether or not a write was needed: a memory that only
        // records the writes it performed is the stale memory described above.
        g_aimSpeedWritten[i] = target[i];
    }
}

// Multiplies all three by mult (1.0 puts the game's own values back).
void AimSpeed_Apply(float mult)
{
    unsigned char* block = AimSpeedBlock();
    if (!block)
        return;
    float keep[3] = {};
    for (int i = 0; i < 3; ++i) {
        if (!TryRead(&keep[i], block + kOffAimSpeed[i], sizeof(float)))
            return;
    }
    // Learning happens here and nowhere else: this runs only while the servo is
    // idle, so whatever is in the block is the game's.
    if (!g_haveAimSpeedOriginal) {
        AimSpeed_WriteTargets(keep, true);
        if (!g_haveAimSpeedOriginal)
            return; // the value there is not usable yet - leave it alone
    }
    // Stranded from an earlier run of the mod: the block holds a dead zero and
    // the game will not put its own value back by itself, so put ours back.
    if (keep[0] <= 0.01f || keep[1] <= 0.01f) {
        Log_Printf("AimSpeed: the game's aim speed was left at %.4f, %.4f - restoring %.4f, %.4f", keep[0], keep[1],
            g_aimSpeedOriginal[0], g_aimSpeedOriginal[1]);
    }
    float target[3];
    for (int i = 0; i < 3; ++i)
        target[i] = g_aimSpeedOriginal[i] * mult;
    AimSpeed_WriteTargets(target, true);
}

// ---- Driving the RATE instead of the stick (2026-09-17) ------------------
// A stick has a dead zone, so a stick can only ask for speeds above some
// minimum, and lifting small requests clear of that zone means the gun can
// never crawl: it moves at the dead zone's worth of speed or not at all, which
// reads as snapping along in little steps. The speeds in this block have no
// threshold anywhere near them. So the stick is held at a fixed push, well
// past the dead zone, where all it carries is DIRECTION, and the speed floats
// carry the rate - smoothly, from a crawl to a flick, with nothing quantised.
//
// Index 0 (+0x2D0) feeds the pitch axis and index 1 (+0x2D4) the yaw, read off
// the integrator: 2D0 lands in [esp+40h], which multiplies the value that ends
// up at [esi+4B8h], and 4B8 is the one the chase proved is pitch.
constexpr float kRateDriveDeflection = 0.6f; // past any plausible dead zone, short of the stops
// 64, not 16 (2026-09-17): the two axes need not be worth the same per unit,
// and if yaw is worth a fraction of pitch a low cap silently limits it - which
// looks exactly like the game capping the turn.
constexpr float kAimSpeedScaleMax = 64.0f;
unsigned long long g_aimRateDriveMs = 0;
// Learned per axis from what the writes actually did: deg/sec per unit of
// scale at the fixed deflection. Both start from the measured 86 at full
// stick. Pitch checks itself against the rig's pitch and yaw against its yaw,
// because assuming the two axes scale alike is what left yaw crawling.
float g_aimRateGain = kGameTurnRateDeg * kRateDriveDeflection;
float g_aimRateGainYaw = kGameTurnRateDeg * kRateDriveDeflection;
// The fastest the gun's bearing has ever been seen to move. If this stops
// climbing while the speed we write keeps going up, the cap is the game's.
float g_servoGunYawFastest = 0.0f;

void AimSpeed_ApplyRates(float pitchScale, float yawScale)
{
    if (!g_haveAimSpeedOriginal)
        AimSpeed_Apply(1.0f); // learn the originals first
    // Never all the way to zero: a zero here is not "hold still", it is an aim
    // that cannot move at all, including the player's own stick, and if
    // anything goes wrong while that value is in place the gun is simply stuck.
    // Holding still is what asking for no rate does; the floor costs nothing.
    constexpr float kAimSpeedScaleMin = 0.02f;
    const auto lim = [kAimSpeedScaleMin](float v) {
        return v < kAimSpeedScaleMin ? kAimSpeedScaleMin : (v > kAimSpeedScaleMax ? kAimSpeedScaleMax : v);
    };
    const float target[3] = { g_aimSpeedOriginal[0] * lim(pitchScale), g_aimSpeedOriginal[1] * lim(yawScale),
        // Left exactly as the game had it (2026-09-17). Forcing this to 1.0
        // looked tidy - it multiplies both axes in a mode we cannot see from
        // here - but if the game's own value is bigger than 1, neutralising it
        // quietly divides the rate we just worked out, and the pitch axis hides
        // that because its trim pushes longer to compensate while yaw, which is
        // fed forward, just comes out slow. The gain we learn absorbs whatever
        // it is worth, as long as we stop moving it.
        g_aimSpeedOriginal[2] };
    AimSpeed_WriteTargets(target, false); // never learn from a value we wrote
    g_aimRateDriveMs = GetTickCount64();
}

// ---- Holding still without deciding what "still" is ----------------------
// A hand never stops moving, so a resting hand shakes the gun, and no fixed
// threshold can separate that from deliberate slow pointing: they overlap in
// speed. They do not overlap in FREQUENCY. Tremor is small and fast, a few
// times a second; pointing is larger and slower. So the filter's strength
// follows the speed of the movement itself - heavy while the hand is idling,
// which holds the gun as still as a stick that has been let go, and almost
// absent while the hand is travelling, which keeps pointing honest.
//
// This is the "one euro" filter (Casiez, Roussel, Vogel), the standard answer
// for exactly this trade in tracked input.
//
// How hard it holds is a taste question, not a correctness one (2026-09-17,
// user: "I don't feel it should be dead still. We want it to feel like someone
// is aiming the gun, there would be slight hand movement"). A locked gun reads
// as a mounted turret; a gun that breathes with the hand holding it reads as a
// person. So the strength is a setting, and the default only removes the part
// of the shimmer that is the sensor rather than the hand.
struct OneEuroFilter {
    float minCutoffHz = 4.5f; // how hard it holds a resting hand: lower holds harder
    float beta = 0.06f;       // how quickly it lets go once the hand moves
    float derivCutoffHz = 1.0f;
    float value = 0.0f, rate = 0.0f;
    bool have = false;

    float Filter(float x, float dt)
    {
        if (!have || dt <= 0.0f) {
            value = x;
            rate = 0.0f;
            have = true;
            return value;
        }
        const auto alphaFor = [dt](float cutoffHz) {
            const float tau = 1.0f / (2.0f * kPi * cutoffHz);
            return 1.0f / (1.0f + tau / dt);
        };
        const float measuredRate = (x - value) / dt;
        rate += alphaFor(derivCutoffHz) * (measuredRate - rate);
        const float cutoff = minCutoffHz + beta * std::fabs(rate);
        value += alphaFor(cutoff) * (x - value);
        return value;
    }
};

// 0 lets the hand through raw, 1 holds the gun still. 6 Hz passes everything a
// wrist actually does; 0.5 Hz passes almost nothing.
float SteadinessToCutoffHz(float steadiness)
{
    const float s = steadiness < 0.0f ? 0.0f : (steadiness > 1.0f ? 1.0f : steadiness);
    return 6.0f - 5.5f * s;
}

// ---- Steadying the head against micro movement (2026-09-18) -------------
// Talking, breathing and a heartbeat each move a headset a fraction of a
// degree, several times a second. Through the lenses that is invisible: the
// picture moves with your skull, so the world stays where it is. Through
// HEAD-FOLLOW it is not, because that vector turns the game's OWN camera -
// so every syllable nudges the culling frustum, the rigs and the character,
// and in the doubling modes it nudges them twice as far as the head moved.
//
// Which is why the steadying goes here and nowhere near the eye matrices.
// Filtering what you SEE against your neck is latency, and latency is what
// makes people ill. Filtering what the GAME DOES about your neck is just a
// camera that does not flinch while you speak.
//
// One Euro rather than a plain low-pass, for the reason it always is: a fixed
// cutoff low enough to hold a resting head still also drags on a real turn,
// and a real turn is 200 deg/sec. One Euro raises its own cutoff with the
// measured rate, so talking - roughly 8 deg/sec at the peak of a 0.3 degree
// wobble - is held hard while a turn goes through untouched.
//
// Yaw and pitch are filtered as ANGLES. Smoothing x, y and z separately
// shortens the vector through every curve, and the shortening reads as a
// pitch error.
constexpr float kHeadSteadyRestHz = 5.0f;   // steadiness just above 0: barely there
constexpr float kHeadSteadyHoldHz = 0.7f;   // steadiness 1: holds hard
constexpr float kHeadSteadyBeta = 0.2f;     // how fast it lets go once you really turn
constexpr float kHeadSteadyJumpDeg = 30.0f; // past this it snapped, it did not move
constexpr float kHeadSteadyGapSec = 0.25f;  // longer and the stored angle means nothing

double QpcMsPerTick()
{
    static double s_scale = 0.0;
    if (s_scale == 0.0) {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        s_scale = freq.QuadPart ? 1000.0 / static_cast<double>(freq.QuadPart) : 0.0;
    }
    return s_scale;
}

struct HeadSteadier {
    OneEuroFilter yaw, pitch;
    float unwrappedYaw = 0.0f;
    long long lastTicks = 0;

    void Reset()
    {
        yaw.have = false;
        pitch.have = false;
    }

    // f is a forward vector in the runtime's frame, steadied in place.
    void Apply(float f[3], float steadiness)
    {
        const float len = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        if (len < 1e-4f)
            return;

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        const float dt = lastTicks
            ? static_cast<float>(static_cast<double>(now.QuadPart - lastTicks) * QpcMsPerTick()) * 0.001f
            : 0.0f;
        lastTicks = now.QuadPart;
        if (dt <= 0.0f || dt > kHeadSteadyGapSec) {
            // Head-follow has been away - aiming, a cutscene, a loading screen
            // - and the angle we stopped at describes somewhere else entirely.
            Reset();
            return;
        }

        const float rawYaw = std::atan2(f[0], f[2]) * 180.0f / kPi;
        float sinPitch = f[1] / len;
        sinPitch = sinPitch < -1.0f ? -1.0f : (sinPitch > 1.0f ? 1.0f : sinPitch);
        const float rawPitch = std::asin(sinPitch) * 180.0f / kPi;

        // Unwrapped, so turning through the back of the play space is one
        // continuous number instead of a 360 degree step the filter would
        // spend half a second easing through.
        if (!yaw.have) {
            unwrappedYaw = rawYaw;
        } else {
            float step = rawYaw - std::fmod(unwrappedYaw, 360.0f);
            while (step > 180.0f)
                step -= 360.0f;
            while (step < -180.0f)
                step += 360.0f;
            unwrappedYaw += step;
        }

        // A jump no neck can make: a recentre, a new area, or the game handing
        // the camera back. Snap to it rather than sweep the camera across the
        // level over the next half second.
        if (yaw.have
            && (std::fabs(unwrappedYaw - yaw.value) > kHeadSteadyJumpDeg
                || std::fabs(rawPitch - pitch.value) > kHeadSteadyJumpDeg)) {
            Reset();
            unwrappedYaw = rawYaw;
        }

        const float s = steadiness < 0.0f ? 0.0f : (steadiness > 1.0f ? 1.0f : steadiness);
        const float cutoff = kHeadSteadyRestHz - (kHeadSteadyRestHz - kHeadSteadyHoldHz) * s;
        yaw.minCutoffHz = cutoff;
        yaw.beta = kHeadSteadyBeta;
        pitch.minCutoffHz = cutoff;
        pitch.beta = kHeadSteadyBeta;

        const float outYaw = yaw.Filter(unwrappedYaw, dt) * kPi / 180.0f;
        const float outPitch = pitch.Filter(rawPitch, dt) * kPi / 180.0f;

        const float cp = std::cos(outPitch);
        f[0] = std::sin(outYaw) * cp;
        f[1] = std::sin(outPitch);
        f[2] = std::cos(outYaw) * cp;
    }
};

// ---- Finding the body's facing (2026-09-17) -----------------------------
// Aiming sideways in RE5 turns the character: the rig's bearing and the body's
// facing differ by exactly 90 degrees in every sample taken. So absolute yaw
// means writing the body's facing, and the root joint's matrix is no good for
// that - it is animation output, several steps downstream of whatever the game
// actually steers.
//
// Rather than another watchpoint chase, this narrows by elimination, the way a
// trainer searches for an unknown value. Every float in the character is
// compared against the facing we can already measure - as radians, as degrees,
// and as its sine and cosine, since a facing is as often stored as a direction
// as an angle. Offsets that match are kept; on the next pass, at a different
// facing, the ones that no longer match are dropped. A few passes of turning
// leaves only the handful that really are the facing.
constexpr int kMaxFacingCandidates = 4096;
struct FacingCandidate {
    DWORD offset;
    int kind; // 0 rad, 1 -rad, 2 deg, 3 -deg, 4 sin, 5 cos, 6 -sin, 7 -cos
};
FacingCandidate g_facingCandidates[kMaxFacingCandidates];
int g_facingCandidateCount = 0;
bool g_facingScanStarted = false;
float g_facingScanLastYaw = 0.0f;
int g_facingScanPasses = 0;

float FacingExpected(int kind, float yawDeg)
{
    const float rad = yawDeg * kPi / 180.0f;
    switch (kind) {
    case 0: return rad;
    case 1: return -rad;
    case 2: return yawDeg;
    case 3: return -yawDeg;
    case 4: return std::sin(rad);
    case 5: return std::cos(rad);
    case 6: return -std::sin(rad);
    default: return -std::cos(rad);
    }
}

bool FacingMatches(int kind, float value, float yawDeg)
{
    const float expected = FacingExpected(kind, yawDeg);
    const float tolerance = (kind == 2 || kind == 3) ? 1.5f : 0.02f;
    return std::fabs(value - expected) <= tolerance;
}

void BodyFacingScan(unsigned char* character, float bodyYawDeg)
{
    constexpr DWORD kScanBytes = 0x3800;
    if (!g_facingScanStarted) {
        g_facingCandidateCount = 0;
        for (DWORD off = 0; off + 4 <= kScanBytes && g_facingCandidateCount < kMaxFacingCandidates; off += 4) {
            float v = 0.0f;
            if (!TryRead(&v, character + off, sizeof(v)))
                continue;
            if (!std::isfinite(v))
                continue;
            for (int kind = 0; kind < 8 && g_facingCandidateCount < kMaxFacingCandidates; ++kind) {
                if (FacingMatches(kind, v, bodyYawDeg)) {
                    g_facingCandidates[g_facingCandidateCount].offset = off;
                    g_facingCandidates[g_facingCandidateCount].kind = kind;
                    ++g_facingCandidateCount;
                }
            }
        }
        g_facingScanStarted = true;
        g_facingScanLastYaw = bodyYawDeg;
        g_facingScanPasses = 1;
        Log_Printf("BodyFacing: first pass at %.1f deg kept %d candidate(s) - now turn, and keep turning",
            bodyYawDeg, g_facingCandidateCount);
        return;
    }

    // Only worth narrowing once the body has actually turned: at the same
    // facing every candidate still matches and nothing is learned.
    float moved = bodyYawDeg - g_facingScanLastYaw;
    while (moved > 180.0f)
        moved -= 360.0f;
    while (moved < -180.0f)
        moved += 360.0f;
    if (std::fabs(moved) < 20.0f)
        return;

    int kept = 0;
    for (int i = 0; i < g_facingCandidateCount; ++i) {
        float v = 0.0f;
        if (!TryRead(&v, character + g_facingCandidates[i].offset, sizeof(v)))
            continue;
        if (FacingMatches(g_facingCandidates[i].kind, v, bodyYawDeg))
            g_facingCandidates[kept++] = g_facingCandidates[i];
    }
    g_facingCandidateCount = kept;
    g_facingScanLastYaw = bodyYawDeg;
    ++g_facingScanPasses;

    static const char* const kKindNames[]
        = { "radians", "-radians", "degrees", "-degrees", "sin", "cos", "-sin", "-cos" };
    Log_Printf("BodyFacing: pass %d at %.1f deg leaves %d candidate(s)", g_facingScanPasses, bodyYawDeg,
        g_facingCandidateCount);
    if (g_facingCandidateCount > 0 && g_facingCandidateCount <= 24) {
        for (int i = 0; i < g_facingCandidateCount; ++i) {
            float v = 0.0f;
            TryRead(&v, character + g_facingCandidates[i].offset, sizeof(v));
            Log_Printf("BodyFacing:   +0x%04lX holds %.4f, which is the facing as %s",
                static_cast<unsigned long>(g_facingCandidates[i].offset), v, kKindNames[g_facingCandidates[i].kind]);
        }
    }
}

// ---- Turning the body to point (2026-09-17) -----------------------------
// The scan narrowed 43 passes down to eight offsets, and they are not eight
// separate values: they are two 4x4 transforms, at character+0x60 and +0xA0,
// each holding the same yaw.
//
//   +0x60 row 0 = [sin, 0, cos]      +0xA0 row 0 = [sin, 0, cos]
//   +0x80 row 2 = [-cos, 0, sin]     +0xC0 row 2 = [-cos, 0, sin]
//
// Rows are 0x10 apart, elements 0 and 2 of each row are the yaw, and the two
// copies agree to four decimal places through every turn - the game keeps a
// pair, so both have to be written or the next frame puts the old one back.
//
// The rotation is applied as a DELTA about the world's vertical rather than
// rebuilt from the desired angle, so whatever scale, lean or tilt the animation
// has put in the matrix survives untouched. Only the heading changes.
// (kOffBodyTransform is declared with the other character offsets above.)
// THIS is the one that decides which way pointing goes: the controller's yaw
// grows to the left (see XrInput_GetGunAim) and the body's grows with
// atan2(x, z), and at -1 pointing right turned the body left.
constexpr float kAbsoluteYawSign = 1.0f;
float g_absoluteYawOffset = 0.0f;
bool g_absoluteYawTied = false;

bool BodyFacingDegrees(unsigned char* character, float* outDeg)
{
    float row0[4] = {};
    if (!TryRead(row0, character + kOffBodyTransform[0], sizeof(row0)))
        return false;
    if (row0[0] * row0[0] + row0[2] * row0[2] < 1e-6f)
        return false;
    *outDeg = std::atan2(row0[0], row0[2]) * 180.0f / kPi;
    return true;
}

// The transforms are not the source either (2026-09-17). Writing them worked
// and never lasted, so the watchpoint went on +0x60 and caught exe+48AAC3,
// which is a quaternion being expanded into a matrix:
//
//   0048AA29: movss xmm0,[ecx+40h]   ; x        the character is in ecx
//   0048AA2E: movss xmm4,[ecx+44h]   ; y
//   0048AA33: movss xmm6,[ecx+48h]   ; z
//   0048AA55: movss xmm2,[ecx+4Ch]   ; w
//   ...       the usual pile of products, doubled
//   0048AABE: movss [ecx+60h],xmm5   ; and out comes the matrix we were writing
//
// So the facing is a quaternion at character+0x40, and the matrix is rebuilt
// from it every frame - which is exactly why our matrix survived the write and
// not the frame. Turning the quaternion turns the character, and the matrix
// follows on its own.
//
// Applied as a delta about the world vertical, by multiplying in a yaw-only
// quaternion, so any pitch or roll the animation has put in is preserved.
// (kOffBodyQuaternion is declared with the other character offsets above.)
// This one is NOT the inversion (2026-09-17). There are two signs in play and
// they do different jobs: this turns a requested delta into a rotation, and the
// other maps a hand's heading to a body's. Flipping this one made the body turn
// away from the target, so the gap grew every frame until it passed the 90
// degree sanity limit and every write stopped - yaw looked dead rather than
// backwards. The body did move before the flip, so this sign was right.
constexpr float kBodyQuatSign = 1.0f;

void TurnBodyBy(unsigned char* character, float deltaDeg)
{
    float q[4] = {}; // x, y, z, w
    if (!TryRead(q, character + kOffBodyQuaternion, sizeof(q)))
        return;
    const float half = deltaDeg * kBodyQuatSign * kPi / 360.0f;
    const float s = std::sin(half), c = std::cos(half);
    // (0, s, 0, c) * q, Hamilton order, so the turn happens in world space
    // rather than in the character's own.
    const float out[4] = {
        c * q[0] + s * q[2],
        c * q[1] + s * q[3],
        c * q[2] - s * q[0],
        c * q[3] - s * q[1],
    };
    TryWrite(character + kOffBodyQuaternion, out, sizeof(out));
}

// The stick the aim servo wants, -1..1, published for the pad the game reads.
// Only used when the direct write below isn't available: the stick is capped
// by the game's own aim rate (86 deg/sec measured, sensitivity at maximum),
// which is less than half a wrist flick.
std::atomic<float> g_aimStickX{ 0.0f };
std::atomic<float> g_aimStickY{ 0.0f };
std::atomic<unsigned long long> g_aimStickMs{ 0 };
// Degrees the wrist has turned that the gun has not turned yet. Camera-hook
// thread only. Cleared whenever the gun comes down, so a debt can't survive an
// aim and snap the gun somewhere on the next one.
float g_aimYawDebtDeg = 0.0f;

// The servo's live numbers for the in-headset tuning readout. Written by the
// camera hook, read by the menu on the present thread; atomics rather than a
// lock because a torn readout is worth nothing worse than one odd frame.
std::atomic<float> g_servoControllerPitch{ 0.0f }, g_servoRigPitch{ 0.0f }, g_servoError{ 0.0f };
std::atomic<float> g_servoWristPitchRate{ 0.0f }, g_servoWristYawRate{ 0.0f };
std::atomic<float> g_servoStickX{ 0.0f }, g_servoStickY{ 0.0f };
std::atomic<float> g_servoMaxRate{ 0.0f }, g_servoFastestSeen{ 0.0f }, g_servoYawDebt{ 0.0f };
std::atomic<float> g_servoGunYawRate{ 0.0f }, g_servoPitchScale{ 0.0f }, g_servoYawScale{ 0.0f };
std::atomic<unsigned long long> g_servoMs{ 0 };

// ---- Setting the aim pitch at its source -------------------------------
// The watchpoint run found one instruction writing the character's pitch,
// exe+7607B5, 240 times in 4 seconds, with the character in both ecx and esi.
// Writing the field anywhere else in the frame is pointless - that
// instruction runs every frame and overwrites it before anything reads it -
// so the hook sits immediately after it and puts our value in instead. The
// camera blend, the weapon and the laser all come off this one number, so
// they stay in agreement, and nothing is rate limited.
bool g_aimWriteHooked = false;
void* g_aimWriteTrampoline = nullptr;

// The aim's own struct and the two normalised angles in it, captured at the
// instruction that writes them. See AimAnglesWritten.
void* g_aimAnglesTrampoline = nullptr;
bool g_aimAnglesHooked = false;
std::atomic<unsigned char*> g_aimStruct{ nullptr };
std::atomic<float> g_aimStructValueYaw{ 0.0f }, g_aimStructValuePitch{ 0.0f };
std::atomic<unsigned long long> g_aimStructMs{ 0 };

// Where the controller says the gun should point, written straight into the
// game's own aim. Degrees, and stale after a quarter second so the game has its
// aim back the moment we stop asking.
std::atomic<float> g_absoluteAimPitchDeg{ 0.0f };
std::atomic<unsigned long long> g_absoluteAimMs{ 0 };
// Measured: the stored pitch hits 1.0000 exactly when the rig reaches -50.1
// degrees, and runs linearly to there.
constexpr float kAimPitchDegPerUnit = 50.1f;

// Yaw's stored number, sampled at the integrator's own rate so it can be
// compared against how far the gun actually turned. A plain accumulator: what
// matters is the average and the extremes over a window, not the order.
struct YawValueWindow {
    std::atomic<float> sum{ 0.0f };
    std::atomic<float> absMax{ 0.0f };
    std::atomic<unsigned> count{ 0 };
    void push(float v)
    {
        sum.store(sum.load(std::memory_order_relaxed) + v, std::memory_order_relaxed);
        const float a = v < 0.0f ? -v : v;
        if (a > absMax.load(std::memory_order_relaxed))
            absMax.store(a, std::memory_order_relaxed);
        count.store(count.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }
    void take(float* outMean, float* outAbsMax, unsigned* outCount)
    {
        const unsigned n = count.exchange(0, std::memory_order_relaxed);
        const float s = sum.exchange(0.0f, std::memory_order_relaxed);
        *outAbsMax = absMax.exchange(0.0f, std::memory_order_relaxed);
        *outCount = n;
        *outMean = n ? s / static_cast<float>(n) : 0.0f;
    }
};
YawValueWindow g_yawValueSamples;
std::atomic<unsigned char*> g_aimWriteCharacter{ nullptr };
std::atomic<float> g_aimWriteValue{ 0.0f };
std::atomic<unsigned long> g_aimWriteOffset{ 0 };
// Point to aim is on and VR is running: the view stops following the gun.
std::atomic<bool> g_pointToAimActive{ false };

// ---- The aim point, set where the game writes it -----------------------
// The gun aims along a ray ending at a world point on the character, about
// 5000 units out (+0x2980). The game writes that point every frame from
// exe+776AA0 - found by watchpoint, 211 writes in 4 seconds, character in esi
// - so anything we put there earlier is gone before it is used. The hook
// below sits immediately after that write.
//
// It is handed the eye the ray starts from and the tangent of the pitch we
// want, rather than a finished height: the bearing and range are the game's
// own and it has just written them, so the height is worked out from what is
// in memory at that instant instead of from last frame's numbers.
bool g_aimPointHooked = false;
void* g_aimPointTrampoline = nullptr;
std::atomic<unsigned char*> g_aimPointCharacter{ nullptr };
std::atomic<float> g_aimPointEyeX{ 0.0f }, g_aimPointEyeY{ 0.0f }, g_aimPointEyeZ{ 0.0f };
std::atomic<float> g_aimPointTan{ 0.0f };
std::atomic<unsigned long long> g_aimPointMs{ 0 };
// Test (VR.MotionAimWriteField=4): force the ray upward, to see what reads it.
std::atomic<bool> g_aimPointForceUp{ false };
// 3DOF through the game.s own mouse: counts waiting to be handed over.
std::atomic<long> g_aimMouseDx{ 0 };
std::atomic<long> g_aimMouseDy{ 0 };
std::atomic<unsigned long long> g_aimMouseMs{ 0 };
std::atomic<unsigned long long> g_aimWriteMs{ 0 };

void AimFromController(unsigned char* controller)
{
    // Says why nothing happened, once a second. A run where the gun didn't
    // move and the log had no aim line at all (2026-09-16) could have been
    // any of these three, and guessing cost a test run.
    const auto bail = [](const char* why) {
        static ULONGLONG s_ms = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_ms > 1000) {
            s_ms = now;
            Log_Printf("AimStop: %s", why);
        }
    };

    const XrInputSettings settings = XrInput_GetSettings();
    if (!settings.enabled || !settings.pointToAim) {
        bail(!settings.enabled ? "motion controllers are switched off" : "Point to aim is switched off");
        return;
    }

    float gunYawDeg = 0.0f, gunPitchDeg = 0.0f;
    if (!XrInput_GetGunAim(&gunYawDeg, &gunPitchDeg)) {
        bail("the gun hand isn't being tracked");
        return;
    }

    float rigPitch = 0.0f;
    if (!RigPitchDegrees(controller, &rigPitch)) {
        bail("the camera rig has no direction to compare against");
        return;
    }
    if (settings.aimWriteField == 0)
        bail("no aim method is selected (VR.MotionAimWriteField is 0)");

    // Developer: find the value the GUN aims by (2026-09-16). +0x2DC8 turned
    // out to drive only the camera rig - the servo tracks it to within half a
    // degree and the gun never moves, while the stick moves the gun - so the
    // weapon's own angle is some other float on the character. This prints a
    // block of them once a second next to the rig pitch; sweeping the stick
    // and reading the log says which offsets follow the gun.
    if (settings.findAimWriter) {
        static ULONGLONG s_scanMs = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_scanMs > 1000) {
            s_scanMs = now;
            unsigned char* character = nullptr;
            if (TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
                // The candidate found by correlating the first scan: three
                // floats at +0x2980 that look like a world point, whose
                // height tracks the gun's pitch at r = 0.99 while the other
                // two don't. If it is the point the gun aims at, its
                // direction from the character matches the rig's pitch - and
                // a point carries yaw as well, which a blend factor cannot.
                float aimPoint[3] = {}, selfPos[3] = {};
                if (TryRead(aimPoint, character + 0x2980, sizeof(aimPoint))
                    && TryRead(selfPos, character + kOffCharacterPos, sizeof(selfPos))) {
                    const float dx = aimPoint[0] - selfPos[0], dy = aimPoint[1] - selfPos[1],
                                dz = aimPoint[2] - selfPos[2];
                    const float flat = std::sqrt(dx * dx + dz * dz);
                    const float pointPitch = flat > 1e-3f ? std::atan2(dy, flat) * 180.0f / kPi : 0.0f;
                    const float pointYaw = std::atan2(-dx, -dz) * 180.0f / kPi;
                    Log_Printf("AimPoint: +0x2980 is (%.1f %.1f %.1f), character at (%.1f %.1f %.1f) -> pitch %.1f "
                               "yaw %.1f, distance %.0f (rig pitch %.1f)",
                        aimPoint[0], aimPoint[1], aimPoint[2], selfPos[0], selfPos[1], selfPos[2], pointPitch,
                        pointYaw, std::sqrt(flat * flat + dy * dy), rigPitch);
                }
                // The watchpoint that used to be armed here, on the aim point's
                // height, is gone: the point turned out to be written at the
                // tail of its routine and read by nothing. The debug registers
                // now go on the pitch blend instead, armed further up on aiming
                // alone so an ordinary pad can trigger it.
                Log_Printf("AimScan: rig pitch %.1f, character %p", rigPitch, character);
                for (DWORD base = 0x28C0; base < 0x2E40; base += 0x20) {
                    float v[8] = {};
                    if (!TryRead(v, character + base, sizeof(v)))
                        continue;
                    // Only lines with something plausibly angular in them, to
                    // keep the log readable.
                    bool interesting = false;
                    for (float f : v) {
                        if (f > -4.0f && f < 4.0f && f != 0.0f)
                            interesting = true;
                    }
                    if (!interesting)
                        continue;
                    Log_Printf("AimScan: +0x%04lX  %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f", base, v[0],
                        v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
                }
            }
        }
    }

    float wanted = gunPitchDeg + settings.aimPitchTrimDeg;
    if (wanted > kAimPitchUpDeg)
        wanted = kAimPitchUpDeg;
    if (wanted < kAimPitchDownDeg)
        wanted = kAimPitchDownDeg;
    const float error = wanted - rigPitch;

    // Absolute pitch: hand the angle itself to the hook on the game's own aim
    // write, which puts the gun exactly there rather than steering towards it.
    // Developer builds only, since that is where the hook lives.
    if (g_aimAnglesHooked) {
        g_absoluteAimPitchDeg.store(wanted, std::memory_order_relaxed);
        g_absoluteAimMs.store(GetTickCount64(), std::memory_order_relaxed);
    }

    // Absolute yaw: turn the body to the heading the controller points at.
    // Aiming sideways in RE5 IS turning the character - the rig's bearing and
    // the body's facing differ by a constant 90 degrees in every sample - so
    // this is the yaw equivalent of writing the pitch, and the same thing the
    // servo was trying to achieve by asking the stick nicely.
    //
    // The play space and the world share no north, so the two are tied together
    // at the moment the gun comes up: from then on the gun points where the
    // hand points, one to one, and raising the gun again re-ties it. That is
    // also the recentre, for when a chair has been turned.
    if (settings.absoluteYaw) {
        unsigned char* character = nullptr;
        float bodyNow = 0.0f;
        if (TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character
            && BodyFacingDegrees(character, &bodyNow)) {
            // Tied on the FIRST frame of the aim, before anything is written,
            // so raising the gun can never move the character. Without this the
            // first frame compared a fresh hand heading against a stale tie and
            // turned the body to match - "pressing grip flips me 180 degrees".
            if (!g_absoluteYawTied) {
                g_absoluteYawOffset = bodyNow - gunYawDeg * kAbsoluteYawSign;
                g_absoluteYawTied = true;
                Log_Printf("AbsoluteYaw: tied to the body at %.1f deg with the hand at %.1f deg", bodyNow, gunYawDeg);
                g_servoMs.store(GetTickCount64(), std::memory_order_relaxed);
                g_aimStickX.store(0.0f, std::memory_order_relaxed);
                g_aimStickY.store(0.0f, std::memory_order_relaxed);
                return;
            }
            float want = gunYawDeg * kAbsoluteYawSign + g_absoluteYawOffset;
            float delta = want - bodyNow;
            while (delta > 180.0f)
                delta -= 360.0f;
            while (delta < -180.0f)
                delta += 360.0f;
            // Anything this big is not a wrist: it is tracking loss, a
            // cutscene, or a tie that has gone stale. Re-tie rather than
            // refusing to move, which is what made yaw look dead once the gap
            // had grown past the limit - and rather than SWINGING there, which
            // is what threw the character 180 degrees at the press of a grip
            // (2026-09-17). Whatever the hand is doing at that moment becomes
            // the new zero, so the worst case is a lost reference, not a spin.
            if (std::fabs(delta) >= 90.0f) {
                g_absoluteYawOffset = bodyNow - gunYawDeg * kAbsoluteYawSign;
                Log_Printf("AbsoluteYaw: the hand was %.0f deg from the body - re-tying rather than swinging there",
                    delta);
                delta = 0.0f;
            }
            const bool asked = std::fabs(delta) > 0.02f;
            if (asked)
                TurnBodyBy(character, delta);

            // Did it take? Read the facing straight back: if it still reads
            // what it did before the write, either the write failed or
            // something puts it back, and those need different answers.
            static ULONGLONG s_logMs = 0;
            const ULONGLONG nowLog = GetTickCount64();
            if (nowLog - s_logMs > 1000) {
                s_logMs = nowLog;
                // No read-back check here any more: it compared the MATRIX,
                // which is only rebuilt from the quaternion on the next frame,
                // so it cried "the write did not take" on every line while the
                // body was in fact following the hand to within a degree. What
                // matters is the gap between where the hand points and where
                // the body faces, so that is what gets logged.
                Log_Printf("AbsoluteYaw: hand %.1f + tie %.1f = want %.1f; body at %.1f, off by %+.1f deg",
                    gunYawDeg, g_absoluteYawOffset, want, bodyNow, bodyNow - want);
            }
        } else {
            static ULONGLONG s_missMs = 0;
            const ULONGLONG nowMiss = GetTickCount64();
            if (nowMiss - s_missMs > 1000) {
                s_missMs = nowMiss;
                Log_Printf("AbsoluteYaw: no character, or its transform could not be read - nothing to turn");
            }
        }
        // Keep the readout alive: it publishes from the stick servo below,
        // which this path skips entirely.
        g_servoControllerPitch.store(wanted, std::memory_order_relaxed);
        g_servoRigPitch.store(rigPitch, std::memory_order_relaxed);
        g_servoError.store(error, std::memory_order_relaxed);
        g_servoMaxRate.store(0.0f, std::memory_order_relaxed);
        g_servoMs.store(GetTickCount64(), std::memory_order_relaxed);
        // Nothing below this point applies: the stick servo exists to chase the
        // heading, and the heading is now simply set.
        g_aimStickX.store(0.0f, std::memory_order_relaxed);
        g_aimStickY.store(0.0f, std::memory_order_relaxed);
        return;
    }

    // With the writer hooked, set the number itself: work out the field value
    // that lands the gun where the controller points, and let the hook put it
    // in right after the game writes its own. Degrees per unit is not a
    // constant - measured between 13 and 51 across the range - so it is
    // learned from what the last frame's change actually did rather than
    // assumed. Converging takes a frame or two, and nothing caps how far one
    // frame may move.
    // The aim point (2026-09-16). Three floats at +0x2980 hold a world point
    // about 5000 units out whose height follows the gun and whose bearing
    // holds still: an aim ray projected to a fixed range. Setting its height
    // aims the gun in one frame, with no ladder, no learned gain and no
    // servo, and the same point carries yaw when we come to it.
    //
    // Only the height is touched here. The bearing is left exactly as the
    // game has it, because our controller's yaw is measured in the play space
    // and the game's is world space, and lining those two frames up is the
    // yaw milestone, not this one.
    g_aimPointForceUp.store(settings.aimWriteField == 4, std::memory_order_relaxed);
    if (settings.aimWriteField == 3 || settings.aimWriteField == 4) {
        unsigned char* character = nullptr;
        if (TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
            float point[3] = {}, eye[3] = {};
            if (TryRead(point, character + 0x2980, sizeof(point)) && TryRead(eye, controller + 0x170, sizeof(eye))) {
                const float dx = point[0] - eye[0], dz = point[2] - eye[2];
                const float flat = std::sqrt(dx * dx + dz * dz);
                if (flat > 1.0f) {
                    float wanted = gunPitchDeg + settings.aimPitchTrimDeg;
                    if (wanted > kAimPitchUpDeg)
                        wanted = kAimPitchUpDeg;
                    if (wanted < kAimPitchDownDeg)
                        wanted = kAimPitchDownDeg;
                    const float tanWanted = std::tan(wanted * kPi / 180.0f);
                    g_aimPointEyeX.store(eye[0], std::memory_order_relaxed);
                    g_aimPointEyeY.store(eye[1], std::memory_order_relaxed);
                    g_aimPointEyeZ.store(eye[2], std::memory_order_relaxed);
                    g_aimPointTan.store(tanWanted, std::memory_order_relaxed);
                    g_aimPointCharacter.store(character, std::memory_order_relaxed);
                    g_aimPointMs.store(GetTickCount64(), std::memory_order_relaxed);

                    static ULONGLONG s_logMs = 0;
                    const ULONGLONG now = GetTickCount64();
                    if (now - s_logMs > 1000) {
                        s_logMs = now;
                        Log_Printf("AimPoint: controller %.1f -> height %.1f (game had %.1f), ray from the eye is "
                                   "%.0f long, rig pitch %.1f%s",
                            wanted, eye[1] + flat * tanWanted, point[1], flat, rigPitch,
                            g_aimPointHooked ? "" : " - NOT HOOKED, the game will overwrite it");
                    }
                }
            }
        }
        g_aimStickY.store(0.0f, std::memory_order_relaxed);
        return;
    }

    // Which field, if any. +0x2DC8 was the first one found and it turned out
    // to be the CAMERA's pitch: writing it tilted the view and never moved
    // the gun, which is backwards for 3DOF (2026-09-16, user: "should only
    // move the gun, never the camera"). Off until the gun's own angle is
    // found; the laser code's neighbouring field at +0x2918 says to look
    // around +0x2900.
    const DWORD aimField = settings.aimWriteField == 1 ? 0x2DC8 : settings.aimWriteField == 2 ? 0x2908 : 0;
    if (g_aimWriteHooked && aimField) {
        unsigned char* character = nullptr;
        if (TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
            float field = 0.0f;
            if (TryRead(&field, character + aimField, sizeof(field))) {
                static float s_degPerUnit = 45.0f;
                static float s_lastField = 0.0f, s_lastPitch = 0.0f;
                static unsigned char* s_lastCharacter = nullptr;
                if (s_lastCharacter == character) {
                    const float dField = field - s_lastField;
                    const float dPitch = rigPitch - s_lastPitch;
                    if (std::fabs(dField) > 0.02f) {
                        // The blend is minus the field, so a positive step
                        // lowers the gun.
                        const float measured = -dPitch / dField;
                        if (measured > 8.0f && measured < 120.0f)
                            s_degPerUnit += (measured - s_degPerUnit) * 0.3f;
                    }
                }

                // Damped, and capped per frame. Asking for the whole error at
                // once threw the gun corner to corner (2026-09-16: controller
                // level, rig pitch swinging +77 to -77 every second), because
                // the feedback is a frame behind and a unit is worth up to 50
                // degrees near the centre. A third of the error per frame,
                // never more than this much of the ladder, still crosses the
                // full range in a few frames - far quicker than the stick's
                // 86 degrees a second.
                constexpr float kAimDamping = 0.35f;
                constexpr float kAimMaxStep = 0.2f;
                float step = -(error / s_degPerUnit) * kAimDamping;
                if (step > kAimMaxStep)
                    step = kAimMaxStep;
                if (step < -kAimMaxStep)
                    step = -kAimMaxStep;
                if (std::fabs(error) < 0.5f)
                    step = 0.0f; // settled: leave the game's own value alone

                float next = field + step;
                if (next > 2.5f)
                    next = 2.5f;
                if (next < -2.5f)
                    next = -2.5f;

                s_lastField = field;
                s_lastPitch = rigPitch;
                s_lastCharacter = character;

                g_aimWriteOffset.store(aimField, std::memory_order_relaxed);
                g_aimWriteValue.store(next, std::memory_order_relaxed);
                g_aimWriteCharacter.store(character, std::memory_order_relaxed);
                g_aimWriteMs.store(GetTickCount64(), std::memory_order_relaxed);

                static ULONGLONG s_logMs = 0;
                const ULONGLONG now = GetTickCount64();
                if (now - s_logMs > 1000) {
                    s_logMs = now;
                    Log_Printf("AimPitch: controller %.1f (trim %.1f) vs rig %.1f -> error %.1f, field %.3f to "
                               "%.3f, %.1f deg per unit (written at the source)",
                        gunPitchDeg, settings.aimPitchTrimDeg, rigPitch, error, field, next, s_degPerUnit);
                }
            }
        }
        g_aimStickY.store(0.0f, std::memory_order_relaxed);
        return;
    }

    // Through the game's own mouse (2026-09-16). Every value we found and
    // wrote turned out to be downstream - +0x2DC8 is one line of a block copy
    // of camera floats, and the aim ray at +0x2980 is written at the tail of
    // its routine and read by nothing, proven by forcing it skyward and
    // watching nothing move. The stick works but tops out at 86 degrees a
    // second. A mouse doesn't: the game turns by however many counts arrive,
    // so the same error becomes mouse movement and the gun can keep up with a
    // wrist. Counts per degree is learned from what the last batch actually
    // did, since it depends on the player's own sensitivity setting.
    if (settings.aimWriteField == 5) {
        static float s_countsPerDeg = 3.0f;
        static float s_lastPitch = 0.0f;
        static long s_lastSent = 0;
        static bool s_have = false;
        if (s_have && s_lastSent != 0) {
            const float moved = rigPitch - s_lastPitch;
            if (std::fabs(moved) > 0.3f) {
                const float measured = static_cast<float>(s_lastSent) / moved;
                // Sign included: which way the game reads mouse Y is its own
                // business, and this learns it rather than assuming. Kept
                // inside a believable range: one wild frame used to teach it
                // a huge number and the next correction then threw the gun
                // across the room (2026-09-16, "camera spaz out on aim").
                if (std::fabs(measured) > 0.5f && std::fabs(measured) < 40.0f)
                    s_countsPerDeg += (measured - s_countsPerDeg) * 0.2f;
                if (s_countsPerDeg > 20.0f)
                    s_countsPerDeg = 20.0f;
                if (s_countsPerDeg < 1.0f)
                    s_countsPerDeg = 1.0f;
            }
        }

        // Yaw, driven by how far the controller TURNED since the last frame
        // rather than by an absolute angle. The controller's yaw is measured
        // in the play space and the game's in the world, and those two never
        // line up; a delta needs no shared reference at all, and a mouse is
        // the one input that takes deltas natively. Pitch stays absolute,
        // since gravity gives both sides the same zero.
        long dx = 0;
        {
            static float s_lastYaw = 0.0f;
            static bool s_haveYaw = false;
            float turned = gunYawDeg - s_lastYaw;
            while (turned > 180.0f)
                turned -= 360.0f;
            while (turned < -180.0f)
                turned += 360.0f;
            s_lastYaw = gunYawDeg;
            // Negated: turning the controller left was moving the gun right
            // (2026-09-16). Yaw also gets its own scale rather than borrowing
            // the pitch loop's, which is learned against a different axis.
            constexpr float kYawCountsPerDeg = -3.0f;
            // A jump this large is tracking loss or a recentre, not a wrist.
            if (s_haveYaw && std::fabs(turned) < 45.0f && std::fabs(turned) > 0.05f)
                dx = static_cast<long>(turned * kYawCountsPerDeg);
            s_haveYaw = true;
        }
        g_aimMouseDx.store(dx, std::memory_order_relaxed);

        // Pitch by how far the controller TILTED since the last frame, the
        // same as yaw above (2026-09-16). It used to chase an absolute angle
        // by feeding back the rig's pitch, which meant our loop and the
        // game's own aim were both steering at once and pulling against each
        // other: "up and down still fights me". A delta cannot fight anything
        // - the gun simply moves as far as the wrist did.
        long dy = 0;
        {
            static float s_lastTilt = 0.0f;
            static bool s_haveTilt = false;
            const float wanted = gunPitchDeg + settings.aimPitchTrimDeg;
            const float tilted = wanted - s_lastTilt;
            s_lastTilt = wanted;
            constexpr float kPitchCountsPerDeg = -3.0f; // mouse Y grows downward
            if (s_haveTilt && std::fabs(tilted) < 45.0f && std::fabs(tilted) > 0.05f)
                dy = static_cast<long>(tilted * kPitchCountsPerDeg);
            s_haveTilt = true;
        }
        s_lastPitch = rigPitch;
        s_lastSent = dy;
        s_have = true;
        g_aimMouseDy.store(dy, std::memory_order_relaxed);
        g_aimMouseMs.store(GetTickCount64(), std::memory_order_relaxed);
        g_aimStickY.store(0.0f, std::memory_order_relaxed);

        static ULONGLONG s_logMs = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_logMs > 1000) {
            s_logMs = now;
            Log_Printf("AimMouse: controller %.1f vs rig %.1f -> error %.1f, sending %ld counts, %.1f counts per "
                       "degree",
                gunPitchDeg, rigPitch, error, dy, s_countsPerDeg);
        }
        return;
    }

    // Fallback: drive the stick instead. The virtual pad is ours already, so
    // the servo asks for a deflection and the game does the rest, at its own
    // pace.
    static float s_appliedY = 0.0f;
    static float s_lastPitch = 0.0f;
    static bool s_haveLast = false;
    static int s_stickSign = 1; // +1 if pushing up raises the gun
    if (s_haveLast && std::fabs(s_appliedY) > 0.25f) {
        const float dPitch = rigPitch - s_lastPitch;
        if (std::fabs(dPitch) > 0.4f) {
            // Which way the game reads its own stick, learned rather than
            // assumed. One disagreeing frame proves nothing (recoil, a step,
            // a scripted nudge all move pitch on their own), so it takes
            // three in a row to flip.
            static int s_wrongInARow = 0;
            const bool agreed = (dPitch > 0.0f) == (s_appliedY > 0.0f);
            s_wrongInARow = agreed ? 0 : s_wrongInARow + 1;
            if (s_wrongInARow >= 3) {
                s_wrongInARow = 0;
                s_stickSign = -s_stickSign;
                Log_Printf("AimPitch: the stick reads the other way round, flipping to %s",
                    s_stickSign > 0 ? "positive up" : "negative up");
            }
        }
    }

    // How fast the game can actually turn the gun right now. 86 deg/sec is what
    // it ships with; the aim speed multiplier scales the floats the integrator
    // reads, so the ceiling moves with it. Everything below asks for a RATE and
    // divides by this, which is what keeps the feel the same at 1x and at 8x.
    // Rate drive sets the speed itself, so the game's 86 deg/sec is no longer
    // the ceiling and must not be used as one (2026-09-17: it still was, with
    // the multiplier at 1x, so a 90 degree flick was clamped to 86 deg/sec and
    // left and right went back to needing exaggerated movement). The ceiling is
    // the player's own maximum instead. Only the old stick-deflection path is
    // still bounded by what full deflection buys.
    const float maxRate = settings.aimRateDrive
        ? settings.aimMaxRateDeg
        : kGameTurnRateDeg * (settings.aimSpeedMult > 1.0f ? settings.aimSpeedMult : 1.0f);

    // Close the remaining error over this long. A rate, not a deflection: the
    // old version asked for full stick whenever it was more than 12 degrees off
    // and nothing at all below one degree, which at 1x was a reasonable ramp
    // because the game could not move fast enough to overshoot. Multiply the
    // game's speed by four and the same code becomes bang-bang - full tilt,
    // arrive, stop dead - which is what "snappy, it just kinda clicked into
    // place, it wasn't very smooth" describes (user, 2026-09-17). Asking for
    // error/tau instead means the request shrinks as the gun arrives, so it
    // eases in at any speed.
    const float kServoTauSec = settings.aimServoMs / 1000.0f;
    constexpr float kDeadbandDeg = 0.3f;

    // How long since the last pass, on the performance counter. Both axes need
    // it now, and GetTickCount64's 15 ms steps are too coarse for a frame.
    // Only measure over a window worth measuring (2026-09-17). This runs from
    // the camera hook, which fires more than once per game frame, and the
    // tracked pose only changes once. The extra passes therefore see no
    // movement across almost no time, and a rate is movement over time: one
    // pass says the hand is dead still, the next divides a whole frame's
    // movement by a fraction of a millisecond and says it is flying. That is
    // where the spikes and dead spots in the log came from - "gun managed 0"
    // next to "gun managed 943" while a hand swept smoothly. Four milliseconds
    // is the same floor the culling lookahead uses for the same reason.
    float dtSec = 0.0f;
    bool freshSample = false;
    {
        static LARGE_INTEGER s_last = {};
        static const LARGE_INTEGER s_freq = [] {
            LARGE_INTEGER f;
            QueryPerformanceFrequency(&f);
            return f;
        }();
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        if (!s_last.QuadPart) {
            s_last = now;
        } else {
            const float d = static_cast<float>(now.QuadPart - s_last.QuadPart) / static_cast<float>(s_freq.QuadPart);
            if (d > 0.2f) {
                s_last = now; // a pause: menu, loading, a cutscene
            } else if (d >= 0.004f) {
                dtSec = d;
                freshSample = true;
                s_last = now;
            }
        }
    }

    // Pitch, rebuilt the way yaw works (2026-09-17, user: it felt jerky "on the
    // up/down too"). It used to be pure feedback: look at how far off the gun
    // is and push the stick by that. Everything it does is therefore a
    // reaction, arriving two or three frames after the wrist moved - our pad
    // value, the game's own integration, then the rig we measure - and a
    // feedback loop with that much delay in it oscillates. That is the same
    // trap the mouse path fell into and got out of by using deltas: "up and
    // down still fights me".
    //
    // So the wrist's own speed is fed forward, which needs no feedback at all
    // and is where nearly all of the motion now comes from, and the absolute
    // error is left to trim out what the feedforward misses. The trim is the
    // only part that can ring, and it is the part doing the least work.
    // The rates both axes want, in degrees a second. Whether those become a
    // stick deflection or a speed written into the game's own block is decided
    // once both are known - see the rate drive below.
    float wantedPitchRate = 0.0f, wantedYawRate = 0.0f;
    float stickY = 0.0f;
    {
        static float s_lastTilt = 0.0f;
        static float s_rate = 0.0f;
        static bool s_have = false;
        static OneEuroFilter s_smooth;
        // Filtered before anything measures a rate from it, so shimmer never
        // enters the loop at all rather than being subtracted out later.
        s_smooth.minCutoffHz = SteadinessToCutoffHz(settings.aimSteadiness);
        const float smoothedPitch = freshSample ? s_smooth.Filter(gunPitchDeg, dtSec) : s_smooth.value;
        const float tilted = smoothedPitch - s_lastTilt;
        if (freshSample)
            s_lastTilt = smoothedPitch;
        if (s_have && freshSample && std::fabs(tilted) < 45.0f) {
            // Half-and-half with the last reading. Dividing a tracked pose by a
            // frame time amplifies every twitch in it, and one noisy frame
            // should not become a jab of stick.
            s_rate += ((tilted / dtSec) - s_rate) * 0.5f;
        } else if (!s_have) {
            s_rate = 0.0f;
        }
        if (freshSample)
            s_have = true;
        g_servoWristPitchRate.store(s_rate, std::memory_order_relaxed);

        const float trim = std::fabs(error) > kDeadbandDeg ? error / kServoTauSec : 0.0f;
        wantedPitchRate = s_rate + trim;
        if (std::fabs(error) <= kDeadbandDeg && std::fabs(s_rate) < 1.0f)
            wantedPitchRate = 0.0f;
        if (wantedPitchRate > maxRate)
            wantedPitchRate = maxRate;
        if (wantedPitchRate < -maxRate)
            wantedPitchRate = -maxRate;
        stickY = (wantedPitchRate / maxRate) * static_cast<float>(s_stickSign);
    }

    // Yaw on the stick. A stick sets a turn RATE, not an angle, so matching it
    // to an absolute controller yaw would need the play space and the world to
    // share a north, which they never do. The first version therefore asked for
    // the rate the wrist was turning at and nothing more - and that can only
    // ever lag. Turn your wrist at 40 degrees a second and it asked for 40,
    // which is less than half deflection, so the gun crawled and the shortfall
    // was simply lost (user, 2026-09-17: "left/right didn't seem to have a full
    // left/full right, and kept things pretty slow").
    //
    // Every degree the wrist turns is now a degree the gun OWES. The rate asked
    // for is the wrist's rate plus what it takes to pay the debt off, so falling
    // behind pushes the stick harder instead of quietly writing the difference
    // off, and a pause lets the gun catch all the way up. The 86 deg/sec
    // ceiling still applies - it is what full deflection buys - but nothing is
    // dropped on the floor beneath it any more.
    float stickX = 0.0f;
    {
        constexpr float kDebtMaxDeg = 120.0f; // a spin is not a debt worth repaying
        // The clock matters here (2026-09-17). This used to time itself with
        // GetTickCount64, whose resolution is about 15.6 ms - and a frame is
        // 14 to 20 ms. So dt came back as 0, 16 or 31 ms for frames that really
        // took the same length of time, and turned/dt, the wrist's speed,
        // swung by a factor of two frame to frame from nothing but clock
        // rounding. The pitch axis never divides by dt, which is why only
        // left and right felt jerky. QPC has no such step.
        static float s_lastYaw = 0.0f;
        static float s_rate = 0.0f;
        static bool s_haveYaw = false;
        static float s_unwrapped = 0.0f; // yaw wraps at 180, and a filter cannot
        static float s_lastRawYaw = 0.0f;
        static bool s_haveRaw = false;
        static OneEuroFilter s_smooth;
        s_smooth.minCutoffHz = SteadinessToCutoffHz(settings.aimSteadiness);
        const float dt = dtSec;
        if (freshSample) {
            if (s_haveRaw) {
                float step = gunYawDeg - s_lastRawYaw;
                while (step > 180.0f)
                    step -= 360.0f;
                while (step < -180.0f)
                    step += 360.0f;
                s_unwrapped += step;
            }
            s_lastRawYaw = gunYawDeg;
            s_haveRaw = true;
        }
        const float smoothedYaw = freshSample ? s_smooth.Filter(s_unwrapped, dtSec) : s_smooth.value;
        const float turned = smoothedYaw - s_lastYaw;
        if (freshSample)
            s_lastYaw = smoothedYaw;
        if (s_haveYaw && freshSample && std::fabs(turned) < 45.0f) {
            // Smoothed the same way as pitch: one twitch in the tracked pose
            // should not become a jab of stick.
            s_rate += ((turned / dt) - s_rate) * 0.5f;
            g_servoWristYawRate.store(s_rate, std::memory_order_relaxed);
            // The wrist's own rate, plus whatever is still owed spread over the
            // same time constant the pitch axis uses. The debt term alone would
            // always sit one tau behind a steady turn; the rate term carries the
            // turn and the debt only pays off what the ceiling swallowed.
            // The gun does not move unless the hand is moving (2026-09-17,
            // user: it tracks correctly and then "incorrectly comes back").
            // Chasing the last degree of error after a movement has ended is
            // never worth it. Some of that error is real overshoot and some is
            // just the gun being a frame behind the hand, we cannot tell which,
            // and correcting either one is a movement the player did not ask
            // for - which is far more noticeable than the error it removes,
            // because the eye is drawn to motion that starts on its own. A real
            // lag, the kind left by a fast whip hitting the ceiling, is still
            // worth catching up; four degrees is the line between the two.
            //
            // No test for the hand being "still" (2026-09-17, user: "I don't
            // know that we ever wanna truly be still, that goes against VR.
            // Someone's hand will always be moving slightly cause no one is
            // truly perfectly still. We just don't want overcompensated
            // movements"). Every threshold tried here failed the same way from
            // one side or the other: high enough to ignore a resting hand also
            // ignored deliberate slow pointing, and low enough to catch slow
            // pointing let a resting hand drift.
            //
            // So nothing is classified. The correction is simply never allowed
            // to be faster than the hand that is being corrected towards. A
            // hand barely moving gets a barely moving correction, a hand
            // sweeping gets a correction that can keep up, and the gun can
            // never set off on a journey of its own while the hand sits there -
            // which is all "overcompensated" ever meant. No thresholds, no
            // states, and it scales itself to however anybody holds a
            // controller.
            constexpr float kCorrectionShare = 0.5f; // of the hand's own speed
            constexpr float kCorrectionFloor = 1.0f; // deg/sec, so it always converges eventually
            const float correctionCap = std::fabs(s_rate) * kCorrectionShare + kCorrectionFloor;
            float correction = g_aimYawDebtDeg / kServoTauSec;
            if (correction > correctionCap)
                correction = correctionCap;
            if (correction < -correctionCap)
                correction = -correctionCap;
            float wanted = s_rate + correction;
            if (wanted > maxRate)
                wanted = maxRate;
            if (wanted < -maxRate)
                wanted = -maxRate;
            wantedYawRate = wanted;
            stickX = -wanted / maxRate;
            // Owed this frame, minus what the gun actually turned in return.
            //
            // This used to work the repayment out from the stick deflection
            // times the ceiling, which was true while deflection WAS the
            // request. Under the rate drive the stick sits at a fixed 0.6 no
            // matter what, so that sum claimed a constant 0.6 x ceiling was
            // being achieved every frame whether the gun moved or not. The debt
            // was therefore always paid off on paper - the log showed it
            // hovering at a degree or two through whole 90 degree whips - so
            // the one mechanism that should have noticed yaw falling behind and
            // pushed harder never fired once (2026-09-17).
            // What the gun actually turned, measured as an ANGLE rather than
            // worked out from a rate (2026-09-17). The rate version was the
            // reason a small cock of the wrist threw the gun half a screen: it
            // repaid the debt with (measured rate x dt), and that measurement
            // is smoothed, lagged and sampled on its own schedule, so it always
            // under-reported what the gun had really done. The debt therefore
            // never closed, the servo kept commanding, and a two degree flick
            // of a wrist turned into fifteen degrees of gun. Comparing where
            // the gun IS against where the hand IS cannot do that: the moment
            // the gun has travelled as far as the hand, the error is zero and
            // the request stops, whatever any rate estimate thinks.
            float gunTurned = 0.0f;
            {
                static float s_prevGunYaw = 0.0f;
                static bool s_havePrevGunYaw = false;
                static float s_signVote = 0.0f;
                float gunYawNow = 0.0f;
                if (RigYawDegrees(controller, &gunYawNow)) {
                    if (s_havePrevGunYaw) {
                        gunTurned = gunYawNow - s_prevGunYaw;
                        while (gunTurned > 180.0f)
                            gunTurned -= 360.0f;
                        while (gunTurned < -180.0f)
                            gunTurned += 360.0f;
                        // Which way the world's bearing runs against the play
                        // space's, voted on rather than assumed: the two need
                        // not share a handedness, and getting it backwards
                        // would make every correction add to the error.
                        if (std::fabs(turned) > 0.2f && std::fabs(gunTurned) > 0.2f) {
                            s_signVote += turned * gunTurned > 0.0f ? 1.0f : -1.0f;
                            s_signVote = s_signVote > 20.0f ? 20.0f : (s_signVote < -20.0f ? -20.0f : s_signVote);
                        }
                        if (s_signVote < 0.0f)
                            gunTurned = -gunTurned;
                    }
                    s_prevGunYaw = gunYawNow;
                    s_havePrevGunYaw = true;
                }
            }
            // Tracking noise must not become debt (2026-09-17, user: small
            // movements "drift and overshoot rather than slowly moving where my
            // vr controller is"). Yaw has no absolute reference - the play
            // space and the world share no north - so the debt is the ONLY
            // thing holding the gun to the hand, and every scrap of jitter in
            // the pose integrates straight into it. A hand held still for ten
            // seconds banks a debt out of nothing, and then spends it.
            if (std::fabs(turned) > 0.02f)
                g_aimYawDebtDeg += turned;
            g_aimYawDebtDeg -= gunTurned;
            // And what does slip through bleeds away rather than accumulating
            // forever. Costs a little lag on a long slow sweep, which is worth
            // far more than a gun that wanders off on its own.
            g_aimYawDebtDeg -= g_aimYawDebtDeg * (dt / 1.5f);
            // No settle band either, for the same reason: it was another
            // threshold deciding when a hand counts as stopped. The capped
            // correction above makes it unnecessary - a small leftover error
            // is worked off at a crawl instead of being spent in one jump.
            if (g_aimYawDebtDeg > kDebtMaxDeg)
                g_aimYawDebtDeg = kDebtMaxDeg;
            if (g_aimYawDebtDeg < -kDebtMaxDeg)
                g_aimYawDebtDeg = -kDebtMaxDeg;
        }
        if (freshSample)
            s_haveYaw = true;
    }

    // The game throws away small stick pushes before it looks at them, and the
    // aim speed multiplier made that bite hard (2026-09-17, user: "my left/right
    // movement have to be very exaggerated to get it to respond"). At 4x, a
    // gentle 30 deg/sec wrist turn only asks for 0.09 of full tilt, which lands
    // inside RE5's own dead zone and does absolutely nothing - and the first
    // push that does clear it arrives already big, which is the other half of
    // the jerkiness. Raising the speed made our requests smaller, so the faster
    // the aim, the more of the useful range disappeared.
    //
    // So anything we actually want is lifted clear of the dead zone, and the
    // rest of the range is squeezed in above it. The size is a setting because
    // it is the game's number, not ours, and a slider finds it faster than
    // another disassembly hunt.
    // Rate drive: the stick says which way, the game's own speed floats say how
    // fast. No dead zone stands between a small request and a small movement,
    // so slow tracking glides instead of stepping along in dead-zone-sized
    // jumps. See AimSpeed_ApplyRates.
    if (settings.aimRateDrive) {
        const float gain = g_aimRateGain > 1.0f ? g_aimRateGain : 1.0f;
        const float gainYaw = g_aimRateGainYaw > 1.0f ? g_aimRateGainYaw : 1.0f;
        float pitchScale = std::fabs(wantedPitchRate) / gain;
        float yawScale = std::fabs(wantedYawRate) / gainYaw;

        // Smoothed, and with the direction held (2026-09-17, user: the rate
        // drive "does make it turn fast...but snappy/jittery instead of
        // smooth"). Two things in this loop chatter frame to frame. The speed
        // we write is recomputed every pass from an estimate of how fast a
        // hand is moving, and the stick's sign flips the instant the wanted
        // rate crosses zero, so a hand that is nearly still makes it slam
        // between full left and full right. Neither is motion anybody asked
        // for. So the written speed is eased rather than jumped, and the
        // direction only changes when the request is decisively the other way.
        static float s_pitchScale = 0.0f, s_yawScale = 0.0f;
        static int s_pitchDir = 0, s_yawDir = 0;
        // Only low enough to stop the stick flapping between left and right
        // when the request is hovering either side of zero. Not a "the hand is
        // still" test - it was 4, which swallowed slow deliberate pointing
        // whole, and the correction cap now handles what that was there for.
        constexpr float kMinCommandedRate = 0.5f;
        constexpr float kScaleEase = 0.35f;
        // Eased on the way UP, dropped at once on the way down (2026-09-17,
        // user: the gun tracks the hand "then seems to doubt itself and moves
        // back a bit"). Easing both ways meant the speed we had written was
        // still in the game's hands for three or four frames after the hand
        // stopped, so the gun carried on past where it should have stopped.
        // The positional error then correctly noticed it had gone too far and
        // walked it back, which is the doubt. Ramping up smoothly is what
        // avoids a jolt at the start of a movement; there is nothing to be
        // gained by coasting at the end of one.
        const auto ease = [kScaleEase](float wanted, float& held) {
            held = wanted > held ? held + (wanted - held) * kScaleEase : wanted;
            return held;
        };
        pitchScale = ease(pitchScale, s_pitchScale);
        yawScale = ease(yawScale, s_yawScale);

        const auto direction = [kMinCommandedRate](float rate, int& held) {
            if (rate > kMinCommandedRate)
                held = 1;
            else if (rate < -kMinCommandedRate)
                held = -1;
            else if (std::fabs(rate) < kMinCommandedRate * 0.5f)
                held = 0;
            return held;
        };
        const int pitchDir = direction(wantedPitchRate, s_pitchDir);
        const int yawDir = direction(wantedYawRate, s_yawDir);
        stickY = pitchDir == 0 ? 0.0f
                               : static_cast<float>(pitchDir) * kRateDriveDeflection * static_cast<float>(s_stickSign);
        stickX = yawDir == 0 ? 0.0f : static_cast<float>(-yawDir) * kRateDriveDeflection;
        AimSpeed_ApplyRates(pitchScale, yawScale);

        // What a unit of scale is really worth, learned from the one axis with
        // feedback. The deflection may not be linear and the player's own
        // sensitivity setting scales everything, so measuring beats assuming.
        static float s_prevPitch = 0.0f;
        static float s_lastScale = 0.0f;
        static bool s_havePrev = false;
        // Learning what a unit of speed is worth is itself a feedback loop -
        // the scale is the wanted rate over the gain, and the gain is measured
        // from what that scale achieved - so it can ring like any other. It
        // only learns from a big, steady push, and never moves more than a
        // tenth of the way in one go.
        if (s_havePrev && freshSample && s_lastScale > 0.05f) {
            const float achieved = std::fabs(rigPitch - s_prevPitch) / dtSec;
            const bool steady = std::fabs(pitchScale - s_lastScale) < s_lastScale * 0.25f;
            if (achieved > 20.0f && steady) {
                const float measured = achieved / s_lastScale;
                if (measured > 5.0f && measured < 4000.0f)
                    g_aimRateGain += (measured - g_aimRateGain) * 0.08f;
            }
        }
        if (freshSample) {
            s_prevPitch = rigPitch;
            s_lastScale = stickY != 0.0f ? pitchScale : 0.0f;
            s_havePrev = true;
        }

        // The same for yaw, against the rig's own bearing. This is also the
        // measurement that says whether the game caps the turn: if the achieved
        // rate stops climbing while the scale keeps going up, something beyond
        // these floats is holding it, and no amount of speed will fix it.
        float rigYaw = 0.0f;
        const bool haveYaw = RigYawDegrees(controller, &rigYaw);
        static float s_prevYaw = 0.0f;
        static float s_lastYawScale = 0.0f;
        static bool s_havePrevYaw = false;
        if (s_havePrevYaw && haveYaw && freshSample && s_lastYawScale > 0.05f) {
            float moved = rigYaw - s_prevYaw;
            while (moved > 180.0f)
                moved -= 360.0f;
            while (moved < -180.0f)
                moved += 360.0f;
            const float achieved = std::fabs(moved) / dtSec;
            g_servoGunYawRate.store(achieved, std::memory_order_relaxed);
            if (achieved > g_servoGunYawFastest && achieved < 2000.0f)
                g_servoGunYawFastest = achieved;
            const bool steady = std::fabs(yawScale - s_lastYawScale) < s_lastYawScale * 0.25f;
            if (achieved > 20.0f && steady) {
                const float measured = achieved / s_lastYawScale;
                if (measured > 5.0f && measured < 4000.0f)
                    g_aimRateGainYaw += (measured - g_aimRateGainYaw) * 0.08f;
            }
        }
        if (freshSample) {
            if (haveYaw)
                s_prevYaw = rigYaw;
            s_lastYawScale = stickX != 0.0f ? yawScale : 0.0f;
            s_havePrevYaw = haveYaw;
        }
        g_servoPitchScale.store(pitchScale, std::memory_order_relaxed);
        g_servoYawScale.store(yawScale, std::memory_order_relaxed);
    }

    const auto clearDeadzone = [&](float s) {
        const float dz = settings.aimStickDeadzone;
        if (settings.aimRateDrive || dz <= 0.0f || s == 0.0f)
            return s; // rate drive already sits clear of the dead zone
        const float mag = std::fabs(s);
        const float lifted = dz + mag * (1.0f - dz);
        return s > 0.0f ? lifted : -lifted;
    };
    stickX = clearDeadzone(stickX);
    stickY = clearDeadzone(stickY);

    s_appliedY = stickY;
    s_lastPitch = rigPitch;
    s_haveLast = true;
    g_aimStickX.store(stickX, std::memory_order_relaxed);
    g_aimStickY.store(stickY, std::memory_order_relaxed);
    g_aimStickMs.store(GetTickCount64(), std::memory_order_relaxed);

    g_servoControllerPitch.store(wanted, std::memory_order_relaxed);
    g_servoRigPitch.store(rigPitch, std::memory_order_relaxed);
    g_servoError.store(error, std::memory_order_relaxed);
    g_servoStickX.store(stickX, std::memory_order_relaxed);
    g_servoStickY.store(stickY, std::memory_order_relaxed);
    g_servoMaxRate.store(maxRate, std::memory_order_relaxed);
    g_servoYawDebt.store(g_aimYawDebtDeg, std::memory_order_relaxed);
    g_servoMs.store(GetTickCount64(), std::memory_order_relaxed);

    // How fast the game will actually turn the gun. This is the number that
    // decides whether pointing can ever feel one to one: the servo can ask
    // for full stick and no more, so the game's own aim rate is the ceiling.
    static ULONGLONG s_rateStartMs = 0;
    static float s_ratePitch = 0.0f;
    static float s_bestRate = 0.0f;
    const ULONGLONG nowMs = GetTickCount64();
    if (std::fabs(stickY) > 0.95f) {
        if (!s_rateStartMs) {
            s_rateStartMs = nowMs;
            s_ratePitch = rigPitch;
        } else if (nowMs - s_rateStartMs >= 250) {
            const float rate = std::fabs(rigPitch - s_ratePitch) * 1000.0f / static_cast<float>(nowMs - s_rateStartMs);
            if (rate > s_bestRate) {
                s_bestRate = rate;
                g_servoFastestSeen.store(s_bestRate, std::memory_order_relaxed);
            }
            s_rateStartMs = nowMs;
            s_ratePitch = rigPitch;
        }
    } else {
        s_rateStartMs = 0;
    }

    static ULONGLONG s_lastLogMs = 0;
    if (nowMs - s_lastLogMs > 1000) {
        s_lastLogMs = nowMs;
        Log_Printf("AimPitch: controller %.1f (trim %.1f) vs rig %.1f -> error %.1f, asking the stick for %.2f "
                   "(up is %s), fastest the game has turned the gun %.0f deg/sec",
            gunPitchDeg, settings.aimPitchTrimDeg, rigPitch, error, stickY, s_stickSign > 0 ? "positive" : "negative",
            s_bestRate);
        // Yaw's own line (2026-09-17). Up and down tracks a hand and left and
        // right will not, so the question is whether the game is refusing to
        // turn faster or we are failing to ask. These four say which: what the
        // wrist did, what we asked for, what we multiplied the game's speed by,
        // and what the gun actually managed.
        Log_Printf("AimYaw: wrist %.0f deg/sec, asked %.0f deg/sec (owed %+.1f deg, ceiling %.0f), speed written "
                   "x%.2f, gun managed %.0f deg/sec (fastest %.0f), a unit of speed is worth %.0f deg/sec",
            g_servoWristYawRate.load(std::memory_order_relaxed), wantedYawRate, g_aimYawDebtDeg, maxRate,
            g_servoYawScale.load(std::memory_order_relaxed), g_servoGunYawRate.load(std::memory_order_relaxed),
            g_servoGunYawFastest, g_aimRateGainYaw);
        // Where the gun's own bearing really lives (2026-09-17). Measuring it
        // off the camera rig gives zeroes half the time, because with the view
        // split the rig carries the HEAD's direction, not the gun's - so a gun
        // that swings while the head holds still registers as no movement at
        // all, and the servo is flying blind on the one axis that needs it.
        // The integrator writes yaw to [esi+4B4h] right beside the pitch at
        // [esi+4B8h], and the block copy puts that pitch on the character at
        // +0x2DC8, so its neighbours are where the yaw should have landed.
        unsigned char* character = nullptr;
        float rigYawNow = 0.0f;
        RigYawDegrees(controller, &rigYawNow);
        if (TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
            // +0x2DC4 turned out to shadow the pitch and +0x2DCC is always
            // zero, so the yaw is not in that block. Two candidates left, both
            // logged together rather than one guess per test run:
            //  * the aim ray at +0x2980, whose bearing was noted as holding
            //    still - but that was while only the pitch was being driven;
            //  * the character's own facing, off the root joint's world matrix,
            //    which must turn during a 90 degree sweep because RE5 turns the
            //    body to aim sideways.
            float point[3] = {}, selfPos[3] = {};
            float pointYaw = 0.0f;
            if (TryRead(point, character + 0x2980, sizeof(point))
                && TryRead(selfPos, character + kOffCharacterPos, sizeof(selfPos))) {
                pointYaw = std::atan2(point[0] - selfPos[0], point[2] - selfPos[2]) * 180.0f / kPi;
            }
            float bodyYaw = 0.0f;
            unsigned char* joints = nullptr;
            if (TryRead(&joints, character + kOffCharacterJoints, sizeof(joints)) && joints) {
                float row0[4] = {};
                if (TryRead(row0, joints + kOffJointWorldMatrix, sizeof(row0)))
                    bodyYaw = std::atan2(row0[0], row0[2]) * 180.0f / kPi;
            }
            Log_Printf("AimYawScan: head-driven rig bearing %.1f, aim ray bearing %.1f, body facing %.1f",
                rigYawNow, pointYaw, bodyYaw);
            // The elimination scan has done its job - it found the two
            // transforms - so the checkbox now arms a watchpoint on one of them
            // instead. The scan stays for the next unknown value worth hunting.
            if (false && joints)
                BodyFacingScan(character, bodyYaw);
        }
        // The aim's own two numbers against the angles they produce. Both are
        // normalised rather than degrees - the integrator divides by the range
        // before storing - so this is the pairing that says what a unit of each
        // is worth, which is what absolute pointing needs in order to work out
        // what to write for "point here".
        if (GetTickCount64() - g_aimStructMs.load(std::memory_order_relaxed) < 250) {
            // Yaw's number against how far the gun's bearing really moved in
            // the same second. If its average tracks the bearing's rate it is a
            // turn speed; if its size tracks the distance turned it is an
            // offset; if neither, it is something else again.
            float yawMean = 0.0f, yawAbsMax = 0.0f;
            unsigned yawCount = 0;
            g_yawValueSamples.take(&yawMean, &yawAbsMax, &yawCount);
            static float s_bearingAtLastLog = 0.0f;
            static ULONGLONG s_bearingLogMs = 0;
            const ULONGLONG nowLog = GetTickCount64();
            float bearingMoved = rigYawNow - s_bearingAtLastLog;
            while (bearingMoved > 180.0f)
                bearingMoved -= 360.0f;
            while (bearingMoved < -180.0f)
                bearingMoved += 360.0f;
            const float sinceSec = s_bearingLogMs ? (nowLog - s_bearingLogMs) / 1000.0f : 0.0f;
            s_bearingAtLastLog = rigYawNow;
            s_bearingLogMs = nowLog;

            Log_Printf("AimRaw: the game's own aim is yaw %.4f, pitch %.4f -> rig pitch %.1f deg, rig bearing %.1f "
                       "deg; the controller is at pitch %.1f, yaw %.1f",
                g_aimStructValueYaw.load(std::memory_order_relaxed),
                g_aimStructValuePitch.load(std::memory_order_relaxed), rigPitch, rigYawNow, gunPitchDeg, gunYawDeg);
            if (sinceSec > 0.1f) {
                Log_Printf("AimYawValue: over %.1f s the stored yaw averaged %.3f (biggest %.3f, %u samples) while "
                           "the gun's bearing moved %+.1f deg, which is %+.0f deg/sec",
                    sinceSec, yawMean, yawAbsMax, yawCount, bearingMoved, bearingMoved / sinceSec);
            }
        } else if (g_aimAnglesHooked) {
            Log_Printf("AimRaw: the aim angle capture is installed but has not fired - the integrator has not run");
        }
    }
}

void AimWalk(unsigned char* controller)
{
    if (!kMoveWhileAiming || !IsPlayerController(controller))
        return;
    // First person is what this was built for, but third person is how you
    // SEE it: whether the legs walk or the character slides is not a question
    // you can answer from inside your own head (2026-09-19).
    if (!g_enabled && !g_aimWalkThirdPerson.load(std::memory_order_relaxed))
        return;

    static LARGE_INTEGER s_freq = {}, s_last = {};
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    if (!s_freq.QuadPart)
        QueryPerformanceFrequency(&s_freq);
    float dt = s_last.QuadPart ? static_cast<float>(now.QuadPart - s_last.QuadPart) / static_cast<float>(s_freq.QuadPart) : 0.0f;
    s_last = now;
    if (dt > 0.1f)
        dt = 0.1f;

    unsigned char aim = 0;
    if (!TryRead(&aim, controller + kOffControllerAimFlag, sizeof(aim)) || aim != 1)
        return;

    const auto held = [](int vk) { return (GetAsyncKeyState(vk) & 0x8000) ? 1.0f : 0.0f; };
    float forward = held('W') - held('S');
    float strafe = held('D') - held('A');
    float speedScale = 1.0f; // keys are full speed; the stick is analog
    if (forward == 0.0f && strafe == 0.0f) {
        if (!ReadLeftStick(&forward, &strafe))
            return;
        speedScale = std::sqrt(forward * forward + strafe * strafe);
    }

    float fx = g_camForward[0], fz = g_camForward[2];
    float rx = g_camRight[0], rz = g_camRight[2];
    const float fl = std::sqrt(fx * fx + fz * fz), rl = std::sqrt(rx * rx + rz * rz);
    if (fl < 1e-3f || rl < 1e-3f)
        return;
    fx /= fl; fz /= fl; rx /= rl; rz /= rl;
    // View-only (2026-09-15): the drawn camera follows the head, so walking
    // "forward" while aiming went where you looked. The user wants walking to
    // always follow Chris' body forward - the game camera - aiming or not. Walk relative to the
    // game camera - the gun's direction - instead: turn both drawn axes by the
    // yaw from the drawn forward to the controller's own eye->target
    // (+0x170 -> +0x190), which needs no knowledge of the world's handedness.
    if (g_aimViewTest.load(std::memory_order_relaxed)) {
        float eye[3], target[3];
        if (TryRead(eye, controller + 0x170, sizeof(eye)) && TryRead(target, controller + 0x190, sizeof(target))) {
            float gx = target[0] - eye[0], gz = target[2] - eye[2];
            const float gl = std::sqrt(gx * gx + gz * gz);
            if (gl > 1e-3f) {
                gx /= gl;
                gz /= gl;
                const float c = fx * gx + fz * gz;
                const float s = fx * gz - fz * gx;
                const float nfx = c * fx - s * fz, nfz = s * fx + c * fz;
                const float nrx = c * rx - s * rz, nrz = s * rx + c * rz;
                fx = nfx; fz = nfz; rx = nrx; rz = nrz;
            }
        }
    }
    float dx = forward * fx + strafe * rx;
    float dz = forward * fz + strafe * rz;
    const float dl = std::sqrt(dx * dx + dz * dz);
    if (dl < 1e-3f)
        return;
    dx /= dl;
    dz /= dl;

    unsigned char* character = nullptr;
    if (!TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) || !character)
        return;
    float* pos = reinterpret_cast<float*>(character + kOffCharacterPos);
    pos[0] += dx * kAimWalkSpeed * speedScale * dt;
    pos[2] += dz * kAimWalkSpeed * speedScale * dt;

    // Commit the step the way the game's own movement code does after a
    // root-motion step (re5dx9.exe+87A69B / +87ACF8): mark the character as
    // moved (character+0x2D7C |= 0x10000000 - all exe+75C350 does with that
    // flag), then run the handler on character+0x2F30 (exe+864B40). The bare
    // position write never reached the co-op partner: they saw you frozen
    // while aiming, then a teleport when aiming ended (2026-09-11). Whether
    // these are what sends the move online is untested - needs a co-op session.
    const int commit = g_aimWalkCommit.load(std::memory_order_relaxed);
    bool commitNow = commit != kCommitOff;
    if (commitNow) {
        // Paced off the performance counter. GetTickCount64 moves in steps of
        // about 15.6 ms, which is coarser than the interval being timed.
        const float hz = g_aimWalkCommitHz.load(std::memory_order_relaxed);
        if (hz > 0.0f) {
            static LARGE_INTEGER freq = {};
            if (!freq.QuadPart)
                QueryPerformanceFrequency(&freq);
            LARGE_INTEGER nowTick;
            QueryPerformanceCounter(&nowTick);
            const double nowSec = static_cast<double>(nowTick.QuadPart) / static_cast<double>(freq.QuadPart);
            static double lastSec = 0.0;
            if (nowSec - lastSec < 1.0 / hz)
                commitNow = false;
            else
                lastSec = nowSec;
        }
    }
    if (commitNow) {
        DWORD* const flags = reinterpret_cast<DWORD*>(character + kOffCharacterStepFlags);
        const DWORD wasFlags = *flags;
        if (commit != kCommitHandler)
            *flags |= kStepMovedFlag;
        if (commit != kCommitFlag) {
            void* handler = nullptr;
            if (TryRead(&handler, character + kOffCharacterStepHandler, sizeof(handler)) && handler) {
                typedef void(__fastcall * StepHandler_t)(void* self, void* edxUnused);
                const auto update = reinterpret_cast<StepHandler_t>(
                    reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr)) + kRvaStepHandlerUpdate);
                update(handler, nullptr);
            }
        }
        // Put the bit back the way it was found, and ONLY the bit we set: the
        // handler may have changed other flags for reasons of its own and those
        // are not ours to undo. If the handler already clears it, this does
        // nothing, which is the outcome that tells us the weapon was reading
        // something else entirely.
        if (commit == kCommitTidy && !(wasFlags & kStepMovedFlag))
            *flags &= ~kStepMovedFlag;
    }
}

// Low-pass the rig-space eye position - see g_vrStabiliseEye. Kept per
// controller so a partner's rig can't drag the player's filter around, and
// reset when a controller has been away long enough that continuing to
// smooth from its old value would slide the camera across the level.
void StabiliseEye(const unsigned char* controller, float* eyeNormal, float* eyeAim)
{
    struct Smoothed {
        const unsigned char* owner;
        unsigned long long lastMs;
        float normal[3];
        float aim[3];
    };
    static Smoothed s_state[4] = {};

    const unsigned long long now = GetTickCount64();
    Smoothed* slot = nullptr;
    for (Smoothed& s : s_state) {
        if (s.owner == controller) {
            slot = &s;
            break;
        }
    }
    if (!slot) {
        for (Smoothed& s : s_state) {
            if (!s.owner || now - s.lastMs > 2000) {
                slot = &s;
                slot->owner = nullptr; // force the re-seed below
                break;
            }
        }
    }
    if (!slot)
        return; // all four busy - leave the position untouched rather than guess

    const unsigned long long sinceMs = now - slot->lastMs;
    const bool reseed = slot->owner != controller || sinceMs > 250;
    slot->owner = controller;
    slot->lastMs = now;
    if (reseed) {
        std::memcpy(slot->normal, eyeNormal, sizeof(slot->normal));
        std::memcpy(slot->aim, eyeAim, sizeof(slot->aim));
        return;
    }

    const float dt = static_cast<float>(sinceMs) * 0.001f;
    // Frame-rate independent exponential smoothing: at dt >> tau this
    // approaches 1 and the filter simply follows, which is what we want after
    // a hitch.
    float alpha = 1.0f - std::exp(-dt / kEyeSmoothTimeConstantSec);
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    for (int i = 0; i < 3; ++i) {
        slot->normal[i] += (eyeNormal[i] - slot->normal[i]) * alpha;
        slot->aim[i] += (eyeAim[i] - slot->aim[i]) * alpha;
        eyeNormal[i] = slot->normal[i];
        eyeAim[i] = slot->aim[i];
    }
}

// Is head-follow actually 1:1? The user reports having to turn slowly for it
// to feel right, which means something downstream is easing toward the
// direction we write - we set it instantly. This measures two candidates at
// once, without guessing:
//
//   * how often the camera code runs. If OnRigsReady fires at 30 Hz while the
//     head moves at 90, tracking is steppy no matter how correct the value.
//   * how far the rendered camera's yaw trails the head's yaw. Both are in
//     different frames, so the OFFSET is meaningless - the swing in that
//     offset while turning is the lag. Standing still it should barely move;
//     if it opens up during a fast turn and closes again afterwards, the
//     camera is being smoothed and we go find the smoothing.
void MeasureHeadFollowLag()
{
    // Compare how FAST each one turns, not how far apart they point. The two
    // yaws live in different frames, so their difference carries a constant
    // offset and wraps at +-180 - the first version of this measured that
    // wrap and reported ~358 deg every window, standing still included. Turn
    // rates need no shared frame: if the camera's peak rate is well under the
    // head's, something is rate-limiting or smoothing it, and the ratio says
    // how much. A single writer isn't guaranteed here (the camera code runs
    // ~180x/sec and evidently re-enters), so the window is guarded.
    static volatile LONG s_busy = 0;
    static unsigned long long s_windowStartMs = 0;
    static unsigned long long s_lastSampleMs = 0;
    static unsigned int s_calls = 0;
    static float s_prevHeadYaw = 0.0f;
    static float s_prevCamYaw = 0.0f;
    static bool s_havePrev = false;
    static float s_peakHeadRate = 0.0f;
    static float s_peakCamRate = 0.0f;

    if (InterlockedCompareExchange(&s_busy, 1, 0) != 0)
        return;

    ++s_calls;

    const int front = g_headForwardFront.load(std::memory_order_acquire);
    const float headYaw = std::atan2(g_headForward[front][0], g_headForward[front][2]);
    const float camYaw = std::atan2(g_camForward[0], g_camForward[2]);
    const unsigned long long now = GetTickCount64();

    if (s_havePrev && now > s_lastSampleMs) {
        const float dt = static_cast<float>(now - s_lastSampleMs) * 0.001f;
        auto wrapped = [](float a) {
            while (a > kPi)
                a -= 2.0f * kPi;
            while (a < -kPi)
                a += 2.0f * kPi;
            return a;
        };
        // Accumulated rotation, not peak rate. Peaks were useless: the camera
        // yaw is decoded from the cached shader constant matrix, which
        // belongs to whichever pass ran last, so sampling it at ~165 Hz
        // manufactures spikes and the camera "turned" up to 4x faster than
        // the head. Summing absolute change over three seconds lets those
        // cancel while real turning adds up, so the ratio means something.
        const float headStep = std::fabs(wrapped(headYaw - s_prevHeadYaw)) * 180.0f / kPi;
        const float camStep = std::fabs(wrapped(camYaw - s_prevCamYaw)) * 180.0f / kPi;
        (void)dt;
        if (headStep < 45.0f && camStep < 45.0f) { // skip cuts and teleports
            s_peakHeadRate += headStep;
            s_peakCamRate += camStep;
        }
    }
    if (!s_havePrev || now > s_lastSampleMs) {
        s_prevHeadYaw = headYaw;
        s_prevCamYaw = camYaw;
        s_lastSampleMs = now;
        s_havePrev = true;
    }

    if (s_windowStartMs == 0)
        s_windowStartMs = now;
    const unsigned long long elapsed = now - s_windowStartMs;
    if (elapsed >= 3000) {
        const float ratio = s_peakHeadRate > 1.0f ? s_peakCamRate / s_peakHeadRate : 1.0f;
        Log_Printf("CameraRigHook: head-follow - camera updated %.0f times/sec | rotation this window: head %.0f deg, "
                   "camera %.0f deg (camera is %.0f%% of head)",
            s_calls * 1000.0 / static_cast<double>(elapsed), s_peakHeadRate, s_peakCamRate, ratio * 100.0f);
        s_windowStartMs = now;
        s_calls = 0;
        s_peakHeadRate = 0.0f;
        s_peakCamRate = 0.0f;
    }

    InterlockedExchange(&s_busy, 0);
}

extern "C" void CameraRigHook_OnRigsReady(unsigned char* controller)
{
    // Ticking 'Find the arms' searches again (2026-09-25, user: "sheva is in
    // the logs with arm scan" - and she was not).
    //
    // The search only runs while the arms are UNKNOWN, so turning the setting
    // on after they have been found does nothing at all and says nothing about
    // doing nothing. The user ticked it, waited, and got an empty log, which is
    // the worst way for a diagnostic to fail. A checkbox called 'Find the arms'
    // should find the arms.
    {
        static bool s_findArmsWas = false;
        const bool findNow = XrInput_GetSettings().findArms;
        if (findNow && !s_findArmsWas) {
            Log_Printf("Arms: 'Find the arms' just came on - letting go of the arms so the search runs again "
                       "and prints the whole bone table");
            CameraRigHook_ForgetArms();
            ArmIk_ForgetSkeleton();
        }
        s_findArmsWas = findNow;
    }
    // Keeps the live list, the F4 before-write samples and the F5 boom
    // finder working exactly as they did with the legacy hooks.
    RecordHit(controller + kOffNormalMiddle, /*aim=*/false);
    RecordHit(controller + kOffAimMiddle, /*aim=*/true);

    float eyeNormal[3] = { g_targetHorizontalDistance, g_targetVerticalDistance, g_targetDistance };
    float eyeAim[3] = { g_targetHorizontalDistance, g_targetVerticalDistance, g_targetDistance };
    const bool vrActive = g_vrActive.load(std::memory_order_relaxed);
    UpdateHead(controller, vrActive, eyeNormal, eyeAim);
    if (vrActive && g_vrStabiliseEye.load(std::memory_order_relaxed))
        StabiliseEye(controller, eyeNormal, eyeAim);
    // Arm IK runs whether or not the first-person camera is on (2026-09-17),
    // so the write test can be watched from the third-person camera where the
    // arms are plainly in view. Everything below this point is the camera.
    //
    // Against g_playerController rather than IsPlayerController, which reads a
    // flag UpdateHead only reaches while first person is on. The latch itself
    // still needs first person once - which character is you is worked out from
    // the camera sitting inside its head, and there is no other signal for it
    // yet - but once it has happened the pointer holds, so the arms can be
    // watched from any camera. Without this the whole block was silently doing
    // nothing in third person, which is exactly how it was first tested.
    {
        const bool isPlayer = controller == g_playerController;
        // Somebody else on screen - the co-op partner. Their skeleton is what
        // your hands stop against, and this hook walks past it every frame.
        // "Not the player" has to mean it (2026-09-24, user: "we're still
        // having an issue of accidentally driving each other's character
        // model", and "we may be having an issue when transporting levels and
        // breaking a few things on our personal character IK/aiming").
        //
        // Those are one bug. isPlayer is `controller == g_playerController`,
        // and that latch is only made once first person has run. Change level
        // and every pointer moves, so until it re-latches YOUR OWN controller
        // reads as somebody else - and everything below adopts it as the co-op
        // partner and drives your character with your partner's hands. Your
        // own arms and aim break at the same moment, for the same reason.
        //
        // Pointer equality on the controller is too thin a thread to hang that
        // on. The character and the skeleton are checked as well, and while
        // nobody is known to be the player, nobody is adopted at all.
        bool couldBeMe = !g_playerController;
        if (!isPlayer && g_playerController) {
            unsigned char* mine = nullptr;
            unsigned char* theirs = nullptr;
            if (TryRead(&mine, g_playerController + kOffControllerCharacter,
                    sizeof(mine))
                && TryRead(&theirs, controller + kOffControllerCharacter, sizeof(theirs)) && mine && theirs
                && mine == theirs)
                couldBeMe = true;
        }
        if (!isPlayer && couldBeMe) {
            static unsigned long long s_toldMine = 0;
            const unsigned long long nowMine = GetTickCount64();
            if (nowMine - s_toldMine >= 3000) {
                s_toldMine = nowMine;
                Log_Printf("ArmIk: a controller that is not the player points at %s - not adopting it",
                    g_playerController ? "the player's own character" : "anything, nobody is the player yet");
            }
        }
        if (!isPlayer && !couldBeMe) {
            unsigned char* otherJoints = JointArray(controller);
            // Nor the skeleton we are already driving as our own.
            if (otherJoints && otherJoints == g_lastPlayerJoints)
                otherJoints = nullptr;
            if (otherJoints)
                ArmIk_NoteOtherSkeleton(otherJoints, kMaxJoints);

            // And as a body to DRIVE, not just to bump into (2026-09-20).
            // Their arms are posed from hands that arrive over the network, so
            // the solver needs exactly what it needs for the player: the
            // character, the joints, and which joints are the arms. FindArms
            // works by shape, so it reads Sheva as readily as Chris, and it is
            // cached per character because walking a whole skeleton is not a
            // per-frame job.
            unsigned char* otherCharacter = nullptr;
            if (otherJoints
                && TryRead(&otherCharacter, controller + kOffControllerCharacter, sizeof(otherCharacter))
                && otherCharacter) {
                // There has to BE a partner (2026-09-23, user: "Mercenaries
                // does NOT always have a partner. you can go solo"). The lock
                // below keeps the mod from swapping partners every few frames,
                // but it still adopts SOMEBODY, and playing solo that somebody
                // is whichever Majini walked past first. Then a Majini's arms
                // are solved as though they were a player's, on a body built
                // nothing like one - which is what stood behind the aiming
                // bug: "following character 0C162F20 as the co-op partner" at
                // 06:45:27, while the same log says "no partner yet" and the
                // partner is "NOT being published".
                //
                // An arm is driven only for someone whose hands are actually
                // arriving over the network. Bumping into other characters is
                // decided further up and is not touched by this: you can still
                // put your hand on anybody.
                // Sticky, because a gap in the post is not a missing person
                // (2026-09-24). This test was added to stop the mod adopting a
                // Majini when nobody is there, and it does that job - but it
                // treated one late packet as nobody being there, threw the
                // partner away, and re-adopted a moment later. A tester's log
                // shows it flapping four times in seven seconds, and the body
                // does not survive that, so their arms were never driven once
                // while twenty packets a second were arriving and being used.
                //
                // Hands arriving still decide whether there IS a partner. They
                // just no longer have to keep arriving every single frame to
                // keep one.
                {
                    static unsigned long long s_handsSeenMs = 0;
                    IkSyncHands liveHands;
                    const unsigned long long nowHands = GetTickCount64();
                    if (IkSync_GetPartnerHands(liveHands))
                        s_handsSeenMs = nowHands;
                    if (!s_handsSeenMs || nowHands - s_handsSeenMs > 5000)
                        otherCharacter = nullptr;
                }

                // Lock on to ONE partner (2026-09-20).
                //
                // "Not the player" is not the same as "the co-op partner": it
                // matches every other character in the scene, Majini included.
                // So the partner body was being republished as whoever passed
                // through last, the tie to their skeleton was cleared every few
                // frames, and the solve ran on whatever body happened to be
                // current. That is what the yes-no-yes-no on "Driving their
                // arms" was, and why it went wrong the moment enemies turned up.
                //
                // Whoever is adopted first is kept, and only given up after two
                // seconds of not being seen - long enough that a reload or a
                // moment out of view does not hand the slot to a Majini.
                static unsigned char* s_partnerLock = nullptr;
                static unsigned long long s_partnerLockMs = 0;
                const unsigned long long nowLock = GetTickCount64();
                // A new level is a new everything (2026-09-24). Carrying a
                // partner across one means holding a pointer into memory the
                // game has freed and reused, which is the worst kind of stale:
                // it still reads, and what it reads is somebody else.
                {
                    static const void* s_lockedToPlayer = nullptr;
                    if (s_lockedToPlayer != g_playerController) {
                        s_lockedToPlayer = g_playerController;
                        if (s_partnerLock) {
                            s_partnerLock = nullptr;
                            Log_Printf("ArmIk: the player changed - letting the old partner go");
                        }
                    }
                }
                if (!otherCharacter) {
                    if (s_partnerLock) {
                        s_partnerLock = nullptr;
                        Log_Printf("ArmIk: nobody's hands are arriving - not following a partner");
                    }
                } else if (s_partnerLock == otherCharacter) {
                    s_partnerLockMs = nowLock;
                } else if (!s_partnerLock || nowLock - s_partnerLockMs > 2000) {
                    s_partnerLock = otherCharacter;
                    s_partnerLockMs = nowLock;
                    Log_Printf("ArmIk: following character %p as the co-op partner", otherCharacter);
                } else {
                    otherCharacter = nullptr; // somebody else on screen; not ours to drive
                }
            }
            if (otherJoints && otherCharacter) {
                static unsigned char* s_partnerSeen = nullptr;
                static unsigned char* s_partnerSeenJoints = nullptr;
                static ArmChains s_partnerArms;
                // Per skeleton, like the player's: the partner changes costume too.
                // The partner's arms get the same two checks the player's have
                // (2026-09-24). Measured at the very end of the frame, with
                // nothing half-built, the partner's arm came back 83.8 units
                // one frame and 29.7 the next against a resting 53.1 - and
                // then the resting length itself moved to 64.4. None of those
                // is a stretched arm: the sum of two rigid bones is the same
                // in every pose, and 29.7 is shorter than the arm can be. Both
                // numbers say the three joints being read are not a shoulder,
                // an elbow and a wrist on one chain.
                //
                // This cache only ever compared POINTERS, and a skeleton can
                // be rebuilt at the same address with the bones in new places
                // - a costume finishing its load is the usual way. The player
                // learned that on 2026-09-22 and got a links check; the
                // partner never did, so it carried on reading joints 46, 50
                // and 54 of something else entirely.
                static unsigned int s_partnerLinks[6] = {};
                static float s_partnerArmUnits = 0.0f;
                bool findPartnerArms = otherCharacter != s_partnerSeen || otherJoints != s_partnerSeenJoints;
                const char* whyFind = "a partner we have not seen before";

                // Rebuilt in place: the joints are still there, they are just
                // somebody else's now. Id and parent only - the other bytes
                // are state and change all the time.
                if (!findPartnerArms && s_partnerArms.valid) {
                    const int ids[6] = { s_partnerArms.left.shoulder, s_partnerArms.left.elbow,
                        s_partnerArms.left.wrist, s_partnerArms.right.shoulder, s_partnerArms.right.elbow,
                        s_partnerArms.right.wrist };
                    for (int k = 0; k < 6; ++k) {
                        unsigned int nowLink = 0;
                        if (ids[k] < 0
                            || !TryRead(&nowLink, otherJoints + ids[k] * kJointStride + kOffJointLinks,
                                sizeof(nowLink)))
                            continue;
                        if ((nowLink & 0xFF00FF00u) != (s_partnerLinks[k] & 0xFF00FF00u)) {
                            findPartnerArms = true;
                            whyFind = "their skeleton was rebuilt where it stood";
                            break;
                        }
                    }
                }

                // And a check the player does not have, because the player has
                // never needed it: is this chain even rigid? Bones do not
                // change length, so if the two of them stop adding up to what
                // they added up to when we found them, these are not the two
                // bones we found. This catches a wrong guess that the links
                // check cannot, because the links are perfectly valid - they
                // just belong to a chain that is not an arm.
                if (!findPartnerArms && s_partnerArms.valid && s_partnerArmUnits > 1.0f) {
                    float sP[3], eP[3], wP[3];
                    if (JointWorldPos(otherJoints, s_partnerArms.right.shoulder, sP)
                        && JointWorldPos(otherJoints, s_partnerArms.right.elbow, eP)
                        && JointWorldPos(otherJoints, s_partnerArms.right.wrist, wP)) {
                        const float uv[3] = { eP[0] - sP[0], eP[1] - sP[1], eP[2] - sP[2] };
                        const float fv[3] = { wP[0] - eP[0], wP[1] - eP[1], wP[2] - eP[2] };
                        const float nowUnits = Length3(uv) + Length3(fv);
                        static int s_wrongRun = 0;
                        if (std::fabs(nowUnits - s_partnerArmUnits) > s_partnerArmUnits * 0.15f) {
                            // Several in a row, so one frame read mid-update
                            // does not throw away a chain that is fine.
                            if (++s_wrongRun > 30) {
                                s_wrongRun = 0;
                                findPartnerArms = true;
                                whyFind = "their arm stopped being a fixed length, so it was never their arm";
                            }
                        } else {
                            s_wrongRun = 0;
                        }
                    }
                }

                if (findPartnerArms) {
                    s_partnerSeen = otherCharacter;
                    s_partnerSeenJoints = otherJoints;
                    s_partnerArms = FindArms(otherJoints, otherCharacter);
                    s_partnerArmUnits = 0.0f;
                    std::memset(s_partnerLinks, 0, sizeof(s_partnerLinks));
                    if (s_partnerArms.valid) {
                        const int ids[6] = { s_partnerArms.left.shoulder, s_partnerArms.left.elbow,
                            s_partnerArms.left.wrist, s_partnerArms.right.shoulder, s_partnerArms.right.elbow,
                            s_partnerArms.right.wrist };
                        for (int k = 0; k < 6; ++k) {
                            if (ids[k] >= 0)
                                TryRead(&s_partnerLinks[k], otherJoints + ids[k] * kJointStride + kOffJointLinks,
                                    sizeof(unsigned int));
                        }
                        float sP[3], eP[3], wP[3];
                        if (JointWorldPos(otherJoints, s_partnerArms.right.shoulder, sP)
                            && JointWorldPos(otherJoints, s_partnerArms.right.elbow, eP)
                            && JointWorldPos(otherJoints, s_partnerArms.right.wrist, wP)) {
                            const float uv[3] = { eP[0] - sP[0], eP[1] - sP[1], eP[2] - sP[2] };
                            const float fv[3] = { wP[0] - eP[0], wP[1] - eP[1], wP[2] - eP[2] };
                            s_partnerArmUnits = Length3(uv) + Length3(fv);
                        }
                    }
                    Log_Printf("ArmIk: the partner's arms %s - %s; joints %d %d %d on the right, arm %.1f units",
                        s_partnerArms.valid ? "were found" : "could not be made out on this skeleton", whyFind,
                        s_partnerArms.right.shoulder, s_partnerArms.right.elbow, s_partnerArms.right.wrist,
                        s_partnerArmUnits);
                }
                if (s_partnerArms.valid) {
                    // Re-note the collision skeleton with its REAL length. The
                    // call above has to guess at kMaxJoints because it runs
                    // before the character is read, and a hand stopping against
                    // joints that are not there is the same fault as solving
                    // them, just quieter.
                    ArmIk_NoteOtherSkeleton(otherJoints, s_partnerArms.count);
                    ArmIkBody partner = {};
                    partner.joints = otherJoints;
                    partner.character = otherCharacter;
                    // The count FindArms measured, not the ceiling (2026-09-20).
                    // This was kMaxJoints, which told the solver the array was
                    // far bigger than it is - so it walked off the end of a
                    // 54-joint skeleton into whatever memory followed and wrote
                    // rotations into it. On the partner's screen that came out
                    // as the character's geometry stretched across the level,
                    // and the log gave it away by naming joints 79 and 105.
                    partner.jointCount = s_partnerArms.count;
                    partner.head = s_partnerArms.head;
                    partner.left.shoulder = s_partnerArms.left.shoulder;
                    partner.left.elbow = s_partnerArms.left.elbow;
                    partner.left.wrist = s_partnerArms.left.wrist;
                    partner.left.valid = s_partnerArms.left.valid;
                    partner.right.shoulder = s_partnerArms.right.shoulder;
                    partner.right.elbow = s_partnerArms.right.elbow;
                    partner.right.wrist = s_partnerArms.right.wrist;
                    partner.right.valid = s_partnerArms.right.valid;
                    unsigned char partnerAim = 0;
                    partner.aiming
                        = TryRead(&partnerAim, controller + kOffControllerAimFlag, sizeof(partnerAim))
                        && partnerAim == 1;
                    ArmIk_SetPartnerBody(partner);
                    // And solved here, now, rather than waiting for the
                    // player's own body to be figured out. On a machine where
                    // the player's arms or head are never identified - which
                    // is silent, because nothing reports a head it did not
                    // find - the partner used to be dropped along with them.
                    ArmIk_SolvePartnerNow();
                }
            }
        }
#if RE5VR_DIAGNOSTICS
        const XrInputSettings waitSettings = XrInput_GetSettings();
        if (!g_playerController
            && (waitSettings.armIk || waitSettings.armIkTest || waitSettings.findArmWriter)) {
            static unsigned long long s_toldMs = 0;
            const unsigned long long nowMs = GetTickCount64();
            if (nowMs - s_toldMs >= 3000) {
                s_toldMs = nowMs;
                Log_Printf("ArmIk: waiting - nobody is the player yet. Turn first person on once so the "
                           "camera sits inside a head; after that it holds whichever camera you use.");
            }
        }
#endif
        // What the arms are, and how long. Found once per character and kept:
        // the search walks the whole skeleton, which is not something to do every
        // frame, and the indices do not change while the character lives. Arm IK
        // then runs off the cache each frame.
        if (isPlayer) {
            const XrInputSettings armSettings = XrInput_GetSettings();
            // Arm sharing counts as wanting arms, even with 6DOF off
            // (2026-09-20). This gate decides whether the player's body gets
            // built at all, and the partner's solve is driven from there - so
            // with it off, a flatscreen player received hands perfectly and
            // then had nowhere to run the solve. They do not want their OWN
            // arms driven, and they still will not be: that is gated separately
            // inside the solve. They want to watch a VR partner's arms move,
            // which is the whole point of the feature for somebody on a pad.
            const bool wantArms = armSettings.findArms || armSettings.measureGrips || armSettings.measureShake
                || armSettings.findGunBones
                || armSettings.armIk
                || armSettings.armIkTest
                || armSettings.findArmWriter || IkSync_GetSettings().enabled;
            static unsigned char* s_reported = nullptr;
            // The skeleton the cached arms belong to (2026-09-22). A costume
            // change keeps the character and swaps the skeleton under it - a
            // different joint count, different indices - and the cache was keyed
            // on the character alone, so the old costume's joint numbers kept
            // being read and written into the new array, past its end. That
            // was the crash on changing costume in VR.
            static unsigned char* s_reportedJoints = nullptr;
            static ArmChains s_arms;
            // Each arm joint's own id and parent, as they were when found. A
            // costume can finish loading seconds after the swap and rebuild the
            // skeleton at the SAME address with the bones in new places, which
            // no pointer comparison can see. The joint's own links can.
            static unsigned int s_armLinks[6] = {};
            if (g_forgetArms.exchange(false, std::memory_order_relaxed)) {
                Log_Printf("Arms: asked to look again - dropping the cached arm joints");
                s_reported = nullptr;
                s_reportedJoints = nullptr;
                s_arms.valid = false;
                ArmIk_ForgetSkeleton();
            }
            unsigned char* character = nullptr;
            unsigned char* joints = nullptr;
            const bool haveBody = wantArms
                && TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character
                && TryRead(&joints, character + kOffCharacterJoints, sizeof(joints)) && joints;
            // A search that found nothing is not an answer (2026-09-23, user:
            // "my arm IK broke while aiming at one point after the cutscene
            // that held me in place and required restarting the chapter").
            // The result was cached either way, keyed on this character and
            // this skeleton - so one look at a skeleton the game was still
            // putting together switched the arms off until the level was
            // reloaded, because nothing would ever look again. Only a search
            // that found arms is kept now. A failed one is looked at again
            // four times a second, which is often enough to be invisible and
            // rare enough that the walk over every joint costs nothing.
            static unsigned long long s_lastArmSearchMs = 0;
            const unsigned long long armSearchNow = GetTickCount64();
            const bool newSkeleton = haveBody && (character != s_reported || joints != s_reportedJoints);
            const bool retrySearch = haveBody && !newSkeleton && !s_arms.valid
                && armSearchNow - s_lastArmSearchMs >= 250;
            if (newSkeleton || retrySearch) {
                s_lastArmSearchMs = armSearchNow;
                const ArmChains arms = FindArms(joints, character);
                if (newSkeleton && character == s_reported)
                    Log_Printf("Arms: same character, new skeleton (a costume change?) - finding the arms again");
                if (newSkeleton && !arms.valid)
                    Log_Printf("Arms: no arms on this skeleton yet - looking again until there are");
                if (retrySearch && arms.valid)
                    Log_Printf("Arms: found them on a later look - the arms are back");
                s_reported = character;
                s_reportedJoints = joints;
                {
                    const int ids[6] = { arms.left.shoulder, arms.left.elbow, arms.left.wrist, arms.right.shoulder,
                        arms.right.elbow, arms.right.wrist };
                    for (int k = 0; k < 6; ++k) {
                        s_armLinks[k] = 0;
                        if (ids[k] >= 0)
                            TryRead(&s_armLinks[k], joints + ids[k] * kJointStride + kOffJointLinks, sizeof(unsigned int));
                    }
                }
                s_arms = arms;
                if (!armSettings.findArms) {
                    // Cached without a word. The report is the developer finder: a
                    // wrong guess at these indices is worth catching before
                    // anything is written, but only when someone is looking.
                } else if (!arms.valid) {
                    Log_Printf("Arms: could not make out two arms on this skeleton");
                } else {
                    const auto report = [&](const char* side, const ArmChain& arm) {
                        float s[3], e[3], w[3];
                        if (!JointWorldPos(joints, arm.shoulder, s) || !JointWorldPos(joints, arm.elbow, e)
                            || !JointWorldPos(joints, arm.wrist, w))
                            return;
                        const float upperV[3] = { e[0] - s[0], e[1] - s[1], e[2] - s[2] };
                        const float foreV[3] = { w[0] - e[0], w[1] - e[1], w[2] - e[2] };
                        const float upper = Length3(upperV);
                        const float fore = Length3(foreV);
                        Log_Printf("Arms: %s - shoulder joint %d, elbow %d, wrist %d; upper arm %.1f units, forearm "
                                   "%.1f, reach %.1f",
                            side, arm.shoulder, arm.elbow, arm.wrist, upper, fore, upper + fore);
                    };
                    report("right", arms.right);
                    report("left", arms.left);
                }
            }
            if (haveBody && s_arms.valid && s_reported == character && s_reportedJoints == joints) {
                const int ids[6] = { s_arms.left.shoulder, s_arms.left.elbow, s_arms.left.wrist, s_arms.right.shoulder,
                    s_arms.right.elbow, s_arms.right.wrist };
                for (int k = 0; k < 6; ++k) {
                    unsigned int now = 0;
                    if (ids[k] < 0 || !TryRead(&now, joints + ids[k] * kJointStride + kOffJointLinks, sizeof(now)))
                        continue;
                    // Parent (byte 1) and id (byte 3) only; the other bytes may be state.
                    if ((now & 0xFF00FF00u) != (s_armLinks[k] & 0xFF00FF00u)) {
                        Log_Printf("Arms: the skeleton was rebuilt in place - joint %d was %08X, now %08X - finding "
                                   "the arms again",
                            ids[k], s_armLinks[k], now);
                        s_reported = nullptr;
                        s_arms.valid = false;
                        ArmIk_ForgetSkeleton();
                        break;
                    }
                }
            }
#if RE5VR_DIAGNOSTICS
            // The whole chain in one line, because every link in it can fail
            // silently and each one has cost a round trip tonight.
            if (IkSync_GetSettings().enabled) {
                static unsigned long long s_toldChain = 0;
                const unsigned long long nowChain = GetTickCount64();
                if (nowChain - s_toldChain >= 2000) {
                    s_toldChain = nowChain;
                    ArmIkPartnerStatus ps = {};
                    ArmIk_GetPartnerStatus(ps);
                    Log_Printf("ArmShare: your body %s, your arms %s, your head %s (joint %d) - their body %s, "
                               "their arms %s, their hands %s, their scale %s, tied %s, solving %s",
                        haveBody ? "yes" : "NO", s_arms.valid ? "yes" : "NO",
                        s_arms.head >= 0 ? "yes" : "NO", s_arms.head, ps.haveBody ? "yes" : "NO",
                        ps.armsFound ? "yes" : "NO", ps.handsFresh ? "yes" : "NO", ps.haveScale ? "yes" : "NO",
                        ps.tied ? "yes" : "NO", ps.solving ? "yes" : "NO");
                }
            }
#endif
            if (haveBody && s_arms.valid && s_arms.head >= 0 && s_reported == character
                && s_reportedJoints == joints) {
                ArmIkBody ikBody;
                ikBody.joints = joints;
                ikBody.character = character;
                ikBody.jointCount = s_arms.count;
                ikBody.head = s_arms.head;
                ikBody.left.shoulder = s_arms.left.shoulder;
                ikBody.left.elbow = s_arms.left.elbow;
                ikBody.left.wrist = s_arms.left.wrist;
                ikBody.left.valid = s_arms.left.valid;
                ikBody.right.shoulder = s_arms.right.shoulder;
                ikBody.right.elbow = s_arms.right.elbow;
                ikBody.right.wrist = s_arms.right.wrist;
                ikBody.right.valid = s_arms.right.valid;
                unsigned char aimNow = 0;
                ikBody.aiming = TryRead(&aimNow, controller + kOffControllerAimFlag, sizeof(aimNow))
                    && aimNow == 1;
                ArmIk_SetBody(ikBody);
                ArmIk_LeanSpine(); // does nothing unless the body is set to lean with you
                ArmIk_SteadyGun(); // does nothing unless 'Steady the gun when firing' is ticked
            }
        }
    }

    // Called BEFORE the first-person early-out, so it can also run in third
    // person when asked. It has its own gate now.
    AimWalk(controller);
    // The co-op arm sharing, on the game's own thread. Every Steam call it
    // makes happens here rather than on a worker, so two threads are never in
    // that interface at once.
    IkSync_Pump();
    if (!g_enabled)
        return;
    // Compositor only needs the wide cone whatever the checkbox says: with the
    // camera no longer following the head, the game's own FOV hides everything
    // to the side the moment you look at it.
    const bool wideFov = g_vrWideFov.load(std::memory_order_relaxed) ||
        StereoTest_GetPictureTurnMode() == kPictureTurnModeCompositorOnly;
    const float fov = vrActive && wideFov ? VrCullVerticalFov() : FirstPersonVerticalFov();

    // Every direction is read from the game's untouched rigs before any of
    // them is rewritten - the normal group may borrow the aim group's.
    float aimDy[3], aimDz[3];
    bool aimOk[3];
    for (int i = 0; i < 3; ++i)
        aimOk[i] = RigViewDirection(controller + kOffAimRigs + i * kRigStride, &aimDy[i], &aimDz[i]);

    // Head-follow, but only while the gun is down. Pointing the game's camera
    // where you are looking is what fills the culled-away world back in - it
    // also makes you aim with your head, which the user tested and disliked
    // ("a shame"). Aiming is exactly when you don't need it: the gun is up,
    // you are looking down the laser, and the camera already points where the
    // shot goes. So follow the head while free-looking, and hand aim straight
    // back the moment the aim flag comes on - the same instant the game
    // switches rig sets anyway, so the change rides an existing transition.
    unsigned char aimFlag = 0;
    const bool aiming = TryRead(&aimFlag, controller + kOffControllerAimFlag, sizeof(aimFlag)) && aimFlag == 1;
    // Only YOUR camera follows your head (2026-09-13). This hook runs for every
    // camera controller the game updates, and Sheva's goes through it too -
    // both showed up in the same session (controllers 09B9D6C0 and 09BE4020).
    // Head-follow had no player check, so her camera was being steered by the
    // headset as well, the measurement below counted both (~240 calls/sec
    // instead of ~165, in exactly the windows the user felt as jitter), and
    // g_headFollowDriving was overwritten by whichever controller ran last.
    // Not yet proven to be the jitter; wrong regardless.
    const bool isPlayer = IsPlayerController(controller);
    if (isPlayer) {
        // Separating the gun from the view only makes sense when something
        // other than the stick is aiming (user, 2026-09-16). With a pad the
        // stick is the only aim there is, so the view should follow it, the
        // way it always has. So: VR, pointing turned on, and a controller
        // actually in hand.
        const XrInputSettings motion = XrInput_GetSettings();
        float gunYaw = 0.0f, gunPitch = 0.0f;
        g_pointToAimActive.store(
            vrActive && motion.enabled && motion.pointToAim && XrInput_GetGunAim(&gunYaw, &gunPitch),
            std::memory_order_relaxed);
    }

    // 3DOF aiming (2026-09-16): the gun's pitch follows where the controller
    // points. What the probe run established:
    //  * the camera's pitch blend at [controller+0x1D0] is exactly minus one
    //    of two fields on the character, +0x2DC8 or +0x2908, and which one it
    //    mirrors changes with the weapon state - so pick whichever matches;
    //  * +0x2DC8 saturates at +-0.997 while +0x2908 runs to +-2.5, and the
    //    blend reaches +-2.5, so the fields are a ladder across rig presets,
    //    not an angle. Degrees per unit is not constant: measured between 13
    //    and 51 depending on where in the range you are.
    // That rules out a formula. Instead this closes the loop on the game's
    // own number: read the pitch the rig came out at, compare it with the
    // controller's, and nudge the field. It converges in a few frames, and
    // the gain re-learns itself from what the last nudge actually did, so a
    // different weapon or character needs no new constant.
    // Finding the aim rate (2026-09-17). The stick tops out at 86 deg/sec and
    // the servo can only ask for full deflection, so the ceiling is the game's,
    // not ours. Raising it means finding the instruction that turns stick input
    // into aim angle and patching what it multiplies by. The pitch blend at
    // +0x1D0 is the one live float known to follow the gun's pitch exactly, so
    // a write watchpoint on it lands inside that arithmetic - and this arms on
    // aiming alone, with no VR and no motion controllers, so it can be caught
    // with an ordinary pad on the desktop.
    //
    // The first run (2026-09-17) answered the first link: exe+8455E7 writes the
    // blend as a constant minus the character's own pitch, either +0x2DC8 or
    // +0x2908 depending on the weapon state, which is the ladder we already
    // knew about. So the chase carries on from there, one link per window:
    //   1. who writes the character's pitch - known to be exe+7607B5, which
    //      copies it from [edx+0x4B8], so this window is really here to catch
    //      edx, the struct the value actually lives in;
    //   2. who writes THAT, which is the arithmetic turning stick into angle.
    // Raising the game's own aim speed. Applied whether or not VR or motion
    // controllers are in play: this is the game's ceiling, and it caps a pad
    // just as hard. Kept running for one more frame after it goes back to 1 so
    // the originals are restored rather than left scaled.
    if (isPlayer) {
        const float mult = XrInput_GetSettings().aimSpeedMult;
        static bool s_wasScaling = false;
        // Not while the servo is setting the speeds itself: it owns the block
        // for as long as the gun is up, and a flat multiplier written over the
        // top would undo the rate it just worked out.
        const bool servoOwnsIt = GetTickCount64() - g_aimRateDriveMs < 200;
        // Unconditionally, whenever the servo is not driving (2026-09-17, user:
        // "you broke something I can no longer control the gun at all"). This
        // used to run only when the multiplier was above 1, which was fine
        // while a multiplier was the only thing that ever touched the block.
        // The rate drive writes it too, and writes a scale of nearly zero when
        // it wants the gun to hold still - so with the multiplier at 1.0x,
        // lowering the gun left the game's own aim speed sitting at zero with
        // nothing to put it back, and the pad stopped turning the gun at all.
        if (!servoOwnsIt) {
            AimSpeed_Apply(mult);
            s_wasScaling = mult > 1.001f;
        }
        (void)s_wasScaling;
    }

    // Who owns the body's facing (2026-09-17). Writing the transform at
    // character+0x60 provably works - the value reads back exactly as written -
    // and provably does not last: the next second it is back to the same
    // -151.9 it held before, every time. So that transform is another copy,
    // like the character's pitch at +0x2DC8 was, and the thing that rebuilds it
    // every frame is what actually has to be steered.

    if (isPlayer && aiming && XrInput_GetSettings().findBodyFacing) {
        static bool s_armed = false;
        unsigned char* character = nullptr;
        if (!s_armed && TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character) {
            s_armed = true;
            AimFinder_Start(character + 0x60, "the body's facing (+0x60)");
        }
    }

    if (isPlayer && aiming && XrInput_GetSettings().findAimWriter) {
        static int s_stage = 0;
        static int s_settle = 0;
        unsigned char* character = nullptr;
        TryRead(&character, controller + kOffControllerCharacter, sizeof(character));
        if (s_stage == 0 && character) {
            s_stage = 1;
            AimFinder_Start(character + 0x2DC8, "the character's aim pitch (+0x2DC8)");
        } else if (s_stage == 1) {
            AimFinderHit hit{};
            // Two polls, so the window's own report reaches the log before the
            // next one clears it.
            if (AimFinder_BusiestHit(hit) && ++s_settle >= 2) {
                const uintptr_t base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
                unsigned char* source = reinterpret_cast<unsigned char*>(hit.edx) + 0x4B8;
                if (hit.edx > 0x10000) {
                    Log_Printf("AimChase: exe+%lX copied the pitch %lu times with edx=%08lX, so the value itself "
                               "lives at %p ([edx+0x4B8]) - watching that next",
                        static_cast<unsigned long>(hit.eip - base), hit.count, hit.edx, source);
                    AimFinder_Rearm();
                    s_stage = 2;
                    AimFinder_Start(source, "the pitch at its source ([edx+0x4B8])");
                } else {
                    Log_Printf("AimChase: exe+%lX wrote the pitch but edx=%08lX is not a pointer, so the copy came "
                               "from somewhere else - stopping here",
                        static_cast<unsigned long>(hit.eip - base), hit.edx);
                    s_stage = 3;
                }
            }
        }
    }

    if (isPlayer && vrActive) {
        if (aiming) {
            AimFromController(controller);
        } else {
            // The gun being down is itself a reason for nothing happening,
            // and it is the one the log could never show before.
            // Let go of the stick the instant the gun comes down, rather than
            // leaving the last push to go stale on its own.
            g_aimStickX.store(0.0f, std::memory_order_relaxed);
            g_aimStickY.store(0.0f, std::memory_order_relaxed);
            g_aimMouseDx.store(0, std::memory_order_relaxed);
            g_aimMouseDy.store(0, std::memory_order_relaxed);
            g_aimYawDebtDeg = 0.0f;
            // The gun coming down is what re-ties absolute yaw: the next time
            // it goes up, wherever the hand is pointing becomes wherever the
            // body faces.
            g_absoluteYawTied = false;

            static ULONGLONG s_ms = 0;
            const ULONGLONG now = GetTickCount64();
            const XrInputSettings motion = XrInput_GetSettings();
            if (motion.enabled && motion.pointToAim && now - s_ms > 3000) {
                s_ms = now;
                Log_Printf("AimStop: the game doesn't think the gun is up (its aim flag is off)");
            }
        }
    }

    // View split (2026-09-15): the head never touches the game's rigs.
    // The renderer and culling get the head view at GetViewMatrix instead, so
    // the game camera - and aiming, which faces it - stays the game's own.
    // Steering the rigs with the head made Chris turn toward where you looked
    // every time you pressed aim, and the game camera glide between head and
    // gun at every aim start and stop.
    // Picture turning 4 (compositor only) is the one mode where the camera must
    // NOT follow the head: the whole point is a single rotation source, so the
    // runtime's reprojection has nothing to disagree with. See g_pictureTurnMode.
    const bool headWanted = vrActive && isPlayer && g_headFollow.load(std::memory_order_relaxed) &&
        StereoTest_GetPictureTurnMode() != kPictureTurnModeCompositorOnly;
    const bool aimViewTest = headWanted && g_aimViewTest.load(std::memory_order_relaxed);
    const bool headFollow = headWanted && !aiming && !aimViewTest;
    if (isPlayer)
        g_headFollowDriving.store(headFollow, std::memory_order_release);
    const int hfBack = 1 - (g_hfTargetsFront.load(std::memory_order_relaxed) == 1 ? 1 : 0);
    HeadFollowTargets& hf = g_hfTargets[hfBack];
    const int avBack = 1 - (g_aimViewFront.load(std::memory_order_relaxed) == 1 ? 1 : 0);
    AimViewTargets& av = g_aimView[avBack];
    if (headFollow || aimViewTest) {
        // Which head pose steers the camera this update.
        //  - Double (v0.4.1): the pose LATCHED for this frame - the same one
        //    the eye matrices add on top. Reading the live pose there made a
        //    fast turn overshoot and snap back (2026-09-12): two snapshots of
        //    one head, moments apart, reconverging.
        //  - Game camera only (2026-09-15): the camera is the only thing that
        //    turns, so there is nothing to disagree with - take the NEWEST
        //    pose, and tag the frame with its id at Present so the compositor
        //    corrects from the pose the camera really used. The latched pose
        //    is taken at the previous Present, and the game updates its camera
        //    on its own schedule, so steering from it and tagging with the
        //    next latch plausibly made every frame claim to be newer than it
        //    was - the "unresponsive" the user felt.
        bool haveHead = false;
        if (StereoTest_GetPictureTurnMode() == kPictureTurnModeCameraOnly) {
            // One read, views and id together (2026-09-16). This used to
            // fetch the id, then the views, then the id again, and accept the
            // pair only if the id hadn't moved - a retry loop around exactly
            // the race that VRBridge_GetEyeViews now closes by returning both
            // under one lock.
            XRBridgeEyeView l, r;
            XRBridgePoseId id = 0;
            if (VRBridge_GetEyeViews(l, r, &id) && id != 0) {
                hf.headForward[0] = l.rotationDelta[6];
                hf.headForward[1] = l.rotationDelta[7];
                hf.headForward[2] = l.rotationDelta[8];
                hf.poseId = id;
                haveHead = true;
            }
        }
        if (!haveHead) {
            float latched[3];
            if (StereoTest_GetLatchedHeadForward(latched)) {
                std::memcpy(hf.headForward, latched, sizeof(latched));
                hf.poseId = StereoTest_GetLatchedPoseId();
            } else {
                const int front = g_headForwardFront.load(std::memory_order_acquire);
                std::memcpy(hf.headForward, g_headForward[front], sizeof(hf.headForward));
                hf.poseId = 0;
            }
        }
        // Steadied on the way to the camera, and only on that copy:
        // hf.headForward stays the raw pose hf.poseId names, so the frame tag
        // and stereo_test's "how far has the head moved on since" stay true.
        // The doubling modes then double a steadied aim rather than a shaky
        // one, which is where the shake was worst.
        float steadyForward[3];
        std::memcpy(steadyForward, hf.headForward, sizeof(steadyForward));
        {
            const float steadiness = g_vrHeadSteady.load(std::memory_order_relaxed);
            // The rig update re-enters (see MeasureHeadFollowLag). Two threads
            // inside one filter produce a value that is neither of theirs, so
            // the loser leaves the pose raw for that one update.
            static HeadSteadier s_steadier;
            static volatile LONG s_steadyBusy = 0;
            if (steadiness > 0.0f && InterlockedCompareExchange(&s_steadyBusy, 1, 0) == 0) {
                s_steadier.Apply(steadyForward, steadiness);
                InterlockedExchange(&s_steadyBusy, 0);
            }
        }
        if (StereoTest_GetPictureTurnMode() == kPictureTurnModeDoubleFixed)
            DoubleHeadAim(steadyForward, hf.cameraForward);
        else
            std::memcpy(hf.cameraForward, steadyForward, sizeof(hf.cameraForward));

    }
    for (int i = 0; i < 3; ++i) {
        unsigned char* normalRig = controller + kOffNormalRigs + i * kRigStride;
        float baseDy = 0.0f, baseDz = 1.0f;
        if (kNormalUsesAimPitchRange && aimOk[i]) {
            baseDy = aimDy[i];
            baseDz = aimDz[i];
        } else if (!RigViewDirection(normalRig, &baseDy, &baseDz)) {
            baseDy = 0.0f;
            baseDz = 1.0f;
        }

        float nx = 0.0f, ny = baseDy, nz = baseDz;
        if (headFollow) {
            ApplyHeadFollow(hf.cameraForward, &nx, &ny, &nz);
            RigDirToWorld(controller, kOffNormalTransform, nx, ny, nz, hf.worldDir[i]);
        } else if (aimViewTest) {
            float vx = nx, vy = ny, vz = nz;
            ApplyHeadFollow(hf.cameraForward, &vx, &vy, &vz);
            RigDirToWorld(controller, kOffNormalTransform, vx, vy, vz, hf.worldDir[i]);
        }
        PlaceRigAtEye(normalRig, eyeNormal, nx, ny, nz, fov);

        float ax = 0.0f;
        float ay = aimOk[i] ? aimDy[i] : baseDy;
        float az = aimOk[i] ? aimDz[i] : baseDz;
        if (headFollow) {
            ApplyHeadFollow(hf.cameraForward, &ax, &ay, &az);
            RigDirToWorld(controller, kOffAimTransform, ax, ay, az, hf.worldDir[3 + i]);
        } else if (aimViewTest) {
            // The rigs keep the gun's direction; only record where the view
            // would point, for the main-camera hook and stereo_test's match.
            float vx = ax, vy = ay, vz = az;
            ApplyHeadFollow(hf.cameraForward, &vx, &vy, &vz);
            RigDirToWorld(controller, kOffAimTransform, ax, ay, az, av.gunDir[i]);
            RigDirToWorld(controller, kOffAimTransform, vx, vy, vz, av.viewDir[i]);
            std::memcpy(hf.worldDir[3 + i], av.viewDir[i], sizeof(av.viewDir[i]));
        }
        PlaceRigAtEye(controller + kOffAimRigs + i * kRigStride, eyeAim, ax, ay, az, fov);
    }
    if (aimViewTest) {
        g_aimViewFront.store(avBack, std::memory_order_release);
        g_aimViewMs.store(GetTickCount64(), std::memory_order_release);
    }
    if (headFollow || aimViewTest) {
        // The single view direction the renderer should use: the head-driven
        // rig directions of the set in play (aim while aiming, else normal),
        // blended exactly as the rig blend at exe+446BE9 blends their targets -
        // factor [controller+0x1D0] > 0 toward the first rig, < 0 toward the
        // third, from the middle one. Built here, from our own numbers, so
        // the game's glides between gun and head never reach the picture.
        float blend = 0.0f;
        TryRead(&blend, controller + 0x1D0, sizeof(blend));
        blend = blend < -1.0f ? -1.0f : (blend > 1.0f ? 1.0f : blend);
        // 3DOF aiming must move the gun and nothing else (2026-09-16). This
        // blend is the same pitch factor the gun's pitch drives, so leaving it
        // in tilts the picture every time the gun goes up or down, which reads
        // as the head moving rather than the gun.
        //
        // It does not matter what moved the gun. The first version only let go
        // of the blend while our own aim write was live, so aiming with a pad
        // pitched the view again (user, 2026-09-16: "this time I used an xbox
        // controller and the gun was no longer separate"). Independence is the
        // point, so while Point to aim is on the view never follows the gun,
        // whether the pitch came from a wrist, a stick or a mouse.
        //
        // Only while the gun is actually UP (2026-09-17). Without that test it
        // also flattened the blend with the gun down, and the blend is what
        // carries the player's own look up and down: "with 3dof enabled, you
        // can no longer move your camera up or down with joystick while not
        // aiming, your head just rolls". With it at zero the view is stuck on
        // the level rig, which is exactly that.
        if (aiming && g_pointToAimActive.load(std::memory_order_relaxed))
            blend = 0.0f;
        // Always the NORMAL set: switching to the aim set at aim start moved
        // the view a little (their bases need not agree) - the small snap.
        const float (*const set)[3] = hf.worldDir;
        const float* middle = set[1];
        const float* toward = blend >= 0.0f ? set[0] : set[2];
        const float w = blend >= 0.0f ? blend : -blend;
        float dir[3];
        for (int k = 0; k < 3; ++k)
            dir[k] = middle[k] * (1.0f - w) + toward[k] * w;
#if RE5VR_DIAGNOSTICS
        if (aimViewTest && aiming) {
            // How far the aim set's view would differ, for the log.
            const float* am = av.viewDir[1];
            const float* at = blend >= 0.0f ? av.viewDir[0] : av.viewDir[2];
            float ad[3];
            for (int k = 0; k < 3; ++k)
                ad[k] = am[k] * (1.0f - w) + at[k] * w;
            const float la = std::sqrt(ad[0] * ad[0] + ad[1] * ad[1] + ad[2] * ad[2]);
            const float ln = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            if (la > 1e-4f && ln > 1e-4f) {
                float cth = (ad[0] * dir[0] + ad[1] * dir[1] + ad[2] * dir[2]) / (la * ln);
                cth = cth < -1.0f ? -1.0f : (cth > 1.0f ? 1.0f : cth);
                static float s_maxDeg = 0.0f;
                static ULONGLONG s_windowMs = 0;
                const float deg = std::acos(cth) * 180.0f / kPi;
                s_maxDeg = deg > s_maxDeg ? deg : s_maxDeg;
                const ULONGLONG nowMs = GetTickCount64();
                if (!s_windowMs)
                    s_windowMs = nowMs;
                if (nowMs - s_windowMs >= 3000) {
                    Log_Printf("CameraRigHook: view-only - while aiming, the aim rigs' view would differ from the normal "
                               "rigs' by up to %.1f deg", s_maxDeg);
                    s_maxDeg = 0.0f;
                    s_windowMs = nowMs;
                }
            }
        }
#endif
        const float dl = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        if (dl > 1e-4f) {
            for (int k = 0; k < 3; ++k)
                hf.worldDir[6][k] = dir[k] / dl;
        } else {
            std::memcpy(hf.worldDir[6], middle, sizeof(hf.worldDir[6]));
        }

#if RE5VR_DIAGNOSTICS
        // Diagnostic (2026-09-15): the user saw the camera creep "lower and
        // lower to the ground" by pressing aim repeatedly with the head still.
        // Log, at every aim press and release, each value that could creep: the
        // game's pitch blend, the eye height we place, the game camera's pitch
        // and the drawn view's pitch.
        {
            static int s_prevAim = -1;
            const int aimNow = aiming ? 1 : 0;
            if (aimNow != s_prevAim) {
                s_prevAim = aimNow;
                float ctrlEye[3] = {}, ctrlTarget[3] = {};
                const bool haveCtrl = TryRead(ctrlEye, controller + 0x170, sizeof(ctrlEye)) &&
                    TryRead(ctrlTarget, controller + 0x190, sizeof(ctrlTarget));
                const auto pitchDeg = [](float x, float y, float z) {
                    const float h = std::sqrt(x * x + z * z);
                    return std::atan2(y, h) * 180.0f / kPi;
                };
                const float camPitch = haveCtrl
                    ? pitchDeg(ctrlTarget[0] - ctrlEye[0], ctrlTarget[1] - ctrlEye[1], ctrlTarget[2] - ctrlEye[2])
                    : 0.0f;
                const float viewPitch = pitchDeg(hf.worldDir[6][0], hf.worldDir[6][1], hf.worldDir[6][2]);
                const float middlePitch = pitchDeg(middle[0], middle[1], middle[2]);
                Log_Printf("CameraRigHook: aim %s - pitch blend %.3f, eye placed (rig) up %.1f fwd %.1f, game camera eye "
                           "height %.1f pitch %.1f deg, view pitch %.1f deg (middle rig %.1f), head pitch %.1f deg",
                    aiming ? "PRESSED" : "released", blend, eyeNormal[1], eyeNormal[2], haveCtrl ? ctrlEye[1] : 0.0f,
                    camPitch, viewPitch, middlePitch,
                    std::asin(std::fmax(-1.0f, std::fmin(1.0f, hf.headForward[1]))) * 180.0f / kPi);
            }
        }
#endif
        g_hfTargetsFront.store(hfBack, std::memory_order_release);
        g_hfTargetsMs.store(GetTickCount64(), std::memory_order_release);
        AcquireSRWLockExclusive(&g_hfHistoryLock);
        g_hfHistoryHead = (g_hfHistoryHead + 1) % kHfHistory;
        g_hfHistory[g_hfHistoryHead] = hf;
        if (g_hfHistoryCount < kHfHistory)
            ++g_hfHistoryCount;
        ReleaseSRWLockExclusive(&g_hfHistoryLock);
    }
}

// ---- View-matrix caller census (2026-09-15) ----------------------------
// The gun reads the camera, so turning the main camera's view toward the head
// while aiming turned the gun too (both experiments). The renderer builds its
// view on demand through the camera classes' virtual GetViewMatrix
// (exe+435C20, vtable slot +0x34), from the canonical +0x30/+0x40/+0x50. If
// the renderer/culling and the aim code call it from different places, the
// view can be split by caller. This logs every distinct caller on the
// player's main camera, with the thread it runs on, every 10 s.
using GetViewMatrixFn = float*(__thiscall*)(void* self, float* out);
GetViewMatrixFn g_origGetViewMatrix = nullptr;
std::atomic<DWORD> g_renderThreadId{ 0 };

struct ViewCaller {
    std::atomic<DWORD> ret{ 0 };
    std::atomic<DWORD> thread{ 0 };
    std::atomic<unsigned> count{ 0 };
    std::atomic<unsigned> otherObjects{ 0 };
};
constexpr int kMaxViewCallers = 48;
ViewCaller g_viewCallers[kMaxViewCallers];

void NoteViewCaller(DWORD ret, bool mainCamera)
{
    for (int i = 0; i < kMaxViewCallers; ++i) {
        DWORD cur = g_viewCallers[i].ret.load(std::memory_order_relaxed);
        if (cur == 0) {
            DWORD expected = 0;
            if (!g_viewCallers[i].ret.compare_exchange_strong(expected, ret)) {
                if (expected != ret)
                    continue;
            } else {
                g_viewCallers[i].thread.store(GetCurrentThreadId(), std::memory_order_relaxed);
            }
            cur = ret;
        }
        if (cur == ret) {
            if (mainCamera)
                g_viewCallers[i].count.fetch_add(1, std::memory_order_relaxed);
            else
                g_viewCallers[i].otherObjects.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }
}

// The callers on the player's main camera (census 2026-09-15, all game-side
// threads): +436311 builds projection * view in the camera's post-update (the
// frustum planes), +8E235A the same on a worker thread (visibility), +0400BA
// copies the view into a render context, +3CFD0F / +3A1FB3 read the camera's
// position and axes for scene work. None changes rate when aiming, so the
// gun does not aim through this function - it reads the camera's fields.
// While aiming, these callers get a view turned toward the head and the
// fields stay as the game set them.
// THE OTHER CALLERS (2026-09-27, from the limiter never seeing a whip).
//
// The limiter caught eight fast turns across a session of gap jumps, a jump
// down, a level change, a cutscene and two stomps, and the largest was 54
// degrees. A whip is not 54 degrees. So the turn does not arrive through our
// view code - and a refusal would have logged too, so our view code is not
// being CALLED on those frames.
//
// The caller census says why. Twelve places fetch the view, five were on this
// list, and the busiest one asking for the PLAYER camera was not:
//
//     exe+05FB34   main camera 1882, other 0
//     exe+22D530   main camera 710
//     exe+2396E7   main camera 710
//     exe+2397FC   main camera 710
//     exe+2398D9   main camera 710
//     exe+23AF9E   main camera 710
//     exe+436311   main camera 354      (was on the list)
//
// Every one of those asks for the player camera specifically, with "other
// cameras 0", so they are not shadow or reflection passes wanting the game
// view. They had simply never been given ours.
//
// Added, with the mainCamera test still in front of them, which is what keeps
// this from repeating the 2026-09-17 mistake of answering every camera in the
// game and making the view fight itself.
bool IsRenderViewCaller(DWORD exeOffset)
{
    switch (exeOffset) {
    case 0x436311:
    case 0x8E235A:
    case 0x0400BA:
    case 0x3CFD0F:
    case 0x3A1FB3:
    case 0x05FB34:
    case 0x22D530:
    case 0x2396E7:
    case 0x2397FC:
    case 0x2398D9:
    case 0x23AF9E:
        return true;
    default:
        return false;
    }
}

// Rotates v about unit axis k by the angle with cosine c and sine s.
void RotateAboutAxisRig(float v[3], const float k[3], float c, float s)
{
    const float kv[3] = { k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0] };
    const float kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    for (int i = 0; i < 3; ++i)
        v[i] = v[i] * c + kv[i] * s + k[i] * kd * (1.0f - c);
}

std::atomic<unsigned long long> g_aimViewRendered{ 0 };

// The main camera's view pointed along the direction head-follow published
// this update (HeadFollowTargets::worldDir[6]), into out. False when there is
// nothing fresh (test off, VR off, a scripted camera owns the view).
bool BuildAimRenderView(void* self, float* out)
{
    if (!g_aimViewTest.load(std::memory_order_relaxed))
        return false;
    const int front = g_hfTargetsFront.load(std::memory_order_acquire);
    if (front < 0 || GetTickCount64() - g_hfTargetsMs.load(std::memory_order_acquire) > 100)
        return false;
    // GetViewMatrix reads only eye +0x30, up +0x40 and target +0x50, so hand
    // the game's own look-at a copy with the target moved.
    unsigned char fake[0x60];
    if (!TryRead(fake, static_cast<unsigned char*>(self), sizeof(fake)))
        return false;
    float eye[3], target[3];
    std::memcpy(eye, fake + 0x30, sizeof(eye));
    std::memcpy(target, fake + 0x50, sizeof(target));
    const float d[3] = { target[0] - eye[0], target[1] - eye[1], target[2] - eye[2] };
    float dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (dl < 1e-3f)
        dl = kFirstPersonTargetDistance;
    const float* dir = g_hfTargets[front].worldDir[6];
    const float vl = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (vl < 1e-4f)
        return false;
    for (int i = 0; i < 3; ++i)
        target[i] = eye[i] + dir[i] / vl * dl;
    std::memcpy(fake + 0x50, target, sizeof(target));
    g_origGetViewMatrix(fake, out);
    g_aimViewRendered.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// ---- Keeping the camera on the head (2026-09-17) ------------------------
// The eye is placed at the head joint by the camera rig hook, and that hook
// stops running the moment the game takes the camera - a sprint's own camera,
// a vault, a jump through a window. The view is then wherever the game puts
// it, which in first person means watching your own neck arrive in front of
// you, or being left behind on a window ledge while Chris goes through it.
//
// The head joint's world position is known every frame regardless, so the view
// can simply be put there instead, here, where the renderer asks for it. Same
// trick as BuildAimRenderView: GetViewMatrix reads only eye, up and target out
// of the camera, so it gets a copy with the eye moved and the direction kept.
//
// Not during a real cutscene, though: a shot framed from across the room is
// deliberate, and dragging it onto somebody's head would wreck it. Distance
// decides - the game's camera is near the head during gameplay and an action,
// and far away when a director has put it somewhere.
std::atomic<bool> g_keepCameraOnHead{ true };
// How far the game's camera may be from your head and still be dragged back to
// it. 400 was far too mean (2026-09-17, user: "we want to keep the camera in
// chris' head when he jump out windows") - going through a window throws the
// camera a long way, and that is precisely the moment worth keeping. Generous
// by default, because the failure it prevents is only a cutscene framed from
// across a room, and those are much further away still.
std::atomic<float> g_headLockMaxDistance{ 2500.0f }; // about 29 metres
std::atomic<bool> g_headLockDirection{ true };
// The fastest the game may turn your view, degrees a second. A stick gives 86
// and a mouse flick a few hundred; a stomp was measured at 1353.
std::atomic<float> g_viewTurnLimitDeg{ 450.0f };

// The eye inside the body, for whoever is drawing (2026-09-23). The VR path
// builds its view out of the game's own camera matrices, so when an action
// camera takes over - a kick, a stomp, a vault - the headset is dragged out
// of the character with it. This is the same placement the rig does, worked
// out from the skeleton, so it is available exactly when the rig is not.
void ViewRefused(const char* why);

bool ComputeBodyEye(const float forward[3], const float camPos[3], float outEye[3])
{
    if (!g_enabled || !g_keepCameraOnHead.load(std::memory_order_relaxed) || !g_lastPlayerHead || !forward
        || !outEye)
        return false;
    float pivot[3];
    if (!TryRead(pivot, g_lastPlayerHead + kOffJointWorldPos, sizeof(pivot)))
        return false;
    if (camPos) {
        const float away[3] = { camPos[0] - pivot[0], camPos[1] - pivot[1], camPos[2] - pivot[2] };
        if (Length3(away) > g_headLockMaxDistance.load(std::memory_order_relaxed)) {
            // A shot framed from across the room: leave it alone. Worth saying
            // out loud, because a jump that throws the camera past the reach
            // hands the whole view back to the game mid-manoeuvre, which is
            // exactly what a sudden wild rotation looks like.
            ViewRefused("the camera is beyond the head lock reach");
            return false;
        }
    }
    float eyeAbove = kEyeAbovePivot;
    if (g_vrEyeOnNeck.load(std::memory_order_relaxed) && g_lastPlayerJoints) {
        unsigned char links[4] = {};
        const ptrdiff_t off = g_lastPlayerHead - g_lastPlayerJoints;
        if (off >= 0 && off % kJointStride == 0 && TryRead(links, g_lastPlayerHead + kOffJointLinks, sizeof(links))) {
            const int headIndex = static_cast<int>(off / kJointStride);
            const int neckIndex = links[1];
            float neck[3];
            if (neckIndex != headIndex && neckIndex < 200
                && TryRead(neck, g_lastPlayerJoints + neckIndex * kJointStride + kOffJointWorldPos, sizeof(neck))) {
                const float gap[3] = { pivot[0] - neck[0], pivot[1] - neck[1], pivot[2] - neck[2] };
                if (Length3(gap) < 60.0f) {
                    std::memcpy(pivot, neck, sizeof(neck));
                    eyeAbove = g_vrEyeAboveNeck.load(std::memory_order_relaxed);
                }
            }
        }
    }
    const float flat = std::sqrt(forward[0] * forward[0] + forward[2] * forward[2]);
    const float aheadX = flat > 1e-4f ? forward[0] / flat : 0.0f;
    const float aheadZ = flat > 1e-4f ? forward[2] / flat : 1.0f;
    // The same running lead the rig applies, so the two placements agree.
    outEye[0] = pivot[0] + aheadX * kEyeAheadOfPivot + g_leadX.load(std::memory_order_relaxed);
    outEye[1] = pivot[1] + eyeAbove;
    outEye[2] = pivot[2] + aheadZ * kEyeAheadOfPivot + g_leadZ.load(std::memory_order_relaxed);
    return true;
}

// WHY THE VIEW WAS NOT OURS (2026-09-26, user: "can we fix that crazy
// rotation that happens some times?").
//
// The position is pinned now and the orientation is not, so the obvious move
// is to pin that too - but writing a rotation into the camera is a good deal
// more dangerous than writing a point, because an ill-formed basis breaks
// everything downstream of it rather than putting it in the wrong place.
//
// And there is a cheaper explanation to rule out first. If the head-locked
// view REFUSES on those frames then the game's own camera orientation is what
// reaches the screen, and no amount of pinning will touch it. The fix would
// then be whichever guard is firing, which is a much smaller thing than
// synthesising a basis every frame.
//
// Three guards can refuse, plus the reach check below. Each says so once every
// two seconds.
void ViewRefused(const char* why)
{
    static unsigned long long s_at = 0;
    static const char* s_last = nullptr;
    const unsigned long long now = GetTickCount64();
    if (why == s_last && now - s_at < 2000)
        return;
    s_at = now;
    s_last = why;
    Log_Printf("HeadLock: the view is the game's this frame - %s", why);
}

bool BuildHeadLockedView(void* self, float* out)
{
    if (!g_enabled) {
        ViewRefused("first person is off");
        return false;
    }
    if (!g_keepCameraOnHead.load(std::memory_order_relaxed)) {
        ViewRefused("keep the camera on the head is unticked");
        return false;
    }
    if (!g_lastPlayerHead) {
        ViewRefused("there is no head joint");
        return false;
    }
    float pivot[3];
    if (!TryRead(pivot, g_lastPlayerHead + kOffJointWorldPos, sizeof(pivot)))
        return false;
    // On the neck instead, when asked for (2026-09-23). The head bone is a
    // head: it nods, it rocks on every shot, it swings on a run. The neck is
    // the top of the spine and does far less of any of that, which is what
    // "stay in his body" actually needs. The eye is then raised back to eye
    // level by hand rather than by following the skull.
    float eyeAbove = kEyeAbovePivot;
    bool onNeck = false;
    float headAboveNeck = 0.0f;
    if (g_vrEyeOnNeck.load(std::memory_order_relaxed) && g_lastPlayerJoints) {
        const ptrdiff_t off = g_lastPlayerHead - g_lastPlayerJoints;
        unsigned char links[4] = {};
        if (off >= 0 && off % kJointStride == 0
            && TryRead(links, g_lastPlayerHead + kOffJointLinks, sizeof(links))) {
            const int headIndex = static_cast<int>(off / kJointStride);
            const int neckIndex = links[1];
            float neckPos[3];
            if (neckIndex != headIndex && neckIndex < 200
                && TryRead(neckPos, g_lastPlayerJoints + neckIndex * kJointStride + kOffJointWorldPos,
                    sizeof(neckPos))) {
                headAboveNeck = pivot[1] - neckPos[1];
                // Only a neck: a joint a long way from the head is not one.
                const float d[3] = { pivot[0] - neckPos[0], pivot[1] - neckPos[1], pivot[2] - neckPos[2] };
                if (Length3(d) < 60.0f) {
                    std::memcpy(pivot, neckPos, sizeof(neckPos));
                    eyeAbove = g_vrEyeAboveNeck.load(std::memory_order_relaxed);
                    onNeck = true;
                }
            }
        }
    }
    // Standing still should BE standing still (2026-09-23, user: "standing
    // completely still seems to jiggle in the headset"). The eye sits on the
    // neck bone and the neck bone is animated the whole time he is alive -
    // breathing, shifting his weight, the little settle at the end of a step.
    // On a screen that reads as a character who is alive. In a headset it is
    // your own skull being moved for you, and there is no amount of it that
    // feels good.
    //
    // What is taken out is only the bone's movement AGAINST HIS ROOT. Walking,
    // turning and being shoved all move the root and arrive untouched, so
    // nothing lags; it is the animation on top that is held still. Bounded to
    // a few centimetres as well, so this can never walk the eye away from the
    // real head - which is what hides it, and what a previous attempt at this
    // broke.
    {
        const float steady = g_vrViewSteady.load(std::memory_order_relaxed);
        float rootM[16];
        if (steady > 0.01f && g_lastPlayerJoints
            && TryRead(rootM, g_lastPlayerJoints + kOffJointWorldMatrix, sizeof(rootM))) {
            const float rootPos[3] = { rootM[12], rootM[13], rootM[14] };
            float rootRot[9];
            bool rok = true;
            for (int r = 0; r < 3 && rok; ++r) {
                const float e0 = rootM[r * 4], e1 = rootM[r * 4 + 1], e2 = rootM[r * 4 + 2];
                const float l = std::sqrt(e0 * e0 + e1 * e1 + e2 * e2);
                rok = l > 0.01f;
                if (rok) {
                    rootRot[r * 3] = e0 / l;
                    rootRot[r * 3 + 1] = e1 / l;
                    rootRot[r * 3 + 2] = e2 / l;
                }
            }
            if (rok) {
                const float d[3] = { pivot[0] - rootPos[0], pivot[1] - rootPos[1], pivot[2] - rootPos[2] };
                float rel[3];
                for (int r = 0; r < 3; ++r)
                    rel[r] = rootRot[r * 3] * d[0] + rootRot[r * 3 + 1] * d[1] + rootRot[r * 3 + 2] * d[2];
                static float s_held[3] = {};
                static bool s_haveHeld = false;
                static unsigned char* s_heldJoints = nullptr;
                const float jump[3] = { rel[0] - s_held[0], rel[1] - s_held[1], rel[2] - s_held[2] };
                if (!s_haveHeld || s_heldJoints != g_lastPlayerJoints || Length3(jump) > 25.0f) {
                    std::memcpy(s_held, rel, sizeof(rel));
                    s_haveHeld = true;
                    s_heldJoints = g_lastPlayerJoints;
                } else {
                    // 1 follows a tenth of the way a frame, 0 follows all of it.
                    const float follow = 1.0f - steady * 0.9f;
                    for (int r = 0; r < 3; ++r)
                        s_held[r] += (rel[r] - s_held[r]) * follow;
                }
                // Never further from the animated bone than this.
                const float fix[3] = { s_held[0] - rel[0], s_held[1] - rel[1], s_held[2] - rel[2] };
                const float away = Length3(fix);
                constexpr float kMostUnits = 6.0f; // about seven centimetres
                if (away > kMostUnits) {
                    const float k = kMostUnits / away;
                    for (int r = 0; r < 3; ++r)
                        s_held[r] = rel[r] + fix[r] * k;
                }
                for (int k = 0; k < 3; ++k)
                    pivot[k] = rootPos[k] + rootRot[k] * s_held[0] + rootRot[3 + k] * s_held[1]
                        + rootRot[6 + k] * s_held[2];
            }
        }

    }

    // Running lead (2026-09-22, user: "keeping the head with the body at all
    // times, no breakouts"). At a run the view lands in Chris's neck: the head
    // bone read here is most likely the last game tick's, and at 5 m/s one
    // tick is 8 cm. So the eye is moved on by how far the character itself
    // travelled over the last tick - taken from where he stands, not from the
    // bone, so the animation's bob and lean are not in it - level, never
    // vertical. Standing still the step is zero and nothing changes. The head
    // only reappears for a camera that jumps 50 units in a frame to 60 away,
    // so a lead of a few units that builds up with speed never trips it.
    const float runLead = g_vrRunLead.load(std::memory_order_relaxed);
    float leadApplied = 0.0f;
    float charStep = 0.0f;
    float leadVec[2] = { 0.0f, 0.0f };
    {
        static float s_prevPos[3], s_step[3];
        static bool s_havePos = false;
        static unsigned char* s_prevChar = nullptr;
        unsigned char* controller = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
        unsigned char* character = nullptr;
        float charPos[3];
        if (controller && TryRead(&character, controller + kOffControllerCharacter, sizeof(character)) && character
            && TryRead(charPos, character + kOffBodyTransform[0] + 0x30, sizeof(charPos))) {
            if (!s_havePos || character != s_prevChar) {
                std::memcpy(s_prevPos, charPos, sizeof(s_prevPos));
                s_step[0] = s_step[1] = s_step[2] = 0.0f;
                s_havePos = true;
                s_prevChar = character;
            } else if (charPos[0] != s_prevPos[0] || charPos[2] != s_prevPos[2]) {
                // A new game tick: this is how far one tick carried him. The
                // renderer asks several times a tick, so only a change counts.
                s_step[0] = charPos[0] - s_prevPos[0];
                s_step[2] = charPos[2] - s_prevPos[2];
                std::memcpy(s_prevPos, charPos, sizeof(s_prevPos));
            }
            charStep = std::sqrt(s_step[0] * s_step[0] + s_step[2] * s_step[2]);
            // A step longer than any run is a teleport or a cut; lead nothing.
            if (g_vrActive.load(std::memory_order_relaxed) && runLead > 0.0f && charStep > 0.05f && charStep < 30.0f) {
                pivot[0] += s_step[0] * runLead;
                pivot[2] += s_step[2] * runLead;
                leadApplied = charStep * runLead;
                g_leadUnits.store(leadApplied, std::memory_order_relaxed);
                leadVec[0] = s_step[0] * runLead;
                leadVec[1] = s_step[2] * runLead;
            }
        }
    }
    g_leadX.store(leadVec[0], std::memory_order_relaxed);
    g_leadZ.store(leadVec[1], std::memory_order_relaxed);

    unsigned char fake[0x60];
    if (!TryRead(fake, static_cast<unsigned char*>(self), sizeof(fake)))
        return false;
    float eye[3], target[3];
    std::memcpy(eye, fake + 0x30, sizeof(eye));
    std::memcpy(target, fake + 0x50, sizeof(target));

    // MEASURED ON THE GAME'S CAMERA (2026-09-27, user: "I could not seem to
    // make a difference with the swing/reach sliders - they didn't seem to
    // change anything").
    //
    // They could not. This distance is read from +0x30, which the pin
    // overwrites with your neck, so with the pin on it is always about zero -
    // the reach is never exceeded and the slider has nothing to act on. The
    // same mistake as the view direction, one step further up: a test about
    // where the GAME put its camera has to be given where the game put its
    // camera.
    float reachFrom[3] = { eye[0], eye[1], eye[2] };
    if (XrInput_GetSettings().cameraFollowsEye && g_haveGameEye && g_gameEyeFrom == self)
        std::memcpy(reachFrom, g_gameEye, sizeof(reachFrom));
    const float away[3] = { reachFrom[0] - pivot[0], reachFrom[1] - pivot[1], reachFrom[2] - pivot[2] };
    const float distance = Length3(away);
    // AND IT MUST NOT LET GO MID-VAULT (2026-09-27, user: "still had that bug
    // of the camera not seating back to its proper position though and could
    // orbit around").
    //
    // A vault throws the camera past the reach, the lock releases, and nothing
    // brings it back on its own - which is why only jumping down off something
    // re-seats it, since that is what finally moves the camera inside the
    // reach again. Taking the view and keeping it are different questions, so
    // once it is held the reach is doubled before it will let go. Same
    // hysteresis as the departure test, for the same reason.
    static bool s_holding = false;
    const float reach = g_headLockMaxDistance.load(std::memory_order_relaxed) * (s_holding ? 2.0f : 1.0f);
    if (distance > reach) {
        // Which grabs and cutscenes are being refused, and by how much - "when
        // a zombie grabs you from behind, the camera still goes 3rd person, it
        // may just be far enough away to trip it". If a grab shows up here at,
        // say, 1400, the reach is simply set too short.
#if RE5VR_DIAGNOSTICS
        static ULONGLONG s_ms = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - s_ms > 2000) {
            s_ms = now;
            Log_Printf("HeadLock: left alone - the camera is %.0f units from your head, past the %.0f unit reach",
                distance, g_headLockMaxDistance.load(std::memory_order_relaxed));
        }
#endif
        s_holding = false;
        return false; // a chosen shot: leave it alone
    }
    s_holding = true;
    // NOT WHEN WE PUT IT THERE (2026-09-27, after "played out the same way").
    //
    // This early out is what the pin has been tripping every single frame.
    // "Already there, nothing to do" is true of the POSITION and false of
    // everything else: the view still has to be rebuilt so the orientation
    // comes from the target rather than from whatever the game is doing, and
    // with the pin on the camera is always within a unit of your head by
    // construction. So the pin was switching off the one thing that had been
    // keeping your head straight for weeks, and every symptom since - the
    // whip, the orbit, the body sliding - has been the game's own view showing
    // through.
    //
    // It also explains why writing harder never helped. The pin runs at
    // EndScene, which is after the view for this frame has already been built,
    // so those writes were never going to steer it. The head lock is what
    // steers it, and it has to be allowed to run.
    if (distance < 1.0f && !XrInput_GetSettings().cameraFollowsEye)
        return false; // already there, nothing to do

    // NOT FROM THE PINNED EYE (2026-09-27, user: "this stutter thing is also
    // annoying, that shit needs to go").
    //
    // The direction is measured from the eye, and the eye is read from +0x30 -
    // which the pin overwrites with the neck joint. So with the pin on, the
    // view direction is measured from a joint that bobs once per game tick,
    // and with a target only a metre or two ahead a few centimetres of bob
    // swings it a couple of degrees. Every tick. That is the stutter, it is
    // our own arithmetic rather than anything fighting us, and it explains why
    // it arrived the moment the pin started working and is worst while
    // walking.
    //
    // The pin keeps a copy of the value the game had before it wrote, taken at
    // the only moment it is still there. The direction uses that. Where the
    // camera IS stays ours; what it is looking along stays the game's.
    float fromEye[3] = { eye[0], eye[1], eye[2] };
    if (XrInput_GetSettings().cameraFollowsEye && g_haveGameEye && g_gameEyeFrom == self)
        std::memcpy(fromEye, g_gameEye, sizeof(fromEye));
    float dir[3] = { target[0] - fromEye[0], target[1] - fromEye[1], target[2] - fromEye[2] };

    // ---- Does our view reach the screen at all? (2026-09-27) --------------
    //
    // The single unknown that every failed fix tonight depended on, and which
    // none of them established. The limiter refused a 102 degree turn and the
    // whip arrived anyway; the whip happens in flatscreen, where the VR path
    // does not run. Both are consistent with our view being computed
    // perfectly and then not being used.
    //
    // This settles it in one keypress and needs no interpretation. While the
    // Scroll Lock window is open, the view faces due world north, ignoring
    // the game entirely. Scroll Lock does nothing in the game itself, so
    // nothing else changes.
    //
    //   The view snaps to a fixed heading for two seconds  -> our view IS what
    //   you see, the whip comes through this function, and the fix belongs
    //   here.
    //
    //   Nothing happens  -> our view is discarded, everything we have done to
    //   it was always going to be wasted, and the real path is elsewhere.
    //
    // Either answer removes more doubt than anything else available, and the
    // second would explain the entire night.
    if (g_northTestUntilMs && GetTickCount64() <= g_northTestUntilMs) {
        dir[0] = 0.0f;
        dir[1] = 0.0f;
        dir[2] = 1.0f;
        static unsigned long long s_saidAt = 0;
        const unsigned long long nowProve = GetTickCount64();
        if (nowProve - s_saidAt > 500) {
            s_saidAt = nowProve;
            Log_Printf("HeadLock: End test - forcing the view due north. If the picture did not move, "
                       "this view is not the one you are looking through.");
        }
    }
    const float dl = Length3(dir);
    if (dl < 1e-3f)
        return false;
    for (int i = 0; i < 3; ++i)
        dir[i] /= dl;

    // Where the view LOOKS, while the game has the camera (2026-09-17, user:
    // "when getting revived, the camera did the rotate around animation").
    // Moving the eye onto the head fixes where you are standing and does
    // nothing about where you are facing, so a scripted camera still swings
    // the whole world around you - which is a camera move in a game and a ride
    // in a headset. While the game is driving, the view faces the way the
    // character faces instead, and your own head rotation is added on top as
    // usual. Only while it is driving: in normal play the camera direction is
    // the player's own and must not be touched.
    // Held still, not handed to the body (2026-09-17). Facing where the
    // character faces was not enough - "yes, still feel them" - because during
    // a stomp or a revive the CHARACTER turns: he swings round to bring a boot
    // down, he kneels and twists to pull somebody up. Following his facing
    // faithfully reproduces that as a spin, which is the thing being complained
    // about. So the direction is latched the moment the game takes the camera
    // and held for as long as it has it. Your own head rotation still goes on
    // top, so you can look around normally throughout; the world simply stops
    // being turned for you. When control comes back, the latch is dropped and
    // the game's camera takes over again.
    //
    // The direction to hold is the one from BEFORE the game took over, not the
    // one on the first frame afterwards (2026-09-17, user: "now the camera puts
    // it over your shoulder like looking behind you"). By the time we notice
    // the camera has been taken, it has already been swung somewhere, and
    // latching then just freezes that shot in place. So the player's own
    // direction is remembered continuously while they still have the camera,
    // and that is what gets held.
    //
    // A speed limit, not a state test (2026-09-17). Every attempt to detect
    // "an action is happening" failed, and the log finally said why: during a
    // stomp the view turns at 1353 degrees a second while our scripted-camera
    // test reports the game does NOT have the camera. It is not taking the
    // camera away at all, it is spinning the player's own.
    //
    // So nothing is detected. Rotation faster than a person could ask for is
    // simply not passed on. A stick at full tilt turns you 86 degrees a second
    // and a mouse flick maybe four hundred; at a thousand and up it is the
    // game moving you, not you. What is refused is only the excess: the view
    // still travels, at the fastest rate a player could have asked for, so it
    // arrives where the game wants it a moment later without throwing anybody
    // across the room.
    // ONCE A FRAME, AND IN MICROSECONDS (2026-09-27, from the Home trace).
    //
    // The trace settled it. Every frame reads "aim 0 / headlock N / the game
    // 0", so the view is ours even through a whip - and the whip is right
    // there in the numbers, 136 and 145 degrees between consecutive frames.
    // The limiter simply was not catching it, and the same line says why: this
    // function runs FOURTEEN times per presented frame.
    //
    // The old guard was "now > s_prevMs" against a millisecond clock. Fourteen
    // calls a frame means most land in the same millisecond as the last, so
    // that test failed and the limiter was skipped - while the remembered
    // direction was updated to the new one regardless. The whip passed
    // unlimited, and by the time a call arrived with a nonzero dt the
    // reference had already been overwritten with the whipped value. Nothing
    // left to limit, which is exactly the three feeble catches we saw.
    //
    // Two changes. Elapsed time comes from the performance counter, so a
    // sub-millisecond gap is a real number rather than zero. And repeats
    // within a frame return the same answer as the first call instead of being
    // re-limited against themselves: the direction is limited when it CHANGES
    // and replayed otherwise.
    //
    // The reference is what we last GAVE BACK rather than what we were last
    // asked for, so a hold actually holds instead of ratcheting along behind
    // the game one frame at a time.
    bool facedByBody = false;
    {
        static float s_askedFor[3] = { 0.0f, 0.0f, 1.0f };
        static float s_gaveBack[3] = { 0.0f, 0.0f, 1.0f };
        static bool s_havePrev = false;
        static LARGE_INTEGER s_changedAt = {};
        static LARGE_INTEGER s_freq = {};
        if (!s_freq.QuadPart)
            QueryPerformanceFrequency(&s_freq);
        LARGE_INTEGER nowQpc;
        QueryPerformanceCounter(&nowQpc);
        const float limit = g_viewTurnLimitDeg.load(std::memory_order_relaxed);

        const bool sameAsk = s_havePrev && std::fabs(dir[0] - s_askedFor[0]) < 1e-6f
            && std::fabs(dir[1] - s_askedFor[1]) < 1e-6f && std::fabs(dir[2] - s_askedFor[2]) < 1e-6f;
        if (sameAsk) {
            std::memcpy(dir, s_gaveBack, sizeof(s_gaveBack));
        } else {
            std::memcpy(s_askedFor, dir, sizeof(s_askedFor));
            const float dt = (s_havePrev && s_freq.QuadPart)
                ? static_cast<float>(nowQpc.QuadPart - s_changedAt.QuadPart) / static_cast<float>(s_freq.QuadPart)
                : 0.0f;
            s_changedAt = nowQpc;
            // AND NEVER REFUSE THE PLAYER (2026-09-27, user: "even at 2000 it
            // cannot keep up with a full speed mouse move").
            //
            // That retires the rate test as a discriminator. A mouse flick and
            // a stomp are both faster than a person could ask for, because a
            // flick IS a person asking, and no threshold separates them.
            //
            // The real question is whether you asked, and it is answerable
            // outright: input_block sees the raw mouse before the game does.
            // If the mouse moved in the last fifth of a second the turn is
            // yours and passes untouched however fast it is. A stomp moves the
            // view with no input at all, so it is caught by the same test that
            // lets a flick through. Rate stays only as a backstop for input we
            // cannot see.
            const unsigned long long lookedAt = InputBlock_LastLookMs();
            // AND THE AIM SERVO COUNTS AS YOU (2026-09-27, user: "everything
            // was fine til I pressed the aim button on the vr controller, then
            // catastrophy").
            //
            // Aiming in VR runs a closed loop: it steers the game's aim toward
            // where your controller points by feeding synthetic mouse deltas,
            // watches the view move, and keeps pushing until it arrives. Hold
            // the view still in front of that and it sees no progress, so it
            // pushes harder every frame. Classic windup, and it goes exactly
            // as violently as described.
            //
            // The servo stamps g_aimMouseMs every time it steers, so its
            // turns are as much "you asked" as a hand on a mouse - more so,
            // since they came from your wrist. Never limit them.
            //
            // ONLY WHEN IT ACTUALLY STEERED (2026-09-27, user: "the stomp
            // didn't seem to catch"). The timestamp is stamped every time the
            // aim path runs, not only when it moves anything, so with motion
            // controllers on it is always fresh and "you asked" was
            // permanently true - which switches the limiter off entirely. The
            // deltas it sent are the honest test.
            //
            // AND ONLY WHILE ACTUALLY AIMING (2026-09-27, user: "some melee
            // actions had checkboard... some didn't - so there's something
            // going on there for a perfect storm").
            //
            // That is the storm. In VR the servo steers whenever your hand
            // moves at all, wobble included, so "you asked" was true whenever
            // you were not perfectly still. A stomp with your hands moving got
            // no limiting; a stomp while you happened to be still got caught.
            // The checkerboard followed the same coin toss, because it follows
            // whether the view held.
            //
            // The servo has no business steering unless the gun is up, and the
            // game keeps that flag itself at controller+0x1B1. Hand wobble
            // with the gun down no longer excuses anything.
            bool aimingNow = false;
            {
                unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
                unsigned char aimFlag = 0;
                aimingNow = pc && TryRead(&aimFlag, pc + kOffControllerAimFlag, sizeof(aimFlag)) && aimFlag == 1;
            }
            const unsigned long long servoAt = g_aimMouseMs.load(std::memory_order_relaxed);
            const bool servoSteered = aimingNow
                && (g_aimMouseDx.load(std::memory_order_relaxed) != 0
                    || g_aimMouseDy.load(std::memory_order_relaxed) != 0);
            const unsigned long long nowAsk = GetTickCount64();
            // Running counts too: this game turns the camera to follow where
            // you steer, and a stomp involves no movement input at all.
            const bool youAsked = InputBlock_MovementHeld() || (lookedAt && nowAsk - lookedAt < 200)
                || (servoSteered && servoAt && nowAsk - servoAt < 200);

            // AND A HOLD MUST NEVER RUN AWAY (2026-09-27, user: "the camera
            // did not stay in my head and was all over the world ... this was
            // before even stomping").
            //
            // In VR the mouse never moves, so the "did you ask" test is false
            // on every frame and everything falls back to the rate threshold.
            // With that set low, every deliberate stick turn was over the line
            // and got held, creeping back at twenty degrees a second while the
            // player kept turning. The view simply never catches up, which is
            // how it ends up anywhere but with you.
            //
            // The slider fixes that case, but nothing should be able to hold
            // the view indefinitely whatever the slider says. An action camera
            // lasts a second or two; anything longer is us being wrong, and
            // being wrong should expire.
            static unsigned long long s_holdingSince = 0;
            const unsigned long long nowHold = GetTickCount64();
            // FOUR SECONDS, NOT ONE AND A HALF (2026-09-27, the log: every leak
            // reads "held too long 1"). A stomp runs longer than a second and a
            // half, so the guard meant to stop a runaway was cutting the hold
            // off partway through and handing the rest of the swing to the
            // player. The guard still matters - a mis-set threshold must not be
            // able to keep the view forever - but it has to outlast the thing
            // it is protecting, not interrupt it.
            const bool heldTooLong = s_holdingSince && nowHold - s_holdingSince > 4000;
            if (heldTooLong) {
                static unsigned long long s_toldAt = 0;
                if (nowHold - s_toldAt > 2000) {
                    s_toldAt = nowHold;
                    Log_Printf("HeadLock: holding for over four seconds - giving the view back, the "
                               "swing limit is probably too low for how you turn");
                }
            }
            // WHY IT DECLINED (2026-09-27, user: "the stomps aren't getting
            // caught"). Five conditions have to hold and I have now guessed
            // wrong about which one fails four times running. Every large turn
            // that is NOT limited says so, with the state of all five, so the
            // next answer comes from the log rather than from me.
            if (s_havePrev) {
                float d = dir[0] * s_gaveBack[0] + dir[1] * s_gaveBack[1] + dir[2] * s_gaveBack[2];
                d = d > 1.0f ? 1.0f : (d < -1.0f ? -1.0f : d);
                const float bigTurn = std::acos(d) * 180.0f / kPi;
                const bool wouldLimit = !youAsked && !heldTooLong && limit > 0.0f && dt > 0.0f
                    && bigTurn > limit * (dt < 0.25f ? dt : 0.25f);
                if (bigTurn > 25.0f && !wouldLimit) {
                    static unsigned long long s_toldAt = 0;
                    if (nowAsk - s_toldAt > 300) {
                        s_toldAt = nowAsk;
                        Log_Printf("HeadLock: let a %.0f deg turn through - you asked %d (mouse %llu ms ago, "
                                   "servo %d, aiming %d), held too long %d, dt %.1f ms, limit %.0f",
                            bigTurn, youAsked ? 1 : 0,
                            lookedAt ? static_cast<unsigned long long>(nowAsk - lookedAt) : 9999ull,
                            servoSteered ? 1 : 0, aimingNow ? 1 : 0, heldTooLong ? 1 : 0, dt * 1000.0f, limit);
                    }
                }
            }
            // THE RATE TEST IS RETIRED (2026-09-27, from the VR stomp log).
            //
            // Two lines, same millisecond:
            //
            //   let a 89 deg turn through ... dt 5624.3 ms, limit 473
            //   the game has moved your view 88 deg without you asking - held
            //
            // Capping dt at a quarter second to stop long gaps waiving the
            // limiter turned it into a licence instead: 473 degrees a second
            // times a quarter second is an allowance of 118 degrees in a
            // single step. The rate test was letting the opening frame of the
            // stomp through, and the drift anchor was left cleaning up after
            // it.
            //
            // It cannot be repaired by tuning, because rate was never the
            // right question - you proved that with the full speed mouse
            // move, and again with a camera animation that tilts you slowly.
            // The anchor asks the question that matters, how far the game has
            // moved you without being asked, and it subsumes this completely.
            //
            // Left in place, switched off, because the reasoning above is
            // worth more than the code and deleting it would take both.
            constexpr bool kUseRateLimit = false;
            if (kUseRateLimit && !youAsked && !heldTooLong && s_havePrev && limit > 0.0f && dt > 0.0f) {
                float dot = dir[0] * s_gaveBack[0] + dir[1] * s_gaveBack[1] + dir[2] * s_gaveBack[2];
                dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
                const float turned = std::acos(dot) * 180.0f / kPi;
                // A LONG GAP IS NOT A LICENCE (2026-09-27, user: "first it started
                // rotated toward the ground, then stuck behind my back").
                //
                // That is us latching a direction that had already swung. The
                // log shows how: "dt 1163.6 ms" - when the gap between two
                // direction changes ran over a quarter second the limiter was
                // skipped altogether, so the opening swing of a stomp passed
                // untouched and we then held you facing wherever it had put
                // you. The file already warned about exactly this: the
                // direction to hold is the one from BEFORE the game took over.
                //
                // A long gap means we have no rate information, which is a
                // reason to be careful rather than permissive. The allowance
                // is capped at a quarter second of travel instead of being
                // waived, so a big turn after a stall is still judged.
                const float judgeDt = dt < 0.25f ? dt : 0.25f;
                const float allowed = limit * judgeDt;
                if (turned > allowed && turned > 0.01f) {
                    // Refused, not deferred. Clamping to the limit still
                    // delivers the whole turn over the following frames, which
                    // is a slower whip rather than none. Anything faster than
                    // a person could have asked for is treated as not asked
                    // for: the view holds and creeps after the game at a
                    // walking pace, so a vault ending where it started costs
                    // almost no movement while a genuine change of direction
                    // still arrives under its own power.
                    // HOLD, DO NOT CREEP (2026-09-27, user: "its like a slomo
                    // drift back to my actual view, and still angled. Again,
                    // if anything it should just hold your view where it was
                    // when the stomp happened").
                    //
                    // Exactly so. Creeping at twenty degrees a second was
                    // still travelling toward the direction the stomp wanted,
                    // just slowly - which is the slow motion drift, and it is
                    // why you end up angled: it spends the whole stomp quietly
                    // going somewhere you never asked to go.
                    //
                    // Zero means the view simply stays. And nothing is needed
                    // to end the hold: once the action finishes and the game
                    // asks for a direction near the one we latched, the turn
                    // is small, falls under the allowance, and passes through
                    // on its own. The hold releases by the game agreeing with
                    // it rather than by us deciding when to stop.
                    constexpr float kCreepDegPerSec = 0.0f;
                    const float creep = kCreepDegPerSec * dt;
                    const float k = (creep < turned ? creep : turned) / turned;
                    for (int i = 0; i < 3; ++i)
                        dir[i] = s_gaveBack[i] + (dir[i] - s_gaveBack[i]) * k;
                    const float l = Length3(dir);
                    if (l > 1e-4f) {
                        for (int i = 0; i < 3; ++i)
                            dir[i] /= l;
                    }
                    facedByBody = true;
                    if (!s_holdingSince)
                        s_holdingSince = nowHold;
                    // LEVEL WHILE HELD (2026-09-27, user: "it does catch a
                    // stomp now, but unfortunately does not hold the camera
                    // straight and it has a bit of a cock to it").
                    //
                    // Holding the direction does not hold the roll: the view
                    // is built from this direction and an up vector that is
                    // still the game's, so a stomp that tilts the camera tilts
                    // you even while the heading is held. A tilted horizon is
                    // the one thing in a headset worse than a whip, and there
                    // is no case in this game where a rolled gameplay camera
                    // is wanted, so while we are holding, the view is levelled
                    // outright.
                    // The roll is dealt with where the view is actually built,
                    // further down: see "AND LEVEL THE HORIZON WHILE HOLDING".
                    static unsigned long long s_toldAt = 0;
                    const unsigned long long nowFast = GetTickCount64();
                    if (nowFast - s_toldAt > 250) {
                        s_toldAt = nowFast;
                        Log_Printf("HeadLock: the game asked for %.0f deg in %.1f ms (%.0f deg/s), allowed "
                                   "%.1f - held",
                            turned, dt * 1000.0f, turned / dt, allowed);
                    }
                }
            }
            // HOW FAR, NOT HOW FAST (2026-09-27, user: "the camera stayed
            // forward but rotated toward the ground", and "its like we're
            // taking the first frame of the camera animation, which would tilt
            // down to show the stomp happening").
            //
            // Both describe the same gap, and it is the limit of a rate test.
            // A stomp that pitches you forty five degrees over half a second
            // is ninety degrees a second - slower than a person turning - so
            // every individual frame passes the rate check and the movement
            // accumulates anyway. Rate can catch a whip; it can never catch a
            // slow, large, unrequested move, and a camera animation is exactly
            // that.
            //
            // What matters is how far the game has moved your view in TOTAL
            // without you asking. So an anchor is kept: the direction you were
            // last looking when you last asked for something. While you are
            // not asking, the view may wander a few degrees from it and no
            // further. Fast or slow makes no difference, which is the point.
            //
            // The anchor follows you whenever you do ask, so ordinary play
            // never meets it - it only exists during the moments nobody
            // requested anything, which is precisely when a camera animation
            // is playing.
            {
                static float s_anchor[3] = { 0.0f, 0.0f, 1.0f };
                static bool s_haveAnchor = false;
                constexpr float kMaxDriftDeg = 8.0f;
                // THE ANCHOR MUST PREDATE THE ANIMATION (2026-09-27, the
                // trace: yaw -13 pitch -50, unchanging, for a whole stomp).
                //
                // The hold worked perfectly. It held you fifty degrees at the
                // ground, because the anchor is taken on the frame we notice
                // you stopped asking - and by then the game has already
                // tilted you into the first frame of its animation. The note
                // from 2026-09-17 says exactly this: the direction to hold is
                // the one from BEFORE the game took over, not the one on the
                // first frame afterwards.
                //
                // So two anchors are kept, a quarter second apart, and the
                // OLDER one is used. While you are asking they both roll
                // forward and the lag never shows. The moment you stop, the
                // one in hand is a quarter to half a second stale, which is
                // comfortably before any animation began.
                static float s_prevAnchor[3] = { 0.0f, 0.0f, 1.0f };
                static unsigned long long s_rolledAt = 0;
                // ASK THE GAME, DO NOT GUESS (2026-09-28). The anchor now
                // engages exactly while controller+0x634 says an action
                // camera is running, and at no other time. Every heuristic
                // this replaces - mouse movement, the servo, movement keys,
                // hold expiry - existed only because we could not tell.
                const bool gameDriving = GameIsDrivingTheCamera();
                // Says when the flag flips, so "the melee cam was not being
                // stopped" can be told apart from "the flag never said melee".
                // Those want completely different work and the log can settle
                // it in one stomp instead of another round of guessing.
                {
                    static int s_was = -1;
                    const int now = gameDriving ? 1 : 0;
                    if (now != s_was) {
                        s_was = now;
                        Log_Printf("HeadLock: the game %s the camera (controller+634)",
                            now ? "has taken" : "has given back");
                    }
                }
                if (!gameDriving || !s_haveAnchor) {
                    if (!s_haveAnchor || nowHold - s_rolledAt > 250) {
                        s_rolledAt = nowHold;
                        std::memcpy(s_prevAnchor, s_anchor, sizeof(s_prevAnchor));
                    }
                    std::memcpy(s_anchor, dir, sizeof(s_anchor));
                    if (!s_haveAnchor)
                        std::memcpy(s_prevAnchor, dir, sizeof(s_prevAnchor));
                    s_haveAnchor = true;
                } else {
                    // Held against the older one, which predates whatever the
                    // game started doing.
                    std::memcpy(s_anchor, s_prevAnchor, sizeof(s_anchor));
                    float d = dir[0] * s_anchor[0] + dir[1] * s_anchor[1] + dir[2] * s_anchor[2];
                    d = d > 1.0f ? 1.0f : (d < -1.0f ? -1.0f : d);
                    const float drifted = std::acos(d) * 180.0f / kPi;
                    if (drifted > kMaxDriftDeg) {
                        const float k = kMaxDriftDeg / drifted;
                        for (int i = 0; i < 3; ++i)
                            dir[i] = s_anchor[i] + (dir[i] - s_anchor[i]) * k;
                        const float l = Length3(dir);
                        if (l > 1e-4f) {
                            for (int i = 0; i < 3; ++i)
                                dir[i] /= l;
                        }
                        facedByBody = true;
                        if (!s_holdingSince)
                            s_holdingSince = nowHold;
                        static unsigned long long s_toldAt = 0;
                        if (nowHold - s_toldAt > 500) {
                            s_toldAt = nowHold;
                            Log_Printf("HeadLock: the game has moved your view %.0f deg without you asking - "
                                       "held at %.0f",
                                drifted, kMaxDriftDeg);
                        }
                    }
                }
            }

            if (!facedByBody)
                s_holdingSince = 0; // the clock resets the moment it stops holding
            std::memcpy(s_gaveBack, dir, sizeof(s_gaveBack));
            s_havePrev = true;
        }
    }

    // The eye sits a little above and ahead of the pivot, the same offsets the
    // rig placement uses, along the direction the view is already looking.
    const float flat = std::sqrt(dir[0] * dir[0] + dir[2] * dir[2]);
    const float aheadX = flat > 1e-4f ? dir[0] / flat : 0.0f;
    const float aheadZ = flat > 1e-4f ? dir[2] / flat : 1.0f;
    float newEye[3] = {
        pivot[0] + aheadX * kEyeAheadOfPivot,
        pivot[1] + eyeAbove,
        pivot[2] + aheadZ * kEyeAheadOfPivot,
    };
    float newTarget[3];
    for (int i = 0; i < 3; ++i)
        newTarget[i] = newEye[i] + dir[i] * dl;
    // WHAT WE ACTUALLY HAND THE RENDERER (2026-09-28).
    //
    // TraceTheCamera has been reading the GAME's camera object all along, and
    // that thing is supposed to misbehave - during a door kick it flies a
    // metre and a half from the head and swings thirty degrees of pitch. None
    // of that is what you see, because we build the view ourselves from the
    // head joint and the clamped direction. So every trace I have read has
    // been describing a camera we deliberately ignore.
    //
    // This is the one that matters: the eye and direction we write into the
    // copy, which is what the renderer is given. If these stay put during a
    // kick and you still get moved, the view is being changed after we return
    // it. If they move, the hold is not working and I can see by how much.
    if (g_actionTraceUntilMs && GetTickCount64() <= g_actionTraceUntilMs) {
        const float yaw = std::atan2(dir[0], dir[2]) * 180.0f / kPi;
        const float flatDir = std::sqrt(dir[0] * dir[0] + dir[2] * dir[2]);
        const float pitchDeg = flatDir > 1e-4f ? std::atan2(dir[1], flatDir) * 180.0f / kPi : 0.0f;
        Log_Printf("Ours: eye (%.0f %.0f %.0f) yaw %.0f pitch %.0f | pivot (%.0f %.0f %.0f) | held %d",
            newEye[0], newEye[1], newEye[2], yaw, pitchDeg, pivot[0], pivot[1], pivot[2],
            facedByBody ? 1 : 0);
    }
    std::memcpy(fake + 0x30, newEye, sizeof(newEye));
    std::memcpy(fake + 0x50, newTarget, sizeof(newTarget));

    // AND LEVEL THE HORIZON WHILE HOLDING (2026-09-27, the two screenshots:
    // the held view is rolled about thirty degrees against the same spot).
    //
    // Holding the direction does not hold the roll. We hand the game a copy of
    // its camera with our eye and target written in, and it builds a look-at
    // from that - a look-at needs an up vector, and the only place for one in
    // this 0x60 byte block is between the eye at +30 and the target at +50. So
    // a stomp that rolls the camera hands us a rolled up vector and we pass it
    // on faithfully.
    //
    // While we are holding, it goes to world vertical. There is no gameplay
    // camera in this game that wants roll, and a tilted horizon is the one
    // thing in a headset that is worse than the whip we are already refusing.
    // Only while holding, so a cutscene that genuinely rolls still can.
    if (facedByBody) {
        const float straightUp[3] = { 0.0f, 1.0f, 0.0f };
        float wasUp[3];
        std::memcpy(wasUp, fake + 0x40, sizeof(wasUp));
        std::memcpy(fake + 0x40, straightUp, sizeof(straightUp));
#if RE5VR_DIAGNOSTICS
        // Confirms +0x40 really is the up vector: it should read close to
        // straight up in ordinary play and tilt during a stomp. If it never
        // resembles an up vector, this is the wrong field and the roll lives
        // somewhere else in the block.
        static unsigned long long s_toldAt = 0;
        const unsigned long long nowUp = GetTickCount64();
        if (nowUp - s_toldAt > 500) {
            s_toldAt = nowUp;
            Log_Printf("HeadLock: levelling while held - the camera's up was (%.2f %.2f %.2f)", wasUp[0],
                wasUp[1], wasUp[2]);
        }
#endif
    }
    g_origGetViewMatrix(fake, out);

    // LEVEL THE RESULT, NOT THE INPUT (2026-09-27, user: "it still is
    // rolling").
    //
    // Writing world up into the copy at +0x40 did nothing, even though the log
    // proves that field IS an up vector and IS rolled - "(-0.77 0.62 -0.15)",
    // about fifty degrees over. So the function takes its roll from somewhere
    // we are not supplying, and we hand it only 0x60 bytes of a much larger
    // object.
    //
    // Chasing that is unnecessary. The roll is visible in the matrix that
    // comes back, so it can be taken out of the matrix that comes back, and
    // that works whatever the source. The rows are right, up and forward,
    // scaled; rebuild right and up about world vertical while keeping the
    // forward the view already has, and the horizon is level by construction.
    // ONLY WHEN THERE IS ROLL WORTH REMOVING (2026-09-27, user: "in VR, melee
    // actions, kicking doors make the screen go black for a second").
    //
    // This rebuilds rows of the view matrix, and holding is far more frequent
    // since the drift limit went in, so it went from running rarely to running
    // almost always. A rebuild that is subtly wrong is survivable once in a
    // while and fatal every frame.
    //
    // It is only here to take out roll, so it should only run when there IS
    // roll. An up vector already within a few degrees of vertical is left
    // completely alone, which keeps it off the common path entirely.
    bool rollWorthFixing = false;
    if (facedByBody && out) {
        const float uy = out[5];
        const float ul = Length3(&out[4]);
        // out[5] over the row length is the cosine between the view's up and
        // world up. Above 0.996 is under five degrees of roll.
        // THE LEVELLING IS OFF (2026-09-28: the stomp went black again).
        //
        // It has caused a black screen three times now - twice when it was
        // first added, and again during this stomp - and its benefit was never
        // demonstrated. It rebuilds rows of the view matrix whenever the hold
        // is engaged, and the hold flickers on and off through a stomp, so it
        // runs intermittently on a matrix it may be rebuilding wrongly.
        //
        // The trace also removed the reason for it: what we hand the renderer
        // during a stomp is already steady, three degrees of yaw and six of
        // pitch. There is no roll left in our own view to take out. Whatever
        // roll was seen came from somewhere downstream, and rebuilding a
        // matrix that is already correct can only do harm.
        constexpr bool kLevelTheHorizon = false;
        rollWorthFixing = kLevelTheHorizon && ul > 1e-4f && (uy / ul) < 0.996f;
    }
    if (rollWorthFixing) {
        float f[3] = { out[8], out[9], out[10] };
        const float fl = Length3(f);
        if (fl > 1e-4f) {
            for (int i = 0; i < 3; ++i)
                f[i] /= fl;
            // Straight up or down has no horizontal part to level against, and
            // inventing one there would spin the view wildly.
            if (f[1] < 0.99f && f[1] > -0.99f) {
                const float sx = Length3(&out[0]);
                const float sy = Length3(&out[4]);
                float r[3] = { f[2], 0.0f, -f[0] }; // world up crossed with forward
                const float rl = Length3(r);
                if (rl > 1e-4f && sx > 1e-4f && sy > 1e-4f) {
                    for (int i = 0; i < 3; ++i)
                        r[i] /= rl;
                    // TAKE THE HANDEDNESS FROM THE MATRIX (2026-09-27, user:
                    // "the screen went black during one of the stomps, and
                    // behind my back in another").
                    //
                    // Both are one fault. I built right as world up crossed
                    // with forward without checking which convention this
                    // matrix uses, so it can come out negated - which mirrors
                    // the view, and with the up derived from it can invert
                    // the whole matrix into nothing.
                    //
                    // There is no need to know the convention. The matrix
                    // already contains a correct right and up for this view;
                    // they are merely rolled. So the rebuilt axes are flipped
                    // to agree with the ones already there, which removes the
                    // roll and cannot change anything else.
                    if (r[0] * out[0] + r[1] * out[1] + r[2] * out[2] < 0.0f) {
                        for (int i = 0; i < 3; ++i)
                            r[i] = -r[i];
                    }
                    float u[3] = { r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2],
                        r[0] * f[1] - r[1] * f[0] };
                    if (u[0] * out[4] + u[1] * out[5] + u[2] * out[6] < 0.0f) {
                        for (int i = 0; i < 3; ++i)
                            u[i] = -u[i];
                    }
                    for (int i = 0; i < 3; ++i) {
                        out[i] = r[i] * sx;
                        out[4 + i] = u[i] * sy;
                    }
                    // The translation terms belong to the rows they sit in, so
                    // they have to be rebuilt from the same vectors or the eye
                    // lands somewhere else entirely.
                    out[3] = -(r[0] * newEye[0] + r[1] * newEye[1] + r[2] * newEye[2]) * sx;
                    out[7] = -(u[0] * newEye[0] + u[1] * newEye[1] + u[2] * newEye[2]) * sy;
                }
            }
        }
    }
#if RE5VR_DIAGNOSTICS
    // How far the view was actually being left behind, and how often. A sprint
    // that still trails means either this is not running or the head joint
    // itself lags the character, and those need different answers.
    {
        static ULONGLONG s_ms = 0;
        static float s_worst = 0.0f;
        static unsigned s_count = 0;
        static float s_maxLead = 0.0f, s_maxStep = 0.0f;
        // Where the character's position really lives (2026-09-23): two guesses
        // have now reported either teleports or nothing at all, so the candidates
        // are measured side by side while you run and the biggest sane mover wins.
        static float s_prevCand[4][3];
        static float s_maxCand[4] = {};
        static bool s_haveCand = false;
        {
            unsigned char* c2 = nullptr;
            unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
            float cand[4][3] = {};
            bool ok[4] = {};
            if (pc && TryRead(&c2, pc + kOffControllerCharacter, sizeof(c2)) && c2) {
                ok[0] = TryRead(cand[0], c2 + kOffCharacterPos, sizeof(cand[0]));
                ok[1] = TryRead(cand[1], c2 + kOffBodyTransform[0] + 0x30, sizeof(cand[1]));
                ok[2] = TryRead(cand[2], c2 + kOffBodyTransform[1] + 0x30, sizeof(cand[2]));
            }
            if (g_lastPlayerJoints)
                ok[3] = TryRead(cand[3], g_lastPlayerJoints + kOffJointWorldPos, sizeof(cand[3]));
            for (int c = 0; c < 4; ++c) {
                if (!ok[c])
                    continue;
                if (s_haveCand) {
                    float d[3];
                    for (int i = 0; i < 3; ++i)
                        d[i] = cand[c][i] - s_prevCand[c][i];
                    const float step = Length3(d);
                    if (step > s_maxCand[c] && step < 500.0f)
                        s_maxCand[c] = step;
                }
                std::memcpy(s_prevCand[c], cand[c], sizeof(cand[c]));
            }
            s_haveCand = true;
        }
        if (leadApplied > s_maxLead)
            s_maxLead = leadApplied;
        if (charStep > s_maxStep)
            s_maxStep = charStep;
        ++s_count;
        if (distance > s_worst)
            s_worst = distance;
        const ULONGLONG now = GetTickCount64();
        if (!s_ms)
            s_ms = now;
        if (now - s_ms >= 3000) {
            // Both explanations for a sprint that still trails, in one line
            // rather than one run each: how far the game's camera was behind
            // the head (this fix's job), and how far the head joint itself was
            // behind the character it belongs to (the animation's own lag,
            // which no camera work can help).
            float charPos[3] = {};
            float headBehind = -1.0f;
            unsigned char* character = nullptr;
            // Through the player's own controller, not through "self": this
            // now runs for whatever camera is being drawn, and most of them are
            // not the main one, so the main camera's layout does not apply.
            unsigned char* playerController = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
            if (playerController
                && TryRead(&character, playerController + kOffControllerCharacter, sizeof(character))
                && character && TryRead(charPos, character + kOffCharacterPos, sizeof(charPos))) {
                const float d[3] = { pivot[0] - charPos[0], pivot[1] - charPos[1], pivot[2] - charPos[2] };
                headBehind = std::sqrt(d[0] * d[0] + d[2] * d[2]); // flat: the head is meant to be above
            }
            Log_Printf("HeadLock: view brought back to the head %u time(s) in 3 s, worst gap %.1f units; the head "
                       "joint sits %.1f units from the character%s; running lead up to %.1f units (a tick carried "
                       "him up to %.1f); eye on the %s, head sits %.1f above the neck",
                s_count, s_worst, headBehind, facedByBody ? "; direction held still" : "", s_maxLead, s_maxStep,
                onNeck ? "neck" : "head", headAboveNeck);
            Log_Printf("HeadLock: biggest step in 3 s - character+0x30 %.1f, transform A %.1f, transform B %.1f, "
                       "root joint %.1f units",
                s_maxCand[0], s_maxCand[1], s_maxCand[2], s_maxCand[3]);
            std::memset(s_maxCand, 0, sizeof(s_maxCand));
            s_maxLead = 0.0f;
            s_maxStep = 0.0f;
            s_ms = now;
            s_count = 0;
            s_worst = 0.0f;
        }
    }
#endif
    return true;
}

float* __fastcall GetViewMatrixHook(void* self, void* /*edx*/, float* out)
{
    const DWORD ret = static_cast<DWORD>(reinterpret_cast<uintptr_t>(_ReturnAddress()));
    const unsigned char* player = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    const bool mainCamera = player && self == static_cast<void*>(const_cast<unsigned char*>(player) - kOffMainCameraPlayerController);
// NOT DEVELOPER ONLY (2026-09-28, while answering "does that require VR").
//
// Everything below was inside the diagnostics guard, which meant a RELEASE
// build never collapsed the rig here, never told the culling which camera was
// the main one, and never learned which camera object is ours - so the melee
// hold, which asks exactly that question, would have done nothing at all for
// anyone but us. None of it is a diagnostic. Only NoteViewCaller is.

    // COLLAPSE IT BEFORE IT IS USED, NOT AFTER (2026-09-27, user: "their
    // freeze DEFINITELY worked").
    //
    // Ours re-applies at Present, which is the END of the frame - so the
    // camera for that frame has already been built from the melee values and
    // we are correcting one frame late, every frame. The trainer stops the
    // values landing at all, which is why theirs works and ours does not.
    //
    // Here is immediately before the view is produced, which is the last
    // moment that matters.
    if (mainCamera)
        KeepTheRigCollapsed();
#if RE5VR_DIAGNOSTICS
    NoteViewCaller(ret, mainCamera);
#endif
    if (mainCamera) {
        CullingPatch_NoteMainCamera(self);
        // A DIFFERENT CAMERA MEANS A DIFFERENT EVERYTHING (2026-09-27, user:
        // "the stuttering was a lot less, until I transferred to a new
        // level").
        //
        // A level change builds a new camera, and the eye we remembered
        // belongs to the old one. Carrying it over feeds the view direction a
        // position from the previous map, which is as wrong as a value can be
        // and shows up as exactly the stutter this was meant to cure.
        if (g_theCameraRaw != self) {
            g_haveGameEye = false;
            g_gameEyeFrom = nullptr;
        }
        g_theCamera.store(self, std::memory_order_relaxed);
        g_theCameraRaw = self;
    }
#if RE5VR_DIAGNOSTICS
    // Is the view we are fixing even the one being drawn? (2026-09-17: the
    // head lock reports itself working hundreds of times a second and the
    // camera still swings behind you during a grab.) If the renderer is asking
    // a DIFFERENT camera object for its matrix while the game has control, then
    // every correction we make is to a camera nobody is looking through.
    {
        static ULONGLONG s_ms = 0;
        static unsigned s_mine = 0, s_other = 0;
        static const DWORD s_dbgBase = static_cast<DWORD>(reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr)));
        if (IsRenderViewCaller(ret - s_dbgBase)) {
            if (mainCamera)
                ++s_mine;
            else
                ++s_other;
        }
        // How far from your head those other cameras sit. A camera that has run
        // away with the player during a stomp should be a few metres off; one
        // that was never looking at you at all will be much further, or will
        // not move with you. That difference is what the head lock needs in
        // order to tell them apart, since distance alone clearly cannot.
        static float s_otherNear = 1e9f, s_otherFar = 0.0f;
        if (!mainCamera && IsRenderViewCaller(ret - s_dbgBase) && g_lastPlayerHead) {
            float pivotP[3], eyeP[3];
            unsigned char peekO[0x60];
            if (TryRead(pivotP, g_lastPlayerHead + kOffJointWorldPos, sizeof(pivotP))
                && TryRead(peekO, static_cast<unsigned char*>(self), sizeof(peekO))) {
                std::memcpy(eyeP, peekO + 0x30, sizeof(eyeP));
                const float d[3] = { eyeP[0] - pivotP[0], eyeP[1] - pivotP[1], eyeP[2] - pivotP[2] };
                const float dist = Length3(d);
                if (dist < s_otherNear)
                    s_otherNear = dist;
                if (dist > s_otherFar)
                    s_otherFar = dist;
            }
        }
        const ULONGLONG now = GetTickCount64();
        if (!s_ms)
            s_ms = now;
        if (now - s_ms >= 3000) {
            if (s_other)
                Log_Printf("ViewSource: the renderer asked the player's camera %u time(s) and some OTHER camera %u "
                           "time(s) in 3 s; those others sat between %.0f and %.0f units from your head",
                    s_mine, s_other, s_otherNear > 1e8f ? -1.0f : s_otherNear, s_otherFar);
            s_otherNear = 1e9f;
            s_otherFar = 0.0f;
            s_ms = now;
            s_mine = s_other = 0;
        }
    }
#endif
#if RE5VR_DIAGNOSTICS
    // What a stomp or a revive actually does to the view (2026-09-17: it stays
    // on the head and "just rotates like 180 degrees", and the latch meant to
    // stop that fires only during what we call a scripted camera). So: how fast
    // does the view swing during one, and does our scripted test even agree
    // that something is happening? If the swing is large while the test says
    // no, then the game is turning the player's OWN camera rather than taking
    // it away, and the fix has to hang off something else entirely.
    if (mainCamera) {
        static float s_prevFwd[3] = { 0.0f, 0.0f, 1.0f };
        static bool s_havePrev = false;
        static ULONGLONG s_prevMs = 0;
        unsigned char peek[0x60];
        if (TryRead(peek, static_cast<unsigned char*>(self), sizeof(peek))) {
            float eyeP[3], targetP[3];
            std::memcpy(eyeP, peek + 0x30, sizeof(eyeP));
            std::memcpy(targetP, peek + 0x50, sizeof(targetP));
            float f[3] = { targetP[0] - eyeP[0], targetP[1] - eyeP[1], targetP[2] - eyeP[2] };
            const float fl = Length3(f);
            const ULONGLONG now = GetTickCount64();
            if (fl > 1e-3f) {
                for (int i = 0; i < 3; ++i)
                    f[i] /= fl;
                if (s_havePrev && now != s_prevMs) {
                    float dot = f[0] * s_prevFwd[0] + f[1] * s_prevFwd[1] + f[2] * s_prevFwd[2];
                    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
                    const float turned = std::acos(dot) * 180.0f / kPi;
                    const float rate = turned * 1000.0f / static_cast<float>(now - s_prevMs);
                    static ULONGLONG s_saidMs = 0;
                    if (rate > 200.0f && now - s_saidMs > 500) {
                        s_saidMs = now;
                        Log_Printf("ViewSwing: the view turned %.0f deg/sec and the game %s have the camera",
                            rate, CameraRigHook_InScriptedCamera() ? "DOES" : "does NOT");
                    }
                }
                std::memcpy(s_prevFwd, f, sizeof(s_prevFwd));
                s_prevMs = now;
                s_havePrev = true;
            }
        }
    }
#endif
    {
        static const DWORD s_base = static_cast<DWORD>(reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr)));
        if (IsRenderViewCaller(ret - s_base)) {
            if (mainCamera && BuildAimRenderView(self, out)) {
                ++g_viewFromAim;
                return out;
            }
            // Back to the player's own camera only (2026-09-17). Applying this
            // to every camera the renderer asks made the view "fight itself
            // while standing still", and the log shows why: gaps of 1780 to
            // 1850 units, which are cameras twenty metres away being yanked
            // onto somebody's head. Those are other views the game keeps for
            // its own purposes, and the reach - raised to 2500 for grabs -
            // was wide enough to catch them.
            //
            // So distance alone cannot tell "a camera that has run away with
            // the player" from "a camera that was never looking at the player".
            // Which camera it IS has to be part of the test.
            if (mainCamera && BuildHeadLockedView(self, out)) {
                ++g_viewFromHeadLock;
                return out;
            }
            if (mainCamera)
                ++g_viewFromGame;
        }
    }
    return g_origGetViewMatrix(self, out);
}

void LogViewCallers()
{
    static ULONGLONG s_lastMs = 0;
    const ULONGLONG now = GetTickCount64();
    if (now - s_lastMs < 10000)
        return;
    s_lastMs = now;
    HMODULE exe = GetModuleHandleA(nullptr);
    const DWORD base = static_cast<DWORD>(reinterpret_cast<uintptr_t>(exe));
    const DWORD renderThread = g_renderThreadId.load(std::memory_order_relaxed);
    Log_Printf("CameraRigHook: GetViewMatrix callers in the last 10 s (exe+offset of the instruction after the call; "
               "render thread %lu; aim views turned toward the head so far %llu):",
        renderThread, g_aimViewRendered.load(std::memory_order_relaxed));
    for (int i = 0; i < kMaxViewCallers; ++i) {
        const DWORD ret = g_viewCallers[i].ret.load(std::memory_order_relaxed);
        if (!ret)
            break;
        const unsigned mainCount = g_viewCallers[i].count.exchange(0, std::memory_order_relaxed);
        const unsigned others = g_viewCallers[i].otherObjects.exchange(0, std::memory_order_relaxed);
        if (!mainCount && !others)
            continue;
        const DWORD thread = g_viewCallers[i].thread.load(std::memory_order_relaxed);
        Log_Printf("CameraRigHook:   exe+%06lX  main camera %u, other cameras %u, thread %lu%s", ret - base, mainCount,
            others, thread, thread == renderThread ? " (render)" : "");
    }
}

// Right after the game has written the character's aim pitch. See the notes
// by g_aimWriteHooked. Runs every frame while the player exists, so it does
// as little as possible and leaves the value alone unless 3DOF aiming has one
// waiting for this exact character.
void AimPitchWritten(unsigned char* character)
{
    if (!character || character != g_aimWriteCharacter.load(std::memory_order_relaxed))
        return;
    const unsigned long long when = g_aimWriteMs.load(std::memory_order_relaxed);
    if (!when || GetTickCount64() - when > 200)
        return;
    const unsigned long offset = g_aimWriteOffset.load(std::memory_order_relaxed);
    if (!offset)
        return;
    const float value = g_aimWriteValue.load(std::memory_order_relaxed);
    TryWrite(character + offset, &value, sizeof(value));
}

// Right after the game has written the aim point. The bearing and range it
// just wrote are kept; only the height changes, so the gun swings up and down
// to where the controller points without touching where it points across.
void AimPointWritten(unsigned char* character)
{
    if (!character || character != g_aimPointCharacter.load(std::memory_order_relaxed))
        return;
    const unsigned long long when = g_aimPointMs.load(std::memory_order_relaxed);
    if (!when || GetTickCount64() - when > 200)
        return;

    float point[3] = {};
    if (!TryRead(point, character + 0x2980, sizeof(point)))
        return;

    // Test mode: slam the ray a long way up whatever the controller says. If
    // nothing visible moves, nothing reads this ray and the search moves on;
    // if the laser swings to the ceiling, we own aiming and only the sign and
    // scale are left to sort out (2026-09-16).
    if (g_aimPointForceUp.load(std::memory_order_relaxed)) {
        const float dx = point[0] - g_aimPointEyeX.load(std::memory_order_relaxed);
        const float dz = point[2] - g_aimPointEyeZ.load(std::memory_order_relaxed);
        const float flatTest = std::sqrt(dx * dx + dz * dz);
        const float up = g_aimPointEyeY.load(std::memory_order_relaxed) + flatTest;
        TryWrite(character + 0x2984, &up, sizeof(up));
        // And the origin vector just above it, in case the ray is read as a
        // pair rather than a point.
        float origin[3] = {};
        if (TryRead(origin, character + 0x2970, sizeof(origin))) {
            static ULONGLONG s_logMs = 0;
            const ULONGLONG now = GetTickCount64();
            if (now - s_logMs > 1000) {
                s_logMs = now;
                Log_Printf("AimRay: origin +0x2970 (%.1f %.1f %.1f), ray +0x2980 (%.1f %.1f %.1f), forcing height "
                           "to %.1f",
                    origin[0], origin[1], origin[2], point[0], point[1], point[2], up);
            }
        }
        return;
    }
    const float eyeX = g_aimPointEyeX.load(std::memory_order_relaxed);
    const float eyeY = g_aimPointEyeY.load(std::memory_order_relaxed);
    const float eyeZ = g_aimPointEyeZ.load(std::memory_order_relaxed);
    const float dx = point[0] - eyeX, dz = point[2] - eyeZ;
    const float flat = std::sqrt(dx * dx + dz * dz);
    if (flat < 1.0f)
        return;
    const float newY = eyeY + flat * g_aimPointTan.load(std::memory_order_relaxed);
    TryWrite(character + 0x2984, &newY, sizeof(newY));
}

typedef void(__cdecl* CallAimPointWritten_t)(unsigned char*);
CallAimPointWritten_t g_callAimPointWritten = &AimPointWritten;

__declspec(naked) void AimPointHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call g_callAimPointWritten
        add esp, 4
        popfd
        popad
        jmp g_aimPointTrampoline
    }
}

// ---- The aim angles at their source (2026-09-17) ------------------------
// exe+7879AA stores the pitch into [esi+4B8h], right after storing the yaw
// into [esi+4B4h], at the tail of the integrator the watchpoint chase found.
// This sits one instruction later, so esi is still the struct those two live
// in - the only copy of the aim that everything else is derived from.
//
// Read-only for now: it publishes the two numbers so the mapping between them
// and real degrees can be worked out from a log. Writing them is what turns
// pointing from a servo chasing deltas into the gun simply being where the
// controller points, and that is the next step.
void AimAnglesWritten(unsigned char* aimStruct)
{
    if (!aimStruct)
        return;
    float yaw = 0.0f, pitch = 0.0f;
    if (!TryRead(&yaw, aimStruct + 0x4B4, sizeof(yaw)) || !TryRead(&pitch, aimStruct + 0x4B8, sizeof(pitch)))
        return;
    g_aimStructValueYaw.store(yaw, std::memory_order_relaxed);
    g_aimStructValuePitch.store(pitch, std::memory_order_relaxed);
    g_aimStruct.store(aimStruct, std::memory_order_relaxed);
    g_aimStructMs.store(GetTickCount64(), std::memory_order_relaxed);

    // Absolute pitch (2026-09-17). The run paired the stored value against the
    // angle it produces: it is linear and inverted, saturating at exactly
    // 1.0000 when the rig sits at -50.1 degrees, so a degree is a fiftieth of a
    // unit and the whole range is one number. That is everything absolute
    // pointing needs on this axis - no servo, no debt, no learned gain. Write
    // where the controller points and the gun is there this frame.
    const float wantPitch = g_absoluteAimPitchDeg.load(std::memory_order_relaxed);
    if (GetTickCount64() - g_absoluteAimMs.load(std::memory_order_relaxed) < 250) {
        float value = -wantPitch / kAimPitchDegPerUnit;
        if (value > 1.0f)
            value = 1.0f;
        if (value < -1.0f)
            value = -1.0f;
        TryWrite(aimStruct + 0x4B8, &value, sizeof(value));
    }

    // Yaw's number is not an angle at all - see the log run - so it is only
    // watched here, paired with how far the gun's bearing moved in the same
    // window, to find out what it really is.
    g_yawValueSamples.push(yaw);
}

// The skeleton build (2026-09-17). exe+183B29 is the instruction after the
// outermost of the three writers the wrist watchpoint found, with ecx still
// holding the joint whose world matrix was just composed. Arm IK counts these
// to find the end of the build and solves there, ahead of the weapon attach.
void* g_skeletonTrampoline = nullptr;

void SkeletonJointBuilt(unsigned char* joint)
{
    ArmIk_OnJointBuilt(joint);
}

// The animation finishing a joint's rotation (2026-09-18). Three nested
// routines write it, each storing four floats one at a time; these are the
// instructions just after the last float of each, with esi holding the joint.
// All three are hooked because only the last write of the frame decides the
// pose, and which of the three that is depends on the blend.
void* g_poseDoneTrampoline[1] = {};

void PoseComplete(unsigned char* joint)
{
    ArmIk_OnPoseComplete(joint);
}

typedef void(__cdecl* CallPoseComplete_t)(unsigned char*);
CallPoseComplete_t g_callPoseComplete = &PoseComplete;

__declspec(naked) void PoseDoneHook0_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call g_callPoseComplete
        add esp, 4
        popfd
        popad
        jmp g_poseDoneTrampoline[0]
    }
}

// The other half of the wrist watch (2026-09-17): exe+19CB74 reads the hand's
// world matrix exactly once a frame with ebx holding an object that is neither
// the character nor the skeleton. Nothing else in the report fits a weapon, and
// a weapon has to come from somewhere, so the pointer is worth keeping.
void* g_handReaderTrampoline = nullptr;

void HandMatrixRead(unsigned char* source, unsigned char* object)
{
    ArmIk_NoteHandReader(source, object);
}

typedef void(__cdecl* CallHandMatrixRead_t)(unsigned char*, unsigned char*);
CallHandMatrixRead_t g_callHandMatrixRead = &HandMatrixRead;

__declspec(naked) void HandReaderHook_Stub()
{
    __asm {
        pushad
        pushfd
        push ebx
        push ecx
        call g_callHandMatrixRead
        add esp, 8
        popfd
        popad
        jmp g_handReaderTrampoline
    }
}

typedef void(__cdecl* CallSkeletonJointBuilt_t)(unsigned char*);
CallSkeletonJointBuilt_t g_callSkeletonJointBuilt = &SkeletonJointBuilt;

__declspec(naked) void SkeletonHook_Stub()
{
    __asm {
        pushad
        pushfd
        push ecx
        call g_callSkeletonJointBuilt
        add esp, 4
        popfd
        popad
        jmp g_skeletonTrampoline
    }
}

typedef void(__cdecl* CallAimAnglesWritten_t)(unsigned char*);
CallAimAnglesWritten_t g_callAimAnglesWritten = &AimAnglesWritten;

__declspec(naked) void AimAnglesHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call g_callAimAnglesWritten
        add esp, 4
        popfd
        popad
        jmp g_aimAnglesTrampoline
    }
}

typedef void(__cdecl* CallAimPitchWritten_t)(unsigned char*);
CallAimPitchWritten_t g_callAimPitchWritten = &AimPitchWritten;

__declspec(naked) void AimPitchHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call g_callAimPitchWritten
        add esp, 4
        popfd
        popad
        jmp g_aimWriteTrampoline
    }
}

typedef void(__cdecl* CallOnRigsReady_t)(unsigned char*);
CallOnRigsReady_t g_callOnRigsReady = &CameraRigHook_OnRigsReady;

__declspec(naked) void RigsReadyHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call g_callOnRigsReady
        add esp, 4
        popfd
        popad
        jmp g_rigsReadyTrampoline
    }
}


// Is this still the head joint we think it is? Same three checks
// FindHeadJoint makes, so a freed or reused allocation fails them rather than
// being written to. Cheap enough to run every frame the hook is quiet.
bool HeadJointStillValid(unsigned char* joints, unsigned char* head)
{
    if (!joints || !head)
        return false;
    DWORD rootClass = 0, headClass = 0;
    unsigned char links[4] = {};
    if (!TryRead(&rootClass, joints, sizeof(rootClass)) || !rootClass)
        return false;
    if (!TryRead(&headClass, head, sizeof(headClass)) || headClass != rootClass)
        return false;
    if (!TryRead(links, head + kOffJointLinks, sizeof(links)))
        return false;
    if (links[3] != kHeadJointId)
        return false;
    const unsigned char* parent = joints + links[1] * kJointStride;
    unsigned char parentLinks[4] = {};
    if (!TryRead(parentLinks, parent + kOffJointLinks, sizeof(parentLinks)))
        return false;
    return parentLinks[3] == kChestJointId;
}

// Called every frame from EndScene, which keeps running when the game's own
// camera code does not. If the camera hook has gone quiet, the character is
// in something scripted - a vault, a stomp, a cutscene - and the head must
// not be left collapsed for the duration of it.
void HeadWatchdog_Tick()
{
    if (!g_enabled || !g_lastPlayerHead || !g_lastPlayerHeadMs)
        return;
    const unsigned long long now = GetTickCount64();
    if (now - g_lastPlayerHeadMs < kHookQuietMs)
        return; // the camera hook is alive; UpdateHead owns the head
    if (!HeadJointStillValid(g_lastPlayerJoints, g_lastPlayerHead)) {
        // Whoever that was is gone. Forget them rather than write into it.
        g_lastPlayerHead = nullptr;
        g_lastPlayerJoints = nullptr;
        g_watchdogRestored = false;
        return;
    }

    // While the hook is quiet the watchdog owns the head completely - showing
    // it is not enough. Measured 2026-09-12: after a vault, the watchdog gave
    // the head back in 109 ms (good) and it then stayed visible for 3.5
    // SECONDS, because only UpdateHead can hide it again and the game had not
    // handed the camera back yet. The action was long over.
    //
    // Nothing about hiding needs the camera hook, though. The head joint's own
    // world position is readable here, and the rendered camera comes from the
    // vertex constants, so the same question - is the camera back inside the
    // head? - can be answered every frame regardless. In first person the
    // camera sits about 19 from the head PIVOT (the eye is 15 ahead and 11
    // above it), so 35 is inside-the-head with margin to spare.
    bool wantVisible = true;
    float pivot[3], cam[3];
    if (TryRead(pivot, g_lastPlayerHead + kOffJointWorldPos, sizeof(pivot)) && LastCameraPosition(cam)) {
        const float d[3] = { pivot[0] - cam[0], pivot[1] - cam[1], pivot[2] - cam[2] };
        wantVisible = Length3(d) > kWatchdogHideDistance;
    }
    // The flicker guard outranks this: if two cameras are being read as one,
    // the watchdog would happily join in the strobing.
    if (HeadLockedHidden(now) || !g_showHeadDuringActions.load(std::memory_order_relaxed))
        wantVisible = false;

    if (wantVisible == g_watchdogRestored)
        return; // already in the state we want
    SetHeadScale(g_lastPlayerHead, wantVisible ? 1.0f : 0.0f);
    g_watchdogRestored = wantVisible;
    if (wantVisible) {
        g_watchdogRestores.fetch_add(1, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: camera hook quiet for %llu ms and the camera is away from the head "
                   "- scripted action, head restored by the watchdog",
            now - g_lastPlayerHeadMs);
    } else {
        HideTrace_Open(now);
        Log_Printf("CameraRigHook: camera back inside the head while the hook is still quiet - head hidden again "
                   "by the watchdog");
    }
}
// ---- Where the action camera actually moves you (2026-09-27) --------------
//
// The stomp window caught a writer that never appears in ordinary play:
//
//   exe+43A560  (CC padded, so a real function entry)
//     8B 44 24 04   mov eax,[esp+4]      ; first argument
//     D9 00         fld  [eax]
//     D9 59 30      fstp [ecx+30]        ; eye
//     ... x, y, z ...
//     8B 44 24 08   mov eax,[esp+8]      ; second argument
//     D9 00 / D9 59 50                   ; target
//
// A thiscall SetLookAt(eye, target), with ecx pointing at camera+0x380 - so
// its +30 is the +3B0 we have been watching all night and its +50 is the
// target at +3D0. It fired three times during a stomp and not once otherwise.
//
// THIS is what moves you during an action, and it explains why nothing we did
// downstream held: the pin corrects at Present, and this runs mid-frame,
// before the view is built. It is also the clean intervention we never had -
// a real function handing over both ends of the look-at, so there is no
// accumulator to fight, no copies to chase, and no value of ours going
// anywhere the game can feed back from.
//
// What it is given instead: your eye for the position, and for the target,
// the direction the camera ALREADY had, re-originated at your eye. So an
// action camera may no longer turn your head at all - which is the stated
// requirement, "enough to sell the effect of jumping, but anything intense
// causes discomfort" - while everything else about the shot proceeds normally.
using SetLookAt_t = void(__fastcall*)(void* self, void* edx, const float* eye, const float* target);
SetLookAt_t g_origSetLookAt = nullptr;
bool g_lookAtHooked = false;
unsigned long g_lookAtsCaught = 0;

// TELLING THE PATCH FROM THE VALUES (2026-09-27, after the level-load crash).
//
// Two things could have caused it and they want opposite fixes. Either the
// five byte detour itself is unsafe - something jumping into the middle of the
// six bytes MinHook relocated, which a packed executable makes impossible to
// rule out by reading - or the substituted look-at leaves the game holding
// state it derived from the values it thought it passed. The faulting read was
// E284003C, garbage rather than a small offset from null, which smells like
// the second but does not prove it.
//
// A hook that changes nothing separates them in a single run. If it still
// crashes, the detour is at fault and the values are innocent. If it survives,
// the detour is fine and the substitution is what needs rethinking. It cannot
// make the game any worse than no hook at all, because functionally it IS no
// hook at all.
// REACHED FROM THE CALL SITES, NOT FROM A DETOUR (2026-09-28).
//
// The five byte detour on this function is proven unsafe: a hook that changed
// nothing crashed the level load exactly as the substituting one did, so the
// values were never the problem and the relocation always was. Something
// jumps into the middle of the bytes MinHook has to move, and a packed exe
// hides that completely.
//
// So do not touch the function. Touch the callers.
//
// Every "call exe+43A560" in the decrypted code section is five bytes: E8
// followed by a relative offset. Rewriting that offset to point here is the
// same five bytes in the same place - nothing is relocated, nothing is
// trampolined, and the function itself stays byte for byte as the game wrote
// it. We call the real one ourselves afterwards, so the game gets its call.
//
// It also answers the question the trainer was going to answer: the return
// address says WHICH action camera is speaking, so a stomp, a door kick and a
// vault are finally distinguishable.
unsigned long g_lookAtSiteCount = 0;

void __fastcall SetLookAtHook(void* self, void* edx, const float* eye, const float* target)
{
    const void* from = _ReturnAddress();
    const bool ours = self && g_theCameraRaw
        && static_cast<unsigned char*>(self) == static_cast<unsigned char*>(g_theCameraRaw) + 0x380;

    bool mine = false;
    float newEye[3] = {}, newTarget[3] = {}, asked[3] = {};
    if (ours && eye && target && g_enabled && g_holdViewInMelee.load(std::memory_order_relaxed)) {
        __try {
            const float* was = reinterpret_cast<const float*>(static_cast<unsigned char*>(self) + 0x30);
            const float* wasAt = reinterpret_cast<const float*>(static_cast<unsigned char*>(self) + 0x50);
            // The direction we already had. On the first call of an action
            // that is where you were looking; on the second and third it is
            // what we just wrote, so it holds of its own accord.
            float keep[3] = { wasAt[0] - was[0], wasAt[1] - was[1], wasAt[2] - was[2] };
            float len = Length3(keep);
            if (!(len > 1e-3f)) {
                // Nothing to keep - fall back to the direction being asked
                // for, which at least leaves the shot coherent.
                keep[0] = target[0] - eye[0];
                keep[1] = target[1] - eye[1];
                keep[2] = target[2] - eye[2];
                len = Length3(keep);
            }
            // Where the eye goes. The position is the part that SHOULD move:
            // it is what sells the jump or the stomp. Take it from the pin if
            // the pin is running, otherwise from your head, otherwise leave
            // the camera where the game wants it and only refuse the turn.
            float here[3] = { eye[0], eye[1], eye[2] };
            if (g_havePinnedEye) {
                std::memcpy(here, g_pinnedEye, sizeof(here));
            } else if (g_lastPlayerHead) {
                float head[3] = {};
                if (TryRead(head, g_lastPlayerHead + kOffJointWorldPos, sizeof(head)))
                    std::memcpy(here, head, sizeof(here));
            }
            asked[0] = target[0] - eye[0];
            asked[1] = target[1] - eye[1];
            asked[2] = target[2] - eye[2];
            if (len > 1e-3f && std::isfinite(here[0]) && std::isfinite(here[1]) && std::isfinite(here[2])) {
                for (int k = 0; k < 3; ++k) {
                    newEye[k] = here[k];
                    newTarget[k] = here[k] + keep[k];
                }
                mine = true;
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            mine = false;
        }
    }

    if (ours) {
        ++g_lookAtsCaught;
        // Capped, because if the premise is wrong and this fires every frame
        // the log has to stay readable enough to say so.
        if (g_lookAtsCaught <= 300) {
            static uintptr_t s_exe = 0;
            if (!s_exe)
                s_exe = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
            const float flat = std::sqrt(asked[0] * asked[0] + asked[2] * asked[2]);
            Log_Printf("LookAt: #%lu from exe+%X | the game asked for yaw %.0f pitch %.0f | %s",
                g_lookAtsCaught,
                static_cast<unsigned>(reinterpret_cast<uintptr_t>(from) - s_exe),
                std::atan2(asked[0], asked[2]) * 180.0f / kPi,
                flat > 1e-4f ? std::atan2(asked[1], flat) * 180.0f / kPi : 0.0f,
                mine ? "held your view instead" : "let through");
        }
    }
    if (mine) {
        g_origSetLookAt(self, edx, newEye, newTarget);
        return;
    }
    g_origSetLookAt(self, edx, eye, target);
}

// ---- Stopping the tug of war (2026-09-27) ---------------------------------
//
// The user's diagnosis, and it is the right one: "the body stutter is from us
// constantly forcing the camera to a spot, it no longer drifts around and is
// always trying to re-write to that root position."
//
// Overwriting an ACCUMULATOR is not the same as overriding a value the game
// recomputes from scratch. ApplyOverride works because the rig distances are
// rebuilt from their profile every frame, so our write is simply the last
// word. Here the game does pos += delta, and its next delta is very likely
// derived from where the camera currently is - so each frame we move it, and
// each frame it pulls back. A fight at frame rate, which is the stutter. It
// also explains why the symptom got worse as the pin got more accurate: the
// better we placed it, the further it was from where the game wanted it, and
// the harder the pull.
//
// So stop fighting and remove the other side. The delta arrives in xmm4, xmm5
// and xmm6 at exe+442714:
//
//     0F 28 C4                 movaps xmm0, xmm4
//     F3 0F 58 86 B0 03 00 00  addss  xmm0, [esi+3B0]
//     F3 0F 11 86 B0 03 00 00  movss  [esi+3B0], xmm0
//
// Zero those three registers and the add becomes a no-op: the camera stops
// moving on its own entirely, and the position we write each frame is the only
// thing placing it. Nothing to pull back, because nothing is pulling.
//
// Only while the pin is on. With it off the game is untouched, which is why
// this is safe to try - if it goes wrong, unticking the box restores the game
// completely rather than leaving something half-patched.
void* g_camDeltaTrampoline = nullptr;
bool g_camDeltaHooked = false;

// WHICH SITES ARE ACTUALLY IN (2026-09-28).
//
// The exe is packed and decrypts itself as it starts, so a page can still be
// non-executable at the moment we try to hook it. MinHook answers
// MH_ERROR_NOT_EXECUTABLE and the site is missing for the rest of the run -
// silently, because the install ran once and never looked again. These say
// what got in, so the install can be repeated until it is all there.
bool g_rigsReadyHooked = false;
bool g_skeletonHooked = false;
bool g_poseDoneHooked = false;
bool g_handReaderHooked = false;
bool g_viewMatrixHooked = false;
int g_installAttempt = 0;
unsigned long g_killDelta = 0; // read from the asm stub, so a plain long

// OURS ONLY (2026-09-27, user: "with the checkbox on, it bugs shadows out").
//
// That is the tell, and it says this routine is not the player camera's own.
// It is the generic camera update the game runs for EVERY camera it owns, the
// shadow camera included, so zeroing the movement delta stopped all of them
// moving rather than one. Shadows are rendered from a camera, so they broke
// first and most visibly - and the pin appeared to help nothing because we
// were quietly breaking things beside it the whole time.
//
// The object being updated is in esi, and the view hook already knows which
// one is the player's. So compare, and leave every other camera completely
// alone. A plain pointer beside the atomic because the stub reads it in asm.

__declspec(naked) void CamDeltaHook_Stub()
{
    __asm {
        pushfd
        push eax
        cmp g_killDelta, 0
        je passThrough
        mov eax, g_theCameraRaw
        test eax, eax
        je passThrough
        cmp esi, eax
        jne passThrough
        xorps xmm4, xmm4
        xorps xmm5, xmm5
        xorps xmm6, xmm6
    passThrough:
        pop eax
        popfd
        jmp g_camDeltaTrampoline
    }
}

// ---- Redirecting the calls to SetLookAt (2026-09-28) ---------------------
//
// A five byte call is E8 followed by a signed offset from the end of the
// instruction. Scan the decrypted code section for every one of those whose
// offset lands on exe+43A560 and rewrite the offset to land on our function
// instead. Same address, same five bytes, same length - the only thing that
// changes is where it goes, and nothing anywhere is relocated.
//
// The risk this carries is a false positive: an E8 that is not a call but
// happens to be followed by exactly the right four bytes. That is one chance
// in four billion per position, so roughly one in three hundred whole scans,
// and every site is logged with the bytes around it so a wrong one can be
// recognised on sight.
//
// Reversible: the original offsets are kept.
// A PASS-THROUGH THAT CANNOT GET THE CONVENTION WRONG (2026-09-28, after the
// redirect crashed the level load).
//
// The first attempt sent the call into a C function declared __fastcall with
// two stack arguments, on the strength of "SetLookAt(eye, target)". The crash
// says that guess was wrong: exe+43A592 read B81DC01D with exe+43C153 - the
// instruction after the first redirected call - on the stack, and not one
// LookAt line was logged first, so nothing had been substituted. A PURE
// pass-through crashed, which leaves only the shape of the call.
//
// All we actually proved about this function is its first two instructions:
// mov eax,[esp+4] / fld [eax], so argument one is a pointer to floats. How
// many arguments there are and who cleans them up was inferred, and the
// inference is what broke. It is also, in hindsight, exactly what killed the
// MinHook version: the same four-argument declaration, the same imbalance.
//
// So stop declaring it. This stub touches nothing: it saves every register,
// reports what it can see, restores everything and JUMPS to the real
// function. The real function then returns straight to the original caller
// and cleans up however it likes, because from its point of view the call
// arrived exactly as it always did. It cannot be wrong about a convention it
// never states.
//
// What it buys: which call site fires, when, and whether our camera is the
// one being placed. Plus the function is dumped byte for byte below, and its
// "ret" says how many arguments it really takes - after which the hold can be
// written against a fact instead of a guess.
// WHAT THE FUNCTION ACTUALLY TAKES (2026-09-28, read off its own bytes).
//
//   43A560  8B 44 24 04   mov eax,[esp+4]    arg 1, a float[3] -> [ecx+30]
//   43A564  D9 00         fld  [eax]
//   43A566  8A 54 24 14   mov dl,[esp+14]    arg 5, a byte
//   43A576  8B 44 24 08   mov eax,[esp+8]    arg 2, a float[3] -> [ecx+50]
//   43A58B  8B 44 24 0C   mov eax,[esp+0C]   arg 3, a float[3] -> [ecx+40]
//   43A592  D9 00         fld  [eax]
//
// Five arguments, not two. Declaring it with two is what crashed the level
// load at exactly 43A592: the third pointer was never pushed, so the function
// read whatever happened to be on the stack and dereferenced it. The MinHook
// version carried the same wrong declaration, which is why that crashed too -
// the detour was innocent all along.
//
// So keep the naked stub and never declare it. Arguments one and two are
// pointers the function is certain to read, which makes their stack slots
// safe to change; everything else is left exactly as the caller built it.
//
// NOTHING MOVES, NOT EVEN THE POSITION (2026-09-28, user: "the trainer
// freezes the camera completely. It does not allow any movement what so
// ever").
//
// I had the trainer backwards. Its "freeze melee camera" does not mean "hold
// your view steady while the action camera runs" - it means the melee camera
// never takes over in the first place. "It is like the melee hook never
// happens. I am free to move the camera up and down during the animation."
// Your ordinary camera stays in charge for the whole animation, which is why
// it worked so well with first person on: our collapsed rig was still the one
// placing the camera, exactly as it is the rest of the time.
//
// So this call is made a no-op rather than a softened one. Both vectors come
// back as whatever the camera already had, so the function writes the values
// that were already there and the action camera has no effect at all.
//
// Both slots are safe to change for the same reason: the function provably
// dereferences argument one and argument two. Argument three onward is left
// exactly as the caller built it.
uintptr_t g_realSetLookAt = 0;

float g_heldEye[3] = {};
float g_heldTarget[3] = {};
float* g_heldEyePtr = g_heldEye;
float* g_heldTargetPtr = g_heldTarget;
unsigned long g_holdTargetNow = 0; // read from the asm stub, so a plain long

void __cdecl LookAtSeen(void* self, const float* firstArg, const float* secondArg, const float* thirdArg,
    const void* from)
{
    g_holdTargetNow = 0;
    static uintptr_t s_exe = 0;
    if (!s_exe)
        s_exe = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    const unsigned where = static_cast<unsigned>(reinterpret_cast<uintptr_t>(from) - s_exe);
    const bool ours = self && g_theCameraRaw
        && static_cast<unsigned char*>(self) == static_cast<unsigned char*>(g_theCameraRaw) + 0x380;
    if (ours)
        ++g_lookAtsCaught;

    bool held = false;
    float kept[3] = {};
    if (ours && firstArg && secondArg && g_enabled && g_holdViewInMelee.load(std::memory_order_relaxed)) {
        unsigned char* block = static_cast<unsigned char*>(self);
        float was[3] = {}, wasAt[3] = {};
        if (TryRead(was, block + 0x30, sizeof(was)) && TryRead(wasAt, block + 0x50, sizeof(wasAt))) {
            const float dir[3] = { wasAt[0] - was[0], wasAt[1] - was[1], wasAt[2] - was[2] };
            const float len = Length3(dir);
            // SOME PLACEMENTS ARE REAL. A level load, a teleport or a cutscene
            // cut has to be allowed through, or the camera starts life at the
            // origin and stays there. Those are exactly the calls where the
            // camera is NOT already sitting on the player, so ask: is the
            // camera somewhere sensible right now? If it is, the game is
            // moving it away from you and that is the thing to refuse. If it
            // is not, the game is putting it somewhere for the first time and
            // that has to happen.
            bool campedOnHim = false;
            unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
            unsigned char* character = nullptr;
            float charPos[3] = {};
            if (pc && TryRead(&character, pc + kOffControllerCharacter, sizeof(character)) && character
                && TryRead(charPos, character + kOffCharacterPos, sizeof(charPos))) {
                const float d[3] = { was[0] - charPos[0], was[1] - charPos[1], was[2] - charPos[2] };
                campedOnHim = Length3(d) < 500.0f;
            }
            if (campedOnHim && len > 1e-3f && std::isfinite(was[0]) && std::isfinite(was[1])
                && std::isfinite(was[2])) {
                for (int k = 0; k < 3; ++k) {
                    g_heldEye[k] = was[k];
                    g_heldTarget[k] = wasAt[k];
                    kept[k] = was[k];
                }
                g_holdTargetNow = 1;
                held = true;
            }
        }
    }

    // Two of these sites may well fire every frame for every camera the game
    // owns, so say the first few from each and then only the ones that are
    // ours. Enough to see the shape without burying the log.
    static unsigned s_site[8] = {};
    static unsigned s_hits[8] = {};
    static int s_known = 0;
    int idx = -1;
    for (int i = 0; i < s_known; ++i) {
        if (s_site[i] == where) {
            idx = i;
            break;
        }
    }
    if (idx < 0 && s_known < 8) {
        idx = s_known++;
        s_site[idx] = where;
        s_hits[idx] = 0;
    }
    const unsigned hits = idx >= 0 ? ++s_hits[idx] : 0u;
    if (hits > 3 && !(ours && g_lookAtsCaught <= 100))
        return;

    float v[3] = {}, w[3] = {}, u[3] = {};
    const bool readable = firstArg && TryRead(v, firstArg, sizeof(v));
    const bool readable2 = secondArg && TryRead(w, secondArg, sizeof(w));
    const bool readable3 = thirdArg && TryRead(u, thirdArg, sizeof(u));
    unsigned char flag = 0xFF;
    unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (pc)
        TryRead(&flag, pc + kOffControllerActionCam, sizeof(flag));
    const float ask[3] = { w[0] - v[0], w[1] - v[1], w[2] - v[2] };
    const float flat = std::sqrt(ask[0] * ask[0] + ask[2] * ask[2]);
    Log_Printf("LookAt: exe+%X call #%u%s | action flag %d | eye %s(%.0f %.0f %.0f) | the game wanted yaw %.0f "
               "pitch %.0f | third vector %s(%.2f %.2f %.2f) | %s",
        where, hits, ours ? " - OUR CAMERA" : "", static_cast<int>(flag), readable ? "" : "unreadable ", v[0],
        v[1], v[2], readable2 ? std::atan2(ask[0], ask[2]) * 180.0f / kPi : -999.0f,
        (readable2 && flat > 1e-4f) ? std::atan2(ask[1], flat) * 180.0f / kPi : -999.0f,
        readable3 ? "" : "unreadable ", u[0], u[1], u[2],
        held ? "REFUSED, the camera does not move" : (ours ? "let through" : "not ours"));
    if (held)
        Log_Printf("LookAt:   left it where it was, at (%.0f %.0f %.0f)", kept[0], kept[1], kept[2]);
}

typedef void(__cdecl* OnLookAt_t)(void*, const float*, const float*, const float*, const void*);
OnLookAt_t g_onLookAt = &LookAtSeen;

__declspec(naked) void SetLookAtCallStub()
{
    __asm {
        pushad
        pushfd
        mov eax, [esp + 0x24]  // the return address: which call site this is
        mov ebx, [esp + 0x28]  // arg 1, the eye
        mov edx, [esp + 0x2C]  // arg 2, the target
        mov esi, [esp + 0x30]  // arg 3, whatever the third vector is
        push eax
        push esi
        push edx
        push ebx
        push ecx
        call g_onLookAt
        add esp, 20
        popfd
        popad
        // One slot, and only one the function is certain to read. Everything
        // else on this stack is exactly as the caller left it, which is why
        // the jump below cannot be wrong about a convention.
        cmp g_holdTargetNow, 0
        je letItThrough
        push eax
        mov eax, g_heldEyePtr
        mov [esp + 0x08], eax
        mov eax, g_heldTargetPtr
        mov [esp + 0x0C], eax
        pop eax
    letItThrough:
        jmp g_realSetLookAt
    }
}

namespace lookatcalls {

struct Site {
    unsigned char* at;
    int wasRel;
};
Site g_found[64];
int g_count = 0;

void FindAndRedirect()
{
    if (g_count)
        return;
    static bool s_complained = false;

    HMODULE exe = GetModuleHandleA(nullptr);
    if (!exe)
        return;
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(exe);
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(reinterpret_cast<unsigned char*>(exe) + dos->e_lfanew);
    auto* sec = IMAGE_FIRST_SECTION(nt);
    uintptr_t start = 0;
    size_t span = 0;
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
        if ((sec[i].Characteristics & IMAGE_SCN_MEM_EXECUTE) == 0)
            continue;
        start = reinterpret_cast<uintptr_t>(exe) + sec[i].VirtualAddress;
        span = sec[i].Misc.VirtualSize;
        break;
    }
    if (!start || !span)
        return;
    if (span > 0x1000000)
        span = 0x1000000;

    unsigned char* look = reinterpret_cast<unsigned char*>(reinterpret_cast<uintptr_t>(exe) + 0x43A560);
    static const unsigned char kLook[] = { 0x8B, 0x44, 0x24, 0x04, 0xD9, 0x00 };
    unsigned char saw[sizeof(kLook)] = {};
    if (!TryRead(saw, look, sizeof(saw)) || std::memcmp(saw, kLook, sizeof(kLook)) != 0) {
        if (!s_complained) {
            s_complained = true;
            Log_Printf("LookAtCalls: exe+43A560 holds %02X %02X %02X %02X %02X %02X, not SetLookAt - action "
                       "cameras stay the game\x27s",
                saw[0], saw[1], saw[2], saw[3], saw[4], saw[5]);
        }
        return;
    }

    const unsigned char* code = reinterpret_cast<const unsigned char*>(start);
    const uintptr_t want = reinterpret_cast<uintptr_t>(look);
    int found = 0;
    __try {
        for (size_t i = 0; i + 5 <= span; ++i) {
            if (code[i] != 0xE8)
                continue;
            int rel = 0;
            std::memcpy(&rel, code + i + 1, sizeof(rel));
            uintptr_t dest = start + i + 5;
            dest += static_cast<uintptr_t>(static_cast<intptr_t>(rel));
            if (dest != want)
                continue;
            ++found;
            if (g_count < 64) {
                g_found[g_count].at = const_cast<unsigned char*>(code + i);
                g_found[g_count].wasRel = rel;
                ++g_count;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        if (!s_complained) {
            s_complained = true;
            Log_Printf("LookAtCalls: could not read the code section");
        }
        g_count = 0;
        return;
    }

    if (!found) {
        if (!s_complained) {
            s_complained = true;
            Log_Printf("LookAtCalls: no direct call to exe+43A560 anywhere in %u bytes of code - it is reached "
                       "some other way",
                static_cast<unsigned>(span));
        }
        return;
    }

    // The bytes of the function itself, so its "ret" can be read off and the
    // argument count stops being a guess. C3 is ret with nothing to clean,
    // C2 xx 00 is ret with xx bytes of arguments cleaned by the callee.
    {
        char line[3 * 32 + 1] = {};
        for (int row = 0; row < 2; ++row) {
            int w = 0;
            for (int b = 0; b < 32; ++b)
                w += sprintf_s(line + w, sizeof(line) - w, "%02X ", look[row * 32 + b]);
            Log_Printf("LookAtCalls: exe+%X  %s", 0x43A560u + row * 32u, line);
        }
    }

    g_origSetLookAt = reinterpret_cast<SetLookAt_t>(look);
    g_realSetLookAt = reinterpret_cast<uintptr_t>(look);
    const uintptr_t mine = reinterpret_cast<uintptr_t>(&SetLookAtCallStub);
    int done = 0;
    for (int k = 0; k < g_count; ++k) {
        unsigned char* at = g_found[k].at;
        const int rel = static_cast<int>(static_cast<intptr_t>(mine - (reinterpret_cast<uintptr_t>(at) + 5)));
        DWORD old = 0;
        if (!VirtualProtect(at + 1, 4, PAGE_EXECUTE_READWRITE, &old))
            continue;
        std::memcpy(at + 1, &rel, sizeof(rel));
        DWORD back = 0;
        VirtualProtect(at + 1, 4, old, &back);
        FlushInstructionCache(GetCurrentProcess(), at, 5);
        ++done;
        Log_Printf("LookAtCalls: exe+%X redirected (before it: %02X %02X %02X, after it: %02X %02X %02X)",
            static_cast<unsigned>(reinterpret_cast<uintptr_t>(at) - reinterpret_cast<uintptr_t>(exe)),
            at[-3], at[-2], at[-1], at[5], at[6], at[7]);
    }
    g_lookAtHooked = done > 0;
    g_lookAtSiteCount = static_cast<unsigned long>(done);
    Log_Printf("LookAtCalls: %d call(s) to SetLookAt in the code, %d redirected%s", found, done,
        found > 64 ? " (only the first 64 were kept)" : "");
}

void PutItBack()
{
    for (int k = 0; k < g_count; ++k) {
        unsigned char* at = g_found[k].at;
        DWORD old = 0;
        if (!VirtualProtect(at + 1, 4, PAGE_EXECUTE_READWRITE, &old))
            continue;
        std::memcpy(at + 1, &g_found[k].wasRel, sizeof(int));
        DWORD back = 0;
        VirtualProtect(at + 1, 4, old, &back);
        FlushInstructionCache(GetCurrentProcess(), at, 5);
    }
    g_count = 0;
    g_lookAtHooked = false;
}

} // namespace lookatcalls

// ---- The melee camera, finally (2026-09-28) -----------------------------
//
// Found by asking the CPU instead of guessing. A hardware write watchpoint on
// the live camera eye, armed with End and followed immediately by a stomp,
// named three instructions in four seconds:
//
//   exe+44287F   296 writes  - once per frame, every frame. The known
//                              pipeline copy: fld [esi+640] / lea eax,[esi+30]
//                              / fstp [eax]. This is how the camera normally
//                              gets its position and it must not be touched.
//
//   exe+441286    81 writes  |  Roughly one second of frames each, out of a
//   exe+44152C    78 writes  |  four second window - which is exactly how
//                               long the stomp lasted. These two are the
//                               melee camera, and neither is a rig, a
//                               SetLookAt or anything else we have spent a
//                               day chasing.
//
// What they do, read off the bytes the watchpoint printed:
//
//   441271  52                    push edx            (a vector on the stack)
//   441272  E8 89 AD C7 FF        call exe+BC000
//   441277  85 C0                 test eax,eax
//   441279  74 21                 jz  +0x21           skip if it failed
//   44127B  F3 0F 10 44 24 60     movss xmm0,[esp+60]
//   441281  F3 0F 11 47 30        movss [edi+30],xmm0 the camera eye
//           ...and +34 and +38
//
// So: work out a position, and if that succeeded, put the camera there. The
// whole copy is already behind a conditional jump, which makes the smallest
// possible intervention a single byte - jz becomes jmp, the way the laser
// sight patch turns je into jmp. The game computes its melee position and
// then simply does not use it.
//
//   441522  F3 0F 11 47 34        movss [edi+34],xmm0
//   441527  F3 0F 11 57 30        movss [edi+30],xmm2
//   44152C  F3 0F 11 5F 38        movss [edi+38],xmm3
//
// The second has no conditional to borrow, so its three stores become
// fifteen nops. Nothing else in the routine changes: it still reads the
// camera target two instructions later and carries on.
//
// Verified before anything is written, and reversible byte for byte, exactly
// like the steady hands patch. If the bytes are not what this build should
// have, the feature reports itself unavailable and nothing is touched.
namespace meleecam {

// A FLAG WAS NEVER GOING TO DO IT (2026-09-28, user: "its still not quite
// there and fighting").
//
// controller+0x634 was on for 18 of 68 frames, then 43 of 73, then 24 of 76
// - about half of each animation. So the drift was killed on half the frames
// of a stomp and applied on the other half, which is precisely what being
// fought feels like. The flag says an action camera EXISTS, not that it is
// placing the camera this frame.
//
// There is a signal that does say that, and it is the instruction we are
// already standing on. exe+440325 runs on exactly the frames the melee
// camera places the camera, and on no others - that is how it was found. So
// instead of nopping it, call ourselves from it.
//
// The patch is still five bytes over one five byte instruction, so nothing
// is relocated. The stub notes the frame, restores everything, reproduces
// the one instruction in the skipped range that is not a store, and returns
// past the rest:
//
//   440320  F3 0F 59 47 30   mulss xmm0,[edi+30]     left alone
//   440325  F3 0F 11 4F 34   movss [edi+34],xmm1     becomes our call
//   44032A  F3 0F 11 57 38   movss [edi+38],xmm2     skipped
//   44032F  F3 0F 58 C3      addss xmm0,xmm3         done in the stub
//   440333  F3 0F 11 47 30   movss [edi+30],xmm0     skipped
//   440338                                           we return here
//
// Every xmm register is saved and restored around the call, because the C
// function is free to use all of them and the game needs them afterwards.
// WATCH FIRST, REFUSE SECOND (2026-09-28, user: "its fighting like hell").
//
// With the edi guard in, exe+440338 turned out to run for the PLAYER camera
// on most frames and often on every one - 71 of 71, 87 of 87, 90 of 90. So
// it is not the melee camera, it is an ordinary part of your camera pipeline
// that is merely busier during an action, and refusing it suppresses your
// normal camera placement most of the time. That is the fight.
//
// The bad inference was reading the 09:06 standing window, where this
// instruction was already nopped by us, as evidence that the GAME does not
// run it while standing. The absence was my own patch.
//
// So the stub now counts by default and refuses only when the box is ticked.
// Counting costs nothing and is a far better instrument than a four second
// watchpoint: it runs the whole session, only for our camera, and separates
// walking from stomping over minutes rather than seconds.
// AND THE ANSWER, FROM ONE ORDINARY SESSION (2026-09-28).
//
// Counting the blend for our camera alone, beside the action camera flag,
// second by second:
//
//   blend  8, flag  8      blend 77, flag 77
//   blend 49, flag 49      blend 98, flag 98
//   blend 41, flag 41      blend 82, flag 82
//
// Every frame the game is driving the camera, the blend runs. Exactly, never
// off by one. So it IS the mechanism - but there are also seconds reading
// "blend 96, flag 0", because the same blend eases the camera for ordinary
// work too: aiming pulls it over the shoulder, a turn swings it round. That
// is why refusing it wholesale fought everything.
//
// The flag was never flickering either. "18 of 68" was a one second bucket
// holding a third of a second of action, and I read a short animation as an
// unreliable signal.
//
// So the decision belongs at the moment of the call, on the game thread,
// where both facts are available at once: this is our camera AND the game is
// driving it. Anything else goes through untouched.
unsigned long long g_blendAtMs = 0;
unsigned long long g_refusedAtMs = 0;
unsigned long g_blendHits = 0;
unsigned long g_blendRefused = 0;
unsigned long g_refuseNow = 0; // read from the asm stub, so a plain long
unsigned long g_refuseBlend = 0; // whether the feature is on at all

void __cdecl NoteMeleeBlend()
{
    const unsigned long long now = GetTickCount64();
    g_blendAtMs = now;
    ++g_blendHits;
    // The stub has already established that this is our camera. The only
    // question left is whether the game is driving it, and this is the one
    // place and moment where asking is both cheap and exactly timed.
    const bool driving = g_refuseBlend != 0 && g_enabled && GameIsDrivingTheCamera();
    g_refuseNow = driving ? 1u : 0u;
    if (driving) {
        g_refusedAtMs = now;
        ++g_blendRefused;
    }
}

// True just after a frame on which we refused, so the accumulator can be
// held on the same frames and nothing drifts into the gap.
bool RefusedRecently()
{
    const unsigned long long now = GetTickCount64();
    return g_refusedAtMs && now >= g_refusedAtMs && now - g_refusedAtMs <= 100;
}

typedef void(__cdecl* OnBlend_t)();
OnBlend_t g_onMeleeBlend = &NoteMeleeBlend;

// OURS ONLY (2026-09-28, from the stub counting 57 calls a second while the
// user stood still pressing F4).
//
// exe+440325 is not melee-only. It runs every frame, for EVERY camera the
// game owns, and only during a melee does it run for the PLAYER camera -
// which is exactly why the watchpoint on camera+0x3B0 saw it 62 and 112
// times during stomps and not once while standing. Skipping it for every
// camera is the same mistake the camera delta hook already had to learn, and
// it went straight back in.
//
// edi is the +0x380 block of whichever camera is being placed, so ours is
// g_theCameraRaw + 0x380 and nothing else. Anything else gets its four
// instructions carried out here exactly as they were written and carries on
// untouched.
__declspec(naked) void MeleeBlendStub()
{
    __asm {
        push eax
        pushfd
        mov eax, g_theCameraRaw
        test eax, eax
        je notOurs
        add eax, 0x380
        cmp edi, eax
        jne notOurs
        popfd
        pop eax

        sub esp, 128
        movups [esp + 0x00], xmm0
        movups [esp + 0x10], xmm1
        movups [esp + 0x20], xmm2
        movups [esp + 0x30], xmm3
        movups [esp + 0x40], xmm4
        movups [esp + 0x50], xmm5
        movups [esp + 0x60], xmm6
        movups [esp + 0x70], xmm7
        pushad
        pushfd
        call g_onMeleeBlend
        popfd
        popad
        movups xmm0, [esp + 0x00]
        movups xmm1, [esp + 0x10]
        movups xmm2, [esp + 0x20]
        movups xmm3, [esp + 0x30]
        movups xmm4, [esp + 0x40]
        movups xmm5, [esp + 0x50]
        movups xmm6, [esp + 0x60]
        movups xmm7, [esp + 0x70]
        add esp, 128
        cmp g_refuseNow, 0
        je carryOn
        // The one instruction in the skipped range that is not a store.
        addss xmm0, xmm3
        // Return past the two stores we are refusing.
        add dword ptr [esp], 0x0E
        ret

    notOurs:
        popfd
        pop eax
    carryOn:
        // Do precisely what stood here, then the rest of the range we jump
        // over, and leave the game alone.
        movss dword ptr [edi + 0x34], xmm1
        movss dword ptr [edi + 0x38], xmm2
        addss xmm0, xmm3
        movss dword ptr [edi + 0x30], xmm0
        add dword ptr [esp], 0x0E
        ret
    }
}

// True while the melee camera is actually placing the camera, which is a
// different and much sharper question than "is an action camera running".
bool PlacingRightNow()
{
    const unsigned long long now = GetTickCount64();
    return g_blendAtMs && now >= g_blendAtMs && now - g_blendAtMs <= 100;
}

// AND AT LAST, THE SOURCE (2026-09-28, from watching all four links of the
// chain in one window).
//
// The camera is three copies of the same look-at struct - eye at +0x30,
// target at +0x50 - at camera+0x000, camera+0x380 and camera+0x610, copied
// downward once a frame:
//
//   exe+4427F4   fld [esi+3B0] / fstp [esi+640]    the eye,    380 -> 610
//   exe+44281F   fld [esi+3D0] / fstp [esi+660]    the target, 380 -> 610
//   exe+44287F   fld [esi+640] / fstp [esi+30]     the eye,    610 -> 000
//   exe+44289F   fld [esi+660] / fstp [esi+50]     the target, 610 -> 000
//
// Which is why refusing writes to +0x30 and +0x50 changed nothing, twice:
// they are two copies downstream of a decision already taken.
//
// At the head of it, two instructions write the eye at +0x3B0, and their
// counts SUM to the frame count - so every frame, one or the other places
// the camera and they are alternatives:
//
//                     uppercut window   stomp window   frames
//   exe+440084             181              118
//   exe+440338              62              112
//                          ---              ---        ---
//                          243              230        242 / 230
//
// The second one grew with the length of the action. And it is not a plain
// store, it is a blend:
//
//   440320  F3 0F 59 47 30   mulss xmm0,[edi+30]   t * where the camera is
//   440325  F3 0F 11 4F 34   movss [edi+34],xmm1
//   44032A  F3 0F 11 57 38   movss [edi+38],xmm2
//   44032F  F3 0F 58 C3      addss xmm0,xmm3       + where it is being sent
//   440333  F3 0F 11 47 30   movss [edi+30],xmm0
//
// A camera easing toward somewhere it has been told to go, running only
// while it has somewhere to go. That is the melee camera, and 0x2B4 further
// back in the same routine is the ordinary one.
//
// The three stores become nops. The blend still computes; it simply does not
// land, so +0x3B0 keeps whatever the ordinary path last put there and the
// whole chain below it carries that instead.
const unsigned char kACheck[] = { 0xF3, 0x0F, 0x59, 0x47, 0x30, 0xF3, 0x0F, 0x11, 0x4F, 0x34, 0xF3, 0x0F,
    0x11, 0x57, 0x38, 0xF3, 0x0F, 0x58, 0xC3, 0xF3, 0x0F, 0x11, 0x47, 0x30 };
const unsigned char kAWas[] = { 0xF3, 0x0F, 0x11, 0x4F, 0x34, 0xF3, 0x0F, 0x11, 0x57, 0x38, 0xF3, 0x0F, 0x58,
    0xC3, 0xF3, 0x0F, 0x11, 0x47, 0x30 };
// Filled in at install: E8 plus the offset from exe+44032A to our stub.
unsigned char kANow[] = { 0xE8, 0x00, 0x00, 0x00, 0x00 };

struct Site {
    const char* what;
    uintptr_t checkRva;
    const unsigned char* check;
    int checkLen;
    uintptr_t rva;
    const unsigned char* was;
    const unsigned char* now;
    int len;
};

const Site kSites[] = {
    { "the action camera blend at exe+440325", 0x440320, kACheck, sizeof(kACheck), 0x440325, kAWas, kANow,
        sizeof(kANow) },
};
constexpr int kCount = sizeof(kSites) / sizeof(kSites[0]);

uintptr_t g_base = 0;
bool g_available = false;
bool g_on = false;

bool Poke(unsigned char* at, const unsigned char* what, int len)
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

// The call goes in as soon as the bytes are recognised and stays in: it is
// how we count, and while nothing is being refused it does exactly what it
// replaced. The checkbox only decides whether the stores are skipped.
void EnsurePatched()
{
    if (!g_available || g_on)
        return;
    int done = 0;
    const bool on = true;
    for (int i = 0; i < kCount; ++i) {
        unsigned char* at = reinterpret_cast<unsigned char*>(g_base + kSites[i].rva);
        if (Poke(at, on ? kSites[i].now : kSites[i].was, kSites[i].len))
            ++done;
    }
    g_on = on;
    Log_Printf("MeleeCam: watching the camera blend at exe+440325 (%d site(s) written)", done);
}

// PINNED OFF WHILE WE IDENTIFY IT (2026-09-28). A stale HoldViewInMelee=1
// in re5vr.ini meant the previous "neutral" build was still refusing, which
// wasted a session and, worse, poisoned the measurement: a refused frame
// stops the camera moving, which changes whether the game blends on the next
// one, so the counts were measuring our own interference.
//
// A default cannot fix a value that is already written down. This can.
// OFF AGAIN (2026-09-28, user: "ours feels like a hacky mess, the trainer
// is exactly what we want").
//
// Refusing the blend on exactly the right frames still freezes the camera
// while the game goes on trying to pan it, and a freeze is not what the
// trainer does - under theirs the melee camera never engages at all and you
// keep full control of your own. Leaving our version switched on would also
// poison every measurement of theirs, since we would be holding the very
// bytes we are trying to watch them hold.
constexpr bool kRefusingAllowed = false;

void SetOn(bool refuse)
{
    EnsurePatched();
    if (!kRefusingAllowed) {
        static bool s_said = false;
        if (refuse && !s_said) {
            s_said = true;
            Log_Printf("MeleeCam: the checkbox is on but refusing is pinned off in this build - the blend is "
                       "being counted, not changed, so the numbers mean something");
        }
        g_refuseBlend = 0;
        return;
    }
    if ((g_refuseBlend != 0) == refuse)
        return;
    g_refuseBlend = refuse ? 1u : 0u;
    Log_Printf("MeleeCam: the camera blend is %s",
        refuse ? "refused on your camera while the game is driving it" : "left alone, counting only");
}

void Install(bool wanted)
{
    if (g_available)
        return;
    g_base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    if (!g_base)
        return;
    int matched = 0;
    for (int i = 0; i < kCount; ++i) {
        unsigned char seen[32] = {};
        const unsigned char* at = reinterpret_cast<const unsigned char*>(g_base + kSites[i].checkRva);
        if (!TryRead(seen, at, kSites[i].checkLen))
            continue;
        // Already patched counts as matched, in case a trainer got there
        // first - in that case we simply agree with it.
        unsigned char patched[32] = {};
        std::memcpy(patched, kSites[i].check, kSites[i].checkLen);
        std::memcpy(patched + (kSites[i].rva - kSites[i].checkRva), kSites[i].now, kSites[i].len);
        if (std::memcmp(seen, kSites[i].check, kSites[i].checkLen) == 0
            || std::memcmp(seen, patched, kSites[i].checkLen) == 0)
            ++matched;
    }
    if (matched != kCount)
        return; // said once by the caller, and try again next pass

    // The call can only be built once the module base is known.
    {
        const uintptr_t from = g_base + 0x440325 + 5;
        const int rel = static_cast<int>(
            static_cast<intptr_t>(reinterpret_cast<uintptr_t>(&MeleeBlendStub) - from));
        std::memcpy(kANow + 1, &rel, sizeof(rel));
    }
    g_available = true;
    Log_Printf("MeleeCam: %d of %d sites recognised - the melee camera can be refused", matched, kCount);
    SetOn(wanted);
}

bool IsAvailable()
{
    return g_available;
}

} // namespace meleecam

} // namespace

// Defined further down, beside the disassembly that explains it. EndScene
// calls it and EndScene comes first in the file.
void PinCameraToBody();

// Defined further down with the rest of the byte patches; the install has
// to reach it from up here.
namespace actioncam {
void Install();
void Set(int which, bool on);
bool Ready(int which);
}

void CameraRigHook_Install()
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("CameraRigHook_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    // This runs more than once now, so only the first pass narrates a
    // failure. A success always says so, whichever pass it lands on.
    ++g_installAttempt;
    const bool sayIt = g_installAttempt == 1;

    uintptr_t moduleBase = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));

    // KEEP ASKING, DO NOT GIVE UP (2026-09-28, from a session where the whole
    // camera system was simply absent).
    //
    // MinHook answered 7 here - MH_ERROR_NOT_EXECUTABLE - because the packer
    // had not yet decrypted the page this address lives on. The old code
    // logged one line and returned, which meant every hook below it never
    // went in either: no rig override, no skeleton, no arm IK, no view split,
    // no camera pin. First person was "on" only in the sense that the
    // checkbox said so, and a whole test session was spent reading vanilla
    // behaviour as if it were ours.
    //
    // Two seconds later the neighbouring page WAS executable - the culling
    // hook at exe+437C10 went in without complaint - so this is a race with
    // the unpacker, not a wrong address.
    if (!g_rigsReadyHooked) {
        void* rigsReadyTarget = reinterpret_cast<void*>(moduleBase + 0x446965);
        MH_STATUS rrSt
            = MH_CreateHook(rigsReadyTarget, reinterpret_cast<void*>(&RigsReadyHook_Stub), &g_rigsReadyTrampoline);
        if (rrSt == MH_OK || rrSt == MH_ERROR_ALREADY_CREATED)
            rrSt = MH_EnableHook(rigsReadyTarget);
        g_rigsReadyHooked = rrSt == MH_OK;
        if (g_rigsReadyHooked || sayIt)
            Log_Printf("CameraRigHook_Install: rigs-ready hook %s -> %d (target=%p, attempt %d)",
                g_rigsReadyHooked ? "enabled" : "NOT IN YET", static_cast<int>(rrSt), rigsReadyTarget,
                g_installAttempt);
    }

    {
        // 3DOF aiming patches the game in two places, and 3DOF aiming is not
        // finished (see ui/menu.cpp: no checkbox, no ini keys). A release has
        // no business patching game code for something nobody can switch on,
        // so these go in only when that work resumes.
        //
        // What the two sites are, for when it does: exe+7607B5 is right after
        // the game writes the character's pitch at +0x2DC8, and exe+776AA0 is
        // right after it writes the aim ray at +0x2980. Both were found by
        // watchpoint, both hold the character in esi, and both turned out to
        // be downstream of the real aim: +0x2DC8 is one line of a block copy
        // of camera floats (fld [edx+4B8] / fstp [ecx+2DC8]) and the ray is
        // written at the tail of its routine and read by nothing, proven by
        // forcing it skyward and watching nothing move.
        if (kInstallAimHooks) {
            void* pointWrite = reinterpret_cast<void*>(moduleBase + 0x776AA0);
            MH_STATUS ppSt
                = MH_CreateHook(pointWrite, reinterpret_cast<void*>(&AimPointHook_Stub), &g_aimPointTrampoline);
            if (ppSt == MH_OK || ppSt == MH_ERROR_ALREADY_CREATED)
                ppSt = MH_EnableHook(pointWrite);
            g_aimPointHooked = ppSt == MH_OK;

            void* aimWrite = reinterpret_cast<void*>(moduleBase + 0x7607B5);
            MH_STATUS apSt
                = MH_CreateHook(aimWrite, reinterpret_cast<void*>(&AimPitchHook_Stub), &g_aimWriteTrampoline);
            if (apSt == MH_OK || apSt == MH_ERROR_ALREADY_CREATED)
                apSt = MH_EnableHook(aimWrite);
            g_aimWriteHooked = apSt == MH_OK;
            Log_Printf("CameraRigHook_Install: aim hooks -> point %d, pitch %d", static_cast<int>(ppSt),
                static_cast<int>(apSt));
        }

        // NOT developer-only (2026-09-18): these two are how 6DOF works. They
        // were inside the diagnostics guard while they were being found, which
        // meant a release build installed neither and the arms could not hold a
        // pose no matter what the player ticked. The finders below stay guarded.
        // The skeleton build, so arm IK can write into the pose before the game
        // reads the hand to hang a weapon off it. Byte-checked only by logging
        // what is there: the exe is packed, so there is nothing on disk to
        // compare against, and a signature has to be taken from a live run.
        const XrInputSettings ikSettings = XrInput_GetSettings();
        {
            unsigned char* skel = reinterpret_cast<unsigned char*>(moduleBase + 0x183B29);
            unsigned char sig[8] = {};
            TryRead(sig, skel, sizeof(sig));
            bool hooked = false;
            // Always, not only when 6DOF is currently on (2026-09-18). Install
            // runs once at startup and the checkbox is ticked later, so gating it
            // on the setting meant 6DOF could never be switched on without a
            // relaunch. The callbacks do nothing while it is off.
            if (!g_skeletonHooked) {
                MH_STATUS skSt = MH_CreateHook(
                    skel, reinterpret_cast<void*>(&SkeletonHook_Stub), &g_skeletonTrampoline);
                if (skSt == MH_OK || skSt == MH_ERROR_ALREADY_CREATED)
                    skSt = MH_EnableHook(skel);
                hooked = skSt == MH_OK;
                g_skeletonHooked = hooked;
                if (hooked || sayIt)
                    Log_Printf("CameraRigHook_Install: skeleton build hook at exe+183B29 -> %s (%d), bytes "
                               "%02X %02X %02X %02X %02X %02X %02X %02X",
                        hooked ? "OK" : "FAILED", static_cast<int>(skSt), sig[0], sig[1], sig[2], sig[3],
                        sig[4], sig[5], sig[6], sig[7]);
                ArmIk_NoteSkeletonHooked(hooked);
            }
        }

        // The three places the animation finishes an arm rotation.
        if (!g_poseDoneHooked) {
            // One site (2026-09-18). The inner routine cannot be patched at all:
            // hooking exe+1A1A1C crashed the game on level start, and so did
            // exe+1A1893 on its own - same fault address, same return into
            // exe+1A059D, despite being a nine-byte movss with room to spare. So
            // it is not the instruction's shape; something in that routine cannot
            // survive a detour. exe+189E91, the outermost, has never crashed.
            const DWORD rvas[1] = { 0x189E91 };
            void* stubs[1] = { reinterpret_cast<void*>(&PoseDoneHook0_Stub) };
            for (int i = 0; i < 1; ++i) {
                unsigned char* at = reinterpret_cast<unsigned char*>(moduleBase + rvas[i]);
                unsigned char sig[6] = {};
                TryRead(sig, at, sizeof(sig));
                MH_STATUS st = MH_CreateHook(at, stubs[i], &g_poseDoneTrampoline[i]);
                if (st == MH_OK || st == MH_ERROR_ALREADY_CREATED)
                    st = MH_EnableHook(at);
                g_poseDoneHooked = st == MH_OK;
                if (g_poseDoneHooked || sayIt)
                    Log_Printf("CameraRigHook_Install: arm pose hook at exe+%lX -> %s (%d), bytes "
                               "%02X %02X %02X %02X %02X %02X",
                        static_cast<unsigned long>(rvas[i]), st == MH_OK ? "OK" : "FAILED",
                        static_cast<int>(st), sig[0], sig[1], sig[2], sig[3], sig[4], sig[5]);
            }
        }

#if RE5VR_DIAGNOSTICS
        // Whatever reads the hand once a frame, for the gun. Developer only for
        // now: it is one more mid-function hook on a packed exe and it buys
        // nothing yet beyond a pointer to look at.
        if (!g_handReaderHooked) {
            unsigned char* reader = reinterpret_cast<unsigned char*>(moduleBase + 0x19CB74);
            unsigned char sig[8] = {};
            TryRead(sig, reader, sizeof(sig));
            MH_STATUS hrSt = MH_CreateHook(
                reader, reinterpret_cast<void*>(&HandReaderHook_Stub), &g_handReaderTrampoline);
            if (hrSt == MH_OK || hrSt == MH_ERROR_ALREADY_CREATED)
                hrSt = MH_EnableHook(reader);
            g_handReaderHooked = hrSt == MH_OK;
            if (g_handReaderHooked || sayIt)
                Log_Printf("CameraRigHook_Install: hand-reader hook at exe+19CB74 -> %s (%d), bytes "
                           "%02X %02X %02X %02X %02X %02X %02X %02X",
                    hrSt == MH_OK ? "OK" : "FAILED", static_cast<int>(hrSt), sig[0], sig[1], sig[2], sig[3],
                    sig[4], sig[5], sig[6], sig[7]);
        }

        // The aim angles at their source. One instruction after the integrator
        // stores the pitch, where esi still holds the struct both angles live
        // in. Read-only, and developer builds only until it is doing something
        // worth a player's risk.
        if (!g_aimAnglesHooked) {
            void* aimAngles = reinterpret_cast<void*>(moduleBase + 0x7879B2);
            MH_STATUS aaSt
                = MH_CreateHook(aimAngles, reinterpret_cast<void*>(&AimAnglesHook_Stub), &g_aimAnglesTrampoline);
            if (aaSt == MH_OK || aaSt == MH_ERROR_ALREADY_CREATED)
                aaSt = MH_EnableHook(aimAngles);
            g_aimAnglesHooked = aaSt == MH_OK;
            if (g_aimAnglesHooked || sayIt)
                Log_Printf("CameraRigHook_Install: aim angle capture at exe+7879B2 -> %s (%d)",
                    g_aimAnglesHooked ? "OK" : "FAILED", static_cast<int>(aaSt));
        }
#endif
    }

    {
        // SetLookAt, which is how an action camera moves you. Signature
        // checked first: mov eax,[esp+4] / fld [eax] / fstp [ecx+30].
        // OFF AFTER A CRASH (2026-09-27, user: "game crashed when trying to load
        // a level"). 0xC0000005 at exe+43C1E6 reading E284003C, with exe+440641
        // on the stack - inside the camera code, a few hundred bytes from the
        // function we patched, dereferencing a garbage pointer.
        //
        // The find stands and is the right intervention point; the patch is what
        // is wrong. Two candidates worth checking before it goes back in:
        // something may jump into the middle of the six bytes MinHook relocated,
        // which a packed exe makes impossible to rule out by reading; or the
        // substituted look-at leaves the game holding state it derived from the
        // values it thought it passed.
        //
        // Everything else from tonight is independent of this and stays.
        // UNSAFE, PROVEN (2026-09-27). A hook that changed nothing crashed the
        // level load exactly as the substituting one did, so the five byte
        // detour at this address is the fault and the values never were.
        // Something jumps into the middle of the bytes MinHook has to
        // relocate, which a packed executable hides completely. This address
        // is still the right PLACE - it is simply not patchable this way.
        constexpr bool kHookSetLookAt = false;
        void* look = reinterpret_cast<void*>(moduleBase + 0x43A560);
        static const unsigned char kLook[] = { 0x8B, 0x44, 0x24, 0x04, 0xD9, 0x00 };
        unsigned char sawLook[sizeof(kLook)] = {};
        if (kHookSetLookAt && TryRead(sawLook, static_cast<unsigned char*>(look), sizeof(sawLook))
            && std::memcmp(sawLook, kLook, sizeof(kLook)) == 0) {
            MH_STATUS lSt = MH_CreateHook(look, reinterpret_cast<void*>(&SetLookAtHook),
                reinterpret_cast<void**>(&g_origSetLookAt));
            if (lSt == MH_OK || lSt == MH_ERROR_ALREADY_CREATED)
                lSt = MH_EnableHook(look);
            g_lookAtHooked = lSt == MH_OK;
            Log_Printf("CameraRigHook_Install: SetLookAt hook at exe+43A560 -> %s (%d)",
                g_lookAtHooked ? "OK" : "FAILED", static_cast<int>(lSt));
        } else if (sayIt) {
            Log_Printf("CameraRigHook_Install: exe+43A560 is %02X %02X %02X %02X %02X %02X, not SetLookAt - "
                       "action cameras will still move you",
                sawLook[0], sawLook[1], sawLook[2], sawLook[3], sawLook[4], sawLook[5]);
        }
    }

    // The call sites, which is the safe way into that same function. Repeated
    // passes cost nothing: it returns immediately once the sites are in.
    lookatcalls::FindAndRedirect();
    actioncam::Install();
    // MEASURED INERT (2026-09-28). With both sites suppressed, the watchpoint
    // over a stomp showed exe+44287F as the ONLY writer of the camera eye and
    // the melee camera moved exactly as before. So those 81 and 78 writes
    // were being overwritten by the per-frame copy anyway and refusing them
    // buys nothing. The finding is worth keeping written down; the patch is
    // not worth applying to a running game.
    meleecam::Install(g_holdViewInMelee.load(std::memory_order_relaxed));

    {
        // The camera's own movement, so the pin has nothing to fight. Checked
        // by signature first: this is a mid-function hook on a packed exe and
        // eleven bytes of it have to be exactly what the watchpoint showed.
        void* delta = reinterpret_cast<void*>(moduleBase + 0x442714);
        static const unsigned char kAdd[] = { 0x0F, 0x28, 0xC4, 0xF3, 0x0F, 0x58, 0x86, 0xB0, 0x03, 0x00, 0x00 };
        unsigned char seen[sizeof(kAdd)] = {};
        if (!g_camDeltaHooked && TryRead(seen, static_cast<unsigned char*>(delta), sizeof(seen))
            && std::memcmp(seen, kAdd, sizeof(kAdd)) == 0) {
            MH_STATUS dSt = MH_CreateHook(delta, reinterpret_cast<void*>(&CamDeltaHook_Stub), &g_camDeltaTrampoline);
            if (dSt == MH_OK || dSt == MH_ERROR_ALREADY_CREATED)
                dSt = MH_EnableHook(delta);
            g_camDeltaHooked = dSt == MH_OK;
            if (g_camDeltaHooked || sayIt)
                Log_Printf("CameraRigHook_Install: camera movement hook at exe+442714 -> %s (%d)",
                    g_camDeltaHooked ? "OK" : "FAILED", static_cast<int>(dSt));
        } else if (!g_camDeltaHooked && sayIt) {
            Log_Printf("CameraRigHook_Install: exe+442714 is %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X "
                       "%02X, not the camera add - the pin will have to fight it",
                seen[0], seen[1], seen[2], seen[3], seen[4], seen[5], seen[6], seen[7], seen[8], seen[9],
                seen[10]);
        }
    }

    {
        // GetViewMatrix: the view split. First bytes: push ebp / mov ebp,esp /
        // and esp,-16 (55 8B EC 83 E4 F0).
        void* gvm = reinterpret_cast<void*>(moduleBase + 0x435C20);
        static const unsigned char kExpected[] = { 0x55, 0x8B, 0xEC, 0x83, 0xE4, 0xF0 };
        unsigned char actual[sizeof(kExpected)] = {};
        if (!g_viewMatrixHooked && TryRead(actual, reinterpret_cast<unsigned char*>(gvm), sizeof(actual)) &&
            std::memcmp(actual, kExpected, sizeof(kExpected)) == 0) {
            MH_STATUS gSt = MH_CreateHook(gvm, reinterpret_cast<void*>(&GetViewMatrixHook), reinterpret_cast<void**>(&g_origGetViewMatrix));
            if (gSt == MH_OK || gSt == MH_ERROR_ALREADY_CREATED)
                gSt = MH_EnableHook(gvm);
            g_viewMatrixHooked = gSt == MH_OK;
            if (g_viewMatrixHooked || sayIt)
                Log_Printf("CameraRigHook_Install: GetViewMatrix view-split hook -> %d (target=%p)",
                    static_cast<int>(gSt), gvm);
        } else if (!g_viewMatrixHooked && sayIt) {
            Log_Printf("CameraRigHook_Install: GetViewMatrix has unexpected bytes - view split unavailable, culling "
                       "follows the game camera");
        }
    }

    if (!kInstallLegacyWriteHooks)
        return;

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

// ---- Keeping at it until the hooks are in (2026-09-28) -------------------
//
// Called once a frame. While any site is still missing it runs the install
// again, half a second apart, for thirty seconds. Every site is written to
// skip itself once it is in, so a repeat pass only touches what is absent and
// costs nothing once everything is there.
//
// Thirty seconds is chosen to cover the main menu: the packer finishes long
// before that, and if a site is still missing by then the address is wrong
// rather than early, which is worth saying out loud exactly once.
void CameraRigHook_RetryInstall()
{
    static bool s_settled = false;
    if (s_settled)
        return;

    const bool allIn = g_rigsReadyHooked && g_skeletonHooked && g_poseDoneHooked && g_camDeltaHooked
        && g_viewMatrixHooked;
    if (allIn) {
        s_settled = true;
        if (g_installAttempt > 1)
            Log_Printf("CameraRigHook: every hook is in after %d attempt(s) - the packer just needed a moment",
                g_installAttempt);
        return;
    }

    static unsigned long long s_nextAt = 0;
    const unsigned long long now = GetTickCount64();
    if (s_nextAt && now < s_nextAt)
        return;
    s_nextAt = now + 500;

    if (g_installAttempt >= 60) {
        s_settled = true;
        Log_Printf("CameraRigHook: giving up after %d attempt(s) - rigs-ready %d, skeleton %d, arm pose %d, "
                   "camera movement %d, view split %d. What is missing is missing because the address is "
                   "wrong on this build, not because it was early.",
            g_installAttempt, g_rigsReadyHooked ? 1 : 0, g_skeletonHooked ? 1 : 0, g_poseDoneHooked ? 1 : 0,
            g_camDeltaHooked ? 1 : 0, g_viewMatrixHooked ? 1 : 0);
        return;
    }

    CameraRigHook_Install();
}

// ONCE A FRAME (2026-09-27, from the F8 window during a jump down).
//
// The watchpoint counted twelve thousand writes into camera+3B0 in four
// seconds, from an address outside the executable - which is this code. The
// pin was being called from CameraRigHook_OnEndScene, and this game calls
// EndScene about thirty-four times for every frame it presents. So the camera
// was being yanked onto your neck thirty-odd times a frame, at arbitrary
// points during the game's own camera work, each one landing at a different
// stage of it.
//
// That is a much better explanation of a stutter that is worst while walking
// and present while idling than anything I proposed for it, and it is the
// kind of thing only a watchpoint would have shown - the code looks perfectly
// correct at the call site.
// ---- Watching a whip happen (2026-09-27) ---------------------------------
//
// "I think in order to tackle the camera whip, we need to figure out why it is
// happening to begin with." Correct, and it is where this should have gone
// three interceptions ago. Every attempt so far has been to stop something
// inferred rather than observed: we have never once looked at what the camera
// actually does, frame by frame, while it throws you.
//
// So this records it. Scroll Lock opens a two second window - the key already
// exists here for exactly this purpose and does nothing in the game - and
// every presented frame inside it writes one line: where the camera sits,
// where it is looking, and where your head is.
//
// What the shape of those numbers will tell us, without any more guessing:
//   - the target jumping while the eye holds -> the game is re-aiming, and
//     the whip is a look-at change
//   - the eye swinging round while the target holds -> it is orbiting you
//   - both moving together -> the whole camera is being flown somewhere
//   - neither moving while you still get thrown -> it is not this camera at
//     all, and everything we have done to it was always going to be wasted
//
// Those four want completely different fixes and we have been unable to
// choose between them all night.
void TraceTheCamera()
{
    const unsigned long long now = GetTickCount64();
    if (!g_actionTraceUntilMs || now > g_actionTraceUntilMs)
        return;
    unsigned char* camera = static_cast<unsigned char*>(g_theCamera.load(std::memory_order_relaxed));
    if (!camera)
        return;

    float eye[3] = {}, target[3] = {}, head[3] = {};
    bool haveHead = false;
    __try {
        std::memcpy(eye, camera + 0x30, sizeof(eye));
        std::memcpy(target, camera + 0x50, sizeof(target));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (g_lastPlayerHead)
        haveHead = TryRead(head, g_lastPlayerHead + kOffJointWorldPos, sizeof(head));

    const float look[3] = { target[0] - eye[0], target[1] - eye[1], target[2] - eye[2] };
    const float yaw = std::atan2(look[0], look[2]) * 180.0f / kPi;
    const float flat = std::sqrt(look[0] * look[0] + look[2] * look[2]);
    const float pitch = flat > 1e-4f ? std::atan2(look[1], flat) * 180.0f / kPi : 0.0f;
    Log_Printf("Whip: view from aim %lu / headlock %lu / the game %lu | eye (%.0f %.0f %.0f) looking %.0f yaw "
               "%.0f pitch, %.0f from the target; head (%.0f %.0f %.0f), %.0f from the eye",
        g_viewFromAim, g_viewFromHeadLock, g_viewFromGame,
        eye[0], eye[1], eye[2], yaw, pitch, Length3(look), head[0], head[1], head[2],
        haveHead ? std::sqrt((eye[0] - head[0]) * (eye[0] - head[0]) + (eye[1] - head[1]) * (eye[1] - head[1])
                       + (eye[2] - head[2]) * (eye[2] - head[2]))
                 : -1.0f);
    g_viewFromAim = 0;
    g_viewFromHeadLock = 0;
    g_viewFromGame = 0;
}

// ---- Finding the action camera flag (2026-09-27) -------------------------
//
// "Is there anything at all that triggers an action camera flag?" Not one we
// have found, but the game certainly has the state: exe+43A560 is a SetLookAt
// that fires three times during a stomp and never once in ordinary play. It
// knows. We just cannot hook that function safely on a packed exe.
//
// A flag is the better target anyway, and there is a precedent that says one
// exists: "the gun is up" lives in a single byte at controller+0x1B1, found
// the same way this will be.
//
// The method is the one that has worked twice tonight - narrow by
// elimination. End takes a snapshot of the player controller. Press it during
// ordinary play, then again in the middle of a stomp, and the candidates are
// the bytes that differ. Press it again in ordinary play and the candidates
// shrink to those that changed back. A handful of rounds and what is left is
// the flag, because nothing else in the structure follows that pattern.
//
// Read only. It never writes anything.
// ---- The melee rig (2026-09-27) ------------------------------------------
//
// From a screenshot of the trainer: it offers Horizontal Distance, Vertical
// Distance, Distance, Horizontal Adjustment, Vertical Adjustment, Angle and
// Field of View - which is exactly the seven field rig this mod has driven
// since first person worked - and separate freezes for "cam 1", "cam 2" and
// "melee camera".
//
// So the melee camera is another rig of the same shape, and we hook two of
// them: normal at controller+0x480 and aim at controller+0x3F0. ApplyOverride
// has never seen the melee one, which is why melee escapes every single thing
// we have built tonight. Collapse its distances the way the others are
// collapsed and it sits in your body by construction - no anchor, no
// threshold, no guessing about whether you twitched the mouse.
//
// Finding it needs no watchpoint because a rig announces itself: a field of
// view in degrees at +0x24 with plausible distances around it. The detector is
// validated by the two we already know - if it does not report 0x3F0 and
// 0x480 among its finds, it is wrong and nothing else it says can be trusted.
// ---- What did the trainer patch? (2026-09-27) ----------------------------
//
// The trainer is packed - not a readable string in it - so reading its code is
// a detour. It does not matter. It patches the GAME, and we are inside the
// game, so we can simply look at what it changed.
//
// "With the trainer freeze on, it is like the melee hook never happens. I am
// free to move the camera up and down during the animation." That is not a
// value being clamped, it is the melee camera never engaging, which means a
// code patch. A code patch is a handful of bytes that differ.
//
// So: snapshot the game's code with the freeze OFF, toggle it ON, snapshot
// again, and the difference is the answer - the exact address and the exact
// bytes. Whatever they wrote, we can write.
//
// Read only, and it never touches the trainer.
namespace codediff {

unsigned char* g_before = nullptr;
size_t g_span = 0;
uintptr_t g_base = 0;
bool g_have = false;

// OUR OWN HANDWRITING (2026-09-28, user: "remember they will disable our
// first person camera hook, so we will want to dance around that").
//
// This mod patches the game in a dozen places and hooks it in a dozen more.
// If anything of ours goes in or out between the two snapshots - and running
// a trainer alongside us is exactly the situation where it might - the diff
// fills up with our own work and buries theirs. So every address we are
// known to write is listed here and labelled in the report. Anything NOT on
// this list is somebody else.
struct Mine {
    unsigned rva;
    unsigned len;
    const char* what;
};
const Mine kMine[] = {
    { 0x440325, 5, "our melee blend call" },
    { 0x43C14E, 5, "our SetLookAt call redirect" },
    { 0x4426F1, 5, "our SetLookAt call redirect" },
    { 0x20DB47, 2, "our steady hands" },
    { 0x76E78D, 6, "our steady hands" },
    { 0x76E80E, 6, "our steady hands" },
    { 0x787227, 8, "our steady hands" },
    { 0x769A91, 6, "our forced laser sight" },
    { 0x776DC1, 6, "our forced laser sight" },
    { 0x777151, 6, "our forced laser sight" },
    { 0x3C2FF7, 8, "our colour filter removal" },
    { 0x5F8D19, 8, "our HUD probe" },
    { 0x5F8EB0, 8, "our HUD probe" },
    { 0x5F9125, 8, "our HUD probe" },
    { 0x446965, 8, "our rigs-ready hook" },
    { 0x183B29, 8, "our skeleton hook" },
    { 0x189E91, 8, "our arm pose hook" },
    { 0x19CB74, 8, "our hand reader hook" },
    { 0x7879B2, 8, "our aim angle hook" },
    { 0x442714, 12, "our camera movement hook" },
    { 0x435C20, 8, "our GetViewMatrix hook" },
    { 0x835A90, 8, "our culling hook" },
    { 0x437C10, 8, "our frustum hook" },
    { 0xB576C0, 8, "our fade hook" },
    { 0xB55410, 8, "our fade hook" },
};

const char* WhoseIsThis(unsigned rva)
{
    for (const Mine& m : kMine) {
        if (rva + 16 > m.rva && rva < m.rva + m.len + 16)
            return m.what;
    }
    return nullptr;
}

void Look()
{
    HMODULE exe = GetModuleHandleA(nullptr);
    if (!exe) {
        Log_Printf("CodeDiff: no module");
        return;
    }
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(exe);
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(reinterpret_cast<unsigned char*>(exe) + dos->e_lfanew);
    auto* sec = IMAGE_FIRST_SECTION(nt);
    uintptr_t start = 0;
    size_t span = 0;
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
        if ((sec[i].Characteristics & IMAGE_SCN_MEM_EXECUTE) == 0)
            continue;
        start = reinterpret_cast<uintptr_t>(exe) + sec[i].VirtualAddress;
        span = sec[i].Misc.VirtualSize;
        break; // the first executable section is the code
    }
    if (!start || !span) {
        Log_Printf("CodeDiff: no executable section found");
        return;
    }
    if (span > 0x1000000)
        span = 0x1000000;

    if (!g_have) {
        if (g_before) {
            free(g_before);
            g_before = nullptr;
        }
        g_before = static_cast<unsigned char*>(malloc(span));
        if (!g_before) {
            Log_Printf("CodeDiff: out of memory for %u bytes", static_cast<unsigned>(span));
            return;
        }
        __try {
            std::memcpy(g_before, reinterpret_cast<void*>(start), span);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            Log_Printf("CodeDiff: could not read the code section");
            return;
        }
        g_span = span;
        g_base = start;
        g_have = true;
        Log_Printf("CodeDiff: %u bytes of code recorded. Now TOGGLE the trainer option and press End again.",
            static_cast<unsigned>(span));
        return;
    }

    if (start != g_base || span != g_span) {
        Log_Printf("CodeDiff: the module moved - starting over");
        g_have = false;
        return;
    }
    const unsigned char* now = reinterpret_cast<const unsigned char*>(start);
    int runs = 0;
    __try {
        for (size_t i = 0; i < g_span; ++i) {
            if (now[i] == g_before[i])
                continue;
            // Report the whole run of changed bytes, which is the patch.
            size_t j = i;
            char was[64] = {}, is[64] = {};
            int w = 0, s = 0;
            while (j < g_span && j - i < 16 && now[j] != g_before[j]) {
                w += sprintf_s(was + w, sizeof(was) - w, "%02X ", g_before[j]);
                s += sprintf_s(is + s, sizeof(is) - s, "%02X ", now[j]);
                ++j;
            }
            // The offset plus the section base, which is what "exe+" means
            // everywhere else in this project. Reporting the raw index sent
            // the hand tremor addresses a page adrift.
            const unsigned rva
                = static_cast<unsigned>(i + (g_base - reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr))));
            const char* mine = WhoseIsThis(rva);
            Log_Printf("CodeDiff: exe+%X was [%s] now [%s]%s%s", rva, was, is, mine ? "   <- IGNORE, " : "",
                mine ? mine : "");
            i = j;
            if (++runs >= 24) {
                Log_Printf("CodeDiff: ... and more, stopping at 24");
                break;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        Log_Printf("CodeDiff: fault while comparing");
        return;
    }
    if (!runs)
        Log_Printf("CodeDiff: nothing in the code changed - whatever it does, it is not a code patch");
    else
        Log_Printf("CodeDiff: %d run(s) of changed bytes. Anything not marked IGNORE is not ours.", runs);
    // Whatever we just saw becomes the new baseline, so toggling back and
    // forth keeps reporting the same patch from both directions.
    std::memcpy(g_before, now, g_span);
}

} // namespace codediff

namespace meleerig {

bool LooksLikeARig(const unsigned char* at)
{
    float f[13];
    std::memcpy(f, at, sizeof(f));
    for (int i = 0; i < 13; ++i) {
        if (f[i] != f[i])
            return false; // NaN is not a rig
    }
    const float fov = f[9]; // +0x24
    if (!(fov > 20.0f && fov < 130.0f))
        return false;
    // The three distances and two adjustments: real values, not wild ones.
    const int spots[5] = { 0, 1, 2, 4 };
    for (int s = 0; s < 4; ++s) {
        const float v = f[spots[s]];
        if (!(v > -4000.0f && v < 4000.0f))
            return false;
    }
    return true;
}

void Hunt()
{
    unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (!pc) {
        Log_Printf("MeleeRig: no player controller yet");
        return;
    }
    constexpr int kSpan = 0x1200;
    char list[512];
    int w = 0;
    int found = 0;
    __try {
        for (int off = 0; off + 64 <= kSpan; off += 4) {
            if (!LooksLikeARig(pc + off))
                continue;
            ++found;
            if (w < static_cast<int>(sizeof(list)) - 48) {
                const float* f = reinterpret_cast<const float*>(pc + off);
                w += sprintf_s(list + w, sizeof(list) - w, "%s+%X(d %.0f/%.0f/%.0f fov %.0f)", w ? ", " : "",
                    off, f[0], f[1], f[2], f[9]);
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        Log_Printf("MeleeRig: could not read the controller");
        return;
    }
    Log_Printf("MeleeRig: %d rig-shaped block(s): %s", found, found ? list : "none");
    Log_Printf("MeleeRig: the known ones are +3F0 (aim) and +480 (normal) - if those are not in that list the "
               "detector is wrong. Press End again DURING a melee and compare.");
}

} // namespace meleerig

namespace actionflag {

constexpr int kSpan = 0x2000;
constexpr int kMaxCandidates = 4096;

unsigned char g_quiet[kSpan];
bool g_haveQuiet = false;
int g_cand[kMaxCandidates];
int g_candCount = 0;
bool g_narrowing = false;
int g_rounds = 0;

void Snapshot()
{
    unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (!pc) {
        Log_Printf("ActionFlag: no player controller yet");
        return;
    }
    static unsigned char now[kSpan];
    __try {
        std::memcpy(now, pc, sizeof(now));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        Log_Printf("ActionFlag: could not read the controller");
        return;
    }

    if (!g_haveQuiet) {
        std::memcpy(g_quiet, now, sizeof(g_quiet));
        g_haveQuiet = true;
        g_narrowing = false;
        g_rounds = 0;
        Log_Printf("ActionFlag: quiet state recorded. Now press End again DURING an action camera.");
        return;
    }

    if (!g_narrowing) {
        // Second press: during the action. Anything that differs is a
        // candidate.
        g_candCount = 0;
        for (int i = 0; i < kSpan && g_candCount < kMaxCandidates; ++i) {
            if (now[i] != g_quiet[i])
                g_cand[g_candCount++] = i;
        }
        g_narrowing = true;
        g_rounds = 1;
        Log_Printf("ActionFlag: %d byte(s) differ during the action. Press End again in ORDINARY play.",
            g_candCount);
        return;
    }

    // Alternating from here: in ordinary play a real flag must read its quiet
    // value again, and during an action it must differ again.
    ++g_rounds;
    const bool expectQuiet = (g_rounds % 2) == 0;
    int kept = 0;
    for (int i = 0; i < g_candCount; ++i) {
        const int at = g_cand[i];
        const bool same = now[at] == g_quiet[at];
        if (same == expectQuiet)
            g_cand[kept++] = at;
    }
    g_candCount = kept;
    if (g_candCount <= 12) {
        char list[256];
        int w = 0;
        for (int i = 0; i < g_candCount && w < static_cast<int>(sizeof(list)) - 16; ++i)
            w += sprintf_s(list + w, sizeof(list) - w, "%s+%X(%02X vs %02X)", i ? ", " : "", g_cand[i],
                g_quiet[g_cand[i]], now[g_cand[i]]);
        Log_Printf("ActionFlag: round %d leaves %d - %s", g_rounds, g_candCount, g_candCount ? list : "nothing");
    } else {
        Log_Printf("ActionFlag: round %d leaves %d candidate(s) - press End again in %s", g_rounds, g_candCount,
            expectQuiet ? "an ACTION" : "ORDINARY play");
    }
    if (!g_candCount) {
        Log_Printf("ActionFlag: nothing survived - starting over, press End in ordinary play");
        g_haveQuiet = false;
    }
}

} // namespace actionflag

// ---- What the trainer holds (2026-09-28) --------------------------------
//
// Eleven megabytes of executable, compared across their melee freeze in both
// directions: nothing in the code changed. The same differ found four byte
// patches for their Hand Tremors Fix in this very game, so it works - this
// feature simply is not one. They hold a value.
//
// So run the same narrowing that found controller+0x634, but keyed on the
// TRAINER TOGGLE instead of on an action, and over the camera object as well
// as the player controller, because there is no reason to assume it lives in
// the one we happen to have looked at before.
//
// A value someone holds from outside has a signature nothing else has: it
// reads one way every time the option is on and another way every time it is
// off, round after round. Everything that merely moves - positions, timers,
// animation - fails that within two or three presses.
//
// Stand still while doing this. The fewer things that are genuinely changing,
// the faster it narrows.
namespace trainerdiff {

constexpr int kCamSpan = 0x1000;
constexpr int kPcSpan = 0x2000;
constexpr int kMax = 8192;

unsigned char g_offCam[kCamSpan];
unsigned char g_offPc[kPcSpan];
bool g_haveOff = false;
bool g_narrowing = false;
int g_rounds = 0;

struct Cand {
    unsigned char region; // 0 camera, 1 controller
    unsigned short off;
};
Cand g_cand[kMax];
int g_count = 0;

unsigned char g_nowCam[kCamSpan];
unsigned char g_nowPc[kPcSpan];

unsigned char Was(const Cand& c)
{
    return c.region == 0 ? g_offCam[c.off] : g_offPc[c.off];
}

unsigned char Now(const Cand& c)
{
    return c.region == 0 ? g_nowCam[c.off] : g_nowPc[c.off];
}

void Look()
{
    unsigned char* cam = static_cast<unsigned char*>(g_theCameraRaw);
    unsigned char* pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (!cam || !pc) {
        Log_Printf("TrainerDiff: no camera or no player controller yet - load a level first");
        return;
    }
    __try {
        std::memcpy(g_nowCam, cam, sizeof(g_nowCam));
        std::memcpy(g_nowPc, pc, sizeof(g_nowPc));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        Log_Printf("TrainerDiff: could not read them");
        return;
    }

    if (!g_haveOff) {
        std::memcpy(g_offCam, g_nowCam, sizeof(g_offCam));
        std::memcpy(g_offPc, g_nowPc, sizeof(g_offPc));
        g_haveOff = true;
        g_narrowing = false;
        g_rounds = 0;
        Log_Printf("TrainerDiff: recorded with the trainer option OFF (camera %p, controller %p). Turn it ON "
                   "and press F4 again.",
            cam, pc);
        return;
    }

    if (!g_narrowing) {
        g_count = 0;
        for (int i = 0; i < kCamSpan && g_count < kMax; ++i) {
            if (g_nowCam[i] != g_offCam[i]) {
                g_cand[g_count].region = 0;
                g_cand[g_count].off = static_cast<unsigned short>(i);
                ++g_count;
            }
        }
        for (int i = 0; i < kPcSpan && g_count < kMax; ++i) {
            if (g_nowPc[i] != g_offPc[i]) {
                g_cand[g_count].region = 1;
                g_cand[g_count].off = static_cast<unsigned short>(i);
                ++g_count;
            }
        }
        g_narrowing = true;
        g_rounds = 1;
        Log_Printf("TrainerDiff: %d byte(s) differ with it ON. Turn it OFF and press F4 again.", g_count);
        return;
    }

    // Alternating: with the option off a held value must read its off value
    // again, and with it on it must differ again. Nothing that is merely
    // moving survives many rounds of that.
    ++g_rounds;
    const bool expectOff = (g_rounds % 2) == 0;
    int kept = 0;
    for (int i = 0; i < g_count; ++i) {
        const bool same = Now(g_cand[i]) == Was(g_cand[i]);
        if (same == expectOff)
            g_cand[kept++] = g_cand[i];
    }
    g_count = kept;

    if (g_count && g_count <= 16) {
        char list[512];
        int w = 0;
        for (int i = 0; i < g_count && w < static_cast<int>(sizeof(list)) - 32; ++i)
            w += sprintf_s(list + w, sizeof(list) - w, "%s%s+%X(%02X off, %02X on)", i ? ", " : "",
                g_cand[i].region == 0 ? "camera" : "controller", g_cand[i].off, Was(g_cand[i]),
                Now(g_cand[i]));
        Log_Printf("TrainerDiff: round %d leaves %d - %s", g_rounds, g_count, list);
    } else {
        Log_Printf("TrainerDiff: round %d leaves %d candidate(s) - press F4 again with it %s", g_rounds,
            g_count, expectOff ? "ON" : "OFF");
    }
    if (!g_count) {
        Log_Printf("TrainerDiff: nothing survived - starting over, press F4 with the option OFF");
        g_haveOff = false;
    }
}

} // namespace trainerdiff

// ---- What stops moving (2026-09-28) -------------------------------------
//
// "However the trainer handles it is beautiful, especially cause you can
// still move the camera during an animation. We try to hard lock it, and
// then its still fighting the entire time."
//
// That sentence is the whole diagnosis. Under the trainer the melee camera
// never engages, so the ORDINARY camera is still in charge and still answers
// the mouse. We freeze instead, which is a different thing and will always
// feel like a fight because the game is still trying to pan.
//
// Both earlier differs came back empty, and I now think they were asked in
// the wrong place: standing in ordinary play, where the melee camera is
// dormant and there is nothing for a trainer to be holding. A value frozen
// only while the animation runs is invisible until the animation runs.
//
// So ask a different question, one that needs no toggling between sessions
// and no values to match across runs:
//
//   WHAT MOVES DURING A MELEE WITH THEIR FREEZE OFF, AND STOPS MOVING WITH
//   IT ON?
//
// Two samples 300 ms apart inside one animation give the set of bytes that
// changed. Do that once with their freeze off and once with it on, and
// subtract. Whatever is in the first set and not the second is precisely
// what they are holding still - which is the definition of a freeze, and it
// survives the fact that no two stomps are ever in the same place.
//
// Read only. It never writes anything.
namespace freezefind {

constexpr int kCam = 0x1000;
constexpr int kPc = 0x2000;
constexpr int kAll = kCam + kPc;

unsigned char g_s1[kAll];
unsigned char g_s2[kAll];
unsigned char g_offS1[kAll];
unsigned char g_offS2[kAll];
bool g_moves[kAll];
bool g_movesOff[kAll];
bool g_answer[kAll];
bool g_haveOff = false;
bool g_haveAnswer = false;
int g_pairs = 0;

unsigned char* g_cam = nullptr;
unsigned char* g_pc = nullptr;
bool g_pending = false;
bool g_flagAtStart = false;
unsigned long long g_dueAt = 0;

bool Grab(unsigned char* into)
{
    __try {
        std::memcpy(into, g_cam, kCam);
        std::memcpy(into + kCam, g_pc, kPc);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
    return true;
}

void Where(int i, char* out, size_t n)
{
    if (i < kCam)
        sprintf_s(out, n, "camera+%X", i);
    else
        sprintf_s(out, n, "controller+%X", i - kCam);
}

void Start()
{
    g_cam = static_cast<unsigned char*>(g_theCameraRaw);
    g_pc = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (!g_cam || !g_pc) {
        Log_Printf("FreezeFind: no camera or player controller yet");
        return;
    }
    if (g_pending) {
        Log_Printf("FreezeFind: already sampling, wait a moment");
        return;
    }
    if (!Grab(g_s1)) {
        Log_Printf("FreezeFind: could not read them");
        return;
    }
    g_flagAtStart = GameIsDrivingTheCamera();
    g_pending = true;
    g_dueAt = GetTickCount64() + 300;
}

void Tick()
{
    if (!g_pending || GetTickCount64() < g_dueAt)
        return;
    g_pending = false;
    const bool flagAtEnd = GameIsDrivingTheCamera();
    if (!Grab(g_s2)) {
        Log_Printf("FreezeFind: could not read them a second time");
        return;
    }

    int moved = 0;
    for (int i = 0; i < kAll; ++i) {
        g_moves[i] = g_s1[i] != g_s2[i];
        if (g_moves[i])
            ++moved;
    }

    if (!g_haveOff) {
        std::memcpy(g_movesOff, g_moves, sizeof(g_movesOff));
        std::memcpy(g_offS1, g_s1, sizeof(g_offS1));
        std::memcpy(g_offS2, g_s2, sizeof(g_offS2));
        g_haveOff = true;
        Log_Printf("FreezeFind: freeze OFF - %d byte(s) moved in 300 ms (action camera %s at the start, %s at "
                   "the end). Now turn their freeze ON and press F4 during the next melee.",
            moved, g_flagAtStart ? "YES" : "no", flagAtEnd ? "YES" : "no");
        return;
    }

    int held = 0;
    static bool thisOne[kAll];
    for (int i = 0; i < kAll; ++i) {
        thisOne[i] = g_movesOff[i] && !g_moves[i];
        if (thisOne[i])
            ++held;
    }
    if (!g_haveAnswer) {
        std::memcpy(g_answer, thisOne, sizeof(g_answer));
        g_haveAnswer = true;
    } else {
        for (int i = 0; i < kAll; ++i)
            g_answer[i] = g_answer[i] && thisOne[i];
    }
    ++g_pairs;
    g_haveOff = false;

    int left = 0;
    for (int i = 0; i < kAll; ++i)
        if (g_answer[i])
            ++left;
    Log_Printf("FreezeFind: freeze ON - %d byte(s) moved (action camera %s at the start, %s at the end). %d "
               "moved with it off and held still with it on; after %d pair(s), %d survive.",
        moved, g_flagAtStart ? "YES" : "no", flagAtEnd ? "YES" : "no", held, g_pairs, left);

    if (!left) {
        Log_Printf("FreezeFind: nothing survived - either the capture missed the animation, or what they hold "
                   "is not in the camera object or the player controller. Press F4 again during a melee with "
                   "their freeze OFF to start another pair.");
        g_haveAnswer = false;
        return;
    }
    if (left > 96) {
        Log_Printf("FreezeFind: too many to list - do another pair and they will narrow.");
        return;
    }
    // Runs of consecutive bytes, which is how a float or a vector shows up.
    int said = 0;
    for (int i = 0; i < kAll && said < 24; ++i) {
        if (!g_answer[i])
            continue;
        int j = i;
        while (j < kAll && j - i < 16 && g_answer[j])
            ++j;
        char where[32] = {};
        Where(i, where, sizeof(where));
        char was[64] = {}, became[64] = {};
        int w = 0, b = 0;
        for (int k = i; k < j; ++k) {
            w += sprintf_s(was + w, sizeof(was) - w, "%02X ", g_offS1[k]);
            b += sprintf_s(became + b, sizeof(became) - b, "%02X ", g_offS2[k]);
        }
        Log_Printf("FreezeFind:   %s ran %s-> %s with their freeze off, and did not move with it on", where,
            was, became);
        ++said;
        i = j;
    }
}

} // namespace freezefind

// ---- The action camera gate (2026-09-28) --------------------------------
//
// From Anthony, who went at it with a debugger while we were narrowing by
// elimination from inside the process. His addresses are absolute and ours
// are offsets from the module base, so every one of his is 0x400000 lower
// here.
//
//   00B563A4  80 BE 78 10 00 00 00   cmp byte ptr [esi+1078],00   145 reads
//   00B563AB                         the conditional jump after it
//   00B56383  C6 86 78 10 00 00 01   mov byte ptr [esi+1078],01   melee sets it
//   00B56362  C6 86 78 10 00 00 00   mov byte ptr [esi+1078],00   clears it
//   00B54906  C6 87 78 10 00 00 00   mov byte ptr [edi+1078],00   clears it
//
// So +0x1078 is the real action camera flag. controller+0x634, which we
// found by elimination and leaned on all day, was only ever correlated with
// it - which is why refusing on that signal caught the right frames and
// still fought. We were suppressing the effect instead of stopping the
// cause, and the game went on trying to pan the whole time.
//
// The flag is read by a compare with a conditional jump straight after, so
// the whole thing is one byte: make the jump unconditional and the action
// camera branch is never taken. Same shape as the laser sight patch turning
// je into jmp.
//
// His warning is a feature here: touching that compare also covers jumping
// down and opening doors, which is exactly the set of things that has been
// throwing the view around.
//
// Two more in the same family:
//
//   exe+7743C8   the line that makes the camera follow your partner when you
//                hold locate. Change its 01 to 00 and the locate icons still
//                show while the camera stays where you left it.
//   exe+75AF53   the third person sniper check. Forcing that jump keeps
//                every scoped weapon in third person, which is the
//                difference between a scope being unusable in a headset and
//                simply working.
//
// Every site is verified before anything is written, works out its own
// replacement from the instruction that is really there, and is reversible
// byte for byte.
namespace actioncam {

enum Kind {
    kForceJump,  // a conditional jump becomes unconditional
    kZeroTheOne, // mov byte ptr [..],01 becomes ..,00
};

struct Site {
    const char* what;
    uintptr_t rva;
    Kind kind;
    unsigned char was[8];
    unsigned char now[8];
    int len;
    int at; // offset from rva of the byte(s) we change
    bool ready;
    bool on;
};

Site g_sites[] = {
    { "the action camera gate at exe+7563AB", 0x7563AB, kForceJump, {}, {}, 0, 0, false, false },
    { "the camera following your partner at exe+7743C8", 0x7743C8, kZeroTheOne, {}, {}, 0, 0, false, false },
    { "scoped weapons in third person at exe+75AF53", 0x75AF53, kForceJump, {}, {}, 0, 0, false, false },
};
constexpr int kCount = sizeof(g_sites) / sizeof(g_sites[0]);

uintptr_t g_base = 0;
bool g_looked = false;

bool Poke(unsigned char* at, const unsigned char* what, int len)
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

// C6 /0 ib - mov byte ptr [something], imm8. Where the imm8 sits depends on
// the addressing mode, so decode it rather than assuming a shape: this same
// instruction appears in the log as seven bytes with a disp32 and as four
// with a disp8.
int WhereIsTheImmediate(const unsigned char* b)
{
    if (b[0] != 0xC6)
        return -1;
    const int mod = b[1] >> 6;
    const int rm = b[1] & 7;
    int len = 2;
    if (mod != 3 && rm == 4)
        ++len; // SIB
    if (mod == 0) {
        if (rm == 5)
            len += 4;
    } else if (mod == 1) {
        len += 1;
    } else if (mod == 2) {
        len += 4;
    }
    return len;
}

bool WorkOutTheChange(Site& s)
{
    unsigned char* at = reinterpret_cast<unsigned char*>(g_base + s.rva);
    unsigned char seen[8] = {};
    if (!TryRead(seen, at, sizeof(seen)))
        return false;
    std::memcpy(s.was, seen, sizeof(s.was));
    std::memcpy(s.now, seen, sizeof(s.now));

    if (s.kind == kZeroTheOne) {
        const int imm = WhereIsTheImmediate(seen);
        if (imm < 0 || imm > 6) {
            Log_Printf("ActionCam: %s holds %02X %02X %02X %02X %02X %02X %02X, which is not a byte store - "
                       "not touching it",
                s.what, seen[0], seen[1], seen[2], seen[3], seen[4], seen[5], seen[6]);
            return false;
        }
        if (seen[imm] == 0x00) {
            Log_Printf("ActionCam: %s already writes zero - somebody got here first", s.what);
            s.ready = true;
            s.on = true;
            s.len = 0;
            return true;
        }
        if (seen[imm] != 0x01) {
            Log_Printf("ActionCam: %s writes %02X, not 01 - not touching it", s.what, seen[imm]);
            return false;
        }
        s.at = imm;
        s.len = 1;
        s.now[imm] = 0x00;
        s.ready = true;
        return true;
    }

    if (seen[0] == 0xEB || seen[0] == 0xE9) {
        Log_Printf("ActionCam: %s is already an unconditional jump - somebody got here first", s.what);
        s.ready = true;
        s.on = true;
        s.len = 0;
        return true;
    }
    if (seen[0] == 0x74 || seen[0] == 0x75) {
        s.len = 1;
        s.now[0] = 0xEB;
        s.ready = true;
        return true;
    }
    if (seen[0] == 0x0F && (seen[1] == 0x84 || seen[1] == 0x85)) {
        int rel = 0;
        std::memcpy(&rel, seen + 2, sizeof(rel));
        ++rel; // five bytes of jmp where there were six of jcc
        s.len = 6;
        s.now[0] = 0xE9;
        std::memcpy(s.now + 1, &rel, sizeof(rel));
        s.now[5] = 0x90;
        s.ready = true;
        return true;
    }
    Log_Printf("ActionCam: %s holds %02X %02X %02X %02X %02X %02X, which is not a conditional jump - not "
               "touching it",
        s.what, seen[0], seen[1], seen[2], seen[3], seen[4], seen[5]);
    return false;
}

void Set(int which, bool on)
{
    if (which < 0 || which >= kCount)
        return;
    Site& s = g_sites[which];
    if (!s.ready || s.len == 0 || on == s.on)
        return;
    unsigned char* at = reinterpret_cast<unsigned char*>(g_base + s.rva + s.at);
    if (!Poke(at, (on ? s.now : s.was) + s.at, s.len))
        return;
    s.on = on;
    Log_Printf("ActionCam: %s is now %s", s.what, on ? "OFF - the game never takes that branch" : "back as it was");
}

bool Ready(int which)
{
    return which >= 0 && which < kCount && g_sites[which].ready;
}

void Install()
{
    if (g_looked)
        return;
    g_base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    if (!g_base)
        return;

    // The compare this gate belongs to, so a wrong address or a different
    // build announces itself instead of being silently patched.
    static const unsigned char kCmp[] = { 0x80, 0xBE, 0x78, 0x10, 0x00, 0x00, 0x00 };
    unsigned char sawCmp[sizeof(kCmp)] = {};
    const bool haveCmp = TryRead(sawCmp, reinterpret_cast<unsigned char*>(g_base + 0x7563A4), sizeof(sawCmp))
        && std::memcmp(sawCmp, kCmp, sizeof(kCmp)) == 0;
    if (!haveCmp) {
        static bool s_said = false;
        if (!s_said) {
            s_said = true;
            Log_Printf("ActionCam: exe+7563A4 holds %02X %02X %02X %02X %02X %02X %02X, not the flag compare "
                       "- waiting for the packer",
                sawCmp[0], sawCmp[1], sawCmp[2], sawCmp[3], sawCmp[4], sawCmp[5], sawCmp[6]);
        }
        return;
    }
    g_looked = true;
    Log_Printf("ActionCam: exe+7563A4 is cmp byte [esi+1078],0 - the action camera flag, confirmed");

    for (int i = 0; i < kCount; ++i)
        WorkOutTheChange(g_sites[i]);

    // The rest of what Anthony listed, printed rather than patched: what
    // sets the flag, what clears it, and the wider jump. Writing the bytes
    // down here means the next idea needs no round trip.
    static const struct {
        uintptr_t rva;
        const char* what;
    } kLook[] = {
        { 0x756383, "sets the flag to 1 (melee)" },
        { 0x756362, "clears the flag" },
        { 0x754906, "clears the flag" },
        { 0x75631D, "the wider conditional jump" },
    };
    for (const auto& l : kLook) {
        unsigned char b[10] = {};
        if (!TryRead(b, reinterpret_cast<unsigned char*>(g_base + l.rva), sizeof(b)))
            continue;
        Log_Printf("ActionCam: exe+%X  %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X  (%s)",
            static_cast<unsigned>(l.rva), b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], l.what);
    }
}

} // namespace actioncam

// ---- Keeping the rig collapsed all frame (2026-09-27) --------------------
//
// From the trainer: "there are no sliders for melee camera - freezing it just
// uses the normal camera sliders". So the melee camera is not a separate rig
// at all. It works by writing over the NORMAL rig during a melee, and the
// trainer's freeze simply stops that write sticking.
//
// Which explains the whole night. We already override that rig - it is why
// first person works - but only from the rigs-ready hook, once, early in the
// frame. The melee write happens afterwards and therefore wins, and every
// anchor, threshold and limiter since has been trying to undo its effect on
// the view instead of stopping it at the rig.
//
// The trainer's freeze is just "keep putting the values back". So do that:
// re-apply the override at the end of every frame as well, to whichever rig
// bases are currently live. A rig the game has just rewritten is collapsed
// again before it can place a camera.
//
// This costs nothing in ordinary play, because the values are already what we
// want and writing them again changes nothing.

void CameraRigHook_OnPresent()
{
#if RE5VR_DIAGNOSTICS
    freezefind::Tick();
#endif
    KeepTheRigCollapsed();
    TraceTheCamera();
    PinCameraToBody();
}

void CameraRigHook_OnEndScene()
{
    g_renderThreadId.store(GetCurrentThreadId(), std::memory_order_relaxed);

    // Before anything reads a hook: some of them may not be in yet.
    CameraRigHook_RetryInstall();
    // NOT DIAGNOSTICS (2026-09-28, user: "make sure everything that is
    // required for everything to work does not require a diagnostic build").
    //
    // ArmIk_OnEndScene runs SampleGunAnim, which is how a weapon with its
    // magazine bone set to -1 gets worked out by watching one reload and
    // written into re5vr_magbones.ini. That is a shipped feature the help
    // text tells people about, and in a release build it never ran once.
    // Its own diagnostics are guarded inside it.
    ArmIk_OnEndScene();
#if RE5VR_DIAGNOSTICS
    LogViewCallers();
#endif
    // Before anything else: the game's camera code may not have run this
    // frame, and if it has not run for a while the head is stuck collapsed.
    HeadWatchdog_Tick();

    // The camera hook reads the head scale back immediately after writing it,
    // which is the one moment it is guaranteed to look correct. The user sees
    // the head deflate "like a balloon" over several frames regardless, so
    // sample the same joint at the OTHER end of the frame - here, after the
    // game has run its own animation and skeleton work. If these two readings
    // disagree, the game is easing our value and the write needs to move.
#if RE5VR_DIAGNOSTICS
    if (g_actionTraceUntilMs && GetTickCount64() <= g_actionTraceUntilMs && g_lastPlayerHead &&
        HeadJointStillValid(g_lastPlayerJoints, g_lastPlayerHead)) {
        float localScale[3] = { -1.0f, -1.0f, -1.0f }, row0[3] = { 0.0f, 0.0f, 0.0f };
        TryRead(localScale, g_lastPlayerHead + kOffJointScale, sizeof(localScale));
        TryRead(row0, g_lastPlayerHead + kOffJointWorldMatrix, sizeof(row0));
        Log_Printf("CameraRigHook: endscene trace - head joint scale %.3f, world %.3f",
            localScale[0], Length3(row0));
    }
#endif // RE5VR_DIAGNOSTICS

    // F is the user's action button, so it is ground truth for "an action
    // starts now" - but it is also the action itself, which makes it useless
    // for tracing anything that is NOT an action (sprinting, say: pressing F
    // there kicks or stomps instead). Scroll Lock does the same job with no
    // side effect in game, so either key opens the window.
#if RE5VR_DIAGNOSTICS
    static bool prevActionDown = false;
    const bool fDown = (GetAsyncKeyState('F') & 0x8000) != 0;
    // PAGE DOWN AS WELL (2026-09-27, user: "i dont have scroll lock on my
    // keyboard"). Compact and laptop layouts drop it, the same reason
    // pixel_constant_probe moved off Home. Page Down is unclaimed and present
    // on everything; Scroll Lock stays for anyone who has one.
    // HOME, NOT PAGE DOWN (2026-09-27, user: "page down does something to the
    // game"). Page Down is bound in game, so the recording was moving the
    // player while recording them. Home is unclaimed here and sits beside End,
    // which this keyboard is already known to have. Scroll Lock stays for
    // anyone who does have one.
    const bool scrollDown = (GetAsyncKeyState(VK_SCROLL) & 0x8000) != 0
        || (GetAsyncKeyState(VK_HOME) & 0x8000) != 0;
    const bool actionDown = fDown || scrollDown;
    if (actionDown && !prevActionDown && g_enabled) {
        g_actionTraceUntilMs = GetTickCount64() + kActionTraceMs;
        Log_Printf("CameraRigHook: %s pressed - tracing the camera for %llu ms",
            fDown ? "F (action)" : "Scroll Lock or Home (no action)", kActionTraceMs);
    }
    prevActionDown = actionDown;

    // F4: the code differ, for comparing a trainer against itself.
    //
    // NOT gated on first person (2026-09-28, user: "remember they will
    // disable our first person camera hook, so we will want to dance around
    // that"). Running a trainer alongside us is the one situation where our
    // own hooks may be missing, and that is exactly when this has to work.
    //
    // Press once to record, toggle the trainer option, press again. Two
    // comparisons run off the same press: the executable section, and the
    // camera object plus the player controller. Their melee freeze turned out
    // not to be a code patch at all, so the second one is the one that
    // matters now - but the first costs nothing and keeps us honest.
    {
        static bool s_wasF4 = false;
        const bool f4Down = (GetAsyncKeyState(VK_F4) & 0x8000) != 0;
        if (f4Down && !s_wasF4) {
            // The code differ stays: pressed DURING a melee it now also
            // catches a patch that only exists while the animation runs,
            // which is the one thing the earlier runs could not have seen.
            codediff::Look();
            freezefind::Start();
        }
        s_wasF4 = f4Down;
    }

    // End: the north test, on its own key so it never runs during a recording.
    {
        static bool s_wasEnd = false;
        const bool endDown = (GetAsyncKeyState(VK_END) & 0x8000) != 0;
        if (endDown && !s_wasEnd && g_enabled) {
            // STOP GUESSING AT THE WRITER (2026-09-28, user: "still not
            // catching the melee camera. What does the trainer do that we
            // are not").
            //
            // Four theories have now been killed by measurement rather than
            // by argument, which is progress of a kind:
            //
            //   the rig array      - all six entries are collapsed, before
            //                        every view, and the melee still moves
            //   SetLookAt          - every call to it is now REFUSED outright
            //                        and the melee still moves
            //   our view matrix    - provably steady through a whole stomp
            //   the camera delta   - hooked, and zeroing it breaks shadows
            //
            // So something writes the camera position during a melee that is
            // none of those, and there is exactly one way to find out what:
            // ask the CPU. A hardware write watchpoint on the live camera eye
            // names the instruction, gives its registers and prints the code
            // either side of it. That is how the ammunition mirror, the arm
            // writer and the aim pitch were all found, and it has never once
            // needed a guess.
            //
            // Press End and then immediately do the melee. Four seconds.
            unsigned char* cam = static_cast<unsigned char*>(g_theCameraRaw);
            if (!cam) {
                Log_Printf("CamWriter: no camera yet - load a level first");
            } else {
                // UPSTREAM, NOT THE COPY (2026-09-28, from the walking
                // window).
                //
                // Watching +0x50 while simply walking about caught one
                // writer, exe+44289F, and the bytes around it finish the
                // picture that exe+44287F started:
                //
                //   D9 86 40 06 00 00  fld [esi+640] -> [esi+30]   the eye
                //   D9 86 50 06 00 00  fld [esi+650] -> [esi+40]   the up
                //   D9 86 60 06 00 00  fld [esi+660] -> [esi+50]   the target
                //
                // One routine, once a frame, copying a block at +0x640 down
                // onto the look-at at +0x30. So +0x30 and +0x50 are never
                // where the camera is decided - they are the last copy of a
                // decision taken upstream, which is why refusing writes to
                // them changed nothing at all. Twice.
                //
                // +0x640 and +0x660 are the real thing. Whoever writes THOSE
                // during a stomp is the melee camera.
                // ALL FOUR IN ONE WINDOW (2026-09-28, user: "so each time I
                // press end, one of them will fix it?").
                //
                // No - End measures, it does not fix, and asking for four
                // melees to answer one question was a bad trade. x86 has four
                // debug registers and we were using one of them. These are
                // the whole chain in a single press: the accumulator, the two
                // sources the frame copy reads, and the copy itself. Whoever
                // writes the sources during a stomp is the melee camera, and
                // there is nothing downstream of them left to hide behind.
                void* addresses[4] = {
                    cam + 0x3B0,
                    cam + 0x640,
                    cam + 0x660,
                    cam + 0x30,
                };
                const char* names[4] = {
                    "the position accumulator (+0x3B0)",
                    "the eye SOURCE (+0x640, copied onto +0x30 every frame)",
                    "the target SOURCE (+0x660, copied onto +0x50 every frame)",
                    "the eye itself (+0x30), the copy, for comparison",
                };
                AimFinder_Rearm();
                AimFinder_StartMany(addresses, names, 4);
                Log_Printf("CamWriter: watching the whole camera chain for 4 s - DO THE MELEE NOW");
            }
        }
        s_wasEnd = endDown;
    }
#endif // RE5VR_DIAGNOSTICS

    // Hand the camera hook (game thread) a plain "VR is on" flag, so it never
    // calls into the VR bridge itself.
    static unsigned long long s_lastVrCheckMs = 0;
    const unsigned long long nowMs = GetTickCount64();
    // Every frame while head-follow is on: the game thread needs a fresh head
    // direction, not one up to 100 ms old.
    if (g_headFollow.load(std::memory_order_relaxed) || nowMs - s_lastVrCheckMs >= 100) {
        s_lastVrCheckMs = nowMs;
        XRBridgeEyeView left, right;
        const bool haveViews = VRBridge_GetEyeViews(left, right);
        g_vrActive.store(haveViews, std::memory_order_relaxed);
        if (haveViews)
            PublishHeadForward(left.rotationDelta);
    }

    // Hotkeys are gone (2026-09-13): F4, F9, F11, F6, '`', ',' '.' and the F5
    // / both-sticks reset view all live in the in-game menu now (ui/menu.cpp),
    // which also owns the controller chord - a tap of both sticks opens it, a
    // one-second hold still resets the view.

    // Nothing here dereferences a camera struct any more (2026-09-11). The
    // END-OF-FRAME and WATCH diagnostics that used to read live-list bases at
    // this point are gone: a base hit within the last 250ms is NOT proof it
    // still exists. The WATCH read crashed the game at 04:18 on 2026-09-11
    // (d3d9.dll+3B63, `movss xmm0,[edx+30h]` - the +0x30 field) when Chris's
    // intro outfit change destroyed the camera controllers - the same class
    // of bug as the 20:01 crash on 2026-09-10. Both diagnostics had done
    // their job (the pop-out is solved), so they were removed, not patched.
    // Game camera memory is now read only inside the hook callbacks, where
    // the game itself is using it, or through SEH-guarded copies
    // (boom_finder.cpp).
    //
    // No end-of-frame freeze either: the rigs are re-copied from their
    // profile sources at the start of every camera update, so an EndScene
    // write can never reach the blend. The rigs-ready hook is the only
    // write that counts.
    ExpireLiveBases();
}

bool CameraRigHook_IsEnabled()
{
    return g_enabled;
}

void* CameraRigHook_GetPlayerController()
{
    return const_cast<unsigned char*>(g_playerController);
}

bool CameraRigHook_IsVrActive()
{
    return g_vrActive.load(std::memory_order_relaxed);
}

bool CameraRigHook_TakeAimMouse(long* dx, long* dy)
{
    const unsigned long long when = g_aimMouseMs.load(std::memory_order_relaxed);
    if (!when || GetTickCount64() - when > 200)
        return false;
    const long pendingY = g_aimMouseDy.exchange(0, std::memory_order_relaxed);
    const long pendingX = g_aimMouseDx.exchange(0, std::memory_order_relaxed);
    if (!pendingX && !pendingY)
        return false;
    if (dx)
        *dx = pendingX;
    if (dy)
        *dy = pendingY;
    return true;
}

bool CameraRigHook_GetBodyEye(const float forward[3], const float camPos[3], float outEye[3])
{
    return ComputeBodyEye(forward, camPos, outEye);
}

bool CameraRigHook_LastCameraPosition(float out[3])
{
    return LastCameraPosition(out);
}

// ---- Keeping the game's own camera in your body (2026-09-26) --------------
//
// The watchpoint settled the mechanism. The camera's position is never
// assigned, it is ACCUMULATED:
//
//     movaps xmm0, xmm4
//     addss  xmm0, [esi+3B0]
//     movss  [esi+3B0], xmm0        and the same for 3B4 and 3B8
//
// and +640, +30 and +1F0 are copies of the result. So an action camera does
// not run different code - it feeds a different delta into the same add, which
// is exactly why standing still and vaulting produced an identical list of
// writers. It also means there is no special case to find: correcting the
// result covers every one of them.
//
// This is the same shape as ApplyOverride, which has kept ordinary play in
// first person for weeks: let the game compute whatever it likes, then put the
// answer back where it belongs before anything reads it.
//
// WHY THE FIRST ATTEMPT SKEWED THE EYES. It wrote LastCameraPosition plus the
// eye offset - and LastCameraPosition is decoded from the view matrix we
// ourselves replace, so during a hold it already WAS the eye. It fed its own
// output back in and walked away a little further every frame. The address was
// never wrong; the value was. The head joint's world position is absolute,
// comes from the skeleton, and cannot feed back into anything.

// The same four the hunt found, written together so consumers this frame see
// it too rather than waiting for the copies to catch up next frame.
constexpr DWORD kOffCameraPlace[4] = { 0x30, 0x1F0, 0x3B0, 0x640 };

// And where it is looking. Not found by hunting: BuildHeadLockedView has been
// reading it from here all along, one field past the eye.
//
//     std::memcpy(eye, fake + 0x30, sizeof(eye));
//     std::memcpy(target, fake + 0x50, sizeof(target));
constexpr DWORD kOffCameraTarget = 0x50;

// WHERE THE ORIENTATION LIVES (2026-09-27, user: "any time a gap is jumped,
// the camera whips all over the place ... like someone grabs your head and
// just forces and moves your head around like crazy").
//
// We have never touched the rotation. The position is pinned and its delta is
// dead, so what is left swinging on a vault is the orientation, and it is
// entirely the game's.
//
// Finding it does not need a watchpoint, because a rotation announces itself.
// Three consecutive vectors, each of unit length, each at right angles to the
// other two - that is a basis, and random floats are never accidentally
// orthonormal. So the object can simply be read and asked.
//
// Once only, and read-only. What it finds is a shortlist to aim the next
// watchpoint at, not something to write to yet: an ill-formed basis breaks
// far more than a misplaced point, and we have already learned tonight what
// writing into the middle of a pipeline does.
void FindTheRotation(unsigned char* camera)
{
    static bool s_looked = false;
    if (s_looked || !camera)
        return;
    s_looked = true;

    constexpr int kScan = 0x800;
    char found[256];
    int at = 0;
    int hits = 0;
    __try {
        for (int off = 0; off + 36 <= kScan; off += 4) {
            const float* m = reinterpret_cast<const float*>(camera + off);
            bool ok = true;
            for (int r = 0; r < 3 && ok; ++r) {
                const float len = m[r * 3] * m[r * 3] + m[r * 3 + 1] * m[r * 3 + 1] + m[r * 3 + 2] * m[r * 3 + 2];
                ok = len > 0.99f && len < 1.01f;
            }
            if (!ok)
                continue;
            // And mutually perpendicular, which is what rules out three
            // unrelated directions that happen to be normalised.
            for (int a = 0; a < 3 && ok; ++a) {
                const int b = (a + 1) % 3;
                const float dot = m[a * 3] * m[b * 3] + m[a * 3 + 1] * m[b * 3 + 1] + m[a * 3 + 2] * m[b * 3 + 2];
                ok = dot > -0.01f && dot < 0.01f;
            }
            if (!ok)
                continue;
            ++hits;
            if (at < static_cast<int>(sizeof(found)) - 12)
                at += sprintf_s(found + at, sizeof(found) - at, "%s+%X", at ? ", " : "", off);
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (hits)
        Log_Printf("CameraRot: %d orthonormal basis or bases in the camera object, at %s", hits, found);
    else
        Log_Printf("CameraRot: no orthonormal basis anywhere in the first %d bytes - the orientation is kept as "
                   "something else, a quaternion most likely",
            kScan);
}

void PinCameraToBody()
{
    // AND SAY WHY IF NOT (2026-09-26: not one line in a whole session).
    //
    // Five conditions have to hold and the routine reported none of them, so
    // guessing which one fails costs a build per guess. Each now names itself
    // once every five seconds, which costs one build total.
    const char* no = nullptr;
    const bool wanted = XrInput_GetSettings().cameraFollowsEye;
    unsigned char* camera = static_cast<unsigned char*>(g_theCamera.load(std::memory_order_relaxed));
    if (!wanted)
        no = "the setting is off";
    else if (!g_enabled)
        no = "first person is off";
    else if (!g_vrActive.load(std::memory_order_relaxed))
        no = "VR is not active as far as the rig knows";
    else if (!camera)
        no = "the main camera has not been seen by the view hook";
    else if (!g_lastPlayerHead)
        no = "there is no head joint";
    // AND THE DRIFT, WHICH IS WHAT WAS FIGHTING IT (2026-09-28, user: "I can
    // tell it TRIED to work that time, but it is getting fought").
    //
    // The standing-still window settled it. exe+440084 places the camera at
    // +0x3B0 on every single frame of ordinary play - 326 of 326 - and
    // during a stomp it does not run at all. So once the melee blend is
    // refused, the only thing left touching +0x3B0 is exe+442727, the
    // accumulator, adding a delta every frame with nothing to anchor it.
    //
    // Removing what shoves you and leaving what drifts you is half a fix.
    // We already hook that instruction and can zero the delta for our camera
    // alone; it has simply never been switched on for this case. So kill it
    // on exactly the frames the game has taken the camera away, and on no
    // others - in ordinary play exe+440084 is doing the placing and the
    // accumulator is part of how the camera is meant to move.
    // The blend itself, not the flag. The flag stays as a second opinion
    // for anything that moves the camera without going through the blend.
    // Exactly the frames the blend was refused on, and no others. Holding
    // the accumulator anywhere else suppresses camera movement the game is
    // entitled to make.
    const bool refusingMelee = meleecam::RefusedRecently();
    g_killDelta = ((wanted || refusingMelee) && g_enabled && g_camDeltaHooked) ? 1u : 0u;

    {
        static unsigned long long s_saidDrift = 0;
        static unsigned s_killed = 0, s_frames = 0, s_actionFrames = 0;
        ++s_frames;
        if (g_killDelta)
            ++s_killed;
        // Side by side with the blend count, so one session says whether the
        // blend has anything to do with action cameras at all.
        if (GameIsDrivingTheCamera())
            ++s_actionFrames;
        const unsigned long long nowDrift = GetTickCount64();
        if (nowDrift - s_saidDrift > 1000) {
            s_saidDrift = nowDrift;
            if (meleecam::g_blendRefused || s_killed)
                Log_Printf("MeleeCam: %u frame(s) - blend on YOUR camera %lu, of which %lu refused; action "
                           "camera flag %u; drift held %u",
                    s_frames, meleecam::g_blendHits, meleecam::g_blendRefused, s_actionFrames, s_killed);
            meleecam::g_blendHits = 0;
            meleecam::g_blendRefused = 0;
            s_actionFrames = 0;
            s_killed = 0;
            s_frames = 0;
        }
    }
    if (camera)
        FindTheRotation(camera);
    if (no) {
        static unsigned long long s_whyAt = 0;
        const unsigned long long nowWhy = GetTickCount64();
        if (nowWhy - s_whyAt > 5000) {
            s_whyAt = nowWhy;
            Log_Printf("CameraPin: not holding the camera - %s", no);
        }
        return;
    }

    // THE EYE, NOT THE JOINT (2026-09-26, user: "the camera sat lower than
    // usual").
    //
    // I wrote the bare head joint on the reasoning that this position only
    // feeds sorting, fading and detail levels, where a few centimetres cannot
    // matter. That was wrong and the low camera proves it: the game builds its
    // view from here as well, so it wants the place the eye actually is, which
    // is the joint plus the neck mount, the eye height and the running lead.
    //
    // ComputeBodyEye is exactly that sum and the head lock already uses it, so
    // the two placements now agree by construction instead of by coincidence.
    // Null for the camera position deliberately: that argument is only a reach
    // check for "a shot framed from across the room", and feeding it a value
    // decoded from the view matrix we ourselves replace is the circularity
    // that skewed the eyes last time.
    // NOTHING THAT DEPENDS ON OUR OWN VIEW (2026-09-26, user: "it seems to
    // jitter around the body, like trying to snap back to the original
    // position").
    //
    // ComputeBodyEye was the wrong thing to reach for. It places the eye a
    // little AHEAD of the pivot along a forward vector, and the only forward
    // available here is decoded from the view matrix we replace. So turning
    // your head moved the pinned point, which moved the camera, which moved
    // the forward. A loop with a lag in it, which is what the jitter is - and
    // the same circularity that skewed the eyes, arriving through a different
    // argument. Twice now, so it is worth naming the rule: nothing written
    // into the camera may be derived from anything the camera produces.
    //
    // The pin does not need the forward. The complaint was height, so take the
    // height and nothing else: the pivot the head lock uses, raised the way it
    // raises it, plus the running lead, which comes from the character's own
    // travel rather than from any view.
    float head[3];
    if (!TryRead(head, g_lastPlayerHead + kOffJointWorldPos, sizeof(head)))
        return;
    float eyeAbove = kEyeAbovePivot;
    if (g_vrEyeOnNeck.load(std::memory_order_relaxed) && g_lastPlayerJoints) {
        const ptrdiff_t off = g_lastPlayerHead - g_lastPlayerJoints;
        unsigned char links[4] = {};
        if (off >= 0 && off % kJointStride == 0
            && TryRead(links, g_lastPlayerHead + kOffJointLinks, sizeof(links))) {
            const int headIndex = static_cast<int>(off / kJointStride);
            const int neckIndex = links[1];
            float neck[3];
            if (neckIndex != headIndex && neckIndex < 200
                && TryRead(neck, g_lastPlayerJoints + neckIndex * kJointStride + kOffJointWorldPos, sizeof(neck))) {
                const float gap[3] = { head[0] - neck[0], head[1] - neck[1], head[2] - neck[2] };
                if (Length3(gap) < 60.0f) {
                    std::memcpy(head, neck, sizeof(neck));
                    eyeAbove = g_vrEyeAboveNeck.load(std::memory_order_relaxed);
                }
            }
        }
    }
    head[0] += g_leadX.load(std::memory_order_relaxed);
    head[1] += eyeAbove;
    head[2] += g_leadZ.load(std::memory_order_relaxed);
    for (int k = 0; k < 3; ++k)
        if (!(head[k] > -1e7f && head[k] < 1e7f))
            return;

    // NEAR THE MAN, OR NOT AT ALL (2026-09-27, user: "climbing a ladder sent
    // my camera to space").
    //
    // Ten million was never a guard. The only thing that makes this value
    // sensible is that it is supposed to be somebody's head, and a head is
    // always within a couple of metres of the character it belongs to. A
    // ladder or a level seam can leave the joint pointer looking at something
    // that is no longer a skeleton, and the arithmetic downstream is perfectly
    // happy to fly the camera into orbit with it.
    //
    // So it is measured against the character's own world position, which
    // comes from a different structure entirely and cannot go wrong in the
    // same way. Four hundred units is about four and a half metres: far wider
    // than any head, far narrower than space. Out of range and nothing is
    // written at all, which leaves the game's camera exactly as it was.
    {
        float man[3];
        if (CameraRigHook_GetPlayerWorldPos(man)) {
            const float dx = head[0] - man[0], dy = head[1] - man[1], dz = head[2] - man[2];
            const float off = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (!(off < 400.0f)) {
                static unsigned long long s_toldAt = 0;
                const unsigned long long nowBad = GetTickCount64();
                if (nowBad - s_toldAt > 2000) {
                    s_toldAt = nowBad;
                    Log_Printf("CameraPin: refusing to move the camera - the head reads %.0f units from the man, "
                               "which is not a head",
                        off);
                }
                return;
            }
        }
    }
    // The eye sits a little above the joint, the same way the head lock places
    // it, so the game is told the same story we are drawing.
    // The joint itself, with nothing added. Where the eye is DRAWN is a
    // separate question that already has an answer; this is only about telling
    // the game where to sort, fade and pick detail from, and the head joint is
    // close enough for all three.

    // AND NO SMOOTHING (2026-09-27, user: "its the body snapping back trying to
    // get underneath the camera").
    //
    // That is the difference between two placements, not a stutter in one.
    // BuildHeadLockedView computes the eye it renders from; the pin was
    // deriving its own point and then lagging it by fifty milliseconds, so the
    // two disagreed whenever anything moved at all, idle breathing included,
    // and the body slid under the view by exactly that difference. A filter
    // could only ever make it worse.

    // AND THE TARGET, WHICH IS THE WHOLE PROBLEM (2026-09-27, user: "any time
    // a gap is jumped, the camera whips all over the place", and "the orbit bug
    // still happens").
    //
    // BuildHeadLockedView reads this camera's eye from +30 and its LOOK-AT
    // TARGET from +50, and then:
    //
    //     if (distance < 1.0f)
    //         return false; // already there, nothing to do
    //
    // Our pin puts the eye exactly on the pivot, so that distance is zero and
    // the head lock stands down every frame. From then on the view is the
    // game's: positioned at your head, aimed at whatever target the game left
    // behind. Hence the orbiting, the whipping, and the body sliding out from
    // under you. The pin was disabling the very thing keeping your head
    // straight.
    //
    // So aim it as well. A target one metre ahead along the direction the
    // HEADSET is facing - which comes from the runtime and not from anything
    // the camera produces, so it cannot feed back the way camPos and forward
    // both did.
    // AIMED BY THE BODY (2026-09-27, user: "so what needs implemented for it
    // to stay in place but not go crazy").
    //
    // One thing: the look-at needs BOTH ends. Position from the neck, target
    // from the body's facing.
    //
    // The head lock stands down as soon as the camera is within a unit of your
    // head, so pinning the position switches it off and hands the orientation
    // back to the game. That is fine as long as the game is aiming somewhere
    // sensible, and during a vault it is not - hence the whip.
    //
    // The first attempt aimed along the HEADSET and was wrong, because our own
    // stereo code already rotates the view by the headset delta, so your head
    // turn arrived twice. The division has to be: the game supplies where the
    // BODY faces, we supply where your HEAD faces, and neither supplies the
    // other's. Then turning your body turns the view through the game, turning
    // your head turns it through us, and a vault changes neither.
    //
    // The facing comes from the character's own transform at +0x60, rebuilt
    // from the body quaternion every frame. BodyFacingDegrees already reads
    // it. Nothing about it comes from the camera, so it cannot feed back - the
    // mistake that has now cost three builds.
    float aim[3];
    bool haveAim = false;
    {
        unsigned char* controller = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
        unsigned char* man = nullptr;
        float row0[4] = {};
        if (controller && TryRead(&man, controller + kOffControllerCharacter, sizeof(man)) && man
            && TryRead(row0, man + kOffBodyTransform[0], sizeof(row0))) {
            const float flat = std::sqrt(row0[0] * row0[0] + row0[2] * row0[2]);
            if (flat > 1e-3f) {
                // Level, because the pitch is yours and arrives through the
                // headset. A metre ahead is plenty; only the direction of this
                // vector matters, not its length.
                aim[0] = head[0] + (row0[0] / flat) * 86.0f;
                aim[1] = head[1];
                aim[2] = head[2] + (row0[2] / flat) * 86.0f;
                haveAim = true;
            }
        }
    }

    std::memcpy(g_pinnedEye, head, sizeof(g_pinnedEye));
    g_havePinnedEye = true;

    __try {
        // Before anything is written: this is the only moment the game's own
        // value is still there to be read.
        float was[3];
        std::memcpy(was, camera + 0x30, sizeof(was));
        if (was[0] == was[0] && was[1] == was[1] && was[2] == was[2]) {
            std::memcpy(g_gameEye, was, sizeof(was));
            g_haveGameEye = true;
            g_gameEyeFrom = camera;
        }
        for (DWORD off : kOffCameraPlace)
            std::memcpy(camera + off, head, sizeof(head));
        // THE AIM IS NOT OURS TO SET (2026-09-27, user: "even pressing the
        // aiming button sends my character into a full non stop 360").
        //
        // Aiming makes the game turn the character to face where the camera
        // looks, and we were setting where the camera looks from the character
        // facing. Body turns, camera follows, game turns the body to match,
        // round and round. Positive feedback, not an error in the arithmetic.
        //
        // Third time tonight, so the rule is worth stating exactly: nothing
        // written into the camera may be derived from anything the game
        // derives FROM the camera. camPos failed it, then forward, and now the
        // body facing, because aiming is what closes that particular loop.
        //
        // So the target stays the game's. It was only there to stop the whip
        // on a vault, and a whip is worth a great deal less than a spin.
        (void)aim;
        (void)haveAim;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    static unsigned long long s_saidAt = 0;
    const unsigned long long nowPin = GetTickCount64();
    if (nowPin - s_saidAt > 1000) {
        s_saidAt = nowPin;
        // WHICH KIND OF WRONG (2026-09-27, user: "my head was whipped to the
        // side when jumping a gap, like a full look left").
        //
        // Two causes, needing opposite fixes. Either the transform row I read
        // is the body's RIGHT axis rather than its forward, which would be a
        // steady ninety degrees off at all times, or the body genuinely spins
        // during the vault animation and we are following it faithfully. One
        // is a constant and the other is a spike, and a yaw printed every
        // second separates them without another guess.
        if (g_lookAtsCaught) {
            Log_Printf("CameraPin: %lu action-camera placement(s) through %lu redirected call site(s), %s",
                g_lookAtsCaught, g_lookAtSiteCount,
                g_holdViewInMelee.load(std::memory_order_relaxed) ? "your view held" : "let through untouched");
            g_lookAtsCaught = 0;
        }
        Log_Printf("CameraPin: held at (%.0f %.0f %.0f), aiming %.0f deg", head[0], head[1], head[2],
            haveAim ? std::atan2(aim[0] - head[0], aim[2] - head[2]) * 180.0f / kPi : -999.0f);
    }
}

void CameraRigHook_GetRunLead(float out[3])
{
    out[0] = g_leadX.load(std::memory_order_relaxed);
    out[1] = 0.0f;
    out[2] = g_leadZ.load(std::memory_order_relaxed);
}

bool CameraRigHook_GetPlayerWorldPos(float out[3])
{
    unsigned char* controller = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    unsigned char* character = nullptr;
    if (!out || !controller || !TryRead(&character, controller + kOffControllerCharacter, sizeof(character))
        || !character)
        return false;
    return TryRead(out, character + kOffBodyTransform[0] + 0x30, sizeof(float) * 3);
}

void CameraRigHook_ForgetArms()
{
    g_forgetArms.store(true, std::memory_order_relaxed);
    g_boneTableTold = nullptr;
}

// A DROPPED FRAME IS NOT A SCRIPTED CAMERA (2026-09-26, user: "my camera is
// escaping my character now when sprinting up stairs").
//
// The log has it plainly: "the game took the camera - it was 4 units from
// where the body puts the eye". Four units is nothing - the camera is sitting
// exactly where the body puts it, so nothing has taken it anywhere. What
// actually happened is that this test asks whether the rig has been quiet for
// 100 ms, and the game is running at 16 frames a second, where a single frame
// is 62 ms. Two frames in a row without a rig update and ordinary play reads
// as a cutscene.
//
// The watchdog that gives the head back still wants a short fuse, so that one
// keeps its 100 ms. This is a different question and gets its own: quiet for
// long enough that no frame rate explains it.
constexpr unsigned long long kScriptedQuietMs = 400;

bool CameraRigHook_InScriptedCamera()
{
    if (!g_enabled)
        return false;
    const unsigned long long last = g_lastPlayerHeadMs;
    return last != 0 && GetTickCount64() - last >= kScriptedQuietMs;
}

bool CameraRigHook_GetAimServoStatus(AimServoStatus& out)
{
    const unsigned long long when = g_servoMs.load(std::memory_order_relaxed);
    if (!when || GetTickCount64() - when > 250)
        return false;
    out.controllerPitchDeg = g_servoControllerPitch.load(std::memory_order_relaxed);
    out.rigPitchDeg = g_servoRigPitch.load(std::memory_order_relaxed);
    out.errorDeg = g_servoError.load(std::memory_order_relaxed);
    out.wristPitchRate = g_servoWristPitchRate.load(std::memory_order_relaxed);
    out.wristYawRate = g_servoWristYawRate.load(std::memory_order_relaxed);
    out.stickX = g_servoStickX.load(std::memory_order_relaxed);
    out.stickY = g_servoStickY.load(std::memory_order_relaxed);
    out.maxRateDeg = g_servoMaxRate.load(std::memory_order_relaxed);
    out.fastestSeenDeg = g_servoFastestSeen.load(std::memory_order_relaxed);
    out.yawDebtDeg = g_servoYawDebt.load(std::memory_order_relaxed);
    out.gunYawRate = g_servoGunYawRate.load(std::memory_order_relaxed);
    out.pitchScale = g_servoPitchScale.load(std::memory_order_relaxed);
    out.yawScale = g_servoYawScale.load(std::memory_order_relaxed);
    return true;
}

bool CameraRigHook_GetAimStick(float* x, float* y)
{
    // Stale the moment the gun comes down or the aim loop stops running.
    const unsigned long long when = g_aimStickMs.load(std::memory_order_relaxed);
    if (!when || GetTickCount64() - when > 200)
        return false;
    const float sx = g_aimStickX.load(std::memory_order_relaxed);
    const float sy = g_aimStickY.load(std::memory_order_relaxed);
    if (sx == 0.0f && sy == 0.0f)
        return false;
    if (x)
        *x = sx;
    if (y)
        *y = sy;
    return true;
}

float CameraRigHook_GetVrCullFovDeg(bool* atCap)
{
    const float deg = g_vrCullFovDegShown.load(std::memory_order_relaxed);
    if (atCap)
        *atCap = deg >= kVrCullFovMaxDeg;
    return deg;
}

bool CameraRigHook_MatchHeadFollowTargets(const float camForward[3], HeadFollowTargets& out, int* outAge, float* outErrDeg)
{
    // Deliberately NOT gated on g_headFollowDriving (2026-09-15): around the
    // start and end of aiming the game still draws a frame or two from the
    // camera head-follow last wrote, while the flag has already flipped, and
    // the two maths disagreed mid-turn - the user's jitter while aiming. The
    // question is what THIS camera was built from, so the direction decides:
    // within kMatchToleranceDeg of a recent write, it's a head-follow camera.
    if (GetTickCount64() - g_hfTargetsMs.load(std::memory_order_acquire) > 250)
        return false;
    // Left-right only (world y is up). The camera sits a steady ~2 deg off the
    // written directions vertically - the game's look-up/look-down rig blend -
    // while every rig gets exactly the same sideways turn, so yaw picks the right
    // update even mid-turn, when neighbouring updates are only a few degrees
    // apart. Matching in 3D let that vertical offset pick the wrong one: the
    // catch-up then over-rotated 1.2-1.4x and flickered (2026-09-15).
    const float cl = std::sqrt(camForward[0] * camForward[0] + camForward[2] * camForward[2]);
    if (cl < 1e-4f)
        return false;

    HeadFollowTargets history[kHfHistory];
    int head = 0, count = 0;
    AcquireSRWLockShared(&g_hfHistoryLock);
    std::memcpy(history, g_hfHistory, sizeof(history));
    head = g_hfHistoryHead;
    count = g_hfHistoryCount;
    ReleaseSRWLockShared(&g_hfHistoryLock);
    if (head < 0 || count == 0)
        return false;

    int bestAge = -1;
    float bestDot = -2.0f;
    for (int age = 0; age < count; ++age) {
        const HeadFollowTargets& t = history[(head - age + kHfHistory) % kHfHistory];
        for (int r = 0; r < 7; ++r) {
            const float* d = t.worldDir[r];
            const float dl = std::sqrt(d[0] * d[0] + d[2] * d[2]);
            if (dl < 1e-4f)
                continue;
            const float dot = (d[0] * camForward[0] + d[2] * camForward[2]) / (dl * cl);
            // Strictly better only: at a tie the newer update, seen first, keeps it.
            if (dot > bestDot + 1e-5f) {
                bestDot = dot;
                bestAge = age;
            }
        }
    }
    constexpr float kMatchToleranceDeg = 5.0f;
    if (bestAge < 0 || bestDot < std::cos(kMatchToleranceDeg * kPi / 180.0f))
        return false;
    out = history[(head - bestAge + kHfHistory) % kHfHistory];
    if (outAge)
        *outAge = bestAge;
    if (outErrDeg)
        *outErrDeg = std::acos(std::fmax(-1.0f, std::fmin(1.0f, bestDot))) * 180.0f / kPi;
    return true;
}

void CameraRigHook_SetAimViewTest(bool on)
{
    if (g_aimViewTest.exchange(on) != on)
        Log_Printf("CameraRigHook: aim-view test (view follows the head while aiming) now %s", on ? "ON" : "OFF");
}

bool CameraRigHook_GetAimViewTest()
{
    return g_aimViewTest.load(std::memory_order_relaxed);
}

void CameraRigHook_DoubleHeadAim(const float f[3], float out[3])
{
    DoubleHeadAim(f, out);
}

bool CameraRigHook_GetHeadFollowTargets(HeadFollowTargets& out)
{
    const int front = g_hfTargetsFront.load(std::memory_order_acquire);
    if (front < 0 || !g_headFollowDriving.load(std::memory_order_acquire))
        return false;
    if (GetTickCount64() - g_hfTargetsMs.load(std::memory_order_acquire) > 250)
        return false;
    out = g_hfTargets[front];
    return true;
}

bool CameraRigHook_HeadFollowDrivingCamera()
{
    return g_headFollowDriving.load(std::memory_order_acquire);
}

int CameraRigHook_GetLiveBases(void** outBases, bool* outAim, int maxCount)
{
    ExpireLiveBases();
    const int n = g_liveCount.load(std::memory_order_relaxed);
    int out = 0;
    for (int i = 0; i < n && out < maxCount; ++i) {
        if (!g_liveBases[i].base)
            continue;
        outBases[out] = g_liveBases[i].base;
        outAim[out] = g_liveBases[i].aim;
        ++out;
    }
    return out;
}

// ---- Settings and status for the in-game menu (2026-09-13) -------------
// Everything the old hotkeys changed. Written from the render thread and read
// by the camera hook on the game thread - the same race the hotkeys always
// had, and harmless for single bools and floats on x86.

namespace {
float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
} // namespace

void CameraRigHook_SetFirstPerson(bool on)
{
    if (on == g_enabled)
        return;
    g_enabled = on;
    Log_Printf("CameraRigHook: first-person camera override now %s", g_enabled ? "ON" : "OFF");
    // First person puts the camera inside Chris, which is exactly what
    // triggers the game's near-camera fade - so the fade goes with it.
    FadePatch_SetEnabled(g_enabled);
    Log_Printf("CameraRigHook: head collapse so far - %lu frame(s), game reset the head scale %lu time(s); "
               "the game cut the camera away %lu time(s). While the head stayed hidden the camera reached %ld "
               "from the eye, biggest single-frame move %ld (needs %.0f away AND +%.0f in one frame)",
        g_headCollapseFrames.load(std::memory_order_relaxed), g_headScaleResets.load(std::memory_order_relaxed),
        g_headShownEvents.load(std::memory_order_relaxed), g_headNearMaxDistance.load(std::memory_order_relaxed),
        g_headNearMaxJump.load(std::memory_order_relaxed), kHeadShowDistance, kHeadShowJump);
    Log_Printf("CameraRigHook: the watchdog gave the head back %lu time(s) (camera hook quiet over %llu ms); "
               "the flicker guard locked the head hidden %lu time(s)",
        g_watchdogRestores.load(std::memory_order_relaxed), kHookQuietMs,
        g_headLockouts.load(std::memory_order_relaxed));
    g_diagSamplesRemaining.store(12, std::memory_order_relaxed);
}

CameraRigSettings CameraRigHook_GetSettings()
{
    CameraRigSettings s;
    s.firstPerson = g_enabled;
    s.headFollow = g_headFollow.load(std::memory_order_relaxed);
    s.vrStabilise = g_vrStabiliseEye.load(std::memory_order_relaxed);
    s.vrMatchCullFov = g_vrWideFov.load(std::memory_order_relaxed);
    s.vrCullMarginPct = g_vrCullMarginPct.load(std::memory_order_relaxed);
    s.vrHeadSteady = g_vrHeadSteady.load(std::memory_order_relaxed);
    s.vrRunLead = g_vrRunLead.load(std::memory_order_relaxed);
    s.vrViewSteady = g_vrViewSteady.load(std::memory_order_relaxed);
    s.vrEyeOnNeck = g_vrEyeOnNeck.load(std::memory_order_relaxed);
    s.vrEyeAboveNeck = g_vrEyeAboveNeck.load(std::memory_order_relaxed);
    s.vrCullTurnLookaheadMs = g_vrCullTurnLookaheadMs.load(std::memory_order_relaxed);
    s.showHeadDuringActions = g_showHeadDuringActions.load(std::memory_order_relaxed);
    s.holdViewInMelee = g_holdViewInMelee.load(std::memory_order_relaxed);
    s.partnerNoCamera = g_partnerNoCamera.load(std::memory_order_relaxed);
    s.scopeThirdPerson = g_scopeThirdPerson.load(std::memory_order_relaxed);
    s.headLock = g_keepCameraOnHead.load(std::memory_order_relaxed);
    s.headLockReach = g_headLockMaxDistance.load(std::memory_order_relaxed);
    s.headLockDirection = g_headLockDirection.load(std::memory_order_relaxed);
    s.viewTurnLimitDeg = g_viewTurnLimitDeg.load(std::memory_order_relaxed);
    s.aimWalkCommit = g_aimWalkCommit.load(std::memory_order_relaxed);
    s.aimWalkCommitHz = g_aimWalkCommitHz.load(std::memory_order_relaxed);
    s.aimWalkThirdPerson = g_aimWalkThirdPerson.load(std::memory_order_relaxed);
    s.flatFovDeg = g_flatFovDeg;
    s.flatEyeUp = g_flatEye.up;
    s.flatEyeAhead = g_flatEye.ahead;
    s.vrEyeUp = g_vrEye.up;
    s.vrEyeAhead = g_vrEye.ahead;
    return s;
}

void CameraRigHook_ApplySettings(const CameraRigSettings& in)
{
    CameraRigSettings s = in;
    s.flatFovDeg = Clamp(s.flatFovDeg, kFlatFovMin, kFlatFovMax);
    s.flatEyeUp = Clamp(s.flatEyeUp, 0.0f, kEyeUpMax);
    s.vrEyeUp = Clamp(s.vrEyeUp, 0.0f, kEyeUpMax);
    s.flatEyeAhead = Clamp(s.flatEyeAhead, kEyeAheadMin, kEyeScaleMax);
    s.vrEyeAhead = Clamp(s.vrEyeAhead, kEyeAheadMin, kEyeScaleMax);
    s.vrCullMarginPct = Clamp(s.vrCullMarginPct, 0.0f, kVrCullMarginMaxPct);
    s.vrCullTurnLookaheadMs = Clamp(s.vrCullTurnLookaheadMs, 0.0f, kVrCullTurnLookaheadMaxMs);
    s.vrHeadSteady = Clamp(s.vrHeadSteady, 0.0f, 1.0f);

    const CameraRigSettings old = CameraRigHook_GetSettings();
    CameraRigHook_SetFirstPerson(s.firstPerson);
    const auto flag = [](std::atomic<bool>& a, bool v, bool was, const char* name) {
        a.store(v, std::memory_order_relaxed);
        if (v != was)
            Log_Printf("CameraRigHook: %s now %s", name, v ? "ON" : "OFF");
    };
    flag(g_headFollow, s.headFollow, old.headFollow, "head tracking turns the game camera");
    flag(g_vrStabiliseEye, s.vrStabilise, old.vrStabilise, "VR camera stabilisation");
    flag(g_vrWideFov, s.vrMatchCullFov, old.vrMatchCullFov, "VR culling FOV matched to the headset");
    flag(g_showHeadDuringActions, s.showHeadDuringActions, old.showHeadDuringActions, "show head during action cameras");
    flag(g_holdViewInMelee, s.holdViewInMelee, old.holdViewInMelee, "refusing the melee and jump cameras");
    flag(g_partnerNoCamera, s.partnerNoCamera, old.partnerNoCamera, "partner locate leaving the camera alone");
    flag(g_scopeThirdPerson, s.scopeThirdPerson, old.scopeThirdPerson, "scoped weapons in third person");
    meleecam::SetOn(s.holdViewInMelee);
    actioncam::Set(0, s.holdViewInMelee);
    actioncam::Set(1, s.partnerNoCamera);
    actioncam::Set(2, s.scopeThirdPerson);
    flag(g_keepCameraOnHead, s.headLock, old.headLock, "keeping the view on your head during action cameras");
    s.headLockReach = Clamp(s.headLockReach, 100.0f, 4000.0f);
    g_headLockMaxDistance.store(s.headLockReach, std::memory_order_relaxed);
    s.viewTurnLimitDeg = Clamp(s.viewTurnLimitDeg, 0.0f, 2000.0f);
    if (s.viewTurnLimitDeg != old.viewTurnLimitDeg)
        Log_Printf("CameraRigHook: the game may swing your view at %.0f deg/sec at most", s.viewTurnLimitDeg);
    g_viewTurnLimitDeg.store(s.viewTurnLimitDeg, std::memory_order_relaxed);
    {
        static const char* const kCommitNames[] = { "off", "the moved flag only",
            "the step handler only", "both", "both, then the flag put back" };
        const int mode = s.aimWalkCommit < 0 || s.aimWalkCommit > kCommitTidy ? kCommitOff : s.aimWalkCommit;
        g_aimWalkCommit.store(mode, std::memory_order_relaxed);
        if (mode != old.aimWalkCommit)
            Log_Printf("CameraRigHook: aim-walk step commit is now %s", kCommitNames[mode]);
        flag(g_aimWalkThirdPerson, s.aimWalkThirdPerson, old.aimWalkThirdPerson,
            "walking while aiming in third person");
        const float hz = Clamp(s.aimWalkCommitHz, 0.0f, 120.0f);
        g_aimWalkCommitHz.store(hz, std::memory_order_relaxed);
        if (hz != old.aimWalkCommitHz) {
            if (hz > 0.0f)
                Log_Printf("CameraRigHook: aim-walk commits at most %.0f times a second", hz);
            else
                Log_Printf("CameraRigHook: aim-walk commits every frame");
        }
    }

    g_vrCullMarginPct.store(s.vrCullMarginPct, std::memory_order_relaxed);
    if (s.vrCullMarginPct != old.vrCullMarginPct)
        Log_Printf("CameraRigHook: VR culling margin now %.0f%%", s.vrCullMarginPct);
    g_vrHeadSteady.store(s.vrHeadSteady, std::memory_order_relaxed);
    g_vrViewSteady.store(s.vrViewSteady < 0.0f ? 0.0f : (s.vrViewSteady > 1.0f ? 1.0f : s.vrViewSteady),
        std::memory_order_relaxed);
    g_vrRunLead.store(s.vrRunLead < 0.0f ? 0.0f : (s.vrRunLead > 4.0f ? 4.0f : s.vrRunLead),
        std::memory_order_relaxed);
    g_vrEyeOnNeck.store(s.vrEyeOnNeck, std::memory_order_relaxed);
    g_vrEyeAboveNeck.store(s.vrEyeAboveNeck < 0.0f ? 0.0f : (s.vrEyeAboveNeck > 60.0f ? 60.0f : s.vrEyeAboveNeck),
        std::memory_order_relaxed);
    if (s.vrHeadSteady != old.vrHeadSteady) {
        if (s.vrHeadSteady <= 0.0f)
            Log_Printf("CameraRigHook: head steadying off - the camera follows every twitch");
        else
            Log_Printf("CameraRigHook: head steadying now %.2f (resting cutoff %.1f Hz)", s.vrHeadSteady,
                kHeadSteadyRestHz - (kHeadSteadyRestHz - kHeadSteadyHoldHz) * s.vrHeadSteady);
    }
    g_vrCullTurnLookaheadMs.store(s.vrCullTurnLookaheadMs, std::memory_order_relaxed);
    if (s.vrCullTurnLookaheadMs != old.vrCullTurnLookaheadMs)
        Log_Printf("CameraRigHook: VR culling turn lookahead now %.0f ms", s.vrCullTurnLookaheadMs);

    g_flatFovDeg = s.flatFovDeg;
    g_flatEye.up = s.flatEyeUp;
    g_flatEye.ahead = s.flatEyeAhead;
    g_vrEye.up = s.vrEyeUp;
    g_vrEye.ahead = s.vrEyeAhead;
    if (s.flatFovDeg != old.flatFovDeg || s.flatEyeUp != old.flatEyeUp || s.flatEyeAhead != old.flatEyeAhead ||
        s.vrEyeUp != old.vrEyeUp || s.vrEyeAhead != old.vrEyeAhead) {
        Log_Printf("CameraRigHook: flat FOV %.0f, flat eye up %.2f fwd %.2f, VR eye up %.2f fwd %.2f",
            g_flatFovDeg, g_flatEye.up, g_flatEye.ahead, g_vrEye.up, g_vrEye.ahead);
    }
}

void CameraRigHook_GetStatus(CameraRigStatus& out)
{
    out = CameraRigStatus{};
    const SkeletonId skel = g_playerSkeleton;
    if (skel.valid) {
        out.playerJointCount = skel.jointCount;
        out.player = skel.headIndex == 4 ? 1 : (skel.headIndex == 23 ? 2 : 3);
    }
    const unsigned long long last = g_lastPlayerHeadMs;
    out.cameraHookAgeMs = last ? GetTickCount64() - last : ~0ull;
    // Under the view split nothing steers the game camera any more; "driving"
    // means the drawn view is following the head right now.
    out.headFollowDriving = g_headFollowDriving.load(std::memory_order_relaxed) ||
        (g_aimViewTest.load(std::memory_order_relaxed) &&
            GetTickCount64() - g_hfTargetsMs.load(std::memory_order_relaxed) < 250);
    out.headCutaways = g_headShownEvents.load(std::memory_order_relaxed);
    out.watchdogRestores = g_watchdogRestores.load(std::memory_order_relaxed);
    out.flickerLockouts = g_headLockouts.load(std::memory_order_relaxed);
}
