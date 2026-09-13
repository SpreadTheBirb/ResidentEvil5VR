#include "camera_rig_hook.h"
#include "constant_probe.h"
#include "fade_patch.h"
#include "../render/stereo_test.h"
#include "../vr/openxr_bridge.h"
#include "../util/log.h"
#include "../util/build_config.h"

#include <MinHook.h>
#include <windows.h>
#include <Xinput.h>

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
constexpr float kVrCullFovMargin = 1.15f;
constexpr float kVrCullFovMinDeg = 90.0f;
constexpr float kVrCullFovMaxDeg = 170.0f;

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

    float deg = std::fmax(vertical, verticalForHorizontal) * 180.0f / kPi * kVrCullFovMargin;
    if (deg < kVrCullFovMinDeg)
        deg = kVrCullFovMinDeg;
    if (deg > kVrCullFovMaxDeg)
        deg = kVrCullFovMaxDeg;

    static float s_lastLogged = 0.0f;
    if (std::fabs(deg - s_lastLogged) > 1.0f) {
        s_lastLogged = deg;
        Log_Printf("CameraRigHook: VR culling FOV from the headset - %.0f deg vertical (headset %.0f v / %.0f h, "
                   "+%.0f%% margin)",
            deg, vertical * 180.0f / kPi, horizontal * 180.0f / kPi, (kVrCullFovMargin - 1.0f) * 100.0f);
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
void ApplyHeadFollow(float* dx, float* dy, float* dz)
{
    // Take the head direction from the pose LATCHED for this frame - the very
    // same one the eye matrices use. Reading the live published pose here
    // instead (which is what this did until 2026-09-12) meant the camera was
    // steered by a fresher snapshot than the image was rendered with: at
    // moderate speed the view dragged behind, and on a fast turn it overshot
    // and snapped back as the two reconverged. One pose per frame, everywhere;
    // the remaining age is what the compositor's reprojection is for, and it
    // is told exactly which pose the frame used.
    float latched[3];
    const bool haveLatched = StereoTest_GetLatchedHeadForward(latched);
    const int front = g_headForwardFront.load(std::memory_order_acquire);
    const float fx = haveLatched ? latched[0] : g_headForward[front][0];
    const float fy = haveLatched ? latched[1] : g_headForward[front][1];
    const float fz = haveLatched ? latched[2] : g_headForward[front][2];
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
constexpr float kHeadShowDistance = 60.0f;             // never on a camera nearer than this...
constexpr float kHeadShowJump = 50.0f;                 // ...and only when it got there in one frame
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

    // Eye in rig space. VR gets its own forward offset (g_vrEyeAheadScale).
    float pivotNormal[3], pivotAim[3];
    WorldToRig(controller, kOffNormalTransform, headWorld, pivotNormal);
    WorldToRig(controller, kOffAimTransform, headWorld, pivotAim);
    // Plausibility: a head pivot is roughly above the root, at head height.
    const bool plausible = pivotNormal[1] >= 80.0f && pivotNormal[1] <= 250.0f &&
        std::fabs(pivotNormal[0]) <= 100.0f && std::fabs(pivotNormal[2]) <= 100.0f;
    const EyeOffsetScale& eyeScale = vrActive ? g_vrEye : g_flatEye;
    const float ahead = track->eyeAhead * eyeScale.ahead;
    const float up = track->eyeUp * eyeScale.up;
    const float eyeRigNormal[3] = { pivotNormal[0], pivotNormal[1] + up, pivotNormal[2] + ahead };
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
            if (track->distance > kHeadShowDistance && jump > kHeadShowJump && !HeadLockedHidden(now)) {
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
constexpr DWORD kOffCharacterPos = 0x30;
constexpr float kAimWalkSpeed = 100.0f; // render units per second - first guess, tune by feel
// After each step, do what the game's movement code does (see the end of
// AimWalk): set the character's "moved" bit and run its step handler.
// Multiplayer test switch (F6), default OFF. While on, each aim-walk step is
// committed the way the game's own movement code does: the character's
// "moved" flag plus its step handler. That may be what makes the move reach a
// co-op partner - but the weapon path consumes the same flag, so the gun
// fires several rounds per trigger pull while it is on (user, 2026-09-11).
// For a co-op sync test only; leave it off to play.
std::atomic<bool> g_aimWalkCommit{ false };
constexpr DWORD kOffCharacterStepFlags = 0x2D7C;
constexpr DWORD kStepMovedFlag = 0x10000000;
constexpr DWORD kOffCharacterStepHandler = 0x2F30;
constexpr DWORD kRvaStepHandlerUpdate = 0x864B40;

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

bool ReadLeftStick(float* forward, float* strafe)
{
    XINPUT_STATE st = {};
    if (!ReadPadState(&st))
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

void AimWalk(unsigned char* controller)
{
    if (!kMoveWhileAiming || !IsPlayerController(controller))
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
    if (g_aimWalkCommit.load(std::memory_order_relaxed)) {
        *reinterpret_cast<DWORD*>(character + kOffCharacterStepFlags) |= kStepMovedFlag;
        void* handler = nullptr;
        if (TryRead(&handler, character + kOffCharacterStepHandler, sizeof(handler)) && handler) {
            typedef void(__fastcall * StepHandler_t)(void* self, void* edxUnused);
            const auto update = reinterpret_cast<StepHandler_t>(
                reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr)) + kRvaStepHandlerUpdate);
            update(handler, nullptr);
        }
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
    if (!g_enabled)
        return;
    AimWalk(controller);
    const float fov = vrActive && g_vrWideFov.load(std::memory_order_relaxed) ? VrCullVerticalFov()
                                                                              : FirstPersonVerticalFov();

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
    if (vrActive && isPlayer && g_headFollow.load(std::memory_order_relaxed))
        MeasureHeadFollowLag();
    const bool headFollow = vrActive && isPlayer && !aiming && g_headFollow.load(std::memory_order_relaxed);
    if (isPlayer)
        g_headFollowDriving.store(headFollow, std::memory_order_release);
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
        if (headFollow)
            ApplyHeadFollow(&nx, &ny, &nz);
        PlaceRigAtEye(normalRig, eyeNormal, nx, ny, nz, fov);

        float ax = 0.0f;
        float ay = aimOk[i] ? aimDy[i] : baseDy;
        float az = aimOk[i] ? aimDz[i] : baseDz;
        if (headFollow)
            ApplyHeadFollow(&ax, &ay, &az);
        PlaceRigAtEye(controller + kOffAimRigs + i * kRigStride, eyeAim, ax, ay, az, fov);
    }
}

__declspec(naked) void RigsReadyHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call CameraRigHook_OnRigsReady
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
    if (HeadLockedHidden(now))
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
} // namespace

void CameraRigHook_Install()
{
    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("CameraRigHook_Install: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    uintptr_t moduleBase = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));

    void* rigsReadyTarget = reinterpret_cast<void*>(moduleBase + 0x446965);
    MH_STATUS rrSt = MH_CreateHook(rigsReadyTarget, reinterpret_cast<void*>(&RigsReadyHook_Stub), &g_rigsReadyTrampoline);
    if (rrSt != MH_OK && rrSt != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("CameraRigHook_Install: rigs-ready MH_CreateHook failed -> %d", static_cast<int>(rrSt));
        return;
    }
    rrSt = MH_EnableHook(rigsReadyTarget);
    Log_Printf("CameraRigHook_Install: rigs-ready hook enabled -> %d (target=%p)", static_cast<int>(rrSt), rigsReadyTarget);

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

void CameraRigHook_OnEndScene()
{
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
    const bool scrollDown = (GetAsyncKeyState(VK_SCROLL) & 0x8000) != 0;
    const bool actionDown = fDown || scrollDown;
    if (actionDown && !prevActionDown && g_enabled) {
        g_actionTraceUntilMs = GetTickCount64() + kActionTraceMs;
        Log_Printf("CameraRigHook: %s pressed - tracing the camera for %llu ms",
            fDown ? "F (action)" : "Scroll Lock (no action)", kActionTraceMs);
    }
    prevActionDown = actionDown;
#endif // RE5VR_DIAGNOSTICS

    static bool prevF4Down = false;
    bool f4Down = (GetAsyncKeyState(VK_F4) & 0x8000) != 0;
    if (f4Down && !prevF4Down) {
        g_enabled = !g_enabled;
        Log_Printf("CameraRigHook: F4 pressed, first-person camera override now %s", g_enabled ? "ON" : "OFF");
        // First person puts the camera inside Chris, which is exactly what
        // triggers the game's near-camera fade - so the fade goes with it.
        FadePatch_SetEnabled(g_enabled);
        Log_Printf("CameraRigHook: head collapse so far - %lu frame(s), game reset the head scale %lu time(s); "
                   "the game cut the camera away %lu time(s). While the head stayed hidden the camera reached %ld "
                   "from the eye, biggest single-frame move %ld (needs %.0f away AND +%.0f in one frame)",
            g_headCollapseFrames.load(std::memory_order_relaxed), g_headScaleResets.load(std::memory_order_relaxed),
            g_headShownEvents.load(std::memory_order_relaxed), g_headNearMaxDistance.load(std::memory_order_relaxed),
            g_headNearMaxJump.load(std::memory_order_relaxed), kHeadShowDistance, kHeadShowJump);
        Log_Printf("CameraRigHook: the watchdog gave the head back %lu time(s) (camera hook quiet over %llu ms)",
            g_watchdogRestores.load(std::memory_order_relaxed), kHookQuietMs);
        Log_Printf("CameraRigHook: the flicker guard locked the head hidden %lu time(s)",
            g_headLockouts.load(std::memory_order_relaxed));
        // Stub hit counts since process start. If these stay at 0 across a
        // whole session, the hook sites are never executed and no amount of
        // tuning the written values will ever do anything.
        Log_Printf("CameraRigHook: stub hits so far - NORMAL=%llu AIM=%llu",
            g_normalHits.load(std::memory_order_relaxed),
            g_aimHits.load(std::memory_order_relaxed));
        g_diagSamplesRemaining.store(12, std::memory_order_relaxed);
    }

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

    // F11 = VR camera stabilisation on/off (see g_vrStabiliseEye), so the
    // idle-animation shake can be compared directly.
    static bool prevF11Down = false;
    const bool f11Down = (GetAsyncKeyState(VK_F11) & 0x8000) != 0;
    if (f11Down && !prevF11Down) {
        const bool on = !g_vrStabiliseEye.load(std::memory_order_relaxed);
        g_vrStabiliseEye.store(on, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: F11 pressed, VR camera stabilisation now %s", on ? "ON" : "OFF");
    }
    prevF11Down = f11Down;

    // F9 = the head-follow experiment (see g_headFollow). VR only.
    static bool prevF9Down = false;
    const bool f9Down = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
    if (f9Down && !prevF9Down) {
        const bool on = !g_headFollow.load(std::memory_order_relaxed);
        g_headFollow.store(on, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: F9 pressed, game camera follows the head now %s%s", on ? "ON" : "OFF",
            on ? " - VR only; the game aims where its camera points, so you will aim with your head" : "");
    }
    prevF9Down = f9Down;

    // Reset view (testers: "def needs some reset view function"). F5 on the
    // keyboard, or both sticks clicked in and HELD for a second on a pad - in
    // a headset you cannot see the keyboard. Stick clicks were picked because
    // RE5 leans on the d-pad and face buttons (inventory, "hold B + d-pad" to
    // call Sheva), and the hold keeps a stray double-click from firing it.
    // Diagnostics builds also bind F5 to the boom finder; release builds don't.
    const bool vrOn = g_vrActive.load(std::memory_order_relaxed);
    static bool prevF5Down = false;
    const bool f5Down = (GetAsyncKeyState(VK_F5) & 0x8000) != 0;
    if (f5Down && !prevF5Down && vrOn)
        VRBridge_RequestRecenter("F5");
    prevF5Down = f5Down;

    // EndScene runs many times a frame; the pad only needs looking at ~20x/s.
    static ULONGLONG s_lastPadPollMs = 0, s_sticksHeldSinceMs = 0;
    static bool s_sticksFired = false;
    const ULONGLONG padNowMs = GetTickCount64();
    if (vrOn && padNowMs - s_lastPadPollMs >= 50) {
        s_lastPadPollMs = padNowMs;
        XINPUT_STATE pad = {};
        constexpr WORD kBothSticks = XINPUT_GAMEPAD_LEFT_THUMB | XINPUT_GAMEPAD_RIGHT_THUMB;
        const bool held = ReadPadState(&pad) && (pad.Gamepad.wButtons & kBothSticks) == kBothSticks;
        if (!held) {
            s_sticksHeldSinceMs = 0;
            s_sticksFired = false;
        } else if (!s_sticksHeldSinceMs) {
            s_sticksHeldSinceMs = padNowMs;
        } else if (!s_sticksFired && padNowMs - s_sticksHeldSinceMs >= 1000) {
            s_sticksFired = true; // once per hold
            VRBridge_RequestRecenter("both sticks held");
        }
    }

    // F6 = the co-op aim-walk sync test (see g_aimWalkCommit).
    static bool prevF6Down = false;
    const bool f6Down = (GetAsyncKeyState(VK_F6) & 0x8000) != 0;
    if (f6Down && !prevF6Down) {
        const bool on = !g_aimWalkCommit.load(std::memory_order_relaxed);
        g_aimWalkCommit.store(on, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: F6 pressed, aim-walk step commit now %s%s", on ? "ON" : "OFF",
            on ? " - co-op sync test; the gun will fire several rounds per trigger pull" : "");
    }
    prevF6Down = f6Down;

    // '`' = wide culling FOV in VR on/off (g_vrWideFov).
    static bool prevGraveDown = false;
    const bool graveDown = (GetAsyncKeyState(VK_OEM_3) & 0x8000) != 0;
    if (graveDown && !prevGraveDown) {
        const bool wide = !g_vrWideFov.load(std::memory_order_relaxed);
        g_vrWideFov.store(wide, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: '`' pressed, VR culling FOV now %s",
            wide ? "WIDE (derived from the headset frustum)" : "normal (90 deg horizontal)");
    }
    prevGraveDown = graveDown;

    // Live eye tuning: ',' / '.' = back / forward, Shift with them = down / up.
    // Whichever mode is running gets tuned, so a VR session never disturbs the
    // flat-screen position or the other way round.
    static bool prevCommaDown = false, prevPeriodDown = false;
    const bool commaDown = (GetAsyncKeyState(VK_OEM_COMMA) & 0x8000) != 0;
    const bool periodDown = (GetAsyncKeyState(VK_OEM_PERIOD) & 0x8000) != 0;
    const bool commaPressed = commaDown && !prevCommaDown;
    const bool periodPressed = periodDown && !prevPeriodDown;
    if (commaPressed || periodPressed) {
        const float direction = periodPressed ? 1.0f : -1.0f;
        const bool fov = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
        const bool height = !fov && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
        if (fov) {
            // 90 reads wide and makes Chris feel short of his 6 feet (user,
            // 2026-09-11); 75-80 usually feels right. Flat first person only:
            // VR renders with the headset's own FOV, and the FOV the game
            // culls with in VR is separate again (kVrCullVerticalFovDeg).
            g_flatFovDeg += direction * kFlatFovStep;
            if (g_flatFovDeg < kFlatFovMin)
                g_flatFovDeg = kFlatFovMin;
            if (g_flatFovDeg > kFlatFovMax)
                g_flatFovDeg = kFlatFovMax;
            Log_Printf("CameraRigHook: flat first-person FOV now %.0f deg horizontal", g_flatFovDeg);
        } else {
            const bool vrActive = g_vrActive.load(std::memory_order_relaxed);
            EyeOffsetScale& eye = vrActive ? g_vrEye : g_flatEye;
            float& value = height ? eye.up : eye.ahead;
            value += direction * kEyeScaleStep;
            const float minValue = height ? 0.0f : kEyeAheadMin;
            if (value < minValue)
                value = minValue;
            const float maxValue = height ? kEyeUpMax : kEyeScaleMax;
            if (value > maxValue)
                value = maxValue;
            Log_Printf("CameraRigHook: %s eye %s now %.1f (1.0 = the skeleton eye, 0 = the head joint) - VR up %.1f fwd %.1f, flat up %.1f fwd %.1f",
                vrActive ? "VR" : "flat", height ? "height" : "forward", value,
                g_vrEye.up, g_vrEye.ahead, g_flatEye.up, g_flatEye.ahead);
        }
    }
    prevCommaDown = commaDown;
    prevPeriodDown = periodDown;

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
    prevF4Down = f4Down;
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
