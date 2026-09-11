#include "camera_rig_hook.h"
#include "constant_probe.h"
#include "fade_patch.h"
#include "../vr/openxr_bridge.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <atomic>
#include <cmath>

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
constexpr float kFirstPersonHorizontalFovDeg = 90.0f;
constexpr float kPi = 3.14159265358979f;
float g_screenAspect = 16.0f / 9.0f;

float FirstPersonVerticalFov()
{
    const float halfH = kFirstPersonHorizontalFovDeg * 0.5f * kPi / 180.0f;
    return 2.0f * std::atan(std::tan(halfH) / g_screenAspect) * 180.0f / kPi;
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
std::atomic<bool> g_vrWideFov{true};
constexpr float kVrCullVerticalFovDeg = 150.0f;

// Eye placement. The flat-screen eye (15 ahead of the head pivot for Chris)
// felt too far forward in VR, where head tracking moves the view on top of
// it. This scales the forward offset in VR only; ',' / '.' tune it live.
// 0.2: the user settled at 0.0 on 2026-09-11 but said "probably a bit more
// forward" - and 0.0 puts the eye right above the neck pivot, inside the
// collar/neck geometry that the head collapse leaves (it belongs to the
// chest joint), a likely source of that session's black flicker.
float g_vrEyeAheadScale = 0.2f;
constexpr float kVrEyeAheadStep = 0.1f;

void PlaceRigAtEye(unsigned char* rig, const float eye[3], float dy, float dz, float fovDeg)
{
    float* f = reinterpret_cast<float*>(rig); // f[0..2] eye, f[4..6] target, f[9] FOV (+0x24)
    f[0] = eye[0];
    f[1] = eye[1];
    f[2] = eye[2];
    f[4] = eye[0];
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
};
HeadTrack g_heads[8] = {};
std::atomic<unsigned long> g_headCollapseFrames{0};
std::atomic<unsigned long> g_headScaleResets{0};
std::atomic<int> g_eyeLogsRemaining{0};

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
unsigned char* FindHeadJoint(unsigned char* joints, float* eyeUp, float* eyeAhead)
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
// UpdateHead. Sticky: only changes when another controller clearly matches.
const unsigned char* g_playerController = nullptr;
constexpr float kPlayerEyeMaxDistance = 25.0f;

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
            // Sx = Sy / aspect whatever the FOV, so this is the screen's shape.
            const float aspect = sy / sx;
            if (aspect > 1.0f && aspect < 3.0f)
                g_screenAspect = aspect;
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
        track->head = joints ? FindHeadJoint(joints, &track->eyeUp, &track->eyeAhead) : nullptr;
        track->collapsed = false;
        if (track->head)
            Log_Printf("CameraRigHook: controller %p follows a skeleton with its head at joint index %d; eye %.1f above, "
                       "%.1f ahead of the pivot",
                controller, static_cast<int>((track->head - joints) / kJointStride), track->eyeUp, track->eyeAhead);
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
        return;
    }

    // Eye in rig space. VR gets its own forward offset (g_vrEyeAheadScale).
    float pivotNormal[3], pivotAim[3];
    WorldToRig(controller, kOffNormalTransform, headWorld, pivotNormal);
    WorldToRig(controller, kOffAimTransform, headWorld, pivotAim);
    // Plausibility: a head pivot is roughly above the root, at head height.
    const bool plausible = pivotNormal[1] >= 80.0f && pivotNormal[1] <= 250.0f &&
        std::fabs(pivotNormal[0]) <= 100.0f && std::fabs(pivotNormal[2]) <= 100.0f;
    const float ahead = track->eyeAhead * (vrActive ? g_vrEyeAheadScale : 1.0f);
    const float eyeRigNormal[3] = { pivotNormal[0], pivotNormal[1] + track->eyeUp, pivotNormal[2] + ahead };
    const float eyeRigAim[3] = { pivotAim[0], pivotAim[1] + track->eyeUp, pivotAim[2] + ahead };

    // Which character is YOU: the one whose EYE is where last frame's camera
    // was. The first version compared head PIVOTS to the camera - but your
    // own pivot is ~19 units from your eye, so whenever Sheva's head came
    // within ~19 units of the camera she won and her head collapsed instead
    // (user, flat and VR). Your eye is essentially at the camera; hers
    // practically never is. Sticky, so a frame of bad camera data (a stray
    // pass slipping through the decode) can't flip it.
    track->distance = 1e9f;
    float cam[3];
    if (plausible && LastCameraPosition(cam)) {
        float eyeWorld[3];
        RigToWorld(controller, kOffNormalTransform, eyeRigNormal, eyeWorld);
        const float d[3] = { eyeWorld[0] - cam[0], eyeWorld[1] - cam[1], eyeWorld[2] - cam[2] };
        track->distance = Length3(d);
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
    const HeadTrack* best = nullptr;
    for (const HeadTrack& t : g_heads) {
        if (!t.controller || !t.head || now - t.ms > kHeadTrackExpiryMs || (anyDirect && !t.direct))
            continue;
        if (!best || t.distance < best->distance)
            best = &t;
    }
    if (best && best->distance < kPlayerEyeMaxDistance) {
        if (g_playerController != best->controller)
            Log_Printf("CameraRigHook: player controller now %p (%s, eye %.1f from the rendered camera)",
                best->controller, best->direct ? "main-camera candidate" : "fallback", best->distance);
        g_playerController = best->controller;
    }
    const bool nearest = controller == g_playerController;

    track->isPlayer = nearest;
    if (nearest) {
        const float* scale = reinterpret_cast<const float*>(head + kOffJointScale);
        if (track->collapsed && scale[0] != 0.0f)
            g_headScaleResets.fetch_add(1, std::memory_order_relaxed);
        SetHeadScale(head, 0.0f);
        track->collapsed = true;
        g_headCollapseFrames.fetch_add(1, std::memory_order_relaxed);
    } else if (track->collapsed) {
        SetHeadScale(head, 1.0f);
        track->collapsed = false;
    }

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
// forward, flattened), so A/D can't come out mirrored. Keyboard only for
// now; first person (F4) only, player character only, aim flag at
// controller+0x1B1 (the camera function's aim-group switch).
constexpr bool kMoveWhileAiming = true;
constexpr DWORD kOffControllerAimFlag = 0x1B1;
constexpr DWORD kOffCharacterPos = 0x30;
constexpr float kAimWalkSpeed = 100.0f; // render units per second - first guess, tune by feel

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
    const float forward = held('W') - held('S');
    const float strafe = held('D') - held('A');
    if (forward == 0.0f && strafe == 0.0f)
        return;

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
    pos[0] += dx * kAimWalkSpeed * dt;
    pos[2] += dz * kAimWalkSpeed * dt;
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
    if (!g_enabled)
        return;
    AimWalk(controller);
    const float fov = vrActive && g_vrWideFov.load(std::memory_order_relaxed) ? kVrCullVerticalFovDeg
                                                                              : FirstPersonVerticalFov();

    // Every direction is read from the game's untouched rigs before any of
    // them is rewritten - the normal group may borrow the aim group's.
    float aimDy[3], aimDz[3];
    bool aimOk[3];
    for (int i = 0; i < 3; ++i)
        aimOk[i] = RigViewDirection(controller + kOffAimRigs + i * kRigStride, &aimDy[i], &aimDz[i]);

    for (int i = 0; i < 3; ++i) {
        unsigned char* normalRig = controller + kOffNormalRigs + i * kRigStride;
        float dy = 0.0f, dz = 1.0f;
        if (kNormalUsesAimPitchRange && aimOk[i]) {
            dy = aimDy[i];
            dz = aimDz[i];
        } else if (!RigViewDirection(normalRig, &dy, &dz)) {
            dy = 0.0f;
            dz = 1.0f;
        }
        PlaceRigAtEye(normalRig, eyeNormal, dy, dz, fov);
        PlaceRigAtEye(controller + kOffAimRigs + i * kRigStride, eyeAim,
            aimOk[i] ? aimDy[i] : dy, aimOk[i] ? aimDz[i] : dz, fov);
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
    static bool prevF4Down = false;
    bool f4Down = (GetAsyncKeyState(VK_F4) & 0x8000) != 0;
    if (f4Down && !prevF4Down) {
        g_enabled = !g_enabled;
        Log_Printf("CameraRigHook: F4 pressed, first-person camera override now %s", g_enabled ? "ON" : "OFF");
        // First person puts the camera inside Chris, which is exactly what
        // triggers the game's near-camera fade - so the fade goes with it.
        FadePatch_SetEnabled(g_enabled);
        Log_Printf("CameraRigHook: head collapse so far - %lu frame(s), game reset the head scale %lu time(s)",
            g_headCollapseFrames.load(std::memory_order_relaxed), g_headScaleResets.load(std::memory_order_relaxed));
        g_eyeLogsRemaining.store(g_enabled ? 3 : 0, std::memory_order_relaxed);
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
    if (nowMs - s_lastVrCheckMs >= 100) {
        s_lastVrCheckMs = nowMs;
        XRBridgeEyeView left, right;
        g_vrActive.store(VRBridge_GetEyeViews(left, right), std::memory_order_relaxed);
    }

    // '`' = wide culling FOV in VR on/off (g_vrWideFov).
    static bool prevGraveDown = false;
    const bool graveDown = (GetAsyncKeyState(VK_OEM_3) & 0x8000) != 0;
    if (graveDown && !prevGraveDown) {
        const bool wide = !g_vrWideFov.load(std::memory_order_relaxed);
        g_vrWideFov.store(wide, std::memory_order_relaxed);
        Log_Printf("CameraRigHook: '`' pressed, VR culling FOV now %s", wide ? "WIDE (150 deg vertical)" : "normal (90 deg horizontal)");
    }
    prevGraveDown = graveDown;

    // Live VR tuning: ',' / '.' = eye forward offset (g_vrEyeAheadScale).
    static bool prevCommaDown = false, prevPeriodDown = false;
    const bool commaDown = (GetAsyncKeyState(VK_OEM_COMMA) & 0x8000) != 0;
    const bool periodDown = (GetAsyncKeyState(VK_OEM_PERIOD) & 0x8000) != 0;
    const bool commaPressed = commaDown && !prevCommaDown;
    const bool periodPressed = periodDown && !prevPeriodDown;
    if (commaPressed || periodPressed) {
        g_vrEyeAheadScale += (periodPressed ? 1.0f : -1.0f) * kVrEyeAheadStep;
        if (g_vrEyeAheadScale < 0.0f)
            g_vrEyeAheadScale = 0.0f;
        if (g_vrEyeAheadScale > 1.5f)
            g_vrEyeAheadScale = 1.5f;
        Log_Printf("CameraRigHook: VR eye forward scale now %.1f (1.0 = the flat-screen position)", g_vrEyeAheadScale);
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
