#include "culling_patch.h"
#include "aim_finder.h"
#include "camera_rig_hook.h"
#include "../render/stereo_test.h"
#include "../util/log.h"
#include "../vr/xr_input.h"

#include <MinHook.h>
#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

// ---- Why (2026-09-11) --------------------------------------------------
// VR culling was "god awful": the game only draws what its own camera can
// see, but the headset shows a wider view and the head turns independently
// of the game camera. Widening the game FOV was tried and removed - even
// 165 deg vertical is only +-85 deg horizontal at 16:9, so anything over
// the shoulder stays outside any frustum around the game's forward.
//
// Found by walking the camera data flow with F5 watchpoints (controller
// output -> main camera -> the renderer's readers) and the code dump:
//
//   re5dx9.exe+435A90  bool Camera::IsSphereVisible(const float sphere[4])
//     thiscall on a camera, sphere = (x, y, z, radius), ret 4. Tests the
//     sphere against six planes stored in the camera at +0xE0, +0xF0,
//     +0x100, +0x110, +0x120, +0x130 (each nx, ny, nz, d): returns 0 as
//     soon as dot(n, centre) + d < -radius for any plane, else 1.
//
// The model draw-list builder calls it per object for each camera
// (+3E4845), and on a model's bounding sphere at model+0x2D0 (+3E4288).
// While VR is on this detour answers "visible" every time; flat screen is
// untouched. It also counts how often the game WOULD have culled, so the
// log shows whether this is the test that matters for what's missing.
//
// Open question at the time of writing: level geometry may be culled by a
// different routine (a box test?) - being checked separately.

namespace {

constexpr uintptr_t kOffSphereVisible = 0x435A90;

typedef bool(__fastcall* SphereVisible_t)(void* camera, void* edxUnused, const float* sphere);
SphereVisible_t g_origSphereVisible = nullptr;

volatile LONG g_calls = 0;
volatile LONG g_wouldCull = 0;
unsigned long long g_lastReportMs = 0;
// A camera the game really uses, caught from the sphere test, so a watchpoint
// can be put on its frustum planes.
std::atomic<void*> g_lastCamera{ nullptr };

// The frustum planes this test reads, pushed outward (2026-09-17).
//
// Answering "visible" here covers everything that asks THIS function, and in
// normal play that is enough because the camera rig hook is also widening the
// game's own culling FOV every frame. During a cutscene it is not: the game
// drives its own camera, our hook never runs, the FOV stays narrow, and
// whatever else culls against these planes starts removing scenery at the
// edges of a headset view that is far wider than the game thinks it is.
//
// The planes are stored in the camera at +0xE0 through +0x130, each as a
// normal and a distance. Adding to the distance moves a plane outward without
// touching its direction, so the shape of the frustum is kept and only its
// reach changes. Cutscenes only: in normal play this would hand the renderer
// far more to draw for nothing, and the frame budget is not free.
constexpr uintptr_t kOffFrustumPlanes[6] = { 0xE0, 0xF0, 0x100, 0x110, 0x120, 0x130 };
constexpr float kPlanePushOut = 4000.0f;

// MOVE IT TO THE EYE, THEN WIDEN IT (2026-09-26, user: "so do proper culling
// then", after "the culling not rendering 360 degrees basically").
//
// Everything before this only ever pushed the planes OUTWARD, which makes the
// game camera's frustum bigger while leaving it exactly where the game camera
// is. That is an approximation, and a bad one whenever the eye and the camera
// disagree: to cover a metre of disagreement it has to grow by a metre in
// every direction, which is why the working value was four thousand units and
// why it still left gaps at the edges.
//
// A plane is n.x + d = 0, so translating the whole frustum by delta gives
// n.(x - delta) + d = 0, which is d' = d - n.delta. One dot product per plane,
// no change of shape, and the frustum ends up centred on the eye. That is what
// culling from where you are actually looking means, rather than culling from
// where the game thinks you are and over-compensating.
//
// The widening stays on top of it, because the headset sees a wider angle than
// the game's own FOV and translation cannot fix an angle - but it no longer
// has to cover the distance as well, which is most of what it was for.
// A SUPERSET, ALWAYS (2026-09-26, user: "culling is worse").
//
// Translating is not the safe edit that widening was. Widening can only ever
// ADD to what is drawn. A translation moves the far side in as it moves the
// near side out, and this hook fires for every frustum the game builds, not
// only the one the eye looks through - so anything built for a shadow pass or
// a second camera got shoved sideways by the eye offset and lost geometry it
// needed. Worse in exactly the way the user described.
//
// For a unit normal, n.delta can never exceed |delta|. So translating by delta
// and widening by |delta| gives
//
//     d' = d - n.delta + |delta|  >=  d
//
// on every plane: the new frustum CONTAINS the old one, whatever the hook was
// called for and however far the eye has wandered. Eye-centred where that
// helps, and never less than the game asked for anywhere else.
void MoveAndWidenFrustum(void* camera, const float delta[3], float by)
{
    auto* bytes = static_cast<unsigned char*>(camera);
    for (uintptr_t off : kOffFrustumPlanes) {
        float* plane = reinterpret_cast<float*>(bytes + off);
        const float nx = plane[0], ny = plane[1], nz = plane[2];
        // Only touch something that really looks like a unit normal: this runs
        // on whatever pointer the game passed in.
        const float len2 = nx * nx + ny * ny + nz * nz;
        if (len2 < 0.9f || len2 > 1.1f)
            return;
    }
    // And never carry a NaN into the planes. One of these reaching a plane
    // distance makes every comparison against it false, which culls the entire
    // world and turns the screen black - which is precisely what happened.
    float move[3] = {};
    float reach = 0.0f;
    if (delta) {
        for (int k = 0; k < 3; ++k) {
            if (!(delta[k] > -1e6f && delta[k] < 1e6f))
                return;
            move[k] = delta[k];
        }
        reach = std::sqrt(move[0] * move[0] + move[1] * move[1] + move[2] * move[2]);
    }
    if (!(by >= 0.0f && by < 1e6f))
        return;
    for (uintptr_t off : kOffFrustumPlanes) {
        float* plane = reinterpret_cast<float*>(bytes + off);
        plane[3] -= plane[0] * move[0] + plane[1] * move[1] + plane[2] * move[2];
        plane[3] += by + reach;
    }
}

void WidenFrustum(void* camera, float by)
{
    MoveAndWidenFrustum(camera, nullptr, by);
}

// ---- Where the camera keeps its own position (2026-09-26) ------------------
//
// The stipple during a stomp is not culling and not the model fade. Both were
// ruled out with the log. It is that we replace the view matrix at
// GetViewMatrix and nothing else, so the camera OBJECT never moves - and the
// renderer sorts its transparencies, measures its fade distances and picks its
// detail levels from that object. With the body lock off there is no
// disagreement and no stipple, which is exactly what the user observed; with
// it on, the disagreement is as large as the camera excursion, which is why a
// stomp is the worst case.
//
// To move the object we have to know where in it the position lives, and that
// is findable rather than guessable. We already decode the rendered camera
// position from the shader constants every frame. Whatever field tracks that
// number IS the position. So: scan the object for float triples that match it,
// then keep only the ones that STILL match after the camera has moved a long
// way, several times over. Coincidences do not survive that.
namespace camerafind {

constexpr int kScanBytes = 0x4000; // the fade reads +30D8h, so it is at least this big
constexpr int kMaxCandidates = 96;
// NOT THAT PRECISELY (2026-09-26: "first pass over the camera object - 0
// place(s) hold its position", every round). Zero on the FIRST pass means the
// test was wrong, not that the field is absent - a first pass with a loose
// test should match plenty and then whittle down. The position is decoded out
// of shader constants that have been through a projection and back, so it is
// good to a fraction of a percent rather than to a unit, and at a thousand
// units from the origin that is already several units of error.
constexpr float kTolerance = 3.0f;    // units, plus the relative term below
constexpr float kRelative = 0.004f;   // and four parts in a thousand
constexpr float kMustMove = 40.0f;    // before a round counts, half a metre
constexpr int kRoundsToTrust = 5;

int g_cand[kMaxCandidates];
int g_candCount = 0;
int g_rounds = 0;
bool g_started = false;
bool g_settled = false;
float g_wasAt[3] = {};
unsigned long long g_lastRound = 0;

bool Near(float a, float b)
{
    if (!(b == b) || !(a == a))
        return false;
    float slack = kTolerance + std::fabs(b) * kRelative;
    const float d = a - b;
    return d > -slack && d < slack;
}

std::atomic<void*> g_mainCamera{ nullptr };

// THE WRONG OBJECT (2026-09-26). Zero matches on the first pass, every pass,
// with a perfectly sane camera position to look for - which says the object
// being searched does not hold one. It would not: the sphere test is handed
// the thing that owns the frustum PLANES, and a frustum does not need to know
// where its apex is in world terms.
//
// The object that does is the one the renderer asks for the player view of.
// The rig hook already identifies it, since the player controller sits at a
// known offset inside it, so it hands it over rather than being guessed at.
void Round(void* camera)
{
    if (void* main = g_mainCamera.load(std::memory_order_relaxed))
        camera = main;
    if (!camera || g_settled)
        return;
    float now[3];
    if (!CameraRigHook_LastCameraPosition(now))
        return;
    const unsigned long long ms = GetTickCount64();
    if (ms - g_lastRound < 200)
        return;
    const float moved = std::sqrt((now[0] - g_wasAt[0]) * (now[0] - g_wasAt[0])
        + (now[1] - g_wasAt[1]) * (now[1] - g_wasAt[1]) + (now[2] - g_wasAt[2]) * (now[2] - g_wasAt[2]));
    if (g_started && moved < kMustMove)
        return; // standing still proves nothing: everything still matches
    g_lastRound = ms;
    std::memcpy(g_wasAt, now, sizeof(g_wasAt));

    auto* bytes = static_cast<unsigned char*>(camera);
    __try {
        if (!g_started) {
            g_candCount = 0;
            for (int off = 0; off + 12 <= kScanBytes && g_candCount < kMaxCandidates; off += 4) {
                const float* f = reinterpret_cast<const float*>(bytes + off);
                if (Near(f[0], now[0]) && Near(f[1], now[1]) && Near(f[2], now[2]))
                    g_cand[g_candCount++] = off;
            }
            g_started = true;
            g_rounds = 1;
            Log_Printf("CameraFind: camera at %.1f %.1f %.1f - first pass over the object holds it in %d place(s)",
                now[0], now[1], now[2], g_candCount);
            return;
        }
        int kept = 0;
        for (int i = 0; i < g_candCount; ++i) {
            const float* f = reinterpret_cast<const float*>(bytes + g_cand[i]);
            if (Near(f[0], now[0]) && Near(f[1], now[1]) && Near(f[2], now[2]))
                g_cand[kept++] = g_cand[i];
        }
        g_candCount = kept;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_started = false;
        g_candCount = 0;
        return;
    }
    ++g_rounds;
    if (g_candCount == 0) {
        Log_Printf("CameraFind: nothing survived round %d - starting again", g_rounds);
        g_started = false;
        return;
    }
    if (g_rounds < kRoundsToTrust) {
        Log_Printf("CameraFind: round %d, %d place(s) still tracking", g_rounds, g_candCount);
        return;
    }
    g_settled = true;
    char list[256];
    int at = 0;
    for (int i = 0; i < g_candCount && at < static_cast<int>(sizeof(list)) - 12; ++i)
        at += sprintf_s(list + at, sizeof(list) - at, "%s+%X", i ? ", " : "", g_cand[i]);
    Log_Printf("CameraFind: settled after %d rounds - the camera keeps its position at %s", g_rounds, list);
}

// And once it is known, put the eye there while the eye is somewhere else.
// Written every frame, and the game recomputes the field from its own camera
// state every frame, so nothing accumulates and there is nothing to restore.
void PutTheCameraOnTheEye(void* camera, const float delta[3])
{
    if (!g_settled || !camera || !g_candCount)
        return;
    float at[3];
    if (!CameraRigHook_LastCameraPosition(at))
        return;
    // ONLY WHILE THEY DISAGREE (2026-09-26, user: "but does that keep the
    // camera with the body if I sprint up stairs?").
    //
    // It does not, and it is not meant to. That is the departure threshold,
    // which moved from 60 to 90 units in the same build because the log has
    // stair sprints at 66 to 75 and melee at 102 and above. This is a
    // different job: making the renderer agree with the view it is handed.
    //
    // So there is no reason to write anything while the two already agree, and
    // a good reason not to - one of these four fields may be an input to the
    // game own camera smoothing rather than purely an output, and the less it
    // is touched the less there is to find out the hard way. Ten units is well
    // inside the drift between the two placements during ordinary play.
    float eye[3];
    float reach = 0.0f;
    for (int k = 0; k < 3; ++k) {
        if (!(delta[k] > -1e6f && delta[k] < 1e6f))
            return;
        eye[k] = at[k] + delta[k];
        reach += delta[k] * delta[k];
    }
    if (reach < 10.0f * 10.0f)
        return;
    auto* bytes = static_cast<unsigned char*>(camera);
    __try {
        for (int i = 0; i < g_candCount; ++i)
            std::memcpy(bytes + g_cand[i], eye, sizeof(eye));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

} // namespace camerafind

bool __fastcall hkSphereVisible(void* camera, void* edxUnused, const float* sphere)
{
    const bool visible = g_origSphereVisible(camera, edxUnused, sphere);
    if (!CameraRigHook_IsVrActive())
        return visible;
    // This test runs well under once a frame - 147 to 266 calls per FIVE
    // seconds, measured 2026-09-17 - so it is not what removes the scenery, and
    // widening the planes from in here barely ran at all. What it is good for
    // is handing us a live camera: the planes it tests against are the ones
    // every other culling routine will be testing against too, so this is where
    // the hunt for their writer starts.
    g_lastCamera.store(camera, std::memory_order_relaxed);
    if (CameraRigHook_InScriptedCamera() && camera) {
        __try {
            WidenFrustum(camera, kPlanePushOut);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
        }
    }
    InterlockedIncrement(&g_calls);
    if (!visible)
        InterlockedIncrement(&g_wouldCull);
    return true;
}

// ---- Widening the planes where they are built (2026-09-17) --------------
// The F7 watchpoint caught exe+437793 writing the planes, inside a routine that
// builds all six from the camera's corners:
//
//   00837773: call  006E5BC0            ; plane from points
//   0083778B: movss [esi+000000E0h],xmm0 ; ...and out it goes, esi = the camera
//   ...       five more, the same shape
//   00837C08: movss [esi+0000013Ch],xmm3 ; the last one
//   00837C10: pop   esi                  ; <- hooked here: all six are written
//   00837C14: ret   4                    ;    and esi still holds the camera
//
// This runs every frame for every camera, which is what the sphere test does
// not, so widening here reaches every culling test in the game rather than the
// one rare consumer. Scripted cameras only: in normal play the camera rig hook
// already widens the game's own culling FOV, and doing both would hand the
// renderer far more to draw for nothing.
void* g_frustumTrampoline = nullptr;
bool g_frustumHooked = false;

volatile LONG g_widened = 0;
volatile LONG g_frustumBuilds = 0;

void FrustumBuilt(void* camera)
{
    if (!camera || !CameraRigHook_IsVrActive())
        return;
    InterlockedIncrement(&g_frustumBuilds);
    // "Always" is the diagnostic that settles it (2026-09-17). Gating on our
    // guess at what a cutscene is means a failure to widen and a wrong guess
    // look exactly alike from the outside, and the log only said it had
    // widened ONCE PER SESSION, so there was no way to tell whether it ran
    // during the cutscene at all.
    // Pushed out by however far the eye has actually been moved (2026-09-23,
    // user: "there is also a slight graphics flicker" ... "it's the level
    // geometry that flickers"). The game builds this frustum around the camera
    // IT thinks it is drawing from, and the eye is now somewhere else in three
    // different ways: leaning, roomscale, and the hold that keeps you in your
    // body during an action. Anything past the edge of the game's frustum has
    // been thrown away before we ever see it, and at the edge of vision that
    // reads as scenery blinking.
    //
    // So the amount is measured rather than assumed. Standing still and not
    // leaning it is nothing and this costs nothing; lean a foot out from a
    // wall and the frustum reaches a foot further, plus a margin. Far cheaper
    // than the flat four thousand units the cutscene path uses, which is why
    // that one was only ever allowed to run during cutscenes.
    // AN ACTION CAMERA IS NOT A SCRIPTED ONE (2026-09-25, user: "I think the
    // solution is to fix culling during a cutscene camera for the in game
    // stuff", and the checkerboarding during actions before it).
    //
    // The full four thousand units of push-out was reserved for
    // InScriptedCamera, which means the rig has gone QUIET. A kick, a stomp, a
    // vault and a chest reveal never make it go quiet - the rig keeps being
    // called throughout - so all of those got `moved + 25` instead, twenty-five
    // units of margin against a camera that has just walked several metres away
    // from where the eye is being held. Everything outside the game camera's own
    // frustum is thrown away before it is drawn, and what is left has holes in
    // it.
    //
    // Worse, `moved` is measured while the eyes are built and read here on the
    // next frame's draws, so during the fastest part of a camera pulling away it
    // is always behind.
    //
    // So the eye being a long way from the camera is enough on its own, whatever
    // the rig is doing. Thirty units to start, ten to stop, so it cannot chatter
    // on the boundary, and the stale reading has four thousand units of slack to
    // be wrong inside instead of twenty-five.
    // The frustum goes WHERE THE EYE IS, every frame, however small the
    // difference - that part is exact and costs a dot product. What is left
    // for the widening is only the angle the headset sees beyond the game's
    // own FOV, which does not grow with how far the camera has wandered.
    float delta[3] = {};
    StereoTest_EyeOffset(delta);
    const float moved = StereoTest_EyeOffsetUnits();
    const bool always = XrInput_GetSettings().wideCullAlways || CameraRigHook_InScriptedCamera();
    // A margin for the angle, and for this reading being a frame old. It no
    // longer has to stand in for the distance, so it does not need thousands.
    const float by = always ? kPlanePushOut : 150.0f;
    __try {
        MoveAndWidenFrustum(camera, delta, by);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    camerafind::Round(camera);
    if (XrInput_GetSettings().cameraFollowsEye)
        camerafind::PutTheCameraOnTheEye(camera, delta);
    (void)moved;
    InterlockedIncrement(&g_widened);
}

__declspec(naked) void FrustumBuiltHook_Stub()
{
    __asm {
        pushad
        pushfd
        push esi
        call FrustumBuilt
        add esp, 4
        popfd
        popad
        jmp g_frustumTrampoline
    }
}

} // namespace

void CullingPatch_NoteMainCamera(void* camera)
{
    camerafind::g_mainCamera.store(camera, std::memory_order_relaxed);
}

void CullingPatch_Install()
{
    const uintptr_t base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    void* target = reinterpret_cast<void*>(base + kOffSphereVisible);

    // push ebp / mov ebp,esp / and esp,-16 / mov eax,[ebp+8]
    static const unsigned char kPrologue[] = { 0x55, 0x8B, 0xEC, 0x83, 0xE4, 0xF0, 0x8B, 0x45, 0x08 };
    if (std::memcmp(target, kPrologue, sizeof(kPrologue)) != 0) {
        Log_Printf("CullingPatch_Install: sphere-test prologue doesn't match - not hooking (different game build?)");
        return;
    }
    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&hkSphereVisible),
        reinterpret_cast<void**>(&g_origSphereVisible));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("CullingPatch_Install: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("CullingPatch_Install: frustum sphere-test hook enabled -> %d (target=%p), active while VR is on",
        static_cast<int>(st), target);

    // pop esi / mov esp,ebp / pop ebp / ret 4, immediately after the sixth
    // plane is stored. Checked before hooking, because five of these bytes get
    // replaced and being wrong about them is a crash rather than a bug.
    void* frustumEnd = reinterpret_cast<void*>(base + 0x437C10);
    static const unsigned char kTail[] = { 0x5E, 0x8B, 0xE5, 0x5D, 0xC2, 0x04, 0x00 };
    if (std::memcmp(frustumEnd, kTail, sizeof(kTail)) != 0) {
        Log_Printf("CullingPatch_Install: the frustum builder's tail doesn't match - not hooking it");
        return;
    }
    MH_STATUS fst = MH_CreateHook(frustumEnd, reinterpret_cast<void*>(&FrustumBuiltHook_Stub), &g_frustumTrampoline);
    if (fst == MH_OK || fst == MH_ERROR_ALREADY_CREATED)
        fst = MH_EnableHook(frustumEnd);
    g_frustumHooked = fst == MH_OK;
    Log_Printf("CullingPatch_Install: frustum builder hook at exe+437C10 -> %s (%d)",
        g_frustumHooked ? "OK" : "FAILED", static_cast<int>(fst));
}

void CullingPatch_FindPlaneWriter()
{
    // Re-armable, whether or not the last window caught anything (2026-09-17).
    // It used to rearm only when there was a hit to report, so the run that
    // found NOTHING - the interesting one - left it stuck: the next two F7
    // presses during the same cutscene did nothing at all. Rearm is safe while
    // a window is still open; it refuses on its own.
    AimFinder_Rearm();
    static bool s_armed = false;
    s_armed = false;
    void* camera = g_lastCamera.load(std::memory_order_relaxed);
    if (!camera) {
        static unsigned long long s_saidMs = 0;
        const unsigned long long now = GetTickCount64();
        if (now - s_saidMs > 3000) {
            s_saidMs = now;
            Log_Printf("CullingPatch: no camera seen yet - the sphere test has to run once before its planes can be "
                       "watched");
        }
        return;
    }
    s_armed = true;
    // The first plane's normal. Whatever writes it builds all six.
    AimFinder_Start(static_cast<unsigned char*>(camera) + kOffFrustumPlanes[0], "the camera's frustum planes (+0xE0)");
}

// WHO MOVES THE CAMERA IN A MELEE (2026-09-26).
//
// In ordinary play nothing does, because ApplyOverride zeroes the rig's
// distances every frame and the camera collapses onto the character. A stomp
// does not go through those rigs, so nothing gets zeroed and the camera leaves
// - and every symptom we have been chasing, the checkerboard and the stairs
// jump and the disorientation, is us compensating for that from outside.
//
// We now know where the camera keeps its position, so the question "what moves
// it" is answerable rather than guessable: watch the field and let the
// hardware name the writer. Whatever that turns out to be is the melee's
// equivalent of ApplyOverride, and gets the same treatment.
void CullingPatch_FindCameraWriter()
{
    AimFinder_Rearm();
    void* camera = camerafind::g_mainCamera.load(std::memory_order_relaxed);
    if (!camera || !camerafind::g_settled || camerafind::g_candCount <= 0) {
        Log_Printf("CullingPatch: the camera's position has not been found yet - walk around for a few seconds "
                   "first, the log says when it settles");
        return;
    }
    // Round robin, so every candidate can be tried without a rebuild. They all
    // track the position; only some of them are likely to be what the renderer
    // reads, and only one of them is likely to be what a melee writes.
    // AND NOW WE KNOW WHICH ONE (2026-09-26, from the +640 window).
    //
    // "fld [esi+3B0] / fstp [esi+640]" - so +3B0 feeds +640, which feeds +30,
    // which feeds +1F0. One value copied down a pipeline, not four places the
    // camera keeps its position, which is also why writing into the middle of
    // it skewed the eyes.
    //
    // So there is no reason to cycle any more. Every press watches the source,
    // and presses are not wasted on copies.
    //
    // THE CHAIN, NOT THE COPIES (2026-09-26, from the first two windows).
    //
    // +30 is written from +640, and +1F0 is written from +30 - the watchpoint
    // caught both moves verbatim: "fld [esi+640] / lea eax,[esi+30] / fstp
    // [eax]", then "fld [esi+30] / fstp [esi+1F0]". So three of the four
    // places are copies and only the highest is upstream of the rest.
    //
    // Which also explains the eyes going wrong when the write was switched on:
    // it was writing into the middle of that chain, so the camera was being
    // fed its own position back before the per-eye matrices were built.
    //
    // Highest offset first, so the source is the first thing looked at.
    constexpr int kTheSource = 0x3B0;
    int off = -1;
    for (int i = 0; i < camerafind::g_candCount; ++i)
        if (camerafind::g_cand[i] == kTheSource)
            off = kTheSource;
    if (off < 0) {
        // A different build or a different camera: fall back to the highest,
        // which is the most likely to be upstream of the others.
        off = camerafind::g_cand[camerafind::g_candCount - 1];
    }
    static char what[96];
    sprintf_s(what, "the main camera's position (+%X)", off);
    AimFinder_Start(static_cast<unsigned char*>(camera) + off, what);
}

// WHERE THE DISPLACEMENT ENTERS (2026-09-26, from two matched windows).
//
// Standing still and vaulting produce exactly the same three writers of +3B0,
// in the same order, differing only in how often they ran. So the vault does
// not run different camera code - it runs the same code with a different
// answer, and watchpoints cannot tell us any more than that.
//
// What can is reading the pipeline itself. +3B0 is the source, it is copied to
// +640, something adds an offset to it there, and it reaches +30 and +1F0 from
// that. Print all four beside the player position and the stage where the
// camera stops agreeing with the body names itself.
//
// Only while something is actually happening, so ordinary play does not fill
// the log: the camera has to be at least half a metre from the player.
void WatchThePipeline()
{
    void* camera = camerafind::g_mainCamera.load(std::memory_order_relaxed);
    if (!camera || !camerafind::g_settled)
        return;
    float man[3];
    if (!CameraRigHook_GetPlayerWorldPos(man))
        return;

    auto* bytes = static_cast<unsigned char*>(camera);
    float at[4][3];
    __try {
        for (int i = 0; i < 4 && i < camerafind::g_candCount; ++i)
            std::memcpy(at[i], bytes + camerafind::g_cand[i], sizeof(at[i]));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (camerafind::g_candCount < 4)
        return;

    // How far each stage has drifted from the player. The first stage that is
    // a long way out is the one doing it.
    // SIDEWAYS AND UP, NOT JUST FAR (2026-09-26, user: "oh but it did leave my
    // body when i jumped off ledges and stuff").
    //
    // And a plain distance could never have shown that. A third person camera
    // ORBITS: it swings from above your head to behind your shoulder at a more
    // or less fixed radius, so the one number I was printing stays near 170
    // throughout and reads as "it never moved". It moved the whole time.
    //
    // Horizontal distance is the one that tells the truth. In your head it is
    // near zero and the height is all of it; behind your shoulder the
    // horizontal part is most of it.
    float away[4], flat[4], up[4];
    for (int i = 0; i < 4; ++i) {
        const float dx = at[i][0] - man[0], dy = at[i][1] - man[1], dz = at[i][2] - man[2];
        away[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
        flat[i] = std::sqrt(dx * dx + dz * dz);
        up[i] = dy;
    }
    float worst = 0.0f;
    for (int i = 0; i < 4; ++i)
        if (away[i] > worst)
            worst = away[i];
    float worstFlat = 0.0f;
    for (int i = 0; i < 4; ++i)
        if (flat[i] > worstFlat)
            worstFlat = flat[i];
    // Only when the camera is genuinely out to the SIDE of him. Standing in
    // his head is a large height and almost no horizontal offset, and that is
    // not worth a line.
    if (worstFlat < 45.0f)
        return;

    static unsigned long long s_saidAt = 0;
    const unsigned long long nowMs = GetTickCount64();
    if (nowMs - s_saidAt < 100)
        return;
    s_saidAt = nowMs;
    Log_Printf("CamPipe: +%X is %.0f out sideways and %.0f up, +%X is %.0f/%.0f, +%X is %.0f/%.0f, +%X is "
               "%.0f/%.0f",
        camerafind::g_cand[0], flat[0], up[0], camerafind::g_cand[1], flat[1], up[1], camerafind::g_cand[2],
        flat[2], up[2], camerafind::g_cand[3], flat[3], up[3]);
    (void)worst;
    (void)away;
}

void CullingPatch_OnEndScene()
{
    WatchThePipeline();
    // F7 arms it by hand (2026-09-17, user: "How bout I press something on the
    // keyboard when a cutscene is about to happen?"). Far better than arming
    // itself: the window is four seconds and fires once, so it has to land on
    // the moment scenery is actually vanishing, and only the person watching
    // knows when that is about to be. The checkbox stays for arming as soon as
    // a camera appears.
    {
        static bool s_wasDown = false;
        const bool down = (GetAsyncKeyState(VK_F7) & 0x8000) != 0;
        if (down && !s_wasDown) {
            Log_Printf("CullingPatch: F7 - arming the frustum plane watchpoint now");
            CullingPatch_FindPlaneWriter();
        }
        s_wasDown = down;
    }
    {
        static bool s_wasF8 = false;
        const bool down = (GetAsyncKeyState(VK_F8) & 0x8000) != 0;
        if (down && !s_wasF8) {
            Log_Printf("CullingPatch: F8 - watching the camera's own position for the next 4 seconds. Trigger the "
                       "vault or the melee NOW.");
            CullingPatch_FindCameraWriter();
        }
        s_wasF8 = down;
    }
    if (XrInput_GetSettings().findCullPlanes)
        CullingPatch_FindPlaneWriter();
    const unsigned long long now = GetTickCount64();
    if (now - g_lastReportMs < 5000)
        return;
    g_lastReportMs = now;
    const LONG calls = InterlockedExchange(&g_calls, 0);
    const LONG culls = InterlockedExchange(&g_wouldCull, 0);
    const LONG builds = InterlockedExchange(&g_frustumBuilds, 0);
    const LONG widened = InterlockedExchange(&g_widened, 0);
    if (calls || builds)
        Log_Printf("CullingPatch: last 5 s - %ld sphere test(s), %ld would have been culled, all drawn; the frustum "
                   "was built %ld time(s) and widened %ld of them",
            calls, culls, builds, widened);
}
