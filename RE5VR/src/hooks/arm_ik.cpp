#include "arm_ik.h"

#include "aim_finder.h"
#include "ammo.h"
#include "../render/bone_palette.h"
#include "camera_rig_hook.h"
#include "pose_guard.h"
#include "../util/build_config.h"
#include "../util/log.h"
#include "../vr/openxr_bridge.h"
#include "../net/ik_sync.h"
#include "../vr/xr_input.h"
#include "../render/stereo_test.h"

#include <windows.h>
#include <MinHook.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace {

constexpr int kJointStride = 0x90;
constexpr int kMaxJoints = 200;
constexpr int kOffJointWorldMatrix = 0x50; // 4 rows of 4 floats, translation in row 3
constexpr int kOffJointWorldPos = 0x80;
constexpr DWORD kOffBodyTransform = 0x60; // row 0 lies across the body, pointing LEFT
constexpr int kOffJointLinks = 0x04;      // byte 1 = parent index
constexpr int kOffJointScale = 0x30;
constexpr int kOffJointBindOffset = 0x10; // the child's rest offset, parent space, length in w

// Shoulder to wrist on a person, metres. Everything else about scale follows
// from this one number and the character's own arm, so it is the dial to argue
// with if the world turns out to feel the wrong size.
constexpr float kArmMetres = 0.62f;

// Where the eyes sit above and ahead of the head joint. Must match
// camera_rig_hook.cpp: this is the point the headset stands in for, so the
// offset to a hand is measured from here.
constexpr float kEyeAbovePivot = 11.0f;
constexpr float kEyeAheadOfPivot = 15.0f;

bool TryRead(void* dst, const void* src, size_t n)
{
    __try {
        std::memcpy(dst, src, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

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

void Sub3(const float* a, const float* b, float* out)
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

float Dot3(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross3(const float* a, const float* b, float* out)
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

bool Normalise3(float* v)
{
    const float len = Length3(v);
    if (!(len > 1e-6f))
        return false;
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return true;
}

// 3x3 rotation, row-major, applied as out = M * v with v a column.
struct Rot3 {
    float m[9];
};

Rot3 RotIdentity()
{
    Rot3 r{};
    r.m[0] = r.m[4] = r.m[8] = 1.0f;
    return r;
}

void RotApply(const Rot3& r, const float* v, float* out)
{
    const float x = v[0], y = v[1], z = v[2];
    out[0] = r.m[0] * x + r.m[1] * y + r.m[2] * z;
    out[1] = r.m[3] * x + r.m[4] * y + r.m[5] * z;
    out[2] = r.m[6] * x + r.m[7] * y + r.m[8] * z;
}

// The shortest rotation taking unit vector a onto unit vector b. Rodrigues,
// with the two degenerate cases spelled out: already there, and exactly
// opposite, where the axis is undefined and any perpendicular will do.
Rot3 RotBetween(const float* a, const float* b)
{
    float axis[3];
    Cross3(a, b, axis);
    const float s = Length3(axis);
    const float c = Dot3(a, b);
    if (s < 1e-6f) {
        if (c > 0.0f)
            return RotIdentity();
        float perp[3] = { 1.0f, 0.0f, 0.0f };
        if (std::fabs(a[0]) > 0.9f) {
            perp[0] = 0.0f;
            perp[1] = 1.0f;
        }
        float ax[3];
        Cross3(a, perp, ax);
        if (!Normalise3(ax))
            return RotIdentity();
        // A half turn about that axis.
        Rot3 r{};
        const float x = ax[0], y = ax[1], z = ax[2];
        r.m[0] = 2.0f * x * x - 1.0f;
        r.m[1] = 2.0f * x * y;
        r.m[2] = 2.0f * x * z;
        r.m[3] = 2.0f * x * y;
        r.m[4] = 2.0f * y * y - 1.0f;
        r.m[5] = 2.0f * y * z;
        r.m[6] = 2.0f * x * z;
        r.m[7] = 2.0f * y * z;
        r.m[8] = 2.0f * z * z - 1.0f;
        return r;
    }
    const float x = axis[0] / s, y = axis[1] / s, z = axis[2] / s;
    const float sn = s, cs = c, t = 1.0f - c;
    Rot3 r{};
    r.m[0] = t * x * x + cs;
    r.m[1] = t * x * y - sn * z;
    r.m[2] = t * x * z + sn * y;
    r.m[3] = t * x * y + sn * z;
    r.m[4] = t * y * y + cs;
    r.m[5] = t * y * z - sn * x;
    r.m[6] = t * x * z - sn * y;
    r.m[7] = t * y * z + sn * x;
    r.m[8] = t * z * z + cs;
    return r;
}

bool JointWorldPos(const unsigned char* joints, int index, float out[3])
{
    if (index < 0)
        return false;
    return TryRead(out, joints + index * kJointStride + kOffJointWorldPos, sizeof(float) * 3);
}

// Every joint hanging off root, root itself included. Written into out, which
// must hold kMaxJoints. Returns how many.
int Subtree(const unsigned char* parents, int count, int root, int* out)
{
    bool in[kMaxJoints] = {};
    if (root < 0 || root >= count)
        return 0;
    in[root] = true;
    int found = 0;
    out[found++] = root;
    // The array is not guaranteed parents-first, so sweep until nothing new
    // turns up rather than assuming an order.
    for (int pass = 0; pass < 8; ++pass) {
        bool added = false;
        for (int i = 0; i < count; ++i) {
            if (in[i] || parents[i] >= count || parents[i] == i || !in[parents[i]])
                continue;
            in[i] = true;
            if (found < kMaxJoints)
                out[found++] = i;
            added = true;
        }
        if (!added)
            break;
    }
    return found;
}

// Turn a joint's world matrix about a pivot: the three basis rows rotate, the
// translation orbits. Scale rides along untouched, because it is baked into
// the rows' lengths and a rotation does not change those.
void RotateJointWorld(unsigned char* joints, int index, const Rot3& r, const float pivot[3])
{
    unsigned char* m = joints + index * kJointStride + kOffJointWorldMatrix;
    float rows[16];
    if (!TryRead(rows, m, sizeof(rows)))
        return;
    for (int i = 0; i < 3; ++i) {
        float out[3];
        RotApply(r, &rows[i * 4], out);
        std::memcpy(&rows[i * 4], out, sizeof(out));
    }
    const float rel[3] = { rows[12] - pivot[0], rows[13] - pivot[1], rows[14] - pivot[2] };
    float turned[3];
    RotApply(r, rel, turned);
    rows[12] = pivot[0] + turned[0];
    rows[13] = pivot[1] + turned[1];
    rows[14] = pivot[2] + turned[2];
    TryWrite(m, rows, sizeof(rows));
}

// The pose the solve last worked out, per joint, so it can be put back the
// instant the animation overwrites it (2026-09-17). Writing once at the top of
// the build was not enough: the animation sets these locals one or two times a
// frame AFTER that, at exe+189E75 and below, so ours only survived on the
// frames it happened to win. The arm would take up a pose and stick there.

// The controller's own axes in game space, and the fixed turn between it and
// the hand. Direction comes from your controller; the offset is what makes a
// gun held naturally in your hand point the way it does on screen, and it is
// retaken each time the gun comes up so a twisted wrist can always be reset.
// ---- Whose arm is whose (2026-09-19) ---------------------------------------
// All of the state below used to be indexed by HAND alone - two slots, left and
// right. Perfectly fine while there was one body to drive.
//
// Not fine for co-op. The solver remembers a great deal per hand: the
// controller's basis, the calibration ties, the socket lock that keeps the gun
// in the hand, the forearm twist history, the reach fraction. Running the same
// solve for a partner would write all of it into the same two slots the player
// is using, so Sheva's right arm and Chris's right arm would share one drawer.
// The skeletons being separate does not help at all - it is this memory, not
// the joints, that would collide, which is the whole reason the partner cannot
// simply be handed to the existing solver.
//
// So slots are per BODY and hand: 0 and 1 are the player's left and right, 2
// and 3 the partner's. Existing code passes `hand` and lands exactly where it
// always did, which is what makes this widening a no-op for the player.
constexpr int kArmBodies = 2;
constexpr int kArmSlots = kArmBodies * 2;
constexpr int kPlayerSlot = 0;
constexpr int kPartnerSlot = 2;

float g_ctrlWorld[kArmSlots][9] = {};
bool g_haveCtrl[kArmSlots] = {};
// The controller orientation captured at the T-pose. Twist shared up the arm is
// measured from this, so neutral really is zero and both directions are equal.
float g_ctrlRef[kArmSlots][9] = {};
bool g_haveCtrlRef[kArmSlots] = {};
// The partner's skeleton, for touch. Read only, and only ever this frame's.
unsigned char* g_otherJoints = nullptr;
int g_otherCount = 0;
unsigned long long g_otherMs = 0;
// The partner as a full body, for driving their arms. Separate from the
// joints-only note above, which only ever needed somewhere to bump into.
ArmIkBody g_partnerBody = {};
unsigned char* g_lastCharacter = nullptr;
// Why the partner's arms did not get driven this frame. Set at every point
// the partner path can give up, and reported once a second with a count of
// how many frames ran and how many did not - because "it flickers" is a
// ratio, and a ratio needs both numbers.
const char* g_partnerWhy = "not started";
// Where the partner's pose stops (2026-09-23). Their log says the solve "ran
// but wrote no pose" on 118 frames out of every 120, and none of the specific
// refusals fired on those frames - not the shoulder, not the scale, not the
// two-bone solve, not a joint that would not rebuild. So it gets past every
// test we have and then writes nothing, which means the stage that is failing
// has no report of its own. These count the last few steps so it does.
unsigned g_pwOffered = 0;  // joints the solve handed to the writer
unsigned g_pwRebuild = 0;  // refused because the pose would not rebuild
unsigned g_pwWritten = 0;  // actually written
unsigned g_partnerRan = 0, g_partnerSkipped = 0;
bool g_havePartnerBody = false;
unsigned long long g_partnerBodyMs = 0;
// Whether the gun is up, kept where the solver can see it, and when it went up.
bool g_aimingNow = false;
unsigned long long g_aimUpMs = 0;
// Either edge of the aim button, so the gun hold can decline to learn anything
// while the ready animation is playing.
unsigned long long g_aimEdgeMs = 0;

// The ties are measured against the character's aiming pose, so they are only
// worth taking once that pose EXISTS (2026-09-18). Raising the gun is an
// animated transition of a few hundred milliseconds; a tie taken on the frame
// the flag flips records the arm halfway there and carries that error for as
// long as it lives. Which is the ninety degrees the user keeps having to hold
// by hand - it comes back whenever the capture lands early, so it looks
// intermittent rather than wrong.
constexpr unsigned long long kAimSettleMs = 350;
bool AimSettled()
{
    return g_aimingNow; // rolled back: no settle delay
}
float g_wristOffset[kArmSlots][9] = {};
bool g_haveWristOffset[kArmSlots] = {};
// The turn between your controller and the FOREARM, taken once with the gun up.
// Tying the forearm rather than the hand is what gets 1:1 out of this skeleton:
// the weapon socket hangs below the elbow, so the forearm is what carries the
// gun, and a tie learned from the game's own aiming pose absorbs every constant
// offset at once - the ninety degrees of roll the user had to hold by hand, and
// whatever angle the grip makes with the bone.
float g_armOffset[kArmSlots][9] = {};
bool g_haveArmOffset[kArmSlots] = {};
// The weapon socket: a joint sitting exactly where the wrist is but hanging off
// the ELBOW rather than the hand (joint 51, id 75, on this skeleton). It is what
// a held item is actually attached to, which is why the gun travelled with the
// arm but ignored the wrist turning - the socket never saw that rotation. It
// gets the hand's rotation too.
int g_socket[kArmSlots] = { -1, -1, -1, -1 };
// The fixed turn between the hand and that socket, measured while the gun is
// DOWN - where the user confirms it sits in the hand properly - and then held
// while aiming. See the socket lock in SolveArm.
float g_socketTie[kArmSlots][9] = {};
// What the last solve asked of each arm, for the report: how far out your hand
// wanted it as a fraction of its full span, and what the collarbone took.
float g_lastReachFrac[kArmSlots] = {};
float g_lastClavShare[kArmSlots] = {};

// ---- T-pose calibration (2026-09-18) --------------------------------
// The user's idea, and it retires a whole family of bugs at once.
//
// Every tie so far has been measured against the ANIMATION - the arm as it
// happened to be at the instant the gun came up. That is a moving target, which
// is why they drift, need retaking, land ninety degrees out when the capture is
// unlucky, and occasionally contort the mesh outright.
//
// The bind pose does not move. The log confirmed this skeleton rests in a clean
// T-pose: shoulder to wrist runs (-1.00 0.00 0.00) over 53.2 units, straight out
// to the side, authored data that is the same in a cutscene or on the title
// screen. So the player stands in the pose the skeleton was built in, holding
// their controllers however is natural to them, and both halves are captured at
// once: we know what the bones should be doing, and we measure what the
// controllers are doing. Nothing is assumed about the grip - whatever way they
// are holding it at that moment BECOMES neutral.
//
// The same pose gives the scale. The character's own bind span - wrist to wrist
// through both arms and the shoulders - against the player's real one is units
// per metre, measured rather than assumed. That matters: the reach report has
// been reading 1.03 to 1.21 of full, so the arm was pinned at full stretch every
// frame and push and pull had to be exaggerated to register at all.
volatile LONG g_calibratePending = 0;
// When a costume or character last took the calibration away with it, so the
// menu can put a prompt on screen rather than leaving it in the log.
unsigned long long g_costumeChangedMs = 0;
bool g_tiesCalibrated[kArmSlots] = {};
float g_charRight[3] = { 1.0f, 0.0f, 0.0f };
float g_charUp[3] = { 0.0f, 1.0f, 0.0f };
float g_charForward[3] = { 0.0f, 0.0f, 1.0f };
float g_calUnitsPerMetre[kArmSlots] = {};
float g_calHand[kArmSlots][3] = {};
bool g_calHaveHand[kArmSlots] = {};
// Where each of YOUR shoulders sits relative to your headset, in metres, taken
// from the T-pose. The one measurement that makes shoulder-to-hand mean the
// same thing on both bodies.
float g_shoulderFromHead[kArmSlots][3] = {};
bool g_haveShoulderFromHead[kArmSlots] = {};
// Which way the hand bone runs, published by the solve so the target can be
// pulled back off the controller onto the actual wrist.
int g_foreAxis[kArmSlots] = { -1, -1, -1, -1 };
float g_foreSign[kArmSlots] = { 1.0f, 1.0f, 1.0f, 1.0f };
// A controller is held in the PALM, not at the wrist - its tracked origin sits
// most of a hand forward of the joint the arm actually ends at. Treating it as
// the wrist costs that distance twice over: the arm is asked to reach a hand's
// length further than it should, so it locks out early ("length still feels way
// too short at full extend"), and flexing your wrist swings the controller
// through an arc that the solver reads as the whole hand TRAVELLING, so it moves
// the arm instead of turning the wrist. That is the user's "flapping your hands
// - the real pivot point on a wrist joint".
// ROLLED BACK to 0 on 2026-09-19 (user: "completely revert to the 10:25 build,
// take out everything we've done"). Everything below that was added after that
// build is switched off here rather than unpicked line by line, so any one piece
// can be put back on its own once the gun is tracking again.
constexpr float kGripToWristMetres = 0.0f;

// The gap between the arm the T-pose measures and the arm that actually works,
// which is not what I expected and is worth writing down properly.
//
// The reasoning was: a controller sits about 0.10 m past the wrist, so
// shoulder-to-controller over-states the arm, units-per-metre comes out short,
// and subtracting it should make 1.00 right.
//
// Two calibrations say otherwise, and they agree with each other to better than
// one per cent:
//   measured 0.68 m -> 78.2 units/m, dialled 1.17 -> effective 66.8
//   measured 0.58 m -> 91.7 units/m, dialled 1.38 -> effective 66.4
// So the value that works is about 66.5 units per metre, which against this
// character's 53.2-unit arm means an effective arm of 0.80 m - LONGER than the
// controller measurement, not shorter. The subtraction had the sign backwards.
//
// 0.12 is fitted from those two runs, not derived, and it is only honest to say
// so. The likeliest cause is that the shoulder the T-pose infers is not where
// the shoulder really is, so the distance reachable in play is bigger than the
// pose implied. The peak-reach line in the target code measures exactly that
// and should settle it.
// ZERO from 2026-09-19, once the collarbone existed to do its job properly.
//
// This sat in the DENOMINATOR, so it never was the headroom it was meant to be -
// it shrank the whole mapping. With the user's 0.67 m arm taken as 0.79, their
// full extension only ever reached 0.67/0.79 = 0.85 of the character's, so the
// elbow carried a permanent bend at every distance. A screenshot of a straight
// arm reading 0.80 of full is what it looks like.
//
// The travel it stood in for - the measured 0.17 m a shoulder moves when you
// reach - now comes from the collarbone, which adds it at the top where it
// belongs instead of taking it off everywhere.
// Was a constant, and that is the problem (2026-09-19). It was fitted from ONE
// person's two calibrations, to account for the shoulder travel that lets you
// reach further in play than you can in a T-pose. Then testers reported arms
// "like trex arms", which is exactly what this does to anybody it was not
// fitted to: it inflates the measured arm, so units-per-metre comes out too
// small, and the target then lands short of where the hand really is.
//
// It also stacks with something already there. g_calHand is the CONTROLLER,
// which sits about 9 cm past the wrist, so the measurement is long before this
// adds anything at all. A 0.65 m arm can be taken as 0.86 m, and a scale a
// quarter too small is an arm a quarter too short.
//
// Left at 0.12 so nothing changes for anyone until it has been tested against
// a second body, but it is a dial now: one person's number should never have
// been everybody's.
constexpr float kReachAllowanceMetres = 0.12f;
std::atomic<float> g_reachAllowanceM{ kReachAllowanceMetres };
bool g_haveSocketTie[kArmSlots] = {};
// ---- Asking the game what the grip is (2026-09-19) ------------------
// The video settled what the numbers could not. The gun sits a fixed distance
// from the hand and the DIRECTION of that offset swings as the hand turns: a
// lever arm. So the gun hangs at an offset from the socket rather than at its
// origin, the socket's position is right (measured at 0.0 units from the hand)
// and its ROTATION is what is wrong.
//
// Which kills the identity tie. Bind says the socket and the hand share a
// rotation, and they do - but only matters if the gun sits AT the socket, and it
// does not. The grip angle is real, and it cannot be derived from the bind pose
// or measured off our own output, which is what the first attempt did.
//
// So ask the game. Stop writing the hand and the socket for one frame, let the
// animation pose them as it would with the weapon held properly, then read the
// turn between them. That is the grip, from the only source that actually knows
// it. One frame of the animation's arm is not something anybody will see.
volatile LONG g_gripProbe = 0; // 0 idle, 1 hands off this frame, 2 read it next

// One cache per BODY, not one in total (2026-09-19). This holds the local
// rotations the solve wants, so that ArmIk_OnPoseComplete can put them back
// after the animation has finished writing its own. It is keyed by JOINT
// INDEX - and Chris and Sheva share a rig layout, so their indices land on top
// of each other. One cache between two characters would have each of them
// wearing the other's arm pose.
//
// Which body is being solved comes from g_solvingBody, set once at the top of
// a solve. A parameter would be tidier, but this is read sixteen levels down
// in code that only ever runs on one thread and never reenters, and threading
// it through every call site is a bigger change than the thing it protects.
constexpr int kPlayerBody = 0;
constexpr int kPartnerBody = 1;
// ONE PER THREAD (2026-09-24). PoseGuard has to arm the debug registers on
// about seventy-five threads to hold a pose, because the animation is a job
// system and any worker can be the one that writes a joint. So "only ever runs
// on one thread" was never true of this engine, and two workers posing the two
// characters at the same moment share one g_solvingBody between them: whichever
// set it last decides which cache BOTH of them fill. That crosses the stamps in
// both directions at once, continuously, which is exactly the shape of the
// refusals - 15 one way, then 111 the other, on addresses that never changed.
//
// Thread-local costs nothing and makes the question meaningless: a solve's idea
// of who it is solving cannot be overwritten by another thread's solve.
thread_local int g_solvingBody = kPlayerBody;
// ...and the paragraph above was wrong, which is the whole of tonight's bug
// (2026-09-24). "Only ever runs on one thread and never reenters" was an
// assumption, never a measurement, and the refusals prove it false: on this
// machine BOTH slots ended up stamped with the other character's array, 15
// refusals one way then 111 the other. A slot index set at the top of a solve
// and read sixteen levels down is not an identity; it is a guess about the
// call stack, and any path into the writer that is not underneath the matching
// RunSolve makes that guess wrong.
//
// So the solve now also publishes the ARRAY it is working on, and the writer
// checks the joint it was handed against it. A crossed write is caught where
// it is made, by the one piece of information that cannot be inferred wrongly,
// instead of being cached and caught at the far end after the damage.
thread_local unsigned char* g_solvingJoints = nullptr;

// ONE SOLVE AT A TIME (2026-09-24). The thread ids settled the argument: the
// solve ran on at least four threads and the animation posed on at least eight,
// on both machines. Making the slot index per-thread stopped the two characters
// driving each other, and the logs from that run are clean - no crossed writes,
// no refusals - but it did nothing about the rest of the state, which is shared
// on purpose because it has to persist from frame to frame: the remembered arm
// length, the twist history, the socket ties, the quiet window.
//
// That state cannot be made per-thread, because the thread that solves this
// frame is not the one that solved last. It has to be one solve at a time
// instead, and the log says plainly what happens without it. The arm's honest
// rest length was measured eight separate times and came back 53.2 units on
// every one of them, while the value the solve compared against flapped between
// 0.0, 47.6, 48.8, 49.5, 61.0 and 61.9 - one thread zeroing the length and
// opening the quiet window while another, already past that check, stored a
// measurement taken off an arm we were posing. Ten per cent out throws the
// frame away, so the animation takes the arm, and two seconds of it forgets the
// skeleton entirely. That is the user's "my personal aiming IK broke" and their
// partner's arms being re-found thirty-one times in one session.
//
// Recursive because the entry points are allowed to grow into each other; the
// hold is a fraction of a millisecond and the hot animation hooks do not take
// it at all.
std::recursive_mutex g_solveMutex;
unsigned g_solveWaits = 0; // times a second thread had to queue behind the first

// Counted, not just taken. If this reads zero the threads were never actually
// overlapping and the flapping arm length came from somewhere else.
struct ArmIkSolveLock {
    ArmIkSolveLock()
    {
        if (!g_solveMutex.try_lock()) {
            ++g_solveWaits;
            g_solveMutex.lock();
        }
    }
    ~ArmIkSolveLock() { g_solveMutex.unlock(); }
    ArmIkSolveLock(const ArmIkSolveLock&) = delete;
    ArmIkSolveLock& operator=(const ArmIkSolveLock&) = delete;
};
unsigned g_crossStamps = 0;   // writes whose joint was not the body being solved
unsigned g_crossRefused[kArmBodies] = {}; // cached poses refused as somebody else's
float g_poseQuat[kArmBodies][kMaxJoints][4] = {};
bool g_poseQuatValid[kArmBodies][kMaxJoints] = {};
unsigned long long g_poseQuatMs[kArmBodies] = {};

// And the same again one step earlier. "not ours" on every single call means
// g_poseQuatMs was never stamped, which means the solve never cached a
// rotation - so the fault is upstream of the callback entirely and these say
// how far up it is (2026-09-29).
unsigned long g_jbCalls = 0;        // ArmIk_OnJointBuilt reached
unsigned long g_jbNotOurs = 0;      // the joint was not in the player array
unsigned long g_solveRuns = 0;      // RunSolve entered
unsigned long g_wlrErr = 0;         // a rotation was refused, it would not rebuild
unsigned long g_cached = 0;         // rotations actually cached by the solve
unsigned long g_cacheCleared = 0;   // times the player cache was thrown away
int g_lastClearSite = 0;            // and which line did it

// The exact joint array each cached pose was worked out FOR (2026-09-24,
// user: "what is causing the other skeleton to drive one that isn't theirs?
// They shouldn't even be in the same zipcode. Sheva is sheva, chris is chris.
// No touchy!").
//
// Quite. The reason it kept happening is that nothing carried ownership: every
// write worked out whose joint it was at the far end, by pointer range, by
// index, or by what was near a hand - and each of those inferences has been
// wrong at least once. Fix one and the next one bites.
//
// A pose now remembers the array it belongs to, and is refused against any
// other. Nothing has to be deduced, so nothing can be deduced wrongly, and a
// skeleton swapped under us by a costume or a level fails the test instead of
// being written into.
unsigned char* g_poseQuatJoints[kArmBodies] = {};

// ---- Driving the pose, not its result (2026-09-17) -----------------------
// +0x20 is the joint's local rotation, a unit quaternion in x,y,z,w. Measured,
// not assumed: it reads length 1.000 on every joint, it moves as the arm moves
// (the elbow swinging from 0.251 to 0.801 in y), and the wrist reads a clean
// identity. +0x10 is the bone offset with its length in w, +0x30 the scale,
// +0x50 the world matrix the game composes from all three.
//
// Writing +0x50 moves the arm on screen and nothing else, because everything
// else recomposes from the locals. Writing +0x20 changes the pose itself, so
// the hand, the fingers, the socket at the wrist and whatever is being held
// all follow without any of them having to be found first.
constexpr int kOffJointLocalRot = 0x20;

// How much of the arm's swing the collarbone takes. 0 is a fixed shoulder,
// which is what it was; 1 would put the whole reach in the collarbone and
// leave the arm straight. A third is about what a shoulder really does.
// OFF (2026-09-18). The collarbone servo works by rotating a basis it read
// back from the joint - which is our own write from last frame - so it is a
// feedback loop of exactly the kind that has already been found twice tonight,
// in the shoulder and in the forearm. At rest the hand sits about 0.88 of full
// reach, which is past the onset, so it runs constantly with nobody moving.
//
// It is not being debugged while it is also a suspect in the spin. Zero takes
// it out of the picture; when the spin is settled it comes back built the way
// the shoulder and elbow now are - from the bind pose, stateless, never from
// anything we wrote.
constexpr float kClavicleShare = 0.0f;
// How far out the arm has to be before the collarbone starts helping, as a
// fraction of its full span. Below this the shoulder stays put and the depth
// is entirely the arm's, which is what keeps close work 1:1.
constexpr float kClavicleOnset = 0.75f;
// How much of your hand's turn the forearm carries, leaving the rest for the
// wrist. Half is roughly how a real arm splits pronation, and it keeps either
// joint from being asked for an angle that tears the mesh.
// How much of your hand's twist the forearm carries, the rest staying at the
// wrist. Half is roughly how a real forearm shares pronation.
constexpr float kForearmTwistShare = 0.5f;

// ---- Where a twist actually goes (2026-09-19) ---------------------------
// Turning your hand over is not one joint's work, and modelling it as one is
// what has been tearing the mesh at the extremes. Real anatomy, shoulder down:
//
//   * the WRIST barely twists at all. It flexes and deviates; what feels like
//     wrist rotation is almost entirely happening further up.
//   * the FOREARM does most of it. The radius crosses the ulna - pronation and
//     supination - for about 80 degrees each way from neutral.
//   * past that the HUMERUS rotates at the shoulder, which is why you cannot
//     turn your palm through a full circle with your elbow pinned to your side.
//
// So the forearm takes its share up to its limit, the upper arm takes a quarter
// of everything plus ALL of whatever runs past that limit, and nothing is ever
// capped: the excess always has somewhere further up to go. That is the whole
// point - a joint asked for more than it has is what makes skin tear.
constexpr float kForearmTwistLimitDeg = 80.0f;
// Raised 0.25 -> 0.60 on 2026-09-19, at the user's suggestion that the forearm
// flips because the bicep is not taking enough. Worth testing on its own: if the
// flip moves further out or softens, the humerus was under-driven and this is
// the lever, not the quaternion maths.
// Back to 0.25. Tried at 0.60 on 2026-09-19 to test whether the forearm flips
// because the bicep is under-driven: it changed the flip not at all, and the
// bicep looked right at 0.25. So the flip is insensitive to how the twist is
// SHARED, which rules out range as the cause and points at the measurement.
constexpr float kUpperArmTwistShare = 0.25f;
// Of the twist the forearm carries, how much goes to the bone out at the WRIST
// rather than the one back at the elbow. A forearm winds up along its length, so
// the far end turns most.
constexpr float kDistalTwistShare = 0.70f;

// Which way round the quaternion builds its matrix. Worked out at runtime
// against a joint whose local rotation and world matrices are all known,
// because getting this backwards is a silent mirror rather than an error.
int g_quatConvention = -1; // 0 = as written below, 1 = its transpose

void QuatToMat3(const float q[4], bool transpose, float out[9])
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float r[9] = { 1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y + z * w), 2.0f * (x * z - y * w),
        2.0f * (x * y - z * w), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z + x * w), 2.0f * (x * z + y * w),
        2.0f * (y * z - x * w), 1.0f - 2.0f * (x * x + y * y) };
    if (!transpose) {
        std::memcpy(out, r, sizeof(r));
        return;
    }
    out[0] = r[0]; out[1] = r[3]; out[2] = r[6];
    out[3] = r[1]; out[4] = r[4]; out[5] = r[7];
    out[6] = r[2]; out[7] = r[5]; out[8] = r[8];
}

void Mat3ToQuat(const float m[9], bool transpose, float q[4])
{
    float r[9];
    if (!transpose) {
        std::memcpy(r, m, sizeof(r));
    } else {
        r[0] = m[0]; r[1] = m[3]; r[2] = m[6];
        r[3] = m[1]; r[4] = m[4]; r[5] = m[7];
        r[6] = m[2]; r[7] = m[5]; r[8] = m[8];
    }
    const float trace = r[0] + r[4] + r[8];
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q[3] = 0.25f * s;
        q[0] = (r[5] - r[7]) / s;
        q[1] = (r[6] - r[2]) / s;
        q[2] = (r[1] - r[3]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
        const float s = std::sqrt(1.0f + r[0] - r[4] - r[8]) * 2.0f;
        q[3] = (r[5] - r[7]) / s;
        q[0] = 0.25f * s;
        q[1] = (r[3] + r[1]) / s;
        q[2] = (r[6] + r[2]) / s;
    } else if (r[4] > r[8]) {
        const float s = std::sqrt(1.0f + r[4] - r[0] - r[8]) * 2.0f;
        q[3] = (r[6] - r[2]) / s;
        q[0] = (r[3] + r[1]) / s;
        q[1] = 0.25f * s;
        q[2] = (r[7] + r[5]) / s;
    } else {
        const float s = std::sqrt(1.0f + r[8] - r[0] - r[4]) * 2.0f;
        q[3] = (r[1] - r[3]) / s;
        q[0] = (r[6] + r[2]) / s;
        q[1] = (r[7] + r[5]) / s;
        q[2] = 0.25f * s;
    }
}

// out = a * b, both row-major, rows as basis vectors.
void Mul3(const float a[9], const float b[9], float out[9])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
        }
    }
}

void Transpose3(const float m[9], float out[9])
{
    out[0] = m[0]; out[1] = m[3]; out[2] = m[6];
    out[3] = m[1]; out[4] = m[4]; out[5] = m[7];
    out[6] = m[2]; out[7] = m[5]; out[8] = m[8];
}

// The rotation part of a joint's world matrix, made into an actual rotation.
//
// Dividing each row by its own length is not enough (2026-09-18): the rows come
// back very slightly out of square with each other, and the error carries all
// the way through. A matrix that is not a rotation has no exact quaternion, so
// Mat3ToQuat returns the nearest one and the pose rebuilds wrong by precisely
// the amount the rows were skewed - 0.011 of skew gave 0.021 of error, 0.222
// gave 0.220. Gram-Schmidt: keep row 0, take the part of row 1 perpendicular to
// it, and let row 2 be their cross product.
bool WorldRot(const unsigned char* joints, int index, float out[9])
{
    if (index < 0)
        return false;
    float rows[12];
    if (!TryRead(rows, joints + index * kJointStride + kOffJointWorldMatrix, sizeof(rows)))
        return false;
    float x[3] = { rows[0], rows[1], rows[2] };
    float y[3] = { rows[4], rows[5], rows[6] };
    const float z0[3] = { rows[8], rows[9], rows[10] };
    if (!Normalise3(x))
        return false;
    const float along = Dot3(y, x);
    for (int i = 0; i < 3; ++i)
        y[i] -= x[i] * along;
    if (!Normalise3(y))
        return false;
    float z[3];
    Cross3(x, y, z);
    if (!Normalise3(z))
        return false;
    // Keep the handedness the matrix actually had.
    if (Dot3(z, z0) < 0.0f) {
        z[0] = -z[0];
        z[1] = -z[1];
        z[2] = -z[2];
    }
    std::memcpy(out, x, sizeof(x));
    std::memcpy(out + 3, y, sizeof(y));
    std::memcpy(out + 6, z, sizeof(z));
    return true;
}

// Once, on a joint whose local rotation, own world and parent's world are all
// readable: whichever way round reproduces the world it actually has is the way
// the game reads its quaternions.
void DecideQuatConvention(const unsigned char* joints, const unsigned char* parents, int count, int index)
{
    if (g_quatConvention >= 0 || index < 0 || index >= count)
        return;
    const int parent = parents[index];
    if (parent >= count)
        return;
    float childW[9], parentW[9], q[4];
    if (!WorldRot(joints, index, childW) || !WorldRot(joints, parent, parentW)
        || !TryRead(q, joints + index * kJointStride + kOffJointLocalRot, sizeof(q)))
        return;
    float parentT[9], wanted[9];
    Transpose3(parentW, parentT);
    Mul3(childW, parentT, wanted);
    float error[2];
    for (int mode = 0; mode < 2; ++mode) {
        float built[9];
        QuatToMat3(q, mode == 1, built);
        float sum = 0.0f;
        for (int i = 0; i < 9; ++i) {
            const float d = built[i] - wanted[i];
            sum += d * d;
        }
        error[mode] = std::sqrt(sum);
    }
    g_quatConvention = error[1] < error[0] ? 1 : 0;
    Log_Printf("ArmIk: the local rotation at +0x20 rebuilds joint %d's world %s (error %.4f against %.4f) - "
               "writing the pose that way",
        index, g_quatConvention ? "transposed" : "as-is", error[g_quatConvention], error[1 - g_quatConvention]);
}

// Sets a joint's local rotation so that its world rotation comes out as
// wantWorld, given the world its parent will have.
void WriteLocalRotation(unsigned char* joints, int index, const float wantWorld[9], const float parentWorld[9])
{
    if (index < 0 || g_quatConvention < 0)
        return;
    float parentT[9], local[9], q[4];
    Transpose3(parentWorld, parentT);
    Mul3(wantWorld, parentT, local);
    Mat3ToQuat(local, g_quatConvention == 1, q);
    const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(len > 0.5f))
        return;
    for (int i = 0; i < 4; ++i)
        q[i] /= len;
    // Verify before writing (2026-09-18). A quaternion that does not rebuild
    // the rotation we asked for is worse than no write at all: writing identity
    // puts the joint in its bind pose, and the elbow's bind offset points
    // straight out sideways, so a failed conversion showed up as Chris standing
    // in a T-pose. If it does not check out, leave the animation alone.
    float built[9], rebuilt[9];
    QuatToMat3(q, g_quatConvention == 1, built);
    Mul3(built, parentWorld, rebuilt);
    float err = 0.0f;
    for (int i = 0; i < 9; ++i) {
        const float d = rebuilt[i] - wantWorld[i];
        err += d * d;
    }
    err = std::sqrt(err);
#if RE5VR_DIAGNOSTICS
    if (err > 0.01f) {
        // The conversion round-trips exactly on 200 random rotations on the
        // bench, so a failure here means the matrix going in is not a rotation.
        // Which would mean the world matrix rows at +0x50 are not the
        // orthogonal basis they look like - so measure that and say so.
        static unsigned long long s_ms = 0;
        const unsigned long long now = GetTickCount64();
        if (now - s_ms >= 1000) {
            s_ms = now;
            const float wantSkew[3] = { Dot3(wantWorld, wantWorld + 3), Dot3(wantWorld, wantWorld + 6),
                Dot3(wantWorld + 3, wantWorld + 6) };
            const float parentSkew[3] = { Dot3(parentWorld, parentWorld + 3),
                Dot3(parentWorld, parentWorld + 6), Dot3(parentWorld + 3, parentWorld + 6) };
            const float trace = local[0] + local[4] + local[8];
            Log_Printf("ArmIk: joint %d - wrote nothing, the pose rebuilds %.3f off. local trace %.3f, "
                       "row lengths %.3f %.3f %.3f; want rows dot to (%.3f %.3f %.3f), parent rows to "
                       "(%.3f %.3f %.3f) - zero means square",
                index, err, trace, Length3(local), Length3(local + 3), Length3(local + 6), wantSkew[0],
                wantSkew[1], wantSkew[2], parentSkew[0], parentSkew[1], parentSkew[2]);
        }
    }
#endif
    if (g_solvingBody == kPartnerBody)
        ++g_pwOffered;
    if (err > 0.01f) {
        ++g_wlrErr;
        if (g_solvingBody == kPartnerBody) {
            g_partnerWhy = "a joint's pose would not rebuild, so it was not written";
            ++g_pwRebuild;
        }
        return;
    }
    if (g_solvingBody == kPartnerBody)
        ++g_pwWritten;
    TryWrite(joints + index * kJointStride + kOffJointLocalRot, q, sizeof(q));
    // Whose joint is this, really? g_solvingBody says which cache to fill and
    // the answer is only right while this call is underneath the RunSolve that
    // set it. Anything else - a debug poke, a second solve part way through the
    // first, a hook firing off the frame - stamps one character's pose into the
    // other one's cache, and from then on every write for that body is refused
    // and the animation takes the arm back.
    if (index < kMaxJoints && joints != g_solvingJoints) {
        ++g_crossStamps;
#if RE5VR_DIAGNOSTICS
        static unsigned long long s_toldCross = 0;
        const unsigned long long nowCross = GetTickCount64();
        if (nowCross - s_toldCross >= 2000) {
            s_toldCross = nowCross;
            Log_Printf("ArmIk: a write for %p arrived while %p was being solved (%s) - not caching it",
                joints, g_solvingJoints, g_solvingBody == kPartnerBody ? "their body" : "your body");
        }
#endif
        return;
    }
    if (index < kMaxJoints) {
        std::memcpy(g_poseQuat[g_solvingBody][index], q, sizeof(q));
        g_poseQuatJoints[g_solvingBody] = joints;
        // The guard covers whoever is being solved (2026-09-24, user: "we need
        // to copy the exact same logic we're using in our skeleton to the
        // multiplayer skeleton. Pose guards and all").
        //
        // It used to be the player's alone, for a reason that was real at the
        // time and is no longer: it held a joint BY INDEX on one skeleton, so
        // holding a partner's joint 46 parked their rotation on the player's
        // joint 46. A held slot now carries whose skeleton it belongs to, so
        // the two cannot be confused, and the old reasoning - "a partner's
        // arms are drawn, not aimed, so a frame of animation on them costs
        // nothing worth this" - was written before anybody was watching a
        // partner closely. Watched, it costs plenty: the animation wins
        // whatever share of the frames we do not hold.
        //
        // The old reasoning, kept because it explains the bug it caused:
        // (2026-09-20).
        //
        // It holds a pose by address, and the address it computes is
        // g_joints + index * stride - where g_joints is one skeleton, set once,
        // and that skeleton is the player's. Holding a partner's joint 46
        // therefore parked the PARTNER's rotation on the PLAYER's joint 46, and
        // a hardware watchpoint kept putting it back. That is the co-op
        // tester's "Chris' whole top half of his body vanished".
        //
        // It is also why "Driving their arms" flickered: a joint forced to a
        // rotation from the wrong body leaves a world matrix that no longer
        // round-trips, the rebuild check above then refuses to write, and the
        // pose cache stops being refreshed - which is precisely what that line
        // reports.
        //
        // The guard exists to stop the animation snapping YOUR arm back while
        // aiming. A partner's arms are drawn, not aimed, so a frame of
        // animation on them costs nothing worth this.
        // Same rule for the guard: it is handed the array as well, so a held
        // address can never belong to the other character.
        PoseGuard_Hold(joints, index, q);
        g_poseQuatValid[g_solvingBody][index] = true;
        g_poseQuatMs[g_solvingBody] = GetTickCount64();
        ++g_cached;
    }
}

// Rotates a whole basis by r: each row is a world direction, so each rotates.
void RotBasis(const Rot3& r, const float in[9], float out[9])
{
    for (int i = 0; i < 3; ++i)
        RotApply(r, in + i * 3, out + i * 3);
}

// ---- Posing from scratch, not from where we are (2026-09-18) -------------
// The solver used to work out a delta - the rotation taking the arm from where
// it is to where you are pointing - and apply it to the pose it just measured.
// That is sound only while the pose is sound. The moment aiming corrupted it,
// the next frame measured the corrupted arm, computed a delta from THAT, wrote
// it, and measured it again. Nothing in the loop pulls back towards anything,
// so the arm never recovered even after aim was released - the user's report,
// and the clearest possible description of a feedback loop.
//
// Built from scratch there is no loop: whatever state the arm is in, the pose
// that comes out depends only on where your hand is. A frame of corruption is
// gone the next frame.
//
// A joint's rotation is what points its child's bone. The child sits along a
// fixed direction in the parent's own frame - the bind offset at +0x10, which
// on this skeleton is a clean axis: the elbow's reads (-26.4, 0, 0), straight
// down local -X. So the parent's world rotation is whatever maps local -X onto
// the direction the bone needs to point, and the remaining freedom - the twist
// about the bone - is settled by the pole vector.
bool BindAxis(const unsigned char* joints, int index, int* outAxis, float* outSign)
{
    float bind[4];
    if (!TryRead(bind, joints + index * kJointStride + kOffJointBindOffset, sizeof(bind)))
        return false;
    const float len = Length3(bind);
    if (!(len > 0.01f))
        return false;
    int best = 0;
    for (int i = 1; i < 3; ++i) {
        if (std::fabs(bind[i]) > std::fabs(bind[best]))
            best = i;
    }
    // Only when the bone really does run down one axis; anything else and the
    // caller falls back to the old relative method rather than guessing.
    if (std::fabs(bind[best]) < len * 0.95f)
        return false;
    *outAxis = best;
    *outSign = bind[best] > 0.0f ? 1.0f : -1.0f;
    return true;
}

// Whether a matrix holds its axes as rows or as columns. The quaternion side
// was settled by comparing against a joint the game had already posed; this is
// the same question for the other half, and the same answer method. Guessing it
// stood both arms straight up in the air.
int g_boneInRow = -1;

void DecideBoneConvention(const unsigned char* joints, int joint, int child, int axis, float sign)
{
    if (g_boneInRow >= 0)
        return;
    float w[9], here[3], there[3];
    if (!WorldRot(joints, joint, w) || !JointWorldPos(joints, joint, here)
        || !JointWorldPos(joints, child, there))
        return;
    float dir[3];
    Sub3(there, here, dir);
    if (!Normalise3(dir))
        return;
    float rowErr = 0.0f, colErr = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float r = w[axis * 3 + i] * sign - dir[i];
        const float c = w[i * 3 + axis] * sign - dir[i];
        rowErr += r * r;
        colErr += c * c;
    }
    rowErr = std::sqrt(rowErr);
    colErr = std::sqrt(colErr);
    g_boneInRow = rowErr <= colErr ? 1 : 0;
    Log_Printf("ArmIk: the bone runs down the matrix's %s (error %.4f against %.4f) - building poses that "
               "way",
        g_boneInRow ? "rows" : "columns", g_boneInRow ? rowErr : colErr, g_boneInRow ? colErr : rowErr);
}

// The world rotation a joint needs so that its child's bone points along dir.
// pole settles the twist; handed keeps the basis the same way round as the
// skeleton's own matrices, so nothing comes out mirrored.
// `keep` is the joint's current secondary axis. Direction has to be absolute -
// that is what stops a bad frame becoming permanent - but ROLL does not, and
// taking it from the pole instead threw away the animation's twist and left the
// palms facing up. Roll cannot run away the way position can, so it is safe to
// carry over; the pole is only the fallback when there is nothing to carry.
void BasisForBone(
    int axis, float sign, const float dir[3], const float pole[3], const float keep[3], float handed,
    float out[9])
{
    float primary[3] = { dir[0] * sign, dir[1] * sign, dir[2] * sign };
    Normalise3(primary);
    const int b = (axis + 1) % 3;
    const int c = (axis + 2) % 3;
    // A null reference means "just use the pole", which is what a caller wants
    // when it intends to apply the roll itself afterwards.
    const float keepLen = keep ? Length3(keep) : 0.0f;
    const float* ref = keepLen > 0.5f ? keep : pole;
    const float along = Dot3(ref, primary);
    float second[3] = { ref[0] - primary[0] * along, ref[1] - primary[1] * along, ref[2] - primary[2] * along };
    if (!Normalise3(second)) {
        const float alt[3] = { 0.0f, 1.0f, 0.0f };
        Cross3(primary, alt, second);
        if (!Normalise3(second)) {
            second[0] = 1.0f;
            second[1] = 0.0f;
            second[2] = 0.0f;
        }
    }
    float third[3];
    Cross3(primary, second, third);
    Normalise3(third);
    if (handed < 0.0f) {
        third[0] = -third[0];
        third[1] = -third[1];
        third[2] = -third[2];
    }
    if (g_boneInRow != 0) {
        std::memcpy(out + axis * 3, primary, sizeof(float) * 3);
        std::memcpy(out + b * 3, second, sizeof(float) * 3);
        std::memcpy(out + c * 3, third, sizeof(float) * 3);
    } else {
        for (int i = 0; i < 3; ++i) {
            out[i * 3 + axis] = primary[i];
            out[i * 3 + b] = second[i];
            out[i * 3 + c] = third[i];
        }
    }
}

// +1 or -1: whether the skeleton's own matrices are right or left handed.
float Handedness(const float m[9])
{
    float cross[3];
    Cross3(m, m + 3, cross);
    return Dot3(cross, m + 6) >= 0.0f ? 1.0f : -1.0f;
}

// A joint's bind offset from its parent, expressed in world. Bind LOCAL
// rotations are identity on this rig - the T-pose dump proved it - so the rest
// orientation of every joint is the root's, and carrying an offset through the
// root's world rotation gives where that bone points when nothing has posed it.
// Nothing here reads anything we wrote, which is the whole point.
bool BindDirWorld(const unsigned char* joints, int joint, int rootJoint, float outDir[3], float* outLen)
{
    float off[4], rootRot[9];
    if (joint < 0 || rootJoint < 0 || !TryRead(off, joints + joint * kJointStride + 0x10, sizeof(off)))
        return false;
    if (!WorldRot(joints, rootJoint, rootRot))
        return false;
    for (int i = 0; i < 3; ++i) {
        outDir[i] = g_boneInRow ? off[0] * rootRot[i] + off[1] * rootRot[3 + i] + off[2] * rootRot[6 + i]
                                : off[0] * rootRot[i * 3] + off[1] * rootRot[i * 3 + 1]
                + off[2] * rootRot[i * 3 + 2];
    }
    const float len = Length3(outDir);
    if (outLen)
        *outLen = len;
    return Normalise3(outDir);
}

// Walks to the top of the skeleton. Every joint's rest rotation is this one's.
int RootJoint(const unsigned char* parents, int count, int from)
{
    int j = from;
    for (int guard = 0; guard < 32 && j >= 0 && j < count; ++guard) {
        const int p = parents[j];
        if (p >= count || p == j)
            break;
        j = p;
    }
    return j;
}

// Defined further down, past the solve it lands on top of.
void PokeJoint(const ArmIkBody& body, const unsigned char* parents, int count, const float up[3]);
bool PushOutOfBodies(float target[3], const ArmIkBody& body, const unsigned char* parents, int count,
    const ArmIkArm& arm, float* outDepth);

// ---- What the last update managed ---------------------------------------
SRWLOCK g_statusLock = SRWLOCK_INIT;
ArmIkStatus g_status = {};
unsigned long long g_statusMs = 0;

void PublishStatus(const ArmIkStatus& s)
{
    AcquireSRWLockExclusive(&g_statusLock);
    g_status = s;
    g_statusMs = GetTickCount64();
    ReleaseSRWLockExclusive(&g_statusLock);
}

// The same two swings applied straight to the world matrices, which is what
// the mod did before it learned to drive the pose. It is applied last in the
// frame and nothing recomposes afterwards, so it always shows - the gun is the
// only thing it cannot bring along. Doing BOTH means the arm you see stays
// under control even on the frames the animation wins the pose, and both come
// from the same solve so they cannot disagree.
void ApplyWorldRotations(unsigned char* joints, const unsigned char* parents, int count, const ArmIkArm& arm,
    const Rot3& r1, const Rot3& r2, const float shoulder[3])
{
    int subtree[kMaxJoints];
    {
        const int n = Subtree(parents, count, arm.shoulder, subtree);
        for (int i = 0; i < n; ++i)
            RotateJointWorld(joints, subtree[i], r1, shoulder);
    }
    float elbowNow[3];
    if (!JointWorldPos(joints, arm.elbow, elbowNow))
        return;
    {
        const int n = Subtree(parents, count, arm.elbow, subtree);
        for (int i = 0; i < n; ++i)
            RotateJointWorld(joints, subtree[i], r2, elbowNow);
    }
}

// One arm, solved and written. The shoulder stays where the animation put it;
// only the two bones below it move.
// The parameter called "hand" is a SLOT, not a side (2026-09-24). The caller
// passes `slot`, and a partner's slots are 2 and 3 while the player's are 0 and
// 1. Every bounds test in here read "hand < 2", so for a partner they all
// failed and the entire wrist path was skipped: no twist, no roll, position
// only. That is exactly what the user reported - "we still aren't syncing wrist
// rotation. So twist and roll both need to sync."
//
// Worse, the arrays it guards are per slot, so on a machine whose own player
// WAS in VR the bounds happened to pass for slots 0 and 1 and the partner's arm
// was steered from the local player's controller. That is the other half of the
// report: a flatscreen player driving a partner's right arm, and then driving
// stranger things again the moment they put a headset on.
//
// Renaming it would touch a hundred lines; saying so here costs nothing and the
// bounds are now the real ones.
bool SolveArm(unsigned char* joints, const unsigned char* parents, int count, const ArmIkArm& arm,
    const float target[3], const float poleDir[3], int hand /* really a slot */, float* outReach,
    float* outMiss)
{
    float shoulder[3], elbow[3], wrist[3];
    if (!JointWorldPos(joints, arm.shoulder, shoulder) || !JointWorldPos(joints, arm.elbow, elbow)
        || !JointWorldPos(joints, arm.wrist, wrist))
        return false;

    float upperV[3], foreV[3];
    Sub3(elbow, shoulder, upperV);
    Sub3(wrist, elbow, foreV);
    const float l1 = Length3(upperV);
    const float l2 = Length3(foreV);
    if (!(l1 > 1.0f) || !(l2 > 1.0f))
        return false;
    if (outReach)
        *outReach = l1 + l2;

    float toTarget[3];
    Sub3(target, shoulder, toTarget);
    float dir[3] = { toTarget[0], toTarget[1], toTarget[2] };
    if (!Normalise3(dir))
        return false;
    const float wanted = Length3(toTarget);
    const float minD = std::fabs(l1 - l2) + 0.5f;
    const float maxD = l1 + l2 - 0.5f;
    float d = wanted;
    if (d < minD)
        d = minD;
    if (d > maxD)
        d = maxD;
    if (outMiss)
        *outMiss = wanted - d;

    // Where the elbow has to be for the two bones to span d, pushed around the
    // shoulder-to-hand line until it points the way an elbow points.
    const float a = (l1 * l1 - l2 * l2 + d * d) / (2.0f * d);
    float hSq = l1 * l1 - a * a;
    if (hSq < 0.0f)
        hSq = 0.0f;
    const float h = std::sqrt(hSq);
    const float along = Dot3(poleDir, dir);
    float perp[3] = { poleDir[0] - dir[0] * along, poleDir[1] - dir[1] * along, poleDir[2] - dir[2] * along };
    if (!Normalise3(perp)) {
        // The pole lies along the arm: take any perpendicular rather than
        // giving up, since the elbow only has to end up somewhere sensible.
        const float alt[3] = { 0.0f, 1.0f, 0.0f };
        Cross3(dir, alt, perp);
        if (!Normalise3(perp))
            return false;
    }
    float newElbow[3] = { shoulder[0] + dir[0] * a + perp[0] * h, shoulder[1] + dir[1] * a + perp[1] * h,
        shoulder[2] + dir[2] * a + perp[2] * h };
    float newWrist[3] = { shoulder[0] + dir[0] * d, shoulder[1] + dir[1] * d, shoulder[2] + dir[2] * d };

    // Two swings: the upper arm about the shoulder to put the elbow where it
    // belongs, then the forearm about the elbow to put the hand on the target.
    // Both are worked out here as world rotations, and only then turned into
    // whichever kind of write is in use.
    float wantUpper[3];
    Sub3(newElbow, shoulder, wantUpper);
    float haveUpper[3] = { upperV[0], upperV[1], upperV[2] };
    if (!Normalise3(wantUpper) || !Normalise3(haveUpper))
        return false;
    const Rot3 r1 = RotBetween(haveUpper, wantUpper);

    // Where the hand ends up after that first swing, worked out rather than
    // read back: the forearm's rotation has to be measured from there, and
    // with the local-pose write nothing has been moved yet to read.
    float wristRel[3], wristAfter[3];
    Sub3(wrist, shoulder, wristRel);
    RotApply(r1, wristRel, wristAfter);
    wristAfter[0] += shoulder[0];
    wristAfter[1] += shoulder[1];
    wristAfter[2] += shoulder[2];

    float wantFore[3], haveFore[3];
    Sub3(newWrist, newElbow, wantFore);
    Sub3(wristAfter, newElbow, haveFore);
    if (!Normalise3(wantFore) || !Normalise3(haveFore))
        return false;
    const Rot3 r2 = RotBetween(haveFore, wantFore);

    // Always the local rotation for a PARTNER (2026-09-20). Writing world
    // matrices only survives because the player's solve runs late in the frame;
    // the partner's does not, so the game's rebuild would wipe it and their
    // arms would simply never move. That is a setting on the RECEIVER's machine
    // about how their own arms are written, and letting it decide whether a
    // partner appears at all is a failure with no relationship to its cause.
    if (g_solvingBody == kPartnerBody || XrInput_GetSettings().armIkWriteLocal) {
        // The pose itself. The game composes every world matrix below these two
        // joints from them, so the forearm, the hand, the fingers and anything
        // sitting in the socket at the wrist all come along without being
        // touched - which is the whole reason for writing here instead.
        DecideQuatConvention(joints, parents, count, arm.elbow);
        float shoulderW[9], elbowW[9];
        if (!WorldRot(joints, arm.shoulder, shoulderW) || !WorldRot(joints, arm.elbow, elbowW))
            return false;
        int shoulderParent = parents[arm.shoulder];
        float shoulderParentW[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        if (shoulderParent < count)
            WorldRot(joints, shoulderParent, shoulderParentW);

        float shoulderWant[9], elbowAfterR1[9], elbowWant[9];
        RotBasis(r1, shoulderW, shoulderWant);
        RotBasis(r1, elbowW, elbowAfterR1);
        RotBasis(r2, elbowAfterR1, elbowWant);
#if RE5VR_DIAGNOSTICS
        // The two joints keep coming out with the identical quaternion, which
        // means they are being handed the same thing. These are the inputs.
        {
            static unsigned long long s_ms = 0;
            const unsigned long long now = GetTickCount64();
            if (now - s_ms >= 1000) {
                s_ms = now;
                Log_Printf("ArmIk: in - shoulder world row0 (%.3f %.3f %.3f), elbow world row0 "
                           "(%.3f %.3f %.3f), shoulder parent row0 (%.3f %.3f %.3f); r1 row0 "
                           "(%.3f %.3f %.3f), r2 row0 (%.3f %.3f %.3f)",
                    shoulderW[0], shoulderW[1], shoulderW[2], elbowW[0], elbowW[1], elbowW[2],
                    shoulderParentW[0], shoulderParentW[1], shoulderParentW[2], r1.m[0], r1.m[1], r1.m[2],
                    r2.m[0], r2.m[1], r2.m[2]);
            }
        }
#endif

        // Built from scratch where the skeleton allows it: the bone axes are
        // read from the bind offsets, and the pose depends only on where your
        // hand is, never on the pose it is replacing. That is what stops a bad
        // frame becoming a permanent one.
        {
            int upperAxis = 0, foreAxis = 0;
            float upperSign = 1.0f, foreSign = 1.0f;
            // Which child carries each bone: the one on the path to the wrist.
            int foreChild = arm.wrist;
            for (int guard = 0; guard < 8 && foreChild < count && parents[foreChild] != arm.elbow; ++guard)
                foreChild = parents[foreChild];
            int upperChild = arm.elbow;
            for (int guard = 0; guard < 8 && upperChild < count && parents[upperChild] != arm.shoulder;
                 ++guard)
                upperChild = parents[upperChild];
            if (foreChild < count && upperChild < count && BindAxis(joints, upperChild, &upperAxis, &upperSign)
                && BindAxis(joints, foreChild, &foreAxis, &foreSign)) {
                DecideBoneConvention(joints, arm.shoulder, upperChild, upperAxis, upperSign);
                if (hand >= 0 && hand < kArmSlots) {
                    g_foreAxis[hand] = foreAxis;
                    g_foreSign[hand] = foreSign;
                }
                const float handed = Handedness(shoulderW);
                float upperDir[3], foreDir[3];
                Sub3(newElbow, shoulder, upperDir);
                Sub3(newWrist, newElbow, foreDir);

                // 1:1 (2026-09-18). Up to here the forearm points from the elbow
                // to wherever your hand is, which gives roll but can never give
                // pitch: turning your wrist down without moving your hand leaves
                // the bone pointing exactly where it was, so the gun never tips.
                //
                // So the forearm takes its whole orientation from the controller,
                // through a tie learned with the gun up, and the ELBOW moves to
                // allow it: put the elbow one forearm back along that direction
                // from your hand, then pull it onto the shoulder's reach. The gun
                // then points exactly where you point, and the small cost is that
                // the hand can sit slightly off your real one when the arm cannot
                // reach both at once - which is the right thing to give up.
                // The calibration capture. Everything it needs - the bone axis,
                // its sign, the handedness and the controller - is in scope here
                // and nowhere else, so it happens here rather than in a tidier
                // place.
                // Only ever the local player's slots (2026-09-24, user: "the
                // first person to enter VR is who ends up losing the wrist
                // sync"). Widening the bounds two builds ago was right for the
                // wrist path and wrong here: this is the T-pose CAPTURE, and
                // it started running for a partner's slots as well. So the
                // second person to put a headset on would calibrate, and their
                // machine would overwrite the tie it had already made for the
                // first person - from whatever pose that person happened to be
                // standing in, which is not a T-pose and not theirs to give.
                // Hence the first one in loses their wrists on the other's
                // screen, and only ever the first one.
                //
                // Using a tie is everybody's business; MAKING one belongs to
                // whoever is stood in the room with the controllers.
                if (g_calibratePending && hand >= kPlayerSlot && hand < kPlayerSlot + 2 && g_haveCtrl[hand]
                    && g_boneInRow >= 0) {
                    // The hand's rest rotation, READ rather than constructed
                    // (2026-09-19, restart).
                    //
                    // Three attempts constructed it - from the arm's bind
                    // direction, from the forearm's axis convention, from the
                    // character's own axes - and every one came out wrong by a
                    // fixed angle. The gun showed that as a constant offset that
                    // swings with the hand, which reads as an orbit; the hand
                    // showed nothing at all, because joint 54 is zero length from
                    // its parent and a wrong rotation there only moves fingers.
                    //
                    // None of it was necessary. The T-pose dump proved this rig
                    // carries its rest shape entirely in the bone OFFSETS, so every
                    // bind LOCAL rotation is identity - and that makes every
                    // joint's rest WORLD rotation the root's. So read the root.
                    // It cannot disagree with the skeleton's convention, because it
                    // IS the skeleton's convention. It turns with the character,
                    // and so does g_ctrlWorld, so the tie holds whichever way they
                    // are facing.
                    //
                    // That is the whole rotation half of the T-pose: stand with
                    // your arms out and your controllers level, and the angle your
                    // controller is at becomes the angle the character's hand is at
                    // in its rest pose. No aiming needed to find level.
                    int rootJoint = arm.shoulder;
                    for (int guard = 0; guard < 32; ++guard) {
                        const int p = parents[rootJoint];
                        if (p >= count || p == rootJoint)
                            break;
                        rootJoint = p;
                    }
                    float restBasis[9];
                    if (WorldRot(joints, rootJoint, restBasis)) {
                        float ctrlT[9];
                        Transpose3(g_ctrlWorld[hand], ctrlT);
                        Mul3(restBasis, ctrlT, g_wristOffset[hand]);
                        std::memcpy(g_armOffset[hand], g_wristOffset[hand], sizeof(g_armOffset[hand]));
                        // Keep the controller's own orientation too. The twist that
                        // gets shared up the arm is measured from HERE - see the
                        // distribution below - because measuring it from the pole
                        // starts it at an offset, mirrored per hand, and you run out
                        // of turn after ninety degrees one way while having room to
                        // spare the other.
                        std::memcpy(g_ctrlRef[hand], g_ctrlWorld[hand], sizeof(g_ctrlRef[hand]));
                        g_haveCtrlRef[hand] = true;
                        g_haveWristOffset[hand] = true;
                        g_haveArmOffset[hand] = true;
                        g_tiesCalibrated[hand] = true;
                        g_haveSocketTie[hand] = false; // retake the weapon's angle against the new hand
                        Log_Printf("ArmIk: tied the %s hand to your controller against the skeleton's rest "
                                   "rotation (root joint %d)",
                            hand == 1 ? "right" : "left", rootJoint);
                        if (g_tiesCalibrated[0] && g_tiesCalibrated[1])
                            InterlockedExchange(&g_calibratePending, 0);
                    }
                    if (g_tiesCalibrated[0] && g_tiesCalibrated[1])
                        InterlockedExchange(&g_calibratePending, 0);
                }

                bool aimedByHand = false;
                float wantElbowAbs[9];
                const bool haveHand
                    = XrInput_GetSettings().armIkWrist && hand >= 0 && hand < kArmSlots && g_haveCtrl[hand];
                if (haveHand && g_boneInRow >= 0) {
                    float elbowNowB[9];
                    // The same rule: this learns a tie by watching the local
                    // player settle into an aim, and a partner does not aim on
                    // this machine at all.
                    if (hand >= kPlayerSlot && hand < kPlayerSlot + 2 && !g_haveArmOffset[hand]
                        && !g_tiesCalibrated[hand] && AimSettled()
                        && WorldRot(joints, arm.elbow, elbowNowB)) {
                        float ctrlT[9];
                        Transpose3(g_ctrlWorld[hand], ctrlT);
                        Mul3(elbowNowB, ctrlT, g_armOffset[hand]);
                        g_haveArmOffset[hand] = true;
                    }
                    if (g_haveArmOffset[hand]) {
                        Mul3(g_armOffset[hand], g_ctrlWorld[hand], wantElbowAbs);
                        float axis[3];
                        if (g_boneInRow) {
                            axis[0] = wantElbowAbs[foreAxis * 3];
                            axis[1] = wantElbowAbs[foreAxis * 3 + 1];
                            axis[2] = wantElbowAbs[foreAxis * 3 + 2];
                        } else {
                            axis[0] = wantElbowAbs[foreAxis];
                            axis[1] = wantElbowAbs[3 + foreAxis];
                            axis[2] = wantElbowAbs[6 + foreAxis];
                        }
                        for (int i = 0; i < 3; ++i)
                            axis[i] *= foreSign;
                        // SWITCHED OFF (2026-09-19). Everything in here decides
                        // where the elbow and the wrist GO from the controller's
                        // ORIENTATION - it takes the forearm's direction out of the
                        // hand's basis and builds the arm around that.
                        //
                        // Which cannot work now that the hand is tied to the
                        // skeleton's rest rotation. At the moment you calibrate,
                        // the hand's basis IS the root's basis, so the forearm is
                        // sent along a fixed body axis - and from then on it only
                        // turns as your wrist turns. It never reaches for where
                        // your hand actually is. The user: "the IK does not update
                        // as it should, however they were when you calibrated is
                        // what they stay as", and the gun leaving the hand follows
                        // from the same thing - the arm is not where the hand is.
                        //
                        // Position belongs to the solve. The two-bone pass below
                        // already puts the wrist exactly on your hand and bends the
                        // elbow to suit. All the hand should contribute is ROLL,
                        // and that goes through keepElbow further down, which no
                        // longer waits on this block.
                        if (false && Normalise3(axis)) {
                            // DEPTH (2026-09-18). This used to pin the elbow at
                            // exactly one upper arm from the shoulder, along the
                            // bearing of the ideal elbow, and then hang the hand
                            // a forearm beyond it. That throws away the hand's
                            // DISTANCE and keeps only its bearing: pull your
                            // hand in toward your chest and the ideal elbow
                            // slides toward the shoulder, the bearing barely
                            // moves, and the wrist stays out at full stretch.
                            // Which is the user's report exactly - the arm
                            // swings instead of the gun coming closer.
                            //
                            // With rigid bones the hand's position and the
                            // forearm's direction cannot both be exact, so stop
                            // pretending. The hand wins, because a hand in the
                            // wrong place is the thing you can see. The
                            // controller becomes the POLE instead: the elbow
                            // goes to the point on its circle - one upper arm
                            // from the shoulder, one forearm from your hand -
                            // nearest where the controller says it should be.
                            // When the two agree, which is most of the time,
                            // the forearm still lands exactly along the
                            // controller. When they cannot, the gun is a few
                            // degrees out rather than a foot out.
                            const float ideal[3] = { target[0] - axis[0] * l2, target[1] - axis[1] * l2,
                                target[2] - axis[2] * l2 };
                            float pole[3];
                            Sub3(ideal, shoulder, pole);
                            const float alongPole = Dot3(pole, dir);
                            float perpPole[3] = { pole[0] - dir[0] * alongPole, pole[1] - dir[1] * alongPole,
                                pole[2] - dir[2] * alongPole };
                            if (Normalise3(perpPole)) {
                                // a, h, d and dir are the span already worked out
                                // for where your hand really is, so the wrist
                                // stays ON the target and only the elbow's roll
                                // about the shoulder-to-hand line changes.
                                for (int i = 0; i < 3; ++i) {
                                    newElbow[i] = shoulder[i] + dir[i] * a + perpPole[i] * h;
                                    newWrist[i] = shoulder[i] + dir[i] * d;
                                }
                                Sub3(newElbow, shoulder, upperDir);
                                Sub3(newWrist, newElbow, foreDir);
                                if (Normalise3(upperDir) && Normalise3(foreDir))
                                    aimedByHand = true;
                            }
                        }
                    }
                }
                if (g_boneInRow >= 0 && (aimedByHand || (Normalise3(upperDir) && Normalise3(foreDir)))) {
                    float wantShoulder[9], wantElbow[9];
                    // The UPPER arm's roll had the same self-reference the forearm
                    // had, and it was still here after the forearm's was taken out
                    // (2026-09-18). It read the shoulder's current world rotation -
                    // which is our own write from last frame - re-projected it
                    // against an upper-arm direction that had moved, wrote the
                    // result, and read it back. That precesses at a steady rate
                    // with nobody moving, and because it is the TOP of the chain
                    // every bone below inherits the turn: what the user sees
                    // spinning is the forearm, but what is spinning is the shoulder
                    // carrying it round. Fixing the forearm alone could never have
                    // stopped it, which is why it did not.
                    //
                    // The pole is computed fresh from the body's own axes every
                    // frame and never from anything we wrote, so it cannot drift.
                    const float keepShoulder[3] = { 0.0f, 0.0f, 0.0f };
                    BasisForBone(upperAxis, upperSign, upperDir, poleDir, keepShoulder, handed, wantShoulder);
                    // The forearm's ROLL comes from your controller, not from
                    // the animation (2026-09-18). This is what makes the gun turn
                    // over, and it is the user's idea: the weapon hangs off a
                    // socket below the elbow, so rolling the forearm rolls the gun
                    // and the hand follows - exactly what happens in the game's own
                    // poses. Turning the hand alone could never do it, because the
                    // socket sits above the wrist in the hierarchy.
                    //
                    // The controller's RIGHT axis is the reference, mirrored per
                    // hand (2026-09-18). Using its up axis left the roll ninety
                    // degrees out and the user was holding the correction by hand -
                    // the right controller turned right, the left turned left,
                    // which is what a ninety degree error about the bone looks like
                    // on a mirrored pair. Right is that same axis already turned.
                    // ---- The forearm's roll (2026-09-19, rebuilt) ----------
                    // Every version of this so far picked ONE axis out of the
                    // hand's basis and handed it to BasisForBone as a reference.
                    // That axis is only a usable reference while it points across
                    // the bone. Turn your hand until it lines up with the forearm
                    // and its perpendicular part shrinks to nothing, so the
                    // direction that survives is noise - and the forearm chases
                    // that noise as fast as the frames arrive.
                    //
                    // Which is what the user is describing: it spins the whole
                    // time, certain angles send it wild, and it goes quiet in the
                    // one pose where that axis happens to sit squarely across the
                    // bone - the palm-down grip it was calibrated in.
                    //
                    // So stop projecting an axis. Build the forearm from the pole
                    // alone, which is stable, then measure how far the hand is
                    // TWISTED about the bone relative to that and give the forearm
                    // a share of the angle. A rotation about a known axis has no
                    // degenerate direction: when there is no twist the answer is
                    // zero rather than an arbitrary heading.
                    BasisForBone(foreAxis, foreSign, foreDir, poleDir, nullptr, handed, wantElbow);
                    const bool rollFromHand
                        = XrInput_GetSettings().armIkWrist && hand >= 0 && hand < kArmSlots && g_haveCtrl[hand];
                    if (rollFromHand && g_haveArmOffset[hand] && g_quatConvention >= 0) {
                        float poleT[9], rel[9], q[4];
                        Transpose3(wantElbow, poleT);
                        Mul3(wantElbowAbs, poleT, rel);
                        Mat3ToQuat(rel, g_quatConvention == 1, q);
                        // One hemisphere, or the twist changes sign with whichever
                        // of the two equivalent quaternions came back.
                        if (q[3] < 0.0f) {
                            for (int i = 0; i < 4; ++i)
                                q[i] = -q[i];
                        }
                        // The bone runs along this axis in the basis we just built,
                        // so the twist is the part of the turn about it.
                        float ax[3] = { 0.0f, 0.0f, 0.0f };
                        ax[foreAxis] = foreSign;
                        const float along = q[0] * ax[0] + q[1] * ax[1] + q[2] * ax[2];
                        float tq[4] = { ax[0] * along, ax[1] * along, ax[2] * along, q[3] };
                        const float tl
                            = std::sqrt(tq[0] * tq[0] + tq[1] * tq[1] + tq[2] * tq[2] + tq[3] * tq[3]);
                        if (tl > 0.0001f) {
                            for (int i = 0; i < 4; ++i)
                                tq[i] /= tl;
                            // PUT BACK 2026-09-19. Removing this blend was meant to
                            // kill the ninety degree flip and instead wrecked the
                            // forearm outright - so whatever the flip is, this is
                            // also doing real work that nothing else replaces. The
                            // user called it on sight and they were right.
                            // As an ANGLE, unwrapped - not a blend between two
                            // quaternions (2026-09-19). The flip detector caught it
                            // outright: at the moment it happens the twist reads
                            // w = 0.00 with its axis component alternating between
                            // +1.000 and -1.000 frame to frame. w = 0 is a rotation
                            // of exactly 180, and +180 and -180 are the same turn -
                            // but HALFWAY to them is +90 and -90, which are not.
                            // The blend had nothing to choose between, so it chose
                            // differently every frame and the applied roll jumped
                            // 177 to 180 degrees.
                            //
                            // It lands on 180 after only 90 degrees of your hand
                            // because this is measured against the pole, which sits
                            // about 90 from where a hand rests. That is why the
                            // number was always 90 and never moved when the shares
                            // changed.
                            //
                            // Taking the angle and carrying it across frames means
                            // 181 follows 179 instead of becoming -179, so half of
                            // it moves smoothly through the singularity instead of
                            // leaping across it.
                            const float sh = kForearmTwistShare;
                            float rollAng = 2.0f
                                * std::atan2(tq[0] * ax[0] + tq[1] * ax[1] + tq[2] * ax[2], tq[3]);
                            if (hand >= 0 && hand < kArmSlots) {
                                static float s_lastRollAng[kArmSlots] = {};
                                static bool s_haveRollAng[kArmSlots] = {};
                                if (s_haveRollAng[hand]) {
                                    while (rollAng - s_lastRollAng[hand] > 3.14159265358979f)
                                        rollAng -= 6.28318530717959f;
                                    while (rollAng - s_lastRollAng[hand] < -3.14159265358979f)
                                        rollAng += 6.28318530717959f;
                                }
                                const float cap = 6.28318530717959f * 0.75f;
                                if (rollAng > cap)
                                    rollAng = cap;
                                if (rollAng < -cap)
                                    rollAng = -cap;
                                s_lastRollAng[hand] = rollAng;
                                s_haveRollAng[hand] = true;
                            }
                            const float rollHalf = rollAng * sh * 0.5f;
                            const float rollSin = std::sin(rollHalf);
                            float sq[4] = { ax[0] * rollSin, ax[1] * rollSin, ax[2] * rollSin,
                                std::cos(rollHalf) };
                            const float sl
                                = std::sqrt(sq[0] * sq[0] + sq[1] * sq[1] + sq[2] * sq[2] + sq[3] * sq[3]);
                            if (sl > 0.1f) {
                                for (int i = 0; i < 4; ++i)
                                    sq[i] /= sl;
#if RE5VR_DIAGNOSTICS
                                // Catch the flip in the act (2026-09-19). Three
                                // theories about the ninety degrees have now been
                                // wrong, and the last test ruled out range, so stop
                                // reasoning and record the frame it happens on.
                                // A hand cannot turn 45 degrees between frames, so
                                // anything that big is the number jumping, not you.
                                if (hand >= 0 && hand < kArmSlots) {
                                    static float s_prevSq[2][4] = {};
                                    static bool s_havePrevSq[kArmSlots] = {};
                                    if (s_havePrevSq[hand]) {
                                        float dot = 0.0f;
                                        for (int i = 0; i < 4; ++i)
                                            dot += sq[i] * s_prevSq[hand][i];
                                        const float ad = std::fabs(dot) > 1.0f ? 1.0f : std::fabs(dot);
                                        const float jumped = 2.0f * std::acos(ad) * 180.0f / 3.14159265358979f;
                                        if (jumped > 45.0f) {
                                            Log_Printf("ArmIk: FLIP on the %s forearm - the applied roll jumped "
                                                       "%.0f deg in one frame. twist quat now "
                                                       "(%.3f %.3f %.3f w %.3f), axis dot %.3f; applied w %.3f "
                                                       "was %.3f",
                                                hand == 1 ? "right" : "left", jumped, tq[0], tq[1], tq[2],
                                                tq[3], tq[0] * ax[0] + tq[1] * ax[1] + tq[2] * ax[2], sq[3],
                                                s_prevSq[hand][3]);
                                        }
                                    }
                                    std::memcpy(s_prevSq[hand], sq, sizeof(sq));
                                    s_havePrevSq[hand] = true;
                                }
#endif
                                float sm[9], rolled[9];
                                QuatToMat3(sq, g_quatConvention == 1, sm);
                                Mul3(sm, wantElbow, rolled);
                                std::memcpy(wantElbow, rolled, sizeof(wantElbow));
                            }

                            // ---- And the bicep (2026-09-19) -------------------
                            // The upper arm has never twisted about its own length,
                            // so the one part of the arm with the most muscle on it
                            // was the only part doing nothing. Anatomically it is
                            // the third link in the chain: the wrist barely twists
                            // at all, the forearm pronates to about eighty degrees,
                            // and past that the humerus has to rotate at the
                            // shoulder - which is why you cannot turn your hand
                            // right over with your elbow pinned to your side.
                            //
                            // So the bicep takes a quarter of any twist, and ALL of
                            // whatever runs past what a forearm can do. Nothing
                            // stops at a limit; the excess always has somewhere to
                            // go, which is what keeps the mesh from tearing at the
                            // extremes.
                            //
                            // It costs nothing elsewhere. A rotation about the
                            // bone's own axis does not move the elbow, so the solve
                            // is untouched, and every joint below is written in
                            // world terms - so they hold their places and only the
                            // bicep between shoulder and elbow turns.
                            // Unwrapped across frames (2026-09-19). atan2 can only
                            // ever answer between -180 and +180, so the instant a
                            // twist crosses that line the number leaps the whole way
                            // round and the forearm snaps over. The user hits it
                            // "just over 90" rather than at 180 because the twist is
                            // measured against the POLE, which already sits some way
                            // from where a hand rests - so their 90 plus that
                            // baseline is what reaches the edge.
                            //
                            // Carrying the previous value and taking whichever
                            // equivalent angle is nearest it makes the number
                            // continuous through the wrap. This is not feedback: it
                            // does not change the pose, it only picks which of the
                            // identical answers to call it. Bounded well past what a
                            // shoulder allows, so a dropped frame cannot wind it up.
                            const float kTau = 6.28318530717959f;
                            // Measured from the pose you CALIBRATED in, not from the
                            // pole (2026-09-19). The pole is an anatomical hint that
                            // sits well away from where a hand rests, and it is
                            // mirrored per hand - so the twist started at roughly
                            // +90 on one side and -90 on the other. That is the
                            // user's report exactly: ninety degrees of turn one way
                            // before it flips, and hard to reach at all the other,
                            // opposite between the hands.
                            //
                            // The controller's turn since calibration has no such
                            // offset. Neutral is zero because neutral is where you
                            // were standing when you set it, and both directions get
                            // the same room.
                            float theta = 0.0f;
                            if (g_haveCtrlRef[hand]) {
                                float refT[9], since[9], rq[4];
                                Transpose3(g_ctrlRef[hand], refT);
                                Mul3(g_ctrlWorld[hand], refT, since);
                                Mat3ToQuat(since, g_quatConvention == 1, rq);
                                if (rq[3] < 0.0f) {
                                    for (int i = 0; i < 4; ++i)
                                        rq[i] = -rq[i];
                                }
                                // This one turns in world, so the axis is the bone's
                                // world direction.
                                float bone[3] = { foreDir[0] * foreSign, foreDir[1] * foreSign,
                                    foreDir[2] * foreSign };
                                if (Normalise3(bone)) {
                                    const float alongBone2 = rq[0] * bone[0] + rq[1] * bone[1] + rq[2] * bone[2];
                                    theta = 2.0f * std::atan2(alongBone2, rq[3]);
                                }
                            } else {
                                const float sAx = tq[0] * ax[0] + tq[1] * ax[1] + tq[2] * ax[2];
                                theta = 2.0f * std::atan2(sAx, tq[3]);
                            }
                            {
                                static float s_lastTwist[kArmSlots] = {};
                                static bool s_haveTwist[kArmSlots] = {};
                                if (hand >= 0 && hand < kArmSlots) {
                                    if (s_haveTwist[hand]) {
                                        while (theta - s_lastTwist[hand] > kTau * 0.5f)
                                            theta -= kTau;
                                        while (theta - s_lastTwist[hand] < -kTau * 0.5f)
                                            theta += kTau;
                                    }
                                    // No arm turns past three quarters of a circle.
                                    const float cap = kTau * 0.375f;
                                    if (theta > cap)
                                        theta = cap;
                                    if (theta < -cap)
                                        theta = -cap;
                                    s_lastTwist[hand] = theta;
                                    s_haveTwist[hand] = true;
                                }
                            }
                            const float limit = kForearmTwistLimitDeg * 3.14159265358979f / 180.0f;
                            const float mag = std::fabs(theta);
                            const float sign = theta < 0.0f ? -1.0f : 1.0f;
                            const float within = (mag < limit ? mag : limit) * sign;
                            const float beyond = (mag > limit ? mag - limit : 0.0f) * sign;
                            const float upperTwist = within * kUpperArmTwistShare + beyond;
                            if (std::fabs(upperTwist) > 0.001f) {
                                const float half = upperTwist * 0.5f;
                                const float sn = std::sin(half);
                                float uq[4] = { 0.0f, 0.0f, 0.0f, std::cos(half) };
                                uq[upperAxis] = upperSign * sn;
                                float um[9], turned[9];
                                QuatToMat3(uq, g_quatConvention == 1, um);
                                Mul3(um, wantShoulder, turned);
                                std::memcpy(wantShoulder, turned, sizeof(wantShoulder));
                            }

                            // ---- And the forearm itself (2026-09-19) ----------
                            // The poke test found these: joints 51 and 52, both
                            // hanging off the elbow, one out at the wrist and one
                            // back at the elbow. That is a forearm twist pair - the
                            // distal one carries most of the pronation and the
                            // proximal one a little, which is how the radius really
                            // winds around the ulna.
                            //
                            // They have never been written. Every degree of twist
                            // has been landing on the wrist, and when a wrist is
                            // asked for more than a wrist has, the mesh flips over -
                            // the user's "overturn moments", and their read on it
                            // was right: we were not driving everything else far
                            // enough, because we were not driving it at all.
                            //
                            // Found by shape rather than by index, so Sheva and
                            // anyone else work the same: a child of the elbow that
                            // is not on the path to the hand is a twist bone, and
                            // whichever sits further from the elbow is the distal
                            // one.
                            int twistBone[2] = { -1, -1 };
                            float twistAt[2] = { 0.0f, 0.0f };
                            int twistCount = 0;
                            {
                                float elbowPos[3];
                                if (JointWorldPos(joints, arm.elbow, elbowPos)) {
                                    for (int i = 0; i < count && twistCount < 2; ++i) {
                                        if (i == arm.elbow || parents[i] != arm.elbow)
                                            continue;
                                        bool onPath = false;
                                        for (int j = arm.wrist, g = 0; j >= 0 && j < count && g < 8;
                                             j = parents[j], ++g) {
                                            if (j == i) {
                                                onPath = true;
                                                break;
                                            }
                                            if (parents[j] == j)
                                                break;
                                        }
                                        if (onPath)
                                            continue;
                                        float p[3];
                                        if (!JointWorldPos(joints, i, p))
                                            continue;
                                        float d[3];
                                        Sub3(p, elbowPos, d);
                                        twistBone[twistCount] = i;
                                        twistAt[twistCount] = Length3(d);
                                        ++twistCount;
                                    }
                                }
                            }
                            if (twistCount == 2 && twistAt[0] > twistAt[1]) {
                                const int t = twistBone[0];
                                twistBone[0] = twistBone[1];
                                twistBone[1] = t;
                            }
                            // Whatever the bicep did not take, shared between them:
                            // most of it out at the wrist, the remainder near the
                            // elbow.
                            const float forearmTwist = within * (1.0f - kUpperArmTwistShare);
                            const float shareOf[2] = { 1.0f - kDistalTwistShare, kDistalTwistShare };
                            for (int b = 0; b < twistCount; ++b) {
                                if (twistBone[b] < 0)
                                    continue;
                                const float amount = forearmTwist * shareOf[b];
                                const float h2 = amount * 0.5f;
                                float tw[4] = { 0.0f, 0.0f, 0.0f, std::cos(h2) };
                                tw[foreAxis] = foreSign * std::sin(h2);
                                float tm[9], want[9];
                                QuatToMat3(tw, g_quatConvention == 1, tm);
                                Mul3(tm, wantElbow, want);
                                WriteLocalRotation(joints, twistBone[b], want, wantElbow);
                            }
                        }
                    }
                    // ---- The collarbone (2026-09-19, rebuilt stateless) ----
                    //
                    // Measured, at last. The T-pose puts the user's arms at 0.67
                    // and 0.64 m; in play they reach 0.84 and 0.81 m from the
                    // shoulder it inferred. The same +0.17 m on BOTH arms, which is
                    // not noise and not a bad pose - it is the shoulder itself
                    // travelling when you reach, which nothing here was modelling.
                    // kReachAllowanceMetres has been standing in for it.
                    //
                    // The first attempt at this was a servo: it rotated the
                    // collarbone by a share of the turn from the arm's CURRENT
                    // direction, read back off joints we had written, so it fed on
                    // itself and had to be switched off.
                    //
                    // This one cannot. The collarbone swings from its BIND
                    // direction toward the target, by an amount set by how far out
                    // the arm is being asked to reach. Same inputs, same answer,
                    // however many times it runs - there is no previous frame in it.
                    float shoulderParentAbs[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
                    const int clav = parents[arm.shoulder];
                    const bool haveClav = clav < count && WorldRot(joints, clav, shoulderParentAbs);
                    // Zero until the arm is most of the way out, then eased in. A
                    // shoulder does not travel while you move your hand about in
                    // front of your chest, and letting it would eat the depth -
                    // which is exactly what the first version did.
                    float clavShare = 0.0f;
                    {
                        const float span = l1 + l2;
                        const float frac = span > 0.001f ? wanted / span : 0.0f;
                        float tt = (frac - kClavicleOnset) / (1.0f - kClavicleOnset);
                        tt = tt < 0.0f ? 0.0f : (tt > 1.0f ? 1.0f : tt);
                        clavShare = XrInput_GetSettings().armIkShoulder * tt;
                        if (hand >= 0 && hand < 2) {
                            g_lastReachFrac[hand] = frac;
                            g_lastClavShare[hand] = clavShare;
                        }
                    }
                    if (haveClav && clavShare > 0.001f && g_boneInRow >= 0) {
                        const int rootJoint = RootJoint(parents, count, arm.shoulder);
                        float bindDir[3];
                        float clavPos[3];
                        int clavAxis = 0;
                        float clavSign = 1.0f;
                        if (BindDirWorld(joints, arm.shoulder, rootJoint, bindDir, nullptr)
                            && JointWorldPos(joints, clav, clavPos)
                            && BindAxis(joints, arm.shoulder, &clavAxis, &clavSign)) {
                            float toward[3];
                            Sub3(target, clavPos, toward);
                            if (Normalise3(toward)) {
                                // Part of the way from where the bone rests to where
                                // the hand is asking it to point.
                                float clavDir[3];
                                for (int i = 0; i < 3; ++i)
                                    clavDir[i] = bindDir[i] + (toward[i] - bindDir[i]) * clavShare;
                                if (Normalise3(clavDir)) {
                                    // A COLLARBONE SWINGS, IT DOES NOT TWIST
                                    // (2026-09-25, user: "it's almost like the
                                    // shoulder is over twisting or something and
                                    // isn't staying true to the body").
                                    //
                                    // Passing nullptr here means "take the roll
                                    // from the pole", and the pole is the ELBOW
                                    // preference: down, a little out, a little
                                    // back. That is the right reference for a
                                    // bone whose job is to choose an elbow plane.
                                    // It is the wrong one for a collarbone, which
                                    // has no elbow and no business rolling at all.
                                    //
                                    // So this was rebuilding the collarbone's
                                    // whole orientation every frame and taking its
                                    // twist from a vector that has nothing to do
                                    // with it. Rolling a collarbone rotates the
                                    // entire shoulder mass about the line from the
                                    // sternum to the shoulder, which is exactly
                                    // "over twisting and not staying true to the
                                    // body" - and it is worst with the arm raised,
                                    // because that is when the share is at its
                                    // largest and the pole is nearest useless.
                                    //
                                    // It keeps its own roll now. Only the
                                    // DIRECTION it points is ours; a real one
                                    // elevates and protracts, it does not spin.
                                    float keepRoll[3] = {};
                                    const int clavB = (clavAxis + 1) % 3;
                                    if (g_boneInRow != 0) {
                                        std::memcpy(keepRoll, shoulderParentAbs + clavB * 3, sizeof(keepRoll));
                                    } else {
                                        for (int r = 0; r < 3; ++r)
                                            keepRoll[r] = shoulderParentAbs[r * 3 + clavB];
                                    }
                                    const bool haveRoll = Length3(keepRoll) > 0.5f;
#if RE5VR_DIAGNOSTICS
                                    // How much twist it was taking. If this reads
                                    // near zero the collarbone was never the
                                    // problem and this change is a no-op.
                                    if (haveRoll && hand >= 0 && hand < 2) {
                                        float wasClav[9];
                                        BasisForBone(clavAxis, clavSign, clavDir, poleDir, nullptr, handed,
                                            wasClav);
                                        float oldSecond[3];
                                        if (g_boneInRow != 0) {
                                            std::memcpy(oldSecond, wasClav + clavB * 3, sizeof(oldSecond));
                                        } else {
                                            for (int r = 0; r < 3; ++r)
                                                oldSecond[r] = wasClav[r * 3 + clavB];
                                        }
                                        float dotRoll = Dot3(oldSecond, keepRoll);
                                        dotRoll = dotRoll < -1.0f ? -1.0f : (dotRoll > 1.0f ? 1.0f : dotRoll);
                                        const float rollDeg = std::acos(dotRoll) * 57.2957795f;
                                        static unsigned long long s_toldRoll[2] = {};
                                        const unsigned long long nowRoll = GetTickCount64();
                                        if (nowRoll - s_toldRoll[hand] >= 2000) {
                                            s_toldRoll[hand] = nowRoll;
                                            Log_Printf("ArmIk: the %s collarbone was being rolled %.0f deg off "
                                                       "its own - keeping its roll, swinging it %.2f toward "
                                                       "the hand",
                                                hand == 1 ? "right" : "left", rollDeg, clavShare);
                                        }
                                    }
#endif
                                    float wantClav[9], clavParentAbs[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
                                    BasisForBone(clavAxis, clavSign, clavDir,
                                        haveRoll ? keepRoll : poleDir, haveRoll ? keepRoll : nullptr, handed,
                                        wantClav);
                                    if (parents[clav] < count)
                                        WorldRot(joints, parents[clav], clavParentAbs);
                                    WriteLocalRotation(joints, clav, wantClav, clavParentAbs);
                                    // The shoulder hangs off the collarbone we have
                                    // just decided, not the one it is wearing.
                                    std::memcpy(shoulderParentAbs, wantClav, sizeof(shoulderParentAbs));
                                }
                            }
                        }
                    }
                    WriteLocalRotation(joints, arm.shoulder, wantShoulder, shoulderParentAbs);
                    // The elbow hangs off the shoulder we have just decided, not
                    // off whatever it is wearing now.
                    float elbowParentAbs[9];
                    const int ep = parents[arm.elbow];
                    if (ep == arm.shoulder) {
                        std::memcpy(elbowParentAbs, wantShoulder, sizeof(elbowParentAbs));
                    } else if (ep < count) {
                        // A twist bone in between, carried round by the shoulder.
                        float was[9], shoulderNow[9];
                        if (WorldRot(joints, ep, was) && WorldRot(joints, arm.shoulder, shoulderNow)) {
                            float shoulderNowT[9], rel[9];
                            Transpose3(shoulderNow, shoulderNowT);
                            Mul3(was, shoulderNowT, rel);
                            Mul3(rel, wantShoulder, elbowParentAbs);
                        } else {
                            std::memcpy(elbowParentAbs, wantShoulder, sizeof(elbowParentAbs));
                        }
                    } else {
                        std::memcpy(elbowParentAbs, wantShoulder, sizeof(elbowParentAbs));
                    }
                    WriteLocalRotation(joints, arm.elbow, wantElbow, elbowParentAbs);

                    // And the hand. Everything above this places the wrist; this
                    // turns it, which is what makes the gun point down your
                    // controller rather than wherever the forearm's roll left it.
                    //
                    // The tie between your controller and the hand is taken once,
                    // when the gun comes up: at that moment the animation has the
                    // weapon held properly and you are pointing roughly where it
                    // is, so the difference between the two is the grip. After
                    // that the hand simply follows, and lowering and raising the
                    // gun retakes it - which is the way out if it ever looks bent.
                    // BACK ON (2026-09-18). This was switched off because turning
                    // the hand on top of a forearm that was ALSO being turned by
                    // the controller tore the mesh. Both of those have changed.
                    // The forearm's direction now comes from the solve, so the
                    // hand's turn relative to it is small, and the twist is shared
                    // with the roll bone below.
                    //
                    // And it is the right place for it. Every VR game that does
                    // this properly - Blade and Sorcery, Boneworks, FinalIK's
                    // VRIK - gives the controller's rotation to the HAND and lets
                    // the elbow fall where the solve puts it. We drove the FOREARM
                    // instead, for one reason: the gun hangs off a socket parented
                    // to the elbow, so the forearm looked like the only thing that
                    // could carry the weapon's turn. The socket lock below ended
                    // that - the socket rides the hand now - which is what lets
                    // the hand have its rotation back. Exact position from the
                    // solve, exact rotation from your controller, rigid bones.
                    float handWantWorld[9];
                    bool haveHandWant = false;
                    if (rollFromHand && !g_gripProbe) {
                        float wristParentAbs[9];
                        const int wp = parents[arm.wrist];
                        if (wp == arm.elbow || wp >= count) {
                            std::memcpy(wristParentAbs, wantElbow, sizeof(wristParentAbs));
                        } else {
                            // A twist bone between elbow and wrist, carried round
                            // by the elbow the same way the elbow was by the
                            // shoulder.
                            float was[9], elbowNow[9];
                            if (WorldRot(joints, wp, was) && WorldRot(joints, arm.elbow, elbowNow)) {
                                float elbowNowT[9], rel[9];
                                Transpose3(elbowNow, elbowNowT);
                                Mul3(was, elbowNowT, rel);
                                Mul3(rel, wantElbow, wristParentAbs);
                            } else {
                                std::memcpy(wristParentAbs, wantElbow, sizeof(wristParentAbs));
                            }
                        }
                        // Tied only while the gun is UP (2026-09-18). Taken at
                        // rest it recorded the hand hanging down against a
                        // controller held level, and that ninety degrees then rode
                        // along forever - the wrist bent straight down. With the
                        // gun up the pose is known, and the tie holds afterwards,
                        // so the hand still turns with you at rest.
                        if (hand >= kPlayerSlot && hand < kPlayerSlot + 2 && !g_haveWristOffset[hand]
                            && !g_tiesCalibrated[hand] && AimSettled()) {
                            float wristNow[9], ctrlT[9];
                            if (WorldRot(joints, arm.wrist, wristNow)) {
                                Transpose3(g_ctrlWorld[hand], ctrlT);
                                Mul3(wristNow, ctrlT, g_wristOffset[hand]);
                                g_haveWristOffset[hand] = true;
                            }
                        }
                        if (g_haveWristOffset[hand]) {
                            float wantWrist[9];
                            Mul3(g_wristOffset[hand], g_ctrlWorld[hand], wantWrist);

                            // Share the turn with the roll bone (2026-09-18).
                            // Turning the hand while the joint between it and the
                            // elbow stays put tears the mesh across the forearm -
                            // which is what that joint is there to prevent. Half
                            // the rotation goes to it and the hand keeps the rest,
                            // so the twist is spread the way a real forearm
                            // spreads it instead of happening all at one joint.
                            const int roll = parents[arm.wrist];
                            float handParent[9];
                            std::memcpy(handParent, wristParentAbs, sizeof(handParent));
                            if (roll < count && roll != arm.elbow && g_quatConvention >= 0) {
                                float parentT[9], rel[9], relQ[4];
                                Transpose3(wristParentAbs, parentT);
                                Mul3(wantWrist, parentT, rel);
                                Mat3ToQuat(rel, g_quatConvention == 1, relQ);
                                // TWIST only (2026-09-18). This used to take half
                                // of the hand's whole turn, bend included - and a
                                // roll bone cannot bend. The user found it by feel
                                // before the maths said it: "about a quarter of the
                                // way up the forearm is your hinge point, not the
                                // actual wrist". That is exactly what half a bend
                                // applied partway down the forearm looks like once
                                // the skin is weighted across both bones.
                                //
                                // Split the turn instead: the part about the bone's
                                // own axis is twist and belongs here, everything
                                // else is bend and belongs at the wrist. The wrist
                                // still ends up exactly where your hand is, because
                                // it is written to the full target with this bone
                                // as its parent - so whatever this one does not
                                // take, the wrist takes.
                                // One hemisphere (2026-09-19). A quaternion and its
                                // negative are the same rotation, but they are NOT
                                // the same twist once you project onto an axis: the
                                // projection changes sign with them. So whichever
                                // one the conversion happens to hand back decides
                                // which way the twist appears to go, and the split
                                // behaves differently turning one way than the
                                // other. The user measured it before the maths did:
                                // "a bit of a cutoff when twisting left, unlike
                                // twisting right which gives me the proper range".
                                // Pin it to w >= 0 and both directions match.
                                // TWIST only - and the user put the case better than
                                // the maths does (2026-09-19):
                                //
                                //   "if we're rolling, aka turning a doorknob, that
                                //    should follow along to the forearm. In a motion
                                //    like flapping your hand left and right, this
                                //    doesn't mess with the forearm as it doesn't
                                //    require forearm movement to do so."
                                //
                                // Exactly so. A doorknob is rotation about the
                                // forearm's own axis - pronation, which the forearm
                                // must carry. A flap is the wrist alone and the
                                // forearm has no part in it. Half of the WHOLE turn
                                // feeds both into this bone, so a flap kinks the
                                // forearm, and because the mesh is weighted right up
                                // the arm that kink spreads and reads as the bone
                                // itself bending.
                                //
                                // So take only the part about the bone's axis. The
                                // wrist still lands exactly where your hand is - it
                                // is written to the full target with this bone as
                                // its parent, so whatever this one leaves, it takes.
                                //
                                // Tried once before bundled with a reach change and
                                // reverted when the gun came loose; there was no way
                                // to tell which of the two did it. This time it goes
                                // alone.
                                if (relQ[3] < 0.0f) {
                                    for (int i = 0; i < 4; ++i)
                                        relQ[i] = -relQ[i];
                                }
                                float rollAxis[3] = { 0.0f, 0.0f, 0.0f };
                                rollAxis[foreAxis] = foreSign;
                                const float rollAlong = relQ[0] * rollAxis[0] + relQ[1] * rollAxis[1]
                                    + relQ[2] * rollAxis[2];
                                (void)rollAlong;
                                // REVERTED AGAIN 2026-09-19 - and this time it told
                                // us something. Twist-only here breaks the gun AND
                                // leaves the visible bend exactly as it was. So this
                                // bone is NOT what bends the forearm on screen, but
                                // it IS something the weapon depends on. Both halves
                                // of that are news, and both say we have been
                                // writing to the wrong joint.
                                float halfQ[4] = { relQ[0] * 0.5f, relQ[1] * 0.5f, relQ[2] * 0.5f,
                                    (relQ[3] + 1.0f) * 0.5f };
                                const float hl = std::sqrt(halfQ[0] * halfQ[0] + halfQ[1] * halfQ[1]
                                    + halfQ[2] * halfQ[2] + halfQ[3] * halfQ[3]);
                                if (hl > 0.1f) {
                                    for (int i = 0; i < 4; ++i)
                                        halfQ[i] /= hl;
                                    float halfM[9], rollWant[9];
                                    QuatToMat3(halfQ, g_quatConvention == 1, halfM);
                                    Mul3(halfM, wristParentAbs, rollWant);
                                    // The roll bone hangs off the elbow, so its
                                    // own parent is whatever the elbow now is.
                                    float rollParent[9];
                                    const int rp = parents[roll];
                                    if (rp == arm.elbow || rp >= count)
                                        std::memcpy(rollParent, wantElbow, sizeof(rollParent));
                                    else
                                        std::memcpy(rollParent, wristParentAbs, sizeof(rollParent));
                                    WriteLocalRotation(joints, roll, rollWant, rollParent);
                                    std::memcpy(handParent, rollWant, sizeof(handParent));
                                }
                            }
#if RE5VR_DIAGNOSTICS
                            // "Neither wrist moves" with the write landing every
                            // frame and nothing failing verification means the
                            // number we are asking for is wrong, not the writing of
                            // it. These are the two ends of that: the controller
                            // going in, and the hand orientation coming out. If the
                            // controller row moves and the wanted row does not, the
                            // tie is killing it. If both move and the hand still
                            // does not, the write is being undone downstream.
                            {
                                static unsigned long long s_ms[2] = {};
                                const unsigned long long now2 = GetTickCount64();
                                if (hand >= 0 && hand < 2 && now2 - s_ms[hand] >= 1000) {
                                    s_ms[hand] = now2;
                                    float liveWrist[9];
                                    const bool haveLive = WorldRot(joints, arm.wrist, liveWrist);
                                    Log_Printf("ArmIk: %s hand - controller row0 (%.2f %.2f %.2f), wanted row0 "
                                               "(%.2f %.2f %.2f), joint row0 (%.2f %.2f %.2f)",
                                        hand == 1 ? "right" : "left", g_ctrlWorld[hand][0], g_ctrlWorld[hand][1],
                                        g_ctrlWorld[hand][2], wantWrist[0], wantWrist[1], wantWrist[2],
                                        haveLive ? liveWrist[0] : 0.0f, haveLive ? liveWrist[1] : 0.0f,
                                        haveLive ? liveWrist[2] : 0.0f);
                                }
                            }
#endif
                            WriteLocalRotation(joints, arm.wrist, wantWrist, handParent);
                            // The socket is not written here any more: it hangs off
                            // the ELBOW, not the hand, so giving it the hand's
                            // rotation outright was only ever right by accident.
                            // The lock below carries it at the angle it really
                            // sits in the hand, measured with the gun down.
                            std::memcpy(handWantWorld, wantWrist, sizeof(handWantWorld));
                            haveHandWant = true;
                        }
                    }
                    // ---- Keeping the gun in the hand (2026-09-18) ----------
                    // A held weapon does not hang off the hand. It hangs off a
                    // SOCKET joint that sits exactly where the wrist is but is
                    // parented to the ELBOW - joint 51, id 75 on this skeleton,
                    // and the log confirms it: "0.0 from the wrist, NOT under it".
                    //
                    // So the socket is our elbow's child, and its own local
                    // rotation is still the animation's. With the gun down that
                    // local was authored for a forearm close to where we have put
                    // ours, and the gun looks held. Raise the gun and the aim
                    // layer writes a local computed for ITS forearm, which is not
                    // ours, and the hand and the socket stop agreeing - the gun
                    // leaves the hand. Only while aiming, which is exactly what
                    // the user reports.
                    //
                    // The fix is not to fight the aim layer for that joint but to
                    // state the relationship it is breaking: the gun sits in the
                    // hand at a fixed angle. Measure that angle while the gun is
                    // down, where the user confirms it is right, then hold it. The
                    // socket follows the hand wherever the hand goes, so the
                    // weapon is carried by the hand even though the skeleton does
                    // not parent it there.
                    if (hand >= 0 && hand < 2 && !g_gripProbe) {
                        const int socket = g_socket[hand];
                        float handNow[9], elbowNow[9];
                        if (socket >= 0 && socket < count && WorldRot(joints, arm.wrist, handNow)
                            && WorldRot(joints, arm.elbow, elbowNow)) {
                            float socketNow[9];
                            if (WorldRot(joints, socket, socketNow)) {
                                // Measured ONCE, not every frame (2026-09-18).
                                // Re-measuring it continuously reads back a socket
                                // we had just written and folds our own output into
                                // the next answer - the same feedback that spun the
                                // forearm and the shoulder. Taken once while the
                                // gun is down it is a constant, which is what it
                                // is meant to be: the angle a weapon sits at in a
                                // hand does not change while you hold it.
                                // Identity, from the bind pose (2026-09-19). The
                                // measured version compared a socket we had written
                                // against a hand we had also written, a frame
                                // stale - two of our own outputs, which is the
                                // feedback this file keeps rediscovering, and the
                                // gun came out misaligned with the hand.
                                //
                                // The bind dump settles what the angle should be
                                // without measuring anything: socket 51 and hand 54
                                // sit at the same place, both 0.0 from the wrist,
                                // with identity local rotations. So in the pose the
                                // rig was authored in the socket and the hand agree
                                // exactly, and the socket simply wears the hand's
                                // rotation.
                                // ONCE (2026-09-19). Measuring this every frame
                                // reads back a socket we wrote last frame and a
                                // hand we wrote last frame, and folds both into the
                                // next answer - so it walks, a little per frame,
                                // for as long as it keeps measuring.
                                //
                                // And it only measures while the gun is DOWN, which
                                // is why the user could stop the spin by aiming:
                                // raising the gun stops the measuring, so the drift
                                // stops with it. That symptom points at this line
                                // and nothing else in the solve is gated on aiming.
                                //
                                // The angle a weapon sits at in a hand is a
                                // constant, so take it once and keep it. A new
                                // weapon clears it and it is taken again.
                                if (!g_aimingNow && !g_haveSocketTie[hand]) {
                                    float handNowT[9];
                                    Transpose3(handNow, handNowT);
                                    Mul3(socketNow, handNowT, g_socketTie[hand]);
                                    g_haveSocketTie[hand] = true;
                                }
                                if (g_haveSocketTie[hand]) {
                                    // Where the hand ends up once our elbow is in:
                                    // its turn relative to the elbow is the
                                    // animation's and we are not touching it.
                                    float elbowNowT[9], handRel[9], handWant[9], wantSocket[9];
                                    if (haveHandWant) {
                                        // The hand has just been given your
                                        // controller's turn, so THAT is what the
                                        // gun has to sit in - not the animation's
                                        // hand, which is a frame and a pose out.
                                        std::memcpy(handWant, handWantWorld, sizeof(handWant));
                                    } else {
                                        Transpose3(elbowNow, elbowNowT);
                                        Mul3(handNow, elbowNowT, handRel);
                                        Mul3(handRel, wantElbow, handWant);
                                    }
                                    Mul3(g_socketTie[hand], handWant, wantSocket);
                                    // The socket hangs off the elbow, so its parent
                                    // is the elbow we have just decided.
                                    float socketParent[9];
                                    const int sp = parents[socket];
                                    if (sp == arm.elbow || sp >= count)
                                        std::memcpy(socketParent, wantElbow, sizeof(socketParent));
                                    else
                                        std::memcpy(socketParent, handWant, sizeof(socketParent));
                                    // NOT WRITTEN any more (2026-09-19). The poke
                                    // test renamed this joint: 51 and 52 drive the
                                    // FOREARM, 53 and 54 the wrist - and the gun
                                    // follows 53 and 54, not this. So it was never
                                    // the weapon socket.
                                    //
                                    // Which makes this write actively harmful: it
                                    // was putting the HAND's rotation onto a forearm
                                    // bone, forcing the forearm to copy your hand
                                    // outright. That is the user's "flapping my hand
                                    // shouldn't move the forearm", and it bought
                                    // nothing, because the weapon hangs off the
                                    // wrist chain we already drive.
                                    (void)wantSocket;
                                    (void)socketParent;
#if RE5VR_DIAGNOSTICS
                                    // The gun still reads as floating away from the
                                    // hand while the socket's pose is held at 0.000
                                    // drift, so the question is no longer whether
                                    // the write lands. Either the socket is not
                                    // WHERE the hand is - it hangs off the elbow, so
                                    // its position comes from a different path - or
                                    // the gun is not on this joint at all.
                                    {
                                        static unsigned long long s_sm[2] = {};
                                        const unsigned long long n3 = GetTickCount64();
                                        if (n3 - s_sm[hand] >= 1000) {
                                            s_sm[hand] = n3;
                                            float hp[3], sp2[3];
                                            if (JointWorldPos(joints, arm.wrist, hp)
                                                && JointWorldPos(joints, socket, sp2)) {
                                                float d[3];
                                                Sub3(sp2, hp, d);
                                                Log_Printf("ArmIk: %s socket %d is %.1f units from the hand "
                                                           "(%.0f %.0f %.0f) vs (%.0f %.0f %.0f)",
                                                    hand == 1 ? "right" : "left", socket, Length3(d), sp2[0],
                                                    sp2[1], sp2[2], hp[0], hp[1], hp[2]);
                                            }
                                        }
                                    }
#endif
                                }
                            }
                        }
                    }


                    ApplyWorldRotations(joints, parents, count, arm, r1, r2, shoulder);
                    return true;
                }
            }
        }

        // The elbow's parent is not always the shoulder: FindArms walks past
        // zero-length twist bones to find a joint with real length, so there is
        // often a roll joint in between. Whatever it is, it sits under the
        // shoulder and so has already turned by r1.
        float elbowParentW[9];
        const int elbowParent = parents[arm.elbow];
        if (elbowParent == arm.shoulder) {
            std::memcpy(elbowParentW, shoulderWant, sizeof(elbowParentW));
        } else if (elbowParent < count) {
            float was[9];
            if (!WorldRot(joints, elbowParent, was))
                return false;
            RotBasis(r1, was, elbowParentW);
        } else {
            std::memcpy(elbowParentW, shoulderWant, sizeof(elbowParentW));
        }

        WriteLocalRotation(joints, arm.shoulder, shoulderWant, shoulderParentW);
        WriteLocalRotation(joints, arm.elbow, elbowWant, elbowParentW);
#if RE5VR_DIAGNOSTICS
        // Both joints came out holding the identical quaternion, which two
        // different bones cannot. Print what goes in rather than infer it from
        // what comes back out.
        {
            static unsigned long long s_ms = 0;
            const unsigned long long now = GetTickCount64();
            if (now - s_ms >= 500) {
                s_ms = now;
                Log_Printf("ArmIk: shoulder %d (parent %d) <- (%.3f %.3f %.3f %.3f); elbow %d "
                           "(parent %d) <- (%.3f %.3f %.3f %.3f)",
                    arm.shoulder, static_cast<int>(parents[arm.shoulder]),
                    g_poseQuat[g_solvingBody][arm.shoulder][0], g_poseQuat[g_solvingBody][arm.shoulder][1],
                    g_poseQuat[g_solvingBody][arm.shoulder][2], g_poseQuat[g_solvingBody][arm.shoulder][3],
                    arm.elbow, elbowParent, g_poseQuat[g_solvingBody][arm.elbow][0],
                    g_poseQuat[g_solvingBody][arm.elbow][1], g_poseQuat[g_solvingBody][arm.elbow][2],
                    g_poseQuat[g_solvingBody][arm.elbow][3]);
            }
        }
#endif
        return true;
    }

    // Or the result, for when the pose cannot be reached in time: moves the arm
    // on screen and nothing else, because everything else recomposes.
    int subtree[kMaxJoints];
    {
        const int n = Subtree(parents, count, arm.shoulder, subtree);
        for (int i = 0; i < n; ++i)
            RotateJointWorld(joints, subtree[i], r1, shoulder);
    }
    float elbowNow[3];
    if (!JointWorldPos(joints, arm.elbow, elbowNow))
        return false;
    {
        const int n = Subtree(parents, count, arm.elbow, subtree);
        for (int i = 0; i < n; ++i)
            RotateJointWorld(joints, subtree[i], r2, elbowNow);
    }
    return true;
}

ArmIkBody g_body;
bool g_haveBody = false;
unsigned long long g_bodyMs = 0;

bool g_seen[kMaxJoints] = {};
int g_seenCount = 0;
// The joints the solve actually needs built before it can run: both arms and
// everything hanging off them, fingers included.
bool g_needed[kMaxJoints] = {};
int g_neededTotal = 0;
int g_neededSeen = 0;
// How many of them a frame ACTUALLY rebuilds. Three of the fifty-six never
// are, so waiting for all of them waits forever; waiting for as many as the
// last frame managed lands on the final one every time (2026-09-18).
int g_neededPerFrame = 0;
// The six joints the solve actually reads: both shoulders, elbows and wrists.
// Counting arm joints was not enough (2026-09-18) - a frame can reach the
// count with these three still holding last frame's matrices, and reading half
// a skeleton measures bones that are not there. The arm's own length came back
// as anything from 39 to 73 against a true 53.2, and every one of those went
// into the next frame's pose.
int g_key[6] = { -1, -1, -1, -1, -1, -1 };
bool g_keySeen[6] = {};
// Solve on the callback AFTER the last one we need, never on it (2026-09-18).
// The build hook sits one instruction past the store of the world matrix's
// FIRST float, so the joint that completes the set still has fifteen of its
// sixteen floats holding last frame's values. Reading it measured an arm that
// was 45 units long one frame and 73 the next, and every one of those went
// into the pose that came back round again.
bool g_solvePending = false;
// What the arm measures when the reading is trustworthy, so an untrustworthy
// one can be recognised and skipped rather than acted on.
float g_restReach[kArmSlots] = {};
// The lengths of the shoulder's and elbow's world matrix rows, plus their own
// local scale. A rotation matrix has rows of length 1; anything else IS the
// scaling, and this says whether it arrives through the matrix or through the
// scale field the game keeps separately.
float g_armScale[8] = {};
// What the gun hand last asked for, so a pose that will not move can be told
// apart from a target that is not moving.
float g_lastLocal[3] = {}, g_lastTarget[3] = {};
bool g_solvedThisFrame = false;
unsigned char* g_neededFor = nullptr;
unsigned long long g_lateSolveMs = 0;
bool g_skeletonHooked = false;
// Counted in every build: the number of joint rotations actually written
// back by ArmIk_OnPoseComplete. If this is zero while 6DOF is on, nothing
// downstream of it can possibly work, and everything downstream is where the
// hunting has been going (2026-09-29).
unsigned long g_poseWrites = 0;
// Where ArmIk_OnPoseComplete stops, counted separately, because "zero pose
// writes" has four possible causes and reading the code has not separated
// them (2026-09-29).
unsigned long g_poseCalls = 0;      // the stub reached us at all
unsigned long g_poseNoJoint = 0;    // null joint
unsigned long g_poseNotOurs = 0;    // the joint belongs to no body we drive
unsigned long g_poseNoCache = 0;    // no solved rotation for it
unsigned long g_poseNotLocal = 0;   // writeLocal is off for the player
unsigned long g_poseStale = 0;      // the solve has not run for half a second
unsigned long g_lateSolves = 0, g_fallbackSolves = 0, g_badReads = 0, g_renormalised = 0;
int g_neededSeenHigh = 0;

// The solve itself. Called either straight from the camera hook, or - once the
// skeleton hook is running - at the end of the frame's skeleton build.
// While the gun-writer watch holds the debug registers (see ArmIk_SteadyGun).
unsigned long long g_gunFinderUntilMs = 0;

// The gun hand's jitter while firing (2026-09-22). The user's clip shows the
// gun and both gloves jumping together with the room steady - the hand as a
// unit, not the gun in it. Either its TARGET jumps (the shoulder it is
// measured from rides the spine the firing animation rocks) or the arm fails
// to reach the target on some frames. These say which.
struct HandJitter {
    float prevAnchor[3], prevLocal[3], prevTarget[3];
    bool have;
    float sum[2][5]; // [idle, firing][anchor, controller, target, short of target, lost since last frame] cm
    float max[2][5];
    int n[2];
};
HandJitter g_handJitter = {};

// What the solve left the gun wrist as, for the end-of-frame comparison.
float g_solvedGunPos[3] = {}, g_solvedGunRot[9] = {};
int g_solvedGunJoint = -1;
float g_solvedUpm = 0.0f;
volatile unsigned long long g_solvedGunMs = 0;

// The gun's barrel in the gun controller's axes, from the last player solve.
float g_barrelCtrl[3] = {};
volatile unsigned long long g_barrelMs = 0;

// ---- Holding the WHOLE body through a burst (2026-09-23) ----------------
// Measured: the solve reaches its target every single frame (0.00 short) and
// the pose is then lost by 1.4 to 7 cm a frame while firing, against 0.04 when
// not. The hand is where we put it; what moves is what the hand hangs from.
// Holding the collarbone and the spine was not enough, so this holds the lot:
// every joint of the character is pinned to the pose it had when the burst
// began. The arms are the exception that needs no exception - the solve writes
// them again after this runs every frame, so they still follow your hands.
//
// If the gun still shakes with the entire body frozen, nothing on this
// skeleton is moving it and the cause is somewhere else entirely, which is
// worth knowing in one test rather than five.
// Holding the body by its WORLD matrices (2026-09-23). Freezing the local
// rotations held 130 of 130 joints and the user still watched Chris rock on
// every shot, so the animation's motion is not arriving through the locals we
// can reach. The mod already knows where it always shows: the world matrices,
// written at the end of the frame, which "nothing recomposes afterwards".
//
// Each joint is captured RELATIVE TO THE ROOT, so walking, turning and being
// shoved still move him; it is only the animation on top that is pinned.
void HoldBodyWhileFiring(const ArmIkBody& body, int count, bool firing)
{
    static float s_rel[kMaxJoints][12]; // rows 0-2 and the position, in the root's frame
    static bool s_frozen[kMaxJoints];
    static int s_count = 0;
    static bool s_holding = false;

    if (!firing) {
        if (s_holding) {
            s_holding = false;
            s_count = 0;
            Log_Printf("ArmIk: burst over - the body follows its animation again");
        }
        return;
    }

    // The root's own frame this frame: rotation rows and position.
    float rootRot[9], rootPos[3];
    if (!WorldRot(body.joints, 0, rootRot) || !JointWorldPos(body.joints, 0, rootPos))
        return;
    float rootT[9];
    Transpose3(rootRot, rootT);

    if (!s_holding) {
        s_count = count < kMaxJoints ? count : kMaxJoints;
        int taken = 0;
        for (int i = 0; i < s_count; ++i) {
            s_frozen[i] = false;
            float rot[9], pos[3];
            if (!WorldRot(body.joints, i, rot) || !JointWorldPos(body.joints, i, pos))
                continue;
            // Into the root's frame: rows x root^T, and the offset read off
            // against the root's axes.
            float rel[9];
            Mul3(rot, rootT, rel);
            float d[3];
            Sub3(pos, rootPos, d);
            std::memcpy(s_rel[i], rel, sizeof(rel));
            for (int r = 0; r < 3; ++r)
                s_rel[i][9 + r] = Dot3(rootRot + r * 3, d);
            s_frozen[i] = true;
            ++taken;
        }
        s_holding = taken > 0;
        if (s_holding)
            Log_Printf("ArmIk: holding the whole body through this burst - %d of %d joints, by world matrix", taken,
                s_count);
    }

    // Back out of the root's frame and into the world, every frame.
    for (int i = 0; i < s_count && i < kMaxJoints; ++i) {
        if (!s_frozen[i])
            continue;
        float rows[9];
        Mul3(s_rel[i], rootRot, rows);
        float pos[3];
        for (int k = 0; k < 3; ++k) {
            pos[k] = rootPos[k] + rootRot[k] * s_rel[i][9] + rootRot[3 + k] * s_rel[i][10]
                + rootRot[6 + k] * s_rel[i][11];
        }
        // The matrix as the game keeps it: three rows of four, then the
        // position. Only the parts we know are touched.
        float m[16];
        if (!TryRead(m, body.joints + i * kJointStride + kOffJointWorldMatrix, sizeof(m)))
            continue;
        for (int r = 0; r < 3; ++r) {
            const float len = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
            const float k = len > 0.01f ? len : 1.0f; // keep whatever scale the joint had
            m[r * 4] = rows[r * 3] * k;
            m[r * 4 + 1] = rows[r * 3 + 1] * k;
            m[r * 4 + 2] = rows[r * 3 + 2] * k;
        }
        m[12] = pos[0];
        m[13] = pos[1];
        m[14] = pos[2];
        TryWrite(body.joints + i * kJointStride + kOffJointWorldMatrix, m, sizeof(m));
        TryWrite(body.joints + i * kJointStride + kOffJointWorldPos, pos, sizeof(pos));
    }
#if RE5VR_DIAGNOSTICS
    {
        static unsigned long long s_toldMs = 0;
        const unsigned long long nowF = GetTickCount64();
        if (nowF - s_toldMs >= 1000) {
            s_toldMs = nowF;
            Log_Printf("ArmIk: frozen body - %d joint(s) written by world matrix this frame", s_count);
        }
    }
#endif
}

void RunSolve(const ArmIkBody& body, const IkSyncHands* remote = nullptr)
{
    ++g_solveRuns;
    // Which set of slots this solve owns, and whose pose cache it fills.
    const int slotBase = remote ? kPartnerSlot : kPlayerSlot;
    g_solvingBody = remote ? kPartnerBody : kPlayerBody;
    g_solvingJoints = body.joints;
#if RE5VR_DIAGNOSTICS
    // Which threads solve, and how many of them. If this ever names more than
    // one, the shared-global version of this was being trampled and that was
    // the bug; if it only ever names one, the crossing came from somewhere else
    // and the next log says where.
    {
        static unsigned long s_solveThreads[4] = {};
        static int s_solveThreadCount = 0;
        const unsigned long tid = GetCurrentThreadId();
        bool known = false;
        for (int t = 0; t < s_solveThreadCount; ++t)
            known = known || s_solveThreads[t] == tid;
        if (!known && s_solveThreadCount < 4) {
            s_solveThreads[s_solveThreadCount++] = tid;
            Log_Printf("ArmIk: the solve is running on thread %lu (%d thread(s) so far) - %s", tid,
                s_solveThreadCount, remote ? "their body" : "your body");
        }
    }
#endif
    const XrInputSettings settings = XrInput_GetSettings();
    // armIk is about YOUR hands driving YOUR arms, so it gates the player's
    // solve and not the partner's. Somebody on a pad, with no controllers and
    // no reason to turn 6DOF on, still gets to watch a VR partner's arms move.
    if (!remote && !settings.armIk && !settings.armIkTest)
        return;
    if (!body.joints || !body.character || body.jointCount < 8 || !body.left.valid || !body.right.valid) {
        if (remote)
            g_partnerWhy = "their body or arms went missing between publishing and solving";
        return;
    }

    // Not while the game is running its own camera: a cutscene poses the arms
    // deliberately, and a vault or a stomp is an animation nobody wants a
    // controller interrupting.
    if (CameraRigHook_InScriptedCamera()) {
        if (remote)
            g_partnerWhy = "the game has taken the camera";
        return;
    }

    unsigned char parents[kMaxJoints];
    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    for (int i = 0; i < count; ++i) {
        unsigned char links[4] = {};
        if (!TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links))) {
            if (remote)
                g_partnerWhy = "their joint links could not be read";
            return;
        }
        parents[i] = links[1];
    }

    // The character's own axes. Row 0 of the transform at +0x60 is across the
    // body, not along it - the correction the head lock already needed - and it
    // points LEFT, not right (2026-09-17). Nothing before this had a reason to
    // care which way across: the head lock only ever uses the forward axis
    // derived from it, which comes out the same either way. Hands care. Taken
    // as right, the arm search called the left hand the right one and then sent
    // the right controller's target to the character's left, which is one sign
    // showing up as two complaints: the wrong hand moves, and it moves the
    // wrong way.
    float row0[4] = {};
    if (!TryRead(row0, body.character + kOffBodyTransform, sizeof(row0))) {
        if (remote)
            g_partnerWhy = "their body transform could not be read";
        return;
    }
    float across[3] = { row0[0], 0.0f, row0[2] };
    if (!Normalise3(across))
        return;
    float right[3] = { -across[0], 0.0f, -across[2] };
    // Unchanged, and deliberately still built from row 0 as it lies: this is
    // the direction the head lock confirmed the character faces.
    float forward[3] = { -across[2], 0.0f, across[0] };
    const float up[3] = { 0.0f, 1.0f, 0.0f };

    // The facing, steadied while firing (2026-09-22) - the player's only.
    // Everything the hands do is laid onto these axes, rotation included, so a
    // body the game nudges on every shot swings the whole gun round the wrist.
    // During a burst they follow the real facing through an exponential
    // low-pass - the firing-kick share is the fraction taken each frame, so
    // jitter is soaked up and a real turn still comes through, a little late.
    // After the burst they ease back over a few frames rather than snap.
    // Measured either way, so the log says whether the body jitters at all.
    if (!remote) {
        const float yaw = std::atan2(forward[0], forward[2]);
        static float s_steadyYaw = 0.0f, s_prevYaw = 0.0f, s_prevCtrlYaw = 0.0f;
        static bool s_haveYaw = false, s_haveCtrlYaw = false;
        const auto wrap = [](float a) {
            while (a > 3.14159265f)
                a -= 6.28318531f;
            while (a < -3.14159265f)
                a += 6.28318531f;
            return a;
        };
        const unsigned long long nowFacing = GetTickCount64();
        const unsigned long long shot = XrInput_LastShotMs();
        const bool kicking = shot && nowFacing - shot < 300;
#if RE5VR_DIAGNOSTICS
        {
            static float s_sum[2][2], s_max[2][2];
            static int s_n[2];
            static unsigned long long s_toldMs = 0;
            // And where the body stands: the minigun was seen shoving the character's
            // root ten centimetres a frame, which moves the shoulders the hands hang from.
            static float s_prevPos[3];
            static bool s_havePos = false;
            static float s_posSum[2], s_posMax[2];
            float posNow[3] = {};
            const bool havePos = TryRead(posNow, body.character + kOffBodyTransform + 0x30, sizeof(posNow));
            float ctrlYawDeg = 0.0f, ctrlPitchDeg = 0.0f;
            const bool haveCtrl = XrInput_GetGunAim(&ctrlYawDeg, &ctrlPitchDeg);
            if (s_haveYaw && haveCtrl && s_haveCtrlYaw) {
                const int k = kicking ? 1 : 0;
                const float bodyDeg = std::fabs(wrap(yaw - s_prevYaw)) * 57.2957795f;
                float c = ctrlYawDeg - s_prevCtrlYaw;
                while (c > 180.0f)
                    c -= 360.0f;
                while (c < -180.0f)
                    c += 360.0f;
                const float ctrlDeg = std::fabs(c);
                s_sum[k][0] += bodyDeg;
                s_sum[k][1] += ctrlDeg;
                if (bodyDeg > s_max[k][0])
                    s_max[k][0] = bodyDeg;
                if (ctrlDeg > s_max[k][1])
                    s_max[k][1] = ctrlDeg;
                if (havePos && s_havePos) {
                    float dp[3];
                    Sub3(posNow, s_prevPos, dp);
                    const float cm = Length3(dp) * 100.0f / (g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f);
                    s_posSum[k] += cm;
                    if (cm > s_posMax[k])
                        s_posMax[k] = cm;
                }
                ++s_n[k];
            }
            if (havePos)
                std::memcpy(s_prevPos, posNow, sizeof(s_prevPos));
            s_havePos = havePos;
            s_prevCtrlYaw = ctrlYawDeg;
            s_haveCtrlYaw = haveCtrl;
            if (s_n[1] > 5 && nowFacing - s_toldMs >= 3000) {
                s_toldMs = nowFacing;
                Log_Printf("Recoil: while firing the body's facing turned avg %.2f max %.2f deg a frame and your "
                           "controller avg %.2f max %.2f; not firing, body %.2f / %.2f, controller %.2f / %.2f "
                           "; the body moved avg %.1f max %.1f cm a frame firing, %.1f / %.1f not (%d/%d frames, hands "
                    "steadied: %s)",
                    s_sum[1][0] / s_n[1], s_max[1][0], s_sum[1][1] / s_n[1], s_max[1][1],
                    s_n[0] ? s_sum[0][0] / s_n[0] : 0.0f, s_max[0][0], s_n[0] ? s_sum[0][1] / s_n[0] : 0.0f, s_max[0][1],
                    s_posSum[1] / s_n[1], s_posMax[1], s_n[0] ? s_posSum[0] / s_n[0] : 0.0f, s_posMax[0], s_n[1], s_n[0],
                    settings.steadyHandsFiring ? "yes" : "no");
                std::memset(s_posSum, 0, sizeof(s_posSum));
                std::memset(s_posMax, 0, sizeof(s_posMax));
                std::memset(s_sum, 0, sizeof(s_sum));
                std::memset(s_max, 0, sizeof(s_max));
                s_n[0] = s_n[1] = 0;
            }
        }
#endif
        if (!s_haveYaw || !settings.steadyHandsFiring) {
            s_steadyYaw = yaw;
        } else {
            float share = XrInput_GetTwoHand(nullptr) ? settings.kickTwoHanded : settings.kickOneHanded;
            share = share < 0.05f ? 0.05f : (share > 1.0f ? 1.0f : share);
            const float alpha = kicking ? share : 0.3f;
            s_steadyYaw = wrap(s_steadyYaw + wrap(yaw - s_steadyYaw) * alpha);
            const float fx = std::sin(s_steadyYaw), fz = std::cos(s_steadyYaw);
            forward[0] = fx;
            forward[2] = fz;
            right[0] = -fz;
            right[2] = fx;
        }
        s_prevYaw = yaw;
        s_haveYaw = true;
    }
    std::memcpy(g_charRight, right, sizeof(g_charRight));
    std::memcpy(g_charUp, up, sizeof(g_charUp));
    std::memcpy(g_charForward, forward, sizeof(g_charForward));

    // How big a metre is here, taken from this character's own arm.
    float sPos[3], ePos[3], wPos[3];
    if (!JointWorldPos(body.joints, body.right.shoulder, sPos) || !JointWorldPos(body.joints, body.right.elbow, ePos)
        || !JointWorldPos(body.joints, body.right.wrist, wPos))
        return;
    float uv[3], fv[3];
    Sub3(ePos, sPos, uv);
    Sub3(wPos, ePos, fv);
    float armUnits = Length3(uv) + Length3(fv);
    if (!(armUnits > 5.0f))
        return;
    // Bones do not change length. If this frame says they have, the skeleton is
    // caught mid-update and every number taken from it is fiction - including
    // the pose that would go back in and be measured again next frame.
    // One length remembered for YOU and another for your partner (2026-09-23,
    // user: "I had the IK aiming bug just now - loaded into Mercenaries
    // mode"). There was one, shared. Chris measures 53.2 units and Sheva does
    // not, so with a partner on screen the two bodies overwrote each other's
    // answer frame by frame and every frame after that read as a skeleton
    // caught mid-update. The log is unmistakable: "measured 68.2 against
    // 57.0", then "65.5 against 55.8", then "64.1 against 57.8" - the thing
    // it was being measured against was moving as much as the measurement.
    // Nothing was ever posed again, which is the aiming bug.
    const int reachSlot = remote ? 0 : 1;
    // And it has to be taken from an arm NOBODY is posing (2026-09-23). The
    // loopback test caught this cleanly: the partner's arm came back 48.9
    // units one moment and 59.5 the next, twenty one percent apart, so every
    // frame failed the ten percent test and the animation took her arm - and
    // then the two second escape hatch cleared the remembered length and
    // re-learned it from an arm that had just been posed, which is how it
    // healed twice in six seconds and fixed nothing.
    //
    // So the escape hatch now takes its hands off first. For half a second
    // after it fires, this arm is left entirely to the game, and the length is
    // read at the end of that - from the animation, which is the only thing
    // that can be trusted to keep a bone the length it is.
    // A partner's arm is never thrown away over this (2026-09-23). The guard
    // exists because a skeleton caught mid-update gives fiction, and for your
    // own arm that is worth a skipped frame. For a partner it was costing
    // everything: their log shows the writer handed 164 joints and writing all
    // 164 of them, and the arms still only moved on 2 frames in 120, because
    // this guard is the one early return in the whole solve that sets no reason
    // - which is why it hid behind "the solve ran but wrote no pose".
    //
    // And it was feeding itself. We write the pose, the next frame this
    // measures an arm that now has our pose in it, calls the bones changed,
    // throws the frame away, the animation takes the arm back, the measurement
    // looks right again, we write, and round it goes. That oscillation IS the
    // "huge and going crazy": not a stretched arm, an arm being handed back and
    // forth between two poses a hundred times a second.
    //
    // So for a partner the remembered rest length simply stands in. It is a
    // property of the character, the same character on both machines, and it
    // was measured with nothing posing it. A slightly stale length costs a
    // little reach; not solving at all costs the whole feature.
    if (remote && g_restReach[reachSlot] > 1.0f
        && std::fabs(armUnits - g_restReach[reachSlot]) > g_restReach[reachSlot] * 0.10f) {
        static unsigned long long s_toldStand = 0;
        const unsigned long long nowStand = GetTickCount64();
        if (nowStand - s_toldStand >= 3000) {
            s_toldStand = nowStand;
            Log_Printf("ArmIk: the partner's arm read %.1f units this frame against a resting %.1f - using the "
                       "resting one and carrying on rather than dropping the frame",
                armUnits, g_restReach[reachSlot]);
        }
        armUnits = g_restReach[reachSlot];
    }

    // A LENGTH HAS TO HOLD STILL BEFORE IT IS BELIEVED (2026-09-25, user: "my
    // aiming broke after moving through the transition").
    //
    // The log caught it in the act:
    //
    //   42.637  rests at 53.2 - a different arm, so the skeleton is looked for again
    //   42.658  Arms: asked to look again - dropping the cached arm joints
    //   42.679  skipped a frame - measured 53.2 against 87.2
    //   43.684  skipped a frame - measured 53.2 against 87.2
    //   44.691  skipped a frame - measured 53.2 against 87.2
    //
    // Eighty-seven point two. Forgetting the skeleton resets the length to
    // nothing, and the very next frame took whatever the arm happened to
    // measure and wrote it down as the truth - on a frame during a transition,
    // with the skeleton half rebuilt. From then on every honest 53.2 read as
    // wrong, every frame was thrown away, and the aim stayed broken until
    // another two-second heal happened to catch it.
    //
    // One unexamined assignment, and it was the load-bearing one: a measurement
    // taken once is a guess. A real arm does not change length, so the number
    // that IS the arm is the one that keeps coming back the same. Eight frames
    // agreeing inside two per cent is about eighty milliseconds, too quick to
    // notice and far too steady for a skeleton that is still being built.
    //
    // It also explains the heal that misfired a line earlier: the length it
    // compared against was 57.2, already poisoned by an earlier snap judgement,
    // so a perfectly good 53.2 looked like a different arm.
    // WHO ZEROED IT (2026-09-25). The length went from a correct 53.2 to a
    // wrong 71.4 eleven seconds later, which means something set it back to
    // nothing in between and said nothing about it. Only two places do that on
    // purpose and both of them log. So the next log will name the third.
    static float s_lastSeen[kArmSlots] = {};
    if (s_lastSeen[reachSlot] > 1.0f && g_restReach[reachSlot] <= 0.0f) {
        Log_Printf("ArmIk: the %s arm's %.1f units was cleared by something that did not say so",
            remote ? "partner's" : "right", s_lastSeen[reachSlot]);
    }
    s_lastSeen[reachSlot] = g_restReach[reachSlot];

    static bool s_letGo[kArmSlots] = {};
    static float s_candidate[2] = {};
    static int s_candFrames[2] = {};
    static unsigned long long s_settleSince[2] = {};
    const auto settled = [&](float units) {
        const unsigned long long nowSet = GetTickCount64();
        if (!s_settleSince[reachSlot])
            s_settleSince[reachSlot] = nowSet;
        if (s_candFrames[reachSlot] > 0 && s_candidate[reachSlot] > 1.0f
            && std::fabs(units - s_candidate[reachSlot]) < s_candidate[reachSlot] * 0.02f) {
            ++s_candFrames[reachSlot];
        } else {
            s_candidate[reachSlot] = units;
            s_candFrames[reachSlot] = 1;
        }
        // Two seconds of never settling means this arm is simply restless;
        // take the latest rather than never solving at all.
        if (s_candFrames[reachSlot] < 8 && nowSet - s_settleSince[reachSlot] < 2000)
            return false;
        s_candFrames[reachSlot] = 0;
        s_settleSince[reachSlot] = 0;
        return true;
    };

    static unsigned long long s_quietUntil[2] = {};
    static float s_wasReach[2] = {};
    if (s_quietUntil[reachSlot]) {
        const unsigned long long nowQuiet = GetTickCount64();
        if (nowQuiet < s_quietUntil[reachSlot])
            return; // hands off: let it animate, so there is something honest to measure
        // Still hands off until it holds still: the half second is the least it
        // waits, not the most.
        if (!settled(armUnits))
            return;
        s_quietUntil[reachSlot] = 0;
        g_restReach[reachSlot] = armUnits;
        // AND ONLY NOW DECIDE (2026-09-25, user: "some of the level transitions
        // broke my aim IK for a bit, it eventually solves itself").
        //
        // It solved itself because there was nothing wrong with it. After a
        // transition the skeleton is part way through being rebuilt and the arm
        // measures nonsense for a second or two; two seconds of that tripped the
        // heal, which threw the remembered length away, forgot the skeleton,
        // made the arm search run again - and then measured 53.2 units, exactly
        // what it had just discarded. Every time, in every heal in the log. The
        // aim broke for as long as the search took and came back none the wiser.
        //
        // The evidence for "this is a different arm" only exists AFTER the
        // quiet window, and the expensive half of the heal was being done
        // before it. So it waits for the answer now. A length that comes back
        // the same was right all along and nothing else needs doing; a length
        // that really has changed still forgets everything, which is the case
        // the heal was written for.
        const float was = s_wasReach[reachSlot];
        s_wasReach[reachSlot] = 0.0f;
        const bool sameArm = was > 1.0f && std::fabs(armUnits - was) < was * 0.05f;
        Log_Printf("ArmIk: the %s arm rests at %.1f units, measured with nothing posing it%s",
            remote ? "partner's" : "right", armUnits,
            was > 1.0f ? (sameArm ? " - the same arm as before, so the readings were the transition, not the arm"
                                  : " - a different arm, so the skeleton is being looked for again")
                       : "");
        if (was > 1.0f && !sameArm && !remote) {
            ArmIk_ForgetSkeleton();
            CameraRigHook_ForgetArms();
        }
    }
    if (g_restReach[reachSlot] <= 0.0f) {
        // LET GO BEFORE MEASURING (2026-09-25, user: "right after the costume
        // change my aim ik broke and then I believe I was fighting the animation
        // once it fixed").
        //
        // Both halves of that in one line of the log: "taking the right arm as
        // 71.4 units - it read the same for eight frames running". Seventy-one,
        // for an arm that is 53.2 and had been correctly measured as 53.2 eleven
        // seconds earlier.
        //
        // Holding still for eight frames rejects a skeleton mid-rebuild, which
        // is what it was written for, and does nothing at all about a wrong
        // number that is perfectly steady - and an arm WE are posing is the
        // steadiest thing on screen. Standing back from the solve is not enough
        // either, because the build pass keeps writing the cached pose at every
        // joint for half a second after the solve stops. So it measured our own
        // stretched arm, eight frames of it, and believed it.
        //
        // The heal already knew this and let go properly. This path never did.
        // Now it does: the cache goes, the guard goes, the animation gets the
        // arm back, and only then is anything measured. That is also the
        // fighting - once 71.4 was written down every honest 53.2 was refused,
        // so the animation had the arm and we were arguing with it over the
        // frames it did not.
        if (!s_letGo[reachSlot]) {
            s_letGo[reachSlot] = true;
            std::memset(g_poseQuatValid[g_solvingBody], 0, sizeof(g_poseQuatValid[g_solvingBody]));
            g_poseQuatMs[g_solvingBody] = 0;
            g_poseQuatJoints[g_solvingBody] = nullptr;
            PoseGuard_Clear();
        }
        if (!settled(armUnits))
            return;
        s_letGo[reachSlot] = false;
        g_restReach[reachSlot] = armUnits;
        Log_Printf("ArmIk: taking the %s arm as %.1f units - it read the same for eight frames with the "
                   "animation holding it",
            remote ? "partner's" : "right", armUnits);
    } else if (std::fabs(armUnits - g_restReach[reachSlot]) > g_restReach[reachSlot] * 0.10f) {
        ++g_badReads;
        // And a way out if the remembered one is simply wrong for whoever
        // this is now: a new character, a new mode, a reloaded level. A
        // skeleton caught mid-update sorts itself out in a frame or two, so
        // two solid seconds of nothing but bad reads is not that - it is the
        // wrong arm entirely. Forget the length AND the joints it was taken
        // from, and let both be found again.
        {
            static unsigned long long s_badSince[2] = {};
            static unsigned long long s_healedMs[2] = {};
            const unsigned long long nowHeal = GetTickCount64();
            if (!s_badSince[reachSlot])
                s_badSince[reachSlot] = nowHeal;
            if (nowHeal - s_badSince[reachSlot] > 2000 && nowHeal - s_healedMs[reachSlot] > 5000) {
                s_healedMs[reachSlot] = nowHeal;
                s_badSince[reachSlot] = 0;
                Log_Printf("ArmIk: two seconds of nothing but bad reads on the %s arm - letting go of it for half "
                           "a second to see what it rests at.",
                    remote ? "partner's" : "right");
                s_wasReach[reachSlot] = g_restReach[reachSlot];
                g_restReach[reachSlot] = 0.0f;
                s_quietUntil[reachSlot] = nowHeal + 500;
                // Let go of the arm for real. The window says "see what it
                // rests at", and the solve does stand back - but the build pass
                // goes on writing the cached pose at every joint for as long as
                // the cache is fresh, and it stays fresh for the same half
                // second. So the arm was never released and the resting length
                // was taken off an arm we were still holding.
                std::memset(g_poseQuatValid[g_solvingBody], 0, sizeof(g_poseQuatValid[g_solvingBody]));
                g_poseQuatMs[g_solvingBody] = 0;
                g_poseQuatJoints[g_solvingBody] = nullptr;
                PoseGuard_Clear();
                // Forgetting the skeleton used to happen HERE, on no evidence at
                // all - before the quiet window had measured anything. It waits
                // for the measurement now; see the block above.
            }
        }
        // What it measured (2026-09-22): after a costume change nearly every
        // frame came back bad and the animation took the arm, so the numbers
        // themselves are worth having.
        static unsigned long long s_badToldMs = 0;
        const unsigned long long nowBad = GetTickCount64();
        if (nowBad - s_badToldMs >= 1000) {
            s_badToldMs = nowBad;
            Log_Printf("ArmIk: skipped a frame - the %s arm (joints %d %d %d) measured %.1f units against %.1f",
                remote ? "partner's" : "right", body.right.shoulder, body.right.elbow, body.right.wrist, armUnits,
                g_restReach[reachSlot]);
        }
        return;
    }
    // Measured if the player has ever stood in the T-pose, assumed otherwise.
    float unitsPerMetre = armUnits / kArmMetres;
    if (settings.armIkScale > 0.05f)
        unitsPerMetre /= settings.armIkScale;

    // The point the headset stands in for: the character's eyes.
    float headPivot[3];
    if (!JointWorldPos(body.joints, body.head, headPivot))
        return;
    const float eye[3] = { headPivot[0] + up[0] * kEyeAbovePivot + forward[0] * kEyeAheadOfPivot,
        headPivot[1] + up[1] * kEyeAbovePivot + forward[1] * kEyeAheadOfPivot,
        headPivot[2] + up[2] * kEyeAbovePivot + forward[2] * kEyeAheadOfPivot };

#if RE5VR_DIAGNOSTICS
    {
        float rows[12], sc[3];
        const int who[2] = { body.right.shoulder, body.right.elbow };
        for (int j = 0; j < 2; ++j) {
            if (TryRead(rows, body.joints + who[j] * kJointStride + kOffJointWorldMatrix, sizeof(rows))) {
                for (int r = 0; r < 3; ++r) {
                    const float v[3] = { rows[r * 4], rows[r * 4 + 1], rows[r * 4 + 2] };
                    g_armScale[j * 3 + r] = Length3(v);
                }
            }
            if (TryRead(sc, body.joints + who[j] * kJointStride + kOffJointScale, sizeof(sc)))
                g_armScale[6 + j] = sc[0];
        }
    }
#endif

    // The scale, once, when a calibration is asked for (2026-09-18). The
    // character's own bind span - wrist to wrist, through both arms and the
    // shoulders between them - against the player's real one. Both are measured,
    // so a shorter character like Sheva needs no special case and neither does a
    // tall player.
    if (!remote && g_calibratePending && g_calHaveHand[0] && g_calHaveHand[1]
        && g_calUnitsPerMetre[1] <= 1.0f) {
        float span[3];
        Sub3(g_calHand[1], g_calHand[0], span);
        const float playerSpanM = Length3(span);
        // Chained bind offsets, the same way the bind dump does it: the pose is
        // carried entirely in the offsets on this rig, which the T-pose result
        // confirmed.
        float bindWrist[2][3] = {};
        bool haveBoth = true;
        for (int h = 0; h < 2; ++h) {
            const ArmIkArm& a2 = h == 0 ? body.left : body.right;
            if (!a2.valid || a2.wrist < 0 || a2.wrist >= count) {
                haveBoth = false;
                break;
            }
            float acc[3] = { 0.0f, 0.0f, 0.0f };
            int j = a2.wrist;
            for (int guard = 0; guard < 32 && j >= 0 && j < count; ++guard) {
                float off[4];
                if (!TryRead(off, body.joints + j * kJointStride + 0x10, sizeof(off))) {
                    haveBoth = false;
                    break;
                }
                for (int k = 0; k < 3; ++k)
                    acc[k] += off[k];
                const int p = parents[j];
                if (p >= count || p == j)
                    break;
                j = p;
            }
            std::memcpy(bindWrist[h], acc, sizeof(bindWrist[h]));
        }
        if (haveBoth && playerSpanM > 0.6f) {
            float charSpan[3];
            Sub3(bindWrist[1], bindWrist[0], charSpan);
            const float charSpanUnits = Length3(charSpan);
            if (!(charSpanUnits > 20.0f))
                Log_Printf("ArmIk: not calibrating - this character's wrist-to-wrist span reads %.1f units, "
                           "which is too small to be a skeleton",
                    charSpanUnits);
            if (charSpanUnits > 20.0f) {
                // The span only sizes the shoulders now; the scale itself comes
                // from each arm on its own, below.
                // And where your shoulders are. The character's shoulders are
                // this fraction of its own span apart, so yours are the same
                // fraction of yours - measured proportion, not a guess at
                // anybody's build.
                float charShoulder[2][3] = {};
                bool haveShoulders = true;
                for (int h = 0; h < 2; ++h) {
                    const ArmIkArm& a3 = h == 0 ? body.left : body.right;
                    if (!a3.valid || a3.shoulder < 0 || a3.shoulder >= count) {
                        haveShoulders = false;
                        Log_Printf("ArmIk: not calibrating - this character's %s shoulder was not found",
                            h == 1 ? "right" : "left");
                        break;
                    }
                    float acc[3] = { 0.0f, 0.0f, 0.0f };
                    int j = a3.shoulder;
                    for (int guard = 0; guard < 32 && j >= 0 && j < count; ++guard) {
                        float off[4];
                        if (!TryRead(off, body.joints + j * kJointStride + 0x10, sizeof(off))) {
                            haveShoulders = false;
                            break;
                        }
                        for (int k = 0; k < 3; ++k)
                            acc[k] += off[k];
                        const int p2 = parents[j];
                        if (p2 >= count || p2 == j)
                            break;
                        j = p2;
                    }
                    std::memcpy(charShoulder[h], acc, sizeof(charShoulder[h]));
                }
                if (haveShoulders) {
                    float sw[3];
                    Sub3(charShoulder[1], charShoulder[0], sw);
                    const float halfWidthM = (Length3(sw) / charSpanUnits) * playerSpanM * 0.5f;
                    // Averaged between the hands (2026-09-19). Taking each
                    // shoulder's height and depth from its OWN hand means a T-pose
                    // held a little crooked - and they all are - puts one shoulder
                    // higher than the other and leaves one arm measurably shorter.
                    // The user noticed: "the right arm is shorter than the left".
                    // Shoulders are level on a person, so level them here.
                    const float shoulderY = (g_calHand[0][1] + g_calHand[1][1]) * 0.5f;
                    const float shoulderZ = (g_calHand[0][2] + g_calHand[1][2]) * 0.5f;
                    for (int h = 0; h < 2; ++h) {
                        g_shoulderFromHead[h][0] = (h == 1 ? 1.0f : -1.0f) * halfWidthM;
                        g_shoulderFromHead[h][1] = shoulderY;
                        g_shoulderFromHead[h][2] = shoulderZ;
                        g_haveShoulderFromHead[h] = true;
                    }
                    // And the scale from the ARM rather than the whole span
                    // (2026-09-19). Span includes the shoulders, so scaling by it
                    // only lines the two bodies up if their arm-to-width
                    // proportions match. The user's do not: "I have a little bit
                    // more extension after Chris' arm locks out". Reach is decided
                    // by the arm alone, so measure the arm alone and the two lock
                    // out together.
                    float charArmV[3];
                    Sub3(bindWrist[1], charShoulder[1], charArmV);
                    const float charArmUnits = Length3(charArmV);
                    // PER HAND (2026-09-19). Averaging the two arms and using one
                    // scale for both is only right if your arms are the same
                    // length and you held the T-pose perfectly square. Neither is
                    // ever quite true, and the difference lands as one arm
                    // reaching and the other falling short - the user had the left
                    // right and the right still short. Each arm gets its own.
                    for (int h = 0; h < 2; ++h) {
                        float armV[3];
                        Sub3(g_calHand[h], g_shoulderFromHead[h], armV);
                        // Less the grip (2026-09-19). g_calHand is where the
                        // CONTROLLER is, and a controller sits most of a hand
                        // forward of the wrist - so this measured about 0.09 m more
                        // arm than anybody has. That makes units-per-metre too
                        // small, and then the solve ALSO steps the target back by
                        // the same 0.09 m at runtime, so the arm ends up short by
                        // the grip twice over. Roughly 13 per cent of a reach, which
                        // is what "shorter than it should have been" looks like.
                        //
                        // Arithmetic, not taste: measure to the wrist, and full
                        // extension maps to full extension exactly.
                        const float allowance = g_reachAllowanceM.load(std::memory_order_relaxed);
                        const float armM = Length3(armV) + allowance;
                        if (!(charArmUnits > 10.0f) || !(armM > 0.35f)) {
                            Log_Printf("ArmIk: not calibrating your %s arm - it measured %.2f m against "
                                       "%.1f character units, which is not a T-pose I can use",
                                h == 1 ? "right" : "left", armM, charArmUnits);
                        }
                        if (charArmUnits > 10.0f && armM > 0.35f) {
                            g_calUnitsPerMetre[h] = charArmUnits / armM;
                            Log_Printf("ArmIk: your %s arm measures %.2f m, taken as %.2f m against this "
                                       "character's %.1f units, so %.1f units per metre",
                                h == 1 ? "right" : "left", armM - allowance, armM, charArmUnits,
                                g_calUnitsPerMetre[h]);
                        }
                    }
                    Log_Printf("ArmIk: your shoulders are %.2f m apart and %.2f m below your headset",
                        halfWidthM * 2.0f, -shoulderY);
                }
                // g_calUnitsPerMetre is an ARRAY and was being passed to a
                // %.1f, which prints whatever happens to be on the stack and
                // is undefined behaviour through varargs (2026-09-19). Each
                // hand already reports its own number just above; this line
                // only has business reporting the span.
                Log_Printf("ArmIk: calibrated - your span is %.2f m, this character's is %.1f units, and "
                           "it was assuming %.1f units per metre before this",
                    playerSpanM, charSpanUnits, armUnits / kArmMetres);
            }
        } else if (playerSpanM <= 0.6f) {
            Log_Printf("ArmIk: not calibrating the scale - your hands are only %.2f m apart, so that was not a "
                       "T-pose", playerSpanM);
        }
    }

    ArmIkStatus status = {};
    status.unitsPerMetre = unitsPerMetre;

    float weight = settings.armIkWeight;
    if (!(weight > 0.0f))
        weight = 0.0f;
    if (weight > 1.0f)
        weight = 1.0f;

    if (!remote && settings.steadyHandsFiring) {
        const unsigned long long shotT = XrInput_LastShotMs();
        const bool firingNow = shotT && GetTickCount64() - shotT < 300;
        HoldBodyWhileFiring(body, count, firingNow);
        // The arms too (2026-09-23, user: "the arms shake while shooting
        // too"). Leaving the solve running made them the one live part of a
        // frozen body, so the shake could still arrive through them. While the
        // burst lasts, the arms keep the pose they had when it started and
        // your hands stop driving them - a test worth its cost: if the gun is
        // steady now, the shake came through the arm chain; if it still moves,
        // nothing on this skeleton is moving it.
        if (firingNow)
            return;
    }
    // The gun arm goes first for the player (2026-09-22): a hand holding the
    // gun is placed on the gun, and the gun is wherever this frame's solve put
    // the gun arm, not last frame's.
    const int gunArm = settings.characterLeftHanded ? 0 : 1;
    bool haveGunWrist = false;
    float gunWristPos[3] = {}, gunWristRot[9] = {};
    for (int step = 0; step < 2; ++step) {
        const int hand = remote ? step : (step == 0 ? gunArm : 1 - gunArm);
        const ArmIkArm& arm = hand == 0 ? body.left : body.right;
        const float outward = hand == 0 ? -1.0f : 1.0f;
        const int slot = slotBase + hand;
        // This arm's own measured scale, if it has been calibrated.
        // The slider applies to the measured scale too (2026-09-19). It was only
        // ever folded into the fallback, so the moment you calibrated, "Arm reach"
        // stopped doing anything at all - which is exactly what the user found.
        float upm = unitsPerMetre;
        if (g_calUnitsPerMetre[slot] > 1.0f) {
            upm = g_calUnitsPerMetre[slot];
            if (settings.armIkScale > 0.05f)
                upm /= settings.armIkScale;
        }

        float target[3];
        if (settings.armIkTest) {
            // A pose nobody could mistake for the animation: both hands held
            // straight out in front at shoulder height. If the arms do not go
            // there, the write is not landing and nothing else here matters.
            float shoulderPos[3];
            if (!JointWorldPos(body.joints, arm.shoulder, shoulderPos))
                continue;
            for (int i = 0; i < 3; ++i)
                target[i] = shoulderPos[i] + forward[i] * (armUnits * 0.8f) + right[i] * (outward * armUnits * 0.25f);
            status.handTracked[hand] = true;
            g_haveCtrl[slot] = false;
        } else if (remote) {
            // The partner, posed from hands that arrived over the network
            // (2026-09-20).
            //
            // What crossed is their hand and their shoulder, both in metres and
            // both measured from their own head in their own recentred frame.
            // Neither is a position in a room, so neither needs their room to
            // mean anything here. Hand MINUS shoulder is the length and
            // direction of a real arm, and that is the one quantity that means
            // the same thing on two people of different sizes.
            //
            // Their units-per-metre travels with it, because only they know how
            // long their real arm is. The character is the same character on
            // both machines, so their calibration is exactly the right scale to
            // put their reach onto it.
            if (!remote->haveHand[hand] || !remote->haveShoulder[hand]) {
                g_partnerWhy = "one of their hands arrived without a shoulder";
                continue;
            }
            const float theirUpm = remote->unitsPerMetre[hand];
            if (!(theirUpm > 1.0f)) {
                g_partnerWhy = "they have not calibrated, so their reach has no scale";
                continue;
            }

            const float rel[3] = { remote->handFromHead[hand][0] - remote->shoulderFromHead[hand][0],
                remote->handFromHead[hand][1] - remote->shoulderFromHead[hand][1],
                remote->handFromHead[hand][2] - remote->shoulderFromHead[hand][2] };

            // Anchored to where their shoulder RESTS, the same way the player's
            // is: we move the shoulder ourselves through the collarbone, so
            // anchoring to the live joint would have the target chase the
            // shoulder that is chasing the target.
            float shoulderPos[3];
            if (!JointWorldPos(body.joints, arm.shoulder, shoulderPos))
                continue;
            {
                const int clavJ = parents[arm.shoulder];
                const int rootJ = RootJoint(parents, count, arm.shoulder);
                float restDir[3], restLen = 0.0f, clavPos[3];
                if (clavJ < count && BindDirWorld(body.joints, arm.shoulder, rootJ, restDir, &restLen)
                    && JointWorldPos(body.joints, clavJ, clavPos) && restLen > 0.001f) {
                    for (int i = 0; i < 3; ++i)
                        shoulderPos[i] = clavPos[i] + restDir[i] * restLen;
                }
            }
            for (int i = 0; i < 3; ++i) {
                target[i] = shoulderPos[i]
                    + (right[i] * rel[0] + up[i] * rel[1] + forward[i] * rel[2]) * theirUpm;
            }
            // No further than the arm actually reaches (2026-09-23). A real
            // two player session put the partner's arm at 97 units when it
            // measures 53.2 - nearly double - and a bone that has changed
            // length fails the very next frame's sanity check, so the solve
            // threw the arm away and the animation took it back. That is the
            // "animation came back" in the flesh, and whatever is making the
            // reach too long, an arm should never be asked to stretch.
            {
                float reachV[3];
                Sub3(target, shoulderPos, reachV);
                const float want = Length3(reachV);
                const float most = armUnits * 0.98f;
                if (want > most && want > 0.001f) {
                    const float k = most / want;
                    for (int i = 0; i < 3; ++i)
                        target[i] = shoulderPos[i] + reachV[i] * k;
                }
#if RE5VR_DIAGNOSTICS
                static unsigned long long s_toldReach = 0;
                const unsigned long long nowReach = GetTickCount64();
                if (hand == 1 && nowReach - s_toldReach >= 2000) {
                    s_toldReach = nowReach;
                    // Which bone is the long one (2026-09-23). The target is
                    // now provably sane - 38.7 units asked for on a 53.2 unit
                    // arm, well inside its reach, with the new clamp never
                    // firing - and the arm still comes out at 123. So nothing
                    // that crossed the network is wrong any more and the
                    // stretch is happening in the solve. Splitting it tells us
                    // which half: an upper arm and a forearm that are both too
                    // long means everything is being scaled, while one of them
                    // at its right length and the other enormous means the
                    // elbow is being flung off its circle.
                    {
                        float sP[3], eP[3], wP[3];
                        if (JointWorldPos(body.joints, arm.shoulder, sP)
                            && JointWorldPos(body.joints, arm.elbow, eP)
                            && JointWorldPos(body.joints, arm.wrist, wP)) {
                            float uv2[3], fv2[3], miss[3];
                            Sub3(eP, sP, uv2);
                            Sub3(wP, eP, fv2);
                            Sub3(wP, target, miss);
                            Log_Printf("IkSync: their arm as it ended up - upper %.1f units, forearm %.1f, and the "
                                       "wrist is %.1f units from where it was asked to go",
                                Length3(uv2), Length3(fv2), Length3(miss));
                        }
                    }
                    // Every part of it, because the video says the direction is
                    // wrong as well as the length: both arms point straight up
                    // above his head for the whole clip, which no amount of
                    // over-reaching would do on its own. A hand held at chest
                    // height that comes out pointing at the sky means the
                    // shoulder it is measured from is in the wrong place, and
                    // the axis that is wrong will be obvious here.
                    Log_Printf("IkSync: their right hand is (%.2f %.2f %.2f) m from their head, their shoulder is "
                               "(%.2f %.2f %.2f), so the arm is (%.2f %.2f %.2f) = %.2f m at %.1f units per metre, "
                               "which is %.1f units on a %.1f unit arm%s",
                        remote->handFromHead[hand][0], remote->handFromHead[hand][1], remote->handFromHead[hand][2],
                        remote->shoulderFromHead[hand][0], remote->shoulderFromHead[hand][1],
                        remote->shoulderFromHead[hand][2], rel[0], rel[1], rel[2],
                        Length3(const_cast<float*>(rel)), theirUpm, want, armUnits,
                        want > most ? " - HELD BACK" : "");
                }
#endif
            }
            // And their hand's own axes, carried onto this character the same
            // way its position was, so the hand turns as well as travels and
            // whatever they are holding comes with it.
            for (int r = 0; r < 3; ++r) {
                const float* row = remote->basis[hand] + r * 3;
                for (int i = 0; i < 3; ++i)
                    g_ctrlWorld[slot][r * 3 + i] = right[i] * row[0] + up[i] * row[1] + forward[i] * row[2];
            }
            g_haveCtrl[slot] = true;
            status.handTracked[hand] = true;

            // The partner never presses calibrate on THIS machine, so the tie
            // between their controller's axes and this skeleton's rest
            // rotation has to be made the first time we see them. It is the
            // same arithmetic the T-pose does, without needing a T-pose: every
            // joint's rest rotation in this rig is the root's, so the root is
            // the reference, and their hand's rotation is read against it.
            if (!g_haveWristOffset[slot]) {
                const int rootJoint = RootJoint(parents, count, arm.shoulder);
                float restBasis[9];
                if (WorldRot(body.joints, rootJoint, restBasis)) {
                    float ctrlT[9];
                    Transpose3(g_ctrlWorld[slot], ctrlT);
                    Mul3(restBasis, ctrlT, g_wristOffset[slot]);
                    std::memcpy(g_armOffset[slot], g_wristOffset[slot], sizeof(g_armOffset[slot]));
                    std::memcpy(g_ctrlRef[slot], g_ctrlWorld[slot], sizeof(g_ctrlRef[slot]));
                    g_haveWristOffset[slot] = true;
                    g_haveArmOffset[slot] = true;
                    g_haveCtrlRef[slot] = true;
                    g_tiesCalibrated[slot] = true;
                    Log_Printf("ArmIk: tied the partner's %s hand - slot %d, and nothing the local player does "
                               "will overwrite it now",
                        hand == 1 ? "right" : "left", slot);
                    Log_Printf("ArmIk: tied the partner's %s hand to this skeleton",
                        hand == 1 ? "right" : "left");
                }
            }
        } else {
            // Which of YOUR controllers drives which of the character's hands
            // (2026-09-24, user: "our 'Playing as left handed character
            // (Sheva)' checkbox makes the left controller move the right hand
            // and right controller moved the left hand. The button change
            // worked fine, but movement should obviously not change").
            //
            // This asked which hand holds the GUN, which is two different
            // questions wearing one name. "Swap hands" means you are holding
            // your controllers the other way round, and that genuinely should
            // swap them. "The character is left handed" means SHEVA holds her
            // gun in her left hand - it says nothing whatsoever about which of
            // your arms is which. Your right hand drives her right hand either
            // way; all that changes is which of her hands the gun is in.
            const int xrHand = XrInput_GetSettings().swapHands ? (hand == 0 ? 1 : 0) : hand;
            XrHandPose pose;
            float frame[9];
            XRBridgeEyeView leftView, rightView;
            if (!XrInput_GetHandPose(xrHand, pose) || !VRBridge_GetTrackingFrame(frame)
                || !VRBridge_GetEyeViews(leftView, rightView))
                continue;
            // The headset itself, halfway between the eyes.
            const float headXr[3] = { (leftView.positionMeters[0] + rightView.positionMeters[0]) * 0.5f,
                (leftView.positionMeters[1] + rightView.positionMeters[1]) * 0.5f,
                (leftView.positionMeters[2] + rightView.positionMeters[2]) * 0.5f };
            const float raw[3] = { pose.posMeters[0] - headXr[0], pose.posMeters[1] - headXr[1],
                pose.posMeters[2] - headXr[2] };
            // Into the recentred frame: how far right, how far up and how far
            // in front of your own head you are holding it.
            const float local[3] = { raw[0] * frame[0] + raw[1] * frame[1] + raw[2] * frame[2],
                raw[0] * frame[3] + raw[1] * frame[4] + raw[2] * frame[5],
                raw[0] * frame[6] + raw[1] * frame[7] + raw[2] * frame[8] };
            // Anchored to the SHOULDER once calibrated (2026-09-18).
            //
            // Measuring from the head was the real reach bug, and the scale was
            // never more than a contributor. Your hand's offset from your HEAD
            // was being laid onto the character's EYE, and the arm then had to
            // span from its own shoulder to wherever that landed. Any difference
            // between how far your head sits above your shoulders and how far
            // Chris's does is added to the arm, every frame - which is why the
            // reach report sat at 1.2 to 1.36 of full with the arm pinned at
            // full stretch, and why calibrating the scale alone did not fix it.
            //
            // Shoulder to hand is the length of an arm, and it is the same
            // quantity on both bodies. So take YOUR hand from YOUR shoulder and
            // put it on the character's, and the arm is asked for exactly the
            // extension yours has.
            //
            // Where your shoulder is comes out of the T-pose: with your arms
            // straight out, your shoulders are at the height and depth of your
            // hands, and half a shoulder-width to each side. The width is the
            // character's own proportion of its span applied to yours, which
            // beats guessing at it.
            // Back off the controller onto the wrist, along the hand's own bone
            // axis - see kGripToWristMetres. The axis comes from the tie, so it
            // turns with your hand: flex your wrist and the controller swings
            // while this point stays where your wrist really is.
            float gripBack[3] = { 0.0f, 0.0f, 0.0f };
            if (g_foreAxis[slot] >= 0 && g_haveWristOffset[slot] && g_boneInRow >= 0) {
                float handBasis[9];
                Mul3(g_wristOffset[slot], g_ctrlWorld[slot], handBasis);
                const int fa = g_foreAxis[slot];
                float axis[3];
                if (g_boneInRow) {
                    axis[0] = handBasis[fa * 3];
                    axis[1] = handBasis[fa * 3 + 1];
                    axis[2] = handBasis[fa * 3 + 2];
                } else {
                    axis[0] = handBasis[fa];
                    axis[1] = handBasis[3 + fa];
                    axis[2] = handBasis[6 + fa];
                }
                for (int i = 0; i < 3; ++i)
                    axis[i] *= g_foreSign[slot];
                if (Normalise3(axis)) {
                    const float backUnits = XrInput_GetSettings().armIkWristPivotM * upm;
                    for (int i = 0; i < 3; ++i)
                        gripBack[i] = -axis[i] * backUnits;
                }
            }
#if RE5VR_DIAGNOSTICS
            // How far you really get from the shoulder the T-pose inferred, at
            // your furthest, while actually playing. If that peak runs well past
            // what the pose measured, the inferred shoulder is in the wrong place
            // and kReachAllowanceMetres is covering for it - which is worth
            // knowing, because then there is a real fix instead of a constant.
            if (g_haveShoulderFromHead[slot] && hand >= 0 && hand < 2) {
                float v[3];
                Sub3(local, g_shoulderFromHead[slot], v);
                const float reachedM = Length3(v);
                static float s_peak[2] = {};
                static unsigned long long s_peakMs = 0;
                if (reachedM > s_peak[hand])
                    s_peak[hand] = reachedM;
                const unsigned long long nowPeak = GetTickCount64();
                if (nowPeak - s_peakMs >= 5000) {
                    s_peakMs = nowPeak;
                    Log_Printf("ArmIk: furthest from the inferred shoulder so far - left %.2f m, right %.2f m",
                        s_peak[0], s_peak[1]);
                }
            }
#endif
            // Anchored to where the shoulder RESTS, not where it currently is
            // (2026-09-19). We move the shoulder ourselves now, through the
            // collarbone - so anchoring the target to the live joint would have
            // the target chase the shoulder that is chasing the target. Rest
            // position is the collarbone's own position plus the shoulder's bind
            // offset, and the collarbone's position belongs to the spine, which we
            // never touch.
            float shoulderPos[3];
            bool anchored = g_haveShoulderFromHead[slot]
                && JointWorldPos(body.joints, arm.shoulder, shoulderPos);
            if (anchored) {
                const int clavJ = parents[arm.shoulder];
                const int rootJ = RootJoint(parents, count, arm.shoulder);
                float restDir[3], restLen = 0.0f, clavPos[3];
                if (clavJ < count && BindDirWorld(body.joints, arm.shoulder, rootJ, restDir, &restLen)
                    && JointWorldPos(body.joints, clavJ, clavPos) && restLen > 0.001f) {
                    for (int i = 0; i < 3; ++i)
                        shoulderPos[i] = clavPos[i] + restDir[i] * restLen;
                }
            }
            if (anchored) {
                const float rel[3] = { local[0] - g_shoulderFromHead[slot][0],
                    local[1] - g_shoulderFromHead[slot][1], local[2] - g_shoulderFromHead[slot][2] };
                for (int i = 0; i < 3; ++i) {
                    target[i] = shoulderPos[i]
                        + (right[i] * rel[0] + up[i] * rel[1] + forward[i] * rel[2]) * upm
                        + gripBack[i];
                }
#if RE5VR_DIAGNOSTICS
                if (hand == gunArm) {
                    HandJitter& j = g_handJitter;
                    const unsigned long long shotJ = XrInput_LastShotMs();
                    const int k = shotJ && GetTickCount64() - shotJ < 300 ? 1 : 0;
                    if (j.have) {
                        float da[3], dl[3], dt[3];
                        Sub3(shoulderPos, j.prevAnchor, da);
                        Sub3(local, j.prevLocal, dl);
                        Sub3(target, j.prevTarget, dt);
                        const float v[3] = { Length3(da) * 100.0f / upm, Length3(dl) * 100.0f,
                            Length3(dt) * 100.0f / upm };
                        // Did last frame's pose survive? Where the wrist is now against
                        // where it was sent, both measured from the shoulder anchor so the
                        // body moving in between cancels out.
                        float wNow[3], lost = 0.0f;
                        if (JointWorldPos(body.joints, arm.wrist, wNow)) {
                            float a[3], b[3], c[3];
                            Sub3(wNow, shoulderPos, a);
                            Sub3(j.prevTarget, j.prevAnchor, b);
                            Sub3(a, b, c);
                            lost = Length3(c) * 100.0f / upm;
                        }
                        if (v[0] < 50.0f && v[2] < 50.0f) { // bigger is a teleport, not a jitter
                            j.sum[k][4] += lost;
                            if (lost > j.max[k][4])
                                j.max[k][4] = lost;
                            for (int q = 0; q < 3; ++q) {
                                j.sum[k][q] += v[q];
                                if (v[q] > j.max[k][q])
                                    j.max[k][q] = v[q];
                            }
                            ++j.n[k];
                        }
                    }
                    std::memcpy(j.prevAnchor, shoulderPos, sizeof(j.prevAnchor));
                    std::memcpy(j.prevLocal, local, sizeof(j.prevLocal));
                    std::memcpy(j.prevTarget, target, sizeof(j.prevTarget));
                    j.have = true;
                }
#endif
            } else {
                for (int i = 0; i < 3; ++i) {
                    target[i] = eye[i]
                        + (right[i] * local[0] + up[i] * local[1] + forward[i] * local[2]) * upm;
                }
            }
            status.handTracked[hand] = true;
            // The controller's own axes, carried into the world the same way its
            // position was: read off against the recentre reference, then laid
            // onto the character's right, up and facing.
            for (int r = 0; r < 3; ++r) {
                const float* row = pose.rot + r * 3;
                const float lx = row[0] * frame[0] + row[1] * frame[1] + row[2] * frame[2];
                const float ly = row[0] * frame[3] + row[1] * frame[4] + row[2] * frame[5];
                const float lz = row[0] * frame[6] + row[1] * frame[7] + row[2] * frame[8];
                for (int i = 0; i < 3; ++i)
                    g_ctrlWorld[slot][r * 3 + i] = right[i] * lx + up[i] * ly + forward[i] * lz;
            }
            g_haveCtrl[slot] = true;
            if (hand >= 0 && hand < 2) {
                std::memcpy(g_calHand[slot], local, sizeof(g_calHand[slot]));
                g_calHaveHand[slot] = true;
            }
            if (hand == 1) {
                std::memcpy(g_lastLocal, local, sizeof(g_lastLocal));
                std::memcpy(g_lastTarget, target, sizeof(g_lastTarget));
            }
        }

        // Two hands on the gun (2026-09-22). The off hand goes ON the gun and
        // stays there, riding with the gun arm, instead of hovering wherever
        // the controller is. The place is fixed at the moment of the grab:
        //   * out along a long gun, at however far out you took hold - every
        //     long gun measured holds 26 to 58 cm out along the gun wrist's
        //     -X, 8.5 cm below it - so the hand lands on the barrel line;
        //   * cupping a pistol, where the handgun and magnum animations put
        //     it, which agree within 1.5 cm across six guns.
        // Sheva's arm is the mirror of Chris's, and those numbers were taken on
        // Chris, so for her the hand simply keeps where it was on the gun.
        // The partner's off hand goes on the partner's gun too (2026-09-23,
        // user, testing against themselves: "the roll isn't coming over, at
        // least showing on sheva IK" and "the gun is floating"). The gun hand
        // DOES carry its roll - that is applied before the packet is built and
        // the receiver lays their hand's own axes onto this character. What
        // never travelled is the support hand, because putting it on the gun
        // needs the gun, and their gun is on their machine. So the fact is
        // sent instead of the result, and the same snap runs here against the
        // weapon this character is actually holding.
        bool holdingTwoHanded = false;
        if (remote) {
            IkSyncHands theirs;
            holdingTwoHanded = IkSync_GetPartnerHands(theirs) && theirs.twoHanded;
        }
        if ((!remote || holdingTwoHanded) && !settings.armIkTest && hand != gunArm && haveGunWrist
            && status.handTracked[hand]) {
            static bool s_wasHeld = false;
            static float s_hold[3] = {};
            // The hand's turn relative to the gun, taken at the grab. The
            // position alone is locked at the WRIST, so any twist of the
            // controller still swung the palm around it and off the gun - the
            // drift the user saw while moving. Held, the hand turns with the
            // gun and nothing else.
            static float s_relRot[9] = {};
            static bool s_haveRelRot = false;
            // Long guns only: how much higher than an ordinary hold this grab
            // was. Shotgun and SMG grabs land 0 to 1 cm up, the minigun's top
            // handle 8.5, so a high grab is a high handle.
            static float s_grabLift = 0.0f;
            static bool s_longHold = false;
            bool cupped = false;
            bool held = XrInput_GetTwoHand(&cupped);
            if (remote) {
                // Theirs, not ours. Cupped is not sent: a pistol held in two
                // hands is the common case and the wrong guess costs a few
                // centimetres of hand placement, not a wrong pose.
                held = holdingTwoHanded;
                cupped = false;
            }
            const float cmPerUnit = 100.0f / upm;
            if (held && !s_wasHeld) {
                float d[3];
                Sub3(target, gunWristPos, d);
                float rel[3];
                for (int r = 0; r < 3; ++r)
                    rel[r] = Dot3(gunWristRot + r * 3, d) * cmPerUnit;
                if (settings.characterLeftHanded) {
                    std::memcpy(s_hold, rel, sizeof(s_hold));
                } else if (cupped) {
                    s_hold[0] = 0.5f;
                    s_hold[1] = -8.0f;
                    s_hold[2] = -4.3f;
                } else {
                    float x = rel[0];
                    if (x > -20.0f)
                        x = -20.0f;
                    if (x < -70.0f)
                        x = -70.0f;
                    s_hold[0] = x;
                    s_hold[1] = -8.5f;
                    s_hold[2] = 2.0f;
                }
                s_longHold = !settings.characterLeftHanded && !cupped;
                s_grabLift = rel[2] > 3.0f ? rel[2] - 3.0f : 0.0f;
                if (s_grabLift > 12.0f)
                    s_grabLift = 12.0f;
                s_haveRelRot = false;
                if (g_haveCtrl[slot]) {
                    float gunT[9];
                    Transpose3(gunWristRot, gunT);
                    Mul3(g_ctrlWorld[slot], gunT, s_relRot);
                    s_haveRelRot = true;
                }
                Log_Printf("TwoHand: off hand put on the gun at (%.1f %.1f %.1f) cm in the gun wrist's axes - "
                           "grabbed at (%.1f %.1f %.1f)",
                    s_hold[0], s_hold[1], s_hold[2], rel[0], rel[1], rel[2]);
            }
            s_wasHeld = held;
            if (held && s_longHold)
                s_hold[2] = 2.0f + settings.twoHandRaiseCm + s_grabLift;
            if (held) {
                if (s_haveRelRot) {
                    Mul3(s_relRot, gunWristRot, g_ctrlWorld[slot]);
                    // And the twist the gun did not take (2026-09-23). Locked
                    // to the gun, this hand keeps none of its own turn, while
                    // the gun keeps a share of it - so the arm is asked for a
                    // wrist that is the whole roll away from the one you are
                    // actually holding, and it answers by twisting the forearm
                    // until the twist solver flips. Handing the difference
                    // back leaves the two agreeing, whatever the roll slider
                    // is set to.
                    float leftover[9];
                    if (XrInput_GetSupportRoll(leftover)) {
                        float turnedOff[9];
                        for (int r = 0; r < 3; ++r)
                            for (int k = 0; k < 3; ++k)
                                turnedOff[r * 3 + k] = leftover[k * 3] * g_ctrlWorld[slot][r * 3]
                                    + leftover[k * 3 + 1] * g_ctrlWorld[slot][r * 3 + 1]
                                    + leftover[k * 3 + 2] * g_ctrlWorld[slot][r * 3 + 2];
                        std::memcpy(g_ctrlWorld[slot], turnedOff, sizeof(turnedOff));
                    }
                }
                for (int i = 0; i < 3; ++i)
                    target[i] = gunWristPos[i]
                        + (gunWristRot[i] * s_hold[0] + gunWristRot[3 + i] * s_hold[1] + gunWristRot[6 + i] * s_hold[2])
                            / cmPerUnit;
            }
        }

        // Stop the hand at a body before the arm is asked to reach it. Doing it
        // to the TARGET rather than to the result is what makes it resistance
        // and not a correction: the solve never tries to reach inside anyone, so
        // there is nothing to fight and nothing to jitter.
        //
        // The player only (2026-09-20). This tests against g_otherJoints, and
        // when the PARTNER is the body being solved that is the very skeleton
        // being solved - so their hand was pushed out of their own chest every
        // frame, with no skip list to save it. That is what "his arms were
        // going crazy" was. The buzz was equally wrong: it fired the local
        // player's controllers according to what somebody else's arm hit.
        // Resistance is a thing you feel in your own hands, so it belongs to
        // whoever owns them.
        // Not while both hands are on the gun (2026-09-22). Your own body counts
        // as an obstacle, skipping only the arm the hand belongs to - so a hand
        // placed on the gun, 9 cm from the other wrist, was pushed off it every
        // frame and put back by the grip, which is the spasm the user found.
        // Holding the gun is supposed to touch it.
        if (!remote && !XrInput_GetTwoHand(nullptr)) {
            float depth = 0.0f;
            if (status.handTracked[hand] && PushOutOfBodies(target, body, parents, count, arm, &depth)) {
                // Firmer the deeper you press, so brushing past somebody and
                // leaning on them do not feel the same.
                const float amp = depth / 12.0f;
                // The same question, so the same answer: buzz the controller
                // that drives this hand, not the one holding the gun.
                XrInput_Pulse(XrInput_GetSettings().swapHands ? (hand == 0 ? 1 : 0) : hand,
                    amp > 0.45f ? 0.45f : amp, 0.03f);
            }
        }

        if (weight < 1.0f) {
            float animWrist[3];
            if (JointWorldPos(body.joints, arm.wrist, animWrist)) {
                for (int i = 0; i < 3; ++i)
                    target[i] = animWrist[i] + (target[i] - animWrist[i]) * weight;
            }
        }

        // Elbows hang down, a little out from the body and a little behind the
        // hand: enough to stop the solve folding the arm somewhere a person's
        // never goes.
        float pole[3];
        for (int i = 0; i < 3; ++i)
            pole[i] = -up[i] + right[i] * (outward * 0.35f) - forward[i] * 0.25f;
        Normalise3(pole);

        // SLOT, not hand (2026-09-20). This was passing `hand`, so every
        // partner solve ran SolveArm against the PLAYER's slots - their
        // controller basis, their calibration ties, their socket lock, their
        // forearm twist history. That is the exact collision the per-body slots
        // were introduced to stop, sitting in the one place the rename did not
        // reach because it is an argument rather than a lookup. Both bodies
        // were writing over each other's working state every frame.
        if (!SolveArm(body.joints, parents, count, arm, target, pole, slot, &status.reachUnits[hand],
                &status.missUnits[hand])
            && remote) {
            g_partnerWhy = "the two-bone solve refused their arm";
        }
#if RE5VR_DIAGNOSTICS
        if (!remote && hand == gunArm && g_handJitter.have) {
            HandJitter& j = g_handJitter;
            const unsigned long long shotJ = XrInput_LastShotMs();
            const unsigned long long nowJ = GetTickCount64();
            const int k = shotJ && nowJ - shotJ < 300 ? 1 : 0;
            const float miss = status.missUnits[hand] * 100.0f / upm;
            j.sum[k][3] += miss;
            if (miss > j.max[k][3])
                j.max[k][3] = miss;
            static unsigned long long s_toldJ = 0;
            if (j.n[1] > 10 && nowJ - s_toldJ >= 3000) {
                s_toldJ = nowJ;
                const float f = 1.0f / j.n[1];
                const float g = j.n[0] ? 1.0f / j.n[0] : 0.0f;
                Log_Printf("HandJitter: per frame while firing (avg/max cm) - shoulder anchor %.2f/%.2f, your controller "
                           "%.2f/%.2f, hand target %.2f/%.2f, arm short of target %.2f/%.2f | not firing - anchor "
                           "%.2f/%.2f, controller %.2f/%.2f, target %.2f/%.2f, short %.2f/%.2f; arm pose lost since last frame - "
                           "firing %.2f/%.2f, not %.2f/%.2f (%d/%d frames)",
                    j.sum[1][0] * f, j.max[1][0], j.sum[1][1] * f, j.max[1][1], j.sum[1][2] * f, j.max[1][2],
                    j.sum[1][3] * f, j.max[1][3], j.sum[0][0] * g, j.max[0][0], j.sum[0][1] * g, j.max[0][1],
                    j.sum[0][2] * g, j.max[0][2], j.sum[0][3] * g, j.max[0][3],
                    j.sum[1][4] * f, j.max[1][4], j.sum[0][4] * g, j.max[0][4], j.n[1], j.n[0]);
                std::memset(j.sum, 0, sizeof(j.sum));
                std::memset(j.max, 0, sizeof(j.max));
                j.n[0] = j.n[1] = 0;
            }
        }
#endif
        // How far the held hand really lands from its place on the gun. The
        // arm cannot always reach it, and a hand that falls short looks like
        // drift, so the worst of each second is reported while holding.
        if (!remote && hand != gunArm && XrInput_GetTwoHand(nullptr)) {
            static float s_worstCm = 0.0f;
            static unsigned long long s_toldMs = 0;
            const float cm = status.missUnits[hand] * 100.0f / upm;
            if (cm > s_worstCm)
                s_worstCm = cm;
            const unsigned long long nowHeld = GetTickCount64();
            if (nowHeld - s_toldMs >= 1000) {
                s_toldMs = nowHeld;
                Log_Printf("TwoHand: held hand misses its place on the gun by up to %.1f cm", s_worstCm);
                s_worstCm = 0.0f;
            }
        }
        // Where the gun arm ended up, for the other hand, and which way the
        // gun on it points in the controller's axes, for the grab test.
        if (!remote && hand == gunArm && JointWorldPos(body.joints, arm.wrist, gunWristPos)
            && WorldRot(body.joints, arm.wrist, gunWristRot)) {
            haveGunWrist = true;
            std::memcpy(g_solvedGunPos, gunWristPos, sizeof(g_solvedGunPos));
            std::memcpy(g_solvedGunRot, gunWristRot, sizeof(g_solvedGunRot));
            g_solvedGunJoint = arm.wrist;
            g_solvedUpm = upm;
            g_solvedGunMs = GetTickCount64();
            if (g_haveCtrl[slot]) {
                float b[3];
                for (int r = 0; r < 3; ++r)
                    b[r] = -Dot3(g_ctrlWorld[slot] + r * 3, gunWristRot);
                if (Normalise3(b)) {
                    std::memcpy(g_barrelCtrl, b, sizeof(b));
                    g_barrelMs = GetTickCount64();
                }
            }
        }
    }

    // Also the player's alone: poking a joint is a diagnostic aimed at your
    // own character, and the status readout has one set of numbers in it, so
    // letting the partner's solve write it would overwrite yours every frame.
    if (!remote) {
        PokeJoint(body, parents, count, up);
        PublishStatus(status);
    }

#if RE5VR_DIAGNOSTICS
    static unsigned long long s_lastLog = 0;
    const unsigned long long now = GetTickCount64();
    if (now - s_lastLog >= 1000) {
        s_lastLog = now;
        float lw[3] = {}, rw[3] = {};
        JointWorldPos(body.joints, body.left.wrist, lw);
        JointWorldPos(body.joints, body.right.wrist, rw);
        Log_Printf("ArmIk: %.1f units per metre, reach %.1f/%.1f, short by %.1f/%.1f, wrists now "
                   "(%.0f %.0f %.0f) and (%.0f %.0f %.0f)%s; solved %lu in the build and %lu after it, "
                   "best frame had %d of %d arm joints, %lu bad read(s), %lu unsquared; right arm "
                   "world rows %.3f %.3f %.3f and %.3f %.3f %.3f, scale %.3f/%.3f; %lu pose(s) held; "
                   "right hand "
                   "%.2f %.2f %.2f m "
                   "-> (%.0f %.0f %.0f)",
            unitsPerMetre, status.reachUnits[0], status.reachUnits[1], status.missUnits[0], status.missUnits[1],
            lw[0], lw[1], lw[2], rw[0], rw[1], rw[2], settings.armIkTest ? " [test pose]" : "", g_lateSolves,
            g_fallbackSolves, g_neededSeenHigh, g_neededTotal, g_badReads, g_renormalised, g_armScale[0], g_armScale[1], g_armScale[2],
            g_armScale[3], g_armScale[4], g_armScale[5], g_armScale[6], g_armScale[7], PoseGuard_TakeSaveCount(), g_lastLocal[0], g_lastLocal[1],
            g_lastLocal[2], g_lastTarget[0], g_lastTarget[1], g_lastTarget[2]);
        g_lateSolves = 0;
        g_fallbackSolves = 0;
        g_neededSeenHigh = 0;
        g_badReads = 0;
        g_renormalised = 0;
    }
#endif
}


// The object that reads the hand once a frame at exe+19CB74. Captured by the
// hook of the same name in camera_rig_hook.cpp: a weapon lives in its own
// object, nowhere near the character, and this pointer is the only thread we
// have to it.
unsigned char* g_handReader = nullptr;
unsigned long g_handReaderHits = 0;
bool g_handReaderRight = false;

// ---- What does the game build the pose FROM? (2026-09-17) ---------------
// Everything so far has written the world matrix at +0x50, which is the
// OUTPUT: the game recomposes it from each joint's local transform every
// frame. The arms move because the renderer reads that output. Nothing in a
// hand moves with them, from either side of the build and on either arm, and
// the socket that would carry an item - joint 51, sitting at 0.0 from the
// wrist - is one we already rotate. So whatever places a held object is not
// reading the output at all. It composes its own from the locals.
//
// Which means the solve has to write the local pose instead. The layout is a
// guess until it is checked: +0x10 reads as a rest-pose offset, +0x30 as
// scale, +0x50 as the world matrix, and the 16 bytes at +0x40 are exactly the
// right size and place for the rotation quaternion - the character's own
// facing lives at +0x40 of ITS object, same convention. Checked here rather
// than assumed: a quaternion has length 1, and every other candidate will not.
void ReportLocalPose(const ArmIkBody& body)
{
    const int joints[3] = { body.right.shoulder, body.right.elbow, body.right.wrist };
    const char* names[3] = { "shoulder", "elbow", "wrist" };
    for (int k = 0; k < 3; ++k) {
        const unsigned char* j = body.joints + joints[k] * kJointStride;
        float f[4];
        char line[320];
        int at = _snprintf_s(line, sizeof(line), _TRUNCATE, "Pose: right %s (joint %d)", names[k], joints[k]);
        const int offsets[4] = { 0x10, 0x20, 0x30, 0x40 };
        for (int o = 0; o < 4; ++o) {
            if (!TryRead(f, j + offsets[o], sizeof(f)))
                continue;
            const float len4 = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3]);
            at += _snprintf_s(line + at, sizeof(line) - at, _TRUNCATE,
                "; +0x%02X (%.3f %.3f %.3f %.3f) len %.3f", offsets[o], f[0], f[1], f[2], f[3], len4);
        }
        Log_Printf("%s", line);
    }
}

// ---- Is our pose still on the joint when the frame is drawn? (2026-09-18) --
// The gun leaves the hand while aiming, and ONLY while aiming, while the arm
// itself keeps tracking. Everything the solve writes goes through
// WriteLocalRotation, which caches the quaternion in g_poseQuat and hands it to
// PoseGuard. But PoseGuard only WATCHES four addresses - both shoulders and both
// elbows - and it puts the other held joints back only when one of those four is
// written. The whole design rests on the blend writing the arm in one pass.
//
// If the aim layer has a pass that touches the hand, the roll bone or the weapon
// socket WITHOUT touching the shoulder or the elbow, no watchpoint fires, that
// pass stands, and the gun ends up posed by the animation while the arm is posed
// by us. Which is what the user describes, and it would only ever happen while
// aiming, because that is the only time the layer runs.
//
// That is a theory, and this project has paid for enough of those. So measure
// it: a frame after writing, compare what we wrote against what is actually in
// each joint. Whatever has drifted is what is escaping the guard, and the number
// says how far. No more reasoning required.
void ReportPoseHeld(const ArmIkBody& body)
{
    static unsigned long long s_ms = 0;
    const unsigned long long now = GetTickCount64();
    if (now - s_ms < 1000)
        return;
    s_ms = now;

    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    const ArmIkArm& arm = body.right;
    if (!arm.valid || arm.wrist < 0 || arm.wrist >= count)
        return;

    // The roll bone, if there is one between the elbow and the hand.
    int roll = -1;
    {
        unsigned char links[4] = {};
        if (TryRead(links, body.joints + arm.wrist * kJointStride + kOffJointLinks, sizeof(links))) {
            const int p = links[1];
            if (p < count && p != arm.elbow)
                roll = p;
        }
    }

    struct Named {
        const char* name;
        int index;
    };
    const Named list[5] = { { "shoulder", arm.shoulder }, { "elbow", arm.elbow }, { "roll", roll },
        { "wrist", arm.wrist }, { "socket", g_socket[1] } };

    char line[360];
    int at = 0;
    for (int i = 0; i < 5; ++i) {
        const int idx = list[i].index;
        if (idx < 0 || idx >= kMaxJoints || idx >= count || !g_poseQuatValid[g_solvingBody][idx])
            continue;
        float live[4];
        if (!TryRead(live, body.joints + idx * kJointStride + kOffJointLocalRot, sizeof(live)))
            continue;
        float worst = 0.0f;
        for (int k = 0; k < 4; ++k) {
            const float d = std::fabs(live[k] - g_poseQuat[g_solvingBody][idx][k]);
            if (d > worst)
                worst = d;
        }
        const int wrote = _snprintf_s(line + at, sizeof(line) - at, _TRUNCATE, "%s%s(%d) %.3f",
            at ? ", " : "", list[i].name, idx, worst);
        if (wrote <= 0)
            break;
        at += wrote;
    }
    if (!at)
        return;
    // 0.000 means the joint is still wearing our pose. Anything that climbs
    // while the gun is up is being written by something the guard never sees.
    // And how far out the arm is, because "depth is not 1:1" needs a number:
    // reach is what your hand asked for against what the arm can span. Under 1
    // and nothing is clamped, so any lost depth is in the mapping rather than
    // in the solve.
    // The spin, as a number (2026-09-18). Four theories have been wrong about
    // this, so stop describing it and measure it: the forearm's roll about its
    // own bone, in degrees, and how much it moved since the last report. If the
    // arm is still and that step is a steady non-zero, something is precessing,
    // and the SIZE of the step says how fast. If it is zero while the user still
    // sees the forearm turning, then what is turning is not the forearm.
    float rollDeg = 0.0f, rollStep = 0.0f;
    {
        float elbowW[9];
        if (arm.elbow >= 0 && arm.elbow < count && WorldRot(body.joints, arm.elbow, elbowW)) {
            // Row 1 against row 2 is a roll about row 0 whichever way the bone
            // axis runs; the absolute value is arbitrary, the CHANGE is not.
            rollDeg = std::atan2(elbowW[4], elbowW[7]) * 180.0f / 3.14159265358979f;
            static float s_lastRoll = 0.0f;
            static bool s_haveRoll = false;
            if (s_haveRoll) {
                rollStep = rollDeg - s_lastRoll;
                while (rollStep > 180.0f)
                    rollStep -= 360.0f;
                while (rollStep < -180.0f)
                    rollStep += 360.0f;
            }
            s_lastRoll = rollDeg;
            s_haveRoll = true;
        }
    }
    Log_Printf("ArmIk: pose held on the right arm, %s - drift %s; reach %.2f of full, collarbone share %.2f; "
               "forearm roll %.1f deg, moved %+.1f deg in the last second",
        g_aimingNow ? "AIMING" : "gun down", line, g_lastReachFrac[1], g_lastClavShare[1], rollDeg, rollStep);
}

// ---- What pose does this skeleton REST in? (2026-09-18) ------------------
// Every tie taken today has been measured against the animation - the arm as it
// happened to be at the instant the gun came up. That is a moving target, and it
// is why the ties drift, need retaking, and land ninety degrees out whenever the
// capture lands early in the raise.
//
// The bind pose does not move. +0x10 is each joint's offset from its parent with
// the bone's length in w, and it is authored data: it is the same in a cutscene,
// mid-sprint, or on the title screen. If the tie is measured against THAT, it is
// measured once and it is right forever.
//
// Which leaves one thing to find out, and the user is the reason to ask it: they
// pointed out that Chris does not stand in a T-pose, he stands with the pistol up
// across his chest. True, and irrelevant to the bind pose - but it does decide
// what we would ask a player to DO. If this rig rests in a T-pose, "stand in a
// T-pose" is the right instruction; if it rests in an A-pose, asking for a T-pose
// bakes in a fresh forty-five degree error.
//
// So print it. The offsets are chained naively here, which is only correct if the
// bind pose carries its shape in the offsets and not in rotations - and that is
// exactly what the numbers will show: chain them and see whether the result looks
// like a person. Arms out sideways is a T-pose. Arms down and out is an A-pose.
// Nonsense means the offsets need the bind rotations too, and that is worth
// knowing before building anything on top of them.
void ReportBindPose(const ArmIkBody& body)
{
    static const unsigned char* s_toldFor = nullptr;
    if (s_toldFor == body.joints)
        return;
    s_toldFor = body.joints;

    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    unsigned char parents[kMaxJoints];
    for (int i = 0; i < count; ++i) {
        unsigned char links[4] = {};
        if (!TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
            return;
        parents[i] = links[1];
    }

    // Chained from the root, parents first - the array is ordered that way, and
    // anything out of order simply reads as its own offset.
    static float bind[kMaxJoints][3];
    std::memset(bind, 0, sizeof(bind));
    for (int i = 0; i < count; ++i) {
        float off[4];
        if (!TryRead(off, body.joints + i * kJointStride + 0x10, sizeof(off)))
            return;
        const int p = parents[i];
        const bool haveParent = p < count && p != i;
        for (int k = 0; k < 3; ++k)
            bind[i][k] = (haveParent ? bind[p][k] : 0.0f) + off[k];
    }

    const ArmIkArm& arm = body.right;
    const int clav = (arm.shoulder >= 0 && arm.shoulder < count) ? parents[arm.shoulder] : -1;
    int roll = -1;
    if (arm.wrist >= 0 && arm.wrist < count) {
        const int p = parents[arm.wrist];
        if (p < count && p != arm.elbow)
            roll = p;
    }
    struct Named {
        const char* name;
        int index;
    };
    const Named list[6] = { { "collarbone", clav }, { "shoulder", arm.shoulder }, { "elbow", arm.elbow },
        { "roll", roll }, { "wrist", arm.wrist }, { "socket", g_socket[1] } };

    Log_Printf("BindPose: the right arm as the skeleton was authored - x is across the body, y up, z forward");
    for (int i = 0; i < 6; ++i) {
        const int idx = list[i].index;
        if (idx < 0 || idx >= count)
            continue;
        float off[4];
        if (!TryRead(off, body.joints + idx * kJointStride + 0x10, sizeof(off)))
            continue;
        float dir[3] = { off[0], off[1], off[2] };
        const float len = Length3(dir);
        if (len > 0.001f)
            for (int k = 0; k < 3; ++k)
                dir[k] /= len;
        Log_Printf("BindPose:   %s (joint %d, parent %d) sits at (%.1f %.1f %.1f), %.1f long from its parent "
                   "along (%.2f %.2f %.2f)",
            list[i].name, idx, static_cast<int>(parents[idx]), bind[idx][0], bind[idx][1], bind[idx][2], len,
            dir[0], dir[1], dir[2]);
    }
    // Which way the palm faces in the rest pose (2026-09-19). The user takes
    // "level" in the T-pose to mean palms down, and the tie maps their
    // controller onto this skeleton's REST rotation - so those two only agree if
    // the rig rests with its palms down as well. Most T-pose rigs do. This one
    // has not been asked.
    //
    // The bone offsets cannot answer it on their own: they give directions, not
    // roll. The fingers can. They spread across the palm, and the thumb leaves it
    // at an angle, so the plane they define IS the palm and its normal is which
    // way the palm faces. Reported in the character's own axes: x runs to its
    // left, y up, z forward. So (0 -1 0) is palms down, (0 0 1) is palms forward,
    // and anything else is the angle we would otherwise have baked in silently.
    if (arm.wrist >= 0 && arm.wrist < count) {
        float spread[3] = { 0.0f, 0.0f, 0.0f };
        float best[3] = { 0.0f, 0.0f, 0.0f };
        float bestLen = 0.0f;
        int seen = 0;
        for (int i = 0; i < count; ++i) {
            if (parents[i] != arm.wrist || i == arm.wrist)
                continue;
            float d[3];
            Sub3(bind[i], bind[arm.wrist], d);
            const float len = Length3(d);
            if (!(len > 0.01f))
                continue;
            ++seen;
            for (int k = 0; k < 3; ++k)
                spread[k] += d[k] / len;
            if (len > bestLen) {
                bestLen = len;
                std::memcpy(best, d, sizeof(best));
            }
        }
        if (seen >= 2 && Normalise3(spread) && Normalise3(best)) {
            // The widest-apart pair spans the palm; their cross product leaves it.
            float normal[3];
            Cross3(spread, best, normal);
            if (Normalise3(normal)) {
                Log_Printf("BindPose:   the right palm faces (%.2f %.2f %.2f) at rest, from %d finger root(s) "
                           "- (0 -1 0) is palms down, (0 0 1) is palms forward",
                    normal[0], normal[1], normal[2], seen);
            }
        } else {
            Log_Printf("BindPose:   only %d finger root(s) hang off the hand - cannot tell which way the "
                       "palm faces", seen);
        }
    }
    // The one line that answers the question: the direction from shoulder to
    // wrist in the rest pose. Mostly sideways is a T-pose, down-and-out is an
    // A-pose, forward means it rests holding something.
    if (arm.shoulder >= 0 && arm.shoulder < count && arm.wrist >= 0 && arm.wrist < count) {
        float span[3];
        Sub3(bind[arm.wrist], bind[arm.shoulder], span);
        const float len = Length3(span);
        if (Normalise3(span))
            Log_Printf("BindPose:   shoulder to wrist runs (%.2f %.2f %.2f) over %.1f units - sideways is a "
                       "T-pose, down and out is an A-pose",
                span[0], span[1], span[2], len);
    }
}

// ---- The attachment table (2026-09-19) ----------------------------------
// The skeleton scan is exhausted: every joint within reach of the wrist is
// either the arm, one of the two sockets sitting exactly ON the wrist, or a
// finger. The sockets measure 0.0 units from the hand and the grip probe came
// back saying their rotation matches the hand's to a few degrees. So the gun is
// not on a bone, and writing bones will never move it.
//
// The character's own scan found what looks like a table instead: pointers at
// character+0x21D0, +0x2200, +0x2230 and +0x2260 - 0x30 apart - each leading to
// an object carrying a transform, one of them sitting exactly on the hand and
// the others a short way off it. That is the shape of an attachment list.
//
// Which of them is the weapon is decidable without guessing. Express each one's
// position in the HAND's own frame and watch it: anything rigidly attached to
// the hand holds a constant offset there no matter how the arm moves, and the
// one that does NOT is the one that has come adrift - which is the gun, because
// coming adrift is exactly what the user can see it doing.
constexpr int kSlotBase = 0x21D0;
constexpr int kSlotStride = 0x30;
constexpr int kSlots = 8;

void ReportAttachments(const ArmIkBody& body)
{
    static unsigned long long s_ms = 0;
    const unsigned long long now = GetTickCount64();
    if (now - s_ms < 1000)
        return;
    s_ms = now;

    float handPos[3], handRot[9];
    if (!JointWorldPos(body.joints, body.right.wrist, handPos)
        || !WorldRot(body.joints, body.right.wrist, handRot))
        return;

    char line[380];
    int at = 0;
    for (int s = 0; s < kSlots; ++s) {
        unsigned char* obj = nullptr;
        if (!TryRead(&obj, body.character + kSlotBase + s * kSlotStride, sizeof(obj)) || !obj)
            continue;
        // +0xD0 is where the character scan kept finding a position near the
        // hand; +0x2D0 turned up once as well, so try both and take whichever
        // reads as a plausible world position.
        const int offsets[2] = { 0xD0, 0x2D0 };
        for (int o = 0; o < 2; ++o) {
            float p[3];
            if (!TryRead(p, obj + offsets[o], sizeof(p)))
                continue;
            float d[3];
            Sub3(p, handPos, d);
            const float dist = Length3(d);
            if (!(dist < 300.0f))
                continue;
            // Into the hand's own frame, so a rigid attachment reads the same
            // three numbers whatever the arm is doing.
            const float inHand[3] = { Dot3(d, handRot), Dot3(d, handRot + 3), Dot3(d, handRot + 6) };
            const int w = _snprintf_s(line + at, sizeof(line) - at, _TRUNCATE, "%s+0x%X+0x%X (%.1f, in hand %.1f %.1f %.1f)",
                at ? "; " : "", kSlotBase + s * kSlotStride, offsets[o], dist, inHand[0], inHand[1], inHand[2]);
            if (w <= 0) {
                at = static_cast<int>(sizeof(line)) - 1;
                break;
            }
            at += w;
        }
        if (at >= static_cast<int>(sizeof(line)) - 1)
            break;
    }
    if (at)
        Log_Printf("Attach: right hand at (%.0f %.0f %.0f) - %s", handPos[0], handPos[1], handPos[2], line);
}

// ---- What is holding the gun? (2026-09-17) ------------------------------
// The arms move and the weapon does not, from either side of the skeleton
// build, so whatever places it is not reading the joints we are writing. Two
// candidates, and one scan each, because guessing at this has been slower than
// measuring it every time:
//
//   1. A socket joint. If the gun hangs off a joint that sits at the hand but
//      is NOT descended from it - its own little chain, placed alongside the
//      arm rather than under it - then moving the hand would leave it exactly
//      where the user says it is. Every joint near the wrist gets reported
//      with whether it is under the wrist.
//   2. A transform kept on the character. The aim pitch turned out to live in
//      two places, and the body's facing in three, so a copy of the hand held
//      somewhere in the character object is entirely in keeping. Anything in
//      the first 12K that reads as a position near the wrist is reported with
//      its offset.
//
// Both are read-only, developer only, and at most once every two seconds.
void ReportWhatHoldsTheGun(const ArmIkBody& body)
{
    float wrist[3];
    if (!JointWorldPos(body.joints, body.right.wrist, wrist))
        return;

    unsigned char parents[kMaxJoints];
    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    for (int i = 0; i < count; ++i) {
        unsigned char links[4] = {};
        if (!TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
            return;
        parents[i] = links[1];
    }
    int subtree[kMaxJoints];
    const int n = Subtree(parents, count, body.right.wrist, subtree);
    bool under[kMaxJoints] = {};
    for (int i = 0; i < n; ++i)
        under[subtree[i]] = true;

    Log_Printf("Gun: looking for what holds it - right wrist is joint %d at (%.0f %.0f %.0f), with %d joint(s) "
               "under it",
        body.right.wrist, wrist[0], wrist[1], wrist[2], n);

    // Skip the head and everything hanging off it (2026-09-19). Widening the
    // window to 200 units filled this with two dozen face joints, all at the same
    // distance, and the cap meant it never reached anything that could be holding
    // a weapon. A gun is not on somebody's cheekbone.
    bool underHead[kMaxJoints] = {};
    if (body.head >= 0 && body.head < count) {
        int headKids[kMaxJoints];
        const int hn = Subtree(parents, count, body.head, headKids);
        for (int i = 0; i < hn; ++i)
            underHead[headKids[i]] = true;
    }
    int reported = 0;
    for (int i = 0; i < count && reported < 24; ++i) {
        if (underHead[i])
            continue;
        float p[3];
        if (!JointWorldPos(body.joints, i, p))
            continue;
        const float d[3] = { p[0] - wrist[0], p[1] - wrist[1], p[2] - wrist[2] };
        const float dist = Length3(d);
        // Widened to 200 (2026-09-19). At 40 it only ever found the joints sitting
        // exactly ON the wrist, and those are now measured at 0.0 units from the
        // hand with their pose held - so the gun is not on any of them. If it is
        // on a bone at all, that bone is wherever the gun has floated to, which is
        // further away than the old window could see.
        if (dist > 200.0f)
            continue;
        unsigned char links[4] = {};
        TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links));
        ++reported;
        Log_Printf("Gun:   joint %d (id %u, parent %u) is %.1f from the wrist, %s", i,
            static_cast<unsigned>(links[3]), static_cast<unsigned>(links[1]), dist,
            under[i] ? "under it - moves with the hand" : "NOT under it - a socket the hand does not carry");
    }
    if (!reported)
        Log_Printf("Gun:   no joint within 40 units of the wrist at all");

    // And the character object, for a copy of the hand kept outside the
    // skeleton. Read in one block so a bad page costs one failed read, not
    // three thousand.
    constexpr int kScanBytes = 0x3000;
    static unsigned char scan[kScanBytes];
    if (!TryRead(scan, body.character, sizeof(scan))) {
        Log_Printf("Gun:   could not read the character object to scan it");
        return;
    }
    int hits = 0;
    for (int off = 0; off + 12 <= kScanBytes && hits < 16; off += 4) {
        const float* v = reinterpret_cast<const float*>(scan + off);
        const float d[3] = { v[0] - wrist[0], v[1] - wrist[1], v[2] - wrist[2] };
        const float dist = Length3(d);
        if (!(dist < 40.0f))
            continue;
        ++hits;
        Log_Printf("Gun:   character+0x%X reads as a position %.1f from the wrist (%.0f %.0f %.0f)", off, dist, v[0],
            v[1], v[2]);
    }
    if (!hits)
        Log_Printf("Gun:   nothing in the character's first %d bytes sits near the wrist", kScanBytes);

    // Following the character's POINTERS, not just its floats (2026-09-18). A
    // weapon lives in its own object; what the character holds is a pointer to
    // it. Rotating the hand and the socket it hangs off both left the gun
    // untouched, so it is not parented to either bone - it is placed from them,
    // and something else decides which way it points. That something is in an
    // object the character knows about, so walk the pointers and look for one
    // holding a position at the hand.
    int followed = 0, found = 0;
    for (int off = 0; off + 4 <= kScanBytes && found < 8 && followed < 400; off += 4) {
        unsigned long value = 0;
        std::memcpy(&value, scan + off, sizeof(value));
        // Something that could be a heap address, and aligned like one.
        if (value < 0x01000000u || value > 0x7FF00000u || (value & 3u) != 0u)
            continue;
        ++followed;
        constexpr int kObjBytes = 0x400;
        static unsigned char obj[kObjBytes];
        if (!TryRead(obj, reinterpret_cast<const void*>(static_cast<uintptr_t>(value)), sizeof(obj)))
            continue;
        for (int inner = 0; inner + 12 <= kObjBytes; inner += 4) {
            const float* v = reinterpret_cast<const float*>(obj + inner);
            const float d[3] = { v[0] - wrist[0], v[1] - wrist[1], v[2] - wrist[2] };
            const float dist = Length3(d);
            if (!(dist < 25.0f))
                continue;
            // A position, not a coincidence: the three floats should look like
            // world coordinates rather than small numbers that happen to be near
            // zero when the hand is near the origin.
            if (std::fabs(v[0]) < 1.0f && std::fabs(v[2]) < 1.0f)
                continue;
            ++found;
            Log_Printf("Gun:   character+0x%X points at %08lX, and +0x%X of that is %.1f from the hand "
                       "(%.0f %.0f %.0f)",
                off, value, inner, dist, v[0], v[1], v[2]);
            break;
        }
    }
    if (!found)
        Log_Printf("Gun:   followed %d pointer(s) out of the character and none of them holds a position "
                   "at the hand",
            followed);

    // And the object that reads the hand every frame. Everything else here has
    // come up empty, so this is the one thing left that is demonstrably
    // interested in where the hand is.
    if (!g_handReader) {
        Log_Printf("Gun:   nothing has read the hand through exe+19CB74 yet");
        return;
    }
    constexpr int kObjBytes = 0x800;
    static unsigned char obj[kObjBytes];
    if (!TryRead(obj, g_handReader, sizeof(obj))) {
        Log_Printf("Gun:   the hand reader at %p could not be read", g_handReader);
        return;
    }
    Log_Printf("Gun:   the %s hand's reader is %p, read %lu time(s); first 32 bytes %02X %02X %02X %02X %02X %02X "
               "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        g_handReaderRight ? "right" : "left", g_handReader, g_handReaderHits, obj[0], obj[1], obj[2], obj[3], obj[4], obj[5], obj[6], obj[7], obj[8],
        obj[9], obj[10], obj[11], obj[12], obj[13], obj[14], obj[15]);
    int closeHits = 0;
    for (int off = 0; off + 12 <= kObjBytes && closeHits < 16; off += 4) {
        const float* v = reinterpret_cast<const float*>(obj + off);
        const float d[3] = { v[0] - wrist[0], v[1] - wrist[1], v[2] - wrist[2] };
        const float dist = Length3(d);
        if (!(dist < 100.0f))
            continue;
        ++closeHits;
        Log_Printf("Gun:     +0x%X is a position %.1f from the hand (%.0f %.0f %.0f)", off, dist, v[0], v[1],
            v[2]);
    }
    if (!closeHits)
        Log_Printf("Gun:     nothing in its first %d bytes is within 100 units of the hand", kObjBytes);
}

// ---- Which bone is which? (2026-09-19) ----------------------------------
// Tonight has been full of changes that moved something other than the thing
// they were aimed at, and the last one settled it: giving joint 53 twist instead
// of half the whole turn left the visible forearm bend EXACTLY as it was, and
// broke the gun. Both at once. Whatever 53 is, it is not the forearm's roll bone
// carrying the hand, which is what the code has assumed all night.
//
// So stop inferring it. Turn one joint a long way and look. Whatever moves is
// what that joint drives - the mesh, the weapon, the fingers, or nothing at all.
// Twenty seconds a joint, and it ends the guessing.
//
// The pose is absolute rather than relative, so it holds still instead of
// winding up: the joint is put at the skeleton's rest rotation turned by the
// chosen angle about the body's up axis. Same answer every frame.
void PokeJoint(const ArmIkBody& body, const unsigned char* parents, int count, const float up[3])
{
    const int poke = XrInput_GetSettings().armIkPokeJoint;
    // Hand the last one back before taking a new one (2026-09-19). The poke goes
    // through the same write that HOLDS a pose against the animation, so without
    // this the joint stays where it was put - the user saw fingers stay extended
    // and never return. Clearing releases every held joint; the solve re-arms
    // them on the next frame, so nothing else notices.
    {
        static int s_pokedLast = -1;
        if (s_pokedLast != poke) {
            s_pokedLast = poke;
            PoseGuard_Clear();
        }
    }
    if (poke < 0 || poke >= count || g_quatConvention < 0)
        return;
    const int rootJoint = RootJoint(parents, count, poke);
    float rest[9];
    if (!WorldRot(body.joints, rootJoint, rest))
        return;

    const float rad = XrInput_GetSettings().armIkPokeDeg * 3.14159265358979f / 180.0f;
    const float cs = std::cos(rad), sn = std::sin(rad);
    // Rodrigues about the body's up axis, applied to each row of the rest basis.
    float turned[9];
    for (int r = 0; r < 3; ++r) {
        const float* v = rest + r * 3;
        float cross[3];
        Cross3(up, v, cross);
        const float dot = Dot3(up, v) * (1.0f - cs);
        for (int i = 0; i < 3; ++i)
            turned[r * 3 + i] = v[i] * cs + cross[i] * sn + up[i] * dot;
    }

    float parentAbs[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    const int p = parents[poke];
    if (p < count && p != poke)
        WorldRot(body.joints, p, parentAbs);
    WriteLocalRotation(body.joints, poke, turned, parentAbs);

    static int s_told = -2;
    if (s_told != poke) {
        s_told = poke;
        unsigned char links[4] = {};
        TryRead(links, body.joints + poke * kJointStride + kOffJointLinks, sizeof(links));
        Log_Printf("ArmIk: poking joint %d (id %u, parent %u) by %.0f degrees - watch what moves", poke,
            static_cast<unsigned>(links[3]), static_cast<unsigned>(links[1]),
            XrInput_GetSettings().armIkPokeDeg);
    }
}

// ---- Hands that stop at people (2026-09-19) -----------------------------
// The user wants resistance, not a buzz: "I'm looking for more than just
// vibration when you collide with Sheva, yourself, or a wall. This would be a
// physical resistance."
//
// A controller has no brake, so nothing can stop YOUR hand. What can be stopped
// is the character's - hold the arm at the surface while your real hand carries
// on into the space behind it. Your hands visibly part company, and that parting
// IS the resistance; it is what every VR game does and what your eyes read as
// solidity. A pulse on top sells the moment of contact.
//
// A body is approximated as a ball around every bone. Crude, and completely
// adequate: a skeleton's joints already trace out a person, and a hand's width
// around each one fills in the flesh. No mesh, no collision query, nothing to
// find in the executable.
//
// Walls are NOT this. The world's geometry is not in the skeleton and would need
// the game's own collision, which is a separate hunt.
bool PushOutOfBodies(float target[3], const ArmIkBody& body, const unsigned char* parents, int count,
    const ArmIkArm& arm, float* outDepth)
{
    const XrInputSettings settings = XrInput_GetSettings();
    if (!settings.armIkTouch)
        return false;
    const float radius = settings.armIkTouchRadius;
    if (!(radius > 1.0f))
        return false;

    // Your own arm is not an obstacle to itself.
    bool skip[kMaxJoints] = {};
    if (arm.shoulder >= 0 && arm.shoulder < count) {
        int mine[kMaxJoints];
        const int n = Subtree(parents, count, arm.shoulder, mine);
        for (int i = 0; i < n; ++i)
            skip[mine[i]] = true;
    }

    float deepest = 0.0f;
    // Two passes, so a hand wedged between two bones settles instead of being
    // shoved straight back into the one it just left.
    for (int pass = 0; pass < 2; ++pass) {
        const unsigned char* sets[2] = { body.joints, g_otherJoints };
        const int counts[2] = { count, (GetTickCount64() - g_otherMs < 500) ? g_otherCount : 0 };
        for (int s = 0; s < 2; ++s) {
            if (!sets[s])
                continue;
            for (int i = 0; i < counts[s]; ++i) {
                if (s == 0 && skip[i])
                    continue;
                float p[3];
                if (!JointWorldPos(sets[s], i, p))
                    continue;
                float d[3];
                Sub3(target, p, d);
                const float len = Length3(d);
                if (!(len < radius) || len < 0.001f)
                    continue;
                const float depth = radius - len;
                if (depth > deepest)
                    deepest = depth;
                for (int k = 0; k < 3; ++k)
                    target[k] = p[k] + d[k] / len * radius;
            }
        }
    }
#if RE5VR_DIAGNOSTICS
    // Is the partner actually there? Chris felt accurate and Sheva felt thin at
    // the same radius through the same code, which points at her data being
    // stale or absent rather than at the thickness (2026-09-19).
    {
        static unsigned long long s_touchMs = 0;
        const unsigned long long nowTouch = GetTickCount64();
        if (nowTouch - s_touchMs >= 3000) {
            s_touchMs = nowTouch;
            const bool live = g_otherJoints && (nowTouch - g_otherMs < 500);
            int good = 0;
            if (live) {
                for (int i = 0; i < g_otherCount; ++i) {
                    float p[3];
                    if (JointWorldPos(g_otherJoints, i, p) && Length3(p) > 1.0f)
                        ++good;
                }
            }
            Log_Printf("Touch: your own skeleton has %d joints; the partner is %s, %d joint(s) readable", count,
                live ? "LIVE" : "NOT being published", good);
        }
    }
#endif
    if (outDepth)
        *outDepth = deepest;
    return deepest > 0.0f;
}

// ---- When to solve (2026-09-17) -----------------------------------------
// The first version solved from the camera hook, which is late in the frame:
// late enough that the write survives - the arms really do move - and too late
// for the gun, which the game had already placed from the hand it built
// earlier. "The gun just floats in front of you" is that gap, exactly.
//
// The watchpoint found the build: three nested writers, exe+183B29 outermost
// by its stack frame, all composing one joint's world matrix from its parent's.
// So the frame goes build, attach the weapon, camera, draw, and the arms have
// to be moved between the first two.
//
// Which leaves the question of when the build is DONE, since moving a joint
// the loop has not reached yet just gets overwritten. Nothing announces it, so
// it is counted: a joint written twice means a new frame has started, which
// gives the number of writes a frame contains, and from then on the solve runs
// on the last one. Self-calibrating in a single frame, and it makes no
// assumption about the order the skeleton is walked in.
// Works out that set once per skeleton. Solving the moment it is complete is
// as early as the arms can be moved, which is what the gun needs: the weapon
// is attached to the hand partway THROUGH the model update, not after it, so
// waiting for the whole skeleton was still too late.
void BuildNeededSet(const ArmIkBody& body)
{
    std::memset(g_needed, 0, sizeof(g_needed));
    g_neededTotal = 0;
    unsigned char parents[kMaxJoints];
    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    for (int i = 0; i < count; ++i) {
        unsigned char links[4] = {};
        if (!TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
            return;
        parents[i] = links[1];
    }
    int subtree[kMaxJoints];
    const int roots[2] = { body.left.shoulder, body.right.shoulder };
    for (int r = 0; r < 2; ++r) {
        const int n = Subtree(parents, count, roots[r], subtree);
        for (int i = 0; i < n; ++i) {
            if (!g_needed[subtree[i]]) {
                g_needed[subtree[i]] = true;
                ++g_neededTotal;
            }
        }
    }
    g_key[0] = body.left.shoulder;
    g_key[1] = body.left.elbow;
    g_key[2] = body.left.wrist;
    g_key[3] = body.right.shoulder;
    g_key[4] = body.right.elbow;
    g_key[5] = body.right.wrist;
    std::memset(g_keySeen, 0, sizeof(g_keySeen));
    // And the second place, which is every level transition. This runs whenever
    // the joint array is a new allocation, which happens on a reload of the very
    // same character on the very same rig - the case the caller has just
    // announced as "the same body again, keeping your calibration". Throwing the
    // arm length away there contradicts the line above it and guarantees a cold
    // start at the exact moment the skeleton is least worth measuring.
    //
    // Kept, for the same reason as the aim edge: a length that really has
    // changed fails the ten per cent test on the next frame and is re-derived
    // properly. A length thrown away on suspicion has to be re-derived NOW,
    // whatever state the skeleton is in.
    // The socket, per arm: at the wrist, but neither above nor below it.
    for (int side = 0; side < 2; ++side) {
        const ArmIkArm& a = side == 0 ? body.left : body.right;
        g_socket[side] = -1;
        float wristPos[3];
        if (!JointWorldPos(body.joints, a.wrist, wristPos))
            continue;
        int under[kMaxJoints];
        const int n = Subtree(parents, count, a.wrist, under);
        bool below[kMaxJoints] = {};
        for (int i = 0; i < n; ++i)
            below[under[i]] = true;
        for (int i = 0; i < count; ++i) {
            if (below[i] || i == a.elbow || i == a.shoulder)
                continue;
            // Not an ancestor of the wrist either - those are the twist bones.
            bool ancestor = false;
            for (int up = parents[a.wrist], g = 0; up < count && g < 8; up = parents[up], ++g) {
                if (up == i) {
                    ancestor = true;
                    break;
                }
                if (parents[up] == up)
                    break;
            }
            if (ancestor)
                continue;
            // Nor a child of the elbow (2026-09-22). That is what a forearm
            // twist bone IS - the definition the twist code finds them by - and
            // the distal one sits right at the wrist, neither above nor below
            // it, so on Chris's usual rig this rule picked joint 51. Turning it
            // with the hand makes the forearm copy the hand, the exact fault
            // the 09-19 poke test found, and aiming broke in that costume while
            // the intro costume's rig aimed perfectly.
            if (parents[i] == a.elbow)
                continue;
            float p[3];
            if (!JointWorldPos(body.joints, i, p))
                continue;
            float d[3];
            Sub3(p, wristPos, d);
            if (Length3(d) > 3.0f)
                continue;
            g_socket[side] = i;
            break;
        }
        if (g_socket[side] >= 0)
            Log_Printf("ArmIk: the %s hand's socket is joint %d (parent %d; wrist %d, elbow %d) - it will turn with "
                       "the hand",
                side == 0 ? "left" : "right", g_socket[side], parents[g_socket[side]], a.wrist, a.elbow);
        else
            Log_Printf("ArmIk: the %s hand has no separate socket on this rig - the wrist carries the gun",
                side == 0 ? "left" : "right");
    }
    Log_Printf("ArmIk: solving once joints %d %d %d and %d %d %d have been built each frame", g_key[0],
        g_key[1], g_key[2], g_key[3], g_key[4], g_key[5]);
}

} // namespace

// Defined below, next to the partner state it works on.
void SolvePartner();
extern unsigned long long g_partnerSolvedMs;

#if RE5VR_DIAGNOSTICS
// Two-handed grips (2026-09-22). Every weapon has its own place for the off
// hand - the top handle of a minigun, the foregrip of an SMG, the back of a
// pistol - and the game's aiming animation already puts the hand there. So the
// grab point does not need a table: it is wherever the animation left the off
// wrist, measured in the GUN wrist's own axes, because the gun hangs off that
// wrist and the offset then travels with it wherever the controller goes.
//
// Only honest while nothing of ours is posing the arms, so it stands down while
// 6DOF arms are on. Averaged over the whole time the gun is up (after it has
// settled) and reported when it goes down, with the spread, because a grab
// point that wanders 10 cm during the aim is not one.
// FIND IT BY WATCHING IT CHANGE (2026-09-25, after five signature scans found
// nothing but index buffers, and the user: "this is getting tiresome testing").
//
// Entirely my fault, and the lesson is the same one this whole night has been
// teaching: stop inferring, measure. Every scan so far asked "where does this
// number pattern appear", and a game heap is full of triangle indices, which are
// long runs of ascending small integers containing every pattern you can name.
// No signature survives that.
//
// Watching a number CHANGE does. Note every place holding the value the player
// can see, have them fire one shot, and keep only the places that went down by
// one. Index buffers do not move when you pull a trigger. It needs no guess
// about width, stride, layout or whether empty slots are stored, because the
// game points at the answer itself.
//
// Built as a tool with a box to type in rather than a constant to rebuild, so
// the next number anybody wants - health, money, a partner's ammo - costs one
// test and nothing from me.
constexpr int kMaxCandidates = 1 << 21; // two million, 16 MB of them
struct Candidate {
    unsigned char* at;
    int width;
};
Candidate* g_candidates = nullptr;
int g_candidateCount = 0;

// WHAT POINTS AT IT (2026-09-25).
//
// The differential scan found the ammo, but at a heap address that will be
// somewhere else next launch, so on its own it is a fact rather than a feature.
// What makes it usable is a route: something whose address never changes, whose
// contents lead to it.
//
// Two kinds of answer are worth having, and they are found in different places.
// A pointer sitting in the EXE's own data is a static base, which is the whole
// prize - the same number every run. A pointer sitting on the heap is one link
// of a chain, and gets followed up a level. So this walk covers mapped images
// as well as private memory when asked, which the value scan never needed to.
template <typename Visit>
void WalkTheHeap(Visit visit, bool includeImages = false)
{
    unsigned dummy = 0;
    unsigned char* const ourStack = reinterpret_cast<unsigned char*>(&dummy);
    SYSTEM_INFO si = {};
    GetSystemInfo(&si);
    unsigned char* at = static_cast<unsigned char*>(si.lpMinimumApplicationAddress);
    unsigned char* end = static_cast<unsigned char*>(si.lpMaximumApplicationAddress);
    while (at < end) {
        MEMORY_BASIC_INFORMATION mbi = {};
        if (!VirtualQuery(at, &mbi, sizeof(mbi)))
            break;
        unsigned char* base = static_cast<unsigned char*>(mbi.BaseAddress);
        unsigned char* next = base + mbi.RegionSize;
        const DWORD readable = PAGE_READWRITE | PAGE_EXECUTE_READWRITE;
        const bool ours = ourStack >= base && ourStack < next;
        const bool rightKind = mbi.Type == MEM_PRIVATE || (includeImages && mbi.Type == MEM_IMAGE);
        if (mbi.State == MEM_COMMIT && rightKind && (mbi.Protect & readable) != 0
            && (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) == 0 && mbi.RegionSize >= 0x1000
            && mbi.RegionSize <= 0x4000000 && !ours) {
            __try {
                visit(base, mbi.RegionSize);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
            }
        }
        if (next <= at)
            break;
        at = next;
    }
}

void MemFind_First(unsigned value)
{
    if (!g_candidates)
        g_candidates = static_cast<Candidate*>(std::malloc(sizeof(Candidate) * kMaxCandidates));
    if (!g_candidates) {
        Log_Printf("Find: no room to remember where %u is", value);
        return;
    }
    g_candidateCount = 0;
    size_t scanned = 0;
    WalkTheHeap([&](unsigned char* base, size_t bytes) {
        scanned += bytes;
        const unsigned short* s16 = reinterpret_cast<const unsigned short*>(base);
        for (size_t i = 0; i < bytes / 2 && g_candidateCount < kMaxCandidates; ++i) {
            if (s16[i] == static_cast<unsigned short>(value) && value <= 0xFFFFu)
                g_candidates[g_candidateCount++] = { base + i * 2, 2 };
        }
        const unsigned* s32 = reinterpret_cast<const unsigned*>(base);
        for (size_t i = 0; i < bytes / 4 && g_candidateCount < kMaxCandidates; ++i) {
            if (s32[i] == value)
                g_candidates[g_candidateCount++] = { base + i * 4, 4 };
        }
    });
    Log_Printf("Find: %d place(s) hold %u right now, out of %.0f MB. Change it, then press Narrow.",
        g_candidateCount, value, scanned / 1048576.0);
}

void MemFind_Next(unsigned value)
{
    if (!g_candidates || g_candidateCount <= 0) {
        Log_Printf("Find: nothing to narrow - press Find first");
        return;
    }
    int kept = 0;
    for (int i = 0; i < g_candidateCount; ++i) {
        const Candidate& c = g_candidates[i];
        unsigned now = 0;
        bool ok = false;
        __try {
            now = c.width == 2 ? *reinterpret_cast<const unsigned short*>(c.at)
                               : *reinterpret_cast<const unsigned*>(c.at);
            ok = true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
        }
        if (ok && now == value)
            g_candidates[kept++] = c;
    }
    g_candidateCount = kept;
    Log_Printf("Find: %d place(s) held the old value and now hold %u", g_candidateCount, value);
    for (int i = 0; i < g_candidateCount && i < 16; ++i) {
        const Candidate& c = g_candidates[i];
        const unsigned* r = reinterpret_cast<const unsigned*>(c.at - 0x20);
        Log_Printf("Find:   %p (%d byte field) - around it: %08X %08X %08X %08X | %08X | %08X %08X %08X",
            c.at, c.width, r[0], r[2], r[4], r[6], r[8], r[10], r[12], r[14]);
    }
    if (g_candidateCount > 16)
        Log_Printf("Find:   ...and %d more. Change it again and Narrow again.", g_candidateCount - 16);
}

void MemFind_PointersTo(unsigned target)
{
    if (target < 0x10000u) {
        Log_Printf("Points: %08X is not an address worth looking for", target);
        return;
    }
    // FOUR KILOBYTES WAS FAR TOO GENEROUS (2026-09-25). Climbing a second level
    // returned forty hits, every one of them four thousand bytes before the
    // target - which at that distance is not a pointer to anything, it is any
    // number that happens to fall in a very wide window. The real parents never
    // got printed because the junk filled the list first.
    //
    // Half a kilobyte instead, and exact hits reported before near ones. A
    // pointer to the thing itself is worth ten that merely land nearby.
    const unsigned low = target - 0x200u;
    unsigned char* const exe = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
    Log_Printf("Points: looking for anything holding %08X, or up to 4 KB before it", target);
    int found = 0;
    int statics = 0;
    // Two passes: everything pointing AT it, then everything pointing just
    // before it. The first kind is what a chain is made of.
    for (int pass = 0; pass < 2 && found < 40; ++pass)
    WalkTheHeap(
        [&](unsigned char* base, size_t bytes) {
            const unsigned* w = reinterpret_cast<const unsigned*>(base);
            for (size_t i = 0; i < bytes / 4 && found < 40; ++i) {
                const unsigned v = w[i];
                if (pass == 0 ? v != target : (v >= target || v < low))
                    continue;
                unsigned char* at = base + i * 4;
                ++found;
                // Is it in the game's own image? Then it is the same address
                // every launch, and the chase is over.
                MEMORY_BASIC_INFORMATION here = {};
                const bool isImage = VirtualQuery(at, &here, sizeof(here)) && here.Type == MEM_IMAGE;
                if (isImage) {
                    ++statics;
                    // Name the module (2026-09-25). The last scan reported nine
                    // hits at "re5dx9.exe+6D7962B4", which is 1.8 GB past the
                    // exe - they were in some other mapped module and the label
                    // simply subtracted the wrong base. A static in a DLL that
                    // moves between launches is worth nothing, so which module
                    // it is in is the whole point of saying STATIC at all.
                    char which[MAX_PATH] = {};
                    HMODULE mod = nullptr;
                    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                                | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            reinterpret_cast<LPCWSTR>(at), &mod)
                        && mod) {
                        wchar_t wide[MAX_PATH] = {};
                        GetModuleFileNameW(mod, wide, MAX_PATH);
                        const wchar_t* leaf = wcsrchr(wide, L'\\');
                        WideCharToMultiByte(CP_UTF8, 0, leaf ? leaf + 1 : wide, -1, which, sizeof(which) - 1,
                            nullptr, nullptr);
                    }
                    Log_Printf("Points: STATIC - %s+%X holds %08X, which is %u bytes before your address",
                        which[0] ? which : "somewhere mapped",
                        static_cast<unsigned>(at - (mod ? reinterpret_cast<unsigned char*>(mod) : exe)), v,
                        target - v);
                } else {
                    Log_Printf("Points: %p holds %08X, which is %u bytes before your address", at, v,
                        target - v);
                }
            }
        },
        true);
    Log_Printf("Points: %d place(s) point here, %d of them at a fixed address. %s", found, statics,
        statics ? "Those are the route."
                : "None fixed yet - put one of the heap addresses in the box and go up another level.");
}

// FOLLOW IT AND SEE (2026-09-25).
//
// Two fixed places in the exe hold the same value, 195 bytes before the ammo.
// That is either the route or a coincidence, and the difference is one press:
// read the pointer, step forward by the offset, and print what is there. If the
// rounds loaded, the capacity and the reserve are all sitting in the right
// places, it is the route - and it still has to survive a relaunch to be worth
// anything, which is the real test.
// WATCH IT INSTEAD (2026-09-25).
//
// Climbing a pointer chain by hand is slow and may not end anywhere fixed - some
// games only ever reach their player state through a register. Finding the CODE
// that writes the number does not care: whatever instruction decrements your
// magazine has the structure in a register every time it runs, so we read it
// from there and never scan again.
//
// This is not new machinery. It is how the arm pose writer was found on the
// 17th: a hardware watchpoint, and a report of what wrote to it. Only the
// address is different.
void MemFind_Watch(unsigned target)
{
    if (target < 0x10000u) {
        Log_Printf("Watch: %08X is not an address", target);
        return;
    }
    AimFinder_Rearm();
    AimFinder_Start(reinterpret_cast<void*>(target), "the number in the box");
    Log_Printf("Watch: watching %08X for writes - now go and change it", target);
}

void MemFind_Follow(unsigned exeOffset, unsigned addOffset)
{
    unsigned char* const exe = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
    unsigned char* holder = exe + exeOffset;
    unsigned p = 0;
    if (!TryRead(&p, holder, sizeof(p))) {
        Log_Printf("Follow: cannot read re5dx9.exe+%X", exeOffset);
        return;
    }
    Log_Printf("Follow: re5dx9.exe+%X holds %08X; plus %u is %08X", exeOffset, p, addOffset, p + addOffset);
    unsigned char* target = reinterpret_cast<unsigned char*>(p + addOffset);
    unsigned around[16] = {};
    if (!TryRead(around, target - 0x20, sizeof(around))) {
        Log_Printf("Follow: nothing readable there");
        return;
    }
    Log_Printf("Follow:   %p - loaded %u, capacity %u, reserve %u", target, around[8], around[12], around[14]);
    for (int row = 0; row < 2; ++row) {
        Log_Printf("Follow:   %+d: %08X %08X %08X %08X %08X %08X %08X %08X", row * 32 - 32,
            around[row * 8 + 0], around[row * 8 + 1], around[row * 8 + 2], around[row * 8 + 3],
            around[row * 8 + 4], around[row * 8 + 5], around[row * 8 + 6], around[row * 8 + 7]);
    }
}

void LookForTheInventory(unsigned char* character)
{
    (void)character; // the scans are driven from the menu now
}

void MeasureGrip(const ArmIkBody& body, const XrInputSettings& settings)
{
    static bool s_wasAiming = false;
    static unsigned long long s_upMs = 0;
    static int s_frames = 0;
    static float s_sum[3], s_lo[3], s_hi[3], s_sumBody[3], s_sumDist;
    static unsigned char* s_object = nullptr;
    static unsigned char* s_dumped[16] = {};
    static int s_dumpedCount = 0;
    static bool s_toldDriven = false;
    // WHAT WENT DOWN WHEN YOU FIRED (2026-09-25, user: "100 rounds in the
    // magazine" - and nothing in the weapon object is 100).
    //
    // Which settles what that object is: a vtable, some child pointers and a
    // couple of matrices. A render node, not an inventory entry. The count is
    // behind one of those children.
    //
    // Guessing which is pointless when the game will simply show us. The
    // weapon's own words and one level of everything it points at are copied
    // when the gun comes up and compared when it goes down, and anything that
    // DECREASED is named. Fire a known number of shots in between and the word
    // that fell by exactly that many is the magazine. Nothing else in a weapon
    // counts down while you hold the trigger.
    constexpr int kAmmoRoots = 10;   // the object, plus nine children
    constexpr int kAmmoWords = 64;   // 256 bytes of each
    static unsigned char* s_ammoAt[kAmmoRoots] = {};
    static unsigned s_ammoWas[kAmmoRoots][kAmmoWords] = {};
    static bool s_ammoHave = false;
    const unsigned long long nowMs = GetTickCount64();
    const bool leftHanded = settings.characterLeftHanded;

    if (!body.aiming) {
        if (s_wasAiming && s_frames >= 10) {
            const float n = static_cast<float>(s_frames);
            unsigned vtable = 0;
            if (s_object)
                TryRead(&vtable, s_object, sizeof(vtable));
            Log_Printf("Grip: %s hand on the gun, held object %p (vtable %08X), aimed %.1f s - the off wrist sat "
                       "(%.1f %.1f %.1f) cm from the gun wrist in that wrist's axes, spread (%.1f %.1f %.1f) "
                       "over %d frames; %.1f cm apart, which is %.1f forward, %.1f up, %.1f to the right of the "
                       "body",
                leftHanded ? "left" : "right", s_object, vtable, (nowMs - s_upMs) / 1000.0f, s_sum[0] / n,
                s_sum[1] / n, s_sum[2] / n, s_hi[0] - s_lo[0], s_hi[1] - s_lo[1], s_hi[2] - s_lo[2], s_frames,
                s_sumDist / n, s_sumBody[0] / n, s_sumBody[1] / n, s_sumBody[2] / n);
            // The first time each held object turns up, its first 256 bytes: two
            // weapons side by side are how a weapon ID field gets found.
            bool seen = !s_object;
            for (int i = 0; i < s_dumpedCount && !seen; ++i)
                seen = s_dumped[i] == s_object;
            if (!seen && s_dumpedCount < 16) {
                s_dumped[s_dumpedCount++] = s_object;
                unsigned words[64] = {};
                if (TryRead(words, s_object, sizeof(words))) {
                    for (int row = 0; row < 8; ++row) {
                        const unsigned* w = words + row * 8;
                        Log_Printf("Grip:   %p +%02X: %08X %08X %08X %08X %08X %08X %08X %08X", s_object,
                            row * 32, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
                    }
                }
            }
        } else if (s_wasAiming) {
            Log_Printf("Grip: the gun went down after %d measured frame(s) - hold the aim for a couple of seconds",
                s_frames);
        }
        // Once a session, on the first time a gun comes down: the grid is not
        // going to move, and this walks a fair amount of memory.
        {
            static bool s_looked = false;
            if (!s_looked) {
                s_looked = true;
                LookForTheInventory(body.character);
            }
        }
        // The after picture. Only falls are reported: a magazine goes down and
        // clocks, timers and animation counters go up.
        if (s_ammoHave) {
            s_ammoHave = false;
            int said = 0;
            for (int k = 0; k < kAmmoRoots && said < 24; ++k) {
                if (!s_ammoAt[k])
                    continue;
                unsigned now2[kAmmoWords] = {};
                if (!TryRead(now2, s_ammoAt[k], sizeof(now2)))
                    continue;
                for (int w = 0; w < kAmmoWords && said < 24; ++w) {
                    const unsigned a = s_ammoWas[k][w], b = now2[w];
                    if (b >= a || a > 10000u)
                        continue; // only small counters, and only downward
                    ++said;
                    Log_Printf("Ammo: %s +0x%02X went %u -> %u (down %u)",
                        k == 0 ? "the weapon" : "a child of it", w * 4, a, b, a - b);
                }
            }
            if (!said)
                Log_Printf("Ammo: nothing counted down while the gun was up - fire some shots before lowering it");
        }
        s_wasAiming = false;
        return;
    }
    if (!s_wasAiming) {
        s_wasAiming = true;
        s_upMs = nowMs;
        s_frames = 0;
        s_sumDist = 0.0f;
        // The before picture: the weapon and everything it points at.
        s_ammoHave = false;
        if (s_object) {
            unsigned head[16] = {};
            if (TryRead(head, s_object, sizeof(head))) {
                int n = 0;
                s_ammoAt[n++] = s_object;
                for (int i = 1; i < 16 && n < kAmmoRoots; ++i) {
                    // A pointer into the game's heap, and not one we have already.
                    if (head[i] < 0x00400000u || head[i] > 0x7FF00000u || (head[i] & 3u))
                        continue;
                    unsigned char* p = reinterpret_cast<unsigned char*>(head[i]);
                    bool dup = false;
                    for (int k = 0; k < n; ++k)
                        dup = dup || s_ammoAt[k] == p;
                    if (!dup)
                        s_ammoAt[n++] = p;
                }
                for (int k = n; k < kAmmoRoots; ++k)
                    s_ammoAt[k] = nullptr;
                s_ammoHave = true;
                for (int k = 0; k < kAmmoRoots; ++k) {
                    std::memset(s_ammoWas[k], 0, sizeof(s_ammoWas[k]));
                    if (s_ammoAt[k])
                        TryRead(s_ammoWas[k], s_ammoAt[k], sizeof(s_ammoWas[k]));
                }
            }
        }
        for (int i = 0; i < 3; ++i) {
            s_sum[i] = s_sumBody[i] = 0.0f;
            s_lo[i] = 1e9f;
            s_hi[i] = -1e9f;
        }
    }
    if (settings.armIk || settings.armIkTest) {
        if (!s_toldDriven) {
            s_toldDriven = true;
            Log_Printf("Grip: not measuring while your hands drive the arms - turn 'Your hands drive the arms' off");
        }
        s_frames = 0;
        return;
    }
    // The raise is an animated transition; measure the pose, not the way there.
    if (nowMs - s_upMs < 500)
        return;

    const ArmIkArm& gun = leftHanded ? body.left : body.right;
    const ArmIkArm& off = leftHanded ? body.right : body.left;
    float gPos[3], oPos[3], sPos[3], ePos[3], rot[9];
    if (!JointWorldPos(body.joints, gun.wrist, gPos) || !JointWorldPos(body.joints, off.wrist, oPos)
        || !JointWorldPos(body.joints, gun.shoulder, sPos) || !JointWorldPos(body.joints, gun.elbow, ePos)
        || !WorldRot(body.joints, gun.wrist, rot))
        return;
    float uv[3], fv[3];
    Sub3(ePos, sPos, uv);
    Sub3(gPos, ePos, fv);
    const float armUnits = Length3(uv) + Length3(fv);
    if (!(armUnits > 5.0f))
        return;
    const float cmPerUnit = 100.0f * kArmMetres / armUnits;
    float row0[4] = {};
    if (!TryRead(row0, body.character + kOffBodyTransform, sizeof(row0)))
        return;
    float across[3] = { row0[0], 0.0f, row0[2] };
    if (!Normalise3(across))
        return;
    const float right[3] = { -across[0], 0.0f, -across[2] };
    const float forward[3] = { -across[2], 0.0f, across[0] };

    float d[3];
    Sub3(oPos, gPos, d);
    for (int i = 0; i < 3; ++i) {
        const float v = Dot3(rot + i * 3, d) * cmPerUnit;
        s_sum[i] += v;
        if (v < s_lo[i])
            s_lo[i] = v;
        if (v > s_hi[i])
            s_hi[i] = v;
    }
    s_sumBody[0] += Dot3(forward, d) * cmPerUnit;
    s_sumBody[1] += d[1] * cmPerUnit;
    s_sumBody[2] += Dot3(right, d) * cmPerUnit;
    s_sumDist += Length3(d) * cmPerUnit;
    s_object = g_handReader;
    ++s_frames;
}
#endif

#if RE5VR_DIAGNOSTICS
// The firing shake (2026-09-22). The user's video shows the camera steady and
// the gun jumping inside the view on every shot, so an animation is moving
// joints. Which ones decides where it can be damped: below the wrist is the
// socket we already write, the collarbone moves the whole arm, and nothing on
// the skeleton means it is the weapon model's own animation.
//
// Every joint's local rotation is read each frame the gun is up, and the
// angle it turned since the last frame is added up. A joint the shot kicks
// travels far; one that only follows the aim barely moves. Reported per aim,
// in degrees per second, so an aim with shots and one without can be compared.
void MeasureShake(const ArmIkBody& body, const XrInputSettings& settings)
{
    static bool s_wasAiming = false;
    static unsigned long long s_upMs = 0;
    static float s_prev[kMaxJoints][4];
    static bool s_havePrev[kMaxJoints];
    static float s_travel[kMaxJoints];
    static float s_peak[kMaxJoints];
    static int s_frames = 0;
    const int count = body.jointCount < kMaxJoints ? body.jointCount : kMaxJoints;
    const unsigned long long nowMs = GetTickCount64();

    if (!body.aiming) {
        if (s_wasAiming && s_frames > 30) {
            const float secs = (nowMs - s_upMs) / 1000.0f;
            unsigned char parents[kMaxJoints] = {};
            for (int i = 0; i < count; ++i) {
                unsigned char links[4] = {};
                TryRead(links, body.joints + i * kJointStride + kOffJointLinks, sizeof(links));
                parents[i] = links[1];
            }
            // Which part of the body each joint belongs to, for reading it.
            const bool left = settings.characterLeftHanded;
            const ArmIkArm& gun = left ? body.left : body.right;
            const ArmIkArm& off = left ? body.right : body.left;
            static int s_part[kMaxJoints];
            for (int i = 0; i < count; ++i)
                s_part[i] = 0;
            int tmp[kMaxJoints];
            int n = Subtree(parents, count, gun.shoulder, tmp);
            for (int i = 0; i < n; ++i)
                s_part[tmp[i]] = 1;
            n = Subtree(parents, count, gun.wrist, tmp);
            for (int i = 0; i < n; ++i)
                s_part[tmp[i]] = 2;
            n = Subtree(parents, count, off.shoulder, tmp);
            for (int i = 0; i < n; ++i)
                s_part[tmp[i]] = 3;
            if (body.head >= 0) {
                n = Subtree(parents, count, body.head, tmp);
                for (int i = 0; i < n; ++i)
                    s_part[tmp[i]] = 4;
            }
            static const char* kPart[] = { "body", "gun arm", "gun hand/socket", "other arm", "head" };
            Log_Printf("Shake: aimed %.1f s over %d frames - the joints that turned most (deg/s, worst single frame):",
                secs, s_frames);
            bool used[kMaxJoints] = {};
            for (int k = 0; k < 12; ++k) {
                int best = -1;
                for (int i = 0; i < count; ++i)
                    if (!used[i] && (best < 0 || s_travel[i] > s_travel[best]))
                        best = i;
                if (best < 0 || s_travel[best] <= 0.0f)
                    break;
                used[best] = true;
                unsigned char links[4] = {};
                TryRead(links, body.joints + best * kJointStride + kOffJointLinks, sizeof(links));
                Log_Printf("Shake:   joint %d (id %u, parent %u, %s): %.0f deg/s, worst frame %.1f deg", best,
                    static_cast<unsigned>(links[3]), static_cast<unsigned>(links[1]), kPart[s_part[best]],
                    s_travel[best] / secs, s_peak[best]);
            }
        }
        s_wasAiming = false;
        return;
    }
    if (!s_wasAiming) {
        s_wasAiming = true;
        s_upMs = nowMs;
        s_frames = 0;
        for (int i = 0; i < kMaxJoints; ++i) {
            s_havePrev[i] = false;
            s_travel[i] = 0.0f;
            s_peak[i] = 0.0f;
        }
    }
    // Skip the raise, which is a big deliberate swing and not a shake.
    if (nowMs - s_upMs < 500)
        return;
    ++s_frames;
    for (int i = 0; i < count; ++i) {
        float q[4];
        if (!TryRead(q, body.joints + i * kJointStride + kOffJointLocalRot, sizeof(q)))
            continue;
        if (s_havePrev[i]) {
            float d = std::fabs(q[0] * s_prev[i][0] + q[1] * s_prev[i][1] + q[2] * s_prev[i][2] + q[3] * s_prev[i][3]);
            if (d > 1.0f)
                d = 1.0f;
            const float deg = 2.0f * std::acos(d) * 57.2957795f;
            if (deg < 90.0f) { // a frame that turns further than this is a bad read, not a shake
                s_travel[i] += deg;
                if (deg > s_peak[i])
                    s_peak[i] = deg;
            }
        }
        std::memcpy(s_prev[i], q, sizeof(q));
        s_havePrev[i] = true;
    }
}
#endif

#if RE5VR_DIAGNOSTICS
// Where the gun's firing kick lives (2026-09-22). With 6DOF arms on, the gun
// wrist stays where the solve put it through a whole magazine (0.7 deg, 0.5 cm
// on average), yet the gun jumps on screen - so the kick is on the weapon
// model, not the arm. The object that reads the hand's matrix once a frame
// (g_handReader) is the only thread to the weapon we have.
//
// So: scan that object, and everything its first 64 fields point at, for 4x4
// transforms sitting near the hand. Track each one RELATIVE to the hand, so
// aiming around cancels out, and add up how much it moves while a shot is in
// the last 0.3 s against how much it moves otherwise. The transform the kick
// is applied to is the one that only moves while firing.
struct WeaponCand {
    unsigned char* at;
    int ptrOff; // -1: in the object itself
    int off;
    float prevPos[3], prevRot[9];
    bool havePrev;
    float distCm;
    float fireDeg, fireCm, idleDeg, idleCm;
    int fireN, idleN;
};

bool LooksLikeTransform(const float* m, const float hand[3], float* distOut)
{
    for (int i = 0; i < 16; ++i)
        if (!std::isfinite(m[i]))
            return false;
    if (std::fabs(m[3]) > 1e-3f || std::fabs(m[7]) > 1e-3f || std::fabs(m[11]) > 1e-3f || std::fabs(m[15] - 1.0f) > 1e-3f)
        return false;
    float len[3];
    for (int r = 0; r < 3; ++r) {
        len[r] = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
        if (len[r] < 0.5f || len[r] > 2.0f)
            return false;
    }
    if (std::fabs(len[0] - len[1]) > len[0] * 0.05f || std::fabs(len[0] - len[2]) > len[0] * 0.05f)
        return false;
    const float d[3] = { m[12] - hand[0], m[13] - hand[1], m[14] - hand[2] };
    const float dist = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (dist > 150.0f)
        return false;
    *distOut = dist;
    return true;
}

void MeasureWeapon(const ArmIkBody& body, const XrInputSettings& settings)
{
    constexpr int kMaxCand = 96;
    constexpr int kScan = 0x1000;
    static unsigned char* s_obj = nullptr;
    static WeaponCand s_c[kMaxCand];
    static int s_n = 0;
    static unsigned long long s_scanMs = 0, s_toldMs = 0;
    // From the CHARACTER, not the hand reader (2026-09-22). The reader turned
    // out to be the character's own skinning - it reads every joint of the
    // forearm and hand - so it never led to the weapon. The character has to
    // know what it is holding, so every pointer in its first 12 KB is followed
    // and each target searched for transforms near the gun hand. The
    // skeleton's own joints are skipped: they are the arm, not the gun.
    unsigned char* obj = body.character;
    if (!obj)
        return;
    const ArmIkArm& gun = settings.characterLeftHanded ? body.left : body.right;
    float hp[3], hr[9];
    if (!JointWorldPos(body.joints, gun.wrist, hp) || !WorldRot(body.joints, gun.wrist, hr))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    const unsigned long long nowMs = GetTickCount64();
    const unsigned char* jointsLo = body.joints;
    const unsigned char* jointsHi = body.joints + body.jointCount * kJointStride;

    // Rescanned on a new character, when nothing was found, and every 10 s
    // while aiming, so a weapon swap is picked up - never more than every 2 s.
    static bool s_wasAimingW = false;
    const bool aimEdge = body.aiming && !s_wasAimingW;
    s_wasAimingW = body.aiming;
    if ((obj != s_obj || s_n == 0 || aimEdge) && nowMs - s_scanMs > 2000) {
        s_obj = obj;
        s_scanMs = nowMs;
        s_n = 0;
        constexpr int kCharBytes = 0x3000;
        static unsigned int fields[kCharBytes / 4];
        std::memset(fields, 0, sizeof(fields));
        if (!TryRead(fields, obj, sizeof(fields)))
            TryRead(fields, obj, 0x1000);
        static unsigned char block[kScan + 64];
        static unsigned int seenBase[kCharBytes / 4];
        int seenCount = 0;
        int bases = 0;
        for (int b = -1; b < kCharBytes / 4 && s_n < kMaxCand; ++b) {
            unsigned char* base = obj;
            if (b >= 0) {
                const unsigned int ptr = fields[b];
                if (ptr < 0x10000u || ptr >= 0x7FFF0000u || (ptr & 3u))
                    continue;
                base = reinterpret_cast<unsigned char*>(static_cast<uintptr_t>(ptr));
                if (base >= jointsLo && base < jointsHi)
                    continue;
                bool again = false;
                for (int k = 0; k < seenCount && !again; ++k)
                    again = seenBase[k] == ptr;
                if (again)
                    continue;
                seenBase[seenCount++] = ptr;
            }
            int len = kScan + 64;
            if (!TryRead(block, base, len)) {
                len = 0x400;
                if (!TryRead(block, base, len))
                    continue;
            }
            ++bases;
            for (int off = 0; off + 64 <= len && s_n < kMaxCand; off += 4) {
                unsigned char* at = base + off;
                if (at >= jointsLo && at < jointsHi)
                    continue;
                float m[16];
                std::memcpy(m, block + off, sizeof(m));
                float dist = 0.0f;
                if (!LooksLikeTransform(m, hp, &dist))
                    continue;
                bool dup = false;
                for (int k = 0; k < s_n && !dup; ++k)
                    dup = s_c[k].at == at;
                if (dup)
                    continue;
                WeaponCand& c = s_c[s_n++];
                std::memset(&c, 0, sizeof(c));
                c.at = at;
                c.ptrOff = b >= 0 ? b * 4 : -1;
                c.off = off;
                c.distCm = dist * 100.0f / upm;
                off += 60;
            }
        }
        Log_Printf("Weapon: searched the character %p and %d object(s) it points at - %d transform(s) within "
                   "1.5 m of the hand",
            obj, bases, s_n);
    }

    const unsigned long long shot = XrInput_LastShotMs();
    const bool firing = shot && nowMs - shot < 300;
    for (int k = 0; k < s_n; ++k) {
        WeaponCand& c = s_c[k];
        float m[16];
        if (!TryRead(m, c.at, sizeof(m)))
            continue;
        // Relative to the hand: position in the hand's axes, rotation as
        // (weapon rows) x (hand rows)^T.
        float d[3] = { m[12] - hp[0], m[13] - hp[1], m[14] - hp[2] };
        float pos[3];
        for (int r = 0; r < 3; ++r)
            pos[r] = Dot3(hr + r * 3, d) * 100.0f / upm;
        float rows[9];
        for (int r = 0; r < 3; ++r) {
            float v[3] = { m[r * 4], m[r * 4 + 1], m[r * 4 + 2] };
            Normalise3(v);
            std::memcpy(rows + r * 3, v, sizeof(v));
        }
        float hrT[9], rel[9];
        Transpose3(hr, hrT);
        Mul3(rows, hrT, rel);
        if (c.havePrev) {
            float tr = 0.0f;
            for (int r = 0; r < 3; ++r)
                tr += Dot3(rel + r * 3, c.prevRot + r * 3);
            float cs = (tr - 1.0f) * 0.5f;
            cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
            const float deg = std::acos(cs) * 57.2957795f;
            float dp[3];
            Sub3(pos, c.prevPos, dp);
            const float cm = Length3(dp);
            if (deg < 45.0f && cm < 30.0f) { // bigger is a torn read, not a kick
                if (firing) {
                    c.fireDeg += deg;
                    c.fireCm += cm;
                    ++c.fireN;
                } else {
                    c.idleDeg += deg;
                    c.idleCm += cm;
                    ++c.idleN;
                }
            }
        }
        std::memcpy(c.prevPos, pos, sizeof(pos));
        std::memcpy(c.prevRot, rel, sizeof(rel));
        c.havePrev = true;
    }

    if (nowMs - s_toldMs < 3000)
        return;
    s_toldMs = nowMs;
    bool any = false;
    for (int k = 0; k < s_n; ++k)
        any = any || s_c[k].fireN > 5;
    if (any) {
        Log_Printf("Weapon: movement relative to the hand, per frame, firing vs not:");
        bool used[kMaxCand] = {};
        for (int shown = 0; shown < 10; ++shown) {
            int best = -1;
            float bestScore = -1.0f;
            for (int k = 0; k < s_n; ++k) {
                const WeaponCand& c = s_c[k];
                if (used[k] || c.fireN <= 5)
                    continue;
                const float score = (c.fireDeg / c.fireN + c.fireCm / c.fireN)
                    - (c.idleN ? (c.idleDeg / c.idleN + c.idleCm / c.idleN) : 0.0f);
                if (score > bestScore) {
                    bestScore = score;
                    best = k;
                }
            }
            if (best < 0)
                break;
            used[best] = true;
            const WeaponCand& c = s_c[best];
            char where[48];
            if (c.ptrOff < 0)
                sprintf_s(where, "character+0x%X", c.off);
            else
                sprintf_s(where, "character->[+0x%X]+0x%X", c.ptrOff, c.off);
            Log_Printf("Weapon:   %s (%p), %.0f cm from the hand: firing %.2f deg %.2f cm, not firing %.2f deg %.2f cm "
                       "(%d/%d frames)",
                where, c.at, c.distCm, c.fireDeg / c.fireN, c.fireCm / c.fireN,
                c.idleN ? c.idleDeg / c.idleN : 0.0f, c.idleN ? c.idleCm / c.idleN : 0.0f, c.fireN, c.idleN);
        }
    }
    for (int k = 0; k < s_n; ++k)
        s_c[k].fireDeg = s_c[k].fireCm = s_c[k].idleDeg = s_c[k].idleCm = 0.0f, s_c[k].fireN = s_c[k].idleN = 0;
}
#endif

// The gun's skeleton (2026-09-22). Steadying the gun's attach points held, and
// the gun on screen still shook, so the gun is drawn from something else - most
// likely its own model with its own bones, the kick an animation on them. A
// model is the same engine object as the character, so its joint array pointer
// should sit at the same offset (+0x318), and its joints in the same layout.
// Every object the character and the gun-hand attach points refer to is tried.
//
// Once found: every bone's local rotation (+0x20), local offset (+0x10 - a
// slide or a bolt moves by translation, not rotation) and position against the
// hand, added up frame to frame, split by whether a shot went off in the last
// 0.3 s. The root kick and the slide should come out on top, and differently.
namespace {
constexpr DWORD kOffModelJoints = 0x318;
constexpr int kMaxGunSkeletons = 4;
constexpr int kMaxGunJoints = 64;
struct GunSkeleton {
    unsigned char* object;
    unsigned char* joints;
    int count;
    char path[48];
    float prevQ[kMaxGunJoints][4], prevOff[kMaxGunJoints][3], prevRel[kMaxGunJoints][3];
    bool havePrev[kMaxGunJoints];
    float fireRot[kMaxGunJoints], fireOff[kMaxGunJoints], fireRel[kMaxGunJoints];
    float idleRot[kMaxGunJoints], idleOff[kMaxGunJoints], idleRel[kMaxGunJoints];
    int fireN, idleN;
    // Where each bone sits in your hand, and how long it has been somewhere
    // else. An animation is back where it started inside half a second; a new
    // grip is not, and that is the whole difference between the two.
    float holdRot[kMaxGunJoints][9], holdPos[kMaxGunJoints][3];
    unsigned long long offSince[kMaxGunJoints];
    unsigned long long offSinceAll;
    int anchor; // the bone that sits in your hand: the grip
    bool hold;
    // Where the grip has been sitting while we wait for it to stop moving, and
    // for how many frames running.
    float settleRel[3];
    int settleFrames;
    // How this particular weapon sat in the hand the last time it was held.
    // Learned once and then simply reapplied, so raising it again does not mean
    // working it out again.
    float learnedRot[9], learnedPos[3];
    int learnedAnchor;
    bool haveLearned;
};
GunSkeleton g_gunSk[kMaxGunSkeletons];
int g_gunSkCount = 0;

// How many joints from the start of `joints` look like real ones: a unit
// rotation, a parent that comes earlier (or is itself / 0xFF for a root), and a
// finished world matrix. Any joint within reach of the hand is required too.
int CountJointRun(unsigned char* joints, const float hand[3], float upm, bool* nearHand)
{
    *nearHand = false;
    int n = 0;
    for (int i = 0; i < kMaxGunJoints; ++i) {
        unsigned char e[kJointStride];
        if (!TryRead(e, joints + i * kJointStride, sizeof(e)))
            break;
        float q[4], w[16];
        std::memcpy(q, e + kOffJointLocalRot, sizeof(q));
        std::memcpy(w, e + kOffJointWorldMatrix, sizeof(w));
        const float ql = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!(std::fabs(ql - 1.0f) < 0.02f) || !(std::fabs(w[15] - 1.0f) < 1e-3f) || !std::isfinite(w[12]))
            break;
        const unsigned char parent = e[kOffJointLinks + 1];
        if (!(parent == 0xFF || parent <= i))
            break;
        const float d[3] = { w[12] - hand[0], w[13] - hand[1], w[14] - hand[2] };
        if (Length3(d) * 100.0f / upm < 100.0f)
            *nearHand = true;
        ++n;
    }
    return n;
}
} // namespace

// Every skeleton that is in the gun hand (2026-09-23). The hold needs this as
// much as the finder does: the skeleton hanging off the attach object at
// +0x318 turned out not to be the one the shaking mesh is drawn from. It was
// held perfectly still, it measured still at the very end of the frame, and
// the animation carried on playing on screen. So the hold takes every
// skeleton this search turns up, not just that one.
void DiscoverGunSkeletons(const ArmIkBody& body, const XrInputSettings& settings, const float hp[3],
    const float hr[9], float upm)
{
    const bool tell = settings.findGunBones;
    const unsigned long long nowMs = GetTickCount64();
    // Search: on first use, on raising the gun, and every 5 s while nothing is found.
    static unsigned long long s_searchMs = 0;
    static bool s_wasAiming = false;
    const bool aimEdge = body.aiming && !s_wasAiming;
    s_wasAiming = body.aiming;
    if ((aimEdge || g_gunSkCount == 0) && nowMs - s_searchMs > (g_gunSkCount ? 2000u : 5000u)) {
        s_searchMs = nowMs;
        // WHAT WE ALREADY KNEW ABOUT THESE WEAPONS (2026-09-25, user: "my gun is
        // still hopping when aiming. I thought we had this fixed already? Why
        // did it resurface?").
        //
        // Because of the next line, and it has been there the whole time. This
        // search re-runs on the aim edge, and it starts by throwing away every
        // weapon it knows about - then memsets each entry as it re-adds it. So
        // pressing aim wipes the grip, the hold, and the anchor bone, and the
        // gun has to be found and measured again from the middle of the ready
        // animation. That is the hop, it happens on the press AND the release,
        // and it is why learning the grip last build changed nothing: the
        // learned grip was being erased by the very event it was meant to serve.
        //
        // The user's framing settles what the right behaviour is: "the gun is
        // automatically in the correct position when it's equipped, aiming just
        // readies it, so it shouldn't move". Nothing about the weapon changes
        // when you raise it. The search is welcome to run - a different weapon
        // may have been drawn - but it has no business forgetting a weapon that
        // is still there.
        static GunSkeleton s_was[kMaxGunSkeletons];
        const int wasCount = g_gunSkCount;
        for (int s = 0; s < wasCount && s < kMaxGunSkeletons; ++s)
            s_was[s] = g_gunSk[s];
        g_gunSkCount = 0;
        // Candidate objects: every pointer in the character's first 12 KB, and
        // every pointer in the first 0x200 bytes of each attach object.
        static unsigned int cands[0x3000 / 4 + 12 * 0x80 + 12];
        int nc = 0;
        {
            static unsigned int fields[0x3000 / 4];
            std::memset(fields, 0, sizeof(fields));
            if (!TryRead(fields, body.character, sizeof(fields)))
                TryRead(fields, body.character, 0x1000);
            for (unsigned int f : fields)
                cands[nc++] = f;
        }
        for (int e = 0; e < 12; ++e) {
            unsigned int obj = 0;
            if (!TryRead(&obj, body.character + 0x21D0 + e * 0x30, sizeof(obj)) || obj < 0x10000u)
                continue;
            cands[nc++] = obj;
            unsigned int f2[0x80] = {};
            if (TryRead(f2, reinterpret_cast<void*>(static_cast<uintptr_t>(obj)), sizeof(f2)))
                for (unsigned int f : f2)
                    cands[nc++] = f;
        }
        int tried = 0;
        // Each distinct pointer once, remembering where it was first seen. Sorted
        // rather than compared pairwise: there are a few thousand of them.
        static unsigned long long order[0x3000 / 4 + 12 * 0x80 + 12];
        for (int k = 0; k < nc; ++k)
            order[k] = (static_cast<unsigned long long>(cands[k]) << 32) | static_cast<unsigned int>(k);
        std::sort(order, order + nc);
        for (int o = 0; o < nc && g_gunSkCount < kMaxGunSkeletons; ++o) {
            const unsigned int p = static_cast<unsigned int>(order[o] >> 32);
            const int k = static_cast<int>(order[o] & 0xFFFFFFFFu);
            if (o > 0 && static_cast<unsigned int>(order[o - 1] >> 32) == p)
                continue;
            if (p < 0x10000u || p >= 0x7FFF0000u || (p & 3u))
                continue;
            unsigned char* obj = reinterpret_cast<unsigned char*>(static_cast<uintptr_t>(p));
            unsigned char* joints = nullptr;
            if (!TryRead(&joints, obj + kOffModelJoints, sizeof(joints)) || !joints
                || reinterpret_cast<uintptr_t>(joints) < 0x10000)
                continue;
            ++tried;
            // A gun is not a person (2026-09-24, user: "Sheva's arm movement
            // makes Chris' leg and index finger move").
            //
            // This holds every joint of every skeleton it finds near your
            // wrist, rigidly, which is right for a weapon. A partner standing
            // beside you also has bones near your wrist, and once theirs is in
            // the list their legs and fingers are nailed to your hand and move
            // with it. The log had it in plain sight: four skeletons held in
            // one hand, four thousand bone writes every three seconds.
            //
            // It excluded exactly one other skeleton, the last non-player one
            // noticed, and with Majini about that is usually not the partner.
            // Now it excludes every character we know of by name, and refuses
            // anything the size of a person: a weapon has a handful of bones,
            // and a skeleton that runs to the sixty-four this search will count
            // is a body, not a gun.
            if (joints == body.joints || joints == g_otherJoints)
                continue;
            if (g_havePartnerBody && joints == g_partnerBody.joints)
                continue;
            if (g_haveBody && joints == g_body.joints)
                continue;
            bool nearHand = false;
            const int count = CountJointRun(joints, hp, upm, &nearHand);
            if (count < 2 || !nearHand)
                continue;
            // Person-sized, so not a gun. The count stops at kMaxGunJoints, so
            // a character's skeleton comes back saturated at exactly that.
            if (count >= kMaxGunJoints) {
                if (XrInput_GetSettings().findGunBones)
                    Log_Printf("GunBones: skipping %p - %d bones is a body, not a weapon", joints, count);
                continue;
            }
            bool again = false;
            for (int s = 0; s < g_gunSkCount && !again; ++s)
                again = g_gunSk[s].joints == joints;
            if (again)
                continue;
            GunSkeleton& sk = g_gunSk[g_gunSkCount++];
            std::memset(&sk, 0, sizeof(sk));
            sk.object = obj;
            sk.joints = joints;
            sk.count = count;
            // The same weapon as before keeps its grip, its hold and its anchor.
            // Matched on both the object and its joints, so a weapon that has
            // been swapped for another at the same address is not mistaken for
            // the one that was there.
            for (int w = 0; w < wasCount && w < kMaxGunSkeletons; ++w) {
                if (s_was[w].joints != joints || s_was[w].object != obj)
                    continue;
                sk.hold = s_was[w].hold;
                sk.anchor = s_was[w].anchor;
                std::memcpy(sk.holdRot, s_was[w].holdRot, sizeof(sk.holdRot));
                std::memcpy(sk.holdPos, s_was[w].holdPos, sizeof(sk.holdPos));
                std::memcpy(sk.offSince, s_was[w].offSince, sizeof(sk.offSince));
                sk.offSinceAll = s_was[w].offSinceAll;
                std::memcpy(sk.learnedRot, s_was[w].learnedRot, sizeof(sk.learnedRot));
                std::memcpy(sk.learnedPos, s_was[w].learnedPos, sizeof(sk.learnedPos));
                sk.learnedAnchor = s_was[w].learnedAnchor;
                sk.haveLearned = s_was[w].haveLearned;
                break;
            }
            if (k < 0x3000 / 4)
                sprintf_s(sk.path, "character->[+0x%X]", k * 4);
            else
                sprintf_s(sk.path, "an attach object's field");
            if (tell)
                Log_Printf("GunBones: %s (object %p) has a skeleton of %d joint(s) at %p, in the gun hand", sk.path,
                    obj, count, joints);
            for (int i = 0; tell && i < count && i < 24; ++i) {
                unsigned char e[kJointStride];
                if (!TryRead(e, joints + i * kJointStride, sizeof(e)))
                    break;
                float w[16], off[3];
                std::memcpy(w, e + kOffJointWorldMatrix, sizeof(w));
                std::memcpy(off, e + 0x10, sizeof(off));
                const float d[3] = { w[12] - hp[0], w[13] - hp[1], w[14] - hp[2] };
                float rel[3];
                for (int r = 0; r < 3; ++r)
                    rel[r] = Dot3(hr + r * 3, d) * 100.0f / upm;
                Log_Printf("GunBones:   joint %d (id %u, parent %u): %.1f %.1f %.1f cm from the hand in its axes, "
                           "local offset (%.2f %.2f %.2f)",
                    i, static_cast<unsigned>(e[kOffJointLinks + 3]), static_cast<unsigned>(e[kOffJointLinks + 1]),
                    rel[0], rel[1], rel[2], off[0], off[1], off[2]);
            }
        }
        if (!g_gunSkCount && tell)
            Log_Printf("GunBones: tried %d object(s) that had something at +0x318 - no skeleton in the gun hand", tried);
    }
}

#if RE5VR_DIAGNOSTICS
void FindGunBones(const ArmIkBody& body, const XrInputSettings& settings)
{
    const ArmIkArm& gun = settings.characterLeftHanded ? body.left : body.right;
    float hp[3], hr[9];
    if (!JointWorldPos(body.joints, gun.wrist, hp) || !WorldRot(body.joints, gun.wrist, hr))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    const unsigned long long nowMs = GetTickCount64();
    DiscoverGunSkeletons(body, settings, hp, hr, upm);

    // Movement per bone, firing against not.
    const unsigned long long shot = XrInput_LastShotMs();
    const bool firing = shot && nowMs - shot < 300;
    for (int s = 0; s < g_gunSkCount; ++s) {
        GunSkeleton& sk = g_gunSk[s];
        for (int i = 0; i < sk.count; ++i) {
            unsigned char e[kJointStride];
            if (!TryRead(e, sk.joints + i * kJointStride, sizeof(e)))
                continue;
            float q[4], off[3], w[16];
            std::memcpy(q, e + kOffJointLocalRot, sizeof(q));
            std::memcpy(off, e + 0x10, sizeof(off));
            std::memcpy(w, e + kOffJointWorldMatrix, sizeof(w));
            const float d[3] = { w[12] - hp[0], w[13] - hp[1], w[14] - hp[2] };
            float rel[3];
            for (int r = 0; r < 3; ++r)
                rel[r] = Dot3(hr + r * 3, d) * 100.0f / upm;
            if (sk.havePrev[i]) {
                float dot = std::fabs(q[0] * sk.prevQ[i][0] + q[1] * sk.prevQ[i][1] + q[2] * sk.prevQ[i][2]
                    + q[3] * sk.prevQ[i][3]);
                dot = dot > 1.0f ? 1.0f : dot;
                const float deg = 2.0f * std::acos(dot) * 57.2957795f;
                float dOff[3], dRel[3];
                Sub3(off, sk.prevOff[i], dOff);
                Sub3(rel, sk.prevRel[i], dRel);
                const float offCm = Length3(dOff) * 100.0f / upm;
                const float relCm = Length3(dRel);
                if (deg < 45.0f && relCm < 30.0f) {
                    if (firing) {
                        sk.fireRot[i] += deg;
                        sk.fireOff[i] += offCm;
                        sk.fireRel[i] += relCm;
                    } else {
                        sk.idleRot[i] += deg;
                        sk.idleOff[i] += offCm;
                        sk.idleRel[i] += relCm;
                    }
                }
            }
            std::memcpy(sk.prevQ[i], q, sizeof(q));
            std::memcpy(sk.prevOff[i], off, sizeof(off));
            std::memcpy(sk.prevRel[i], rel, sizeof(rel));
            sk.havePrev[i] = true;
        }
        if (firing)
            ++sk.fireN;
        else
            ++sk.idleN;
    }

    static unsigned long long s_toldMs = 0;
    if (nowMs - s_toldMs < 3000)
        return;
    s_toldMs = nowMs;
    for (int s = 0; s < g_gunSkCount; ++s) {
        GunSkeleton& sk = g_gunSk[s];
        if (sk.fireN > 5) {
            Log_Printf("GunBones: skeleton %p, per frame firing (%d frames) vs not (%d): rotation deg / local offset "
                       "cm / against the hand cm",
                sk.joints, sk.fireN, sk.idleN);
            for (int i = 0; i < sk.count && i < 24; ++i) {
                const float fr = sk.fireRot[i] / sk.fireN, fo = sk.fireOff[i] / sk.fireN, fh = sk.fireRel[i] / sk.fireN;
                const float ir = sk.idleN ? sk.idleRot[i] / sk.idleN : 0.0f;
                const float io = sk.idleN ? sk.idleOff[i] / sk.idleN : 0.0f;
                const float ih = sk.idleN ? sk.idleRel[i] / sk.idleN : 0.0f;
                Log_Printf("GunBones:   joint %d: firing %.2f / %.2f / %.2f, not %.2f / %.2f / %.2f", i, fr, fo, fh, ir,
                    io, ih);
            }
        }
        std::memset(sk.fireRot, 0, sizeof(sk.fireRot));
        std::memset(sk.fireOff, 0, sizeof(sk.fireOff));
        std::memset(sk.fireRel, 0, sizeof(sk.fireRel));
        std::memset(sk.idleRot, 0, sizeof(sk.idleRot));
        std::memset(sk.idleOff, 0, sizeof(sk.idleOff));
        std::memset(sk.idleRel, 0, sizeof(sk.idleRel));
        sk.fireN = sk.idleN = 0;
    }
}
#endif

// ---- How big the gun is drawn (2026-09-23) ------------------------------
// Testers found the weapons too big in a headset, and the user agreed. Each
// weapon is its own model hanging off the character's attachment table, with
// its own skeleton (see the gun-bones finder), and its root bone sits exactly
// at the hand - so scaling that bone keeps the grip in the palm and takes the
// size out of the gun ahead of it.
//
// Local only: this is the model on this machine, so a co-op partner sees the
// weapon at its normal size and nothing has to travel over the network. The
// scale field is not animated, so it is written only when it changes.
void ApplyGunScale(const ArmIkBody& body, float scale)
{
    if (!body.character || !body.joints)
        return;
    const ArmIkArm& gun = XrInput_GetSettings().characterLeftHanded ? body.left : body.right;
    float hp[3];
    if (!gun.valid || !JointWorldPos(body.joints, gun.wrist, hp))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
#if RE5VR_DIAGNOSTICS
    // It resized nothing on the first try and said nothing about why, so it
    // reports what it finds, what the scale field holds, and whether the value
    // it writes is still there a moment later (2026-09-23).
    static unsigned long long s_toldMs = 0;
    const bool tell = GetTickCount64() - s_toldMs >= 2000;
    if (tell)
        s_toldMs = GetTickCount64();
    int seen = 0, withBones = 0, inHand = 0;
#endif
    for (int e = 0; e < 12; ++e) {
        unsigned char* object = nullptr;
        if (!TryRead(&object, body.character + 0x21D0 + e * 0x30, sizeof(object))
            || reinterpret_cast<uintptr_t>(object) < 0x10000)
            continue;
#if RE5VR_DIAGNOSTICS
        ++seen;
#endif
        unsigned char* joints = nullptr;
        if (!TryRead(&joints, object + 0x318, sizeof(joints)) || reinterpret_cast<uintptr_t>(joints) < 0x10000)
            continue;
#if RE5VR_DIAGNOSTICS
        ++withBones;
#endif
        // Only what is in the gun hand: the root bone within arm's reach of the
        // wrist. A strap or anything else hanging off the body is left alone.
        float w[16];
        if (!TryRead(w, joints + kOffJointWorldMatrix, sizeof(w)) || std::fabs(w[15] - 1.0f) > 1e-3f)
            continue;
        const float d[3] = { w[12] - hp[0], w[13] - hp[1], w[14] - hp[2] };
        const float away = Length3(d) * 100.0f / upm;
        if (away > 25.0f)
            continue;
#if RE5VR_DIAGNOSTICS
        ++inHand;
#endif
        // The root and the bone under it, which is the one the rest hangs from.
        for (int j = 0; j < 2; ++j) {
            float sc[3];
            if (!TryRead(sc, joints + j * kJointStride + kOffJointScale, sizeof(sc)))
                break;
            const bool sane = sc[0] > 0.05f && sc[1] > 0.05f && sc[2] > 0.05f && sc[0] < 20.0f;
            const float want[3] = { scale, scale, scale };
            bool wrote = false, held = false;
            if (sane
                && (std::fabs(sc[0] - scale) > 0.002f || std::fabs(sc[1] - scale) > 0.002f
                    || std::fabs(sc[2] - scale) > 0.002f)) {
                wrote = TryWrite(joints + j * kJointStride + kOffJointScale, want, sizeof(want));
                float back[3] = {};
                held = wrote && TryRead(back, joints + j * kJointStride + kOffJointScale, sizeof(back))
                    && std::fabs(back[0] - scale) < 0.002f;
            }
#if RE5VR_DIAGNOSTICS
            if (tell && j == 0) {
                Log_Printf("GunScale: entry %d, bones at %p, %.0f cm from the hand - joint %d scale was "
                           "(%.3f %.3f %.3f), wanted %.2f, %s%s. The world matrix's own row lengths are "
                           "%.3f %.3f %.3f",
                    e, joints, away, j, sc[0], sc[1], sc[2], scale,
                    !sane ? "left alone (not a sane scale)" : (wrote ? "written" : "already there"),
                    wrote ? (held ? " and it stayed" : " but it did not stay") : "",
                    std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]),
                    std::sqrt(w[4] * w[4] + w[5] * w[5] + w[6] * w[6]),
                    std::sqrt(w[8] * w[8] + w[9] * w[9] + w[10] * w[10]));
            }
#endif
        }
    }
#if RE5VR_DIAGNOSTICS
    if (tell && !inHand) {
        Log_Printf("GunScale: %d attachment entr(ies), %d with bones, none within 25 cm of the gun hand - nothing "
                   "to resize",
            seen, withBones);
    }
#endif
}

void ArmIk_SetBody(const ArmIkBody& body)
{
    ArmIkSolveLock lock;
    const XrInputSettings settings = XrInput_GetSettings();

    // The finder lives here rather than in the solve: it wants arming once,
    // whether or not anything is driving the arms.
    //
    // Pointed at the LOCAL ROTATION now (2026-09-17). The pose write lands at
    // the top of each build pass - 66 to 92 a second, so it is running - and the
    // animation still snaps the arm back, which means it sets the same field
    // after we do. Whoever that is has to be hooked, so find it: the elbow,
    // because its rotation swings furthest, and writes only, since a read watch
    // on a joint this hot buries the log.
    // Armed by the aim button, not by the clock (2026-09-18). The window is four
    // seconds and it used to start the moment the setting was noticed, which
    // meant catching the press by hand inside it. The aim flag is right here, so
    // every press and every release opens its own window and the interesting
    // moment is always inside one.
    if (settings.findArmWriter && body.joints && body.right.valid) {
        static bool s_armedFor = false;
        static bool s_everArmed = false;
        const bool transition = body.aiming != s_armedFor;
        if (transition || !s_everArmed) {
            s_armedFor = body.aiming;
            s_everArmed = true;
            if (transition)
                AimFinder_Rearm();
            // The W component, not the whole thing (2026-09-18). The animation
            // stores the quaternion one float at a time, so watching the first
            // one found the instruction after x landed - and writing there let
            // the animation overwrite y, z and w, which is what produced a
            // quaternion of length 1.09 and an arm stretched to match. Watching
            // the last component instead finds the instruction where a whole
            // pose has just been set and nothing has read it yet.
            AimFinder_Start(body.joints + body.right.elbow * kJointStride + kOffJointLocalRot + 12,
                body.aiming ? "the arm pose while the gun comes up" : "the arm pose while the gun goes down");
        }
    }

    // GUN SIZE IS A PLAYER SETTING (2026-09-29, user: "gun scale should be
    // available on release").
    //
    // It sits on the VR tab with a slider and a help note, and it was inside
    // the diagnostics guard with the grip and shake finders - so in a
    // release build the slider moved and nothing happened. The finders on
    // either side of it stay guarded; they are behind their own dev
    // checkboxes and do real scanning work.
    {
        // Back to the game's own size once the slider returns to 1, which is
        // why this keeps running after it has been set back.
        static bool s_scaled = false;
        const float want = settings.gunScale < 0.5f ? 0.5f : (settings.gunScale > 1.5f ? 1.5f : settings.gunScale);
        if (std::fabs(want - 1.0f) > 0.005f || s_scaled) {
            ApplyGunScale(body, want);
            s_scaled = std::fabs(want - 1.0f) > 0.005f;
        }
    }
#if RE5VR_DIAGNOSTICS
    if (settings.measureGrips && body.joints && body.character && body.left.valid && body.right.valid)
        MeasureGrip(body, settings);
    if (settings.findGunBones && body.joints && body.character && body.left.valid && body.right.valid)
        FindGunBones(body, settings);
    if (settings.measureShake && body.joints && body.left.valid && body.right.valid) {
        MeasureShake(body, settings);
    }
#endif

#if RE5VR_DIAGNOSTICS
    // Not behind a checkbox (2026-09-17). It was, next to four others with
    // similar names, and the run came back with the wrong one ticked and no
    // scan at all. It costs one pass over the skeleton every two seconds and it
    // stops on its own after a minute of play, so it can simply run.
    if ((settings.armIk || settings.armIkTest) && body.joints && body.character && body.right.valid
        && body.jointCount > 8) {
        static unsigned long long s_toldMs = 0;
        static int s_reports = 0;
        const unsigned long long nowMs = GetTickCount64();
        ReportPoseHeld(body);
        ReportBindPose(body);
        ReportAttachments(body);
        if (s_reports < 30 && nowMs - s_toldMs >= 2000) {
            s_toldMs = nowMs;
            ++s_reports;
            ReportLocalPose(body);
            ReportWhatHoldsTheGun(body);
        }
    }
#endif

    // Four joints, four debug registers - and which four changed on 2026-09-18.
    //
    // It was both shoulders and both elbows. The drift report then measured 0.000
    // on all four while aiming, which says the whole-arm blend is caught by any
    // one of them: a trap on the elbow puts the shoulder back too, because every
    // held joint is restored on every trap. So a shoulder does not need a
    // register of its own to be safe - it only needs to be in the held list, and
    // it stays there.
    //
    // The weapon socket does. It is written by the aim layer, it is what the gun
    // hangs off, and nothing was watching it - which is the user's "the gun does
    // not stay in your hand while aiming", with the arm itself tracking perfectly
    // either side of it. Spend the two freed registers there.
    // Only while the player is actually driving their own arms (2026-09-20).
    // A flatscreen player builds a body now, because that is how the partner's
    // solve is reached - but they are not driving their own arms, so arming
    // four hardware watchpoints on their joints buys nothing and costs a debug
    // trap on every animation write to them.
    // Four debug registers, shared out by who actually needs them
    // (2026-09-24). There are only four in the processor, so guarding two
    // skeletons fully is not on offer and the question is how to spend them.
    //
    // Driving both: two elbows each. The elbow is where the animation fights
    // hardest - measured, the arm pose was lost by 1.4 cm a frame while firing
    // against 0.04 when not - so it is the one joint worth a register apiece.
    //
    // Driving only one of them: that one gets all four, elbows and then the
    // sockets, falling back to the wrists on rigs with no socket. A flatscreen
    // player watching a VR partner drives nobody's arms but theirs, so every
    // register goes to the partner - which until today got none at all.
    {
        const bool driveMine = settings.armIk && settings.armIkWriteLocal && body.joints && body.left.valid
            && body.right.valid;
        const bool driveTheirs = g_havePartnerBody && g_partnerBody.joints && g_partnerBody.left.valid
            && g_partnerBody.right.valid && g_poseQuatMs[kPartnerBody]
            && GetTickCount64() - g_poseQuatMs[kPartnerBody] < 500;
        PoseGuardSlot slots[4] = {};
        int used = 0;
        const auto add = [&](unsigned char* joints, int a, int b) {
            if (used < 4 && a >= 0)
                slots[used++] = { joints, a };
            if (used < 4 && b >= 0)
                slots[used++] = { joints, b };
        };
        if (driveMine && driveTheirs) {
            add(body.joints, body.left.elbow, body.right.elbow);
            add(g_partnerBody.joints, g_partnerBody.left.elbow, g_partnerBody.right.elbow);
        } else if (driveMine) {
            add(body.joints, body.left.elbow, body.right.elbow);
            add(body.joints, g_socket[0] >= 0 ? g_socket[0] : body.left.wrist,
                g_socket[1] >= 0 ? g_socket[1] : body.right.wrist);
        } else if (driveTheirs) {
            add(g_partnerBody.joints, g_partnerBody.left.elbow, g_partnerBody.right.elbow);
            add(g_partnerBody.joints, g_partnerBody.left.wrist, g_partnerBody.right.wrist);
        }
        if (used > 0)
            PoseGuard_Set(slots, used, kJointStride, kOffJointLocalRot);
        else
            PoseGuard_Clear();
#if RE5VR_DIAGNOSTICS
        {
            static int s_toldUsed = -1;
            static bool s_toldMine = false, s_toldTheirs = false;
            if (used != s_toldUsed || driveMine != s_toldMine || driveTheirs != s_toldTheirs) {
                s_toldUsed = used;
                s_toldMine = driveMine;
                s_toldTheirs = driveTheirs;
                Log_Printf("PoseGuard: holding %d joint(s) - yours %s, theirs %s", used,
                    driveMine ? "yes" : "no", driveTheirs ? "yes" : "no");
            }
        }
#endif
    }

    // Aim starting or stopping hands the arm to different code, so anything we
    // are carrying about the pose stops being true at that instant. Clearing it
    // here is what the user was doing by hand - toggling 6DOF off and on to get
    // the arms back - and the wrist tie is retaken from the same moment.
    // The grip probe. Runs whenever a tie is missing - first solve, after a
    // calibration, or after a weapon swap clears it.
    if (body.joints && body.left.valid && body.right.valid) {
        const LONG phase = g_gripProbe;
        if (phase == 2) {
            // The world matrices were built from the animation's own locals this
            // time, so what is in the joints now is the game's answer.
            for (int h = 0; h < 2; ++h) {
                const ArmIkArm& a4 = h == 0 ? body.left : body.right;
                const int sk = g_socket[h];
                if (g_haveSocketTie[h] || sk < 0 || !a4.valid)
                    continue;
                float sw2[9], hw2[9];
                if (WorldRot(body.joints, sk, sw2) && WorldRot(body.joints, a4.wrist, hw2)) {
                    float hwT[9];
                    Transpose3(hw2, hwT);
                    Mul3(sw2, hwT, g_socketTie[h]);
                    g_haveSocketTie[h] = true;
                    Log_Printf("ArmIk: the %s grip, straight from the animation - socket row0 (%.2f %.2f %.2f) "
                               "against hand row0 (%.2f %.2f %.2f)",
                        h == 1 ? "right" : "left", sw2[0], sw2[1], sw2[2], hw2[0], hw2[1], hw2[2]);
                }
            }
            InterlockedExchange(&g_gripProbe, 0);
        } else if (phase == 1) {
            InterlockedExchange(&g_gripProbe, 2);
        } else if (false) {
            // The guard would put our own pose straight back over the animation's,
            // which is the whole thing we are trying to get out of the way.
            PoseGuard_Clear();
            InterlockedExchange(&g_gripProbe, 1);
        }
    }

    g_aimingNow = body.aiming;
    static bool s_wasAiming = false;
    if (body.aiming != s_wasAiming) {
        s_wasAiming = body.aiming;
        // AN AIM PRESS IS NOT A NEW ARM (2026-09-25, and this was the third
        // place clearing the rest length without saying so - the one the trap
        // caught, 36 times in five minutes).
        //
        // Two lines here used to throw the measured arm length away on every
        // press AND every release of the aim button. Dozens of times a minute,
        // each one forcing a cold start that has to let go of the arm and
        // measure it again - and if the gun is on its way up while that happens,
        // it measures a moving arm and writes the answer down as the truth. That
        // is the 71.4 from the last log, and it is why aiming has never been
        // quite steady after a transition.
        //
        // An arm's resting length is a property of the skeleton. It does not
        // change because you raised a gun. Nothing needs re-measuring here, and
        // if the arm ever genuinely is a different one the ten per cent test
        // notices within two seconds and the heal deals with it properly, with
        // the animation actually holding the arm while it measures.
        // Only on the way UP (2026-09-18). Clearing it on the way down too
        // meant the tie could not be retaken until the next press, so the hand
        // stopped following the moment the gun came down - which is not what
        // "always" means. Raising the gun retakes it; lowering keeps it.
        // A calibrated tie is not retaken. It was measured against the bind
        // pose, which is the same today as it was when the gun last came up, so
        // there is nothing for a retake to improve and everything for it to get
        // wrong (2026-09-18).
        if (body.aiming) {
            for (int h = 0; h < 2; ++h) {
                if (g_tiesCalibrated[h])
                    continue;
                g_haveWristOffset[h] = false;
                g_haveArmOffset[h] = false;
            }
        }
        if (body.aiming)
            g_aimUpMs = GetTickCount64();
        g_aimEdgeMs = GetTickCount64();
        Log_Printf("ArmIk: %s - retaking the arm as it is now", body.aiming ? "aim up" : "aim down");
    }

    g_body = body;
    // A different character means a different body, and everything measured
    // about the last one is now wrong (2026-09-20). Sheva is about 0.93 of
    // Chris, so carrying his units-per-metre over to her reads her arms as
    // roughly seven per cent longer than they are, and the rest-pose ties were
    // made against his skeleton rather than hers. None of that announces
    // itself: it just looks like the IK being slightly off, which is the worst
    // kind of wrong to debug. So the calibration goes with the character.
    //
    // BUT only for a different BODY (2026-09-22). The game builds a new
    // character object for the same Chris behind every loading screen, so
    // keying this on the object meant a level change threw the T-pose away.
    // Bones do not change length, so the arm's length says whether this is the
    // same body: Sheva's is about 7 per cent shorter than Chris's, a reload
    // of either is identical. Only the things tied to the old joint array go.
    static float s_bodyArm = 0.0f;
    float armNow = 0.0f;
    {
        float s[3], e[3], w[3];
        if (body.joints && body.right.valid && JointWorldPos(body.joints, body.right.shoulder, s)
            && JointWorldPos(body.joints, body.right.elbow, e) && JointWorldPos(body.joints, body.right.wrist, w)) {
            float a[3], b[3];
            Sub3(e, s, a);
            Sub3(w, e, b);
            armNow = Length3(a) + Length3(b);
        }
    }
    // A body that is not built yet reads as nothing; wait for it.
    const bool measurable = armNow > 5.0f;
    // AND the same skeleton layout (2026-09-22). A costume can be a different
    // rig with arms of exactly the same length - 138 joints became 130 and the
    // arms moved from 36/41/45 to 77/79/81 - and its bones rest at different
    // angles, so the T-pose ties no longer fit and aiming broke. Keeping the
    // calibration is only for a reload of the same rig, which is what a
    // loading screen is; a new layout clears it, as a new character does.
    static int s_layout[7] = {};
    static unsigned char* s_layoutJoints = nullptr;
    const int layoutNow[7] = { body.jointCount, body.left.shoulder, body.left.elbow, body.left.wrist,
        body.right.shoulder, body.right.elbow, body.right.wrist };
    const bool sameLayout = std::memcmp(layoutNow, s_layout, sizeof(layoutNow)) == 0;
    const bool newSkeleton = s_layoutJoints && body.joints != s_layoutJoints;
    const bool bodyChanged = (body.character && body.character != g_lastCharacter) || newSkeleton;
    if (bodyChanged && measurable && s_bodyArm > 5.0f && sameLayout
        && std::fabs(armNow - s_bodyArm) < s_bodyArm * 0.02f) {
        g_lastCharacter = body.character;
        for (int h = 0; h < 2; ++h) {
            const int slot = kPlayerSlot + h;
            g_haveSocketTie[slot] = false;
            g_socket[slot] = -1;
            g_foreAxis[slot] = -1;
            g_foreSign[slot] = 1.0f;
        }
        std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
        g_poseQuatMs[kPlayerBody] = 0;
        ++g_cacheCleared;
        g_lastClearSite = __LINE__;
        Log_Printf("ArmIk: the same body again (arm %.1f units) - keeping your calibration", armNow);
    }
    // Taken before s_bodyArm is overwritten a few lines down - comparing it with
    // itself is why "keeping your scale" never once printed.
    const float armBefore = s_bodyArm;
    const bool clearNow = measurable && (body.character != g_lastCharacter || (newSkeleton && !sameLayout));
    if (measurable) {
        s_bodyArm = armNow;
        std::memcpy(s_layout, layoutNow, sizeof(s_layout));
        s_layoutJoints = body.joints;
    }
    if (body.character && clearNow) {
        // A different rig may store its rotations the other way round too.
        g_quatConvention = -1;
        g_boneInRow = -1;
        const bool wasCalibrated = g_tiesCalibrated[kPlayerSlot] || g_calUnitsPerMetre[kPlayerSlot] > 1.0f
            || g_calUnitsPerMetre[kPlayerSlot + 1] > 1.0f;
        // A COSTUME DOES NOT CHANGE HOW LONG YOUR ARM IS (2026-09-25, user: "when
        // I press the aim button, it's allowing me to aim, it just breaks the
        // IK", after a costume change).
        //
        // The log measures it. For the eight seconds between the costume change
        // and the user re-posing, the hands were short of where they were being
        // pointed by seven to eleven units, every frame:
        //
        //   short by 0.0/20.2 ... 9.6/11.8 ... 8.9/10.8 ... 7.9/10.9 ... 6.7/9.2
        //
        // and then 0.0/0.0 the moment they calibrated again. Ten units is about
        // thirteen centimetres of arm that will not go where you put it, which
        // is not subtle with a gun up.
        //
        // The cause is this loop throwing away the SCALE along with everything
        // else. Scale is your arm in metres against the character's in units,
        // and the log says the character's arm was 53.2 units before the costume
        // and 53.2 after. Nothing about that number changed; only the joint
        // indices did. Dropping it falls back to an assumed 85.8 units a metre
        // against a measured 80.8, and six per cent of an arm is exactly the ten
        // units that went missing.
        //
        // So the ties go, because those are indices into an array that has
        // genuinely been rebuilt and rotations against a rest pose that may sit
        // differently - that is real and it is why a T-pose is still wanted. The
        // scale stays, as long as the character's arm is the same length it was.
        // And judged on the ARM, not on the character pointer. That pointer
        // changes across a costume change too, so requiring it to match meant
        // the scale was dropped every single time - zero "keeping your scale"
        // lines in a whole session. The scale is your metres against this
        // character's units, so the only thing that can invalidate it is the
        // character's arm being a different length, whoever they are.
        const bool keepScale = armBefore > 5.0f && armNow > 5.0f
            && std::fabs(armNow - armBefore) < armBefore * 0.05f;
        g_lastCharacter = body.character;
        for (int h = 0; h < 2; ++h) {
            const int slot = kPlayerSlot + h;
            if (!keepScale) {
                g_calUnitsPerMetre[slot] = 0.0f;
                g_calHaveHand[slot] = false;
                g_haveShoulderFromHead[slot] = false;
            }
            g_tiesCalibrated[slot] = false;
            g_haveWristOffset[slot] = false;
            g_haveArmOffset[slot] = false;
            g_haveCtrlRef[slot] = false;
            g_haveSocketTie[slot] = false;
            g_socket[slot] = -1;
            g_foreAxis[slot] = -1;
            g_foreSign[slot] = 1.0f;
        }
        if (keepScale) {
            Log_Printf("ArmIk: same character, same %.1f unit arm - keeping your scale (%.1f units a metre) "
                       "through the costume change; only the joint ties are being retaken",
                armNow, g_calUnitsPerMetre[kPlayerSlot]);
        }
        std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
        g_poseQuatMs[kPlayerBody] = 0;
        ++g_cacheCleared;
        g_lastClearSite = __LINE__;
        if (wasCalibrated) {
            // And say so where the player can see it (2026-09-25, user asked for
            // a prompt on a costume change). Until now this only went to the log,
            // so the arms quietly stopped fitting and there was nothing on screen
            // to say why or what to do about it.
            g_costumeChangedMs = GetTickCount64();
            Log_Printf("ArmIk: this is a different character or costume (a new skeleton layout), so the calibration "
                       "is cleared. T-pose again - "
                       "the old measurements were taken against the last one's arms.");
        }
    }

    // And the mirror of it: if our own body turns out to be the one we have
    // been driving as the partner, the partner is the stale one and goes.
    if (body.joints && g_havePartnerBody
        && (body.joints == g_partnerBody.joints || body.character == g_partnerBody.character)) {
        Log_Printf("ArmIk: our own body is the one we were driving as the partner - dropping the partner");
        g_havePartnerBody = false;
        std::memset(g_poseQuatValid[kPartnerBody], 0, sizeof(g_poseQuatValid[kPartnerBody]));
        g_poseQuatMs[kPartnerBody] = 0;
    }
    g_haveBody = body.joints != nullptr && body.character != nullptr;
    g_bodyMs = GetTickCount64();
    if (g_haveBody && body.left.valid && body.right.valid && g_neededFor != body.joints) {
        g_neededFor = body.joints;
        BuildNeededSet(body);
        // A new joint array under the same character is a costume change
        // (2026-09-22): the calibration is still good, but anything that
        // names a joint by its index in the old array now names a different
        // bone, or one past the end.
        for (int h = 0; h < 2; ++h) {
            const int slot = kPlayerSlot + h;
            g_haveSocketTie[slot] = false;
            g_socket[slot] = -1;
            g_foreAxis[slot] = -1;
            g_foreSign[slot] = 1.0f;
        }
        std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
        g_poseQuatMs[kPlayerBody] = 0;
        ++g_cacheCleared;
        g_lastClearSite = __LINE__;
        PoseGuard_Clear();
        // Everything below is keyed to the old array: which joints have been
        // built this frame, and a cached pose for indices that now mean other
        // bones. Carrying either over is how the arm ended up frozen in a pose
        // nothing was updating any more.
        std::memset(g_seen, 0, sizeof(g_seen));
        g_seenCount = 0;
        g_neededSeen = 0;
        g_solvedThisFrame = false;
        std::memset(g_keySeen, 0, sizeof(g_keySeen));
        g_solvePending = false;
        // A rotation quaternion has length 1. Anything else builds a matrix with
        // scale in it, and scale inherits all the way down a limb - which is why
        // the hands swell worst and why they stayed swollen after aiming: the
        // bad value sits in the joint and every later blend carries it forward.
        // Whatever put it there, it is never right, so put it back on the unit
        // sphere once a frame before anything composes from it.
        if (settings.armIkWriteLocal) {
            for (int k = 0; k < 6; ++k) {
                if (g_key[k] < 0)
                    continue;
                unsigned char* at = g_body.joints + g_key[k] * kJointStride + kOffJointLocalRot;
                float q[4];
                if (!TryRead(q, at, sizeof(q)))
                    continue;
                const float l = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                if (!(l > 0.1f) || std::fabs(l - 1.0f) < 0.001f)
                    continue;
                for (int i = 0; i < 4; ++i)
                    q[i] /= l;
                TryWrite(at, q, sizeof(q));
                ++g_renormalised;
            }
        }
        std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
        g_poseQuatMs[kPlayerBody] = 0;
        ++g_cacheCleared;
        g_lastClearSite = __LINE__;
    }

    // Fall back to solving here when the skeleton hook is not running - either
    // it failed to install or it has been turned off. The arms still follow;
    // only the gun is left behind.
    //
    // And when it IS running but has gone quiet (2026-09-17). The late path only
    // fires once it has watched every joint of both arms be built, and the game
    // does not always rebuild all of them - with the first version of this the
    // user found the arms only tracked while the aim button was held, which is
    // presumably the state that forces a full update. An arm that stops dead is
    // far worse than a gun that lags, so anything older than a fifth of a second
    // is treated as the late path having failed for this frame.
#if RE5VR_DIAGNOSTICS
    // Silence has meant three different things tonight - no player, no arms,
    // and a solve that quietly stopped after the character was rebuilt - and
    // each time it took a log-reading round trip to tell them apart. So say it.
    if (settings.armIk || settings.armIkTest) {
        static unsigned long long s_quietMs = 0;
        const unsigned long long nowMs = GetTickCount64();
        const bool solvingRecently = g_statusMs && nowMs - g_statusMs < 1000;
        if (!solvingRecently && nowMs - s_quietMs >= 2000) {
            s_quietMs = nowMs;
            Log_Printf("ArmIk: not solving - body %s, arms %s, skeleton hook %s, last build solve %llu ms "
                       "ago, pose cache %s",
                g_haveBody ? "yes" : "NO", (body.left.valid && body.right.valid) ? "yes" : "NO",
                ArmIk_LateWriteActive() ? "on" : "off",
                g_lateSolveMs ? nowMs - g_lateSolveMs : 0ull, g_poseQuatMs[kPlayerBody] ? "held" : "empty");
        }
    }
#endif

    // Driving the pose: solve here, every frame, on a skeleton that has
    // finished being built. The quaternions are cached and go in later, when the
    // animation has set its own and nothing has composed from them yet.
    if (settings.armIkWriteLocal) {
        ++g_lateSolves;
        g_lateSolveMs = GetTickCount64();
        RunSolve(body);
        SolvePartner();
        return;
    }
    const bool lateFresh = g_lateSolveMs && GetTickCount64() - g_lateSolveMs < 200;
    if (!ArmIk_LateWriteActive() || !lateFresh) {
        if (ArmIk_LateWriteActive())
            ++g_fallbackSolves;
        RunSolve(body);
    }
    // The partner goes last, so that whatever happens to their arms cannot
    // leave g_solvingBody pointing away from the player for the next frame.
    SolvePartner();
}

void ArmIk_OnJointBuilt(unsigned char* joint)
{
    ++g_jbCalls;
    if (!g_haveBody || !joint)
        return;
    const XrInputSettings settings = XrInput_GetSettings();
    if (!settings.armIk && !settings.armIkTest)
        return;
    // Ours? The build walks every character in the scene through here, so the
    // joint has to land inside the player's own array, on a joint boundary.
    const ptrdiff_t offset = joint - g_body.joints;
    const int span = g_body.jointCount * kJointStride;
    if (offset < 0 || offset >= span || (offset % kJointStride) != 0) {
        ++g_jbNotOurs;
        return;
    }
    const int index = static_cast<int>(offset / kJointStride);
    if (index < 0 || index >= kMaxJoints)
        return;

    if (g_seen[index]) {
        // Seen twice: the frame turned over, so the animation has finished
        // setting the locals and the build has only just started reading them.
        // That is the one moment the pose can be changed and still be used.
        // What the LAST frame managed, not the most any frame ever has
        // (2026-09-18). One unusual frame that rebuilt 54 arm joints raised the
        // bar above the 53 a normal frame delivers, and the build-time solve
        // never fired again - the log went to "solved 0 in the build".
        if (g_neededSeen > 0)
            g_neededPerFrame = g_neededSeen;
        std::memset(g_seen, 0, sizeof(g_seen));
        g_seenCount = 0;
        g_neededSeen = 0;
        g_solvedThisFrame = false;
        // The animation has finished and the build has not composed anything
        // yet, so this is the one point where a whole quaternion can be put in
        // and be used. Writing at the animation's own store was worse than
        // useless: it stores the four floats one at a time, so ours landed on
        // the first and the animation overwrote the other three - leaving a
        // quaternion of length 1.09 and a matrix scaled to match, which is what
        // was stretching the arm by a fifth.
        if (settings.armIkWriteLocal && g_poseQuatMs[kPlayerBody]) {
            if (GetTickCount64() - g_poseQuatMs[kPlayerBody] > 500) {
                // The solve has stopped - a cutscene, or the arms turned off.
                // Give the animation its arm back rather than hold the last pose.
                std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
                g_poseQuatMs[kPlayerBody] = 0;
                ++g_cacheCleared;
                g_lastClearSite = __LINE__;
            } else if (g_poseQuatJoints[kPlayerBody] != g_body.joints) {
                // Whose pose is in there? (2026-09-24.) This loop is the write
                // that reaches the screen with 'Six degrees of freedom drives
                // the pose' on, and it had no ownership check of any kind: it
                // walked the cache and wrote every valid entry at g_body.joints
                // BY INDEX.
                //
                // So on a frame where the player's cache had been worked out on
                // the partner's skeleton - which the logs show happening for
                // five and a half minutes straight - this put their arm
                // rotations onto your character, joint number for joint number,
                // across the whole body rather than the arms. Their rig has 147
                // joints and yours 130, so joint 79 is an arm bone on one and
                // something else entirely on the other. That is the user's
                // "Sheva's arm movement makes Chris' leg and index finger move"
                // exactly, and it is why the fighting looked like it involved
                // bones no arm solve ever touches.
                //
                // The check added at the other write site did not cover this
                // one, and this is the one that was doing the damage.
                std::memset(g_poseQuatValid[kPlayerBody], 0, sizeof(g_poseQuatValid[kPlayerBody]));
                g_poseQuatMs[kPlayerBody] = 0;
                ++g_cacheCleared;
                g_lastClearSite = __LINE__;
                g_poseQuatJoints[kPlayerBody] = nullptr;
                ++g_crossRefused[kPlayerBody];
#if RE5VR_DIAGNOSTICS
                static unsigned long long s_toldBuild = 0;
                const unsigned long long nowBuild = GetTickCount64();
                if (nowBuild - s_toldBuild >= 2000) {
                    s_toldBuild = nowBuild;
                    Log_Printf("ArmIk: the build pass was about to put a pose worked out for %p onto your "
                               "skeleton at %p - dropped it",
                        g_poseQuatJoints[kPlayerBody], g_body.joints);
                }
#endif
            } else {
                for (int i = 0; i < kMaxJoints && i < g_body.jointCount; ++i) {
                    if (g_poseQuatValid[kPlayerBody][i])
                        TryWrite(g_body.joints + i * kJointStride + kOffJointLocalRot,
                            g_poseQuat[kPlayerBody][i], sizeof(g_poseQuat[kPlayerBody][i]));
                }
            }
        }
    }
    g_seen[index] = true;
    ++g_seenCount;
    for (int k = 0; k < 6; ++k) {
        if (g_key[k] == index)
            g_keySeen[k] = true;
    }
    if (g_needed[index]) {
        ++g_neededSeen;
        if (g_neededSeen > g_neededSeenHigh)
            g_neededSeenHigh = g_neededSeen;
    }
    // On the last arm joint of the frame, where the whole arm has just been
    // composed and its world matrices agree with each other. Solving at the top
    // of a pass instead read a mixture of this frame and the last, and the arm's
    // own bone lengths came out different every frame - 53.2 became 54 to 58 -
    // which is measuring a skeleton that does not exist.
    // Nothing is solved in here any more (2026-09-18). Reading the skeleton
    // mid-build never became safe: a joint's POSITION is the last sixteen bytes
    // of its matrix and this hook fires after the first four, so however long
    // the solve was deferred, some joint was always part-written. The right arm
    // came good at 53.2 and the left went on reading 62 to 68. The camera hook
    // reads a settled skeleton - it is where the world-matrix mode read from,
    // and that never misbehaved - so the solve lives there and only the WRITE
    // happens on the animation's schedule.
    (void)g_solvePending;
}

bool ArmIk_LateWriteActive()
{
    // Writing the local pose has to happen between the animation setting it and
    // the game composing from it, and the only place that boundary is visible is
    // the first joint of a build. So that mode needs the skeleton hook too.
    const XrInputSettings s = XrInput_GetSettings();
    return (s.armIkLateWrite || s.armIkWriteLocal) && g_skeletonHooked;
}

void ArmIk_OnPoseComplete(unsigned char* joint)
{
#if RE5VR_DIAGNOSTICS
    // And which threads the animation writes poses from, for the same reason.
    {
        static unsigned long s_poseThreads[8] = {};
        static int s_poseThreadCount = 0;
        const unsigned long tid = GetCurrentThreadId();
        bool known = false;
        for (int t = 0; t < s_poseThreadCount; ++t)
            known = known || s_poseThreads[t] == tid;
        if (!known && s_poseThreadCount < 8) {
            s_poseThreads[s_poseThreadCount++] = tid;
            Log_Printf("ArmIk: the animation finished a pose on thread %lu (%d thread(s) so far)", tid,
                s_poseThreadCount);
        }
    }
#endif
    // Straight after the animation has stored all four floats of this joint's
    // rotation, and before anything has composed from it. The only window that
    // reaches the screen: writing at the build was too late, because the build
    // turned out to copy an already-finished matrix rather than compose one, and
    // writing after the FIRST float let the animation overwrite three of ours.
    ++g_poseCalls;
    if (!joint) {
        ++g_poseNoJoint;
        return;
    }
    // armIkWriteLocal is the player's own choice about how THEIR arms are
    // written, so it cannot decide whether a partner's pose lands. Checked per
    // body below instead (2026-09-20).
    const bool writeLocal = XrInput_GetSettings().armIkWriteLocal;

    // Which character's joint is this? The hook fires for every skeleton the
    // animation touches, so the pointer decides (2026-09-19). It used to test
    // only the player's array and return; now the partner's is a second range,
    // and a joint belonging to neither is somebody we are not driving.
    int whose = -1;
    int index = -1;
    const struct {
        int body;
        unsigned char* joints;
        int count;
    } bodies[kArmBodies] = {
        { kPlayerBody, g_haveBody ? g_body.joints : nullptr, g_body.jointCount },
        { kPartnerBody, g_havePartnerBody ? g_partnerBody.joints : nullptr, g_partnerBody.jointCount },
    };
    // The NEAREST array below this joint, not the first one that happens to
    // contain it (2026-09-24, user: "we still are hooking into each other").
    //
    // This took the first body whose range contained the joint, and the player
    // is checked first. Two skeletons are two heap allocations and nothing
    // keeps them apart: if the player's array sits below the partner's and its
    // span reaches past their base, then every joint of the partner falls
    // inside the player's range, and the player's cached rotation is written
    // into the partner's joint. Each machine then drives the other's character
    // - which is the symptom exactly, and it does not care who was adopted,
    // which is why tightening adoption did not stop it.
    //
    // A joint belongs to the array that starts closest below it. That is true
    // whichever order the two were allocated in and whatever the counts say.
    {
        ptrdiff_t best = -1;
        for (const auto& candidate : bodies) {
            if (!candidate.joints || !g_poseQuatMs[candidate.body])
                continue;
            const ptrdiff_t offset = joint - candidate.joints;
            const int span = candidate.count * kJointStride;
            if (offset < 0 || offset >= span || (offset % kJointStride) != 0)
                continue;
            if (best >= 0 && offset >= best)
                continue; // a further-away base: somebody else's array reaching over this one
            best = offset;
            whose = candidate.body;
            index = static_cast<int>(offset / kJointStride);
        }
#if RE5VR_DIAGNOSTICS
        // Say it once if the two really do overlap, because that is the thing
        // this guards against and it should be visible rather than inferred.
        if (g_haveBody && g_havePartnerBody && g_body.joints && g_partnerBody.joints) {
            static unsigned char* s_toldPair[2] = {};
            if (s_toldPair[0] != g_body.joints || s_toldPair[1] != g_partnerBody.joints) {
                s_toldPair[0] = g_body.joints;
                s_toldPair[1] = g_partnerBody.joints;
                const ptrdiff_t gap = g_partnerBody.joints - g_body.joints;
                const ptrdiff_t mine = static_cast<ptrdiff_t>(g_body.jointCount) * kJointStride;
                const ptrdiff_t theirs = static_cast<ptrdiff_t>(g_partnerBody.jointCount) * kJointStride;
                const bool over = gap >= 0 ? gap < mine : -gap < theirs;
                Log_Printf("ArmIk: your skeleton is at %p for %d bytes, theirs at %p for %d bytes - %s",
                    g_body.joints, static_cast<int>(mine), g_partnerBody.joints, static_cast<int>(theirs),
                    over ? "THEY OVERLAP, so joints were being claimed by the wrong one" : "clear of each other");
            }
        }
#endif
    }
    if (whose < 0 || index < 0 || index >= kMaxJoints) {
        ++g_poseNotOurs;
        return;
    }
    if (!g_poseQuatValid[whose][index]) {
        ++g_poseNoCache;
        return;
    }
    // And checked where it is used. This is the one line that makes "Sheva is
    // Sheva, Chris is Chris" true by construction rather than by careful
    // reasoning about heap layout.
    if (g_poseQuatJoints[whose] != bodies[whose].joints) {
        ++g_crossRefused[whose];
#if RE5VR_DIAGNOSTICS
        static unsigned long long s_toldWrong = 0;
        const unsigned long long nowWrong = GetTickCount64();
        if (nowWrong - s_toldWrong >= 2000) {
            s_toldWrong = nowWrong;
            Log_Printf("ArmIk: refused a pose for %s - it was worked out for %p and this joint belongs "
                       "to %p (%u refused so far)",
                whose == kPartnerBody ? "their body" : "YOUR body", g_poseQuatJoints[whose],
                bodies[whose].joints, g_crossRefused[whose]);
        }
#endif
        // And then let go of it (2026-09-24, user: "the default animation tried
        // to fire for my own body on my screen"). Exactly so, and this line is
        // why: the check used to return here, in FRONT of the staleness clear
        // below, so a cache that had been stamped for the wrong skeleton once
        // was never aged out and never refreshed. Every write for that body was
        // refused from then until a costume change, which is the animation
        // taking your arms back and keeping them.
        //
        // A wrong cache is worth nothing, so drop it. The next solve stamps a
        // fresh one against the right array and the mistake costs a frame
        // rather than the rest of the session.
        std::memset(g_poseQuatValid[whose], 0, sizeof(g_poseQuatValid[whose]));
        g_poseQuatMs[whose] = 0;
        g_poseQuatJoints[whose] = nullptr;
        return;
    }
    if (whose == kPlayerBody && !writeLocal) {
        ++g_poseNotLocal;
        return;
    }

    if (GetTickCount64() - g_poseQuatMs[whose] > 500) {
        // The solve has stopped; give the animation its arm back rather than
        // hold the character in the last pose we happened to write.
        std::memset(g_poseQuatValid[whose], 0, sizeof(g_poseQuatValid[whose]));
        g_poseQuatMs[whose] = 0;
        ++g_poseStale;
        return;
    }
    ++g_poseWrites;
    TryWrite(joint + kOffJointLocalRot, g_poseQuat[whose][index], sizeof(g_poseQuat[whose][index]));
}

// The T-pose calibration is gone from the UI (2026-09-19, user's call). What is
// left below never runs, because nothing sets g_calibratePending any more - kept
// only so the measurements it made are on record if they are wanted again: your
// arm length and shoulder position against the character's own bind pose, which
// did move reach from 1.3 of full to 0.85. It was bundled with a rotation tie
// that was wrong, and the bundle is what made it impossible to judge.
unsigned long long ArmIk_CostumeChangedMs()
{
    return g_costumeChangedMs;
}

void ArmIk_ClearCostumeChanged()
{
    g_costumeChangedMs = 0;
}

void ArmIk_Calibrate()
{
    // Rolled back: the whole T-pose calibration is out of the loop. With it, the
    // scale, the shoulder anchoring and the grip step-back all switch off and the
    // solve goes back to measuring from the eye with the assumed arm length.
    g_tiesCalibrated[0] = false;
    g_tiesCalibrated[1] = false;
    g_calUnitsPerMetre[0] = 0.0f;
    g_calUnitsPerMetre[1] = 0.0f;
    g_haveShoulderFromHead[0] = false;
    g_haveShoulderFromHead[1] = false;
    InterlockedExchange(&g_calibratePending, 1);
    Log_Printf("ArmIk: calibrating - your arms against this character's");
}

void ArmIk_SetReachAllowance(float metres)
{
    if (metres < 0.0f)
        metres = 0.0f;
    if (metres > 0.35f)
        metres = 0.35f;
    const float was = g_reachAllowanceM.exchange(metres, std::memory_order_relaxed);
    if (was != metres)
        Log_Printf("ArmIk: reach allowance now %.2f m - calibrate again for it to take effect", metres);
}

// See arm_ik.h. Sampled off the end of the frame, in the model's own frame -
// each bone against bone 0 - so that carrying the weapon around the level does
// not read as the weapon moving.
namespace {
unsigned long long g_animFrom = 0, g_animUntil = 0, g_animLast = 0;
float g_animWas[kMaxGunSkeletons][kMaxGunJoints][3];
float g_animFirst[kMaxGunSkeletons][kMaxGunJoints][3];
float g_animTravel[kMaxGunSkeletons][kMaxGunJoints];
float g_animFurthest[kMaxGunSkeletons][kMaxGunJoints];
// A total says a bone moved. A shape says what it did: when it left, how far
// it got, how long it stayed out and when it came back - which is the whole
// of what a reload is, and all of it has to be known before any of it can be
// taken over. Sixteen bones is more than any weapon here has, and one sample
// every tenth of a second over six seconds covers a reload twice.
constexpr int kTrackJoints = 16;
constexpr int kTrackSamples = 64;
float g_animTrack[kMaxGunSkeletons][kTrackJoints][kTrackSamples];
int g_animTrackAt = 0;
unsigned long long g_animTrackLast = 0;
int g_animSamples = 0;
bool g_animPrimed = false;

void SampleGunAnim()
{
    if (!g_animUntil)
        return;
    const unsigned long long now = GetTickCount64();
    if (now >= g_animUntil) {
        g_animUntil = 0;
        Log_Printf("GunAnim: %d sample(s) over six seconds", g_animSamples);
        for (int s = 0; s < g_gunSkCount && s < kMaxGunSkeletons; ++s) {
            const GunSkeleton& g = g_gunSk[s];
            // The best mover is always named, however small, so that a weapon
            // whose magazine barely shifts still says which bone it was.
            int best = -1;
            for (int j = 0; j < g.count && j < kMaxGunJoints; ++j)
                if (best < 0 || g_animFurthest[s][j] > g_animFurthest[s][best])
                    best = j;
            if (best < 0 || g_animFurthest[s][best] < 0.05f) {
                Log_Printf("GunAnim: [%d] %s - rigid, nothing on it moved", s, g.path);
                continue;
            }
            Log_Printf("GunAnim: [%d] %s - %d bone(s), the most of it bone %d at %.2f cm", s, g.path, g.count, best,
                g_animFurthest[s][best]);
            for (int j = 0; j < g.count && j < kMaxGunJoints; ++j) {
                if (g_animFurthest[s][j] <= 0.05f)
                    continue;
                Log_Printf("GunAnim:   bone %2d went %5.1f cm from where it started, %6.1f cm travelled in all", j,
                    g_animFurthest[s][j], g_animTravel[s][j]);
                if (j >= kTrackJoints || g_animFurthest[s][j] < 2.0f)
                    continue;
                char line[kTrackSamples * 6 + 8] = {};
                int w = 0;
                for (int t = 0; t < g_animTrackAt && t < kTrackSamples; ++t)
                    w += sprintf_s(line + w, sizeof(line) - w, "%.0f ", g_animTrack[s][j][t]);
                Log_Printf("GunAnim:   bone %2d cm from rest, every tenth of a second: %s", j, line);
            }
        }
        return;
    }
    if (now - g_animLast < 16)
        return;
    g_animLast = now;
    ++g_animSamples;
    const bool keepThisOne = now - g_animTrackLast >= 94; // a tenth of a second, as the clock can manage it
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    for (int s = 0; s < g_gunSkCount && s < kMaxGunSkeletons; ++s) {
        const GunSkeleton& g = g_gunSk[s];
        float root[3], R[9];
        if (!JointWorldPos(g.joints, 0, root) || !WorldRot(g.joints, 0, R))
            continue;
        for (int j = 0; j < g.count && j < kMaxGunJoints; ++j) {
            float p[3];
            if (!JointWorldPos(g.joints, j, p))
                continue;
            // IN THE WEAPON'S OWN AXES, not the world's (2026-09-25). The first
            // pass took the offset in world terms and every bone on every
            // weapon came back having moved tens of centimetres - which was
            // true, and meaningless. The gun is in your hand and you are in
            // VR: turn a twenty centimetre pistol over and a bone at its
            // muzzle sweeps forty centimetres through the room while the
            // weapon itself does not bend at all. Read against the root's own
            // rotation, a rigid weapon reads zero however it is waved about,
            // and the only thing left in the numbers is animation.
            const float d[3] = { p[0] - root[0], p[1] - root[1], p[2] - root[2] };
            float rel[3];
            for (int r = 0; r < 3; ++r)
                rel[r] = (R[r * 3] * d[0] + R[r * 3 + 1] * d[1] + R[r * 3 + 2] * d[2]) * 100.0f / upm;
            if (g_animPrimed) {
                const float step[3] = { rel[0] - g_animWas[s][j][0], rel[1] - g_animWas[s][j][1],
                    rel[2] - g_animWas[s][j][2] };
                g_animTravel[s][j] += Length3(step);
                const float away[3] = { rel[0] - g_animFirst[s][j][0], rel[1] - g_animFirst[s][j][1],
                    rel[2] - g_animFirst[s][j][2] };
                const float d = Length3(away);
                if (d > g_animFurthest[s][j])
                    g_animFurthest[s][j] = d;
                if (j < kTrackJoints && g_animTrackAt < kTrackSamples)
                    g_animTrack[s][j][g_animTrackAt] = d;
            } else {
                std::memcpy(g_animFirst[s][j], rel, sizeof(rel));
            }
            std::memcpy(g_animWas[s][j], rel, sizeof(rel));
        }
    }
    if (keepThisOne && g_animPrimed && g_animTrackAt < kTrackSamples) {
        g_animTrackLast = now;
        ++g_animTrackAt;
    }
    g_animPrimed = true;
}
} // namespace

void ArmIk_WatchTheReload()
{
    std::memset(g_animTravel, 0, sizeof(g_animTravel));
    std::memset(g_animFurthest, 0, sizeof(g_animFurthest));
    std::memset(g_animTrack, 0, sizeof(g_animTrack));
    g_animTrackAt = 0;
    g_animTrackLast = 0;
    g_animPrimed = false;
    g_animSamples = 0;
    g_animFrom = GetTickCount64();
    g_animLast = 0;
    g_animUntil = g_animFrom + 6000;
    Log_Printf("GunAnim: watching the weapon's bones for six seconds - reload once");
}

void ArmIk_OnEndScene()
{
    SampleGunAnim();

    // ONE LINE A RELEASE BUILD STILL PRINTS (2026-09-29, user: "6dof does not
    // work the same way on a release build").
    //
    // Every per-frame trace that would answer that question is compiled out
    // of the build people actually run, so a report from one is unanswerable.
    // This is the smallest thing that is not: the switches, whether a body
    // and two arms were found, and whether the solve is doing anything. It
    // prints every five seconds only while 6DOF is on, so a player who never
    // enables it never sees it.
    {
        const XrInputSettings st = XrInput_GetSettings();
        static unsigned long long s_saidMs = 0;
        const unsigned long long nowSaid = GetTickCount64();
        // QUIET UNLESS IT IS WRONG (2026-09-29, shipping build). Three lines
        // every five seconds was right while the release build was broken;
        // for a player it is noise. It now says nothing at all while the arms
        // are being driven, and everything it knows the moment they are not.
        const bool armsAreWorking = g_poseWrites > 0;
        if (st.armIk && !armsAreWorking && nowSaid - s_saidMs >= 5000) {
            s_saidMs = nowSaid;
            Log_Printf("ArmIk: 6DOF on - wrist %d, gun lock %d, pose-not-result %d, weight %.2f, reach %.2fx, "
                       "shoulder %.2f | body %s, left arm %s, right arm %s | skeleton hook %s",
                st.armIkWrist ? 1 : 0, st.gunLock ? 1 : 0, st.armIkWriteLocal ? 1 : 0, st.armIkWeight,
                st.armIkScale, st.armIkShoulder, g_haveBody ? "found" : "NOT FOUND",
                (g_haveBody && g_body.left.valid) ? "found" : "NOT FOUND",
                (g_haveBody && g_body.right.valid) ? "found" : "NOT FOUND",
                g_skeletonHooked ? "in" : "MISSING");
            Log_Printf("ArmIk: pose callback in the last 5 s - reached %lu, wrote %lu; stopped at: no "
                       "joint %lu, not ours %lu, nothing solved for it %lu, local writing off %lu, solve "
                       "stale %lu (late write active %d)",
                g_poseCalls, g_poseWrites, g_poseNoJoint, g_poseNotOurs, g_poseNoCache, g_poseNotLocal,
                g_poseStale, ArmIk_LateWriteActive() ? 1 : 0);
            Log_Printf("ArmIk: upstream in the last 5 s - joint-built reached %lu (not ours %lu), solve ran "
                       "%lu, rotations refused as unrebuildable %lu, cross-stamped %lu",
                g_jbCalls, g_jbNotOurs, g_solveRuns, g_wlrErr, g_crossStamps);
            Log_Printf("ArmIk: cache - %lu rotation(s) cached, thrown away %lu time(s) (last at line %d); "
                       "stamp is %lld ms old, cached against %p, body is %p",
                g_cached, g_cacheCleared, g_lastClearSite,
                g_poseQuatMs[kPlayerBody]
                    ? (long long)(GetTickCount64() - g_poseQuatMs[kPlayerBody]) : -1LL,
                g_poseQuatJoints[kPlayerBody], g_haveBody ? g_body.joints : nullptr);
            g_cached = g_cacheCleared = 0;
        }
        if (st.armIk && nowSaid - s_saidMs >= 5000) {
            s_saidMs = nowSaid;
            g_poseWrites = g_poseCalls = g_poseNoJoint = g_poseNotOurs = g_poseNoCache = g_poseNotLocal
                = g_poseStale = 0;
            g_jbCalls = g_jbNotOurs = g_solveRuns = g_wlrErr = g_crossStamps = 0;
        }
    }
#if RE5VR_DIAGNOSTICS
    // The partner's arm, measured where nothing can be half-built (2026-09-24).
    //
    // Everywhere else this has been measured, the numbers are impossible: the
    // sum of two rigid bones came back 97 units one frame and 38 the next
    // against a resting 53.2. A unit rotation cannot change that sum, and 38 is
    // shorter than the bones themselves, so some of what I have been reading is
    // a skeleton part way through being rebuilt rather than a stretched arm.
    // The mod already learned this once for the player - "53.2 became 54 to 58,
    // which is measuring a skeleton that does not exist" - and never for a
    // partner.
    //
    // EndScene is after everything: the game has finished the frame. If the
    // bones measure 53.2 here while the solve sees 97, the stretch is a
    // measurement artefact and the real fault is elsewhere. If they measure 97
    // here too, the arm really is being pulled apart and this is where to dig.
    {
        static float s_worst = 0.0f, s_worstUp = 0.0f, s_worstFore = 0.0f;
        static float s_best = 1e9f;
        static unsigned s_n = 0;
        static unsigned long long s_toldMs = 0;
        if (g_havePartnerBody && g_partnerBody.joints && g_partnerBody.right.valid) {
            float sP[3], eP[3], wP[3];
            if (JointWorldPos(g_partnerBody.joints, g_partnerBody.right.shoulder, sP)
                && JointWorldPos(g_partnerBody.joints, g_partnerBody.right.elbow, eP)
                && JointWorldPos(g_partnerBody.joints, g_partnerBody.right.wrist, wP)) {
                float uv[3], fv[3];
                Sub3(eP, sP, uv);
                Sub3(wP, eP, fv);
                const float up = Length3(uv), fore = Length3(fv);
                const float all = up + fore;
                ++s_n;
                if (all > s_worst) {
                    s_worst = all;
                    s_worstUp = up;
                    s_worstFore = fore;
                }
                if (all < s_best)
                    s_best = all;
            }
        }
        const unsigned long long now = GetTickCount64();
        if (s_n > 30 && now - s_toldMs >= 2000) {
            s_toldMs = now;
            Log_Printf("EndOfFrame: their right arm over %u frame(s) - longest %.1f units (upper %.1f, forearm "
                       "%.1f), shortest %.1f, resting %.1f",
                s_n, s_worst, s_worstUp, s_worstFore, s_best, g_restReach[0]);
            s_worst = s_worstUp = s_worstFore = 0.0f;
            s_best = 1e9f;
            s_n = 0;
        }
    }
    // The last word on the gun hop (2026-09-23). Everything so far has been
    // measured where the mod happens to run, which is not where the renderer
    // reads. This runs at EndScene - the game has finished moving everything
    // for this frame - and it only reads. Three numbers, per frame, split by
    // whether a shot went off in the last 0.3 s:
    //
    //   1. the GUN against the HAND it sits in. Anything here is the weapon
    //      being moved in the hand, and no amount of steadying the body will
    //      ever touch it.
    //   2. the HAND against the character's ROOT. This is the body carrying
    //      the arm - what the freeze was meant to stop.
    //   3. the ROOT itself, in the world: him being shoved about.
    //
    // Whichever of the three moves while firing is the one to fix, and there
    // is nothing left to guess after it.
    const XrInputSettings settings = XrInput_GetSettings();
    if (!g_haveBody || !g_body.joints || !g_body.character)
        return;
    const ArmIkArm& gun = settings.characterLeftHanded ? g_body.left : g_body.right;
    if (!gun.valid)
        return;
    float handRot[9], handPos[3], rootRot[9], rootPos[3];
    if (!WorldRot(g_body.joints, gun.wrist, handRot) || !JointWorldPos(g_body.joints, gun.wrist, handPos)
        || !WorldRot(g_body.joints, 0, rootRot) || !JointWorldPos(g_body.joints, 0, rootPos))
        return;
    // The gun: the root bone of whichever weapon model is in that hand.
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    float gunRot[9], gunPos[3];
    bool haveGun = false;
    for (int e = 0; e < 12 && !haveGun; ++e) {
        unsigned char* object = nullptr;
        if (!TryRead(&object, g_body.character + 0x21D0 + e * 0x30, sizeof(object))
            || reinterpret_cast<uintptr_t>(object) < 0x10000)
            continue;
        unsigned char* joints = nullptr;
        if (!TryRead(&joints, object + 0x318, sizeof(joints)) || reinterpret_cast<uintptr_t>(joints) < 0x10000)
            continue;
        float pos[3];
        if (!JointWorldPos(joints, 1, pos))
            continue;
        const float d[3] = { pos[0] - handPos[0], pos[1] - handPos[1], pos[2] - handPos[2] };
        if (Length3(d) * 100.0f / upm > 25.0f)
            continue;
        if (!WorldRot(joints, 1, gunRot))
            continue;
        std::memcpy(gunPos, pos, sizeof(pos));
        haveGun = true;
    }

    const unsigned long long nowMs = GetTickCount64();
    const unsigned long long shot = XrInput_LastShotMs();
    const int k = shot && nowMs - shot < 300 ? 1 : 0;

    // Each pair as "how much did this move relative to that since last frame".
    static float s_prevGunInHand[3], s_prevHandInRoot[3], s_prevRoot[3];
    static float s_prevGunRot[9], s_prevHandRot[9];
    static bool s_have = false;
    static float s_sum[2][5], s_max[2][5];
    static int s_n[2];
    static unsigned long long s_toldMs = 0;

    float rootT[9], handT[9];
    Transpose3(rootRot, rootT);
    Transpose3(handRot, handT);
    float gunInHand[3] = {}, handInRoot[3] = {}, gunRelRot[9] = {}, handRelRot[9] = {};
    {
        float d[3];
        Sub3(handPos, rootPos, d);
        for (int r = 0; r < 3; ++r)
            handInRoot[r] = Dot3(rootRot + r * 3, d) * 100.0f / upm;
        Mul3(handRot, rootT, handRelRot);
        if (haveGun) {
            Sub3(gunPos, handPos, d);
            for (int r = 0; r < 3; ++r)
                gunInHand[r] = Dot3(handRot + r * 3, d) * 100.0f / upm;
            Mul3(gunRot, handT, gunRelRot);
        }
    }
    if (s_have) {
        const auto turn = [](const float a[9], const float b[9]) {
            float tr = 0.0f;
            for (int r = 0; r < 3; ++r)
                tr += Dot3(a + r * 3, b + r * 3);
            float c = (tr - 1.0f) * 0.5f;
            c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
            return std::acos(c) * 57.2957795f;
        };
        float d[3];
        Sub3(gunInHand, s_prevGunInHand, d);
        const float gunCm = haveGun ? Length3(d) : 0.0f;
        const float gunDeg = haveGun ? turn(gunRelRot, s_prevGunRot) : 0.0f;
        Sub3(handInRoot, s_prevHandInRoot, d);
        const float handCm = Length3(d);
        const float handDeg = turn(handRelRot, s_prevHandRot);
        float dr[3];
        Sub3(rootPos, s_prevRoot, dr);
        const float rootCm = Length3(dr) * 100.0f / upm;
        const float v[5] = { gunCm, gunDeg, handCm, handDeg, rootCm };
        bool sane = true;
        for (float x : v)
            sane = sane && x < 100.0f;
        if (sane) {
            for (int q = 0; q < 5; ++q) {
                s_sum[k][q] += v[q];
                if (v[q] > s_max[k][q])
                    s_max[k][q] = v[q];
            }
            ++s_n[k];
        }
    }
    std::memcpy(s_prevGunInHand, gunInHand, sizeof(gunInHand));
    std::memcpy(s_prevHandInRoot, handInRoot, sizeof(handInRoot));
    std::memcpy(s_prevRoot, rootPos, sizeof(rootPos));
    std::memcpy(s_prevGunRot, gunRelRot, sizeof(gunRelRot));
    std::memcpy(s_prevHandRot, handRelRot, sizeof(handRelRot));
    s_have = true;

    if (s_n[1] > 20 && nowMs - s_toldMs >= 3000) {
        s_toldMs = nowMs;
        const float f = 1.0f / s_n[1];
        const float g = s_n[0] ? 1.0f / s_n[0] : 0.0f;
        Log_Printf("LastWord: per frame at the end of the frame - FIRING: gun in the hand %.2f cm / %.2f deg, hand "
                   "against the root %.2f cm / %.2f deg, the root itself %.2f cm | NOT FIRING: gun %.2f / %.2f, "
                   "hand %.2f / %.2f, root %.2f (%d/%d frames, gun %s)",
            s_sum[1][0] * f, s_sum[1][1] * f, s_sum[1][2] * f, s_sum[1][3] * f, s_sum[1][4] * f, s_sum[0][0] * g,
            s_sum[0][1] * g, s_sum[0][2] * g, s_sum[0][3] * g, s_sum[0][4] * g, s_n[1], s_n[0],
            haveGun ? "found" : "NOT FOUND");
        Log_Printf("LastWord: worst single frame while firing - gun %.2f cm / %.2f deg, hand %.2f cm / %.2f deg, "
                   "root %.2f cm",
            s_max[1][0], s_max[1][1], s_max[1][2], s_max[1][3], s_max[1][4]);
        std::memset(s_sum, 0, sizeof(s_sum));
        std::memset(s_max, 0, sizeof(s_max));
        s_n[0] = s_n[1] = 0;
    }
#endif
}

// The gun's firing kick (2026-09-22). The weapon hangs off entries in the
// character's attachment table - character+0x21D0 onward, 0x30 apart, each a
// pointer to an object with its world transform at +0xE0. Between shots they
// sit still on the hand; firing jolts them 3.2 deg and 3.6 cm a frame against
// it, all together, while the arm itself does not move.
//
// So every frame, once the arm is posed: each entry within 25 cm of the gun
// hand is measured against the hand. Between shots that is its resting place
// and is tracked. During a burst it is put back toward that place, keeping
// only the chosen share of the kick - one value one-handed, a lower one with
// both hands on the gun. The old world-matrix mode found that nothing
// recomposes the attachments after the camera hook ("the gun is the only
// thing it cannot bring along"); if that turns out wrong here, the report
// says the fix is being overwritten.
namespace {
constexpr DWORD kOffAttachTable = 0x21D0;
constexpr DWORD kAttachStride = 0x30;
constexpr int kAttachEntries = 12;
constexpr DWORD kOffAttachWorld = 0xE0;
// The transform +0xE0 is copied FROM, element by element, every frame - the
// only genuine write the watch caught on the gun (re5dx9.exe+18B3ED, fld
// [esi+D0] / fstp [esi+110]). So +0xE0 is most likely last frame's copy and
// +0xA0 the one drawn from; correcting +0xE0 alone changed nothing on screen.
constexpr DWORD kOffAttachCurrent = 0xA0;
// A second transform on the same object, found by the attachment scan sitting
// 17 cm out along the barrel while +0xA0 sits exactly on the grip. Moved with
// the rest from 2026-09-25, because a laser drawn from a muzzle that does not
// follow the gun points at where the gun used to be.
constexpr DWORD kOffAttachAim = 0x2A0;
struct AttachRest {
    unsigned char* object;
    float rot[9]; // rows, relative to the hand
    float pos[3]; // in the hand's axes, units
    bool have;
    float wrote[16];
    bool wroteValid;
    bool onHand;
    // How long this placement has been somewhere other than where it is held.
    // Locking the attach transform completely took out 2.8 deg and 0.5 cm at
    // its worst, so the shake was never in it - the recoil is an animation on
    // the gun's own bones, which are held further down.
    unsigned long long offSince;
};
AttachRest g_attach[kAttachEntries];
unsigned char* g_attachCharacter = nullptr;
bool g_attachLeftHanded = false;

// Turns a toward b by t (0 keeps a, 1 is b), both as row matrices. Rows
// blended and squared up again rather than axis and angle (2026-09-22): the
// game's matrices can be mirrored, a mirrored matrix is not a rotation, and
// axis-angle on one gave 180 degree "corrections". For a kick of a few degrees
// this is as exact as a slerp, and it keeps whichever handedness they had.
void BlendRot(const float a[9], const float b[9], float t, float out[9])
{
    float m[9];
    for (int i = 0; i < 9; ++i)
        m[i] = a[i] + (b[i] - a[i]) * t;
    float x[3] = { m[0], m[1], m[2] };
    float y[3] = { m[3], m[4], m[5] };
    const float z0[3] = { m[6], m[7], m[8] };
    if (!Normalise3(x)) {
        std::memcpy(out, a, sizeof(float) * 9);
        return;
    }
    const float along = Dot3(y, x);
    for (int i = 0; i < 3; ++i)
        y[i] -= x[i] * along;
    if (!Normalise3(y)) {
        std::memcpy(out, a, sizeof(float) * 9);
        return;
    }
    float z[3];
    Cross3(x, y, z);
    if (Dot3(z, z0) < 0.0f) {
        z[0] = -z[0];
        z[1] = -z[1];
        z[2] = -z[2];
    }
    std::memcpy(out, x, sizeof(x));
    std::memcpy(out + 3, y, sizeof(y));
    std::memcpy(out + 6, z, sizeof(z));
}
} // namespace

namespace {
// The resting place plus the chosen share of the kick, as a world transform.
bool SteadiedMatrix(const AttachRest& a, const float m[16], const float hp[3], const float hr[9], float share,
    float out[16], float* fixDeg, float* fixCm, float upm)
{
    const float scale = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (!(scale > 0.01f))
        return false;
    float rows[9];
    for (int r = 0; r < 3; ++r) {
        float v[3] = { m[r * 4], m[r * 4 + 1], m[r * 4 + 2] };
        if (!Normalise3(v))
            return false;
        std::memcpy(rows + r * 3, v, sizeof(v));
    }
    float hrT[9], relRot[9], relPos[3];
    Transpose3(hr, hrT);
    Mul3(rows, hrT, relRot);
    const float d[3] = { m[12] - hp[0], m[13] - hp[1], m[14] - hp[2] };
    for (int r = 0; r < 3; ++r)
        relPos[r] = Dot3(hr + r * 3, d);
    float useRot[9], usePos[3];
    BlendRot(a.rot, relRot, share, useRot);
    for (int i = 0; i < 3; ++i)
        usePos[i] = a.pos[i] + (relPos[i] - a.pos[i]) * share;
    float worldRows[9];
    Mul3(useRot, hr, worldRows);
    for (int r = 0; r < 3; ++r) {
        out[r * 4] = worldRows[r * 3] * scale;
        out[r * 4 + 1] = worldRows[r * 3 + 1] * scale;
        out[r * 4 + 2] = worldRows[r * 3 + 2] * scale;
        out[r * 4 + 3] = m[r * 4 + 3];
    }
    for (int i = 0; i < 3; ++i)
        out[12 + i] = hp[i] + hr[i] * usePos[0] + hr[3 + i] * usePos[1] + hr[6 + i] * usePos[2];
    out[15] = m[15];
    if (fixDeg) {
        float tr = 0.0f;
        for (int r = 0; r < 3; ++r)
            tr += Dot3(useRot + r * 3, relRot + r * 3);
        float cs = (tr - 1.0f) * 0.5f;
        cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
        *fixDeg = std::acos(cs) * 57.2957795f;
    }
    if (fixCm) {
        float dp[3];
        Sub3(usePos, relPos, dp);
        *fixCm = Length3(dp) * 100.0f / upm;
    }
    return true;
}

// ---- At the source (2026-09-22) -------------------------------------------
// Correcting the gun from the camera hook never reached the screen: both of
// its transforms are rebuilt later in the frame, 0 of 1248 held. A clean watch
// (6DOF arms off, so the arm guard was not using the debug registers) found a
// single writer, every frame: re5dx9.exe+18B3ED, 344 writes in four seconds,
// with esi holding the attach object. The code there is a straight run of
// x87 copies, the current transform at +0xA0 into last frame's at +0xE0:
//     fld [esi+CC] / fstp [esi+10C] / fld [esi+D0] / fstp [esi+110] |
//     fld [esi+D4] / fstp [esi+114] / ...
// and the instruction at the hook point is six bytes with nothing relative in
// it, so it moves cleanly into the trampoline.
//
// At that instant the current transform is finished and the copy is part way
// through. So the hook steadies the current transform and writes the whole of
// +0xE0 from it; the copies that follow then carry the steadied values too.
// It does nothing unless a shot went off in the last 0.3 s and esi is one of
// the entries on the gun hand. The floating-point state is saved around it:
// there is a value on the x87 stack between those copies' loads and stores -
// not at this exact point, but the SSE registers the C code uses are the
// game's too.
// Where the CURRENT transform is written (2026-09-22, the second watch):
// re5dx9.exe+8B9E5, 371 writes in four seconds with esi the attach object.
// It stores the matrix one float at a time from locals -
//     movss [esi+CC],xmm0 / movss xmm0,[esp+1C] / movss [esi+D0],xmm0 |
//     movss xmm0,[esp+18] / movss [esi+D4],xmm0 / ...
// - so at the caught instruction the position's y, z and w are still to
// come. The hook goes after the store to +0xDC, the last element; the walk
// that finds it is in ArmIk_SteadyGun. (18B3ED, the first hook, was the copy
// of current into last frame's at the START of the update, so everything it
// corrected was about to be replaced.)
constexpr DWORD kGunWriterRva = 0x8B9E5;
void* g_gunWriteTrampoline = nullptr;
volatile LONG g_gunSourceState = 0; // 0 not tried, 1 installed and on, 2 installed and off, -1 failed
volatile LONG g_sourceWrites = 0;
float g_sourceShare = 1.0f;
// How big the gun should be drawn, for the hook below. Writing the weapon's
// transform from the frame hook was overwritten every frame; inside the
// game's own code, right after it writes it, is the one place that holds.
float g_gunScaleWanted = 1.0f;
volatile LONG g_gunScaleWrites = 0;
// Whether the lock is on, and the worst correction it has had to make, for
// the report. Plain globals rather than a settings call: this runs inside the
// game's own writer, several times a frame.
volatile LONG g_gunLockOn = 0;
float g_gunFixDegMax = 0.0f;
float g_gunFixCmMax = 0.0f;

void __cdecl GunWriteAfter(unsigned char* object)
{
    if (g_gunSourceState != 1 || !object)
        return;
    // Size first, and whether or not a shot just went off: the game has just
    // finished writing this weapon's transform, so its rows are scaled here
    // and the translation - the grip, which sits in the hand - is left alone.
    if (std::fabs(g_gunScaleWanted - 1.0f) > 0.005f) {
        for (int e = 0; e < kAttachEntries; ++e) {
            const AttachRest& a = g_attach[e];
            if (a.object != object || !a.onHand)
                continue;
            float m[16];
            if (!TryRead(m, object + kOffAttachCurrent, sizeof(m)) || std::fabs(m[15] - 1.0f) > 1e-3f)
                break;
            bool ok = true;
            for (int r = 0; r < 3 && ok; ++r) {
                const float len = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1]
                    + m[r * 4 + 2] * m[r * 4 + 2]);
                if (!(len > 0.01f) || !(len < 20.0f)) {
                    ok = false;
                    break;
                }
                // Already our size? Then the game has not rewritten it and
                // scaling again would shrink it a second time.
                if (std::fabs(len - g_gunScaleWanted) < 0.002f)
                    return;
                const float k = g_gunScaleWanted / len;
                m[r * 4] *= k;
                m[r * 4 + 1] *= k;
                m[r * 4 + 2] *= k;
            }
            if (ok && TryWrite(object + kOffAttachCurrent, m, sizeof(m)))
                InterlockedIncrement(&g_gunScaleWrites);
            break;
        }
    }
    // The attach transform is NOT held (2026-09-23). It was, and locking it
    // outright took out 2.8 deg and 0.5 cm at its very worst, which is nothing
    // - the shake was never in it. What it did do, once it ran between shots
    // as well as during them, was roll the gun sideways in the hand and leave
    // it there: the placement is learned where the mod runs and applied here,
    // and our own arm IK moves the wrist between those two points in the
    // frame, so the difference between them is a permanent twist. The weapon's
    // own bones are held instead, where the recoil actually lives, and learned
    // and applied in the same breath so no such gap exists.
    //
    // This hook stays for the gun size above, which is the one change that has
    // always reached the screen from here.

}
} // namespace

__declspec(naked) void GunWriteHook_Stub()
{
    __asm {
        pushad
        pushfd
        mov ebp, esp
        sub esp, 512
        and esp, 0FFFFFFF0h
        fxsave [esp]
        push esi
        call GunWriteAfter
        add esp, 4
        fxrstor [esp]
        mov esp, ebp
        popfd
        popad
        jmp g_gunWriteTrampoline
    }
}

namespace {
// A MAGAZINE YOU CAN SEE (2026-09-25, user: "it'd be nice if once you pressed
// the eject button, we had the mag fall out of the gun, and a magazine
// actually in your hand when you pull from your hip").
//
// No new geometry, and none needed: the weapon already carries its magazine as
// its own bone, and a bone can be written. The game animates it out of the
// well and back during a reload, which is how it was found - it was the only
// thing on the weapon that moved while the gun itself stayed rigid.
//
// So the magazine is borrowed for as long as the reload lasts. Dropped, it
// keeps the rotation it had and falls under its own weight, which is a good
// deal more convincing than making it disappear. Carried, it is written to
// your off hand's wrist, where your actual hand is, because the arm solve has
// already put that wrist on your controller. Seated, it is handed back and the
// game has it again.
//
// Written at the very end of the solve, after the hold has finished placing
// every other bone, so nothing downstream puts it back.
float g_magFallPos[3] = {};
float g_magFallRot[9] = {};
float g_magFallVel[3] = {};
// The last place the mod itself put the magazine. A magazine let go of half
// way to the weapon falls out of your HAND, and the only record of where your
// hand was is this one: by the time the drop is noticed the game has already
// rewritten the bone back into the well, so reading it then would drop it out
// of the gun a second time - which is exactly what it did.
float g_magHeldRows[9] = {};
float g_magHeldPos[3] = {};
bool g_magWasInHand = false;
bool g_magFalling = false;
// Where the magazine sits in the weapon when nothing is happening to it,
// learned continuously and in the WEAPON'S frame, so it can be put back there
// while the reload animation tries to take it somewhere else. Two bones,
// because the round on top travels with it.
constexpr int kMagParts = 2;
float g_magRestRot[kMagParts][9] = {};
float g_magRestPos[kMagParts][3] = {};
float g_magRestLen[kMagParts][3] = {};
bool g_haveMagRest[kMagParts] = {};
int g_magWasState = 0;
unsigned long long g_magFallLast = 0;

bool WriteBoneWorld(unsigned char* joints, int bone, const float rows[9], const float pos[3], const float scale[3])
{
    float w[16];
    if (!TryRead(w, joints + bone * kJointStride + kOffJointWorldMatrix, sizeof(w)))
        return false;
    for (int r = 0; r < 3; ++r) {
        w[r * 4] = rows[r * 3] * scale[r];
        w[r * 4 + 1] = rows[r * 3 + 1] * scale[r];
        w[r * 4 + 2] = rows[r * 3 + 2] * scale[r];
    }
    w[12] = pos[0];
    w[13] = pos[1];
    w[14] = pos[2];
    if (!TryWrite(joints + bone * kJointStride + kOffJointWorldMatrix, w, sizeof(w)))
        return false;
    TryWrite(joints + bone * kJointStride + kOffJointWorldPos, pos, sizeof(float) * 3);
    return true;
}

// Takes nothing but the settings and works the rest out for itself, because it
// has to be able to run on frames where the gun hold does not (2026-09-25,
// user: "occasionally the magazine bone doesn't freeze and the regular
// animation plays"). See ArmIk_SteadyGun.
// ---- Which bone is the magazine, per weapon (2026-09-26) ----------------
//
// User: "I'm guessing we have to go one by one for every gun in the game to
// figure out manual reloading". No. RE5 has around thirty weapons, a table of
// bone numbers is a maintenance burden on a public mod, and it is wrong the
// moment anybody installs a weapon model mod - which this machine already has.
//
// The mod can find them. The magazine is the bone that leaves the weapon and
// comes back during a reload, and that is exactly what the reload watcher
// already spots. So an unfamiliar weapon gets ONE reload left alone: the
// gesture still runs, the ammunition still works, but nothing is written to
// any bone and the animation is watched instead. From the next reload on, that
// weapon is known.
//
// WHICH MOVER IS THE MAGAZINE is the only hard part, and two obvious answers
// are both wrong. Measured on the M92F:
//
//     bone 2 moved  3.0 cm, first at +937 ms   the slide
//     bone 7 moved 63.7 cm, first at +359 ms   the magazine
//     bone 8 moved 68.3 cm, first at  +31 ms   a casing
//
// The casing travels further than the magazine AND moves before it, so neither
// biggest nor earliest can tell them apart. Timing was the first guess and it
// picked the casing, which is how this comment came to be rewritten.
//
// A third rule was tried - nearest the grip at rest, because a magazine lives
// in the grip and a casing in the chamber - and the user shot it down before it
// was ever built: "what if it's a machine gun that the mag sits ahead of your
// hand, there's no guarantee a mag will be near your palm". Which is right. A
// P90 carries its magazine on top, a bullpup carries it behind the grip, and
// "near the palm" is a fact about pistols wearing a fact about guns.
//
// So the guessing stops here. Three rules were invented from measurements of
// ONE weapon and two of them were wrong, which is about the expected hit rate
// for reasoning about thirty guns from a sample of one.
//
// What the mod can do reliably is find the MOVERS: the parts that leave the
// weapon during a reload, with their travel, their timing and where they rest.
// That is measurement and it holds on any weapon. Which of them is the
// magazine is a question with two or three possible answers and a person
// looking straight at it, so the candidates are ordered by a guess, the guess
// is used, and one button steps to the next one and writes it down for good.
// One press per weapon, at most, and never for a weapon the guess got right.
constexpr int kKnownWeapons = 32;
constexpr float kMovedCm = 2.0f; // further than a bone wobbles by itself

constexpr int kMovers = 4;
struct KnownWeapon {
    unsigned key;
    int bone[kMovers];   // the movers, best guess first
    int travel[kMovers]; // and how far each went, in whole centimetres
    int count;
    int pick;            // which of them is currently believed to be the magazine
};
KnownWeapon g_known[kKnownWeapons];
int g_knownCount = 0;
bool g_knownRead = false;

bool g_learning = false;
unsigned g_learnKey = 0;
int g_learnBones = 0;
unsigned long long g_learnFrom = 0;
bool g_learnPrimed = false;
float g_learnStart[kMaxGunJoints][3];
float g_learnFar[kMaxGunJoints];
unsigned g_learnFirstMs[kMaxGunJoints];

void KnownFilePath(char* out, size_t size)
{
    out[0] = 0;
    if (!GetModuleFileNameA(nullptr, out, static_cast<DWORD>(size)))
        return;
    char* slash = std::strrchr(out, '\\');
    if (slash)
        slash[1] = 0;
    strcat_s(out, size, "re5vr_magbones.ini");
}

// Remembered between sessions, because relearning every weapon on every launch
// would be a worse tax than the table this replaces.
void ReadKnown()
{
    g_knownRead = true;
    char path[MAX_PATH];
    KnownFilePath(path, sizeof(path));
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") != 0 || !f)
        return;
    char line[192];
    while (g_knownCount < kKnownWeapons && fgets(line, sizeof(line), f)) {
        unsigned key = 0;
        int pick = 0, count = 0;
        int b[kMovers] = { -1, -1, -1, -1 };
        int t[kMovers] = {};
        const int got = sscanf_s(line, "%x = %d %d %d %d %d %d %d %d %d %d", &key, &pick, &count, &b[0], &t[0],
            &b[1], &t[1], &b[2], &t[2], &b[3], &t[3]);
        if (got >= 5 && key && count > 0) {
            KnownWeapon& k = g_known[g_knownCount++];
            k.key = key;
            k.count = count > kMovers ? kMovers : count;
            k.pick = pick >= 0 && pick < k.count ? pick : 0;
            for (int i = 0; i < kMovers; ++i) {
                k.bone[i] = b[i];
                k.travel[i] = t[i];
            }
        }
    }
    fclose(f);
    Log_Printf("MagBone: %d weapon(s) already known, from %s", g_knownCount, path);
}

void WriteKnown()
{
    char path[MAX_PATH];
    KnownFilePath(path, sizeof(path));
    FILE* f = nullptr;
    if (fopen_s(&f, path, "w") != 0 || !f)
        return;
    fprintf(f, "; Which part of each weapon is its magazine, found by watching a reload.\n");
    fprintf(f, "; Everything that moved is listed; the second number says which one is\n");
    fprintf(f, "; believed to be the magazine, and the menu button steps through them.\n");
    fprintf(f, "; weapon = chosen, how many, then each bone and how far it travelled.\n");
    fprintf(f, "; Delete a line to learn that weapon again, or the file to start over.\n");
    for (int i = 0; i < g_knownCount; ++i) {
        const KnownWeapon& k = g_known[i];
        fprintf(f, "%08X = %d %d", k.key, k.pick, k.count);
        for (int j = 0; j < kMovers; ++j)
            fprintf(f, " %d %d", k.bone[j], k.travel[j]);
        fprintf(f, "\n");
    }
    fclose(f);
}

const KnownWeapon* FindKnown(unsigned key)
{
    if (!g_knownRead)
        ReadKnown();
    for (int i = 0; i < g_knownCount; ++i)
        if (g_known[i].key == key)
            return &g_known[i];
    return nullptr;
}

// The magazine as currently believed, and whatever travels with it: the other
// mover that went furthest, since a part that barely shifted is a slide or a
// bolt and has no business following your hand to your belt.
void BonesOf(const KnownWeapon& k, int& mag, int& with)
{
    mag = k.pick >= 0 && k.pick < k.count ? k.bone[k.pick] : -1;
    with = -1;
    int bestTravel = 0;
    for (int i = 0; i < k.count && i < kMovers; ++i) {
        if (i == k.pick || k.bone[i] < 0)
            continue;
        if (k.travel[i] > bestTravel) {
            bestTravel = k.travel[i];
            with = k.bone[i];
        }
    }
}

KnownWeapon* SlotFor(unsigned key)
{
    for (int i = 0; i < g_knownCount; ++i)
        if (g_known[i].key == key)
            return &g_known[i];
    if (g_knownCount >= kKnownWeapons)
        return nullptr;
    KnownWeapon& k = g_known[g_knownCount++];
    k = KnownWeapon{};
    k.key = key;
    for (int& b : k.bone)
        b = -1;
    return &k;
}

// What the last weapon in hand was, so the menu button knows what it is
// correcting without having to work it out again.
unsigned g_lastKey = 0;

// What makes one weapon different from another, in something that survives a
// relaunch. The ammunition record carries a mark that read 0201 on the M92F,
// whose weapon id is 1, so the low byte looks like the weapon and the high
// byte like its ammunition type. The bone count goes in with it, because two
// weapons sharing a mark cannot share a skeleton shape as well.
unsigned WeaponKey(const GunSkeleton& sk)
{
    // THE WEAPON SAYS WHO IT IS (2026-09-26, user: "couldn't you pick it up off
    // of weapon ID?").
    //
    // It can, and the id is at object+14h. Caught by dumping the head of the
    // weapon in hand: it read 0201, the same value the ammunition record
    // carries, and the low byte is 1 - the M92F, from the user's own list of
    // weapon ids. The high byte is almost certainly the ammunition class.
    //
    // Taking it from HERE rather than from the ammunition matters. This object
    // is known to be the one in your hand, because it was chosen by measuring
    // against the wrist. Ammo_Held only believes it knows, and it ranks by
    // spare rounds and write count, so the pistol you have carried all session
    // outranks the one you just drew. That was very likely why two different
    // pistols came back as the same weapon.
    //
    // The ammunition mark stays as a fallback for a weapon whose object does
    // not hold anything id-shaped there.
    AmmoSlot mag;
    const bool haveAmmo = Ammo_Held(mag);
    // THE MODEL HASH (2026-09-26, and this one is measured rather than
    // reasoned). Two weapon objects dumped side by side:
    //
    //     01027450  0F2F6C52  ...  00000201
    //     01028310  0F2F6452  ...  00000201
    //      vtable    +0x04          +0x14
    //
    // +0x04 differs between weapons and the two values are near neighbours,
    // which is what a hash of a model or resource name looks like. It is not
    // a pointer, it does not move, and it is the same number every launch, so
    // it is what a remembered per-weapon record should be filed under.
    //
    // Three earlier candidates are dead and worth naming so they are not tried
    // again: the ammunition mark at +24h is the ammunition CLASS (0201, 0202,
    // 0203 for handgun, machine gun, shotgun) and is shared by every weapon
    // that fires the same rounds; the ammunition record's +14h reads 1 on
    // everything; and the weapon object's +14h is that same class again.
    unsigned key = 0;
    if (!TryRead(&key, sk.object + 0x04, sizeof(key)) || !key)
        key = (haveAmmo ? mag.mark : 0u) << 8 | static_cast<unsigned>(sk.count & 0xFF);
    const unsigned mark = haveAmmo ? mag.mark : 0u;
    // WHAT MAKES ONE WEAPON DIFFERENT (2026-09-26, user: "switching pistols, it
    // didn't realize that was a different pistol I was on").
    //
    // Then this key does not identify a weapon, and rather than guess a fourth
    // time at what would, every part of it goes in the log the moment the
    // weapon in your hand changes. Two pistols side by side in that log will
    // say outright which field tells them apart - the mark, the bone count, the
    // capacity or none of them - and if none does, the answer is that weapon
    // identity has to come from somewhere that has not been looked at yet.
    // ON A CLOCK AS WELL AS ON A CHANGE (2026-09-26). Printing only when the
    // weapon changes means a session where nothing prints has two possible
    // explanations - it never changed, or this code never ran - and no way to
    // tell them apart afterwards. Three rounds of guessing at which one it was
    // is enough. Every five seconds it says what it thinks it is holding,
    // whether or not that is news.
    static unsigned char* s_toldFor = nullptr;
    static unsigned char* s_dumpedFor = nullptr;
    static unsigned long long s_toldAt = 0;
    const unsigned long long nowTold = GetTickCount64();
    if (sk.object != s_toldFor || nowTold - s_toldAt > 5000) {
        s_toldFor = sk.object;
        s_toldAt = nowTold;
        Log_Printf("MagBone: class %04X in hand, %s at %p, %d bone(s); %d of %d, %d spare -> weapon %08X",
            mark, sk.path, sk.object, sk.count, haveAmmo ? mag.loaded : -1, haveAmmo ? mag.capacity : -1,
            haveAmmo ? mag.reserve : -1, key);
        // THE WEAPON'S OWN ID (2026-09-26, user: "couldn't you pick it up off
        // of weapon ID?").
        //
        // Yes, and it belongs here rather than in the ammunition, because the
        // object below is known to be the one in your hand while the magazine
        // finder only believes it is. The id itself has not been located yet,
        // so this prints the head of the object and the search is done the way
        // the ammunition was found: by comparison. The user's own list is the
        // answer key - M92F is 1, SIG 556 is 9, Ithaca M37 is 12, Jail Breaker
        // is 14 - so drawing two known weapons and diffing these lines names
        // the offset outright. Any small number that changes between them and
        // matches that list is the field.
        if (sk.object && sk.object != s_dumpedFor) {
            s_dumpedFor = sk.object;
            for (int off = 0; off < 0x140; off += 32) {
                unsigned w[8] = {};
                if (!TryRead(w, sk.object + off, sizeof(w)))
                    break;
                Log_Printf("MagBone:   object+%02X: %08X %08X %08X %08X %08X %08X %08X %08X", off, w[0], w[1],
                    w[2], w[3], w[4], w[5], w[6], w[7]);
            }
        }
    }
    return key;
}

void LearnFrom(const GunSkeleton& sk, unsigned key)
{
    if (!g_learning || g_learnKey != key) {
        g_learning = true;
        g_learnKey = key;
        g_learnBones = sk.count;
        g_learnFrom = GetTickCount64();
        g_learnPrimed = false;
        for (int j = 0; j < kMaxGunJoints; ++j) {
            g_learnFar[j] = 0.0f;
            g_learnFirstMs[j] = 0;
        }
        Log_Printf("MagBone: weapon %08X is new - this reload is left alone so its magazine can be found", key);
    }
    float root[3], R[9];
    if (!JointWorldPos(sk.joints, 0, root) || !WorldRot(sk.joints, 0, R))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    const unsigned ms = static_cast<unsigned>(GetTickCount64() - g_learnFrom);
    for (int j = 0; j < sk.count && j < kMaxGunJoints; ++j) {
        float p[3];
        if (!JointWorldPos(sk.joints, j, p))
            continue;
        const float d[3] = { p[0] - root[0], p[1] - root[1], p[2] - root[2] };
        float rel[3];
        for (int r = 0; r < 3; ++r)
            rel[r] = (R[r * 3] * d[0] + R[r * 3 + 1] * d[1] + R[r * 3 + 2] * d[2]) * 100.0f / upm;
        if (!g_learnPrimed) {
            std::memcpy(g_learnStart[j], rel, sizeof(rel));
            continue;
        }
        const float away[3] = { rel[0] - g_learnStart[j][0], rel[1] - g_learnStart[j][1],
            rel[2] - g_learnStart[j][2] };
        // "far" is still a macro in the Windows headers.
        const float gone = Length3(away);
        if (gone > g_learnFar[j])
            g_learnFar[j] = gone;
        if (!g_learnFirstMs[j] && gone > kMovedCm)
            g_learnFirstMs[j] = ms ? ms : 1u;
    }
    g_learnPrimed = true;
}

void FinishLearning()
{
    if (!g_learning)
        return;
    g_learning = false;
    // A RELOAD LASTS ABOUT A SECOND (2026-09-26). The log caught one weapon
    // being learned and given up on six times inside three hundred
    // milliseconds, which is not a reload, it is the weapon being identified
    // differently from one frame to the next. A window too short to contain an
    // animation cannot have seen one, so it is thrown away rather than written
    // down as "nothing moved".
    const unsigned long long lasted = GetTickCount64() - g_learnFrom;
    if (lasted < 250) {
        return;
    }
    // Everything that moved, in guess order. The guess is furthest travelled
    // first, which is no better than the two rules before it - it is simply
    // where the list starts, and being wrong now costs one button press
    // instead of a rebuild.
    int bone[kMovers], travel[kMovers], n = 0;
    for (int j = 0; j < g_learnBones && j < kMaxGunJoints; ++j) {
        if (!g_learnFirstMs[j] || g_learnFar[j] < kMovedCm)
            continue;
        Log_Printf("MagBone:   bone %2d rests %.1f cm from the root, moved %.1f cm, first at +%u ms", j,
            Length3(g_learnStart[j]), g_learnFar[j], g_learnFirstMs[j]);
        if (n < kMovers) {
            bone[n] = j;
            travel[n] = static_cast<int>(g_learnFar[j] + 0.5f);
            ++n;
        }
    }
    if (!n) {
        Log_Printf("MagBone: nothing on weapon %08X moved during that reload - it will be watched again",
            g_learnKey);
        return;
    }
    for (int a = 0; a < n; ++a)
        for (int b = a + 1; b < n; ++b)
            if (travel[b] > travel[a]) {
                const int tb = travel[a], bb = bone[a];
                travel[a] = travel[b];
                bone[a] = bone[b];
                travel[b] = tb;
                bone[b] = bb;
            }
    KnownWeapon* k = SlotFor(g_learnKey);
    if (!k)
        return;
    k->count = n;
    k->pick = 0;
    for (int i = 0; i < kMovers; ++i) {
        k->bone[i] = i < n ? bone[i] : -1;
        k->travel[i] = i < n ? travel[i] : 0;
    }
    WriteKnown();
    int mag = -1, with = -1;
    BonesOf(*k, mag, with);
    Log_Printf("MagBone: weapon %08X has %d moving part(s); bone %d is taken to be the magazine, bone %d travels "
               "with it. If that is the wrong part, the menu will step to the next one.",
        g_learnKey, n, mag, with);
}

constexpr DWORD kOffControllerCharacterHere = 0x140;

bool ThisIsOurs(const void* character)
{
    unsigned char* controller = static_cast<unsigned char*>(CameraRigHook_GetPlayerController());
    if (!controller || !character)
        return false;
    unsigned char* ours = nullptr;
    if (!TryRead(&ours, controller + kOffControllerCharacterHere, sizeof(ours)) || !ours)
        return false;
    return static_cast<const void*>(ours) == character;
}

void MoveTheMagazine(const XrInputSettings& settings)
{
    const int state = XrInput_MagazineHeld();
    if (!settings.manualReload) {
        g_magFalling = false;
        g_magWasInHand = false;
        g_magWasState = 0;
        for (bool& had : g_haveMagRest)
            had = false;
        return;
    }
    if (g_gunSkCount == 0 || !g_haveBody || !g_body.joints)
        return;
    // WHOSE MAGAZINE (2026-09-29, user: "sheva cant manually reload, because
    // pressing the reload button drops chris magazine out of his hand").
    //
    // g_body is whichever body the finder last adopted, and this routine had
    // no ownership check of any kind - so with an AI partner on screen it
    // would happily pick the weapon nearest THEIR gun hand and drop THEIR
    // magazine on the floor. HideTheBody learned this same lesson when Sheva
    // was being cut in half; the magazine never did.
    //
    // Refuse unless the body we are about to reach into is the character the
    // player controller says we are playing, and say so once in a while,
    // because "nothing happened" and "it happened to the wrong person" look
    // identical from inside the headset.
    if (!ThisIsOurs(g_body.character)) {
        static unsigned long long s_toldWhose = 0;
        const unsigned long long nowWhose = GetTickCount64();
        if (nowWhose - s_toldWhose > 5000) {
            s_toldWhose = nowWhose;
            Log_Printf("Magazine: the body in hand is %p, which is not the character you are playing - "
                       "leaving the reload alone",
                g_body.character);
        }
        g_magFalling = false;
        g_magWasInHand = false;
        return;
    }
    const ArmIkArm& gunArm = settings.characterLeftHanded ? g_body.left : g_body.right;
    if (!gunArm.valid)
        return;
    float hp[3];
    if (!JointWorldPos(g_body.joints, gunArm.wrist, hp))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;

    // The weapon in your hand: of everything the search turned up, the one
    // whose root is nearest the hand holding it. Holstered weapons are on the
    // same list and must not have their magazines pulled out.
    // STICKY, because two of them can be in reach at once (2026-09-26). The
    // nearest weapon to your hand is the right answer and it is not a stable
    // one: a holstered weapon at the hip passes within range constantly, and
    // swapping between them from frame to frame restarted the learner six
    // times in a third of a second. Whichever was chosen last keeps the job
    // until something is clearly nearer, and a weapon put away loses it.
    // WHICH WEAPON IS IN YOUR HAND (2026-09-26, settled by measurement at last).
    //
    // Not by distance. Five weapons were drawn in turn and the ammunition
    // followed every one of them - 10 of 10, then 9 of 9, 45 of 45, 15 of 15,
    // 80 of 80 - while the skeleton chosen by nearness never changed once, and
    // stayed on the M92F throughout. RE5 keeps your carried weapons parented at
    // the hand and draws the active one, so they are ALL at zero centimetres
    // from the wrist and "nearest" is a coin toss. Every radius picked tonight
    // was tuning a measurement that cannot answer the question.
    //
    // The two halves identify each other instead. The ammunition record carries
    // a mark - 0201 on the M92F - and the weapon object carries the same number
    // at +14h. The ammunition is demonstrably tracking your weapon switches, so
    // the weapon is simply whichever skeleton's object agrees with it.
    //
    // Nearness is kept only as a fallback, for a weapon whose object holds
    // nothing id-shaped, and it is the reason the radius is generous now: it is
    // no longer deciding anything on its own.
    // THE ATTACH TRANSFORM, NOT BONE 0 (2026-09-26, and this is the measurement
    // that was in the logs the whole time).
    //
    // Every attempt tonight measured bone 0 of each weapon's SKELETON against
    // the wrist, and every one failed, because the game parents your carried
    // weapons at the hand and draws the active one - all of them read about
    // zero and the choice was a coin toss. Four radii were tuned against a
    // number that could not answer the question.
    //
    // The attachment scan has been printing the answer once a second all along:
    //
    //     +0x2200+0xD0   8.2 cm from the hand
    //     +0x2230+0xD0   8.2 cm
    //     +0x2260+0xD0   0.0 cm      <- the weapon being drawn
    //     +0x2290+0xD0  48.3 cm
    //
    // That is the weapon's ATTACH transform - the translation of the matrix at
    // +0xA0, which is where the game actually places a weapon in a hand - and
    // it separates the held weapon from the carried ones by eight centimetres
    // at worst. It is not an inference about ids or timings or where magazines
    // live; it is the position the game itself assigns, and it was measured
    // before any of this started.
    // BACK TO THE SIMPLE ONE (2026-09-26, user: "with the scan option,
    // sometimes switching weapons doesn't work, even if I manually set it to 7
    // and 8").
    //
    // Which is the selection, not the bone numbers, and it is something I made
    // worse tonight. The original took the nearest weapon to your gun hand and
    // that was enough. On top of it I added an attach-transform measurement, a
    // loyalty credit for whichever was chosen last, and finally a match on the
    // ammunition's class - three fixes for a problem that the simple version
    // did not have in practice, each of which introduced a way for a weapon
    // switch not to register.
    //
    // So it is the nearest one again, with no memory and no cleverness. If two
    // weapons genuinely sit on top of each other it may pick wrong, which is
    // the failure the additions were chasing - but a wrong pick that follows
    // your switches beats a right pick that sticks to the gun you put away.
    const GunSkeleton* sk = nullptr;
    float best = 0.0f;
    for (int s = 0; s < g_gunSkCount && s < kMaxGunSkeletons && !sk; ++s) {
        float p[3];
        if (!g_gunSk[s].object
            || !TryRead(p, g_gunSk[s].object + kOffAttachCurrent + 0x30, sizeof(p)))
            continue;
        const float d[3] = { p[0] - hp[0], p[1] - hp[1], p[2] - hp[2] };
        float cm = Length3(d) * 100.0f / upm;
        // HOW NEAR IS IN YOUR HAND (2026-09-26). Forty was too generous: a
        // holstered weapon sits inside it, and with the loyalty credit below
        // that meant drawing a different gun never took the job away.
        //
        // Fifteen was then too tight, and for an embarrassing reason - it came
        // off the ATTACHMENT scan, which measures the weapon's attach transform
        // at 0.0 cm from the wrist. This measures bone 0 of the weapon's
        // SKELETON, which is a different point, and the whole search stopped
        // matching anything at all. Twenty five is what the rest of this file
        // already uses for "this weapon is on this hand", and the log below
        // prints what was actually measured so the next change to this number
        // comes from a reading rather than from a neighbouring one.
        // Four centimetres. The drawn weapon measures zero and the nearest
        // stowed one measured eight, so this has room on both sides - and no
        // loyalty credit, because with a gap that clear there is nothing for
        // stickiness to fix and plenty for it to break.
        if (cm > 4.0f)
            continue;
        if (!sk || cm < best) {
            sk = &g_gunSk[s];
            best = cm;
        }
    }
    // THE MAGAZINE BELONGS TO THIS WEAPON (2026-09-29, user: "when I press
    // reload, chris current ammo drops to 0. instead of shevas").
    //
    // sk is the weapon nearest YOUR gun hand, so it is the one to reload. The
    // ammunition module was picking the busiest magazine instead, and with an
    // AI partner firing constantly that is always theirs, in both directions:
    // "I have never once had it eject sheva mag while playing as chris".
    //
    // The weapon object and its magazine share a number at +14h, so handing
    // that over pins every read and write below to this gun.
    {
        unsigned weaponId = 0;
        const bool haveId = sk && sk->object && TryRead(&weaponId, sk->object + 0x14, sizeof(weaponId));
        Ammo_PreferWeaponId(weaponId, haveId);
    }
    if (!sk) {
        // Nothing was close enough to be in your hand. Says how close the
        // nearest one got, because a search that silently matches nothing is
        // indistinguishable from a feature that is switched off.
        static unsigned long long s_toldNone = 0;
        const unsigned long long nowNone = GetTickCount64();
        if (g_gunSkCount && nowNone - s_toldNone > 3000) {
            s_toldNone = nowNone;
            char how[160] = {};
            int at = 0;
            for (int s = 0; s < g_gunSkCount && s < kMaxGunSkeletons && at < 120; ++s) {
                float p[3];
                if (!g_gunSk[s].object
                    || !TryRead(p, g_gunSk[s].object + kOffAttachCurrent + 0x30, sizeof(p)))
                    continue;
                const float d[3] = { p[0] - hp[0], p[1] - hp[1], p[2] - hp[2] };
                at += sprintf_s(how + at, sizeof(how) - at, "%s%.1f", at ? ", " : "",
                    Length3(d) * 100.0f / upm);
            }
            Log_Printf("MagBone: nothing is being held - %d weapon(s), attach transforms at %s cm from the hand",
                g_gunSkCount, how);
        }
        return;
    }

    // Learned, or set by hand. A bone number in the menu overrides everything
    // for every weapon, which is what it is for: checking a guess without a
    // build. Leave it at -1 and the mod works it out per weapon.
    const unsigned key = WeaponKey(*sk);
    g_lastKey = key;
    int bone = settings.magazineBone;
    int withBone = settings.magazineWithBone;
    if (bone < 0) {
        if (const KnownWeapon* k = FindKnown(key))
            BonesOf(*k, bone, withBone);
    }
    if (bone < 0) {
        // Not known yet. Watch this reload rather than taking it over: the
        // ammunition still works, the gesture still works, and the only thing
        // missing for one reload is the magazine you can see.
        if (state == 3)
            LearnFrom(*sk, key);
        else if (g_learning)
            FinishLearning();
        g_magWasState = state;
        return;
    }
    if (g_learning)
        FinishLearning();
    if (bone >= sk->count || bone >= kMaxGunJoints)
        return;

    // The weapon's own frame, which is what both learning and putting back are
    // expressed in: a magazine seated in a gun does not care where the gun is.
    float rootPos[3], rootRot[9], rootT[9];
    const bool haveRoot = JointWorldPos(sk->joints, 0, rootPos) && WorldRot(sk->joints, 0, rootRot);
    if (haveRoot)
        Transpose3(rootRot, rootT);

    const int parts[kMagParts] = { bone, withBone };

    if (state == 3) {
        // Pinned. The magazine was put here by your hand and the animation is
        // not allowed to argue with that.
        if (!haveRoot)
            return;
        for (int q = 0; q < kMagParts; ++q) {
            const int b = parts[q];
            if (b < 0 || b >= sk->count || b >= kMaxGunJoints || !g_haveMagRest[q])
                continue;
            if (q && b == bone)
                continue;
            float rows[9];
            Mul3(g_magRestRot[q], rootRot, rows);
            float pos[3];
            for (int k = 0; k < 3; ++k)
                pos[k] = rootPos[k] + g_magRestPos[q][0] * rootRot[k] + g_magRestPos[q][1] * rootRot[3 + k]
                    + g_magRestPos[q][2] * rootRot[6 + k];
            WriteBoneWorld(sk->joints, b, rows, pos, g_magRestLen[q]);
        }
        g_magFalling = false;
        g_magWasInHand = false;
        g_magWasState = 3;
        return;
    }

    // Whatever the model scales its rows by, kept. Overwriting a bone's length
    // with one is how a gun ends up the wrong size.
    float cur[16];
    if (!TryRead(cur, sk->joints + bone * kJointStride + kOffJointWorldMatrix, sizeof(cur)))
        return;
    float scale[3];
    for (int r = 0; r < 3; ++r) {
        scale[r] = std::sqrt(cur[r * 4] * cur[r * 4] + cur[r * 4 + 1] * cur[r * 4 + 1]
            + cur[r * 4 + 2] * cur[r * 4 + 2]);
        if (!(scale[r] > 0.001f))
            scale[r] = 1.0f;
    }

    if (state == 0) {
        // Nothing is happening to the magazine, so wherever it is IS where it
        // belongs. Taken every frame, in the weapon's own frame, and the last
        // reading before a reload starts is the seated one - which is the
        // reading the reload puts back.
        if (haveRoot) {
            for (int q = 0; q < kMagParts; ++q) {
                const int b = parts[q];
                if (b < 0 || b >= sk->count || b >= kMaxGunJoints)
                    continue;
                float m[16];
                if (!TryRead(m, sk->joints + b * kJointStride + kOffJointWorldMatrix, sizeof(m)))
                    continue;
                float rows[9];
                for (int r = 0; r < 3; ++r) {
                    g_magRestLen[q][r] = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1]
                        + m[r * 4 + 2] * m[r * 4 + 2]);
                    if (!(g_magRestLen[q][r] > 0.001f))
                        g_magRestLen[q][r] = 1.0f;
                    for (int k = 0; k < 3; ++k)
                        rows[r * 3 + k] = m[r * 4 + k] / g_magRestLen[q][r];
                }
                Mul3(rows, rootT, g_magRestRot[q]);
                const float d[3] = { m[12] - rootPos[0], m[13] - rootPos[1], m[14] - rootPos[2] };
                for (int r = 0; r < 3; ++r)
                    g_magRestPos[q][r] = rootRot[r * 3] * d[0] + rootRot[r * 3 + 1] * d[1]
                        + rootRot[r * 3 + 2] * d[2];
                g_haveMagRest[q] = true;
            }
        }
        g_magFalling = false;
        g_magWasInHand = false;
        g_magWasState = 0;
        return;
    }

    const unsigned long long now = GetTickCount64();

    if (state == 2) {
        // IN YOUR HAND, WITH WHATEVER RIDES ON IT (2026-09-25, user: "7 is the
        // magazine, 8 is a bullet casing ... when you move the magazine from
        // your hip, both 7 and 8 should be grabbed").
        //
        // So this is not one bone written to one place, it is a rigid MOVE:
        // work out the transform that takes the magazine from where the model
        // has it to where your hand is, and apply that same transform to
        // everything travelling with it. The round keeps its place on top of
        // the magazine because it is carried by the same arithmetic rather
        // than aimed at the hand separately.
        const ArmIkArm& offArm = settings.characterLeftHanded ? g_body.right : g_body.left;
        if (!offArm.valid)
            return;
        float wr[9], wp[3];
        if (!WorldRot(g_body.joints, offArm.wrist, wr) || !JointWorldPos(g_body.joints, offArm.wrist, wp))
            return;
        // THE RIGHT WAY UP (2026-09-26, user: "when pulling the mag from the
        // hip, its backwards, it needs to roll 180 degrees, otherwise you're
        // putting the mag in the wrong way").
        //
        // The magazine is handed the wrist's own rotation, and the wrist is
        // not the magazine: a hand reaching down to a belt comes back holding
        // it upside down relative to how it goes into a weapon. Half a turn
        // about the hand's forward axis, which is negating the right and the
        // up rows and leaving forward alone.
        for (int r = 0; r < 2; ++r)
            for (int k = 0; k < 3; ++k)
                wr[r * 3 + k] = -wr[r * 3 + k];
        // A few centimetres out of the wrist, into where fingers close.
        const float outCm = 6.0f * upm / 100.0f;
        float wantPos[3];
        for (int k = 0; k < 3; ++k)
            wantPos[k] = wp[k] + wr[6 + k] * outCm;

        float isRows[9];
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k)
                isRows[r * 3 + k] = cur[r * 4 + k] / scale[r];
        const float isPos[3] = { cur[12], cur[13], cur[14] };
        float isT[9], move[9];
        Transpose3(isRows, isT);
        Mul3(isT, wr, move);

        for (int q = 0; q < kMagParts; ++q) {
            const int b = parts[q];
            if (b < 0 || b >= sk->count || b >= kMaxGunJoints)
                continue;
            if (q && b == bone)
                continue;
            float m[16];
            if (!TryRead(m, sk->joints + b * kJointStride + kOffJointWorldMatrix, sizeof(m)))
                continue;
            float len[3], rows[9];
            for (int r = 0; r < 3; ++r) {
                len[r] = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
                if (!(len[r] > 0.001f))
                    len[r] = 1.0f;
                for (int k = 0; k < 3; ++k)
                    rows[r * 3 + k] = m[r * 4 + k] / len[r];
            }
            float newRows[9];
            Mul3(rows, move, newRows);
            const float d[3] = { m[12] - isPos[0], m[13] - isPos[1], m[14] - isPos[2] };
            float pos[3];
            for (int k = 0; k < 3; ++k)
                pos[k] = wantPos[k] + d[0] * move[k] + d[1] * move[3 + k] + d[2] * move[6 + k];
            WriteBoneWorld(sk->joints, b, newRows, pos, len);
            if (b == bone) {
                std::memcpy(g_magHeldRows, newRows, sizeof(g_magHeldRows));
                std::memcpy(g_magHeldPos, pos, sizeof(g_magHeldPos));
                g_magWasInHand = true;
            }
        }
        g_magFalling = false;
        g_magWasState = 2;
        return;
    }

    // Out of the gun and falling. Caught where it was on the frame it left,
    // and from then on it is only arithmetic - the game is not animating it,
    // because as far as the game is concerned the weapon never reloaded.
    if (!g_magFalling || g_magWasState != 1) {
        const bool fromTheHand = g_magWasState == 2 && g_magWasInHand;
        for (int k = 0; k < 3; ++k) {
            g_magFallPos[k] = fromTheHand ? g_magHeldPos[k] : cur[12 + k];
            g_magFallVel[k] = 0.0f;
        }
        if (fromTheHand) {
            std::memcpy(g_magFallRot, g_magHeldRows, sizeof(g_magFallRot));
        } else {
            for (int r = 0; r < 3; ++r)
                for (int k = 0; k < 3; ++k)
                    g_magFallRot[r * 3 + k] = cur[r * 4 + k] / scale[r];
        }
        // A push down and away, so it clears the grip instead of sinking
        // through it.
        g_magFallVel[1] = -0.35f * upm;
        g_magFalling = true;
        g_magFallLast = now;
    }
    float dt = static_cast<float>(now - g_magFallLast) / 1000.0f;
    g_magFallLast = now;
    if (dt > 0.1f)
        dt = 0.1f; // a stall must not teleport it through the floor
    g_magFallVel[1] -= 9.8f * upm * dt;
    for (int k = 0; k < 3; ++k)
        g_magFallPos[k] += g_magFallVel[k] * dt;
    // Two metres below the hand is under any floor you are standing on, and
    // there is nothing to gain by integrating it to the centre of the earth.
    const float floorAt = hp[1] - 2.0f * upm;
    if (g_magFallPos[1] < floorAt) {
        g_magFallPos[1] = floorAt;
        g_magFallVel[1] = 0.0f;
    }
    WriteBoneWorld(sk->joints, bone, g_magFallRot, g_magFallPos, scale);
    g_magWasState = 1;
}
} // namespace

bool ArmIk_MagazinePick(int& bone, int& which, int& outOf)
{
    const KnownWeapon* k = g_lastKey ? FindKnown(g_lastKey) : nullptr;
    if (!k || k->count <= 0)
        return false;
    int with = -1;
    BonesOf(*k, bone, with);
    which = k->pick + 1;
    outOf = k->count;
    return bone >= 0;
}

void ArmIk_NextMagazinePick()
{
    KnownWeapon* k = nullptr;
    for (int i = 0; i < g_knownCount && !k; ++i)
        if (g_known[i].key == g_lastKey)
            k = &g_known[i];
    if (!k || k->count <= 1) {
        Log_Printf("MagBone: only one moving part was found on this weapon, so there is nothing to step to");
        return;
    }
    k->pick = (k->pick + 1) % k->count;
    WriteKnown();
    int mag = -1, with = -1;
    BonesOf(*k, mag, with);
    Log_Printf("MagBone: weapon %08X - trying bone %d as the magazine instead (%d of %d), with bone %d", k->key,
        mag, k->pick + 1, k->count, with);
}

bool ArmIk_PlayerSkeleton(unsigned char** joints, int* count)
{
    if (!g_haveBody || !g_body.joints || g_body.jointCount <= 0)
        return false;
    if (joints)
        *joints = g_body.joints;
    if (count)
        *count = g_body.jointCount;
    return true;
}

bool ArmIk_Landmarks(ArmIkLandmarks& out)
{
    if (!g_haveBody || !g_body.joints)
        return false;
    out.head = g_body.head;
    out.shoulder[0] = g_body.left.valid ? g_body.left.shoulder : -1;
    out.elbow[0] = g_body.left.valid ? g_body.left.elbow : -1;
    out.wrist[0] = g_body.left.valid ? g_body.left.wrist : -1;
    out.shoulder[1] = g_body.right.valid ? g_body.right.shoulder : -1;
    out.elbow[1] = g_body.right.valid ? g_body.right.elbow : -1;
    out.wrist[1] = g_body.right.valid ? g_body.right.wrist : -1;
    return true;
}

int ArmIk_JointParent(unsigned char* joints, int index)
{
    unsigned char links[4] = {};
    if (!joints || index < 0 || index >= kMaxJoints
        || !TryRead(links, joints + index * kJointStride + kOffJointLinks, sizeof(links)))
        return -1;
    const int parent = links[1];
    return parent == index ? -1 : parent;
}

bool ArmIk_JointWorldMatrix(unsigned char* joints, int index, float out[16])
{
    if (!joints || index < 0 || index >= kMaxJoints)
        return false;
    return TryRead(out, joints + index * kJointStride + kOffJointWorldMatrix, sizeof(float) * 16);
}

void ArmIk_DumpGunBones()
{
    if (g_gunSkCount == 0) {
        Log_Printf("GunBones: no weapon skeleton found yet - hold a gun and raise it once");
        return;
    }
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    for (int s = 0; s < g_gunSkCount && s < kMaxGunSkeletons; ++s) {
        const GunSkeleton& g = g_gunSk[s];
        Log_Printf("GunBones: [%d] %s - %d bone(s), object %p, the grip is bone %d", s, g.path, g.count, g.object,
            g.anchor);
        // Against the ROOT and in the ROOT'S OWN AXES. Measuring a bone's
        // offset in world terms says nothing about the weapon, only about
        // which way you happen to be holding it.
        float gp[3] = {}, R[9] = {};
        if (!JointWorldPos(g.joints, 0, gp) || !WorldRot(g.joints, 0, R))
            continue;
        for (int j = 0; j < g.count && j < kMaxGunJoints; ++j) {
            float p[3];
            if (!JointWorldPos(g.joints, j, p))
                continue;
            const float d[3] = { p[0] - gp[0], p[1] - gp[1], p[2] - gp[2] };
            float rel[3];
            for (int r = 0; r < 3; ++r)
                rel[r] = (R[r * 3] * d[0] + R[r * 3 + 1] * d[1] + R[r * 3 + 2] * d[2]) * 100.0f / upm;
            Log_Printf("GunBones:   %2d: right %+6.1f  up %+6.1f  fwd %+6.1f cm from the root", j, rel[0], rel[1],
                rel[2]);
        }
    }
}

// The gun hold proper. Called only from ArmIk_SteadyGun below, which is what
// the rest of the mod calls: everything in here is allowed to give up early,
// and the magazine is not.
void SteadyGunNow()
{
    const XrInputSettings settings = XrInput_GetSettings();
    // Off means off (2026-09-22): unticked, this returns before touching
    // anything, so a build with it in behaves exactly like one without.
    const bool wantScale = std::fabs(settings.gunScale - 1.0f) > 0.005f;
    if (!settings.gunLock && !wantScale && g_gunSourceState != 1) {
        InterlockedExchange(&g_gunLockOn, 0);
        return;
    }
    // TAKE THE GUN BACK AFTERWARDS (2026-09-28, user: "the weapon sticking
    // back to your hand after an assist jump").
    //
    // The guard below stops this routine entirely during a scripted camera,
    // and an assist jump is one. So for the whole animation the game owns
    // the weapon and puts it wherever the clip says - which is correct, we
    // do not want to fight a cutscene. The problem is the frame after.
    //
    // On that frame two pieces of state are stale. The steadiness test below
    // compares your wrist against where it was BEFORE the jump, so it reads
    // a large movement and refuses to believe anything. And the learned
    // attach placement still describes a grip from before the animation
    // moved the gun, so the displacement gets taken up as though you had
    // deliberately regripped. Between them the gun stays where the jump left
    // it until something else forces a fresh read, which is why re-aiming
    // fixes it.
    //
    // A scripted camera is, as far as the weapon is concerned, exactly as
    // disruptive as a change of character - and that already clears the
    // learned placement a few lines down. So do the same thing here.
    static bool s_wasScripted = false;
    static bool s_forgetGrip = false;
    static bool s_dropPrevWrist = false;
    static bool s_takeGripNow = false;

    // DOES OUR WRITE SURVIVE THE FRAME? (2026-09-28, user: "it wobbles
    // around as you move your hand vs being flush and moving with the gun",
    // and then "its definitely lagging behind".)
    //
    // The weapon transforms are moved rigidly, from this frame\x27s bones, in
    // the same place the bones are moved. If that write stood, the muzzle
    // could not lag. So either it does not stand - something recomputes the
    // weapon from the pose we just replaced, and every frame we are one
    // behind - or the beam is not drawn from these transforms at all and we
    // have been moving the wrong thing.
    //
    // One number separates those: remember what we wrote into the weapon,
    // and look at the same address next frame before writing again. If it is
    // still ours, nothing is fighting us and the laser comes from elsewhere.
    // If it has moved back, that distance IS the lag, and it will grow with
    // how fast your hand is going.
    static float s_lagStepCm = 0.0f;
    static float s_lagMaxStepCm = 0.0f;
    static float s_lagMaxDriftCm = 0.0f;
    static unsigned char* s_lagWatchAt = nullptr;
    static float s_lagWrote[3] = { 0.0f, 0.0f, 0.0f };
    static bool s_lagHaveWrote = false;
    static bool s_lagWatchPoor = true; // nothing better than the first match yet
    static unsigned long long s_lagSaidMs = 0;
    const bool scripted = CameraRigHook_InScriptedCamera();
    if (s_wasScripted && !scripted) {
        s_forgetGrip = true;
        s_dropPrevWrist = true;
        // AND TAKE IT AT ONCE (2026-09-28, user: "the gun hides when you
        // assist jump so it has to re-seat itself to your hand and takes a
        // second to find it").
        //
        // Clearing the learned grip was right but not enough. The placement
        // is only ever taken when your hand is still, and dropping the
        // previous wrist reading means there is nothing to judge stillness
        // against - so the first frame back can never qualify, and the gun
        // waits for you to hold your hand steady before it comes home. That
        // wait is the second he is describing.
        //
        // The one moment we KNOW the grip is stale is this one, so this is
        // the one moment worth taking it regardless of what your hand is
        // doing. Everywhere else the steadiness rule still applies.
        s_takeGripNow = true;
    }
    s_wasScripted = scripted;

    if ((!settings.armIk && !settings.findGunWriter) || !g_haveBody || !g_body.joints || !g_body.character
        || !g_body.left.valid || !g_body.right.valid || scripted)
        return;
    const ArmIkArm& gun = settings.characterLeftHanded ? g_body.left : g_body.right;
    float hp[3], hr[9];
    if (!JointWorldPos(g_body.joints, gun.wrist, hp) || !WorldRot(g_body.joints, gun.wrist, hr))
        return;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    if (g_attachCharacter != g_body.character) {
        g_attachCharacter = g_body.character;
        std::memset(g_attach, 0, sizeof(g_attach));
    }
    if (s_forgetGrip) {
        s_forgetGrip = false;
        std::memset(g_attach, 0, sizeof(g_attach));
        s_lagWatchAt = nullptr;
        s_lagHaveWrote = false;
    }
    const unsigned long long nowMs = GetTickCount64();
    const unsigned long long shot = XrInput_LastShotMs();
    // A second, not a third of one (2026-09-23, user: "it's still not quite a
    // real 0 recoil possibility when set that way"). A shotgun's kick and its
    // pump run for the best part of a second, so a 300 ms window let the tail
    // of every shot through and, worse, let the held placement take the
    // animation up as if it were a new grip.
    const bool kicking = shot && nowMs - shot < 1000;
    // How much of what the game is doing to the gun gets through. While firing
    // that is the kick sliders; the rest of the time it is nothing at all, and
    // the gun is rigid in your hand. Settling by degrees was tried and thrown
    // out (2026-09-23, user: "no amount of settling changed it. That only
    // changed how long the gun took to get back to my hand") - what a real
    // change of grip needs is not a slower blend, it is being recognised as a
    // change, which the held placement above now does by how long it lasts.
    float share = kicking ? (XrInput_GetTwoHand(nullptr) ? settings.kickTwoHanded : settings.kickOneHanded)
                          : 0.0f;
    share = share > 1.0f ? 1.0f : (share > 0.0f ? share : 0.0f);
    InterlockedExchange(&g_gunLockOn, settings.gunLock ? 1 : 0);

    // Is your hand still enough to believe it? (2026-09-23, user: "just
    // shaking my controller really fast caused my gun to stay in a new
    // location".) The rule below accepts a difference that lasts as a real
    // change of grip, and while you are swinging your hand there IS one: the
    // game builds the gun from where your hand was a frame ago, so it reads
    // as a placement that has moved and stayed moved. It is only ever the
    // truth when your hand is still, so that is when it is allowed to count.
    static float s_prevHp[3], s_prevHr[9];
    static bool s_havePrevWrist = false;
    if (s_dropPrevWrist) {
        // The wrist reading either side of an animation is not a movement of
        // your hand, and judging steadiness across it means judging it
        // against something that never happened.
        s_dropPrevWrist = false;
        s_havePrevWrist = false;
    }
    bool handSteady = false;
    if (s_takeGripNow) {
        s_takeGripNow = false;
        handSteady = true;
    } else if (s_havePrevWrist) {
        float dh[3];
        Sub3(hp, s_prevHp, dh);
        float tr = 0.0f;
        for (int r = 0; r < 3; ++r)
            tr += Dot3(hr + r * 3, s_prevHr + r * 3);
        float cs = (tr - 1.0f) * 0.5f;
        cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
        s_lagStepCm = Length3(dh) * 100.0f / upm;
        if (s_lagStepCm > s_lagMaxStepCm)
            s_lagMaxStepCm = s_lagStepCm;
        handSteady = s_lagStepCm < 0.8f && std::acos(cs) * 57.2957795f < 2.0f;
    }
    std::memcpy(s_prevHp, hp, sizeof(s_prevHp));
    std::memcpy(s_prevHr, hr, sizeof(s_prevHr));
    s_havePrevWrist = true;

    // Raising or lowering the gun is a real change of grip, so the hold is
    // dropped - but NOT taken again there and then (2026-09-23, user: "if I'm
    // mid motion, and press the aim button, the gun freezes where I pressed
    // aim, then moves to my hand"). A reading taken mid-swing is the lag, and
    // holding the gun at the lag is exactly what he saw. Until your hand is
    // still the gun is simply left alone, which is how it behaved before any
    // of this. It is also the way out if one ever gets stuck: lower the gun
    // and raise it.
    // ...and raising the gun does NOT drop it any more (2026-09-23, user:
    // "there's still that little gun settle that happens when you first
    // aim"). Dropping it there meant showing the game's own settle while
    // waiting for a still hand to read from, and the grip does not change
    // between held low and aimed anyway - it is the same hand on the same
    // gun. A weapon change brings a new skeleton, which is read afresh, and
    // anything wildly out of place is taken up below.

    static unsigned s_frames = 0, s_held = 0, s_written = 0, s_wroteCurrent = 0;
    static unsigned s_boneWrites = 0;
    static float s_boneMaxDeg = 0.0f, s_boneMaxCm = 0.0f;
    static float s_maxFixDeg = 0.0f, s_maxFixCm = 0.0f;
    static unsigned long long s_toldMs = 0;
    g_sourceShare = share;
    g_attachLeftHanded = settings.characterLeftHanded;
    g_gunScaleWanted = settings.gunScale < 0.5f ? 0.5f : (settings.gunScale > 1.5f ? 1.5f : settings.gunScale);
    if ((settings.gunLock || wantScale) && g_gunSourceState == 0) {
        // Walk the store run from the instruction the watch caught, accepting
        // only the two shapes it is made of - both fixed-length, neither with
        // a relative operand, so either can be moved into a trampoline:
        //     F3 0F 10 44 24 xx          movss xmm0, [esp+xx]      (6 bytes)
        //     F3 0F 11 /r(mod 10, esi) d32  movss [esi+d32], xmmN  (8 bytes)
        // and hook the instruction after the store to +0xDC, the matrix's last
        // element. Anything else on the way and nothing is patched.
        unsigned char* const start = reinterpret_cast<unsigned char*>(GetModuleHandleA(nullptr)) + kGunWriterRva;
        unsigned char code[96] = {};
        TryRead(code, start, sizeof(code));
        // The run is not in element order (2026-09-22): the first hook went in
        // after the store to +0xDC and the store to +0xD8 - the position's z -
        // came after it, so every corrected z was overwritten. So walk the
        // whole run, see both +0xD8 and +0xDC go by, and hook where the stores
        // end. Measured, that is
        //     0F B7 46 22   movzx eax, word [esi+22]
        //     83 E8 00      sub eax, 0
        // seven bytes with nothing relative in them, followed by the jumps of
        // a switch. Only that exact pair is accepted as the site.
        int at = 0, hookAt = -1;
        bool sawD8 = false, sawDC = false;
        while (at + 8 <= static_cast<int>(sizeof(code))) {
            const unsigned char* c = code + at;
            if (c[0] == 0xF3 && c[1] == 0x0F && c[2] == 0x10 && c[3] == 0x44 && c[4] == 0x24) {
                at += 6;
            } else if (c[0] == 0xF3 && c[1] == 0x0F && c[2] == 0x11 && (c[3] & 0xC7) == 0x86) {
                const unsigned int disp = c[4] | (c[5] << 8) | (c[6] << 16) | (static_cast<unsigned int>(c[7]) << 24);
                at += 8;
                sawD8 = sawD8 || disp == 0xD8;
                sawDC = sawDC || disp == 0xDC;
            } else {
                break;
            }
        }
        bool siteOk = false;
        if (sawD8 && sawDC && at + 7 <= static_cast<int>(sizeof(code))) {
            const unsigned char* c = code + at;
            siteOk = c[0] == 0x0F && c[1] == 0xB7 && c[2] == 0x46 && c[4] == 0x83 && c[5] == 0xE8;
            if (siteOk)
                hookAt = at;
        }
        MH_STATUS st = MH_ERROR_UNSUPPORTED_FUNCTION;
        unsigned char* target = nullptr;
        if (siteOk) {
            target = start + hookAt;
            st = MH_CreateHook(target, reinterpret_cast<void*>(&GunWriteHook_Stub), &g_gunWriteTrampoline);
            if (st == MH_OK)
                st = MH_EnableHook(target);
        }
        InterlockedExchange(&g_gunSourceState, st == MH_OK ? 1 : -1);
        char hex[96 * 3 + 1] = {};
        for (int i = 0; i < 64; ++i)
            sprintf_s(hex + i * 3, 4, "%02X ", code[i]);
        if (st == MH_OK) {
            Log_Printf("GunKick: source hook at re5dx9.exe+%lX, %d bytes past the caught store -> on",
                static_cast<unsigned long>(kGunWriterRva + hookAt), hookAt);
        } else {
            Log_Printf("GunKick: source hook NOT placed (%d) - the store run from re5dx9.exe+%lX did not end the way "
                       "it should (walked %d bytes, end %d); code: %s",
                static_cast<int>(st), static_cast<unsigned long>(kGunWriterRva), at, hookAt, hex);
        }
        Log_Printf("GunKick: code from the caught store on: %s", hex);
    } else if (g_gunSourceState == 1 && !settings.gunLock && !wantScale) {
        InterlockedExchange(&g_gunSourceState, 2);
        Log_Printf("GunKick: source hook switched off");
    } else if (g_gunSourceState == 2 && (settings.gunLock || wantScale)) {
        InterlockedExchange(&g_gunSourceState, 1);
        Log_Printf("GunKick: source hook switched back on");
    }
    const bool atSource = g_gunSourceState == 1;

    float hrT[9];
    Transpose3(hr, hrT);
    for (int e = 0; e < kAttachEntries; ++e) {
        unsigned char* object = nullptr;
        if (!TryRead(&object, g_body.character + kOffAttachTable + e * kAttachStride, sizeof(object))
            || reinterpret_cast<uintptr_t>(object) < 0x10000)
            continue;
        AttachRest& a = g_attach[e];
        if (a.object != object) {
            std::memset(&a, 0, sizeof(a));
            a.object = object;
        }
        float m[16];
        if (!TryRead(m, object + kOffAttachWorld, sizeof(m)) || std::fabs(m[15] - 1.0f) > 1e-3f)
            continue;
        // The current transform, if +0xA0 holds one that sits on the hand
        // like +0xE0 does. Written to only when it passes: a local offset or
        // anything else living there must not get a world matrix poured in.
        bool haveCurrent = false;
        {
            float c[16];
            if (TryRead(c, object + kOffAttachCurrent, sizeof(c)) && std::fabs(c[15] - 1.0f) < 1e-3f
                && std::fabs(c[3]) < 1e-3f && std::fabs(c[7]) < 1e-3f && std::fabs(c[11]) < 1e-3f) {
                const float dc[3] = { c[12] - m[12], c[13] - m[13], c[14] - m[14] };
                const float l0 = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
                if (Length3(dc) * 100.0f / upm < 30.0f && l0 > 0.5f && l0 < 2.0f) {
                    std::memcpy(m, c, sizeof(m));
                    haveCurrent = true;
                }
            }
        }
        // Did last frame's correction survive until now? If the game
        // recomposed this transform after we wrote it, it will not match.
        if (a.wroteValid) {
            ++s_frames;
            bool same = true;
            for (int i = 0; i < 16 && same; ++i)
                same = m[i] == a.wrote[i];
            if (same)
                ++s_held;
            else
                a.wroteValid = false; // kept while it holds, or the source hook would steady its own result twice
        }
        const float d[3] = { m[12] - hp[0], m[13] - hp[1], m[14] - hp[2] };
        a.onHand = Length3(d) * 100.0f / upm <= 25.0f;
        if (!a.onHand)
            continue; // not on the gun hand


        const float scale = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        if (!(scale > 0.01f))
            continue;
        float rows[9];
        bool ok = true;
        for (int r = 0; r < 3 && ok; ++r) {
            float v[3] = { m[r * 4], m[r * 4 + 1], m[r * 4 + 2] };
            ok = Normalise3(v);
            std::memcpy(rows + r * 3, v, sizeof(v));
        }
        if (!ok)
            continue;
        float relRot[9], relPos[3];
        Mul3(rows, hrT, relRot);
        for (int r = 0; r < 3; ++r)
            relPos[r] = Dot3(hr + r * 3, d);

        if (!kicking || !a.have) {
            if (!a.have) {
                std::memcpy(a.rot, relRot, sizeof(relRot));
                std::memcpy(a.pos, relPos, sizeof(relPos));
                a.have = true;
                a.offSince = 0;
            } else {
                // A new grip, or just an animation? (2026-09-23, user: "still
                // have that slight drift of weapons when moving my hand
                // around".) Taking the placement as it comes brings the lag
                // with it and the gun trails your hand; holding it outright
                // would stick on a draw or a reload. So it is held, and only a
                // difference that LASTS counts as a real change of grip.
                float tr = 0.0f;
                for (int r = 0; r < 3; ++r)
                    tr += Dot3(a.rot + r * 3, relRot + r * 3);
                float cs = (tr - 1.0f) * 0.5f;
                cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
                float dp[3];
                Sub3(a.pos, relPos, dp);
                const float devCm = Length3(dp) * 100.0f / upm;
                if (!handSteady) {
                    a.offSince = 0; // nothing read while you are moving counts
                } else if (std::acos(cs) * 57.2957795f > 3.0f || devCm > 1.5f) {
                    if (!a.offSince)
                        a.offSince = nowMs;
                    if (nowMs - a.offSince > 400 || devCm > 50.0f) {
                        float taken[9];
                        BlendRot(a.rot, relRot, 0.5f, taken);
                        std::memcpy(a.rot, taken, sizeof(taken));
                        for (int i = 0; i < 3; ++i)
                            a.pos[i] += (relPos[i] - a.pos[i]) * 0.5f;
                    }
                } else {
                    a.offSince = 0;
                }
            }
            if (!kicking)
                continue;
        }

        // A burst: the resting place plus the chosen share of the kick.
        float useRot[9], usePos[3];
        BlendRot(a.rot, relRot, share, useRot);
        for (int i = 0; i < 3; ++i)
            usePos[i] = a.pos[i] + (relPos[i] - a.pos[i]) * share;
        float worldRows[9];
        Mul3(useRot, hr, worldRows);
        float out[16];
        for (int r = 0; r < 3; ++r) {
            out[r * 4] = worldRows[r * 3] * scale;
            out[r * 4 + 1] = worldRows[r * 3 + 1] * scale;
            out[r * 4 + 2] = worldRows[r * 3 + 2] * scale;
            out[r * 4 + 3] = m[r * 4 + 3];
        }
        for (int i = 0; i < 3; ++i)
            out[12 + i] = hp[i] + hr[i] * usePos[0] + hr[3 + i] * usePos[1] + hr[6 + i] * usePos[2];
        out[15] = m[15];
        // Not while the writer watch is on: our own store would be the writer
        // it finds. And not when the source hook is doing it, so the report
        // says which one held.
        if (false && settings.armIk && !atSource && nowMs >= g_gunFinderUntilMs // never reached the screen
            && TryWrite(object + kOffAttachWorld, out, sizeof(out))
            && (!haveCurrent || TryWrite(object + kOffAttachCurrent, out, sizeof(out)))) {
            if (haveCurrent)
                ++s_wroteCurrent;
            std::memcpy(a.wrote, out, sizeof(out));
            a.wroteValid = true;
            ++s_written;
            float tr = 0.0f;
            for (int r = 0; r < 3; ++r)
                tr += Dot3(useRot + r * 3, relRot + r * 3);
            float cs = (tr - 1.0f) * 0.5f;
            cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
            const float fixDeg = std::acos(cs) * 57.2957795f;
            float dp[3];
            Sub3(usePos, relPos, dp);
            const float fixCm = Length3(dp) * 100.0f / upm;
            if (fixDeg > s_maxFixDeg)
                s_maxFixDeg = fixDeg;
            if (fixCm > s_maxFixCm)
                s_maxFixCm = fixCm;
        }
    }

    // The weapon, held in your hand as one rigid thing (2026-09-23).
    //
    // Holding every bone where it sat was too much: it froze the gun's own
    // working parts along with the recoil, and any decision about whether the
    // gun had really moved had to be made bone by bone, which is what the user
    // watched "start to separate". Worse, a gun steered by two hands is
    // legitimately somewhere new every frame, so it kept being taken up as a
    // new grip - "I frequently feel my gun regrab the position".
    //
    // So only ONE thing is held: the bone that sits in your hand, the grip.
    // Whatever rigid move puts that bone back where it belongs is then applied
    // to every bone of the weapon, which leaves the gun's shape and all of its
    // own animation - slide, pump, cylinder, magazine - completely untouched,
    // and takes out exactly what should not be there: the whole gun kicking,
    // and the whole gun lagging behind your hand.
    if (settings.gunLock) {
        DiscoverGunSkeletons(g_body, settings, hp, hr, upm);
        static float s_rel[kMaxGunJoints][9], s_relPos[kMaxGunJoints][3], s_rowLen[kMaxGunJoints][3];
        static float s_mat[kMaxGunJoints][16];
        static bool s_good[kMaxGunJoints];
        const bool twoHandNow = XrInput_GetTwoHand(nullptr);
        for (int s = 0; s < g_gunSkCount; ++s) {
            GunSkeleton& sk = g_gunSk[s];
            const int n = sk.count < kMaxGunJoints ? sk.count : kMaxGunJoints;
            if (n < 2)
                continue;

            // One read of the whole weapon, and how near your hand it is.
            float nearest = 1e9f;
            int nearestIdx = -1;
            for (int i = 0; i < n; ++i) {
                s_good[i] = false;
                float w[16];
                if (!TryRead(w, sk.joints + i * kJointStride + kOffJointWorldMatrix, sizeof(w))
                    || std::fabs(w[15] - 1.0f) > 1e-3f)
                    continue;
                float jrows[9];
                bool jok = true;
                for (int r = 0; r < 3 && jok; ++r) {
                    float v[3] = { w[r * 4], w[r * 4 + 1], w[r * 4 + 2] };
                    s_rowLen[i][r] = Length3(v);
                    jok = s_rowLen[i][r] > 0.01f && Normalise3(v);
                    std::memcpy(jrows + r * 3, v, sizeof(v));
                }
                if (!jok)
                    continue;
                Mul3(jrows, hrT, s_rel[i]);
                const float jd[3] = { w[12] - hp[0], w[13] - hp[1], w[14] - hp[2] };
                for (int r = 0; r < 3; ++r)
                    s_relPos[i][r] = Dot3(hr + r * 3, jd);
                const float away = Length3(jd);
                if (away < nearest) {
                    nearest = away;
                    nearestIdx = i;
                }
                std::memcpy(s_mat[i], w, sizeof(w));
                s_good[i] = true;
            }
            // Something on your back is not something in your hand. This is
            // what caught "the minigun backpack on my back ... being connected
            // to my hand": the search accepts anything within about a metre.
            if (nearestIdx < 0 || nearest > 25.0f) {
                sk.hold = false;
                continue;
            }

            if (!sk.hold && sk.haveLearned && !kicking && sk.learnedAnchor >= 0 && sk.learnedAnchor < n) {
                // STRAIGHT INTO THE HAND (2026-09-25, user: "the gun doesn't
                // arrive into the hand fast enough. The drift shouldn't happen at
                // all. When I press aim, the gun should already be in the hand.
                // This is similar to the hopping bug").
                //
                // Same root as the hopping, yes. Every time the gun comes up the
                // grip is worked out again from scratch, which means waiting for
                // the game's draw animation to finish putting it somewhere worth
                // measuring - and that wait IS the drift. Tightening the test for
                // when it has arrived, which is what the last build did, makes
                // the wait more correct and no shorter.
                //
                // But how a weapon sits in a hand is a property of the weapon.
                // It was true the last time it was raised and it is true now.
                // Once it has been learned there is nothing to wait for: put the
                // gun where it belongs on the first frame it is near the hand and
                // let the draw animation play out underneath, held.
                //
                // Measuring still happens, once, the first time each weapon is
                // raised - and that one time still waits for the grip to stop
                // moving, because a number learned wrongly would now be reapplied
                // instantly for ever after.
                sk.anchor = sk.learnedAnchor;
                std::memcpy(sk.holdRot[0], sk.learnedRot, sizeof(sk.learnedRot));
                std::memcpy(sk.holdPos[0], sk.learnedPos, sizeof(sk.learnedPos));
                sk.offSinceAll = 0;
                sk.settleFrames = 0;
                sk.hold = true;
                continue;
            }
            if (!sk.hold) {
                // Never read mid-swing: the game draws the gun from where your
                // hand was, so a reading taken then is the lag, not the grip.
                // And never read a gun that is not in your hand yet
                // (2026-09-23, user: "occasionally while aiming, the gun
                // didn't grab the right hand position, so I was left with a
                // floating gun that trailed me"). Caught halfway out of a
                // holster, the nearest bone can be a hand's breadth away, and
                // holding it there is exactly a gun that floats and follows.
                // A grip is AT the hand: twelve units, about an inch and a
                // half, or it waits.
                if (!handSteady || kicking || nearest > 12.0f) {
                    sk.settleFrames = 0;
                    continue;
                }
                // AND IT HAS TO HAVE STOPPED MOVING (2026-09-25, user: "when I
                // hit the aim button, the gun takes a little bit to drift into
                // my hand - if you catch it mid drift with your support hand,
                // the gun actually stops in place and won't continue to your
                // hand until you let go").
                //
                // That second sentence is the bug, and it is a lovely piece of
                // reporting. The drift is the game drawing the weapon, which is
                // its own animation and none of our business. Bringing the
                // support hand up steadies the gun hand, which satisfies
                // handSteady, and the nearest bone is already inside twelve
                // units even though the gun is still travelling - so the hold
                // engages mid-draw and pins the grip at whatever wrong offset it
                // had at that instant. It then sits there, exactly as told,
                // until letting go drops the hold and the draw finishes.
                //
                // Being near the hand is not the same as being in it. A grip
                // that has arrived stops moving relative to the hand; one still
                // being drawn does not. Six frames within one unit is about
                // sixty milliseconds and it cannot be satisfied by a gun in
                // flight, whatever the other hand is doing.
                {
                    float moved = 0.0f;
                    if (sk.settleFrames > 0) {
                        const float d3[3] = { s_relPos[nearestIdx][0] - sk.settleRel[0],
                            s_relPos[nearestIdx][1] - sk.settleRel[1],
                            s_relPos[nearestIdx][2] - sk.settleRel[2] };
                        moved = Length3(d3);
                    }
                    std::memcpy(sk.settleRel, s_relPos[nearestIdx], sizeof(sk.settleRel));
                    if (sk.settleFrames > 0 && moved < 1.0f)
                        ++sk.settleFrames;
                    else
                        sk.settleFrames = 1;
                    if (sk.settleFrames < 6)
                        continue;
                    sk.settleFrames = 0;
                }
                sk.anchor = nearestIdx;
                std::memcpy(sk.holdRot[0], s_rel[sk.anchor], sizeof(s_rel[0]));
                std::memcpy(sk.holdPos[0], s_relPos[sk.anchor], sizeof(s_relPos[0]));
                // Learned, so the next time this weapon comes up it goes
                // straight there.
                std::memcpy(sk.learnedRot, sk.holdRot[0], sizeof(sk.learnedRot));
                std::memcpy(sk.learnedPos, sk.holdPos[0], sizeof(sk.learnedPos));
                sk.learnedAnchor = sk.anchor;
                sk.haveLearned = true;
                sk.offSinceAll = 0;
                sk.hold = true;
                continue;
            }
            const int anchor = sk.anchor >= 0 && sk.anchor < n && s_good[sk.anchor] ? sk.anchor : -1;
            // A held place that is not in the hand is a floating gun, whatever
            // it was when it was read. Once it is held the gun sits exactly
            // where it is put, so nothing else would ever notice.
            if (anchor < 0 || Length3(sk.holdPos[0]) > 15.0f) {
                sk.hold = false;
                continue;
            }

            // How far the grip is from where it is held.
            float tr = 0.0f;
            for (int r = 0; r < 3; ++r)
                tr += Dot3(sk.holdRot[0] + r * 3, s_rel[anchor] + r * 3);
            float cs = (tr - 1.0f) * 0.5f;
            cs = cs > 1.0f ? 1.0f : (cs < -1.0f ? -1.0f : cs);
            const float offDeg = std::acos(cs) * 57.2957795f;
            float dv[3];
            Sub3(sk.holdPos[0], s_relPos[anchor], dv);
            const float offCm = Length3(dv) * 100.0f / upm;

            // The held place is LEARNED, not snapshotted (2026-09-23, user:
            // "the gun just didn't quite align with my hand so my laser was in
            // the right spot, gun was offset so using the sights of the gun
            // wasn't landing"). One frame's reading is not the grip: the game
            // draws the gun from where your hand was, so any single reading
            // carries whatever lag was in that moment, and holding it forever
            // holds the lag forever. The laser proves the point - it comes
            // from the game's own placement, so it lands where the gun should
            // be and the model does not.
            //
            // So it is averaged, slowly, and only over frames worth learning
            // from: hand still, not firing, not steering with two hands. On
            // those frames there is no lag to learn, so the average walks onto
            // the true grip and stays there, and a reading taken at a bad
            // moment is corrected within a second or so rather than kept.
            // Half a metre from the hand means the game has genuinely taken the
            // gun somewhere else - a holster, a swap - and there is no sense
            // holding a place it no longer has. But not while it is being
            // readied, for the same reason as below: that is a move we know
            // about and do not want to adopt.
            if (offCm > 50.0f && !(g_aimEdgeMs && nowMs - g_aimEdgeMs < 800)) {
                std::memcpy(sk.holdRot[0], s_rel[anchor], sizeof(s_rel[0]));
                std::memcpy(sk.holdPos[0], s_relPos[anchor], sizeof(s_relPos[0]));
                continue; // a draw or a holster: it IS where it is held
            }
            // NOT WHILE IT IS BEING READIED (2026-09-25, user: "the weapon is in
            // my hand prior to pressing aim... it shouldn't move, or play an
            // animation, it should stay in the exact same spot").
            //
            // This is the drift, and it was never the hold engaging. The held
            // grip is LEARNED, easing three per cent a frame toward wherever the
            // game is drawing the gun - which is right for correcting a reading
            // taken with lag in it, and wrong for the second after you press aim,
            // when the game is deliberately moving the weapon and every frame of
            // that gets averaged in. Three per cent a frame at a hundred frames a
            // second is about a third of a second to follow it across. That is
            // the drift, to the tenth.
            //
            // It is also, exactly, why catching it with the support hand freezes
            // it: twoHandNow is already on this line. Grabbing the gun stops the
            // learning, so the grip stops moving, and letting go starts it again.
            // A better description of the bug than I would have written.
            //
            // The weapon is in your hand before you press aim. There is nothing
            // to learn from the animation that readies it, so nothing is learned
            // until it has finished.
            const bool readying = g_aimEdgeMs && nowMs - g_aimEdgeMs < 800;
            if (handSteady && !kicking && !twoHandNow && !readying) {
                float eased[9];
                BlendRot(sk.holdRot[0], s_rel[anchor], 0.03f, eased);
                std::memcpy(sk.holdRot[0], eased, sizeof(eased));
                for (int k = 0; k < 3; ++k)
                    sk.holdPos[0][k] += (s_relPos[anchor][k] - sk.holdPos[0][k]) * 0.03f;
                // AND WHAT IS LEARNED HAS TO KEEP LEARNING (2026-09-25, user:
                // "sometimes the laser doesn't line up or gets skewed after
                // aiming one time, then you're kind of screwed").
                //
                // Mine, from two builds ago. Remembering the grip so the gun
                // goes straight into the hand was right, but the remembered copy
                // was written once and never again: the ease below refined
                // holdRot and holdPos while the learned copy kept the very first
                // reading, and every re-engage went back to it. One bad first
                // measurement and that was the grip for the rest of the session,
                // which is exactly "screwed".
                //
                // The refinement is the point of the ease. It belongs in both.
                std::memcpy(sk.learnedRot, sk.holdRot[0], sizeof(sk.learnedRot));
                std::memcpy(sk.learnedPos, sk.holdPos[0], sizeof(sk.learnedPos));
                sk.learnedAnchor = anchor;
                sk.haveLearned = true;
            }

            // Where the grip should be, and the one rigid move that puts it
            // there. The share lets the kick back in: 0 is a gun that does not
            // move in your hand at all.
            float useRot[9], usePos[3];
            BlendRot(sk.holdRot[0], s_rel[anchor], share, useRot);
            for (int k = 0; k < 3; ++k)
                usePos[k] = sk.holdPos[0][k] + (s_relPos[anchor][k] - sk.holdPos[0][k]) * share;
            float want[9], wantPos[3];
            Mul3(useRot, hr, want);
            for (int k = 0; k < 3; ++k)
                wantPos[k] = hp[k] + hr[k] * usePos[0] + hr[3 + k] * usePos[1] + hr[6 + k] * usePos[2];
            float isRows[9];
            Mul3(s_rel[anchor], hr, isRows);
            float isPos[3];
            for (int k = 0; k < 3; ++k)
                isPos[k] = hp[k] + hr[k] * s_relPos[anchor][0] + hr[3 + k] * s_relPos[anchor][1]
                    + hr[6 + k] * s_relPos[anchor][2];
            float isT[9], move[9];
            Transpose3(isRows, isT);
            Mul3(isT, want, move); // rows * move takes any bone from where it is to where it belongs

            for (int i = 0; i < n; ++i) {
                if (!s_good[i])
                    continue;
                float rows[9];
                for (int r = 0; r < 3; ++r) {
                    const float v[3] = { s_mat[i][r * 4] / s_rowLen[i][r], s_mat[i][r * 4 + 1] / s_rowLen[i][r],
                        s_mat[i][r * 4 + 2] / s_rowLen[i][r] };
                    std::memcpy(rows + r * 3, v, sizeof(v));
                }
                float newRows[9];
                Mul3(rows, move, newRows);
                const float d[3] = { s_mat[i][12] - isPos[0], s_mat[i][13] - isPos[1], s_mat[i][14] - isPos[2] };
                float pos[3];
                for (int k = 0; k < 3; ++k)
                    pos[k] = wantPos[k] + d[0] * move[k] + d[1] * move[3 + k] + d[2] * move[6 + k];
                float w[16];
                std::memcpy(w, s_mat[i], sizeof(w));
                for (int r = 0; r < 3; ++r) {
                    w[r * 4] = newRows[r * 3] * s_rowLen[i][r];
                    w[r * 4 + 1] = newRows[r * 3 + 1] * s_rowLen[i][r];
                    w[r * 4 + 2] = newRows[r * 3 + 2] * s_rowLen[i][r];
                }
                w[12] = pos[0];
                w[13] = pos[1];
                w[14] = pos[2];
                if (TryWrite(sk.joints + i * kJointStride + kOffJointWorldMatrix, w, sizeof(w))) {
                    TryWrite(sk.joints + i * kJointStride + kOffJointWorldPos, pos, sizeof(pos));
                    ++s_boneWrites;
                }
            }
            // And whatever hangs off the weapon's transform rather than off
            // its bones (2026-09-23, user: "the laser doesn't quite make it
            // back to where it should on the weapon.. so it may be being
            // missed"). The same rigid move, applied to the attach entries on
            // this hand. This is not the old attach lock that rolled the gun
            // sideways: that one learned a placement in one part of the frame
            // and enforced it in another, with the arm IK moving the wrist in
            // between. This is the move already worked out from the bones,
            // here, now, and it cannot disagree with them.
            for (int e = 0; e < kAttachEntries; ++e) {
                const AttachRest& at = g_attach[e];
                if (!at.onHand || at.object != sk.object)
                    continue;
                // AND THE THIRD ONE (2026-09-25, user: "the laser sight is ON,
                // it just isn't aligned to the gun right, so maybe we just
                // aren't aligning it on our end").
                //
                // Which is the answer, and it was in the attachment scan the
                // whole time. That scan reads positions at +0xD0 and +0x2D0,
                // and those are the translations of matrices at +0xA0 and
                // +0x2A0 - a 4x4's translation sits 0x30 into it. Only two
                // matrices were ever moved, +0xA0 and +0xE0, so +0x2A0 has been
                // left wherever the animation put it every single frame.
                //
                // It reads 17 cm from the grip at (-17.6, -2.7, 2.7) in the
                // hand's own axes, which is straight out along the barrel. That
                // is a muzzle, and a laser drawn from a muzzle that never came
                // with the gun is a laser pointing at where the gun used to be.
                //
                // Guarded the same way as the others: anything whose bottom
                // right is not 1.0 is not a transform and is left alone, so a
                // weapon that does not have this is unaffected.
                // EVERYTHING ON THE WEAPON, NOT A LIST (2026-09-27, user: "it
                // will be different for every weapon, so a scan is pointless.
                // Just make sure we are getting everything from the weapon").
                //
                // Exactly right, and it applies here as much as to the lean.
                // These three offsets are the three that happened to be found
                // while chasing a drifting laser on one gun, and the laser
                // still wobbles when the gun is waggled - so there is at least
                // one more, and the next weapon will have a different set.
                //
                // So the weapon is swept and everything transform-shaped moves
                // with the gun: a 4x4 with a 0,0,0,1 bottom row and three rows
                // of plausible length. Anything failing that is not a
                // transform and is left alone, which is the same guard the
                // named three already had, just applied to all of them.
                constexpr DWORD kSweepTo = 0x400;
                for (DWORD off = 0; off + 64 <= kSweepTo; off += 16) {
                    float t[16];
                    if (!TryRead(t, at.object + off, sizeof(t)) || std::fabs(t[15] - 1.0f) > 1e-3f)
                        continue;
                    if (t[3] != 0.0f || t[7] != 0.0f || t[11] != 0.0f)
                        continue; // a transform has no projective row
                    bool looksRight = true;
                    for (int r = 0; r < 3 && looksRight; ++r) {
                        const float len2 = t[r * 4] * t[r * 4] + t[r * 4 + 1] * t[r * 4 + 1]
                            + t[r * 4 + 2] * t[r * 4 + 2];
                        looksRight = len2 > 0.0001f && len2 < 400.0f;
                    }
                    if (!looksRight)
                        continue;
                    float out[16];
                    std::memcpy(out, t, sizeof(t));
                    for (int r = 0; r < 3; ++r) {
                        for (int k = 0; k < 3; ++k) {
                            out[r * 4 + k] = t[r * 4] * move[k] + t[r * 4 + 1] * move[3 + k]
                                + t[r * 4 + 2] * move[6 + k];
                        }
                    }
                    const float td[3] = { t[12] - isPos[0], t[13] - isPos[1], t[14] - isPos[2] };
                    for (int k = 0; k < 3; ++k)
                        out[12 + k] = wantPos[k] + td[0] * move[k] + td[1] * move[3 + k] + td[2] * move[6 + k];
                    // t was read a few lines up, before this write: that is
                    // what the address held when we came back to it.
                    if (s_lagHaveWrote && s_lagWatchAt == at.object + off) {
                        const float back[3]
                            = { t[12] - s_lagWrote[0], t[13] - s_lagWrote[1], t[14] - s_lagWrote[2] };
                        const float cm = Length3(back) * 100.0f / upm;
                        if (cm > s_lagMaxDriftCm)
                            s_lagMaxDriftCm = cm;
                    }
                    TryWrite(at.object + off, out, sizeof(out));
                    // THE MUZZLE IF IT IS THERE (2026-09-28). The first run
                    // latched onto weapon+1D0, which drifted by up to 69
                    // metres between frames with a still hand - so whatever
                    // lives there is not a weapon transform we own, it is a
                    // slot the game rewrites with something else entirely.
                    // +0x2A0 is the one the laser was traced to, so watch
                    // that when the weapon has it.
                    constexpr DWORD kMuzzleOff = 0x2A0;
                    const bool preferThis = (off == kMuzzleOff)
                        || (!s_lagWatchAt && s_lagWatchPoor);
                    if (off == kMuzzleOff)
                        s_lagWatchPoor = false;
                    if (preferThis || s_lagWatchAt == at.object + off) {
                        if (s_lagWatchAt != at.object + off)
                            s_lagHaveWrote = false;
                        s_lagWatchAt = at.object + off;
                    }
                    if (s_lagWatchAt == at.object + off) {
                        s_lagWatchAt = at.object + off;
                        s_lagWrote[0] = out[12];
                        s_lagWrote[1] = out[13];
                        s_lagWrote[2] = out[14];
                        s_lagHaveWrote = true;
                    }
                }
            }
            if (offCm > s_boneMaxCm)
                s_boneMaxCm = offCm;
            if (offDeg > s_boneMaxDeg)
                s_boneMaxDeg = offDeg;
        }
    }


    // ONLY WHEN IT IS ACTUALLY DRIFTING (2026-09-29). This went out in
    // v0.5.0 printing once a second into every player's log while saying
    // 0.00 cm every time, which is our debugging left switched on. It is
    // worth keeping for the weapon whose transform really does get rewritten
    // under us, so it speaks when there is something to say.
    if (s_lagHaveWrote && s_lagMaxDriftCm > 1.0f && nowMs - s_lagSaidMs >= 1000) {
        s_lagSaidMs = nowMs;
        Log_Printf("ArmIk: laser lag - watching weapon+%X; your hand moved up to %.2f cm in one frame, and "
                   "what we wrote there had moved %.2f cm back by the next",
            static_cast<unsigned>(s_lagWatchAt ? reinterpret_cast<uintptr_t>(s_lagWatchAt) & 0xFFFu : 0u),
            s_lagMaxStepCm, s_lagMaxDriftCm);
        s_lagMaxStepCm = 0.0f;
        s_lagMaxDriftCm = 0.0f;
    }

    if (s_boneWrites && nowMs - s_toldMs >= 3000) {
        s_toldMs = nowMs;
        char which[192] = {};
        int atW = 0;
        for (int s = 0; s < g_gunSkCount && atW < static_cast<int>(sizeof(which)) - 40; ++s) {
            const char* whose = "a weapon";
            if (g_haveBody && g_gunSk[s].joints == g_body.joints)
                whose = "YOU";
            else if (g_havePartnerBody && g_gunSk[s].joints == g_partnerBody.joints)
                whose = "YOUR PARTNER";
            else if (g_gunSk[s].joints == g_otherJoints)
                whose = "somebody else";
            atW += sprintf_s(which + atW, sizeof(which) - atW, "%s%p (%d bones, %s)", s ? ", " : "",
                g_gunSk[s].joints, g_gunSk[s].count, whose);
        }
        Log_Printf("GunHold: holding %s - %u bone write(s) in the last 3 s, holding back up to %.1f deg and %.1f cm",
            g_gunSkCount ? which : "nothing", s_boneWrites, s_boneMaxDeg, s_boneMaxCm);
        s_boneWrites = 0;
        s_boneMaxDeg = 0.0f;
        s_boneMaxCm = 0.0f;
    }
    if (wantScale && nowMs - s_toldMs >= 3000) {
        s_toldMs = nowMs;
        Log_Printf("GunScale: the gun's own transform was scaled to %.2f on %ld write(s) in the last 3 s",
            g_gunScaleWanted, InterlockedExchange(&g_gunScaleWrites, 0));
    }
    if (atSource && nowMs - s_toldMs >= 3000) {
        const LONG n = InterlockedExchange(&g_sourceWrites, 0);
        if (n || s_frames) {
            s_toldMs = nowMs;
            Log_Printf("GunLock: %ld correction(s) in the last 3 s at a %.2f share, taking out up to %.1f deg and "
                       "%.1f cm; at the next frame %u of %u still held",
                n, share, g_gunFixDegMax, g_gunFixCmMax, s_held, s_frames);
            g_gunFixDegMax = 0.0f;
            g_gunFixCmMax = 0.0f;
            s_frames = s_held = 0;
        }
    }
    if (s_written && nowMs - s_toldMs >= 3000) {
        s_toldMs = nowMs;
        Log_Printf("GunKick: %u correction(s) in the last 3 s at a %.2f share (%u also on the current transform), "
                   "taking out up to %.1f deg and %.1f cm; the next frame still held %u of %u%s",
            s_written, share, s_wroteCurrent, s_maxFixDeg, s_maxFixCm, s_held, s_frames,
            s_frames && s_held * 2 < s_frames ? " - the game recomposes these after us, so the fix is overwritten" : "");
        s_frames = s_held = s_written = s_wroteCurrent = 0;
        s_maxFixDeg = s_maxFixCm = 0.0f;
    }
}

// THE MAGAZINE IS NOT OPTIONAL (2026-09-25, user: "occasionally the magazine
// bone doesn't freeze and the regular animation plays").
//
// It was written as the last line of the gun hold, which was the right place
// to write it - after every other bone has been placed, so nothing downstream
// puts it back - and the wrong place to DECIDE whether to write it. The hold
// gives up early on plenty of ordinary frames: with the gun lock unticked, with
// no scale asked for, during a scripted camera, before both arms are known. On
// any of those the last line never runs, the game's animation owns the
// magazine for that frame, and you see it move.
//
// So the two are separated. The hold may still give up whenever it likes; the
// magazine is written afterwards either way, and works out for itself where the
// hand is rather than borrowing numbers the hold happened to have.
// ---- How much of yourself you see (2026-09-26) -------------------------
//
// Three settings players asked for: the whole character, forearms and hands,
// or hands alone. All of it is one idea, and it is the idea the head hide has
// used since the beginning - a joint whose local scale is zero collapses, and
// everything hanging off it collapses with it.
//
// So the work is deciding what to KEEP. Everything on the path from a kept
// joint up to the root has to stay, or the kept joint has no parent to hang
// from; everything else goes. That is why hiding the body cannot simply
// collapse the spine: the arms are downstream of it.
//
// What is kept is therefore the arm chain and its descendants, and the spine
// above it is collapsed only where it has no kept descendant. In practice
// that leaves a torso scaled to nothing with two arms still on it.
//
// The seam is the honest unknown. RE5's character is one skinned mesh, so a
// vertex weighted partly to a collapsed bone and partly to a kept one will
// stretch between the two. The wrist is a natural break and should be clean;
// the elbow is mid-limb and may not be. That is why this ships as a setting
// to look at rather than as a promise.
// Now only a courier: the hiding itself happens where the mesh is drawn, in
// render/bone_palette.cpp. Everything this function used to do - writing a
// zero scale into the skeleton - took the camera down with the body and could
// not be undone, and is gone.
// ---- Hiding the body, late and by world matrix (2026-09-26) ------------
//
// The distinction that makes this work, and that every earlier attempt
// missed: a LOCAL SCALE propagates to children, a WORLD MATRIX write does
// not. Shrinking the chest's scale takes the head, the arms and the camera
// with it, which is exactly what happened. Flattening the chest's world
// matrix takes the chest and nothing else, because the children's world
// matrices were composed before it and are stored separately - which is the
// same reason the arm solve can drive your arms without dragging your spine.
//
// So nothing is scaled. The game builds the whole skeleton normally, the arm
// solve runs with an intact frame to work against, and only then - here, at
// the end of everything - the bones we do not want are flattened where they
// stand. The rotation goes to zero and the position is kept, so every vertex
// weighted to that bone collapses onto the bone and disappears into the body
// rather than flying to the world origin.
//
// Nothing needs restoring: the game recomposes world matrices from the
// animation every frame, so not writing them IS the restore. That was untrue
// of local scale, which is why the first attempt trapped the user.
//
// The head keeps its existing collapse. It works, the camera reads that
// joint, and there is no reason to move a working thing into a new mechanism
// on the same night.
int g_armSteps[2] = { 0, 0 };

// WHAT WE OVERWROTE (2026-09-26, user: "flipping back to Full body does not
// resolve the stutters, nor sheva getting cut in half").
//
// The claim that there was nothing to restore was wrong. It rested on the game
// recomposing every joint every frame, and it plainly does not - a character
// off to one side, animating at a distance, keeps whatever was last written
// into the joints the game did not touch. So NaN stays there until something
// happens to recompose it, and turning the setting off changes nothing because
// nothing writes those joints again.
//
// So the originals are kept and put back the moment hiding stops, or the body
// changes, or the mode goes to Full body.
constexpr int kMaxUndo = 200;
struct UndoOne {
    int index;
    float was[16];
};
UndoOne g_undo[kMaxUndo];
int g_undoCount = 0;
unsigned char* g_undoFor = nullptr;

void PutItAllBack()
{
    if (!g_undoFor || !g_undoCount)
        return;
    for (int i = 0; i < g_undoCount; ++i)
        TryWrite(g_undoFor + g_undo[i].index * kJointStride + kOffJointWorldMatrix, g_undo[i].was,
            sizeof(g_undo[i].was));
    Log_Printf("BodyView: put %d joint(s) back the way they were", g_undoCount);
    g_undoCount = 0;
    g_undoFor = nullptr;
}

// WHOSE BODY IS THIS (2026-09-26, the screenshot: Sheva cut off at the waist).
//
// g_body is whatever the body finder last looked at, and it looks at both
// characters. Hiding read it directly, so on any frame it happened to hold the
// partner, the partner got the treatment - her arms and head written out
// exactly the way ours are, which is what the screenshot shows. It never
// announced itself because from our own eyes the result looks correct.
//
// The player controller knows which character is ours. Nothing is written
// unless the body in hand is that one.


struct BodyHideSnap {
    int count;
    float pos[kMaxJoints][3];
    bool ok[kMaxJoints];
    bool keep[kMaxJoints];
    float cutUnits;
};

// THE POSITION FIELD IS NOT THE POSITION (2026-09-26, user: "isn't really
// taking the whole body, just taking a chunk out of the arm").
//
// That is the fingerprint of a bad match, not a bad keep set. The arm matched
// and the torso did not, and the two differ in one way: the arm's joints are
// driven by our own solve, which writes the world position field at +80h, and
// the torso's are left to the game, which evidently does not keep that field
// current for every joint. The world MATRIX at +50h is the one the renderer
// actually skins from, so it is the one the shader's table agrees with.
//
// Reading it is safe. It was WRITING it that black-screened the game.
bool JointDrawnPos(const unsigned char* joints, int index, float out[3])
{
    float w[16];
    if (index >= 0 && TryRead(w, joints + index * kJointStride + kOffJointWorldMatrix, sizeof(w))
        && std::fabs(w[15] - 1.0f) < 1e-3f && w[12] == w[12]) {
        out[0] = w[12];
        out[1] = w[13];
        out[2] = w[14];
        return true;
    }
    return JointWorldPos(joints, index, out);
}
BodyHideSnap g_hideSnap[2];
std::atomic<int> g_hideLive{ -1 };
int g_hideNext = 0;

void HideTheBody(const XrInputSettings& settings)
{
    const int mode = settings.bodyVisible;
    static int s_told = -1;
    if (mode <= 0) {
        PutItAllBack();
        g_hideLive.store(-1, std::memory_order_release);
        if (s_told != 0) {
            s_told = 0;
            Log_Printf("BodyView: the whole character is visible");
        }
        return;
    }
    if (!g_haveBody || !g_body.joints || g_body.jointCount <= 0) {
        PutItAllBack();
        g_hideLive.store(-1, std::memory_order_release);
        return;
    }
    // Ours, or nobody. See ThisIsOurs above - the partner was being hidden too.
    if (!ThisIsOurs(g_body.character)) {
        static unsigned long long s_grumbled = 0;
        const unsigned long long nowWho = GetTickCount64();
        if (nowWho - s_grumbled > 5000) {
            s_grumbled = nowWho;
            Log_Printf("BodyView: the body in hand is not ours - leaving it alone");
        }
        return;
    }
    // A different skeleton than the one we last wrote: put that one back first,
    // or it keeps whatever it was left holding.
    if (g_undoFor && g_undoFor != g_body.joints)
        PutItAllBack();
    const int n = g_body.jointCount < kMaxJoints ? g_body.jointCount : kMaxJoints;

    // What survives: each arm from the elbow (or the wrist) down, and
    // everything hanging off it. Worked out from the skeleton's own parent
    // links rather than from any list of bone numbers.
    static bool keep[kMaxJoints];
    std::memset(keep, 0, sizeof(keep));
    // Where the spine is, so the walk up the arm knows where to stop. Anything
    // on the head's own line of ancestors is torso, and stepping onto it would
    // keep the torso's entire subtree - which is the whole character.
    static bool spine[kMaxJoints];
    std::memset(spine, 0, sizeof(spine));
    {
        int at = g_body.head, guard = 0;
        while (at >= 0 && at < n && guard++ < 64) {
            spine[at] = true;
            unsigned char links[4] = {};
            if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                break;
            const int parent = links[1];
            at = (parent == at) ? -1 : parent;
        }
    }

    const int* wanted = settings.cutStepsUp[mode >= 2 ? 1 : 0];
    int steps[2];
    for (int s = 0; s < 2; ++s) {
        steps[s] = wanted[s];
        if (steps[s] < 0)
            steps[s] = 0;
        if (steps[s] > 8)
            steps[s] = 8;
    }
    int from[2] = { g_body.left.wrist, g_body.right.wrist };
    int reached[2] = { 0, 0 };
    // How long the chain IS, so the slider can be given a range where every
    // position does something (2026-09-26, user: "the arm and hand NaN joint
    // jumps did nothing at all"). It was going to eight on an arm with four
    // joints in it, so over half its travel landed on the same bone - and the
    // saved setting sat in that dead half, keeping the whole arm.
    for (int s = 0; s < 2; ++s) {
        int at = from[s], have = 0;
        while (at >= 0 && at < n && have < 16) {
            unsigned char links[4] = {};
            if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                break;
            const int parent = links[1];
            if (parent == at || parent < 0 || parent >= n || spine[parent])
                break;
            at = parent;
            ++have;
        }
        g_armSteps[s] = have;
        if (steps[s] > have)
            steps[s] = have;
    }
    for (int s = 0; s < 2; ++s) {
        int at = from[s];
        for (int step = 0; step < steps[s]; ++step) {
            if (at < 0 || at >= n)
                break;
            unsigned char links[4] = {};
            if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                break;
            const int parent = links[1];
            // Off the end of the arm, or about to step into the torso.
            if (parent == at || parent < 0 || parent >= n || spine[parent])
                break;
            at = parent;
            ++reached[s];
        }
        from[s] = at;
    }
    {
        static int s_saidFrom[2] = { -2, -2 }, s_saidSteps[2] = { -2, -2 };
        if (s_saidFrom[0] != from[0] || s_saidFrom[1] != from[1] || s_saidSteps[0] != steps[0]
            || s_saidSteps[1] != steps[1]) {
            s_saidFrom[0] = from[0];
            s_saidFrom[1] = from[1];
            s_saidSteps[0] = steps[0];
            s_saidSteps[1] = steps[1];
            Log_Printf("BodyView: left %d step(s) up reached bone %d of %d possible, right %d step(s) reached "
                       "bone %d of %d, out of %d bones",
                steps[0], from[0], reached[0], steps[1], from[1], reached[1], n);
        }
    }
    for (int s = 0; s < 2; ++s)
        if (from[s] >= 0 && from[s] < n)
            keep[from[s]] = true;
    for (int pass = 0; pass < 6; ++pass) {
        for (int i = 0; i < n; ++i) {
            if (keep[i])
                continue;
            unsigned char links[4] = {};
            if (!TryRead(links, g_body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
                continue;
            const int parent = links[1];
            if (parent >= 0 && parent < n && parent != i && keep[parent])
                keep[i] = true;
        }
    }

    // AND WHATEVER IS SITTING ON THEM (2026-09-26, user: "how arms worked
    // with NaN, the left hand worked great ... it was the right hand that was
    // too aggressive").
    //
    // That asymmetry is the clue. Left and right can only differ if the keep
    // set differs, and it is built by walking parent links down from a wrist -
    // so any bone that drives hand geometry without being a DESCENDANT of the
    // wrist is hidden. Rigs are full of those: twist and helper bones that
    // deform the forearm and hand while hanging off somewhere else entirely.
    // This one has a row of them. The gun hand carries more, which is why the
    // right lost more of itself than the left.
    //
    // So anything sitting on top of a kept bone is kept too, whatever the
    // hierarchy says about it. A hand's width is enough to catch a twist bone
    // at the wrist and nowhere near enough to reach the elbow from the hand.
    {
        const float upmNow = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
        // HOW FAR BACK THE CUT SITS (2026-09-26, user: "its almost like hand mode
        // just needs to move slightly farther back to give more wrist").
        //
        // This radius is what catches the twist and helper bones that deform the
        // hand without hanging off the wrist in the hierarchy, and it doubles as
        // the place the cut lands: reach further up the arm and more of the
        // forearm survives, which reads as more wrist.
        //
        // Hands reaches further than Arms on purpose. Arms is already cutting at
        // the elbow and does not want to creep up the upper arm; Hands cutting
        // exactly at the wrist joint leaves a glove with no cuff.
        //
        // 0.15 in Hands was too far back (2026-09-26, user: "the hand mode
        // from my last test was a little too far back, it needs to be closer
        // to the wrist"), so it is a setting now and the default came down.
        //
        // ("near" is a macro in the Windows headers, like "far".)
        float back = settings.bodyCutBack;
        if (!(back > 0.02f) || back > 0.5f)
            back = 0.11f;
        const float closeBy = back * upmNow;
        static float keptAt[kMaxJoints][3];
        int nAt = 0;
        for (int i = 0; i < n; ++i)
            if (keep[i] && JointDrawnPos(g_body.joints, i, keptAt[nAt]))
                ++nAt;
        for (int i = 0; i < n && nAt; ++i) {
            // Same rule as below: near something kept is not a reason to come
            // back unless it is part of the same limb. This is what let a hand
            // resting on a thigh pull the thigh back into view.
            if (keep[i] || spine[i])
                continue;
            float p[3];
            if (!JointDrawnPos(g_body.joints, i, p))
                continue;
            for (int k = 0; k < nAt; ++k) {
                const float dx = p[0] - keptAt[k][0], dy = p[1] - keptAt[k][1], dz = p[2] - keptAt[k][2];
                if (dx * dx + dy * dy + dz * dz < closeBy * closeBy) {
                    keep[i] = true;
                    break;
                }
            }
        }

        // THE FOREARM DOES NOT HANG OFF THE WRIST (2026-09-26, user: "the
        // slider is working but never shows the forearm, it just walks its way
        // on the left side to the shoulder", and "same with the hand slider,
        // they are both showing the exact same").
        //
        // One cause under both. The walk is landing exactly where it should -
        // the log has it reaching bones 80, 79, 77, 75 on the way up - but the
        // joints it passes through carry no visible mesh, so adding them
        // changes nothing. The forearm surface is skinned to twist and roll
        // bones that hang off the UPPER ARM, beside the elbow rather than
        // below it, and a walk up the parent chain never touches them. So you
        // get the hand, then the hand again, and then suddenly the whole arm
        // the moment the walk reaches the bone those twists belong to. Hands
        // and Arms look identical for the same reason: the joints between them
        // are invisible ones.
        //
        // Hierarchy cannot answer this, and it does not have to. The region
        // can: everything physically inside the stretch of arm being kept,
        // however the rig chose to parent it. The sphere is centred on the cut
        // joint and reaches as far as the wrist, which is the forearm exactly,
        // plus the cut margin.
        //
        // Two things are held out of it. The spine, so it can never swallow the
        // torso, and the cut joint own ancestors, so a short forearm on a
        // sphere of its own length cannot reach back up to the shoulder.
        // BOTH SIDES, AND NEITHER REACHING THE OTHER (2026-09-26, user: "yes
        // but is this mirroring to the other side?").
        //
        // It always did run for both - each side measures its own cut joint
        // against its own wrist, with nothing copied across - but the question
        // found a real hole. The ancestors were worked out per side, so the
        // left sphere knew to avoid the left shoulder and clavicle and nothing
        // about the right ones. A clavicle does not sit on the head line, so
        // the spine test does not catch it, and keeping one keeps that whole
        // arm and shoulder with it. Fold your arms and the left elbow reaches
        // the right clavicle easily.
        //
        // So the shoulders above BOTH cut joints are held out of BOTH spheres.
        static bool above[kMaxJoints];
        std::memset(above, 0, sizeof(above));
        for (int s = 0; s < 2; ++s) {
            int at = from[s], guard = 0;
            while (at >= 0 && at < n && guard++ < 64) {
                unsigned char links[4] = {};
                if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                    break;
                const int parent = links[1];
                if (parent == at || parent < 0 || parent >= n)
                    break;
                above[parent] = true;
                at = parent;
            }
        }
        // ON THE ARM, NOT MERELY NEAR IT (2026-09-26, user: "you can also still
        // put your hand over your legs as an example and reveal the hidden
        // limb").
        //
        // That is this pass doing it, and it is the old unfolding fault wearing
        // a new coat. A rescue decided purely by distance rescues whatever
        // happens to be close, so bringing a hand down to your thigh brings the
        // thigh back with it. Position can say how far along a limb something
        // is; it can never say which limb it belongs to.
        //
        // The hierarchy can, and this is the one question it answers well:
        // everything on an arm descends from that arm shoulder, twist and
        // helper bones included, and nothing on a leg ever does. So the rescue
        // is offered only to descendants of the two shoulders, and geometry
        // decides how far down each arm it reaches. Hierarchy for WHICH, the
        // sphere for HOW FAR - each doing the part it is actually good at.
        static bool onAnArm[kMaxJoints];
        std::memset(onAnArm, 0, sizeof(onAnArm));
        {
            for (int s = 0; s < 2; ++s) {
                const int sh = s == 0 ? g_body.left.shoulder : g_body.right.shoulder;
                if (sh >= 0 && sh < n)
                    onAnArm[sh] = true;
            }
            for (int pass = 0; pass < 8; ++pass)
                for (int i = 0; i < n; ++i) {
                    if (onAnArm[i])
                        continue;
                    unsigned char links[4] = {};
                    if (!TryRead(links, g_body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
                        continue;
                    const int parent = links[1];
                    if (parent >= 0 && parent < n && parent != i && onAnArm[parent])
                        onAnArm[i] = true;
                }
        }

        // FROM THE WRIST OUT, NOT FROM THE CUT (2026-09-26, user: "the arm
        // slider still didn't reveal the forearm like I needed").
        //
        // Centring on the cut joint was wrong. At two steps up the cut sits
        // mid forearm and the sphere is small, so the twist bones nearer the
        // elbow - which are the ones carrying the forearm surface - sat outside
        // it, and nothing appeared until the walk reached the bone they hang
        // off directly. Centre it on the WRIST and let the radius reach as far
        // as the cut instead: everything between the two is inside by
        // construction, so each step adds the piece of arm it looks like it
        // should.
        for (int s = 0; s < 2; ++s) {
            const int cut = from[s];
            const int tip = s == 0 ? g_body.left.wrist : g_body.right.wrist;
            if (cut < 0 || cut >= n || tip < 0 || tip >= n)
                continue;
            float cutAt[3], tipAt[3];
            if (!JointDrawnPos(g_body.joints, cut, cutAt) || !JointDrawnPos(g_body.joints, tip, tipAt))
                continue;
            const float dx = cutAt[0] - tipAt[0], dy = cutAt[1] - tipAt[1], dz = cutAt[2] - tipAt[2];
            const float span = std::sqrt(dx * dx + dy * dy + dz * dz) + closeBy;
            for (int i = 0; i < n; ++i) {
                if (keep[i] || spine[i] || above[i] || !onAnArm[i])
                    continue;
                float p[3];
                if (!JointDrawnPos(g_body.joints, i, p))
                    continue;
                const float ex = p[0] - tipAt[0], ey = p[1] - tipAt[1], ez = p[2] - tipAt[2];
                if (ex * ex + ey * ey + ez * ez < span * span)
                    keep[i] = true;
            }
        }
    }

    // WHAT THE CAMERA STANDS ON (2026-09-26, user: "I feel like any time the
    // body shrinks, the camera is gonna go with it though, cause we have it
    // mounted to the neck").
    //
    // Only the rotation is flattened and the position is kept, so a flattened
    // joint does not move and the separate position field at +80h is never
    // touched - but the camera may read a rotation as well as a place, and
    // there is no reason to find out the hard way. The head and its parent,
    // which HeadLock calls the neck, are left completely alone. They are
    // already invisible: the head collapse takes the head and everything
    // hanging off it, and the neck is buried in the collar.
    // ONE POINT, NOT EACH ITS OWN (2026-09-26, user: "I'm getting a thin line
    // still drawn - like a thin line at the legs, thick at the spine, and a
    // thin line for where my arms used to be").
    //
    // Because each bone was collapsed onto ITSELF, which leaves a stick figure
    // of the skeleton: the triangles between two collapsed bones still have
    // two distinct points to stretch between, so they draw as a line - thin
    // down a limb, thicker at the spine where more bones crowd together.
    //
    // Collapse every hidden bone to the SAME place and those triangles have no
    // area at all and draw nothing. The place is the root joint, which is deep
    // inside you, so the vertices that span a seam stretch inwards and stay
    // buried rather than reaching somewhere visible.
    // ...ONTO THE NEAREST KEPT BONE (2026-09-26, user: "now the arms stretch
    // to that point - same when you switch to just arms").
    //
    // One shared point was the worst possible choice and the screenshot showed
    // why: a vertex weighted partly to a hidden bone and partly to a kept one
    // has to land between them, so putting the hidden end at your hips drew a
    // cone from every seam all the way down your body.
    //
    // Collapse a hidden bone onto the nearest bone that is STAYING and the
    // problem disappears from both ends. The upper arm lands on the elbow, so
    // a seam vertex there lands on the elbow too and stretches nowhere. The
    // torso and the legs have no kept neighbour anywhere near, so they all
    // collapse onto the same distant one and their triangles keep zero area,
    // which is what makes them vanish rather than become a skeleton.
    // EACH SIDE TO ITS OWN BREAK POINT (2026-09-26, user: "for the arms it
    // worked great to tie it to nearest - the hands was the only case this was
    // messing up, and I really think rather than just tying it nearest, you
    // should just tie each side to their break point").
    //
    // Nearest-kept was right for Arms and I replaced it for both, which was an
    // over-correction: one broken case does not justify changing the mechanism
    // for the case that worked.
    //
    // What broke Hands was measuring rather than following the skeleton. With
    // only the wrists kept, a bone in the left arm could measure nearer to the
    // RIGHT wrist depending on how you were standing, and the torso split
    // between the two - so triangles bridging that divide drew a line from one
    // hand to the other.
    //
    // So an arm's bones go to that arm's break point, decided by walking up
    // the parents to a shoulder rather than by distance. It cannot get the
    // side wrong. Everything with no side to it - torso, legs - still goes to
    // whichever break point is nearest, which is what Arms was already doing
    // happily.
    static int side[kMaxJoints];
    for (int i = 0; i < n; ++i) {
        side[i] = -1;
        int at = i, guard = 0;
        while (at >= 0 && at < n && guard++ < 64) {
            if (at == g_body.left.shoulder) {
                side[i] = 0;
                break;
            }
            if (at == g_body.right.shoulder) {
                side[i] = 1;
                break;
            }
            unsigned char links[4] = {};
            if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                break;
            const int parent = links[1];
            at = (parent == at) ? -1 : parent;
        }
    }

    const int breakAt[2] = { mode >= 2 ? g_body.left.wrist : g_body.left.elbow,
        mode >= 2 ? g_body.right.wrist : g_body.right.elbow };
    float breakPos[2][3] = {};
    bool haveBreak[2] = {};
    for (int s = 0; s < 2; ++s)
        haveBreak[s] = breakAt[s] >= 0 && breakAt[s] < n && JointWorldPos(g_body.joints, breakAt[s], breakPos[s]);

    static float target[kMaxJoints][3];
    {
        static float keptPos[kMaxJoints][3];
        int nKept = 0;
        for (int i = 0; i < n; ++i) {
            if (!keep[i])
                continue;
            if (!JointWorldPos(g_body.joints, i, keptPos[nKept]))
                continue;
            ++nKept;
        }
        if (!nKept)
            return;
        // WHAT HANGS OFF IT (2026-09-26, user: "you can see it in the string
        // that its the glove texture stretching").
        //
        // That observation named the fault. A glove vertex is weighted partly
        // to the forearm, so it goes wherever the forearm goes - and a
        // glove-textured line running between your two hands means one arm's
        // forearm was collapsing onto the OTHER arm's wrist. Not the torso at
        // all, which is what I had been insisting on.
        //
        // It happened because the side was decided by walking up to a shoulder
        // and falling back to "nearest" when that walk came up empty, and
        // nearest can easily measure the wrong wrist depending on how you are
        // standing. Putting everything behind the head instead did not help:
        // two cones converging behind you project across the view as a line
        // between your hands, which is the same artifact wearing a disguise.
        //
        // So the question becomes the one that cannot be answered wrongly:
        // which kept bone HANGS OFF this one. A forearm's own wrist is its
        // descendant; the other arm's wrist never is, however you stand. Only
        // bones with no kept descendant at all - the legs, the head - fall back
        // to nearest, and none of them wears a glove.
        float behind[3] = {};
        bool haveBehind = false;
        if (mode >= 2) {
            float hp[3], hr[9];
            if (g_body.head >= 0 && g_body.head < n && JointWorldPos(g_body.joints, g_body.head, hp)
                && WorldRot(g_body.joints, g_body.head, hr)) {
                const float back = 2.0f * (g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f);
                for (int k = 0; k < 3; ++k)
                    behind[k] = hp[k] - hr[6 + k] * back;
                haveBehind = true;
            }
        }
        const auto descendsFrom = [&](int child, int ancestor) {
            int at = child, guard = 0;
            while (at >= 0 && at < n && guard++ < 32) {
                if (at == ancestor)
                    return true;
                unsigned char links[4] = {};
                if (!TryRead(links, g_body.joints + at * kJointStride + kOffJointLinks, sizeof(links)))
                    break;
                const int parent = links[1];
                at = (parent == at) ? -1 : parent;
            }
            return false;
        };
        for (int i = 0; i < n; ++i) {
            if (keep[i])
                continue;
            float p[3];
            const bool havePos = JointWorldPos(g_body.joints, i, p);
            int best = -1;
            float bestD = 1e18f;
            // First choice: a kept bone that hangs off this one.
            for (int j = 0; j < n; ++j) {
                if (!keep[j] || !descendsFrom(j, i))
                    continue;
                float q[3];
                if (!JointWorldPos(g_body.joints, j, q))
                    continue;
                const float dd = havePos ? (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1])
                            + (p[2] - q[2]) * (p[2] - q[2])
                                         : 0.0f;
                if (best < 0 || dd < bestD) {
                    bestD = dd;
                    best = j;
                }
            }
            // Nothing of yours hangs off this one - it is torso, or a leg.
            //
            // In Arms that is fine as nearest, because the kept bones are your
            // elbows and the bridge to them is short and lands inside you.
            // In Hands it is the whole problem: the two kept islands are a
            // metre out in front and far apart, so the torso splits between
            // them and the triangles across that divide draw the string.
            //
            // So in Hands they go behind your head instead. One point, so the
            // torso is degenerate rather than split, and past the near plane,
            // so the bridge from it to each arm is clipped rather than drawn.
            if (best < 0 && mode >= 2 && haveBehind) {
                std::memcpy(target[i], behind, sizeof(float) * 3);
                continue;
            }
            // Otherwise whichever kept bone is nearest, which is what the arms
            // were already doing happily.
            if (best < 0 && havePos) {
                for (int j = 0; j < n; ++j) {
                    if (!keep[j])
                        continue;
                    float q[3];
                    if (!JointWorldPos(g_body.joints, j, q))
                        continue;
                    const float dd = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1])
                        + (p[2] - q[2]) * (p[2] - q[2]);
                    if (best < 0 || dd < bestD) {
                        bestD = dd;
                        best = j;
                    }
                }
            }
            if (best < 0 || !JointWorldPos(g_body.joints, best, target[i]))
                std::memcpy(target[i], p, sizeof(float) * 3);
        }
    }

    // The head goes too, by this mechanism rather than the old one (user: "the
    // head keeps popping in"). That collapse writes a local scale the game
    // keeps putting back, and it has never been dependable - it is the reason
    // the head reappears at a running lead of 4. This writes the world matrix,
    // which the animation does not argue with, and it leaves the separate
    // position field at +80h untouched, which is where anything that needs to
    // know where your head IS should be reading from.
    // PUBLISHED, NOT WRITTEN (2026-09-26, user: "my screen actually went
    // completely black and my fps tanked").
    //
    // The log names the cause exactly: "the game took the camera - it was
    // -nan(ind) units from where the body puts the eye", with the frame rate
    // collapsing from sixteen to five right behind it. NaN written into a
    // joint world matrix does not stay in the mesh. Eighteen places in this
    // mod read those matrices, and the first one to read a hidden joint
    // carries NaN out into the eye, the frustum planes and the culling test -
    // at which point nothing is visible and the renderer is thrashing.
    //
    // NaN is still the right answer; the SKELETON is the wrong place to put
    // it. The keep set is worked out here, where the hierarchy is, and the NaN
    // goes in at the last possible moment, in the bone table on its way to the
    // shader. The GPU is then the only thing that ever sees it, and there is
    // no frame after this one for it to persist into.
    // AND THE HEAD'S OWN CHAIN IS NEVER TOUCHED.
    //
    // This is the whole of what went wrong last time. NaN in a joint's world
    // matrix is right for the mesh and poison for everything in this mod that
    // reads one, and the readers that matter all live on the same short chain:
    // the eye sits on the neck, the lean walks up from the head, and the run
    // lead is measured off the root. Leave those three alone and the NaN has
    // nowhere to get out.
    //
    // They cost nothing to keep. The head has its own collapse and is inside
    // the camera anyway, the neck is buried in the collar, and the root is a
    // point at your feet with no geometry on it.
    // TWO, NOT THREE (2026-09-26, user: "the torso is being missed on them
    // both").
    //
    // Three walked one joint too far up. Head, then neck, then the CHEST - and
    // the chest is the bone most of your upper torso is skinned to, so keeping
    // it keeps the torso, in both modes, which is exactly what was seen. The
    // eye and the lean need the head and the neck; nothing needs the chest.
    //
    // The root came out for the same reason. It was kept in case the running
    // lead wanted it, and the lead does not read the skeleton at all - it
    // measures the character transform. Index zero on this rig carries the
    // hips, so keeping it kept them.
    // AND THE NECK CARRIES THE COLLAR (2026-09-26, user: "there is a few
    // chunks of shoulder that are visible though. tiny tiny chunks").
    //
    // Those are the neck. It was being kept to protect the eye, which sits on
    // it - but the eye reads the separate world POSITION field at +80h, and
    // nothing here has ever written that. Only the matrix is touched, and the
    // lean, which is the one other thing reading it, already refuses a matrix
    // it cannot make sense of. So the neck can go, and with it the collar and
    // the tops of the shoulders it skins.
    //
    // The head stays. It has its own collapse and is inside the camera anyway.
    if (g_body.head >= 0 && g_body.head < n)
        keep[g_body.head] = true;

    // COLLAPSE THE JOINT, DO NOT POISON IT (2026-09-28, user: "arms look
    // good - i get a string between my wrists on just hand mode though ...
    // whatever we were doing here is what needs to happen").
    //
    // Here IS where it has to happen - the bone table is no use, its
    // matrices are inverse bind times world and their translations are three
    // thousand units from any joint. So the keep set, worked out here where
    // the hierarchy is, has to do the hiding here too.
    //
    // What changes is WHAT is written. NaN in the world matrix was the
    // mistake: row three of that matrix is the joint\x27s world position, the
    // camera reads it for the eye, the culling reads it for the frustum, and
    // the running lead is measured off the root - so NaN in a joint came
    // straight back out as a NaN camera.
    //
    // Zeroing the 3x3 and KEEPING the translation does the same job to the
    // mesh and none of the damage. Every vertex weighted to the bone lands
    // on the bone, which is a degenerate triangle and draws nothing, while
    // the position stays a real number that anything reading it can use.
    // That is the collapse that was working, and the string between the
    // wrists was Hands mode stretching geometry across the body to a shared
    // point. Hands mode is gone.
    int hidden = 0;
    g_undoCount = 0;
    g_undoFor = g_body.joints;
    for (int i = 0; i < n; ++i) {
        if (keep[i])
            continue;
        unsigned char* jp = g_body.joints + i * kJointStride + kOffJointWorldMatrix;
        float w[16];
        if (!TryRead(w, jp, sizeof(w)) || std::fabs(w[15] - 1.0f) > 1e-3f)
            continue;
        // Keep the last good version of anything written, so it can go back.
        // Only a finite one: overwriting the record with our own NaN would make
        // the restore meaningless.
        if (g_undoCount < kMaxUndo && w[12] == w[12] && w[0] == w[0]) {
            g_undo[g_undoCount].index = i;
            std::memcpy(g_undo[g_undoCount].was, w, sizeof(w));
            ++g_undoCount;
        }
        for (int r = 0; r < 3; ++r) {
            w[r * 4 + 0] = 0.0f;
            w[r * 4 + 1] = 0.0f;
            w[r * 4 + 2] = 0.0f;
        }
        // w[12..14] - the world position - is deliberately left exactly as
        // the game wrote it. That is the whole difference between this and
        // the NaN version, and it is why nothing downstream breaks.
        if (TryWrite(jp, w, sizeof(w)))
            ++hidden;
    }
    {
        BodyHideSnap& snap = g_hideSnap[g_hideNext];
        snap.count = n;
        // WHAT THE SLIDER ACTUALLY CONTROLS (2026-09-26, user: "the slider
        // just didn't work").
        //
        // It did not, and could not. It was a radius for rescuing helper bones
        // in the hierarchy pass, this rig has none where it was looking, and
        // the log showed 90 of 130 hidden at every setting from 6 cm to 20.
        // Worse, a cut made by choosing joints can only ever land ON a joint -
        // wrist or elbow, with nothing in between.
        //
        // In the bone table it can be continuous, because the decision is made
        // per table entry rather than per joint: an entry is kept if it lies
        // within this distance of a joint that is being kept, whatever its
        // nearest joint happens to be. Slide it up and the cut walks smoothly
        // back up the forearm.
        {
            const float upmNow = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
            float backM = settings.bodyCutBack;
            if (!(backM > 0.02f) || backM > 0.5f)
                backM = 0.11f;
            snap.cutUnits = backM * upmNow;
        }
        for (int i = 0; i < n; ++i) {
            snap.keep[i] = keep[i];
            snap.ok[i] = JointDrawnPos(g_body.joints, i, snap.pos[i]);
        }
        g_hideLive.store(g_hideNext, std::memory_order_release);
        g_hideNext ^= 1;
    }
    (void)target;

    static int s_toldCut = -1;
    const int cutNow = static_cast<int>(settings.bodyCutBack * 100.0f + 0.5f);
    if (s_told != mode || s_toldCut != cutNow) {
        s_told = mode;
        s_toldCut = cutNow;
        Log_Printf("BodyView: %s, snapped at %d and %d, cut %d cm back - %d of %d joint(s) hidden",
            mode >= 2 ? "hands only" : "forearms and hands", from[0], from[1], cutNow, hidden, n);
    }
}


bool ArmIk_ArmSteps(int out[2])
{
    if (g_armSteps[0] <= 0 && g_armSteps[1] <= 0)
        return false;
    out[0] = g_armSteps[0];
    out[1] = g_armSteps[1];
    return true;
}

bool ArmIk_BodyHide(ArmIkBodyHide& out)
{
    const int live = g_hideLive.load(std::memory_order_acquire);
    if (live < 0 || live > 1)
        return false;
    const BodyHideSnap& snap = g_hideSnap[live];
    if (snap.count <= 0)
        return false;
    out.count = snap.count;
    out.pos = snap.pos;
    out.ok = snap.ok;
    out.keep = snap.keep;
    out.cutUnits = snap.cutUnits;
    return true;
}

void ArmIk_SteadyGun()
{
    SteadyGunNow();
    HideTheBody(XrInput_GetSettings());
    MoveTheMagazine(XrInput_GetSettings());
}

bool ArmIk_GetBarrelInController(float out[3])
{
    const unsigned long long when = g_barrelMs;
    if (!when || GetTickCount64() - when > 500)
        return false;
    std::memcpy(out, g_barrelCtrl, sizeof(g_barrelCtrl));
    return true;
}

bool ArmIk_GetUnitsPerMetre(int hand, float* out)
{
    if (hand < 0 || hand > 1 || !out)
        return false;
    const float upm = g_calUnitsPerMetre[kPlayerSlot + hand];
    if (upm > 1.0f) {
        *out = upm;
        return true;
    }
    // The same fallback the solve itself uses when nobody has calibrated: the
    // character's own resting arm against an assumed human one. Without this the
    // partner refuses the frame a second time, for "they have not calibrated, so
    // their reach has no scale", even once a shoulder is arriving.
    const float rest = g_restReach[kPlayerSlot + hand];
    if (!(rest > 1.0f))
        return false;
    *out = rest / kArmMetres;
    return true;
}

bool ArmIk_IsCalibrated()
{
    return g_tiesCalibrated[0] && g_tiesCalibrated[1];
}

void ArmIk_SolvePartnerNow()
{
    ArmIkSolveLock lock;
    SolvePartner();
}

// Where the T-pose put your shoulder, relative to your head, in metres. Sent to
// a co-op partner so their machine can work out hand-relative-to-shoulder,
// which is the one measurement that means the same thing on two people of
// different sizes. See net/ik_sync.h.
bool ArmIk_GetShoulderFromHead(int hand, float out[3])
{
    if (hand < 0 || hand > 1 || !out)
        return false;
    if (g_haveShoulderFromHead[hand]) {
        std::memcpy(out, g_shoulderFromHead[hand], sizeof(float) * 3);
        return true;
    }
    // Assume one until it has been measured (2026-09-24, user: "my partner was
    // the first in VR and he saw the animation trying to play").
    //
    // That is not a race and it is not the skeletons: it is this. Calibrating is
    // a button in the mod menu, and until the SECOND person presses it their
    // packets carry hands with no shoulder and no scale, so the first person's
    // solve declines every single frame and they watch the plain animation. His
    // log says it exactly - "one of their hands arrived without a shoulder"
    // from 21:51:36 until 21:52:20, and the last one is the same second the
    // other player calibrated. Forty-four seconds of nothing, and whoever is in
    // VR first always waits the longest, which is why it has always been them.
    //
    // The local solve has never needed the button: uncalibrated it measures from
    // the eye with an assumed arm. There is no reason the packet should be
    // fussier than the solve behind it. So send ordinary adult proportions -
    // shoulders 0.38 m apart and 0.30 m below the headset, which is within a few
    // centimetres of what both of them actually measured - and let calibrating
    // sharpen it rather than switch it on.
    //
    // The far end only ever uses hand MINUS shoulder, as a direction and a
    // length, so an assumed shoulder costs a little accuracy in where the elbow
    // sits. Nothing costs more than not being driven at all.
    out[0] = (hand == 1 ? 1.0f : -1.0f) * 0.19f;
    out[1] = -0.30f;
    out[2] = 0.0f;
#if RE5VR_DIAGNOSTICS
    static bool s_toldAssumed[2] = {};
    if (!s_toldAssumed[hand]) {
        s_toldAssumed[hand] = true;
        Log_Printf("IkSync: sending an assumed %s shoulder until you calibrate, so your partner can drive "
                   "your arm now rather than when you get round to it",
            hand == 1 ? "right" : "left");
    }
#endif
    return true;
}

// Drive the partner's arms from whatever last arrived for them. Called once a
// frame; does nothing at all until a packet has been seen recently, so an
// unmodded partner simply keeps the animation they always had.
unsigned long long g_partnerSolvedMs = 0;

void SolvePartner()
{
    g_partnerSolvedMs = GetTickCount64();
#if RE5VR_DIAGNOSTICS
    // Say something even when nothing happens (2026-09-24). Every refusal in
    // here returns before the report at the bottom, so a partner who is never
    // driven produces no line at all - and three sessions were spent reading
    // silence as "this code is not running" when it was running and refusing.
    // Whatever the reason, it gets said.
    {
        static unsigned long long s_toldQuiet = 0;
        static unsigned long long s_lastRanMs = 0;
        const unsigned long long nowQuiet = GetTickCount64();
        if (g_partnerRan)
            s_lastRanMs = nowQuiet;
        if (nowQuiet - s_lastRanMs > 2000 && nowQuiet - s_toldQuiet >= 2000) {
            s_toldQuiet = nowQuiet;
            Log_Printf("ArmIk: their arms have not been driven for two seconds - %s", g_partnerWhy);
        }
    }
#endif
    if (!g_havePartnerBody || GetTickCount64() - g_partnerBodyMs > 500) {
        g_partnerWhy = "their body has not been seen for half a second";
        ++g_partnerSkipped;
        return;
    }
    IkSyncHands hands;
    if (!IkSync_GetPartnerHands(hands)) {
        g_partnerWhy = "no packet from them in the last half second";
        ++g_partnerSkipped;
        return;
    }
    if (!hands.haveHand[0] && !hands.haveHand[1]) {
        g_partnerWhy = "their packets carry no hands";
        ++g_partnerSkipped;
        return;
    }
    // COUNTING THE CLOCK (2026-09-24). This asks whether g_poseQuatMs changed to
    // decide whether the solve wrote anything, and that field is set from
    // GetTickCount64, which moves in steps of 15.6 ms. So two writes inside one
    // tick read as one write and a skip, and the count saturates at 64 a second
    // no matter how often the arm is actually posed.
    //
    // It is not a small error. The partner's log reads "driven on 64 frames" or
    // "driven on 0", never anything else, across the entire session - 64 being
    // exactly 1000/15.625. The drive rates worked out from it, and reported to
    // the user as 13 and 27 and 43 per cent, were measuring the Windows timer.
    // A counter bumped in the writer says what actually happened.
    const unsigned long long wroteBefore = g_poseQuatMs[kPartnerBody];
    const unsigned wroteCountBefore = g_pwWritten;
    g_partnerWhy = "it ran";
    RunSolve(g_partnerBody, &hands);
    if (g_poseQuatMs[kPartnerBody] != wroteBefore || g_pwWritten != wroteCountBefore) {
        ++g_partnerRan;
#if RE5VR_DIAGNOSTICS
        // Measured ONLY on frames we actually posed, and reported at its worst
        // (2026-09-23). The previous version of this took a reading every two
        // seconds regardless, and with the arm being posed on 2 frames in 120
        // it was almost always measuring the animation - so it reported a
        // perfect 53.2 while the user, stepping through his video frame by
        // frame, could plainly see stretched arms. A measurement that samples
        // the frames we did not touch is not evidence about the frames we did.
        {
            static float s_worstArm = 0.0f;
            static float s_worstUpper = 0.0f, s_worstFore = 0.0f;
            static unsigned s_posed = 0;
            static unsigned long long s_toldPosed = 0;
            float sP[3], eP[3], wP[3];
            if (JointWorldPos(g_partnerBody.joints, g_partnerBody.right.shoulder, sP)
                && JointWorldPos(g_partnerBody.joints, g_partnerBody.right.elbow, eP)
                && JointWorldPos(g_partnerBody.joints, g_partnerBody.right.wrist, wP)) {
                float uvP[3], fvP[3];
                Sub3(eP, sP, uvP);
                Sub3(wP, eP, fvP);
                const float upper = Length3(uvP), fore = Length3(fvP);
                ++s_posed;
                if (upper + fore > s_worstArm) {
                    s_worstArm = upper + fore;
                    s_worstUpper = upper;
                    s_worstFore = fore;
                }
            }
            const unsigned long long nowPosed = GetTickCount64();
            if (s_posed && nowPosed - s_toldPosed >= 2000) {
                s_toldPosed = nowPosed;
                Log_Printf("IkSync: on the %u frame(s) we actually posed their arm, the LONGEST it came out was "
                           "%.1f units - upper %.1f, forearm %.1f - against a resting %.1f",
                    s_posed, s_worstArm, s_worstUpper, s_worstFore, g_restReach[0]);
                s_worstArm = s_worstUpper = s_worstFore = 0.0f;
                s_posed = 0;
            }
        }
#endif
    }
    else {
        ++g_partnerSkipped;
        if (g_partnerWhy == "it ran")
            g_partnerWhy = "the solve ran but wrote no pose";
#if RE5VR_DIAGNOSTICS
        {
            static unsigned long long s_toldStage = 0;
            const unsigned long long nowStage = GetTickCount64();
            if (nowStage - s_toldStage >= 2000) {
                s_toldStage = nowStage;
                Log_Printf("ArmIk: the partner's solve reached the writer with %u joint(s) in the last stretch, "
                           "refused %u for not rebuilding, wrote %u; %u crossed write(s), %u refused as "
                           "yours and %u as theirs, %u solve(s) queued behind another thread",
                    g_pwOffered, g_pwRebuild, g_pwWritten, g_crossStamps, g_crossRefused[kPlayerBody],
                    g_crossRefused[kPartnerBody], g_solveWaits);
                g_pwOffered = g_pwRebuild = g_pwWritten = 0;
                g_crossStamps = 0;
                g_crossRefused[kPlayerBody] = g_crossRefused[kPartnerBody] = 0;
                g_solveWaits = 0;
            }
        }
#endif
    }
    {
        static unsigned long long s_saidMs = 0;
        const unsigned long long nowSay = GetTickCount64();
        if (nowSay - s_saidMs >= 1000) {
            s_saidMs = nowSay;
            if (g_partnerSkipped)
                Log_Printf("ArmIk: partner arms driven on %u frames, skipped on %u - last reason: %s",
                    g_partnerRan, g_partnerSkipped, g_partnerWhy);
            g_partnerRan = g_partnerSkipped = 0;
        }
    }
    g_solvingBody = kPlayerBody; // never leave it pointed at somebody else
    // And no skeleton at all between solves, so a write arriving from anywhere
    // other than underneath a RunSolve is refused rather than filed against
    // whoever happened to be solved last.
    g_solvingJoints = nullptr;
}

void ArmIk_GetPartnerStatus(ArmIkPartnerStatus& out)
{
    out = ArmIkPartnerStatus{};
    const unsigned long long now = GetTickCount64();
    out.haveBody = g_havePartnerBody && now - g_partnerBodyMs < 500;
    out.armsFound = out.haveBody && g_partnerBody.left.valid && g_partnerBody.right.valid;
    IkSyncHands hands;
    if (IkSync_GetPartnerHands(hands)) {
        out.handsFresh = hands.haveHand[0] || hands.haveHand[1];
        out.haveScale = hands.unitsPerMetre[0] > 1.0f || hands.unitsPerMetre[1] > 1.0f;
    }
    out.tied = g_haveWristOffset[kPartnerSlot] || g_haveWristOffset[kPartnerSlot + 1];
    out.solving = g_poseQuatMs[kPartnerBody] && now - g_poseQuatMs[kPartnerBody] < 250;
}

void ArmIk_SetPartnerBody(const ArmIkBody& body)
{
    ArmIkSolveLock lock;
    if (!body.joints || !body.character || body.jointCount < 8) {
        g_havePartnerBody = false;
        return;
    }
    // The partner is not you (2026-09-24). This is the door the whole problem
    // came through, and it was standing open.
    //
    // The refusals named it exactly: on his machine his own skeleton is at
    // 09E4C2D0 and yours at 09E0C330, and a pose worked out for 09E0C330 was
    // arriving at a joint in 09E4C2D0 - both read from the PARTNER slot. So
    // the partner slot had been handed his own bones. Your hands then drove
    // his character by design rather than by accident, which is why every
    // guess about overlapping ranges and proximity came back clean.
    //
    // Whoever decides who the partner is can be wrong - a stale player latch
    // through a level load is enough - so it is checked here instead, at the
    // one place a partner can be published, against the body we are actually
    // driving as our own. Nothing upstream has to be trusted.
    if (g_haveBody && (body.joints == g_body.joints || body.character == g_body.character)) {
        static unsigned long long s_toldMe = 0;
        const unsigned long long nowMe = GetTickCount64();
        if (nowMe - s_toldMe >= 2000) {
            s_toldMe = nowMe;
            Log_Printf("ArmIk: refused to take %p as the partner - that is the body we are driving as our own",
                body.joints);
        }
        g_havePartnerBody = false;
        return;
    }
    // Same rule as the player's, for the same reason (2026-09-20). The tie
    // between their hand and their skeleton's rest pose is made once, on first
    // sight - so if the character changes underneath it, the tie belongs to a
    // body that is no longer there. This is not hypothetical: a partner's
    // character pointer was seen changing five times in a quarter second while
    // their game loaded.
    if (body.character != g_partnerBody.character) {
        for (int h = 0; h < 2; ++h) {
            const int slot = kPartnerSlot + h;
            g_tiesCalibrated[slot] = false;
            g_haveWristOffset[slot] = false;
            g_haveArmOffset[slot] = false;
            g_haveCtrlRef[slot] = false;
            g_haveSocketTie[slot] = false;
            g_haveCtrl[slot] = false;
            g_socket[slot] = -1;
            g_foreAxis[slot] = -1;
            g_foreSign[slot] = 1.0f;
        }
        std::memset(g_poseQuatValid[kPartnerBody], 0, sizeof(g_poseQuatValid[kPartnerBody]));
        g_poseQuatMs[kPartnerBody] = 0;
    }

    g_partnerBody = body;
    g_havePartnerBody = true;
    g_partnerBodyMs = GetTickCount64();
}

void ArmIk_NoteOtherSkeleton(unsigned char* joints, int jointCount)
{
    if (!joints || jointCount < 8)
        return;
    g_otherJoints = joints;
    g_otherCount = jointCount < kMaxJoints ? jointCount : kMaxJoints;
    g_otherMs = GetTickCount64();
}

void ArmIk_NoteHandReader(void* source, void* object)
{
    // exe+19CB74 is a routine every transform in the scene passes through -
    // 25,000 calls a second, a different object each time. Only the ones
    // reading one of OUR wrists mean anything, and the watchpoint report says
    // ecx is the address being read, so that is the filter (2026-09-17).
    if (!g_haveBody || !g_body.joints || !g_body.left.valid || !g_body.right.valid)
        return;
    const unsigned char* wrists[2]
        = { g_body.joints + g_body.left.wrist * kJointStride + kOffJointWorldMatrix,
              g_body.joints + g_body.right.wrist * kJointStride + kOffJointWorldMatrix };
    if (source != wrists[0] && source != wrists[1])
        return;
    g_handReader = static_cast<unsigned char*>(object);
    g_handReaderRight = source == wrists[1];
    ++g_handReaderHits;
}

// ---- Leaning the man, not the camera (2026-09-23) -------------------------
// Lean out from behind a wall and the view already moves, because your head
// moved in the room. The character does not, so you are looking round a
// corner that his shoulder is still behind. This turns the upper body about
// the base of the spine until his head arrives where yours is.
//
// It is a rotation rather than a shove, because that is what a spine does:
// the head travels on an arc, so leaning out also dips you a little, and
// crouching forward tips him forward. One rotation about one pivot, applied
// to everything hanging off it, which keeps the whole torso rigid and
// correct - the cheap version of a spine chain, and the one that tells us
// whether the game's idea of where he can be hit moves with it.
//
// The arms are left out of it deliberately. Your hands are already where your
// controllers are and they should stay there while your body moves under
// them, which is also what stops the arm solve and this fighting each other
// frame by frame. The collarbones still turn, so the shoulders go with the
// chest.
// One joint, turned by M about pivotPos: the world matrix and the world
// position together. Lifted out of the loop below so the gun can be turned by
// exactly the same arithmetic instead of a second copy of it.
// One 4x4, turned about the lean pivot. Anything whose bottom right is not 1.0
// is not a transform and is left alone.
bool LeanOneMatrix(unsigned char* at, const float M[9], const float pivotPos[3], float outPos[3])
{
    float w[16];
    if (!TryRead(w, at, sizeof(w)) || std::fabs(w[15] - 1.0f) > 1e-3f)
        return false;
    float rows[9], len[3];
    bool ok = true;
    for (int r = 0; r < 3 && ok; ++r) {
        float e[3] = { w[r * 4], w[r * 4 + 1], w[r * 4 + 2] };
        len[r] = Length3(e);
        ok = len[r] > 0.01f && Normalise3(e);
        std::memcpy(rows + r * 3, e, sizeof(e));
    }
    if (!ok)
        return false;
    float turned[9];
    Mul3(rows, M, turned);
    const float d[3] = { w[12] - pivotPos[0], w[13] - pivotPos[1], w[14] - pivotPos[2] };
    float pos[3];
    for (int k = 0; k < 3; ++k)
        pos[k] = pivotPos[k] + d[0] * M[k] + d[1] * M[3 + k] + d[2] * M[6 + k];
    for (int r = 0; r < 3; ++r) {
        w[r * 4] = turned[r * 3] * len[r];
        w[r * 4 + 1] = turned[r * 3 + 1] * len[r];
        w[r * 4 + 2] = turned[r * 3 + 2] * len[r];
    }
    w[12] = pos[0];
    w[13] = pos[1];
    w[14] = pos[2];
    if (!TryWrite(at, w, sizeof(w)))
        return false;
    if (outPos)
        std::memcpy(outPos, pos, sizeof(pos));
    return true;
}

bool LeanOneJoint(unsigned char* jp, const float M[9], const float pivotPos[3])
{
    float pos[3];
    if (!LeanOneMatrix(jp + kOffJointWorldMatrix, M, pivotPos, pos))
        return false;
    TryWrite(jp + kOffJointWorldPos, pos, sizeof(pos));
    return true;
}

void ArmIk_LeanSpine()
{
    const XrInputSettings settings = XrInput_GetSettings();
    if (!settings.spineLean || !settings.armIk || !g_haveBody || !g_body.joints || g_body.head < 0)
        return;
    float lean[3];
    if (!StereoTest_GetLeanWorld(lean))
        return;
    const float amount = settings.spineLeanAmount < 0.0f ? 0.0f : settings.spineLeanAmount;
    for (int k = 0; k < 3; ++k)
        lean[k] *= amount;
    const float upm = g_solvedUpm > 1.0f ? g_solvedUpm : 85.8f;
    if (Length3(lean) * 100.0f / upm < 0.5f)
        return; // half a centimetre is standing still

    const int count = g_body.jointCount < kMaxJoints ? g_body.jointCount : kMaxJoints;
    if (count < 8)
        return;
    static unsigned char s_parent[kMaxJoints];
    for (int i = 0; i < count; ++i) {
        unsigned char links[4] = {};
        s_parent[i] = 0xFF;
        if (TryRead(links, g_body.joints + i * kJointStride + kOffJointLinks, sizeof(links)))
            s_parent[i] = links[1];
    }

    // Up the chain from the head. The pivot is the joint one above the root:
    // the base of the spine, which is what a person bends at. The legs hang
    // off the root below it, so they are left standing where they are without
    // anything having to know what a leg is.
    int chain[kMaxJoints];
    int chainLen = 0;
    for (int j = g_body.head; j >= 0 && j < count && chainLen < count; ) {
        chain[chainLen++] = j;
        const int up = s_parent[j];
        if (up == 0xFF || up == j || up >= count)
            break;
        j = up;
    }
    if (chainLen < 4)
        return;
    const int pivot = chain[chainLen - 2];

    // Everything above the pivot, minus each arm from the shoulder down.
    static bool s_turn[kMaxJoints];
    for (int i = 0; i < count; ++i) {
        s_turn[i] = false;
        int j = i, guard = 0;
        while (j >= 0 && j < count && guard++ < count) {
            if (j == pivot) {
                s_turn[i] = true;
                break;
            }
            const int up = s_parent[j];
            if (up == 0xFF || up == j || up >= count)
                break;
            j = up;
        }
    }
    // THE ARMS COME TOO (2026-09-24, user: "it needs fine tuned to not do any
    // weird curves").
    //
    // This used to walk both arms and take them back out of the rotation, so
    // the chest, neck and head tipped over and the arms stayed exactly where
    // they were. Nothing in a skeleton holds a shoulder onto a chest, so the
    // joint simply came apart and the mesh stretched across the gap. That is
    // the weird curve: not the spine bending oddly, the shoulders being left
    // behind by it.
    //
    // The reason they were excluded was to keep the lean from dragging the
    // hands off the controllers, and that reason does not survive contact with
    // the order things run in. This is a WORLD MATRIX pass and the arm solve
    // drives LOCAL rotations that are composed afterwards, so the hands are put
    // back where the controllers say regardless. Leaving the arms out bought
    // nothing and cost the shoulders.
    //
    // So the whole upper body turns as one rigid piece about the base of the
    // spine, which is deliberately the least interesting thing it could do.
    // Spreading the bend over several spine joints is what produces a banana,
    // and a person leaning to look round a corner really does hinge low and
    // keep their back straight.

    float pivotPos[3], headPos[3];
    if (!JointWorldPos(g_body.joints, pivot, pivotPos) || !JointWorldPos(g_body.joints, g_body.head, headPos))
        return;
    float v[3], want[3];
    Sub3(headPos, pivotPos, v);
    for (int k = 0; k < 3; ++k)
        want[k] = v[k] + lean[k];
    float vn[3] = { v[0], v[1], v[2] }, wn[3] = { want[0], want[1], want[2] };
    if (!Normalise3(vn) || !Normalise3(wn))
        return;
    float axis[3] = { vn[1] * wn[2] - vn[2] * wn[1], vn[2] * wn[0] - vn[0] * wn[2], vn[0] * wn[1] - vn[1] * wn[0] };
    const float sn = Length3(axis);
    if (sn < 1e-5f)
        return;
    for (int k = 0; k < 3; ++k)
        axis[k] /= sn;
    float angle = std::atan2(sn, Dot3(vn, wn));
    float most = settings.spineLeanMaxDeg / 57.2957795f;
    if (!(most > 0.0f))
        most = 0.5f;
    if (angle > most)
        angle = most;

    // Rodrigues, then transposed, because the game's matrices hold their axes
    // as ROWS and a row vector turns by the transpose.
    const float s = std::sin(angle), t = 1.0f - std::cos(angle);
    const float kx = axis[0], ky = axis[1], kz = axis[2];
    const float R[9] = { 1.0f + t * (kx * kx - 1.0f), -s * kz + t * kx * ky, s * ky + t * kx * kz,
        s * kz + t * kx * ky, 1.0f + t * (ky * ky - 1.0f), -s * kx + t * ky * kz, -s * ky + t * kx * kz,
        s * kx + t * ky * kz, 1.0f + t * (kz * kz - 1.0f) };
    float M[9];
    Transpose3(R, M);

    int moved = 0;
    for (int i = 0; i < count; ++i) {
        if (!s_turn[i])
            continue;
        if (LeanOneJoint(g_body.joints + i * kJointStride, M, pivotPos))
            ++moved;
    }

    // AND WHATEVER YOU ARE HOLDING (2026-09-25, user: "lean and peak causes the
    // gun to pop out of your hand when leaning").
    //
    // Mine, from last night. The arms used to be taken back out of this
    // rotation and I put them in, because leaving them out tore the shoulder.
    // It did - but it also means the hand MOVES now, and the gun is a separate
    // skeleton that the game has already placed against the hand by the time
    // this runs. Move the hand, leave the gun, and the gun stays behind.
    //
    // So the weapon gets the same turn about the same pivot. Only what is
    // actually in a hand: the search that found these looks near the wrist, but
    // it only re-runs every couple of seconds, and a gun holstered since would
    // otherwise be dragged off the belt. Checked against both wrists, because
    // the off hand holds a long gun too.
    int gunsMoved = 0;
    {
        float wristPos[2][3];
        bool haveWrist[2] = {};
        haveWrist[0] = g_body.left.valid && JointWorldPos(g_body.joints, g_body.left.wrist, wristPos[0]);
        haveWrist[1] = g_body.right.valid && JointWorldPos(g_body.joints, g_body.right.wrist, wristPos[1]);
        for (int s = 0; s < g_gunSkCount && (haveWrist[0] || haveWrist[1]); ++s) {
            const GunSkeleton& sk = g_gunSk[s];
            if (!sk.joints || sk.count <= 0)
                continue;
            const int anchor = sk.anchor >= 0 && sk.anchor < sk.count ? sk.anchor : 0;
            float grip[3];
            if (!JointWorldPos(sk.joints, anchor, grip))
                continue;
            bool inHand = false;
            for (int hnd = 0; hnd < 2 && !inHand; ++hnd) {
                if (!haveWrist[hnd])
                    continue;
                float gap[3];
                Sub3(grip, wristPos[hnd], gap);
                inHand = Length3(gap) < 30.0f;
            }
            if (!inHand)
                continue;
            for (int i = 0; i < sk.count && i < kMaxGunJoints; ++i) {
                if (LeanOneJoint(sk.joints + i * kJointStride, M, pivotPos))
                    ++gunsMoved;
            }
            // AND THE WEAPON'S OWN TRANSFORMS (2026-09-25, user: "I found it.
            // It was lean and peek that broke the laser position").
            //
            // The lean turns bones, and the laser is not drawn from a bone. It
            // is drawn from one of the transforms hanging off the weapon
            // OBJECT, which is not part of any skeleton and so was left exactly
            // where it was while the gun it belongs to leaned away from it.
            // Every centimetre you lean is a centimetre the laser is wrong by,
            // which is why it looked like a misalignment rather than a break.
            //
            // The gun hold already carries these three for the same reason. The
            // lean is the other thing that moves a weapon, and it has to carry
            // them too, or the two halves of the mod disagree about where the
            // gun is.
            if (sk.object) {
                // CARRY EVERY TRANSFORM, NOT THREE OF THEM (2026-09-27, user:
                // "ah, lean and peek.. that makes sense").
                //
                // Three named offsets were the three we happened to find when
                // the laser first drifted, and the laser has drifted again the
                // moment lean and peek came back on. Rather than hunt for a
                // fourth and then a fifth, lean everything on the weapon that
                // looks like a transform.
                //
                // A 4x4 with a bottom row of 0,0,0,1 and three unit-ish rows
                // is a transform; random floats are not, and anything that
                // fails the test is left alone. LeanOneMatrix checks the same
                // thing again before writing, so a false positive costs
                // nothing but a few microseconds.
                // THE WRONG COLUMN (2026-09-28, user: "make sure the laser
                // sticks instead of wobbling to your movement").
                //
                // This guard tested m[12], m[13], m[14] - and in a row major
                // 4x4 those ARE the translation. Requiring them to be zero
                // skipped every transform that actually sits somewhere,
                // which is all of them: the muzzle at +0x2A0 is 17 cm out
                // along the barrel and was rejected every single frame.
                // LeanOneMatrix right below reads w[12..14] as the position
                // and rotates it about the pivot, so it was built for
                // exactly the matrices this was throwing away.
                //
                // The column that must be zero is the projective one -
                // m[3], m[7], m[11] - which is what the gun hold sweep has
                // always checked. So the lean has been carrying nothing on
                // the weapon object, and every centimetre of head movement
                // has been a centimetre the laser was wrong by.
                constexpr DWORD kSweepTo = 0x400;
                for (DWORD off = 0; off + 64 <= kSweepTo; off += 16) {
                    float m[16];
                    if (!TryRead(m, sk.object + off, sizeof(m)))
                        continue;
                    if (std::fabs(m[15] - 1.0f) > 1e-3f || m[3] != 0.0f || m[7] != 0.0f || m[11] != 0.0f)
                        continue;
                    bool unitish = true;
                    for (int r = 0; r < 3 && unitish; ++r) {
                        const float len = m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1]
                            + m[r * 4 + 2] * m[r * 4 + 2];
                        unitish = len > 0.9f && len < 1.1f;
                    }
                    if (unitish)
                        LeanOneMatrix(sk.object + off, M, pivotPos, nullptr);
                }
                // The three known ones - 0xA0, 0xE0 and 0x2A0 - are all
                // sixteen byte aligned and inside the sweep, so they are
                // already covered. Leaning them again here would rotate them
                // twice and break the ones that currently work.
            }
        }
    }
    (void)gunsMoved;
#if RE5VR_DIAGNOSTICS
    {
        static unsigned long long s_toldLean = 0;
        const unsigned long long nowLean = GetTickCount64();
        if (nowLean - s_toldLean >= 2000) {
            s_toldLean = nowLean;
            Log_Printf("SpineLean: pivot joint %d, %d joint(s) turned %.1f deg; your head is %.1f cm from where it "
                       "was taken",
                pivot, moved, angle * 57.2957795f, Length3(lean) * 100.0f / upm);
        }
    }
#endif
}

void ArmIk_ForgetSkeleton()
{
    g_neededFor = nullptr; // the next body rebuilds the needed set and clears the index-keyed caches
    // NOT THE ARM LENGTH (2026-09-25, user: "level transition definitely still
    // break the aim ik"). The fourth place doing this, and the one the canary
    // was still catching seven times a session.
    //
    // This is called when the skeleton is rebuilt in place, which is exactly
    // what a level transition does - the camera hook sees the arm joints' links
    // change and asks for everything to be found again. Fair enough for joint
    // INDICES, which really have moved. The arm's length is not an index. It is
    // 53.2 units before the transition and 53.2 after, and throwing it away
    // forces a cold start on a skeleton that is still being rebuilt, which is
    // the worst possible moment to measure anything.
    //
    // Left alone, like the other three. A length that has genuinely changed
    // fails the ten per cent test on the next frame and the heal re-derives it
    // with the animation properly holding the arm.
}

void ArmIk_NoteSkeletonHooked(bool hooked)
{
    g_skeletonHooked = hooked;
}

bool ArmIk_GetStatus(ArmIkStatus& out)
{
    AcquireSRWLockShared(&g_statusLock);
    const bool fresh = g_statusMs && GetTickCount64() - g_statusMs < 250;
    if (fresh)
        out = g_status;
    ReleaseSRWLockShared(&g_statusLock);
    return fresh;
}
