#pragma once

// Holding the arm pose against code that cannot be hooked (2026-09-18).
//
// Arm IK sets each joint's local rotation, and the animation sets it back. Two
// of the three routines that write it can be detoured; the third, exe+1A1A1C,
// crashes the game on level start whatever is patched there - twice, with the
// same fault address both times, and no crash at all once it was left alone.
//
// While the gun is down that does not matter much: the writers run once or
// twice a frame and ours lands last often enough. Aiming changes the numbers
// entirely. Measured over four seconds with the gun up, exe+1A1A1C wrote the
// elbow's rotation 1505 times against 215 for the routine we hook - about ten
// blends a frame, all of them after us. Our pose becomes one input to a linear
// blend that is never renormalised, and the result is a matrix with the
// rotation partly cancelled out: the arm's world rows measured 0.815 and 1.42
// instead of 1, which is the swelling, and the pose is wrong on top of it.
//
// A debug register does not care which routine does the writing. Four of them,
// one per joint that matters - both shoulders and both elbows - watching the
// last component of each rotation, and a handler that puts our value back the
// instant anything changes it. No code is patched, and a writer we have never
// found is covered the same as the three we have.
//
// The cost is a trap per write, which is a dozen or so a frame while aiming.
// The finder felt ruinous because it watched an address that was READ thousands
// of times a second; this watches writes to four floats.

// A joint to guard, named by the skeleton it belongs to as well as by its
// index (2026-09-24). Both were one skeleton's until now, and the index alone
// is not a name: your character and your partner's both have a joint 46, so
// holding theirs parked their rotation on yours. That is what made a tester's
// Chris lose his top half, and it is why a partner has never been guarded.
struct PoseGuardSlot {
    unsigned char* joints;
    int index;
};

// Starts guarding, or changes what is guarded. Up to four slots, and they may
// belong to different skeletons. Safe to call every frame with the same list.
void PoseGuard_Set(const PoseGuardSlot* slots, int count, int jointStride, int rotOffset);

// The pose to hold for one of those joints, as a unit quaternion. Called by the
// solver whenever it works out a new one. Whose joint, and which.
void PoseGuard_Hold(unsigned char* joints, int index, const float quat[4]);

// Stop guarding and give the animation its arm back.
void PoseGuard_Clear();

// Once a frame from the render thread. Re-arms threads the game has created
// since the last pass, since a watchpoint only exists on the threads it was
// set on, and reports what it has been doing.
void PoseGuard_Tick();

// How many writes it has put back since the last call, for the log.
unsigned long PoseGuard_TakeSaveCount();
