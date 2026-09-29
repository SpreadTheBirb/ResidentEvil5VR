#pragma once

// Arm IK: your controllers drive the character's arms (2026-09-17).
//
// The first real step toward 6DOF. 3DOF aiming turns the gun by writing the
// game's own aim angles, which is what makes a shot go where you point, but
// the arms holding the gun are still playing whatever the animation says. With
// the joints now known - FindArms in camera_rig_hook.cpp walks them out of the
// skeleton - the hands can be put where you are actually holding the
// controllers, and the two bones above each hand solved to match.
//
// Three things had to line up before any of that could be written:
//
//   1. WHERE a hand is, in a frame the game understands. A controller's
//      position arrives in the runtime's raw tracking space. Taking the offset
//      from the headset and reading it off against the recentre reference
//      (VRBridge_GetTrackingFrame) puts it in the recentred frame, where
//      forward means forward. That offset is then laid onto the character's
//      own axes - right, up, and the way the body faces - so turning the
//      character carries your hands around with it, the way shoulders do.
//
//   2. HOW BIG a metre is. RE5's units are close to centimetres but the
//      characters are not all one size (Sheva is 0.93 of Chris), so the scale
//      is taken from the character's own arm rather than assumed: however many
//      units its shoulder-to-wrist measures, divided by a real one.
//
//   3. WHEN to write. The skeleton's world matrices are rebuilt from the
//      animation every frame, so a write that lands before the rebuild is
//      thrown away. The head collapse writes a local scale, which is an INPUT
//      to that rebuild, and so does not care. This does: it sets the world
//      matrices directly. If the arms do not move, that ordering is why, and
//      XrInputSettings::findArmWriter arms a watchpoint on the wrist's world
//      matrix to find the instruction to sit behind instead.
//
// Only the player's skeleton is touched, and only while the game has not taken
// the camera away for a cutscene.

struct ArmIkArm {
    int shoulder = -1, elbow = -1, wrist = -1;
    bool valid = false;
};

struct ArmIkBody {
    unsigned char* joints = nullptr;
    unsigned char* character = nullptr;
    int jointCount = 0;
    int head = -1;
    // Straight from the game's own aim flag on the controller. The arm is posed
    // by different code while the gun is up, so the moment it changes is the
    // moment anything we remember about the pose stops being true.
    bool aiming = false;
    ArmIkArm left, right;
};

// Once per frame from the camera hook's player path: caches who to drive, and
// solves on the spot when the skeleton hook is not running.
void ArmIk_SetBody(const ArmIkBody& body);

// From the skeleton-build hook, once for every joint the game composes. Solves
// on the last one of each frame, which is after the arm is built and before
// the weapon is attached to the hand.
void ArmIk_OnJointBuilt(unsigned char* joint);

// Whether that hook is installed and switched on. When it is not, the solve
// falls back to the camera hook: the arms still follow, the gun does not.
bool ArmIk_LateWriteActive();
void ArmIk_NoteSkeletonHooked(bool hooked);

// The skeleton was rebuilt where it stands (a costume finishing loading):
// same address, bones at different indices. Forget everything keyed to it.
void ArmIk_ForgetSkeleton();

// The object that reads the hand's world matrix once a frame (exe+19CB74).
// A held weapon lives in its own object, not in the character and not in the
// skeleton, so this pointer is the only way to reach it.
// From just after the animation finishes writing a joint's rotation - all
// four floats of it - which is the only point where a pose can be changed and
// still reach the screen. Hooked at three addresses, because the animation
// writes through three nested routines and only the last one in wins.
void ArmIk_OnPoseComplete(unsigned char* joint);

void ArmIk_NoteHandReader(void* source, void* object);

// Somebody else's skeleton - the co-op partner. The camera hook sees their
// controller go past every frame just as it sees the player's, so this is only
// a matter of saying so. Used to stop your hands passing through them.
void ArmIk_NoteOtherSkeleton(unsigned char* joints, int jointCount);

// The partner as a body to DRIVE rather than merely to bump into. Same shape
// as the player's, found the same way, so the same solver can run on it: their
// hands arrive over the network and their arms are posed from them. Call it
// every frame the partner is on screen; it goes stale on its own.
void ArmIk_SetPartnerBody(const ArmIkBody& body);

// Drive the partner's arms, without going through the player's own body
// (2026-09-24). The partner solve has always been called from the tail of
// ArmIk_SetBody, which only runs once the PLAYER's body, arms and head have
// all been identified - so on a machine where any of those three fails, a
// partner sending twenty good packets a second is never drawn and nothing
// anywhere says why. Whose arms we can see has nothing to do with whose arms
// they are.
void ArmIk_SolvePartnerNow();

// Stand in a T-pose, arms straight out to the sides, controllers held level, and
// call this. It sizes the character's arms to yours - where your shoulders are,
// how long each arm is - and ties each controller's angle to the skeleton's own
// rest rotation, so pointing does not need an aim to find level first.
void ArmIk_Calibrate();

// When a costume or character change last cleared the T-pose calibration, or 0.
// The menu turns this into a prompt; clearing it dismisses that prompt.
unsigned long long ArmIk_CostumeChangedMs();
void ArmIk_ClearCostumeChanged();

// Finding a number in the game by watching it change, the way a trainer does.
// Type what you can see, press Find; change it in game; type the new value and
// press Narrow. Repeat until one address is left. Reads only, never writes.
void MemFind_First(unsigned value);
void MemFind_Next(unsigned value);

// And what holds a pointer to an address, so a heap address found once can be
// reached again next launch. Anything in the exe's own data is a fixed base.
void MemFind_PointersTo(unsigned target);

// Read the pointer at re5dx9.exe+exeOffset, step forward by addOffset, and say
// what is there. Proves or kills a route in one press.
void MemFind_Follow(unsigned exeOffset, unsigned addOffset);

// Put a hardware watchpoint on an address and report what writes to it. The way
// to find the code behind a number when no fixed pointer path exists.
void MemFind_Watch(unsigned target);
bool ArmIk_IsCalibrated();

// Where the T-pose put your shoulder relative to your head, in metres, in the
// recentred frame. False before a calibration. Sent to a co-op partner: hand
// relative to SHOULDER is the one measurement that means the same thing on two
// differently sized people. See net/ik_sync.h.
bool ArmIk_GetShoulderFromHead(int hand, float out[3]);

// How much is added to your measured arm at calibration, in metres, to stand
// in for the shoulder travel a T-pose cannot show. Raising it SHORTENS the
// character's arms. See the note on kReachAllowanceMetres.
void ArmIk_SetReachAllowance(float metres);

// This character's units per real metre of YOUR arm, as the T-pose measured
// it. False before calibration. Sent to a co-op partner so their machine can
// scale your reach onto the character you are playing.
bool ArmIk_GetUnitsPerMetre(int hand, float* out);

// Which way the gun on the character's arm really points, in the gun
// controller's own axes (right, up, forward). It is not the controller's
// forward: measured, it sits about 23 degrees above it. False while the arms
// are not being driven.
bool ArmIk_GetBarrelInController(float out[3]);

// Developer, from EndScene: whether anything moved the gun wrist after the
// solve posed it, which tells a recoil layer overriding us apart from the
// weapon model's own firing animation.
void ArmIk_OnEndScene();

// After the arm is posed each frame: puts the weapon's attach transforms back
// toward where they sit on the hand between shots, letting only the chosen
// share of the firing kick through.
void ArmIk_SteadyGun();

// Every bone of the weapon in your hand, where it sits relative to the grip,
// in centimetres. Looking for a magazine: a bone a few centimetres below the
// grip and nowhere near the barrel is one, and a bone that can be found can
// be moved - which is as close to a magazine falling out of the gun and
// appearing in your hand as this mod can get without new geometry.
// Your character's skeleton, for anything that has to reason about it from
// outside - the bone palette matcher, for one. False when nobody is found.
bool ArmIk_PlayerSkeleton(unsigned char** joints, int* count);

// The joints that bound the parts worth keeping: the head to hide, and each
// arm's shoulder, elbow and wrist so a point can be placed above or below the
// elbow. -1 for anything not found.
struct ArmIkLandmarks {
    int head = -1;
    int shoulder[2] = { -1, -1 }; // left, right
    int elbow[2] = { -1, -1 };
    int wrist[2] = { -1, -1 };
};
bool ArmIk_Landmarks(ArmIkLandmarks& out);

// A joint's parent, so a caller can work out what hangs off what. -1 at the
// root or when it cannot be read.
int ArmIk_JointParent(unsigned char* joints, int index);

// One joint's world matrix, sixteen floats, rows of three with the position
// at 12..14. False if it could not be read.
bool ArmIk_JointWorldMatrix(unsigned char* joints, int index, float out[16]);

void ArmIk_DumpGunBones();

// The weapon in your hand, and which of its moving parts is currently taken to
// be the magazine. False before one has been watched through a reload.
// 'which' counts from 1 so it reads as "part 2 of 3".
bool ArmIk_MagazinePick(int& bone, int& which, int& outOf);

// That was the wrong part: step to the next candidate and write it down. The
// mod can find what MOVES on any weapon, which is measurement, but which mover
// is the magazine is a guess - and a person looking at the gun can settle it in
// one press where no rule about grips or timings survives thirty weapons.
void ArmIk_NextMagazinePick();

// WHICH BONE IS THE MAGAZINE (2026-09-25). Where a bone SITS does not say
// what it is - the anchor is the model's root until the hold has learned a
// grip, so every distance is measured from the wrong place. What a bone DOES
// says it outright: during a reload the magazine leaves the weapon by several
// centimetres and comes back, and nothing else on a pistol does that.
//
// Arms six seconds of sampling. Reload once inside it, any way you like, and
// the log names the bone.
void ArmIk_WatchTheReload();

// Lean the upper body by however far your head has moved in the room. Called
// from the camera rig hook, before the gun is dealt with.
void ArmIk_LeanSpine();

// Why the partner's arms are or are not moving. "Nothing arrived" and "it
// arrived and the solve refused it" look identical from the outside, and they
// want completely different fixes, so each step says whether it happened.
struct ArmIkPartnerStatus {
    bool haveBody;    // their character and joints are on screen and found
    bool armsFound;   // and we could make out two arms on that skeleton
    bool handsFresh;  // a packet with hands in it arrived recently
    bool haveScale;   // and it carried a calibration, without which nothing runs
    bool tied;        // their hand has been tied to that skeleton's rest pose
    bool solving;     // and the solve actually ran this last quarter second
};
void ArmIk_GetPartnerStatus(ArmIkPartnerStatus& out);


// WHAT TO HIDE, WITHOUT HIDING IT (2026-09-26, user: "my screen actually went
// completely black and my fps tanked").
//
// The keep set is worked out here, where the hierarchy and the landmarks are,
// and nothing is written to the skeleton. Writing NaN into a joint world
// matrix poisoned everything in this mod that reads one - the arm solve, the
// lean, the gun mount, the eye - and the log caught it arriving as a NaN
// camera distance and a frame rate falling from sixteen to five.
//
// So the answer is published instead, and applied in the bone table on its way
// to the shader, where the GPU is the only thing that ever sees it.
//
// pos/ok/keep all have `count` entries and point at storage refreshed once a
// frame. False means there is nothing to hide.
struct ArmIkBodyHide {
    int count;
    const float (*pos)[3]; // world position of each joint
    const bool* ok;        // whether that position could be read
    const bool* keep;      // whether it survives the current mode
    float cutUnits;        // how far past a kept joint still counts as kept
};
bool ArmIk_BodyHide(ArmIkBodyHide& out);

// How many joints there are between each wrist and the shoulder, which is the
// whole of the useful range for the cut. Four on Chris. False before a body
// has been found.
bool ArmIk_ArmSteps(int out[2]);

// What it managed, for the log and the status readout. False when it has not
// run in the last quarter second.
struct ArmIkStatus {
    float unitsPerMetre;
    float reachUnits[2];    // left, right: how far each arm can stretch
    float missUnits[2];     // how far short of your hand each one had to stop
    bool handTracked[2];
};
bool ArmIk_GetStatus(ArmIkStatus& out);
