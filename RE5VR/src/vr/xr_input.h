#pragma once

#include <windows.h>
#include <Xinput.h>

// Motion controllers as a gamepad (v0.4.3).
//
// RE5 knows nothing about VR controllers and never will, so the controllers
// are translated into the one thing it already understands: an Xbox pad.
// xr_input.cpp owns the OpenXR action side (an action set, suggested bindings
// for every controller we know about, and a sync each frame on the submit
// thread) and publishes a snapshot; ui/input_block.cpp merges that snapshot
// into the XINPUT_STATE the game reads, the same way a second physical pad
// would show up. Nothing about aiming moves yet: this is buttons and sticks.
//
// The mapping, with the game's own defaults in mind (right grip is the aim
// button the user asked for, and RE5 readies the weapon on the left trigger):
//
//   right grip          -> left trigger   aim
//   right trigger       -> right trigger  fire
//   right stick         -> right stick    camera
//   right stick click   -> R3
//   right A / B         -> A / B          action, knife
//   left trigger (hold) -> nothing        d-pad modifier, see DpadMethod
//   left stick          -> left stick     move
//   left stick click    -> L3             (L3 + R3 still opens the mod menu)
//   left X / Y          -> X / Y          reload, inventory
//   left grip           -> left shoulder
//   menu button         -> start

// How the d-pad is reached, since a pad has one and a controller does not.
// UEVR ships several because the hardware differs: only Touch controllers
// have a capacitive thumbrest, so Index, Vive and Pico need another way.
enum class XrDpadMethod {
    LeftTrigger = 0,  // hold the left trigger, then the right stick. Works on everything.
    RightThumbrest,   // rest your thumb on the right thumbrest, then the left stick (Touch only)
    LeftStickClick,   // hold the left stick in, then the right stick
    RightStickAlways, // the right stick is always the d-pad, never the camera
    Off,
};

struct XrInputSettings {
    bool enabled = true;
    int dpadMethod = 0; // an XrDpadMethod, kept as an int so it stores in re5vr.ini
    float deadzone = 0.15f; // stick centre to ignore, as a fraction of full tilt
    bool swapHands = false; // left-handed: the hands trade roles
    // The CHARACTER's handedness, which is a different fact from the player's
    // (2026-09-20). Sheva holds her weapon and her knife in her LEFT hand, so
    // playing her moves everything that means "gun hand" to the other
    // controller: aiming, drawing from a holster, the grip that readies the
    // knife, and which arm the IK drives from which controller.
    //
    // The two combine rather than compete. A left-handed player driving a
    // left-handed character is back where they started, which is why this is
    // exclusive-or and not an override.
    bool characterLeftHanded = false;
    // 3DOF aiming: the gun's pitch follows where the controller points. Yaw
    // comes later, since it has to decide when Chris turns to follow.
    bool pointToAim = false;
    // Developer, ini only (VR.MotionAimFindWriter): arm a watchpoint on the
    // game's aim pitch field and log what writes it. See hooks/aim_finder.h.
    bool findAimWriter = false;
    // Developer, ini only (VR.MotionAimWriteField): which field the direct
    // write targets. 0 none, 1 the camera pitch at +0x2DC8, 2 +0x2908.
    // +0x2DC8 turned out to be the camera only: writing it pitched the view
    // and never moved the gun, which is the opposite of what 3DOF wants.
    int aimWriteField = 0;
    float aimPitchTrimDeg = 0.0f; // hold the controller a little low or high and still aim level
    // How much faster than the game's own "fastest" setting the gun may turn.
    // 1 leaves RE5 exactly as it ships. The integrator at exe+7879B2 multiplies
    // stick by three floats in a settings block and nothing else caps it, so
    // this simply scales those - see AimSpeed_Apply in camera_rig_hook.cpp.
    float aimSpeedMult = 1.0f;
    // How long the servo takes to close the gap between where the gun is and
    // where the controller points. Lower is tighter and readier to overshoot,
    // higher is smoother and lazier.
    float aimServoMs = 120.0f;
    // How much of the stick RE5 ignores around centre. The servo lifts every
    // request it makes clear of this, so small movements are not swallowed.
    float aimStickDeadzone = 0.25f;
    // Hold the stick at a fixed push for direction only and set the rate by
    // writing the game's own aim speeds. Off falls back to asking for the rate
    // with stick deflection, which the game's dead zone coarsens.
    bool aimRateDrive = true;
    // The fastest the gun may turn under the rate drive, degrees a second. A
    // wrist flick runs 150 to 300, so the default leaves headroom over one.
    float aimMaxRateDeg = 450.0f;
    // How much of your hand's own small movement reaches the gun. 0 lets
    // everything through, including sensor noise; 1 holds the gun as still as
    // a stick that has been let go. The point is not a still gun, it is a gun
    // that moves the way a held gun moves, so the default only takes the
    // shimmer that is not really your hand.
    float aimSteadiness = 0.0f;
    // Developer: draw the servo's live numbers over the game, so the dials
    // above can be tuned with the gun up and the menu closed.
    bool aimOverlay = false;
    // Developer: narrow down where the character's facing is stored, by
    // elimination across several turns. See BodyFacingScan.
    bool findBodyFacing = false;
    // Developer: watchpoint the camera's frustum planes, to find what builds
    // them. See CullingPatch_FindPlaneWriter.
    bool findCullPlanes = false;
    // Developer diagnostic: widen the culling frustum on EVERY camera, not just
    // while the game looks like it has taken the camera. Costs frames; the point
    // is to tell a fix that did not work from a fix that never ran.
    bool wideCullAlways = false;
    // Developer: work out which joints are the arms, and report them. The first
    // step of arm IK - see FindArms in camera_rig_hook.cpp.
    bool findArms = false;
    // Developer: while aiming, log where the game's own aim animation puts the
    // off hand relative to the gun hand, per weapon. The grab point for
    // two-handed aiming, taken from Capcom's animation instead of a table.
    // Only measured with 6DOF arms off, so the pose is the animation's.
    bool measureGrips = false;
    // Developer: while aiming, add up how far every joint turns, and when the
    // gun goes down log the ones that turned most - the firing shake shows up
    // as the joints that twitch when you shoot and sit still when you do not.
    bool measureShake = false;
    // Developer: look for the held weapon's own skeleton - a joint array like
    // the character's, sitting in the gun hand - and log which of its bones
    // move while firing. The groundwork for steadying the kick and for
    // reloading and racking by hand.
    bool findGunBones = false;
    // How big the held weapon is drawn, 1 being the game's own size. Testers
    // found the guns too big in VR (2026-09-23). Local only: it scales the
    // weapon model on this machine, so a co-op partner is not affected and
    // nothing needs to travel over the network.
    float gunScale = 1.0f;
    // Point with the body: the character turns to the heading the controller
    // points at, instead of a servo asking the stick to get there.
    bool absoluteYaw = true;
    // 6DOF: each of your hands drives the matching arm, so where you hold a
    // controller is where the character's hand goes. See hooks/arm_ik.cpp.
    bool armIk = false;
    // How much of the way from the animation's hand to yours the arm is moved.
    // 1 is all the way; lower leaves some of the game's own pose showing.
    float armIkWeight = 1.0f;
    // Your arms against the character's. Above 1 reaches further for the same
    // real movement, which is what a shorter character needs.
    float armIkScale = 1.0f;
    // Developer: instead of following a controller, hold the hands out in a
    // fixed pose. The one test that tells "the write does not land" apart from
    // "the write lands somewhere wrong".
    // How far back from the controller the IK aims, in metres. A controller is
    // held in the PALM, so treating its position as the wrist puts the pivot a
    // hand too far out - flex your wrist and the solver reads the whole hand as
    // TRAVELLING and swings the arm instead of turning the wrist. The user feels
    // that as the arm being grabbed a quarter of the way up the forearm.
    //
    // Starts at 0, which is exactly how it behaves today. This was baked in as a
    // constant once before and taken back out, because it was changed at the same
    // time as the arm-length measurement and the two corrections fought. One at a
    // time this time.
    float armIkWristPivotM = 0.0f;
    // Developer: pick one joint and turn it a long way, so it is obvious what
    // that joint actually drives. -1 is off. Everything else about the solve
    // carries on as normal; this lands on top.
    int armIkPokeJoint = -1;
    float armIkPokeDeg = 60.0f;
    // Your hands stop at bodies instead of passing through them - Sheva, and
    // your own. The arm holds at the surface while your real hand carries on,
    // which is the only resistance a controller with no brake can give you, and
    // it is what every VR game means by the word.
    bool armIkTouch = true;
    // How thick a body is, in game units. Roughly a hand's width around every
    // bone, which is enough to feel like a person and not so much that you
    // cannot reach past an arm.
    // Raised 11 -> 16 and then put back to 11 the same day. Sheva read thin at
    // 11, so the ball was made bigger, and at 16 everybody reads fat: "we
    // should also shrink the skeleton where it was before. It's too big."
    // Which says the radius was never the right lever. The balls sit on
    // JOINTS, and a torso has only a few, so a bigger ball fixes the gaps
    // between them by inflating the whole person. And the point being tested
    // is the WRIST, while the hand you can see carries on past it, so contact
    // lands late however big the ball is. Capsules between consecutive joints,
    // and a test point out at the palm, are the two fixes that do not trade
    // one complaint for the other.
    float armIkTouchRadius = 11.0f;
    // Physical melee (2026-09-19). With the knife up, swing a hand and the
    // character swings, instead of reaching for the trigger. The knife itself
    // still comes out on the left grip and has to be held, because a gesture
    // that can put a weapon in your hand has to be read from nothing at all,
    // and a hand moving fast is not a rare enough thing to read it from.
    bool meleeSwing = true;
    // How hard a swing has to be before it counts, metres a second at the hand.
    // A walking arm swing peaks near 1.5 and a deliberate slash runs 3.5 to 6,
    // so the default sits in the gap. Speed is measured as one hand AGAINST the
    // other, so walking, turning and leaning move both and cancel out.
    float meleeSwingSpeed = 3.0f;
    // Holsters (2026-09-19). Reach to a place on your body and squeeze the gun
    // hand's grip, and that place is a weapon: over the left shoulder is the
    // knife, the two hips and the right shoulder and the lower back are the
    // four d-pad slots you have already assigned in the inventory. The mod
    // never learns what a weapon IS. It presses the direction you would have
    // pressed.
    // How much further you can reach in play than you could in the T-pose,
    // because your shoulder travels and the T-pose does not measure that. It
    // is ADDED to your measured arm, so RAISING it makes the character's arms
    // SHORTER. Testers reported "trex arms" on a value fitted to one person,
    // so it is adjustable.
    //
    // Nothing, by default (2026-09-23, user: "0.12 is what it defaults to and
    // 0 gets much better results"). 0.12 was fitted to one pair of shoulders
    // and every report since has been that it takes too much arm away. Zero
    // trusts your T-pose exactly, which is a measurement of you rather than a
    // guess about you, and anyone whose shoulders travel more than most can
    // still put it back.
    float armIkReachAllowanceM = 0.0f;
    // Up and down on the look stick (2026-09-23, user: "we should disable the
    // rotation of the camera with up and down on the joystick. It causes a lot
    // of confusion when 6dof is enabled"). Your head already points the view
    // up and down, so a stick doing the same thing leaves the horizon
    // somewhere you did not put it, and the arms reaching for a body that has
    // tilted under them. Off. Left and right still turn you, and the stick
    // still stands in for the d-pad.
    bool stickPitch = false;
    // Turning with your head (2026-09-24, user: "look left, emulate left
    // stick input. Look right, emulate right stick input. So that your
    // characters body is always forward").
    //
    // Turn your head past the dead angle and the character turns to catch up,
    // which brings your head back toward his forward, which stops the turn.
    // The loop closes itself and needs no tuning, because the thing driving
    // the turn is the very thing the turn reduces: the headset's rotation is
    // reported against the recentred forward, and the game's camera already
    // turns with the character, so that angle IS how far your head is turned
    // from where he is facing.
    //
    // Added to the look stick rather than replacing it, so your thumb still
    // works and the two sum instead of fighting.
    bool headTurn = false;
    // How far you can look before he starts turning, and how far before he is
    // turning as fast as the stick would.
    float headTurnDeadDeg = 25.0f;
    float headTurnFullDeg = 65.0f;
    // Leaning the MAN, not just the camera (2026-09-23, user: "this would be
    // the spine moving I think for actual dodging. There's a chance it would
    // move the hitbox", and "it's almost like a step into roomscale
    // movement"). The upper body turns about the base of the spine so that
    // his head arrives where yours is, which is what makes leaning out from
    // cover a thing that happens in the world rather than a thing that
    // happens to the picture. Whether it moves what an enemy can hit is a
    // question for the game and nobody here can answer it from the code.
    //
    // Off until it has been tried. Leaning has to be on for it to do
    // anything: this follows the same head movement the view already does.
    // On with leaning (2026-09-24, user: "I think it should automatically make
    // your characters torso lean with you"). It costs nothing when leaning is
    // off, because it asks for the lean offset first and there is not one.
    bool spineLean = true;
    // Step in your room and he takes a step (2026-09-23). The spine covers
    // the small offsets; past what a lean can reach, his legs have to do it,
    // and his legs are the only part of this the game itself understands -
    // which is why it is the only version of dodging that can move what an
    // enemy is allowed to hit. Walls and furniture stay the game's problem,
    // because he walks into them the way he always did.
    //
    // Off until tried, and it does nothing while you are aiming, because the
    // game will not let you walk then.
    bool roomStep = false;
    // How far you can stray before his legs start, and how far before they
    // are going flat out, in metres.
    float roomStepDeadM = 0.15f;
    float roomStepFullM = 0.50f;
    // How much of your lean the body takes, and how far it is allowed to go.
    float spineLeanAmount = 1.0f;
    // Twenty, not thirty (2026-09-23, user: "a huge swivel to it which kinda
    // disconnects from Chris and does non-human leaning"). Most of that was
    // the reference being corrupted underneath it, but a spine that bends
    // thirty degrees at one joint does not look like a person either: past
    // about twenty, a person has already moved a foot.
    float spineLeanMaxDeg = 20.0f;
    bool holsters = true;
    // Two hands on the gun (2026-09-22). Squeeze the off hand's grip with it
    // on the gun - cupping a pistol, or out along a long gun's barrel - and
    // the gun points along the line between your hands, the way a real long
    // gun is aimed. With holsters on, that grip is no longer the knife: the
    // knife only comes from its holster, so the grip is free for this.
    bool twoHanded = true;
    // How high on a long gun the support hand sits, in cm, on top of what the
    // grab itself says. The first pass sat every hand under the gun.
    float twoHandRaiseCm = 4.0f;
    // How much of a long gun's aim the support hand owns, 0 to 1. At 1 the
    // gun follows only the line between your hands, which the user found "too
    // locked": switching targets meant moving both hands together. Shared,
    // a flick of the gun wrist moves the aim and the other hand steadies it.
    float twoHandSteer = 1.0f;
    // How much of your support hand's twist the gun takes, 0 to 1
    // (2026-09-23, user: "is there a way to allow some roll from your second
    // hand? I get side to side and up and down but no roll influence").
    // Pointing the gun at your off hand is a shortest-arc turn, and a
    // shortest-arc turn adds no roll by its very definition, so a rifle could
    // be aimed anywhere at all and still lie dead flat. This gives the gun a
    // share of however far your support hand has twisted about the barrel
    // since you took hold. It works on a cupped pistol too, where the gun
    // hand still does the aiming: "this also may help to aim down the iron
    // sights when dual gripping a pistol".
    float twoHandRoll = 0.5f;
    // How much of the gun's own firing kick gets through, 0 to 1 (2026-09-22).
    // Measured with 6DOF arms on, the arm stays where it is posed through a
    // whole magazine; what jumps is the frame the weapon hangs from, 3.2 deg
    // and 3.6 cm a frame against the hand while firing, 0.2 otherwise. Held
    // two-handed, most of it is soaked up.
    // Experimental, never saved: make the correction inside the game's own
    // code, straight after it writes the gun's transform. Placed without
    // having seen the bytes there, so it can crash; a restart turns it off.
    // Locking the gun into the hand (2026-09-23). The gun's own transform is
    // rebuilt by the game every frame from the animated skeleton, not from
    // the wrist this mod poses - which is why it shakes while the arm holding
    // it does not, and why it hops on the way into aim. Measured at the end
    // of the frame, the gun moves in the hand by up to 17 cm and 24 deg on a
    // single firing frame, more than the hand moves on the body.
    //
    // Freezing the weapon's own bones was tried and did nothing: the game
    // recomposes those later, 0 of 1248 corrections survived to the screen.
    // This is done inside the game's own writer instead, straight after it
    // lays the transform down - the same place the gun size is changed, which
    // is the one change that reached the screen.
    bool gunLock = true;
    // Experimental, never saved (2026-09-22): the hands are laid onto the
    // character's facing, so if the game turns the body a little on every
    // shot, the whole gun swings round the wrist by that much - a degree is a
    // centimetre at the muzzle, every round of a machine gun. While firing,
    // the facing the hands use is smoothed; the two firing-kick sliders say
    // how much of the jitter gets through. The camera shake is not touched.
    bool steadyHandsFiring = false;
    float kickOneHanded = 0.5f;
    float kickTwoHanded = 0.15f;
    // A tap of the gun hand on the top of the other wrist opens the inventory,
    // the way you glance at a watch. Hand to hand needs no head pose and no
    // body yaw, only two tracked controllers, so it is the steadiest gesture
    // of the set.
    // Point at the mod menu with your gun hand (2026-09-23, user: "is it
    // possible to navigate the VR Mod menu with a laser on the controller?
    // Basically emulate a mouse?"). The menu already works in cursor terms -
    // a position in its own units, fed to the interface, with an arrow drawn
    // at it - and the real mouse and a pad both just write to that. This is a
    // third writer. The trigger is the left button and the grip is the right.
    bool menuLaser = true;
    bool wristInventory = true;
    // RELOADING BY HAND (2026-09-25, user: "Offhand to grip, pressing grip
    // should grab a magazine object, bring to weapon, rack it").
    //
    // Three beats, each one a thing your hands do rather than a button:
    // reach to your belt and squeeze, and the old magazine drops; bring your
    // hand up to the weapon, and the new one goes in; take your hand away,
    // and that is the rack. The gun is empty for as long as the middle beat
    // takes, which is the whole point of doing it by hand.
    // ON (2026-09-28). Reaching to your belt for a magazine is one of the
    // things people install a VR mod FOR, and leaving it off meant it was
    // only found by people who read every checkbox.
    bool manualReload = true;
    // What was still in the magazine when it left. RE5 keeps spare
    // ammunition as one pool rather than as separate magazines, so putting
    // those rounds back is the honest sum and reloading early costs nothing.
    bool reloadKeepsRounds = true;
    // WHICH BONE IS THE MAGAZINE (2026-09-25). Two of them, on the M92F:
    // one goes out and never comes back, one goes out to your belt and
    // returns seated. Numbers rather than constants, because the answer is
    // very likely different on a shotgun and finding that out should cost a
    // test rather than a build. -1 for neither.
    // SEVEN AND EIGHT (2026-09-26, user: "our mag reload function doesn't
    // quite work, I liked our old way vs scanning for whatever weapon you have
    // out. like the pistol and mp5 both use 7 for mag and 8 for what carries").
    //
    // Two weapons agreeing is decent evidence that these are a convention of
    // the models rather than an accident of one of them, and a fixed pair that
    // works beats a scan that guesses - which the scan was doing, because
    // which mover is the magazine was never something it could measure.
    //
    // -1 on either hands that weapon back to the watcher, which learns from
    // one reload and remembers it in re5vr_magbones.ini. That is the fallback
    // for a weapon these numbers turn out to be wrong for, not the default.
    int magazineBone = 7;
    int magazineWithBone = 8;
    // A click when the magazine leaves and one when it goes in, from
    // re5vr_magout.wav and re5vr_magin.wav beside the game. The game's own
    // reload noise still plays; these fill the silence where the mod took a
    // reload apart, which is mostly the eject.
    // HOW MUCH OF YOURSELF YOU SEE (2026-09-26, asked for by players): 0 the
    // whole character, 1 forearms and hands only, 2 hands only. The head is
    // collapsed in all three, because it is inside the camera.
    //
    // Done by writing NaN into the WORLD MATRIX of the joints that should not
    // be there. Not a local scale, which propagates to children and took the
    // camera down with it, and not a collapse onto a point, which leaves the
    // geometry present so it unfolds again whenever a kept limb comes near the
    // place it was folded into. A vertex weighted to a NaN bone is NaN, the
    // hardware discards any triangle holding one, and nothing needs restoring
    // because the game recomposes world matrices from the animation every
    // frame - not writing them IS the restore.
    int bodyVisible = 0;
    // HOW FAR BACK THE CUT SITS, in metres (2026-09-26, user: "the hand mode
    // from my last test was a little too far back, it needs to be closer to
    // the wrist").
    //
    // This one number does two jobs. It is the radius that catches the twist
    // and helper bones that deform a hand without hanging off the wrist in the
    // hierarchy, which is what kept eating fingers when it was too small, and
    // because those bones sit along the forearm it is also where the cut lands.
    // Bigger means more forearm survives.
    //
    // A slider rather than a constant because it is the kind of thing that
    // takes a few looks in the headset to settle and nobody should need a
    // rebuild between them.
    float bodyCutBack = 0.11f;
    // TELL THE GAME WHERE YOU ARE LOOKING FROM (2026-09-26, user: "you don't
    // see the checkboard if the camera is set to not follow your body").
    //
    // The mod replaces the view matrix and nothing else, so the camera object
    // the game keeps for itself stays where the game put it - and the renderer
    // sorts transparencies, measures fade distances and picks detail levels
    // from that object. Looking from somewhere it does not know about gives
    // wrongly sorted transparency, which reads as a stipple that never
    // resolves. Off until the position field has been found and trusted; the
    // log says when.
    // ON BY DEFAULT (2026-09-28). The position field has been found and
    // trusted for a while now, and with the action camera gate in place the
    // game and the mod finally agree about where the camera is - which is
    // what this setting is for. Leaving it off shipped the checkerboard to
    // everyone who never found the checkbox.
    bool cameraFollowsEye = true;
    // WHICH BONES TO SNAP AT (2026-09-26, user: "I think we just manually
    // assign to the bones like we did before").
    //
    // The same reasoning that settled the magazine on bones 7 and 8: finding
    // them by measurement works until it does not, and when it does not there
    // is nothing to do about it from in here. A number you can type always
    // works. Left at -1 each of these is found the way it always was, and the
    // log prints what was found so there is something to type.
// HOW FAR UP THE ARM, NOT WHICH BONE (2026-09-26, user: "the numbers don't
// seem to make any sense. When I set one side to 1, the whole body is
// visible ... and it seems random as to how it slowly takes over the body").
//
// Exactly so, and naming bones was a bad idea however much it worked for the
// magazine. A cut keeps the named bone AND EVERYTHING HANGING OFF IT, so a low
// number sits near the root and its subtree is the whole character. The
// numbering follows the rig's own order, which has nothing to do with how far
// up your arm anything is, so the useful values are unguessable and the
// useless ones are catastrophic.
//
// Counting steps up the arm instead fixes both halves. Zero is the wrist, one
// is the next joint above it, and so on up - always up the arm you are
// actually looking at, always in order, and it cannot reach your torso because
// the walk stops before the shoulder. A slider over that is honest: every
// value is valid and each one keeps a little more arm than the last.
//
// ONE EACH SIDE (2026-09-26, user: "so shouldn't I have sliders for each
// side?").
//
// Yes. The two arms are not the same chain - the wrists come in at bones 81
// and 54 on Chris, and the gun hand carries extra twist and helper bones - so
// the number of steps to the same PLACE on the arm differs between them. One
// number can only be right on both sides by luck, and when it is wrong it is
// wrong asymmetrically, which is exactly the "left side walks to the shoulder"
// that started this.
//
// First index is the mode, 0 Arms and 1 Hands. Second is the side, 0 left and
// 1 right.
    int cutStepsUp[2][2] = { { 2, 2 }, { 0, 0 } };
    bool reloadSounds = true;
    // These do not go through RE5's mixer, so the game's effects slider has
    // no reach over them and the mod has to carry its own.
    // 0.33 was where the user settled it against the game mix (2026-09-26).
    float reloadSoundVolume = 0.33f;
    // Developer: log where the gun hand is, in body coordinates, every time
    // its grip goes down. The holster zones are boxes in that space, and they
    // should be measured off the player rather than guessed at by me.
    bool holsterMeasure = false;
    bool armIkTest = false;
    // How much the collarbone carries when you reach past comfortable. 0 is a
    // shoulder pinned where the animation left it, which is what "very elbow
    // driven" and "the arm feels on a different rule set" were describing.
    float armIkShoulder = 0.35f;
    // Write the arms into the pose at the end of the game's own skeleton build
    // rather than after it, so the hand is already where you are holding it when
    // the game hangs the weapon off it. Off leaves the arms following and the
    // gun behind, which is also the way out if the hook ever misbehaves.
    // Drive the joint's local rotation at +0x20 - the pose the game composes
    // from - instead of painting over the world matrix it composes INTO. The
    // difference is everything downstream: fingers, the socket at the wrist and
    // whatever is being held all follow, because the game works them out itself.
    // Costs a frame of lag, since the pose is set before the build rather than
    // after it. Off goes back to moving the arm on screen and nothing else.
    bool armIkWriteLocal = true;
    // Turn the character's hand with yours, so the gun points down your
    // controller instead of wherever the forearm's roll leaves it. Tied to the
    // pose each time the gun comes up, which is also how you straighten it out.
    // ON by default from 2026-09-18. This began as "roll the forearm with your
    // hand" and it is now the gate on the whole hand write - the controller's
    // rotation reaching the hand, and through the socket lock, the gun. Without
    // it 6DOF tracks position only, which is not what anybody means by 6DOF.
    bool armIkWrist = true;
    bool armIkLateWrite = false;
    // Developer: watchpoint the right wrist's world matrix and report what
    // writes it, so the IK can be moved to just after the animation instead of
    // racing it. See hooks/aim_finder.h.
    bool findArmWriter = false;
    // Developer: watch the gun's attach transform for four seconds and log the
    // instruction that writes it, so the firing kick can be steadied at the
    // moment it is written - fixing it afterwards was overwritten every frame.
    bool findGunWriter = false;
};

void XrInput_SetSettings(const XrInputSettings& s);
XrInputSettings XrInput_GetSettings();

// Whether the LEFT controller is the gun hand right now, which is the player's
// handedness and the character's taken together. Everything that used to test
// swapHands should ask this instead, or a left-handed player playing Sheva
// ends up swapped twice and back to front.
bool XrInput_GunHandIsLeft();

// Whether the off hand is holding the gun, and whether that hold is a
// pistol's cupping hand rather than a hand out along a long gun.
bool XrInput_GetTwoHand(bool* cupped);

// Is that controller's grip squeezed right now? The raw button, before any of
// the meanings the game gives it. 0 is the left hand, 1 the right, and these
// are the physical controllers rather than the gun and off hands - a chord is
// something you hold, and which hand you hold it in should not depend on which
// character you are playing.
bool XrInput_GripHeld(int hand);

// A magazine is in your off hand right now. Two-handing the weapon and the
// wrist tap both live where a reload passes through, and neither should fire
// while it does.
bool XrInput_ReloadCarrying();

// What the magazine should be doing: 0 the game's business, 1 out of the gun
// and falling, 2 in your off hand, 3 held still in the weapon while the game
// plays its reload. Read by the arm solve, which is the only place a weapon
// bone can be written and still reach the screen.
int XrInput_MagazineHeld();

// The part of your support hand's twist that the gun did NOT take, as a
// rotation to lay on top of that hand's own pose (2026-09-23). The hand is
// locked rigidly to the gun while it holds it, so without this it keeps none
// of its own twist at all, while the gun keeps a share of it - and the two
// then disagree by exactly the amount the gun rolled. The arm answers that by
// twisting the forearm, which is the flip. False when no hand is on the gun.
bool XrInput_GetSupportRoll(float out[9]);

// Where your gun hand is pointing, as an angle left/right and up/down from
// where your head is looking, in radians, plus the two buttons. The menu
// turns those angles into a place on itself, because only it knows how big it
// is on screen. False when there is nothing to point with.
bool XrInput_GetMenuPointer(float* xRad, float* yRad, bool* click, bool* rightClick);

// How far the look stick has been pushed up or down, for scrolling the menu.
// Reported raw: the menu decides what a notch is worth.
float XrInput_GetMenuScroll();

// Where your HAND is, as the same pair of angles from where your head looks.
// Not where it points - where it is. The menu draws a line from there to the
// cursor, which is what makes a laser read as coming out of the controller.
bool XrInput_GetMenuHand(float* xRad, float* yRad);

// When the trigger last fired with the gun up (GetTickCount64), 0 if never.
unsigned long long XrInput_LastShotMs();

// The pad built from the last sync. False when motion controllers are off, a
// controller hasn't answered recently, or VR isn't running.
bool XrInput_GetPad(XINPUT_GAMEPAD* out);

// One line for the menu's Status tab, e.g. "Touch controllers, both hands".
void XrInput_DescribeStatus(char* out, size_t size);

// Where the gun hand is pointing, in the VR reference space, degrees: yaw 0
// straight ahead and growing to the left, pitch positive upward. False when
// the controller isn't being tracked. Rotation only, which is all 3DOF aiming
// needs; the hand's position is ignored.
bool XrInput_GetGunAim(float* yawDeg, float* pitchDeg);

// A whole hand, for arm IK: where the controller is and which way it is
// turned, in the runtime's raw tracking space. Rows of rot are the
// controller's right, up and forward axes (forward is OpenXR's -Z), the same
// convention VRBridge_GetTrackingFrame publishes its reference in, so an
// offset taken between them can be read off one against the other. False
// unless the hand answered in the last half second.
// Buzz a controller. hand 0 is left, 1 is right; amplitude 0..1; seconds is how
// long it runs. Safe from any thread, and does nothing at all when there is no
// VR session, so callers need not check.
void XrInput_Pulse(int hand, float amplitude, float seconds);

struct XrHandPose {
    bool tracked = false;
    float posMeters[3] = {};
    float rot[9] = {};
    unsigned long long ms = 0;
};
bool XrInput_GetHandPose(int hand /* 0 left, 1 right */, XrHandPose& out);

// OpenXR-facing half. Only visible to code that has already included
// openxr.h, so the menu and the input block don't have to.
#if defined(XR_VERSION_1_0)
// Actions must exist before the session does, so this is called as soon as
// the instance is up.
void XrInput_OnInstanceCreated(XrInstance instance);
// Once per session, before the first frame. Attaching is final: no action can
// be created or changed afterwards.
void XrInput_AttachToSession(XrSession session);
// Every frame from the submit thread, after xrWaitFrame.
void XrInput_Sync(XrSession session, XrSpace baseSpace, XrTime displayTime);
void XrInput_OnSessionEnding();
void XrInput_OnInstanceDestroyed();
#endif
