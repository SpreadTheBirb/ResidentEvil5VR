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
//   left X / Y          -> X / Y          reload, partner command
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
};

void XrInput_SetSettings(const XrInputSettings& s);
XrInputSettings XrInput_GetSettings();

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
