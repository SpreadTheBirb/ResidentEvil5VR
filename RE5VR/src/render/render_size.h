#pragma once

#include <d3d9.h>

// ---- Full resolution per eye (2026-09-14) ----------------------------------
// Stereo draws both eyes side by side into one frame, so each eye used to get
// half of whatever resolution the player picked - 640x720 at 1280x720, and a
// wide shape the headset squashed. While VR is on, this makes RE5 itself
// render at twice the runtime's recommended eye width by its eye height, and
// puts the player's own resolution back when VR turns off. The runtime owns
// the scaling (SteamVR's resolution setting, Virtual Desktop's quality); the
// only limit applied here is a memory cap. See render_size.cpp for how.

struct RenderSizeSettings {
    bool fullResPerEye = true; // off: each eye is half the game's own frame, as before
};

RenderSizeSettings RenderSize_GetSettings();
void RenderSize_ApplySettings(const RenderSizeSettings& s);

// The per-eye size VR will use for the runtime's recommendation: the
// recommendation itself, shrunk (keeping its shape) if it would run RE5 out
// of memory. Returns true if it was shrunk.
bool RenderSize_EyeSizeFor(UINT recommendedEyeWidth, UINT recommendedEyeHeight, UINT* outEyeWidth,
    UINT* outEyeHeight);

// Render thread, every frame while VR is on and the runtime's recommendation
// is known. Returns true once the frame is at the size VR should use (or the
// feature is off, unavailable or gave up), i.e. when it's safe to build or
// feed the headset images. False means RE5 is still switching; try next frame.
bool RenderSize_EnterVR(IDirect3DDevice9* device, UINT recommendedEyeWidth, UINT recommendedEyeHeight);

// Render thread, when VR turns off: the player's own resolution comes back.
void RenderSize_ExitVR();

// Present hook: keeps the VR size if RE5 changes its resolution by itself.
void RenderSize_OnPresent(IDirect3DDevice9* device);

// Reset hook, before the real Reset: a VR-sized frame can't be exclusive
// fullscreen, so it is switched to windowed.
void RenderSize_OnBeforeReset(D3DPRESENT_PARAMETERS* pp);

// Reset hook, after the real Reset.
void RenderSize_OnReset(HRESULT hr);

struct RenderSizeStatus {
    bool available = false;  // this exe matches what the patch was made for
    bool active = false;     // RE5 is rendering at the VR size right now
    bool pending = false;    // asked, waiting for RE5 to switch
    bool failed = false;     // RE5 refused the size this session
    UINT vrWidth = 0, vrHeight = 0;     // whole frame at the VR size
    UINT gameWidth = 0, gameHeight = 0; // the player's own resolution
    bool capped = false;     // the runtime's size was shrunk to fit RE5's memory
};
void RenderSize_GetStatus(RenderSizeStatus& out);
