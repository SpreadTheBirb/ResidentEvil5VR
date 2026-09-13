#pragma once

#include <d3d9.h>

// The in-game menu (2026-09-13): Dear ImGui over the game's own D3D9 device,
// opened with Insert or a quick click of both sticks. It replaced the player
// hotkeys (F4, F7, F9, F10, F11, '`', '/', ',' '.', '[' ']' '-', ';' '\'',
// Home/End, Page Up/Down), shows live status values, and saves everything to
// re5vr.ini next to the game.
//
// Flat, it draws straight onto the backbuffer. In VR the backbuffer is side by
// side, so the same UI is drawn once into each eye's half, placed with the
// HUD's per-eye convergence so the two copies fuse at a chosen distance.

// Call at the end of Hooks_OnDeviceCreated, after every other module is
// installed - loading re5vr.ini applies settings through them.
void Menu_Install(IDirect3DDevice9* device);

// Call from the Present hook BEFORE the real Present, so the menu is part of
// the frame that goes to the monitor and the headset.
void Menu_OnPresent(IDirect3DDevice9* device);

// True while the menu is issuing its own BeginScene/EndScene and draws. The
// EndScene hook must pass straight through then.
bool Menu_IsDrawing();

// Around IDirect3DDevice9::Reset: D3DPOOL_DEFAULT resources must go first.
void Menu_OnBeforeReset();
void Menu_OnAfterReset();
