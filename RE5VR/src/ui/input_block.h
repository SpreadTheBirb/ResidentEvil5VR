#pragma once

#include <windows.h>
#include <Xinput.h>

// Keeps the game's hands off the keyboard, mouse and gamepad while the in-game
// menu is open. RE5 reads keyboard and mouse through DirectInput 8 and the pad
// through xinput1_3.dll (both confirmed from the exe's imports, 2026-09-13), so
// swallowing window messages alone would do nothing: these hook
// IDirectInputDevice8::GetDeviceState/GetDeviceData and XInputGetState and hand
// the game an idle device while blocking.
//
// Two details that matter in play:
//  * Whatever was held at the moment the menu CLOSED (the A that pressed
//    "Close", Escape, the mouse button) stays hidden from the game until it is
//    released, so closing the menu never fires a jump, a shot or a pause.
//  * While blocking, the game's own mouse deltas are collected instead of
//    dropped - with an exclusive DirectInput mouse Windows moves no cursor, so
//    this is the only way the menu can have a working pointer.

// Call once the game's code is running (Hooks_OnDeviceCreated). Idempotent.
void InputBlock_Install();

void InputBlock_SetBlocking(bool blocking);
bool InputBlock_IsBlocking();

// The first connected pad, read past the block. False if no pad.
bool InputBlock_ReadPad(XINPUT_STATE* out);

// Mouse movement the game polled while blocking, since the last call.
// fromDirectInput is false when the game has not read its DirectInput mouse
// recently - the caller should fall back to the Windows cursor then.
struct InputBlockMouse {
    long dx = 0, dy = 0, wheel = 0;
    bool buttons[3] = { false, false, false };
    bool fromDirectInput = false;
};
void InputBlock_TakeMouse(InputBlockMouse& out);

// Keyboard and mouse messages the game pulls off its message queue while
// blocking are handed here instead (on the game's window thread).
void InputBlock_SetMessageSink(void (*sink)(UINT msg, WPARAM w, LPARAM l));

// One line describing the game's raw mouse reads since the last call (debug).
void InputBlock_DescribeMouse(char* out, size_t size);

// The pointer graphic the game last set with SetCursor, or null if none yet.
HCURSOR InputBlock_GameCursor();
