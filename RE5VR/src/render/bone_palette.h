#pragma once

// Hiding parts of your body, by where they are (2026-09-26).
//
// Three settings, in the user's own terms:
//   Full body  - the whole thing minus the head
//   Arms       - hide everything that is not elbow to hands
//   Hands      - hide the entire skeleton except the wrists up
//
// None of it touches the skeleton. Two earlier routes are ruled out and worth
// naming so they are not tried again: shrinking bones takes the camera down
// with the body, because the camera sits on the head and the head hangs off
// the spine, and the game does not put those bones back afterwards; and the
// game's own per-model visibility is all or nothing, because the character is
// a single mesh.
//
// This edits the bone table the vertex shader skins from, on its way to the
// shader. Measured off a running game: the table starts at register c23, each
// bone is three registers, and it is uploaded in batches of four bones up to
// 96 registers - 32 bones, which is the per-draw limit, so the character is
// drawn in several batches with a different subset in each.
//
// Which meant identifying bones was a dead end, and it does not matter: each
// entry carries its own world position, so a region test answers the only
// question hiding actually asks. Near your head, gone. Further up the arm than
// your elbow, gone in Arms. Not near any of your landmarks at all, left
// completely alone, which is how your co-op partner keeps their limbs.

// Sets what to hide. hideHead is separate from the mode because the head goes
// in all three - it is inside the camera.
void BonePalette_SetMode(int mode, bool hideHead);

// From the vertex constant hook, before the upload is passed on. Returns true
// when it has written a changed copy into out, which must hold count*4 floats;
// false means send the original through untouched.
bool BonePalette_Filter(unsigned startRegister, const float* data, unsigned count, float* out);

// Once a frame, for the running count in the log.
void BonePalette_OnEndScene();

// Forget the cached landmarks, for after a costume change.
void BonePalette_Relearn();

// What it did over the last window, for the menu.
bool BonePalette_Status(int& blanked, int& kept, int& theirs);
