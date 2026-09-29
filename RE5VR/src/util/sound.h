#pragma once

// A noise of our own (2026-09-26, user: "is it possible to play a sound effect
// when you insert the magazine? ... some type of audible feedback would be
// nice").
//
// Manual reloading took a reload apart, and the sound came apart with it. The
// game's reload noise is one clip covering eject, insert and rack, and it is
// welded to the reload the game performs - so the insert kept its sound,
// because that is where we hand the reload over, and the eject lost its
// entirely. A magazine that leaves a weapon in silence feels like nothing
// happened.
//
// These are ordinary WAV files sitting next to the game, not resources baked
// into the dll. Three reasons: they can be changed without a build, a player
// can use whatever they like, and the mod ships no audio it does not own.
//
// Nothing here is required. A missing file is looked for once and then never
// again, and the feature simply makes no sound.

// Plays <game folder>\re5vr_<name>.wav, loading and caching it on first use.
// Safe to call from anywhere and at any rate; it never blocks and never waits
// on the disk twice for the same name.
void Sound_Play(const char* name);

// Forgets what it has loaded, so a file replaced while the game is running is
// picked up. Called when the menu's sound setting is switched on.
void Sound_Forget();

// How loud, 0 to 1 (2026-09-26, user: "I feel like they should be a bit
// quieter in game - ideally they'd work off of the in game sfx slider - but we
// don't have that").
//
// Quite right that this should follow the game's own effects volume, and it
// cannot: these do not go through RE5's mixer at all, they go to Windows
// beside it. Nothing in the game's settings can reach them, so the mod has to
// carry its own.
//
// PlaySound has no volume of its own either, so this is done by scaling the
// samples. The file as loaded is kept untouched and a scaled copy is built for
// playing, so the slider can be moved as often as you like without the sound
// degrading a little more each time.
void Sound_SetVolume(float zeroToOne);
