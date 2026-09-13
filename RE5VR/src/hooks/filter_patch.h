#pragma once

// RE5's colour filter (the yellow/sepia grade) on/off, from the in-game menu.
// Default: on, as the game ships. See filter_patch.cpp. Call Install once the
// game's code is unpacked, and OnEndScene once a frame for the hotkey.
void FilterPatch_Install();
void FilterPatch_OnEndScene();

// False if the game's bytes did not match (a different exe) - option greyed out.
bool FilterPatch_IsAvailable();
bool FilterPatch_IsFilterRemoved();
void FilterPatch_SetFilterRemoved(bool removed);
