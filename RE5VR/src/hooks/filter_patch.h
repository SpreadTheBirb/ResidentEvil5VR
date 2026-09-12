#pragma once

// RE5's colour filter (the yellow/sepia grade) on/off, toggled with F10.
// Default: on, as the game ships. See filter_patch.cpp. Call Install once the
// game's code is unpacked, and OnEndScene once a frame for the hotkey.
void FilterPatch_Install();
void FilterPatch_OnEndScene();
