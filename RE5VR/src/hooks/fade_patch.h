#pragma once

// Stops RE5's camera-proximity character fade while first person (F4) is on,
// without touching any other use of model visibility. See fade_patch.cpp.

// Hooks the game's model SetVisibility function. Call once, after MinHook
// is initialised.
void FadePatch_Install();

// Called on every F4 toggle.
void FadePatch_SetEnabled(bool enabled);

// Periodic summary of which callers are setting visibility (diagnostic).
void FadePatch_OnEndScene();
