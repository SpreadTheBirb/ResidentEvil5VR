// Minimal file logger for the dgVoodoo2 addon DLL (SampleAddon.dll).
// Deliberately separate from RE5VR.d3d9.dll's own logger (src/util/log.h)
// since this is a different DLL, loaded and driven by dgVoodoo2 itself,
// not by RE5VR's proxy - keeping output in its own file (re5vr_addon.log)
// makes it trivial to tell which module produced which lines.
// Same persistent-handle + fflush-per-write design as log.cpp, learned
// the hard way there: a fopen/fclose per call serialized behind one
// mutex caused multi-second stalls under load (see re5vr_project memory,
// Phase 6). Present callbacks fire every frame, so this matters here too.

#pragma once

void AddonLog_Init();
void AddonLog_Printf(const char* fmt, ...);
