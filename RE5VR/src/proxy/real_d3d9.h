#pragma once

#include <d3d9.h>

// Loads the genuine System32/SysWOW64 d3d9.dll by explicit full path (never
// the bare name "d3d9.dll", to avoid recursively loading this proxy DLL) and
// resolves the real Direct3DCreate9Ex entry point. Safe to call more than
// once; subsequent calls are no-ops.
bool RealD3D9_Init();

typedef HRESULT(WINAPI* PFN_Direct3DCreate9Ex)(UINT SDKVersion, IDirect3D9Ex** ppD3D);

// The genuine entry point, resolved dynamically. Valid only after
// RealD3D9_Init() has succeeded.
extern PFN_Direct3DCreate9Ex g_RealDirect3DCreate9Ex;

// True if we chained to dgVoodoo2, false if we fell back to the system
// d3d9.dll. VR needs dgVoodoo2's D3D12 frames, so a false here means this
// install is flat-screen only - the Linux/Proton route, and the flat-screen
// download. The VR hotkey uses this to refuse rather than half-start.
bool RealD3D9_UsingDgVoodoo();

// Every module in the process right now, by name, in chunks that fit a log
// line. Worth taking more than once: what is loaded at the main menu and what
// is loaded once the game is online are not the same list.
void LogLoadedModules(const char* why);

// Only what has appeared since the previous call. The first call learns the
// baseline and prints nothing. Meant to be polled, so that a module the game
// loads on going online is named at the moment it loads.
void LogNewModules();

// Our intercepting implementations (see real_d3d9.cpp), exported from this
// DLL under their real names but also referenced directly by address from
// hooks/getprocaddress_hook.cpp, which substitutes them whenever the game
// resolves "Direct3DCreate9"/"Direct3DCreate9Ex" via GetProcAddress against
// *any* module - not just this one.
extern "C" IDirect3D9* WINAPI Direct3DCreate9(UINT SDKVersion);
extern "C" HRESULT WINAPI Direct3DCreate9Ex(UINT SDKVersion, IDirect3D9Ex** ppD3D);
