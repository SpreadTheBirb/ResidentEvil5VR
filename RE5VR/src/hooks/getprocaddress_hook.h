#pragma once

// Hooks kernel32!GetProcAddress so that any caller resolving
// "Direct3DCreate9" or "Direct3DCreate9Ex" - against *any* module handle,
// not just this proxy DLL - gets our intercepting implementation instead of
// the genuine one.
//
// Why this exists: re5dx9.exe reaches its (Direct3D9-rendered) main menu
// without ever calling this proxy's own Direct3DCreate9 export. Its static
// import table only references D3DPERF_GetStatus from "d3d9.dll" by name;
// Direct3DCreate9 itself is resolved dynamically, and evidently against a
// separately, explicitly-loaded genuine d3d9.dll module (e.g. via a full
// System32 path) rather than through ordinary DLL search order - which is
// exactly what would let a same-folder proxy DLL get bypassed. Hooking
// GetProcAddress itself makes the interception independent of however the
// game located the module.
void Hooks_InstallGetProcAddressHook();
