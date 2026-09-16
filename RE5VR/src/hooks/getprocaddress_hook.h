#pragma once

// Hooks kernel32!GetProcAddress so that the game resolving
// "Direct3DCreate9" or "Direct3DCreate9Ex" - against *any* module handle,
// not just this proxy DLL - gets our intercepting implementation instead of
// the genuine one.
//
// Only lookups made from inside re5dx9.exe are redirected (2026-09-15).
// Handing our exports to anything else was an instant crash on one tester's
// PC: something else in their process (an overlay or another d3d9 wrapper)
// looked up Direct3DCreate9Ex while the game's own Direct3DCreate9 call was
// still inside dgVoodoo2 starting D3D12 up, and the second entry into
// dgVoodoo2's creation read a null pointer (d3d9_dgvoodoo.dll+B211A). Other
// callers now get the genuine function, which is what they asked for anyway.
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
