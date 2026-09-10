#pragma once

#include <windows.h>

// Resolves function pointers for every real d3d9.dll export we don't
// otherwise intercept, so the naked jmp-trampoline stubs below have
// somewhere to jump. Call once, after the real d3d9.dll is loaded.
void ResolveBlindForwards(HMODULE hRealD3D9);
