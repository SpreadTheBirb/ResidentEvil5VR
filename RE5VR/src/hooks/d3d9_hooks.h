#pragma once

#include <d3d9.h>

// Called once with the first genuine IDirect3D9Ex object we obtain (whether
// the game asked for Direct3DCreate9 or Direct3DCreate9Ex). Installs a hook
// on the CreateDevice vtable slot so we can see every device the game
// creates. The device itself is left as a plain (non-Ex) device - see
// d3d9_hooks.cpp for why. Idempotent.
void Hooks_OnD3D9ExCreated(IDirect3D9Ex* pD3D9Ex);

// Called once with the first device the game creates. Installs an EndScene
// hook (frame counter + Phase 0 debug quad). Idempotent.
void Hooks_OnDeviceCreated(IDirect3DDevice9* pDevice);
