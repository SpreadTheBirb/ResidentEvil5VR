#pragma once

#include <d3d9.h>

// Recognises the game's 2D/HUD shaders from their bytecode, from launch, so
// stereo can draw HUD, inventory, pause and most menus through the per-eye
// viewport path without anyone pressing K. See hud_shaders.cpp.

// Call once the game's device exists, before it loads its shaders.
void HudShaders_Install(IDirect3DDevice9* device);

// True if the vertex and pixel shaders currently bound are the HUD's.
bool HudShaders_IsHudDraw(IDirect3DDevice9* device);

// For the menu's Status page: live HUD shaders recognised, and all shaders seen.
void HudShaders_GetCounts(int* hudVertexShaders, int* hudPixelShaders, unsigned* shadersSeen);
