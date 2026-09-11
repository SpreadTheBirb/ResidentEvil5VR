#pragma once

#include <d3d9.h>

// Does the game use hardware occlusion queries (occlusion culling)? Counts
// IDirect3DDevice9::CreateQuery calls by query type. See query_probe.cpp.
void QueryProbe_Install(IDirect3DDevice9* pDevice);

// Logs the per-type totals whenever they change (checked every 10 s).
void QueryProbe_OnEndScene();
