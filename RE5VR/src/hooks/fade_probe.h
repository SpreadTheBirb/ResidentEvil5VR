#pragma once

#include <d3d9.h>

// Near-camera fade hunt, automated (2026-09-11). See fade_probe.cpp.
//
// '=' captures one short window of every unique draw call's full state to
// re5vr_fadecap_<n>.bin in the game folder; press it once in third person
// (Chris opaque) and once after F4 (Chris faded), then diff offline.

// Always-on shadows of the shader constant files, fed from the existing
// SetVertexShaderConstantF / SetPixelShaderConstantF hooks.
void FadeProbe_OnSetVertexShaderConstantF(UINT startRegister, const float* data, UINT vector4fCount);
void FadeProbe_OnSetPixelShaderConstantF(UINT startRegister, const float* data, UINT vector4fCount);

// Called from the DrawPrimitive / DrawIndexedPrimitive hooks, before the
// real draw. kind: 0 = DrawPrimitive, 1 = DrawIndexedPrimitive.
void FadeProbe_OnDraw(IDirect3DDevice9* pDevice, DWORD kind, D3DPRIMITIVETYPE primType, INT baseVertex,
    UINT minIndex, UINT numVertices, UINT startIndex, UINT primCount);

// Polls the '=' key and closes finished captures.
void FadeProbe_OnEndScene();
