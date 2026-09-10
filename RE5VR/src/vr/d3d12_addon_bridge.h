// Consumer half of RE5VR's dgVoodoo2 D3D12 "Addon" interop bridge - see
// addon/src/addon_main.cpp (the producer half, built as SampleAddon.dll)
// for the full design rationale. In short: when dgVoodoo2 is chained in
// with its D3D12 backend forced (dgVoodoo.conf's OutputAPI) and our
// SampleAddon.dll addon is present, this lets RE5VR read the real
// rendered frame directly as a D3D11 texture (opened from a D3D12
// shared handle) with zero CPU readback and none of the D3D9 shared-
// surface machinery that dgVoodoo2's D3D9 façade rejects outright
// (D3DERR_INVALIDCALL, see re5vr_project memory). This is an ADDITIVE,
// optional path - if the addon isn't present/active, callers should
// fall back to the existing D3D9 CPU-readback bridge unchanged.

#pragma once

#include <d3d11.h>
#include <dxgiformat.h>
#include <windows.h>

struct D3D12AddonBridgeInfo
{
    UINT width = 0;
    UINT height = 0;
    DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
};

// Call once, after the D3D11 device used for the OpenXR graphics binding
// exists (see CreateXrSessionAndSwapchains) and before deciding the
// OpenXR swapchain's own format - outInfo's format is what the caller
// should request a matching (ideally _SRGB) swapchain format for.
// Polls briefly (a few hundred ms) for the addon's first real frame if
// it's loaded but hasn't produced one yet; returns false immediately,
// without polling, if SampleAddon.dll isn't loaded in this process at
// all (i.e. the plain D3D9 path is in use, or dgVoodoo didn't pick it
// up for some other reason).
bool D3D12AddonBridge_TryInit(ID3D11Device* d3d11Device, D3D12AddonBridgeInfo* outInfo);

// True once TryInit has succeeded. Cheap to call every frame.
bool D3D12AddonBridge_IsActive();

// Crops the current full-frame texture's left/right halves (same
// side-by-side stereo layout the old D3D9 backbuffer used) directly
// into the caller's existing per-eye D3D11 textures via
// CopySubresourceRegion - no CPU involved at all. eyeWidth/eyeHeight
// must match what TryInit reported (width/2, height). Returns false if
// the addon hasn't published a frame yet (front slot still unset) -
// treat that the same as "nothing new to copy this call", not an error.
bool D3D12AddonBridge_CopyToEyeSlots(ID3D11DeviceContext* d3d11Context,
    ID3D11Texture2D* leftDst, ID3D11Texture2D* rightDst, UINT eyeWidth, UINT eyeHeight);

// Releases the opened D3D11 textures/module reference. Safe to call
// even if TryInit was never called or failed.
void D3D12AddonBridge_Shutdown();
