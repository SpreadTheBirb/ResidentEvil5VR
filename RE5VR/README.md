# RE5VR (source)

Developer notes for the mod itself. If you just want to play, take the zip
from [Releases](../../releases) - it contains everything built.

## Build

Visual Studio with the C++ desktop workload, **Release | Win32** (the game is
32-bit, a 64-bit build will not load). Two projects:

```
RE5VR.vcxproj                      -> build\Release\d3d9.dll     (the mod)
addon\RE5VR_DgVoodooAddon.vcxproj  -> addon\build\Release\SampleAddon.dll
```

`SampleAddon.dll` is the dgVoodoo2 addon that hands each finished Direct3D 12
frame to the VR bridge; its filename is fixed by dgVoodoo2. Both DLLs go in
the game folder next to `re5dx9.exe`, along with `d3d9_dgvoodoo.dll`,
`dgVoodoo.conf` and `openxr_loader.dll` (see any release zip for the full
set).

## How it loads

`d3d9.dll` is a proxy: it forwards every real d3d9 export, and at load time
resolves `d3d9_dgvoodoo.dll` from its own folder. RE5 never calls the proxy's
`Direct3DCreate9` on Windows, so the device hook is installed from a
throwaway `IDirect3D9Ex` created at load (never released - see
`proxy/real_d3d9.cpp` for why) and from a `GetProcAddress` hook.

If `d3d9_dgvoodoo.dll` is missing, the proxy falls back to the system
`d3d9.dll` and hooks its create functions directly. That is the Linux/Proton
route: DXVK instead of dgVoodoo2, flat screen only, no VR.

## Layout

```
src/
  proxy/    d3d9.dll proxy - export table hand-matched to what RE5 probes
  hooks/    MinHook-based game and D3D9 hooks (camera rig, laser, fade,
            culling, and the diagnostics below)
  render/   stereo_test.cpp - per-eye camera, projection, scissor split
  vr/       openxr_bridge.cpp      - OpenXR session, swapchain, submit
            d3d12_addon_bridge.cpp - consumer side of the addon handoff
  util/     logging
addon/      dgVoodoo2 addon plugin (producer side of the frame handoff)
thirdparty/ minhook, OpenXR SDK, dgVoodoo2 addon API headers
```

## Hotkeys

Shipped: **F4** first person, **F7** VR, **F8** stereo, **`** wide culling FOV
(VR), **,** / **.** eye back/forward, **Shift** with those for height,
**Ctrl** with those for flat FOV, **[** / **]** / **-** world scale, **;** /
**'** VR FOV widen.

## Diagnostics

Everything else is a developer tool and is compiled out: set
`RE5VR_DIAGNOSTICS` to 1 in `src/hooks/d3d9_hooks.cpp` to get back the F5
hardware-watchpoint finder (`hooks/boom_finder.cpp`), the `=` game-state
capture (`hooks/state_probe.cpp`, diffed with a state-diff script), the
skeleton dump, the vertex/pixel constant probes, the head-hide draw browser,
and the blinking debug quad.

The game writes `re5vr.log` and `re5vr_addon.log` into its own folder; both
are worth reading before guessing.

## Third-party

| Component | Where | Notes |
|---|---|---|
| [MinHook](https://github.com/TsudaKageyu/minhook) | `thirdparty/minhook` | vendored at `d94c64d` |
| [OpenXR SDK](https://github.com/KhronosGroup/OpenXR-SDK) | `thirdparty/openxr` | vendored at `288d3a7` (SDK 1.0.34), with the prebuilt Win32 loader |
| dgVoodoo2 addon API | `thirdparty/dgvoodooapi` | headers only |
| [OpenVR SDK](https://github.com/ValveSoftware/openvr) | *not included* | the OpenVR path was replaced by OpenXR; `src/vr/openvr_bridge.cpp` is kept for reference and is not built |
