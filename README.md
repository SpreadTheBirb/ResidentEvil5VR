# Resident Evil 5 VR

A native stereoscopic VR mod for **Resident Evil 5** (`re5dx9.exe`, the 32-bit
DirectX 9 build), driven through OpenXR.

RE5 renders through a `d3d9.dll` proxy that this project supplies. The game's
D3D9 calls are translated to Direct3D 12 by dgVoodoo2, a dgVoodoo2 addon plugin
grabs each finished frame straight off the D3D12 swapchain, and the frame is
handed to an OpenXR runtime as a pair of eye textures. The stereo image itself
is produced by drawing the scene twice per frame into one backbuffer with a
per-eye camera and scissor rectangle.

> **Status:** working and playable, still under active development. The
> install steps and the full list of hotkeys are in the README inside each
> release zip.

## Features

- **Native stereoscopic VR through OpenXR** — each eye rendered with its own
  camera, real head tracking and the headset's actual per-eye lens FOV,
  delivered at the headset's full refresh rate.
- **True first person (F4)** — the camera sits in Chris's head, throughout
  gameplay. It works in VR **and on a normal monitor**, so you can play RE5 in
  first person without a headset.
- **Walk while aiming** — something RE5 never allowed. Keep moving with your
  weapon up, on WASD or a gamepad's left stick (analog).
- **Full look range, no pop-out** — look straight up or down without the
  camera snapping back to third person.
- **No near-camera fade** — Chris and Sheva stay solid up close instead of
  dissolving.
- **Only the head is hidden** — Chris's body and hands stay visible, ready for
  arm IK.
- **Calibrated world scale** — measured from first person, adjustable live.
- **VR culling fixes** — the game draws everything in front of you, including
  the ground at your feet, not just what its own narrower camera would see.
- **Drop-in install** — no game files are modified; delete the files to
  uninstall.

**Next up:** the game camera turning with your head (so nothing disappears
over your shoulder), the HUD in VR, and motion controllers with arm IK.

---

## Getting the mod (most people want this)

Grab the newest **[Release](../../releases)** — it contains the built DLLs and
everything needed to run, so there is nothing to compile. Follow the install
steps in the release notes.

You do **not** need to clone this repository to play.

## Repository layout

```
RE5VR/
  src/
    proxy/        d3d9.dll proxy - export table hand-matched to what RE5 probes
    hooks/        MinHook-based game hooks (camera rig, constant probes, ...)
    render/       stereo_test.cpp - per-eye camera, projection, scissor split
    vr/           openxr_bridge.cpp   - OpenXR session, swapchain, submit
                  d3d12_addon_bridge.cpp - consumer side of the addon handoff
    util/         logging
  addon/          dgVoodoo2 addon plugin - builds to SampleAddon.dll, the
                  producer side that captures the D3D12 backbuffer
  thirdparty/     minhook, OpenXR SDK, dgVoodoo2 addon API headers
  RE5VR.sln       Visual Studio solution (Release|Win32 - 32-bit only)
```

Two DLLs are built and both are required: `d3d9.dll` (the proxy, from
`RE5VR.sln`) and `SampleAddon.dll` (the addon, from
`addon/RE5VR_DgVoodooAddon.vcxproj`). The addon's filename is fixed by
dgVoodoo2 and cannot be changed.

## Building from source

Requires Visual Studio with the C++ desktop workload and the Windows SDK. Build
both projects as **Release | Win32** — the game is 32-bit, so a 64-bit build
will not load.

## Third-party components

| Component | Where | Notes |
|---|---|---|
| [MinHook](https://github.com/TsudaKageyu/minhook) | `RE5VR/thirdparty/minhook` | vendored at `d94c64d` |
| [OpenXR SDK](https://github.com/KhronosGroup/OpenXR-SDK) | `RE5VR/thirdparty/openxr` | vendored at `288d3a7` (SDK 1.0.34), including the prebuilt Win32 loader |
| dgVoodoo2 addon API | `RE5VR/thirdparty/dgvoodooapi` | headers only |
| [OpenVR SDK](https://github.com/ValveSoftware/openvr) | *not included* | see below |

**OpenVR is deliberately not in this repository.** An earlier phase of the
project targeted SteamVR through OpenVR; that path was replaced by OpenXR and
`RE5VR.vcxproj` no longer compiles or links any of it. The SDK is ~495 MB,
almost all of it Unity/Qt/OpenCV sample binaries. `src/vr/openvr_bridge.cpp`
is kept in the tree for reference but is not part of the build. If that path is
ever revived, re-clone the SDK into `RE5VR/thirdparty/openvr/`.
