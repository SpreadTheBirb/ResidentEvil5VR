# Resident Evil 5 VR

A native stereoscopic VR mod for **Resident Evil 5** (`re5dx9.exe`, the 32-bit
DirectX 9 build), driven through OpenXR.

RE5 renders through a `d3d9.dll` proxy that this project supplies. The game's
D3D9 calls are translated to Direct3D 12 by dgVoodoo2, a dgVoodoo2 addon plugin
grabs each finished frame straight off the D3D12 swapchain, and the frame is
handed to an OpenXR runtime as a pair of eye textures. The stereo image itself
is produced by drawing the scene twice per frame into one backbuffer with a
per-eye camera and scissor rectangle.

> **Status:** working and playable, still under active development. Every
> option lives in an in-game menu (**Insert**, or click both sticks on a
> controller). Install steps are in the README inside each release zip.

Quick note, yes, this is a vibecoded VR mod. Just a lot of debugging and steering Claude on my end. This was one of my favorite games to play growing up and I want to experience it in VR. I've tried to make sure that all of the first person features translate to flat screen as well, so it's totally playable in first person, with no weird camera pop out as others have done in the past. My plan is to get 6dof working, or at a minimum 3dof. This hasn't been tested online. For everyone to also be aware out the gate, my solution of using dgVoodoo2 to convert RE5 from d3d9 to d3d12 has some potential risks. I discovered that it existed through the recent DLSS5 modders, and it can flag as potential malware/trojan through your antivirus. Per the author, and various other users as I've read around, this is a false positive..but I'm throwing this out there so that it's clear from the gate. 

There is no hud in VR at the moment...so it's a little tricky, but I hope to have that corrected soon. Right now my focus is pure function and playing through the first area once my pistol is equipped. 

## Features

- **Native stereoscopic VR through OpenXR** — each eye rendered with its own
  camera, real head tracking and the headset's actual per-eye lens FOV,
  delivered at the headset's full refresh rate.
- **In-game menu** — press **Insert**, or click both sticks in on a
  controller. First person, VR on/off, eye position, world scale, HUD
  distance, colour filter and more, all saved between sessions, plus a Status
  page showing live values (frame rate, per-eye resolution, headset refresh,
  whether the 4GB patch is applied). Works with mouse, keyboard or controller,
  and in the headset.
- **HUD in VR** — ammo, health, the inventory, the pause screen and most menus
  appear on a panel in front of you.
- **True first person** — the camera sits in Chris's head, throughout
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
- **Head tracking turns the game camera** — so nothing disappears over your
  shoulder, and it steps aside the moment you aim so the gun still fires down
  the laser.
- **Head comes back for action cameras** — when the game swings its camera out
  for a kick, a vault or a grab, you don't see a headless Chris.
- **Reset view** — from the menu, or hold both sticks in for a second.
- **Drop-in install** — no game files are modified; delete the files to
  uninstall.

**Next up:** full resolution per eye (each eye currently gets half the game's
frame), a proper single-eye view on the desktop for people streaming the game
(the monitor currently shows the side-by-side image), keeping the camera
inside Chris during actions, and motion controllers with arm IK.

---

## Getting the mod 

Grab the newest **[Release](../../releases)** — it contains the built DLLs and
everything needed to run, so there is nothing to compile. Follow the install
steps in the release notes.

You do **not** need to clone this repository to play.

### Do this first: the 4GB patch

**Strongly recommended, with or without VR.** RE5 is 32-bit and capped at 2 GB
of address space no matter how much memory or VRAM you have. Raising that cap
to 4 GB made stutters disappear outright - on a 5090 flat screen, and on a
laptop in VR. It is not extra VRAM; it is headroom the game runs out of long
before your graphics card does.

Get [the 4GB patch](https://ntcore.com/4gb-patch/), run it, and point it at
`...\Steam\steamapps\common\Resident Evil 5\re5dx9.exe`. It keeps the original
alongside as `re5dx9.exe.Bak`. Steam verifying or updating the game restores
the unpatched exe, so re-run it if stutters return.

## Repository layout

```
RE5VR/
  src/
    proxy/        d3d9.dll proxy - export table hand-matched to what RE5 probes
    hooks/        MinHook-based game hooks (camera rig, constant probes, ...)
    render/       stereo_test.cpp - per-eye camera, projection, scissor split
                  hud_shaders.cpp - recognises the HUD's shaders for VR
    ui/           menu.cpp - the in-game Dear ImGui menu and re5vr.ini
                  input_block.cpp - keeps the game's input away while it is open
    vr/           openxr_bridge.cpp   - OpenXR session, swapchain, submit
                  d3d12_addon_bridge.cpp - consumer side of the addon handoff
    util/         logging
  addon/          dgVoodoo2 addon plugin - builds to SampleAddon.dll, the
                  producer side that captures the D3D12 backbuffer
  thirdparty/     minhook, Dear ImGui, OpenXR SDK, dgVoodoo2 addon API headers
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
| [Dear ImGui](https://github.com/ocornut/imgui) | `RE5VR/thirdparty/imgui` | v1.91.9, core plus the DX9 and Win32 backends (MIT) |
| [OpenXR SDK](https://github.com/KhronosGroup/OpenXR-SDK) | `RE5VR/thirdparty/openxr` | vendored at `288d3a7` (SDK 1.0.34), including the prebuilt Win32 loader |
| dgVoodoo2 addon API | `RE5VR/thirdparty/dgvoodooapi` | headers only |
| [OpenVR SDK](https://github.com/ValveSoftware/openvr) | *not included* | see below |

**OpenVR is deliberately not in this repository.** An earlier phase of the
project targeted SteamVR through OpenVR; that path was replaced by OpenXR and
`RE5VR.vcxproj` no longer compiles or links any of it. The SDK is ~495 MB,
almost all of it Unity/Qt/OpenCV sample binaries. `src/vr/openvr_bridge.cpp`
is kept in the tree for reference but is not part of the build. If that path is
ever revived, re-clone the SDK into `RE5VR/thirdparty/openvr/`.
