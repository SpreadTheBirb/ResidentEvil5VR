# Resident Evil 5 VR

A native stereoscopic VR mod for **Resident Evil 5** (`re5dx9.exe`, the 32-bit
DirectX 9 build), driven through OpenXR — and a true first-person mode that
works just as well on a normal monitor.

> **Status:** v0.4.0, working and playable, still under active development.
> There are no hotkeys: everything is in the in-game menu — press **Insert**,
> or click both sticks in on a controller.

Quick note, yes, this is a vibecoded VR mod. Just a lot of debugging and steering Claude on my end. This was one of my favorite games to play growing up and I want to experience it in VR. I've tried to make sure that all of the first person features translate to flat screen as well, so it's totally playable in first person, with no weird camera pop out as others have done in the past. My plan is to get 6dof working, or at a minimum 3dof. This hasn't been tested online. For everyone to also be aware out the gate, my solution of using dgVoodoo2 to convert RE5 from d3d9 to d3d12 has some potential risks. I discovered that it existed through the recent DLSS5 modders, and it can flag as potential malware/trojan through your antivirus. Per the author, and various other users as I've read around, this is a false positive..but I'm throwing this out there so that it's clear from the gate. 

> **Update (v0.4.0):** the HUD now works in VR — ammo, health, the inventory
> and the pause screen all show on a panel in front of you.

---

## Contents

- [Getting the mod](#getting-the-mod)
- [The mod menu](#the-mod-menu)
- [Features](#features)
- [Playing in VR](#playing-in-vr)
- [Performance](#performance)
- [Known issues](#known-issues) · [Reporting a problem](#reporting-a-problem)
- [Next up](#next-up)
- [How it works](#how-it-works) · [Repository layout](#repository-layout) · [Building from source](#building-from-source) · [Third-party components](#third-party-components)

---

## Getting the mod

Grab the newest **[Release](../../releases)** — it contains the built DLLs and
everything needed to run, so there is nothing to compile. You do **not** need
to clone this repository to play.

### Which download?

| Download | For | Files |
|---|---|---|
| `RE5VR-v0.4.0-VR.zip` | Windows with a headset. Plays flat-screen too, so take this one if you have VR at all. | `d3d9.dll`, `d3d9_dgvoodoo.dll`, `dgVoodoo.conf`, `openxr_loader.dll`, `SampleAddon.dll` |
| `RE5VR-v0.4.0-flatscreen.zip` | No headset, or **Linux / Steam Deck via Proton** (the only version that works there). No dgVoodoo2, so the antivirus note doesn't apply. | `d3d9.dll`, `openxr_loader.dll` |

### Do this first: the 4GB patch

**Strongly recommended, with or without VR.** RE5 is 32-bit and capped at 2 GB
of address space no matter how much memory or VRAM you have. Raising that cap
to 4 GB made stutters disappear outright — on a 5090 flat screen, and on a
laptop in VR. It is not extra VRAM; it is headroom the game runs out of long
before your graphics card does.

Get [the 4GB patch](https://ntcore.com/4gb-patch/), run it, and point it at
`...\Steam\steamapps\common\Resident Evil 5\re5dx9.exe`. It keeps the original
alongside as `re5dx9.exe.Bak`. Steam verifying or updating the game restores
the unpatched exe, so re-run it if stutters return. The menu's **Status** page
tells you whether it's applied.

### Install

1. Close the game.
2. Copy every file from the zip into the game folder, next to `re5dx9.exe`:
   `...\Steam\steamapps\common\Resident Evil 5\`
3. Launch through Steam as normal. For the first 10 seconds a small note in
   the corner reminds you how to open the menu.

**Uninstall:** delete those files (and `re5vr.ini` if you want your settings
gone too). No game files or saves are modified.

**Updating from an older version:** overwrite the files. The old hotkeys are
gone — everything they did is in the menu.

---

## The mod menu

Every option lives here, and your choices are saved to `re5vr.ini` in the game
folder, so they come back next time you play.

### Opening it

| | |
|---|---|
| **Insert** | Open / close (keyboard) |
| **Click both sticks in** | Open / close (controller) |
| **Hold both sticks in for 1 second** | Reset the VR view (without opening the menu) |

While the menu is open the game doesn't see any of your input — no stray
shots, no camera spinning, no pausing — and a button still held as you close
it stays ignored until you let go.

### Controls

- **Controller:** d-pad or left stick to move, **A** to select, **B** to
  close, **LB / RB** to switch tabs. On a slider, press **A**, then use the
  d-pad left / right.
- **Keyboard:** arrow keys to move, **Space** to select, **Esc** to close. On
  a slider, press Space then use the arrows, or **Ctrl+click** it to type a
  value.
- **Mouse:** point and click, with RE5's own cursor.

The menu works in the headset too: it's drawn into each eye in front of you.

### Camera tab

| Option | What it does |
|---|---|
| **First person camera** | Puts the camera in Chris's head. Works in VR and flat. |
| **Show head during action cameras** | When the game swings its camera out for a kick, a vault or a grab, Chris's head pops back in so you don't see a headless body. Off keeps it hidden. |
| **Field of view** | Flat screen, horizontal. 90° is the default and felt best in testing. |
| **Eye height / Eye forward** | Flat-screen camera position. 1.0 height is the skeleton's eye. |
| **Remove RE5's colour filter** | Removes the heavy yellow grade over everything (on by default). Untick for the original look. |

### VR tab

| Option | What it does |
|---|---|
| **Enable VR** | Starts VR. Greyed out on the flatscreen package. Shows whether the headset is running, or why it couldn't start. |
| **Start in VR automatically** | Turns VR on by itself a few seconds after the game launches. |
| **Reset view** | Makes wherever you're facing "forward" again (yaw only). Same as holding both sticks. |
| **Head turns the game camera** | While the gun is down, the game's camera follows your head so the world isn't culled away wherever your body faces. Raising the gun hands aim straight back to the mouse or stick. |
| **Stabilise camera** | Smooths Chris's idle-animation sway out of your view. |
| **Eye height / Eye forward** | VR camera position, separate from the flat-screen one. |
| **World scale** | Eye separation. Higher makes the world look smaller, lower makes it bigger. 2.75 was measured to make Sheva, guns and doors feel life-size. |
| **HUD distance** | How far away the HUD panel sits. 0 is closest, 100 is furthest. |

**Advanced** (collapsed by default):

| Option | What it does |
|---|---|
| **Head prediction** | 0–60 ms. Pushes the view ahead while you turn, to hide pipeline and streaming delay. Leave at 0 unless turning feels behind you, then try 15–25. |
| **Head rotation gain** | 1.0 is 1:1 with your neck. Anything else overshoots when you stop — most people should leave it. |
| **Extra FOV** | Widens the rendered field of view beyond the headset's own. |
| **Match culling to the headset's FOV** | Tells the game to draw everything the headset can see. Off, things at the edge of your view vanish. |
| **Mono post-processing** | The light-leak fix: draws bloom and light shafts once instead of per eye. |

### Status tab

Live values, useful for troubleshooting — **screenshot this page when
reporting a problem** (see [Reporting a problem](#reporting-a-problem)):

- **Mod** — release or developer build, VR or flatscreen install, address
  space used, whether the **4GB patch** is applied
- **Rendering** — game frame rate, backbuffer size, stereo on/off and the
  resolution each eye actually gets, HUD recognition
- **VR** — runtime, headset, the per-eye resolution the runtime wants versus
  what's sent, headset refresh rate, frames submitted per second, image age
- **Camera** — first person on/off, which character you're playing, camera
  hook activity, head-visibility counters

### Menu tab

Text size, whether clicking both sticks opens the menu, whether the startup
note shows, a reminder of the controls, and **Reset everything to defaults**.

---

## Features

### First person — VR and flat screen

- **True first person** — the camera sits in Chris's head throughout
  gameplay, including the unarmed intro.
- **Full look range, no pop-out** — look straight up or down without the
  camera snapping back to third person.
- **Only the head is hidden** — Chris's body and hands stay visible, so you
  see your own arms and gun.
- **Head comes back for action cameras** — kicks, vaults and grabs don't
  leave a headless Chris on screen.
- **No near-camera fade** — Chris, Sheva and NPCs stay solid up close instead
  of dissolving.
- **Walk while aiming** — something RE5 never allowed. Keep moving with your
  weapon up, on WASD or a controller's left stick (analog).
- **Laser sight always on**, including with mouse aiming.
- **Colour filter removed** by default.

### VR

- **Native stereoscopic VR through OpenXR** — each eye rendered with its own
  camera, real head tracking and the headset's actual per-eye lens FOV,
  delivered at the headset's full refresh rate.
- **HUD in VR** — ammo, health, the inventory, the pause screen and most menus
  on a panel in front of you, at a distance you choose.
- **Head tracking turns the game camera** — nothing disappears over your
  shoulder, and it steps aside the moment you aim.
- **Calibrated world scale** — measured from first person, adjustable in the
  menu.
- **VR culling fixes** — the game draws everything the headset can see,
  including the ground at your feet and your partner's legs.
- **Reset view** — from the menu, or hold both sticks in for a second.
- **No light leaks** — post-processing is drawn once rather than split per eye.

### Quality of life

- **In-game menu** with saved settings, working with mouse, keyboard or
  controller, on the monitor or in the headset.
- **Live Status page**, including a 4GB-patch check.
- **Drop-in install** — no game files are modified; delete the files to
  uninstall.

---

## Playing in VR

1. **Use SteamVR or Virtual Desktop as your OpenXR runtime.** Meta's own runtime closes the game
   the moment VR starts. The same headset works fine through either of the
   other two:
   - **SteamVR** (Link cable, Air Link, or a PC headset): *SteamVR → Settings
     → OpenXR → Set SteamVR as OpenXR Runtime*.
   - **Virtual Desktop** (wireless Quest): in the headset's Virtual Desktop
     app, set *Settings → Streaming → OpenXR Runtime* to **VDXR**. Virtual
     Desktop registers it as the active runtime while the streamer is
     connected.
2. Start SteamVR, or connect Virtual Desktop, and put the headset on.
3. Launch the game, open the menu and tick **Enable VR** on the VR tab (or
   tick **Start in VR automatically** once and never think about it again).
4. Turn on **First person camera** on the Camera tab.

A controller is recommended over mouse and keyboard.

---

## Performance

- **VR renders the scene twice**, once per eye, so it costs about double a
  flat-screen frame. If your frame rate is low, lower the game's own
  resolution first — that is by far the biggest lever, and it beats any
  headset-side render scale. If you have headroom, raise it.
- **Stutter is a separate problem** from frame rate, and the 4GB patch is the
  fix for it.
- **Streaming over Wi-Fi** (Virtual Desktop, Air Link) adds its own delay and
  frame cap on top; a wired link or LAN is noticeably better.

---

## Known issues

- **Each eye gets half the game's resolution**, because both eyes share one
  side-by-side frame. That's why SteamVR's resolution slider does nothing.
  Raising the game's resolution is the only lever for now, and very high
  resolutions can run the game out of memory.
- **The desktop window shows the side-by-side image**, so it isn't suitable
  for streaming or recording yet.
- Some menus (the title screen, parts of the Organize screen) still don't
  draw correctly in VR.
- Look straight down and you're inside your own torso.
- When the head comes back for an action camera, it can shrink away rather
  than vanish instantly as the camera returns.
- Split-screen co-op: Chris's head stays hidden. **Online co-op is untested.**

### Reporting a problem

Please include these with any bug report or issue:

1. **A screenshot of the menu's Status tab** — open the menu (Insert, or click
   both sticks), switch to **Status**, and screenshot it, ideally while the
   problem is happening. It shows your frame rate, resolution per eye,
   headset and runtime, refresh rate, whether the 4GB patch is applied, and
   more, in one picture.
2. **The log files** from the game folder
   (`...\Steam\steamapps\common\Resident Evil 5\`):
   - `re5vr.log` — always
   - `re5vr_addon.log` — as well, if you were playing in VR

   `re5vr.log` is **replaced every time the game starts**, so copy it before
   launching again, or the session with the problem is gone.

Say roughly when it happened too (the logs have timestamps), and which
download you're using.

---

## Next up

- **Full resolution per eye** instead of half the game's frame.
- **A VR view on the desktop** — a proper single-eye mirror, so streaming and
  recording show something watchable.
- Keeping the camera inside Chris during actions.
- Motion controllers with arm IK.

---

## How it works

RE5 renders through a `d3d9.dll` proxy that this project supplies. The game's
D3D9 calls are translated to Direct3D 12 by dgVoodoo2, a dgVoodoo2 addon plugin
grabs each finished frame straight off the D3D12 swapchain, and the frame is
handed to an OpenXR runtime as a pair of eye textures. The stereo image itself
is produced by drawing the scene twice per frame into one backbuffer with a
per-eye camera and scissor rectangle. HUD draws are recognised by their shader
bytecode and drawn once per eye onto a panel at a fixed convergence.

The menu is Dear ImGui drawn over the game's own D3D9 device. While it is open,
the game's input is hidden at every route RE5 uses — DirectInput keyboard and
mouse, XInput, the Windows cursor and key state, and its message pump — with
the Win32 routes blocked only for calls made from the game's own code.

## Repository layout

```
RE5VR/
  src/
    proxy/        d3d9.dll proxy - export table hand-matched to what RE5 probes
    hooks/        MinHook-based game hooks (camera rig, fade, laser, filter, ...)
    render/       stereo_test.cpp - per-eye camera, projection, scissor split
                  hud_shaders.cpp - recognises the HUD's shaders for VR
    ui/           menu.cpp - the in-game Dear ImGui menu and re5vr.ini
                  input_block.cpp - keeps the game's input away while it is open
    vr/           openxr_bridge.cpp   - OpenXR session, swapchain, submit
                  d3d12_addon_bridge.cpp - consumer side of the addon handoff
    util/         logging, build switches
  addon/          dgVoodoo2 addon plugin - builds to SampleAddon.dll, the
                  producer side that captures the D3D12 backbuffer
  thirdparty/     minhook, Dear ImGui, OpenXR SDK, dgVoodoo2 addon API headers
  RE5VR.sln       Visual Studio solution (Release|Win32 - 32-bit only)
```

Two DLLs are built and both are required for VR: `d3d9.dll` (the proxy, from
`RE5VR.sln`) and `SampleAddon.dll` (the addon, from
`addon/RE5VR_DgVoodooAddon.vcxproj`). The addon's filename is fixed by
dgVoodoo2 and cannot be changed.

## Building from source

Requires Visual Studio with the C++ desktop workload and the Windows SDK. Build
both projects as **Release | Win32** — the game is 32-bit, so a 64-bit build
will not load.

`RE5VR/src/util/build_config.h` has one switch, `RE5VR_DIAGNOSTICS`. Leave it
at `0` for anything you share; `1` compiles in the developer probes and adds a
Developer tab to the menu.

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
