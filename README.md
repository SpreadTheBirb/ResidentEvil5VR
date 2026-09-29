# Resident Evil 5 VR

A native stereoscopic VR mod for **Resident Evil 5** (`re5dx9.exe`, the 32-bit
DirectX 9 build), driven through OpenXR, plus a true first-person mode that
works just as well on a normal monitor.

> **Status:** v0.5.0a, working and playable, still under active development.
> There are no hotkeys: everything is in the in-game menu. Press **Insert**,
> or click both sticks in on a controller.

Quick note, yes, this is a vibecoded VR mod. Just a lot of debugging and steering Claude on my end. This was one of my favorite games to play growing up and I want to experience it in VR. I've tried to make sure that all of the first person features translate to flat screen as well, so it's totally playable in first person, with no weird camera pop out as others have done in the past. My plan is to get 6dof working, or at a minimum 3dof. This hasn't been tested online. For everyone to also be aware out the gate, my solution of using dgVoodoo2 to convert RE5 from d3d9 to d3d12 has some potential risks. I discovered that it existed through the recent DLSS5 modders, and it can flag as potential malware/trojan through your antivirus. Per the author, and various other users as I've read around, this is a false positive..but I'm throwing this out there so that it's clear from the gate. 

> **Update (v0.5.0):** **6DOF arms.** Your real arms drive your character's
> arms, your wrists roll, the gun goes where your hand goes, and you reload
> by reaching to your belt. **And the game stops taking your camera:** melee,
> door kicks, jumping down, vaulting and partner locate all leave your view
> where you put it. SteamVR works. Scoped weapons can stay in third person
> so they are usable in a headset.

> **Update (v0.4.0):** the HUD now works in VR. Ammo, health, the inventory
> and the pause screen all show on a panel in front of you.

> **Update (v0.4.0a):** in VR the game window shows a single eye instead of
> the side-by-side image, so streaming and recording look normal.

> **Update (v0.4.1):** full resolution per eye. VR is now crisp: each eye
> renders at the resolution your VR runtime asks for, so SteamVR's resolution
> setting and Virtual Desktop's quality presets finally do something.

> **Update (v0.4.2a):** VR motion controllers now work as a gamepad, so you
> can play without a pad in your hands (see [Motion
> controllers](#motion-controllers)). PlayStation pads and other
> non-XInput controllers can open and drive the mod menu. Fixes a crash at
> launch on PCs running another program that hooks Direct3D 9.

> **Update (v0.4.2):** no more things vanishing or popping in when you look
> over your shoulder in VR, aiming included. Look around freely while you aim;
> the gun, laser and walking stay with the mouse or stick. Changing resolution
> in exclusive fullscreen no longer crashes. The menu now shows the mod's
> version and tells you when a newer one is on Nexus.

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

Grab the newest **[Release](../../releases)**. It contains the built DLLs and
everything needed to run, so there is nothing to compile. You do **not** need
to clone this repository to play.

### Which download?

| Download | For | Files |
|---|---|---|
| `TrueFP-v0.5.0a-VR.zip` | Windows with a headset. Plays flat-screen too, so take this one if you have VR at all. | `d3d9.dll`, `d3d9_dgvoodoo.dll`, `dgVoodoo.conf`, `openxr_loader.dll`, `SampleAddon.dll` |
| `TrueFP-v0.5.0a-flatscreen.zip` | No headset, or **Linux / Steam Deck via Proton** (the only version that works there). No dgVoodoo2, so the antivirus note doesn't apply. | `d3d9.dll`, `openxr_loader.dll` |

### Do this first: the 4GB patch

**Strongly recommended, with or without VR.** RE5 is 32-bit and capped at 2 GB
of address space no matter how much memory or VRAM you have. Raising that cap
to 4 GB made stutters disappear outright, on a 5090 flat screen, and on a
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

**Updating from an older version:** overwrite every file, including
`dgVoodoo.conf` and `SampleAddon.dll`: full resolution per eye needs both.
The old hotkeys are gone; everything they did is in the menu.

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
| **Left grip + both sticks in (chord)** | Watch a cutscene on the theatre screen (WIP) |

While the menu is open the game doesn't see any of your input (no stray
shots, no camera spinning, no pausing), and a button still held as you close
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
| **Left Eye / Right Eye Desktop View** | Which eye the game window shows while in VR (right by default), so streaming and recording look normal. It fills the window at any size or shape. The headset is not affected. |
| **Full resolution per eye** | On by default. Each eye renders at your VR runtime's own resolution (see [Resolution](#resolution)). Shows the size each eye gets, in orange if it had to be scaled down to fit the game's memory. Off gives each eye half of the game's resolution, like older versions. |
| **Reset view** | Makes wherever you're facing "forward" again (yaw only). Same as holding both sticks. |
| **Culling follows your head** | On by default. The game draws whatever you look at, aiming included, so nothing vanishes over your shoulder. Aiming and walking stay on the mouse or stick. |
| **Stabilise camera** | Smooths Chris's idle-animation sway out of your view. |
| **Eye height / Eye forward** | VR camera position, separate from the flat-screen one. |
| **World scale** | Eye separation. Higher makes the world look smaller, lower makes it bigger. 2.75 was measured to make Sheva, guns and doors feel life-size. |
| **HUD distance** | How far away the HUD panel sits. 0 is closest, 100 is furthest. |

**Advanced** (collapsed by default):

| Option | What it does |
|---|---|
| **Head prediction** | 0 to 60 ms. Pushes the view ahead while you turn, to hide pipeline and streaming delay. Leave at 0 unless turning feels behind you, then try 15 to 25. |
| **Head rotation gain** | 1.0 is 1:1 with your neck. Anything else overshoots when you stop, so most people should leave it. |
| **Extra FOV** | Widens the rendered field of view beyond the headset's own. |
| **Match culling to the headset's FOV** | Tells the game to draw everything the headset can see. Off, things at the edge of your view vanish. |
| **Mono post-processing** | The light-leak fix: draws bloom and light shafts once instead of per eye. |

### Status tab

Live values, useful for troubleshooting. **Screenshot this page when
reporting a problem** (see [Reporting a problem](#reporting-a-problem)):

- **Mod:** version, whether a newer version is on Nexus, VR or flatscreen
  install, address space used, whether the **4GB patch** is applied
- **Rendering:** game frame rate, backbuffer size, stereo on/off and the
  resolution each eye actually gets, the render size in VR versus the game's
  own, HUD recognition
- **VR:** runtime, headset, the per-eye resolution the runtime wants versus
  what's sent, headset refresh rate, frames submitted per second, image age
- **Camera:** first person on/off, which character you're playing, camera
  hook activity, head-visibility counters

### Menu tab

Text size, whether clicking both sticks opens the menu, whether the startup
note shows, **Check Nexus for updates at startup**, a reminder of the controls,
and **Reset everything to defaults**.

**Update check:** a few seconds after launch the mod asks nexusmods.com for this
mod's latest version number. Nothing about you or your game is sent. If a newer
version is out, a note appears in the corner for a few seconds and at the top of
the menu, with a button that opens the Nexus page. Untick the option on the Menu
tab to turn it off.

---

## Features

### First person, in VR and on a flat screen

- **True first person:** the camera sits in Chris's head throughout
  gameplay, including the unarmed intro.
- **Full look range, no pop-out:** look straight up or down without the
  camera snapping back to third person.
- **Only the head is hidden:** Chris's body and hands stay visible, so you
  see your own arms and gun.
- **Head comes back for action cameras:** kicks, vaults and grabs don't
  leave a headless Chris on screen.
- **No near-camera fade:** Chris, Sheva and NPCs stay solid up close instead
  of dissolving.
- **Walk while aiming:** something RE5 never allowed. Keep moving with your
  weapon up, on WASD or a controller's left stick (analog).
- **Laser sight always on**, including with mouse aiming.
- **Colour filter removed** by default.

### 6DOF arms

Enable **6DOF arms** on the VR tab, hold both sticks in and T-pose as
instructed, palms down. The sliders in that section change how the T-pose
lands; the defaults are solid, but recalibrate freely.

**Playing as Sheva?** Tick **Playing a left-handed character (Sheva)** on the
VR tab first. She carries her weapon in the other hand, so without it the
solve drives the wrong arm and the gun will not come to your hand.

- **Your arms are your arms:** shoulders, elbows and wrists solved from where
  your controllers really are, every frame, instead of playing an animation.
- **Wrist roll:** turn your hand and the gun turns with it.
- **The weapon lives in your hand** rather than in the animation.
- **Two hands on the gun:** bring your support hand up and it takes the
  weapon, with its own steering and roll.
- **Holsters:** reach to a spot on your body and take what is there.
- **Reload by hand** (on by default): reach to your belt on your support side
  and squeeze the grip. Pistols are the best supported for now.
- **Your hands stop at people:** reach at Sheva or at yourself and your hands
  hold instead of passing through.
- **Your body leans with you.**
- **Send my arms to my partner** (on by default): IK sync over co-op, so your
  VR buddies can wave at you and a modded flat screen player sees them too.

### VR

- **The game does not take your camera:** melee, door kicks, jumping down,
  vaulting and partner locate all leave your view where you put it.
- **Scoped weapons can stay in third person**, so rifles that zoom are usable
  in a headset instead of filling one eye.
- **Steady the hands:** RE5 shakes the character's hands while the gun is up.
  In a headset that is noise fighting your real hand, and it can be turned
  off.
- **Native stereoscopic VR through OpenXR:** each eye rendered with its own
  camera, real head tracking and the headset's actual per-eye lens FOV,
  delivered at the headset's full refresh rate.
- **Full resolution per eye:** each eye renders at the resolution your VR
  runtime asks for, whatever resolution the game itself is set to. Change it
  in SteamVR or Virtual Desktop; your own resolution comes back when VR is off.
- **Desktop view for streaming:** the game window shows one eye, filling the
  window, instead of the side-by-side image.
- **Clean handoff:** turning VR off returns the game to flat screen and hands
  the headset back to SteamVR or Virtual Desktop.
- **HUD in VR:** ammo, health, the inventory, the pause screen and most menus
  on a panel in front of you, at a distance you choose.
- **Look around freely, aiming included:** the game draws whatever you look
  at, while the gun, laser and walking direction stay with the mouse or stick.
  Press aim while looking somewhere else and the gun just comes up; your view
  doesn't move.
- **Calibrated world scale:** measured from first person, adjustable in the
  menu.
- **VR culling fixes:** the game draws everything the headset can see,
  including the ground at your feet, your partner's legs, and whatever is over
  your shoulder.
- **Reset view:** from the menu, or hold both sticks in for a second.
- **Theatre screen for cutscenes:** hold left grip and click both sticks in
  together, and the cutscene plays on a screen in front of you instead of
  across your face. Still WIP.
- **No light leaks:** post-processing is drawn once rather than split per eye.

### Quality of life

- **In-game menu** with saved settings, working with mouse, keyboard or
  controller, on the monitor or in the headset.
- **Live Status page**, including a 4GB-patch check.
- **Version display and update check:** the menu shows the mod's version and
  lets you know when a newer one is on Nexus.
- **Drop-in install:** no game files are modified; delete the files to
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

**Windowed mode is still the safe choice.** Changing resolution in exclusive
fullscreen no longer crashes as of v0.4.2, but turning VR on while in exclusive
fullscreen hasn't been confirmed fixed on every PC.

### Motion controllers

Your VR controllers work as a gamepad, so RE5 sees an ordinary pad and shows
pad prompts. Nothing needs to be plugged in. They also drive the mod menu:
click both sticks to open it, the sticks move through it, A selects.

| Motion control | Acts as | In RE5 |
| --- | --- | --- |
| Right grip | Left trigger | Aim |
| Right trigger | Right trigger | Fire |
| Right stick | Right stick | Look around |
| Right stick click | R3 | |
| Right A / B | A / B | Action |
| **Left trigger (hold)** | nothing | **Modifier: the right stick becomes the d-pad** |
| Left stick | Left stick | Move |
| Left stick click | L3 | (both stick clicks open the mod menu) |
| Left X / Y | X / Y | Reload, partner command |
| Left grip | Left shoulder | |
| Menu button | Start | |

In game I still recommend the Type C control scheme. Your knife lives over
your left shoulder: reach up and grab it with the grip on whichever hand you
hold it in, so right grip normally, left grip if you are set up left handed.

A pad has a d-pad and a controller doesn't, so one stick stands in for it
while you hold a modifier. The **D-pad** setting on the VR tab picks which:
hold the left trigger (default, works on every headset), rest your thumb on
the right thumbrest (Quest and Rift Touch only), hold the left stick in, or
give up the right stick to the d-pad entirely. There is also a **Stick
deadzone** slider if you drift while standing still.

Turn the lot off with **Use motion controllers** on the VR tab.

### Resolution

Each eye renders at the resolution your VR runtime recommends, so the
runtime's own setting is the dial:

- **SteamVR:** *Settings → Video → Render Resolution* (or per game).
- **Virtual Desktop:** the quality preset (Potato to Godlike). Its "rendering
  resolution" readout shows 100% when the game renders exactly what the preset
  asks for.

After changing it, turn **Enable VR** off and on again in the menu; no restart
needed. RE5 itself switches to the VR size while VR is on, and back to your
own resolution when it's off, so the game's own resolution setting doesn't
affect VR at all.

If the runtime asks for more than the game can hold in memory, the size is
scaled down to the most that fits and the menu shows it in orange. The 4GB
patch and `VRAM = 2048` in `dgVoodoo.conf` (as shipped) give it plenty of
room; without the 4GB patch VR is held to 5 megapixels.

**VR motion controllers are not supported yet.** Play with a gamepad (Xbox,
PlayStation or similar), which is recommended, or mouse and keyboard.

---

## Performance

- **VR renders the scene twice**, once per eye, at your runtime's
  resolution. If your frame rate is low, lower the resolution in SteamVR or
  pick a lower Virtual Desktop preset; that is the lever now. The game's own
  resolution setting only matters for flat screen.
- **The desktop view is free:** it is a copy of one eye, not a second render.
- **Stutter is a separate problem** from frame rate, and the 4GB patch is the
  fix for it.
- **Streaming over Wi-Fi** (Virtual Desktop, Air Link) adds its own delay and
  frame cap on top; a wired link or LAN is noticeably better.

---

## Known issues

- The laser can sit slightly off the muzzle on some weapons. QOL Fixes can
  remove the laser entirely, but starting with it on is worth getting used
  to. Being fine tuned.
- Manual reloading is built around pistols for now, and switching weapons
  does not drop mags for everything. The gesture always works.
- Arms only body mode leaves some skeleton visible. Not the permanent
  implementation.
- **Lean and peek is WIP. Do not use.** Off by default.
- **Turning VR on in exclusive fullscreen** can still crash inside dgVoodoo2 on
  some PCs. Play windowed if it happens to you. (Changing resolution while
  fullscreen is fixed in v0.4.2.)
- Some menus (the title screen, parts of the Organize screen) still don't
  draw correctly in VR.
- Split-screen co-op: Chris's head stays hidden.

### Reporting a problem

Please include these with any bug report or issue:

1. **A screenshot of the menu's Status tab.** Open the menu (Insert, or click
   both sticks), switch to **Status**, and screenshot it, ideally while the
   problem is happening. It shows your frame rate, resolution per eye,
   headset and runtime, refresh rate, whether the 4GB patch is applied, and
   more, in one picture.
2. **The log files** from the game folder
   (`...\Steam\steamapps\common\Resident Evil 5\`):
   - `re5vr.log`: always
   - `re5vr_addon.log`: as well, if you were playing in VR

   `re5vr.log` is **replaced every time the game starts**, so copy it before
   launching again, or the session with the problem is gone.

Say roughly when it happened too (the logs have timestamps), and which
download you're using.

---

## Next up

- Keeping the camera inside Chris during actions.
- Motion controllers with arm IK.

---

## How it works

RE5 renders through a `d3d9.dll` proxy that this project supplies. The game's
D3D9 calls are translated to Direct3D 12 by dgVoodoo2, a dgVoodoo2 addon plugin
grabs each finished frame straight off the D3D12 swapchain, and the frame is
handed to an OpenXR runtime as a pair of eye textures. The stereo image itself
is produced by drawing the scene twice per frame into one backbuffer with a
per-eye camera and scissor rectangle. While VR is on, the mod switches RE5's
own render size (the same values its resolution setting writes) to twice the
runtime's per-eye width by its height, so each half of that frame is a full
resolution eye. HUD draws are recognised by their shader bytecode and drawn
once per eye onto a panel at a fixed convergence.

The menu is Dear ImGui drawn over the game's own D3D9 device. While it is open,
the game's input is hidden at every route RE5 uses (DirectInput keyboard and
mouse, XInput, the Windows cursor and key state, and its message pump), with
the Win32 routes blocked only for calls made from the game's own code.

## Repository layout

```
RE5VR/
  src/
    proxy/        d3d9.dll proxy - export table hand-matched to what RE5 probes
    hooks/        MinHook-based game hooks (camera rig, fade, laser, filter, ...)
    render/       stereo_test.cpp - per-eye camera, projection, scissor split
                  render_size.cpp - full resolution per eye (RE5's render size)
                  hud_shaders.cpp - recognises the HUD's shaders for VR
    ui/           menu.cpp - the in-game Dear ImGui menu and re5vr.ini
                  input_block.cpp - keeps the game's input away while it is open
    vr/           openxr_bridge.cpp   - OpenXR session, swapchain, submit
                  d3d12_addon_bridge.cpp - consumer side of the addon handoff
    util/         logging, build switches, version.h (the mod's version),
                  update_check.cpp - asks Nexus for the latest version
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
both projects as **Release | Win32**. The game is 32-bit, so a 64-bit build
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

Thanks to [RE5Fix](https://github.com/Lyall/RE5Fix) by Lyall (MIT), whose
resolution-limit patch pointed the way to where RE5 keeps its display
settings.

**OpenVR is deliberately not in this repository.** An earlier phase of the
project targeted SteamVR through OpenVR; that path was replaced by OpenXR and
`RE5VR.vcxproj` no longer compiles or links any of it. The SDK is ~495 MB,
almost all of it Unity/Qt/OpenCV sample binaries. `src/vr/openvr_bridge.cpp`
is kept in the tree for reference but is not part of the build. If that path is
ever revived, re-clone the SDK into `RE5VR/thirdparty/openvr/`.
