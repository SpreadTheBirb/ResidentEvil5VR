# RE5VR

VR mod for Resident Evil 5 (Steam DX9 build, `re5dx9.exe`). See
`C:\Users\Afterlife\.claude\plans\hashed-brewing-cocke.md` for the full
architecture/roadmap.

## Phase 0 status: DONE - proxy hook + debug overlay confirmed stable

Confirmed working: the game runs normally through menu → gameplay →
exclusive fullscreen with the debug quad blinking continuously and
`EndScene` firing every frame (6000+ frames logged over ~20s of play), no
crashes.

What's implemented:
- A `d3d9.dll` proxy that sits next to `re5dx9.exe`. RE5 does **not**
  actually load it via ordinary filename search order or a hookable
  `GetProcAddress` call for `Direct3DCreate9` (it appears to resolve the
  real d3d9.dll's export table manually) - what actually works is a
  **bootstrap vtable hook**: we call the real `Direct3DCreate9Ex` ourselves
  once at `DllMain` time purely to obtain a throwaway `IDirect3D9Ex` object,
  read `CreateDevice`'s code address off its vtable, and MinHook that
  address directly. Since that code is shared/global for the one loaded
  d3d9.dll module, this intercepts the game's `CreateDevice` calls too, no
  matter which `IDirect3D9` object instance the game itself is using.
- Every real d3d9.dll export is preserved (verified byte-for-byte against
  the real export table via naked jmp-trampoline stubs), so nothing else
  about the game changes.
- The game's device is left **completely unmodified** (plain
  `CreateDevice`, no Ex upgrade, no Present/Reset interception). An earlier
  attempt to upgrade it to `CreateDeviceEx` (for future shared-surface
  textures) hit `PresentEx`/`ResetEx` returning `D3DERR_INVALIDCALL` during
  RE5's windowed→exclusive-fullscreen transition, even after fixing every
  documented Ex restriction (SwapEffect, desktop-matching BackBufferFormat,
  D3DCREATE_PUREDEVICE, pFullscreenDisplayMode) - this driver's Ex path is
  fragile for that transition. Decision: don't touch the game's device at
  all; Phase 2 will bridge frames out via a **separate, dedicated hidden Ex
  device** instead (see the plan file).
- `EndScene` is hooked to draw a small blinking (red/green, ~0.5s period)
  debug quad in the top-left corner of the screen every frame, and to log a
  frame counter every 300 frames.
- Everything logs to `re5vr.log` next to the DLL.

## Phase 1 status: DONE - real side-by-side stereo rendering confirmed

Press **F8** in-game: every draw call is executed twice (camera matrix
shifted left/right via vertex shader constant register `c0`, found
empirically), scissor-clipped to that eye's half of the screen. Confirmed
via screenshots to show genuine depth-dependent parallax (the near player
character shifts far more between the two halves than the distant
background). See `src/render/stereo_test.h` for why a scissor rect is used
instead of splitting the viewport or redirecting render targets (both
tried first, both broke the pipeline in different ways).

## Phase 2 status: DONE - real image confirmed in the Quest headset

Press **F7** in-game (independent of F8, but forces it on): initializes
OpenVR/SteamVR, creates a separate hidden `IDirect3DDevice9Ex` bridge
device (never touches the game's own device), copies each eye's half of
the finished frame across via a GPU→system-memory→system-memory→GPU
relay (`src/vr/openvr_bridge.cpp`), and submits both eyes to the
compositor every frame.

Needs `openvr_api.dll` (from `thirdparty/openvr/bin/win32/`) copied next
to `d3d9.dll` in the game folder - see Install below.

Known rough edges (expected at this stage, not bugs to chase yet):
- **No head tracking yet** (Phase 3, next) - the image is currently static
  regardless of head rotation, which alone is enough to cause the eye
  strain/discomfort reported when testing.
- Eye separation (50 world units) is an arbitrary placeholder, not
  calibrated to a real IPD.
- The submitted image is a plain flat-camera-FOV crop, not rendered with
  the headset's actual per-eye lens-matched projection.

## Build

Open `RE5VR.sln` in Visual Studio (2026 Community, C++ workload) and build
`Release|Win32`, or from a shell:

```bash
"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" RE5VR.sln -p:Configuration=Release -p:Platform=Win32
```

Output: `build\Release\d3d9.dll`.

## Install (test build)

Copy `build\Release\d3d9.dll` **and** `thirdparty\openvr\bin\win32\openvr_api.dll`
into the game folder:

```
C:\Program Files (x86)\Steam\steamapps\common\Resident Evil 5\d3d9.dll
C:\Program Files (x86)\Steam\steamapps\common\Resident Evil 5\openvr_api.dll
```

Launch the game normally through Steam (flat, not through SteamVR - the
game itself doesn't need to be launched any differently; F7 handles
initializing OpenVR at runtime).

## What to check

1. `re5vr.log` should appear next to `re5dx9.exe` with lines like:
   - `RealD3D9_Init: loaded real d3d9.dll from ...`
   - `Hooks_OnD3D9ExCreated: CreateDevice hook enabled -> 0` (0 = `MH_OK`)
   - `hkCreateDevice: ... real CreateDevice succeeded`
   - `Hooks_OnDeviceCreated: EndScene hook enabled -> 0`
   - periodic `hkEndScene: frame N` lines while playing
2. In the game window itself: a small ~40x40px quad in the top-left corner
   that alternates red/green roughly twice a second, visible over the menu
   and in-game.

Both are confirmed working as of this build. If a future change breaks it,
send back `re5vr.log`'s contents (or its absence) plus anything shown on
screen - that tells us exactly where it broke.

## Uninstall

Delete `d3d9.dll` (and `re5vr.log`) from the game folder to return to a
vanilla install.
