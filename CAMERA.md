# The camera: what is known, and the fix

Written 2026-09-27 at the end of a long session. Uncommitted.

## Why this is a document and not a build

You asked for a one swoop fix. Writing one right now would be the fifth
interception of the night, and the previous four each looked just as reasonable
before I built them:

1. Pin the camera position. Worked for the checkerboard, disabled the head lock
   as a side effect, and took four more builds to unpick.
2. Pin the look-at target from the body facing. Caused a non stop 360 the
   moment you aimed, because aiming turns the body toward the camera.
3. Hook `SetLookAt` at `exe+43A560`. Crashed the level load, and crashed
   identically when changed to do nothing at all, proving the detour itself is
   unsafe on a packed executable.
4. Limit the turn on the camera matrix at shader register 0. Flickered, because
   register 0 is whatever matrix the current pass needs, not the player camera.

All four were built on an assumption I never checked: **that the view we
compute is the view you look through.** Two observations say it may not be.
The limiter refused a 102 degree turn while you were whipped anyway, and the
whip happens in flatscreen, where none of the VR path runs.

So the build you have has two measurements, each costing one keypress, and the
answer to the first one decides which fix to write. That is worth more than
another guess.

## What is established

**The camera has no rotation.** A scan for an orthonormal basis found none
anywhere in its first 2048 bytes. It is a pure look-at: an eye and a target,
orientation implied by the vector between them. There is nothing to pin even if
we wanted to.

**The layout.**

| Field | What it is |
|---|---|
| `+0x30` | eye, copied from `+0x640` |
| `+0x50` | look-at target |
| `+0x1F0` | eye, copied from `+0x30` |
| `+0x3B0` | eye, the source of the chain |
| `+0x3D0` | target, alongside `+0x3B0` |
| `+0x640` | eye, copied from `+0x3B0` |

**The position accumulates, it is not assigned.** At `exe+442714`:

```
movaps xmm0, xmm4
addss  xmm0, [esi+3B0]
movss  [esi+3B0], xmm0      and the same for 3B4, 3B8
```

The delta arrives in `xmm4/5/6`. Zeroing it stops the camera moving itself,
which is what ended the tug of war with the pin. That patch is safe and is in.

**`exe+43A560` is `SetLookAt(eye, target)`**, thiscall, `ecx` pointing at
`camera+0x380`. It fires during a stomp and never in ordinary play. This is
where an action camera places you. It is the right place and it is **not
patchable with a five byte detour** - a hook that changed nothing crashed the
level load exactly as the substituting one did.

**Twelve callers fetch the view**, of which eleven now get ours. The busiest
player camera callers were not on the list until tonight.

**`EndScene` runs about thirty four times per presented frame.** The pin was
called from it and was writing the camera thirty four times a frame, which was
the body stutter. Now once, from Present. Fixed.

## The two measurements in the current build

**Scroll Lock, held for two seconds.** While the window is open the view is
forced due world north, ignoring the game completely.

- The picture snaps to a fixed heading -> our view is what you see, the whip
  comes through `BuildHeadLockedView`, and the fix belongs there.
- Nothing happens -> our view is discarded and everything done to it tonight
  was never going to work.

**The same window logs one line per frame:**

```
Whip: eye (x y z) looking N yaw M pitch, D from the target; head (x y z), E from the eye
```

Press it just before a stomp. The shape of those numbers says which of four
things a whip actually is:

- target moves, eye holds -> the game is re-aiming you
- eye swings, target holds -> it is orbiting you
- both move together -> the whole camera is being flown somewhere
- neither moves and you are still thrown -> it is not this camera

## The fix, both versions

### If the view is ours (Scroll Lock moves the picture)

**Stop reading the game's camera for orientation. Own the view outright.**

Today `BuildHeadLockedView` takes its direction from `target - eye`, so every
action camera reaches you by construction, and everything we do afterwards is
damage control on a number we chose to import.

Instead: keep our own yaw, integrated from player input only - stick, mouse, and
in VR the headset. Position from the neck as now. The game's target is never
read. An action camera then cannot move your view at all, because nothing it
writes is an input to what you see.

What this costs, and needs deciding:

- Anywhere the game legitimately must aim you, mainly cutscenes, needs a way
  back in. That is the cutscene signal we still lack, and the honest interim is
  a setting: hold through everything, or hold through everything except when
  the rig goes quiet for a long time.
- Aiming must not turn the character toward the camera, which you have already
  said you want severed for 6DOF anyway. These are the same piece of work.

This is genuinely one change, in one function, and it removes the whip, the
orbit and the revive spin together rather than one at a time.

### If the view is not ours (Scroll Lock does nothing)

Then the final view is composed somewhere we have not found, and the priority
becomes finding it rather than fixing anything. The way in is the same
watchpoint technique that has worked all night, pointed at the camera matrix in
the shader constant: break on the write to register 0 and read back the call
stack to find who composed it. We know what the matrix looks like and we know
when it is wrong, so it is a bounded hunt.

## What not to try again

- **Deriving anything written into the camera from anything the game derives
  from the camera.** Three separate failures: `camPos`, `forward`, and the body
  facing. Each produced a runaway rather than an error.
- **Five byte detours around `43A560`.** Proven unsafe.
- **Treating register 0 as the player camera.** It is per pass.
- **Assuming the pin and the head lock agree.** They are two placements, and
  they only agree if one is computed from the other.
