# TrueFP: what is left before release

Written 2026-09-26, from the list you laid out. Uncommitted, edit freely.

The short version: your ten items are not ten problems. They are three pieces of
architecture plus four independent features. Most of today's pain came from
attacking symptoms of the first piece without owning it.

---

## The three architectural knots

### A. We do not own the camera during action sequences

**Covers items 1 (checkerboard), 4 (live camera in action), the cutscene
detection problem, and the punch-the-boulder idea.**

In ordinary play we *do* own it. `ApplyOverride` in `camera_rig_hook.cpp` zeroes
the rig's horizontal, vertical and orbit distances every time the game writes
them, which collapses the game's own camera onto the character. That is what
first person is here. There is no compensation, no hold, nothing to drift.

Melee, stomps, vaults and cutscenes do not go through those rigs. Nothing gets
zeroed, the camera leaves, and all we can do downstream is swap the view matrix
at `GetViewMatrix`. That leaves the renderer sorting transparencies, measuring
fade distances and picking detail levels for a camera we are not at, which is
the checkerboard, and it forces the hold-and-release machinery, which is the
stairs jump and the disorientation.

Everything we did today to the culling, the fade and the departure detector was
working on that one gap from the outside.

**What changed today:** the hunt settled.

```
CameraFind: settled after 5 rounds - the camera keeps its position at +30, +1F0, +3B0, +640
```

Four fields on the main camera object that tracked the real position across five
moves. Writing our eye there (`CameraFollowsEye`, now on in your ini) is the
weak form of the fix: the renderer agrees with us, so the checkerboard should
go, but the camera still leaves and we are still pinning.

**The strong form, and the one I would build:** stop the camera leaving at all.
Those four addresses are now a place to put a write watchpoint. Trigger a stomp
with the watchpoint armed and the game tells us exactly which routine moves the
camera during an action sequence. Whatever that is, it is the melee equivalent
of `ApplyOverride`, and the same treatment applies: let it run, then put the
camera back on the body before the frame is drawn.

If that lands:

- Item 1 dissolves. No disagreement, so no checkerboard, and no culling or fade
  work needed at all.
- Item 4 is solved by construction. The camera never goes third person, so there
  is nothing to smooth, hold, or release.
- The whole departure detector can be deleted. No thresholds, no dwell, no
  velocity test, no stairs jump. That machinery only exists to paper over this.
- The uppercut idea becomes mostly free. We already synthesise pad input, so a
  gesture can press the button. What makes it a cutscene is the camera
  animation, and that is the thing we just removed. Chris punching the boulder
  in first person is then a gesture plus a suppressed camera, not a new feature.

**On "there is no flag that says this is a cutscene":** there almost certainly
is, we just have not found it, and our current substitute is bad. Right now we
infer it from the rig going quiet for 400 ms, which at 16 fps is six frames and
misfires constantly. Two better routes:

1. *Diff the globals.* Snapshot a block of the game's state on a hotkey, enter a
   cutscene, snapshot again, diff. A flag that flips exactly when gameplay stops
   will stand out. This suits how you like to work, it is fully automatable, and
   it is bounded: one session of pressing a key at the right moments.
2. *Ask the input system.* During a cutscene the game stops accepting player
   input. Whatever gate does that is a cleaner cutscene signal than anything we
   can infer from the renderer, and it distinguishes a cutscene from a melee
   camera, which is the distinction we actually need.

Worth being precise about why the distinction matters: a cutscene should
probably keep the theatre, and a melee should keep you in your body. Today we
treat them as one thing and get both wrong.

---

### B. The skeleton drives the eye, when the headset should

**Covers items 6 (lean and peek), 7 (roomscale), and most of 8 (collision).**

Today the eye is mounted on the neck joint and the lean rotates the spine chain
to fake a head offset. That is why leaning is a banana curve: rotating a chain
of joints moves your head along an arc, because that is what rotating a chain
does. No amount of tuning makes an arc into a translation.

The inversion: your head position in room space is the truth, the eye goes
exactly there in world space, and the skeleton follows with IK so the body is
somewhere plausible underneath it. Same relationship the arms already have with
your controllers, applied to the head.

This is a real piece of work but it is well understood, and it pays for three
items at once:

- **Item 6** becomes trivial, because leaning is then just your head being where
  your head is. No curve to correct.
- **Item 7** stops fighting the animation. Roomscale currently feeds the stick,
  so the locomotion blendspace plays a full step for a small input, which is the
  stutter. If the eye is driven by the headset, a room step moves the eye
  immediately and the character's world position can be nudged directly, without
  ever touching the movement input. The animation never fires because the game
  does not know it moved.
- **Item 8** gets its natural home. Once we are moving the eye and the body
  ourselves, the question "can I move there" has to be asked anyway, and the
  answer is the same query for walls, for hands and for room steps.

The risk to name: nudging the character's world position directly bypasses the
game's collision, which is exactly why 8 stops being a stretch goal and becomes
a dependency. Options for collision, cheapest first:

1. *Probe by attempting.* Ask the game to move the character a small distance
   and see whether it accepted it. No new addresses, works with whatever
   collision the game has, costs a frame of latency.
2. *Find a raycast.* The AI and the camera both need one. Finding it gives us
   walls, hands and pickups in one go, and it is the better answer if it exists
   in a callable form.
3. *Clamp to the capsule.* Keep the eye inside the character's own collision
   capsule and let the game do the rest. Crude, but it prevents the worst case
   of putting your head through a wall, and it is nearly free.

---

### C. Hiding the body is being done to the wrong thing

**Covers items 2 and 3.**

Today's history, so we do not repeat it:

- *Local bone scale.* Propagates to children, so it took the camera down with
  the body, and the game does not put it back. Unrecoverable in-session.
- *Collapsing bones to a point.* The geometry is still present, so it unfolds
  whenever a kept limb comes near the collapse point. Your own finding, and it
  rules out every variant.
- *NaN in the shader's bone table.* Clean in principle, but those are skinning
  matrices, not world matrices, so a position test can only name bones whose
  bind pose sits near the model origin. It hid your weapon hand and nothing
  else. Permanently ruled out.
- *NaN in the skeleton's world matrices.* Works visually. Three failure modes,
  all found today: it poisons every reader in the mod that touches those
  matrices, it hits the partner whenever `g_body` is holding her, and it sticks
  on any character the game is not fully recomposing, which is why Sheva stayed
  broken after you switched back.

The last two of those are fixed and untested as of 10:42. But the approach is
fragile by nature, because it writes poison into a live structure that eighteen
other places read.

**The option we have not tried, and the one I would try next: skip the draw
call.** We already see every bone table on its way to the shader, and the same
hook tells us which draw each belongs to. If a submesh's bone table is dominated
by bones we want hidden, do not draw it. Nothing is written anywhere, nothing
can leak, the partner cannot be touched because we would only skip draws whose
palette matches our own joints, and there is nothing to restore because the next
frame simply draws it again.

The catch is granularity: it can only hide what the model happens to split into
separate draws. If Chris's torso and arms are one submesh, this cannot separate
them. That is one measurement, not a guess: log the draws for the player and see
how many there are and what each one covers. If the split is fine enough, items
2 and 3 are done properly and permanently. If it is not, we keep skeleton NaN
with today's fixes and accept it needs care.

Worth doing that measurement before any more tuning of the current approach.

---

## The independent features

### Item 5: per-weapon reloading

The structure is already in hand and the design is clear.

- `mark` at ammo `+0x24` is the ammunition class: `0201` handgun, `0202` machine
  gun, `0203` shotgun. That gives correct default behaviour for every weapon in
  the game with no per-weapon work: classes 0201 and 0202 are magazine weapons.
- Weapon object `+0x04` is a per-model hash, stable across launches. That is the
  key for the exceptions, which is the shotguns that do take magazines.
- So: behaviour by class, overridden by a small table keyed on hash. Log any
  hash we have not seen, and the table fills itself as you play.

Shell-by-shell is the interesting one, and RE5 helps. Its shotgun reload already
inserts shells one at a time and is already interruptible by firing, which is
exactly the behaviour you described. So we do not have to build it, we have to
*drive* it: start the game's reload, count shells in, and cancel when the
gesture stops. Firing mid-reload then works because it already works.

The reserve is still a mirror we cannot write, so every one of these has to go
through the game's own reload to debit the inventory correctly. That is already
how the magazine path works.

### Item 9: picking things up

Needs the raycast or proximity query from B. Given that, the gesture is the easy
part: hand near item plus grip, then trigger the game's own pickup. Putting it on
your chest to store it is a second gesture against the inventory, which we
already open from the wrist.

Throwing an item to your partner is a different order of problem, because it
needs the item to exist for both machines. Co-op sync is the last piece of the
project anyway, and this rides on it rather than leading it.

### Item 10: the HUD in world space

The most tractable thing on the list, and largely independent of everything
above. We have the device, the weapon skeleton and its attach transforms, so an
ammo counter on the side of the gun is our own textured quad drawn at a bone
after the scene. Health on the wrist is the same trick against the wrist joint.

This one could be done at any time and would make the mod feel finished out of
proportion to its cost.

### The main menu and pre-rendered cutscenes

Needing theatre mode on the main menu is a symptom of the same missing signal as
A. If we can tell "the game is playing a video" from "the game is rendering a
scene", the menu and the pre-rendered cutscenes take the theatre automatically
and live gameplay never does. Pre-rendered video is a distinct render path, so
this is likely easier to detect than the cutscene flag proper.

---

## Order I would build in

1. **Watchpoint the four camera fields during a stomp.** Highest value per hour
   in the whole list. It is the difference between fixing item 4 and continuing
   to compensate for it, and it deletes a large amount of existing machinery.
2. **Measure the player's draw calls.** One log line's worth of work, and it
   decides whether items 2 and 3 get the clean solution or the careful one.
3. **Find the cutscene flag by diffing globals.** Unblocks the menu, the theatre
   and the melee distinction, and stops us inferring state from frame timing,
   which has burned us twice today.
4. **Item 5**, which is self-contained and mostly designed.
5. **Item 10**, whenever a change of pace is wanted. Visible, satisfying, low
   risk.
6. **The headset-drives-the-eye inversion (B)**, with collision as part of it.
   The biggest single change, and worth doing in one go rather than in pieces.
7. **Co-op sync of everything**, last, because it multiplies whatever exists.

---

## One thing I would push back on

Not on the goals, on the sequencing. Items 6, 7, 8 and 9 all want the same
architectural change and all four are cheaper after it than before it. If we
build 6 and 7 against the current architecture we will build them twice. I would
rather do A first because it deletes code, then B properly, and let the features
land on top of a shape that fits them.

## Added 2026-09-27, from the trainer

**Hand tremor / aim sway.** The trainer has a clean "Hand Tremors Fix" toggle,
so it is one value or one patch site rather than a system. Worth doing for VR
on its own merits: the sway exists to make a gamepad crosshair drift, and in a
headset it fights your actual hand and works against the arm IK, which is
trying to hold the weapon exactly where your controller is. Cheap, visible,
and independent of everything else on this list.

**The melee camera is not a separate rig.** Freezing it in the trainer makes
melee use the normal camera's sliders, which means it writes over the NORMAL
rig during a melee. We already override that rig, but only when the game
builds it; the melee write lands afterwards and wins. Re-applying the override
every frame is the same thing the trainer's freeze does.
