# Animation-driven body: RE notes for overhauling the gait

Reverse engineered from `Fallout4VR.exe` 1.2.72 (Ghidra, `Combined` project) and from
VRIK Player Avatar 0.8.6's shipped Papyrus API. This documents *why* FRIK synthesises
a gait, what the engine already provides, and what remains unknown.

## The architectural difference with VRIK

VRIK contains no procedural gait at all. Its own API documentation defines every
subsystem's disabled state as "follows animation":

- `enableBodyPosture` — "Set to 0 to disable (posture follows animation)"
- `enableBodyBending` — "Set to 0 to disable (body follows animation)"
- `enableHeadPositioning` — "Set to 0 to disable (head follows animation)"
- `enableJumping` — "Set to 0 to have VRIK disallow jumping, which can halt idle animations"

`vrik.ini` has no step length, step time, foot placement, or gait tuning of any kind.
Skyrim animates the player's third-person body and VRIK only *corrects* it.

FRIK instead rebuilds the whole gait: `Skeleton::walk()` runs a four-state machine,
picks a stepping foot with `std::rand()`, and drives feet along a sine arc, because
`restoreNodesToDefault()` first resets every bone to a hardcoded rest pose each frame.

## What Fallout 4 VR already does

The player's animation graph **is** updated every frame — this is not a dormant system.
`PlayerCharacter::UpdateAnimation` (`0x0f0fa50`) sets graph variables and then runs:

```
TESObjectREFR::SetupAnimationUpdateDataForReference(player, deltaTime)
IAnimationGraphManagerHolder::UpdateAnimationGraphManager(player + 0x48, updateData)
```

The animated pose then reaches a second skeleton through
`PlayerCharacter::PostUpdateAnimationGraphManager`, which is `PlayerCharacter::vf023`
in Ghidra (`TESObjectREFR` declares it at vtable slot `0x17`; CommonLibF4 has it as a
no-op stub). Structure of the override:

```c
// NOTE: `this` here is the IAnimationGraphManagerHolder subobject at player+0x48.
// The function reaches the Actor via (this - 0x48), so every offset below is
// holder-relative. Add 0x48 for a PlayerCharacter-relative offset.
if ((holder[0x1256] & 0x10) == 0) {            // gate; player-relative 0x129E
    src = GetFlattenedBoneTree(holder[0x0FA0]) // player-relative 0x0FE8
    dst = GetFlattenedBoneTree(Get3D(false))   // the third-person body
    for (i = 0; i < holder[0x1200]; i++) {     // count; player-relative 0x1248
        map = holder[0x11F0];                  // {int srcIdx, int dstIdx}[]; player-rel 0x1238
        // copies translate x/y/z and a flags word, then:
        BSFlattenedBoneTree::SetBoneRotate(dst, map[i].dstIdx, srcTransform);
    }
}
```

So the engine copies an animated pose, bone by bone, from one flattened bone tree into
the third-person body via an index map. `BSFlattenedBoneTree::SetBoneRotate` is at
`0x05bdf30`.

The node list feeding this is built by `TESObjectREFR::PopulateGraphNodesToTarget`;
`PlayerCharacter::PopulateGraphNodesToTarget` (`0x0f2ec80`) wraps it and appends one
extra node.

The FRIK author had already located this hook point. `F4VROffsets.h` still carries the
note, and the call is commented out in `GameHooks.cpp`:

```cpp
inline REL::Relocation hookAnimationVFunc(REL::Offset(0xf2f0a8));
// This is PostUpdateAnimationGraphManager virtual function that updates the
// player skeleton below the hmd.
```

`0xf2f0a8` is the address of the gate test itself (`TEST byte ptr [RCX+0x1256],0x10`)
inside `vf023`. FRIK's live hook is instead `hook_MainUpdatePlayer` (`0x0f0ff6a`), a
call site in the unnamed function immediately after `UpdateAnimation`.

## What this implies for the overhaul

The gait should stop being synthesised and start being corrected, matching VRIK:

1. Let `PostUpdateAnimationGraphManager` deliver the animated pose to the body.
2. Run FRIK's solver *after* it, and stop resetting leg bones to the rest pose.
3. Keep the arms/hands fully IK-driven (they must track controllers exactly).
4. Reduce `walk()` to reading locomotion state, not generating one.

## The gate selects the behavior graph

The gate bit is not an obscure flag — it selects **which behavior graph the player
runs**, which is exactly the "animation system" that has to be active for the body to
animate. `PlayerCharacter::vf024` watches for the bit disagreeing with the graph's own
state and switches the graph to match:

```c
lVar2 = *param_2;                                   // the animation graph
if (lVar2 != 0 && *(int *)(lVar2 + 0x50) == 2 &&
    ((~(uint)(*(byte *)(holder + 0x1256) >> 4) & 1) != *(uint *)(lVar2 + 0xd8))) {
    PlayerCharacter::SwitchBehaviorGraph(holder - 0x48);
}
```

`(~(bit >> 4) & 1)` is the *inverse* of the gate bit, so:

| gate bit `0x10` | behavior graph | `vf023` bone sync |
|---|---|---|
| clear | third-person graph | **runs** |
| set | first-person graph | skipped |

The same `vf024` also drives `PlayerCharacter::DoShow1stPerson` from the camera state.

So the sequence to get an animated body is: clear the gate bit, let `vf024` call
`SwitchBehaviorGraph`, and `vf023` then syncs the animated pose onto the third-person
skeleton every frame.

`PlayerCharacter::SwitchBehaviorGraph` (`0x00f2a180`, 2006 bytes) has not been fully
decoded. It rebuilds several scrap arrays, which is very likely where the bone index
map used by `vf023` is built. Calling it directly is untested and is not a safe blind
change — it tears down and rebuilds graph state.

## Unresolved — must be answered before implementing

1. **Which bones the map covers.** If the index map only spans the upper body, the
   legs are never animated and a procedural gait remains unavoidable. This is the
   single question that decides the whole design and it is still open.
   `TESObjectREFR::PopulateGraphNodesToTarget` is *not* the source — it only appends
   `Get3D(thirdPerson)`, a single root node. The map is most likely built inside
   `SwitchBehaviorGraph`. It is runtime data, so confirming its contents realistically
   needs a debugger attached to a running game rather than static analysis.
2. **Hook ordering.** Whether FRIK's current hook runs before or after `vf023`, and
   whether the third-person tree is culled or otherwise skipped in VR.
3. **Whether the animated pose survives.** FRIK's rest-pose reset exists because
   "loading a game does NOT reset the skeleton nodes", which suggests those bones are
   *not* being rewritten each frame. That is in tension with an active per-frame sync.
   The likely resolution is that the gate bit is *set* in VR (first-person graph, no
   sync), which is exactly why the author saw a static third-person skeleton — but
   that has not been confirmed.

## Additional verified addresses

| Symbol | Offset | Size |
|---|---|---|
| `PlayerCharacter::SwitchBehaviorGraph` | `0x00f2a180` | 2006 |
| `PlayerCharacter::DoShow1stPerson` | `0x00f299d0` | 1717 |

## Verified addresses (F4VR 1.2.72)

All confirmed as exact function entry points unless noted.

| Symbol | Offset | Size |
|---|---|---|
| `PlayerCharacter::UpdateAnimation` | `0x00f0fa50` | 515 |
| `PlayerCharacter::vf023` (PostUpdateAnimationGraphManager) | `0x00f2f0a0` | 525 |
| gate test inside `vf023` (not an entry point) | `0x00f2f0a8` | — |
| `PlayerCharacter::vf012` (also tests the gate) | `0x00f2eb80` | 293 |
| `PlayerCharacter::vf024` (loads the gate byte) | `0x00f2eff0` | 168 |
| `PlayerCharacter::PopulateGraphNodesToTarget` | `0x00f2ecb0` | 123 |
| `TESObjectREFR::PopulateGraphNodesToTarget` | `0x004194b0` | 118 |
| `PlayerCharacter::Update3rdPSceneGraph` | `0x00f10ed0` | — |
| `BSFlattenedBoneTree::SetBoneRotate` | `0x005bdf30` | — |
| `BSFlattenedBoneTree::UpdateBoneArray` | `0x01c214b0` | — |
| `BSFlattenedBoneTree::AdjustBonesWorldTranslate` | `0x00d98600` | — |

Note `vf023` starts at `0x00f2f0a0` and the gate test is its 9th byte, so
`hookAnimationVFunc(0xf2f0a8)` in `F4VROffsets.h` points *inside* the function
prologue region rather than at a call site — worth accounting for when hooking.
