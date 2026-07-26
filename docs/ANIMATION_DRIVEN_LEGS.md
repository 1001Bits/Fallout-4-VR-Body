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

## The sync is first-person arms onto the body — not animated legs

The map is built by `PlayerCharacter::Generate1stTo3rdBoneMap` (`0x00f09150`), and the
member it fills is literally named `boneMapping1stTo3rd`, a `BSTArray<BSTTuple<int,int>>`:

```c
void Generate1stTo3rdBoneMap(this, node, tree1st, tree3rd, excludeNode)
{
    if (node->collisionObject != nullptr && node != excludeNode) {
        idx3rd = BSFlattenedBoneTree::GetBoneIndex(tree3rd, node->name);
        idx1st = BSFlattenedBoneTree::GetBoneIndex(tree1st, node->name);
        if (both valid and in range)
            boneMapping1stTo3rd.Add({ idx1st, idx3rd });
    }
    for (child : node->children) Generate1stTo3rdBoneMap(this, child, ...);  // recurse
}
```

It is not a fixed upper-body list — it is a full recursive walk. But a node only
enters the map if it satisfies **all three** of: it has a non-null `collisionObject`,
it is not the excluded node, and **its name resolves in both bone trees**.

That last condition is the ceiling. The tuple order `{idx1st, idx3rd}` matches how
`vf023` consumes it — first element indexes the source tree, second the destination —
so the data flows **first-person → third-person**. The map therefore cannot contain
anything absent from the *first-person* skeleton, which in Fallout 4 is arms and hands,
not legs. FRIK's own code agrees: it only ever looks up `RArm_Hand` / `LArm_Hand` in
`getFirstPersonSkeleton()` and takes every leg bone from the third-person root instead.

**Conclusion: this mechanism copies the animated first-person arm pose onto the
visible body's arms. It will never deliver animated legs.** A procedural gait remains
necessary in Fallout 4 VR, so investment belongs in `Skeleton::walk()` rather than in
trying to switch this path on.

This is the substantive difference from Skyrim, where VRIK gets a fully animated
third-person body for free and only has to correct it.

## Still unresolved

1. **Whether the third-person behavior graph can drive the body directly.** The gate
   bit and `SwitchBehaviorGraph` select which graph runs; whether making the
   third-person graph active causes the third-person skeleton to be animated *directly*
   (rather than merely enabling this arm copy) is untested. This is the only remaining
   route to animated legs, and it is unproven — `SwitchBehaviorGraph` is 2006 bytes,
   not fully decoded, and tears down and rebuilds graph state.
2. **Hook ordering.** Whether FRIK's current hook runs before or after `vf023`.
   This matters regardless: if it runs before, FRIK's arm IK may be getting partly
   overwritten by the arm copy.

## Additional verified addresses

| Symbol | Offset | Size |
|---|---|---|
| `PlayerCharacter::SwitchBehaviorGraph` | `0x00f2a180` | 2006 |
| `PlayerCharacter::DoShow1stPerson` | `0x00f299d0` | 1717 |
| `PlayerCharacter::Generate1stTo3rdBoneMap` | `0x00f09150` | 289 |
| `BSFlattenedBoneTree::GetBoneIndex` | `0x01c20c80` | — |

`PlayerCharacter::boneMapping1stTo3rd` lives at player-relative `0x1238` (pointer) with
its count at `0x1248`; `PlayerCharacter::Set3D` and `vf134` also maintain those fields.

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
