# Paper-aligned IK design

This implementation is based on section 3.1 of Mathias Parger's 2018
thesis, *Inverse Kinematics for Virtual Reality*, with the anatomical
head-pivot correction clarified by FRIK's author.

Reference:
`https://diglib.tugraz.at/download.php?id=5c4a48dc5a282&location=browse`

## Coordinate conventions

FRIK stores `RE::NiMatrix3` rotations in the opposite direction to the
usual local-to-world notation used in the paper. Transforming a local
direction into world space therefore uses `world.rotate.Transpose()`.

The configurable HMD pivot offset is defined as:

```
pivotToHmdLocal = anatomical head/neck pivot -> tracked HMD origin
```

For a valid tracked HMD world pose:

```
leverWorld = hmdWorld.rotate.Transpose()
           * (pivotToHmdLocal * hmdWorld.scale)

pivotWorld = hmdWorld.translate - leverWorld
```

The result is computed absolutely each frame. It is never integrated from
rotational deltas and never written back into the camera or tracking nodes.

## Input separation

One coherent HMD transform is sampled once per solver frame. The solver
keeps these concepts separate:

- Raw HMD pose: rendering, head orientation, scopes, and UI.
- Anatomical pivot: body and shoulder anchoring.
- Controller poses: hand targets and weapon interaction.
- Actor/root locomotion: gait and movement compensation.

This prevents head rotation from being mistaken for body translation.

## Normalized body and shoulder estimation

The avatar root remains at scale `1.0`. Parger's scaled calculations are
implemented as dimensionless measurements rather than scene-graph scale:

```
reachRatio = distance(shoulder, hand) / calibratedArmLength
heightRatio = pivotHeightAboveFloor / calibratedStandingHeight
```

Shoulder yaw is estimated from the sum of the independently normalized,
horizontal pivot-to-controller directions. Degenerate or low-confidence
poses retain the previous valid body direction. Head yaw constrains the
result but does not directly drive the torso.

Each clavicle receives a small, soft reach contribution after the shoulder
center has been established. The contribution is based on reach ratio and
is bounded by anatomical limits.

## Two-bone arm solve

The arm solve preserves the tracked hand target and uses:

1. Safe, clamped law-of-cosines triangle geometry.
2. A stable elbow pole initialized from the shoulder/body frame.
3. Hand position in shoulder-local coordinates as the primary elbow cue.
4. Previous-pole continuity near full extension and other singularities.
5. Hand orientation only as a late soft correction outside a wrist-twist
   dead zone.
6. Time-constant filtering derived from frame time.

No cosmetic smoothing is allowed to delay or move the weapon/controller
target.

## Calibration

The HMD lever arm can be estimated from a short rotation sweep while the
user keeps the upper torso approximately stationary:

```
hmdPosition_i = fixedPivot + hmdRotation_i * pivotToHmdLocal
```

The calibrator solves the overdetermined least-squares system, rejects
non-finite/outlying samples, and requires angular coverage on more than
one axis. A yaw-only sweep cannot reliably determine the component along
the yaw axis.

Player height and arm span are solver measurements only. They must never
change the skeleton root scale or weapon transform hierarchy.

## Required invariants

- Pure HMD rotation around a fixed pivot does not translate the body,
  pelvis, feet, or hands and does not start a gait step.
- True HMD/pivot translation passes through without attenuation.
- Every committed transform is finite.
- Reachable arm targets preserve configured segment lengths.
- Unreachable targets use bounded shoulder assistance and soft reach
  limits without NaN, discontinuity, or multi-length stretching.
- Solver behavior remains materially equivalent at 45, 72, 90, and
  120 Hz.
- Tracking loss, recenter, teleport, load, and Power Armor transitions
  reset temporal state instead of producing a one-frame spike.

