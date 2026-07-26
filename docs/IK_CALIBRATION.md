# IK calibration

FRIK keeps physical solver measurements separate from scene-graph scale. `fCalibratedPlayerHeight`,
`fShoulderWidth`, `fLeftArmLength`, and `fRightArmLength` are game-unit inputs to normalized IK math;
they must never be applied to the skeleton root scale.

## In-game capture

Open the existing Body Adjustment screen:

1. Select **Body Height** while standing upright. Keep the shoulders still and slowly look left, right,
   up, and down. Press **Save** to fit the HMD pivot and robust upright height.
2. Select **Arms Length**, stand in a steady horizontal T-pose, and press **Save**. FRIK uses the
   robust wrist-to-wrist controller span and the calibrated shoulder-width ratio to calculate one
   symmetric arm length. Thumbstick adjustment remains available.

Failed or underconstrained captures do not replace prior values. Notifications explain whether more
samples, multi-axis head motion, a steadier torso, or a valid T-pose is needed. **Reset** restores the
embedded conservative defaults for the selected target.

## HMD pivot convention

`fHmdPivotOffsetX/Y/Z` is the vector from the anatomical pivot to the tracked HMD origin in HMD-local
axes: X lateral, Y forward, Z up. Runtime correction is rigid and unsmoothed:

```text
worldPivot = trackedHmdPosition - localToWorldRotation * (worldScale * pivotToHmdOffset)
```

Fallout's runtime `NiMatrix` world rotation uses the opposite mapping, so the capture adapter
transposes it before calling the engine-independent calibration API.

The fitter solves `p_i = c + R_i r` by least squares, then rejects median/MAD outliers and validates
sample count, capture time, angular coverage, normal-matrix conditioning, residual error, tracking
discontinuities, and anatomical bounds. Recenter jumps and long tracking gaps restart capture.

## Migration

INI schema version 17 adds the calibration fields. Existing `PlayerHeight` and `armLength` values seed
the solver-only calibrated height and bilateral arm lengths during migration. Missing or non-finite
values fall back to safe defaults; every new numeric field is range-checked again before persistence.
