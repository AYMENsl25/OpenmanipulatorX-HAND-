# OpenMANIPULATOR-X calibration rebuild

This is deliberately separate from the old controller. The old HOME offsets and
trajectories are not imported.

## Current calibration gate

| ID | RAW min | RAW HOME | RAW max | mathematical min/home/max | direction | status |
|---:|---:|---:|---:|---|:---:|---|
| 11 | 915 | 1941 | 2961 | -90 / 0 / +90 deg | +1 | measured |
| 12 | 2040 | **pending** | 3000 | **pending** | **pending** | motion blocked |
| 13 | 3570 | **pending** | 4062 | **pending** | **pending** | lower endpoint provisional; motion blocked |
| 14 | 71 | **pending** | 995 | **pending** | **pending** | motion blocked |
| 15 | open 1211 | n/a | closed 2672 | gripper only | n/a | measured |

The 20-count inset safe RAW limits are initial conservative software margins,
not validated operating limits. Increase the inset if endpoint testing shows it
is too close to a mechanical stop. Never silently clamp a requested movement.

## Coordinate model

FK uses the official serial chain: J1 axis +Z; J2-J4 axes +Y; joint origins
`(12,0,17)`, `(0,0,59.5)`, `(24,0,128)`, `(124,0,0)` mm; and TCP offset
`(126,0,0)` mm. At the official mathematical all-zero configuration, this code
therefore returns TCP `(286, 0, 204.5)` mm and pitch `0 deg`. This is a model
result, not yet a claim that the reassembled physical robot matches it.

IK accepts X/Y/Z and tool pitch. It solves the two planar elbow branches,
applies nominal ROBOTIS joint limits, and validates every accepted solution by
FK. RAW conversion is a separate piecewise-linear calibration around the
measured HOME, allowing unequal count-per-degree slopes on either side.

## Run the no-motion checks

```powershell
..\venv\Scripts\python.exe offline_tests.py
..\venv\Scripts\python.exe controller.py --test-ik 286 0 204.5 0
```

## Capture the new HOME safely

1. Support the arm against gravity, upload the calibration sketch, and keep a
   hand near power/estop.
2. Run `controller.py --port COMx`, issue `TORQUE_OFF`, manually place the exact
   physical HOME using a square/level or fixture, then issue `CAPTURE_HOME` three
   to five times without moving the arm.
3. Record ID12/13/14 median RAW values. Also photograph and describe the intended
   mathematical HOME pose; HOME does not automatically mean all q values are 0.
4. From HOME, move one joint at a time a small known positive mathematical angle
   (for example +10 deg measured with a digital inclinometer). Record RAW again.
   RAW increasing means direction +1; RAW decreasing means -1.
5. Remeasure every endpoint three times, especially ID13 lower. Define the
   mathematical angle at each endpoint from the official joint-zero convention.
6. Enter those measured fields in `CALIBRATIONS` and the firmware table. Run the
   offline suite, then compare FK against at least five tape/fixture-measured TCP
   poses. Do not enable movement merely because numerical round trips pass.

## Physical go/no-go gate

Automatic movement remains **NO-GO** until J2-J4 HOME, direction and endpoint
joint angles are measured; ID13 lower is repeatable; RAW↔q endpoint/HOME tests
pass; FK matches measured TCP poses; and IK→FK tests pass throughout the intended
workspace. The included firmware intentionally has no automatic motion routine.

## References

- ROBOTIS OpenMANIPULATOR-X specification and URDF explanation
- ROBOTIS `robotis_manipulator` library documentation, including the customized
  OpenMANIPULATOR chain solver and model origins
- ROBOTIS OpenMANIPULATOR-X quick-start documentation

