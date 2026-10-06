# Box calibration and first cube-pick trial

This is a measurement plan, not permission for autonomous robot or gripper motion.
The current camera mapping estimates positions on the **plate plane** only. A
detected cube's image centre is not necessarily its ground-contact centre, and
the cube's Z coordinate is still unknown.

## Current physical references (mm from the rear of ID11)

| Point | X | Y |
| --- | ---: | ---: |
| P1 | 131 | 130 |
| P2 | 295 | 130 |
| P3 | 131 | -92 |
| P4 | 295 | -92 |
| P5 | 295 | 0 |
| P6 | 180 | 78 |
| P7 | 260 | 78 |
| P8 | 180 | -50 |
| P9 | 260 | -50 |

P1-P4 establish the corner map. P5-P9 are independent checks and extra fit
points. The saved CENTER clicks inspected on 2026-09-24 disagreed with the
corner-only map by P5 **13.6 mm**, P6 **10.3 mm**, P7 **12.4 mm**, P8 **4.9 mm**,
and P9 **19.0 mm**. These values change when the operator re-clicks a point;
read the current GUI table rather than relying on this snapshot. They are
consistency errors, not measured camera accuracy. Check the physical X/Y
measurements, mark identities, click positions, pose stability, and lens
distortion. Four corner clicks alone cannot establish accuracy because a
four-point homography fits those four points exactly.

The app migrates old saved P1-P4 names and the old M1=(295,0) click to P5 in
memory. It does not overwrite the saved calibration until a user records a
new click. The GUI shows each extra point's discrepancy against the corner
map. After new clicks, repeat the independent check at points not used in the
fit before treating the mapped coordinates as accurate.

The camera scan region now uses the full image (the previous 20-pixel inset
was removed), and provisional XY is retained up to 10 mm outside the named
point bounds. That 10 mm extension is **detection only**: it does not expand
the robot's motion limits or make a clipped cube a valid grasp target. A cube
touching the image boundary still receives `EDGE_RESCAN`. A physical 10 mm
extension beyond the image itself cannot be captured without moving the camera
or widening its field of view.

## The provisional rear-of-ID11 X frame

The existing robot FK/IK frame still has X=0 at the ID11 rotation-axis centre;
`BASE_X=12 mm` remains an unchanged FK link dimension. The new named operator
frame uses X=0 at the **rear face of ID11**, matching the supplied P1-P9
measurements and manually entered targets. Because the housing is about 50 mm
deep, the rear-to-axis distance is provisionally set to 25 mm:

```text
X_axis = X_rear - 25 mm
X_rear = X_axis + 25 mm
```

Thus an entered rear X=220 mm previews robot-axis X=195 mm before manual
motion. P1-P9 stay numerically unchanged. This is a coordinate-frame
conversion, **not** a change to link lengths or a demonstrated mechanical
accuracy correction. The 25 mm distance must be checked physically at the
rotation axis; the axis need not be the exact midpoint of the housing.

Old P01-P11 point experiments, saved trajectories, and historical FK logs
retain their original ID11-axis frame. Their numbers must not be mixed with
rear-frame numbers without an explicit conversion. Translating the origin
cannot remove the P5-P9 camera corner-check disagreements: those residuals
are unchanged by shifting every reference X equally.

## Diagnose the physical X discrepancy

The supplied encoder/FK log for requested (220,90) ends at about
(221.2,90.0,-2.0) mm in the **axis-centred robot model**, or about 246.2 mm
from the rear with the nominal conversion. It does not measure the physical
midpoint between the jaws. Fill `observed_jaw_x_mm` in
`calibration/box_point_observations_20260923.csv` with a ruler measurement
**from the rear of ID11** to that midpoint at the same held pose. Also fill Y/Z
if available. The CSV's `fk_x_mm` column retains the pasted axis-centred FK;
the analyzer adds 25 mm when comparing to a rear-origin observation. It already contains FK
for P1, P2, P5, P6, P7 and the 220/90 trial; leave unknown cells blank.

Run `venv\Scripts\python.exe robot_experiments\analyze_box_tcp_error.py` for a **read-only**
comparison. Its constant-X and tool-extension fits are hypotheses only.
Do not add a 20-25 mm correction to vision or FK based on the current log:
for the 220/90 pose, the modeled last link is nearly vertical, so an extra
20 mm along that link would mainly alter Z, not X.

## First cube-pick experiment: gated sequence

1. Record camera pose and fixed-pose plate clicks; resolve the P5 discrepancy
   and check several interior points. A moved camera requires a pose-matched
   calibration or a verified camera model; a CENTER homography cannot simply
   be reused after arbitrary J2-J4 movement.
2. Measure cube width, height, plate Z, the camera intrinsics and camera-to-hand
   transform, and the physical jaw-midpoint TCP. Until these are known, log a
   detected object as pixel location and provisional plate XY, **not** cube XYZ.
3. Measure ID15 jaw opening in mm at several encoder positions, including
   fully open and a safe closed position, and verify current/force limits and
   actual OpenCR support. The active XYZ firmware presently addresses only
   IDs 11-14; it has no ID15 command. An older gripper teleop program is not
   evidence that the current firmware supports it.
4. First trial should be manual and supervised: verify IK and clear approach,
   open ID15 under a calibrated command, approach above the cube, descend in
   small confirmed steps, close to a **measured** opening suitable for that
   cube, and record success/offset. Do not infer a motor angle from cube width
   without a measured jaw-opening curve and collision/force guard.
5. Use `calibration/pick_trial_log.csv` to retain the camera estimate, commanded
   target, final FK, physical observed jaw/cube positions, gripper width and
   result. Keep failures: they are needed to diagnose a systematic error.

No pick-motion or ID15 commands are enabled by this document or the current
camera-calibration update.
