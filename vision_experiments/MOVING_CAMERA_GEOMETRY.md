# Moving camera localization: diagnosis and next calibration

## What the current program actually does

`scan_cube_xyz_preview.py` maps a detected contour centre through a manually clicked
plate homography. `vision_panel.py` chooses a pose-specific CENTER/LEFT/RIGHT map when
the encoders match; otherwise it rotates the CENTER homography by measured J1.
It does not assume image centre is robot Y=0, and it does not use a fixed
centimetres-per-degree conversion. The transfer is a planar J1 model, not a
measured camera pose. A gripper/hand XYZ readout is not a camera transform.

Two concrete saved-data issues were found:

* `data/vision_calibration/left.json` contains only two reference clicks at about
  J1=+9.93 degrees. Previously, its angle match suppressed the complete CENTER
  map and produced `NONE` even for valid cube detections. Incomplete maps now
  fall back to the CENTER yaw estimate.
* `data/vision_calibration/right.json` records J1=+9.93 degrees, the direction
  labelled LEFT in the scan. A RIGHT profile at that angle is inconsistent with
  `LEFT = J1 +`. Direction-inconsistent profiles are now ignored; new LEFT/RIGHT
  clicks require a verified direction convention.

The recorded larger cube in `total_scan_20260923_154707.csv` illustrates the
remaining uncertainty: its legacy estimate is (241.6, 25.1) mm at CENTER,
(241.4, 16.7) mm at LEFT +20 degrees, and (237.1, 30.6) mm at RIGHT -20 degrees.
These appear to be the same cube from the saved images; the CSV does not contain
a surveyed cube position, so these numbers measure cross-view disagreement, not
absolute accuracy. A cube silhouette centre may not lie on the table or the
cube-top plane. Lens distortion and a change in camera attitude are also absent
from the planar model.

## Target transform chain

For a rigid camera mount, use the right-handed **internal FK frame**:

```text
T_base_camera(q) = T_base_gripper(q) @ T_gripper_camera
pixel -> undistorted camera ray -> base-frame ray -> cube-top plane -> XYZ
```

The project reports physical Y with the opposite sign from internal Y. Convert
the final intersection to the physical frame only after the rigid-body math.
The existing FK already computes the tool direction from J1 and the accumulated
J2+J3+J4 pitch. `forward_kinematics_official_pose_from_joints()` now exposes
that orientation alongside the existing official gripper-frame position;
the position and IK equations are unchanged. Its local X axis follows the last
link, with zero wrist roll. Use this same frame convention for every hand-eye
sample.

`moving_camera_geometry.py` now provides validated-calibration loading,
`T_base_gripper`, `T_base_camera`, and an undistorted pixel-ray intersection.
It accepts only an explicitly validated measured calibration at the exact
camera resolution. No such calibration exists yet, so it cannot output cube XYZ
in the live GUI. The GUI labels the existing plate XY values as legacy estimates.
The ray function requires a pixel on the cube's **top face**; the detector's
silhouette centre is not automatically suitable.

## Measurements needed for the next stage

1. Measure the printed ChArUco square and marker sizes. Capture diverse
   640x480 images with the camera focus/mount fixed; estimate K and distortion,
   then inspect per-view reprojection errors. `generate_charuco_board.py` and a
   75x105 mm SVG already exist. Printing and physical measurement are still
   decisions for the next stage.
2. With a **fixed** target, capture synchronized board images and encoder
   snapshots from several safe arm poses spanning distinct yaw **and pitch**.
   J1-only poses lack sufficient rotation diversity for a robust full hand-eye
   fit. Estimate target-to-camera poses, solve camera-to-gripper with more than
   one suitable solver, and compare held-out target consistency. OpenCV's
   `calibrateHandEye` expects gripper-to-base and target-to-camera inputs and
   returns camera-to-gripper. The current local OpenCV 5.0.0 build does not
   expose that function; select a compatible verified implementation before
   this step.
3. Measure table Z and cube height. `20 mm` is only an approximate cube height.
   Save the measured cube-top Z and confirm which detected pixel lies on that
   plane. Verify the hand-eye camera origin and orientation against held-out
   board views before computing live cube coordinates.
4. Place a cube at several surveyed physical XYZ positions. At each position,
   record CENTER, LEFT, and RIGHT images with encoder snapshots. Use
   `calibration/cube_pose_validation_template.csv`; calculate axis errors and
   cross-view spread for both the legacy estimate and ray-plane result.
   Only activate `HAND_EYE_RAY_PLANE` after those measured errors are acceptable
   at multiple workspace locations. Preview IK/FK above the cube before any
   robot movement; this stage does not command a grasp.

`moving_camera.json` is intentionally absent. The geometry loader requires
`status="VALIDATED"`, `image_size`, `camera_matrix`, `distortion`,
`T_gripper_camera`, measured `cube_top_z_mm`, and `validation_evidence`.
The file must be created from real measurements and review, never filled with
guessed camera offsets or focal lengths.
