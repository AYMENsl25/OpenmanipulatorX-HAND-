# OpenMANIPULATOR-X Vision Experiment Plan

## Project target

Build a vision pipeline for the current OpenMANIPULATOR-X setup with:

- Piranha RGB webcam mounted on the robot hand/end-effector.
- Approximate full work surface: **300 × 300 mm**.
- Approximate camera view at the current scan pose: **140 × 140 mm**.
- Initial object: **20 × 20 × 20 mm black cube**.
- Existing robot work: working FK/IK, repeatability experiment, known workspace points, teaching mode, motor-angle logging, and Excel error analysis.
- Future goals: multiple objects, robot-frame XYZ, shape/color classification, digit reading, sorting, and later moving objects.

---

## 1. Main design decision

Do **not** begin by training one neural network to directly predict robot XYZ.

Use a modular pipeline:

```text
CAMERA IMAGE
     ↓
OBJECT DETECTOR
(OpenCV first, YOLO later)
     ↓
pixel / mask / corners
     ↓
GEOMETRY + CALIBRATION
     ↓
robot-frame X,Y,Z
     ↓
existing IK
     ↓
OPENMANIPULATOR-X
```

This lets us isolate errors from detection, camera calibration, image-to-robot geometry, FK/IK, and mechanics.

---

## 2. Workspace coverage strategy

The plate is about **300 × 300 mm**, while one current hand-camera image sees about **140 × 140 mm**.

A single image covers only about 22% of the plate area.

### Recommended first layout: 3 × 3 scan grid

```text
┌────────────┬────────────┬────────────┐
│   SCAN 1   │   SCAN 2   │   SCAN 3   │
├────────────┼────────────┼────────────┤
│   SCAN 4   │   SCAN 5   │   SCAN 6   │
├────────────┼────────────┼────────────┤
│   SCAN 7   │   SCAN 8   │   SCAN 9   │
└────────────┴────────────┴────────────┘
```

Use **20–30% overlap** between neighboring views.

For a 140 mm field:
- 20% overlap → about 112 mm center-to-center.
- 30% overlap → about 98 mm center-to-center.

Before committing to nine poses, test a higher safe camera pose. If the camera can see around **180 mm or more per side**, test a 2 × 2 layout.

---

## 3. Phase 0 — freeze the mechanical setup

Before calibration:

- rigidly mount the Piranha,
- fix focus,
- fix resolution and frame rate,
- fix camera angle,
- route USB cable so it does not pull the wrist,
- do not move the camera in the bracket after calibration.

Record:

```text
camera_model
resolution
frame_rate
focus_setting
mount_version
camera_angle
```

Save an exact robot scan pose:

```text
SCAN_CENTER
J1 = ...
J2 = ...
J3 = ...
J4 = ...
```

---

## 4. Phase 1 — camera intrinsic calibration

Use a **ChArUco board**.

Goal: estimate

- fx, fy,
- cx, cy,
- distortion coefficients.

Capture around 15–30 good board views:
- center,
- all image corners,
- different distances,
- different tilts.

Use the same camera resolution planned for the robot experiment.

Save:

```text
camera_matrix
distortion_coefficients
image_width
image_height
reprojection_error
```

Reference:  
https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html  
https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html

---

## 5. Phase 2 — add machine-readable workspace references

The white tapes are useful physical points, but vision cannot identify their IDs reliably.

Add small ArUco markers around the plate.

For every marker store:

```text
marker_id
robot_x_mm
robot_y_mm
robot_z_mm
marker_size_mm
```

Keep markers away from the normal cube area.

---

## 6. Phase 3 — verify pixel → robot coordinates before cube detection

Build a simple program:

1. Move to `SCAN_CENTER`.
2. Show live Piranha image.
3. Undistort frame.
4. User clicks a point.
5. Program prints pixel `(u,v)`.
6. Convert it to robot `(X,Y)`.
7. Compare against a known point.

Example:

```text
Pixel:      (842, 516)
Predicted:  X=146.3, Y=-38.7 mm
True:       X=145.0, Y=-40.0 mm
Error norm: 1.84 mm
```

This proves the geometry before adding AI.

---

## 7. Phase 4 — homography baseline

For one fixed scan pose:

```text
pixel (u,v)
   ↓
homography H
   ↓
robot (X,Y)
```

Use OpenCV:

```python
cv2.findHomography()
cv2.perspectiveTransform()
```

Use more than four known points when possible and keep independent validation points.

Reference:  
https://docs.opencv.org/4.13.0/d7/dff/tutorial_feature_homography.html

### Limitation

A homography calibrated on the table plane is not exact for the cube top, which is 20 mm above the table. With a tilted camera this creates parallax.

Therefore homography is a **baseline**, not the final method.

---

## 8. Phase 5 — fit the real physical table plane

Your previous experiments showed that physical table contact does not always correspond to FK `Z=0`.

Use actual contact points:

```text
(X1,Y1,Z1)
(X2,Y2,Z2)
...
```

Fit:

```text
aX + bY + cZ + d = 0
```

This gives the actual work-surface plane in robot coordinates.

For an upright 20 mm cube, define a parallel plane 20 mm above the table.

This directly reuses your earlier repeatability/contact experiment.

---

## 9. Phase 6 — ray/plane intersection

Once intrinsics and camera pose are known, convert a detected pixel into a camera ray.

For pixel:

```text
p = [u,v,1]^T
```

compute:

```text
r_camera = inv(K) * p
```

Transform the ray to robot coordinates and intersect it with:

- the table plane, or
- the cube-top plane.

This gives metric robot XYZ.

### Experiment

Compare:

```text
Method A: homography
Method B: ray-plane
```

at 10–20 known cube locations.

Log X/Y/Z and error norm.

---

## 10. Phase 7 — black-cube detection with OpenCV

Before YOLO, use a simple detector:

```text
frame
 ↓
undistort
 ↓
grayscale or HSV
 ↓
dark threshold
 ↓
morphological cleanup
 ↓
findContours
 ↓
filter by area/aspect ratio/rectangularity
 ↓
minAreaRect or contour centroid
 ↓
pixel center
 ↓
robot XYZ
```

This works well as a transparent baseline because the cube is black and the plate is much brighter.

---

## 11. Phase 8 — safe robot approach test

Do not immediately grasp.

First:

```text
DETECT
 ↓
CALCULATE XYZ
 ↓
MOVE ABOVE CUBE
 ↓
VERIFY
 ↓
DESCEND
 ↓
GRASP
```

Move to a safe Z above the predicted cube center, visually confirm centering, and log error.

---

## 12. Phase 9 — full 3 × 3 multi-view scan

For each scan pose:

```text
move robot
 ↓
wait for settling
 ↓
capture frame
 ↓
detect objects
 ↓
convert detections to robot coordinates
 ↓
store detections
```

Do **not** stitch images unless needed.

Transform all detections directly into the common robot coordinate frame.

---

## 13. Merge duplicate detections

Overlapping scans may detect the same cube twice.

Initial merge rule:

- same class/color,
- robot-space centers within roughly 10–15 mm.

Prefer:
- detection closest to image center, or
- confidence-weighted average.

Keep raw detections for later analysis.

---

## 14. Phase 10 — collect your own dataset

Your own Piranha-camera dataset is the most important dataset.

Capture:
- all scan poses,
- all plate regions,
- many cube rotations,
- center and image edges,
- different lighting,
- 1/2/3+ cubes,
- robot/gripper in frame,
- shadows,
- partial occlusion,
- slight blur.

Start with one class:

```text
cube
```

Prefer roughly **500–1000 diverse real images** rather than thousands of nearly identical consecutive frames.

Suggested split:

```text
train 70–80%
val   10–20%
test  10–20%
```

Split by scene/condition, not random neighboring video frames.

---

## 15. Phase 11 — YOLO transfer learning

Do not train from scratch.

### Best first neural model: YOLO segmentation

Architecture:

```text
YOLO segmentation
   ↓
cube mask
   ↓
mask centroid
   ↓
ray-plane XYZ
```

Official docs:  
https://docs.ultralytics.com/tasks/segment

### Alternative: YOLO OBB

Useful for rotation/orientation.

https://docs.ultralytics.com/tasks/obb

### Later: YOLO pose/keypoints

Train four top-face keypoints:

```text
top_left
top_right
bottom_right
bottom_left
```

https://docs.ultralytics.com/tasks/pose

---

## 16. Phase 12 — solvePnP / square pose

The cube top is a known 20 × 20 mm square.

Use four known model corners:

```text
(-10,+10,0)
(+10,+10,0)
(+10,-10,0)
(-10,-10,0)
```

with their four detected image corners and camera intrinsics.

Use OpenCV `solvePnP()` to estimate translation and orientation.

Reference:  
https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html

Strong experiment:

```text
Homography
vs
Ray-plane
vs
PnP
```

Compare X/Y/Z error and runtime.

---

## 17. Phase 13 — eye-in-hand hand-eye calibration

Long term, avoid storing one homography for every scan pose.

Use `cv2.calibrateHandEye()`.

For many robot poses save:

```text
gripper pose from FK
camera pose relative to fixed ChArUco target
```

Estimate the fixed:

```text
camera → gripper
```

transform.

Then at runtime:

```text
base→gripper  from FK
×
gripper→camera from hand-eye calibration
=
base→camera
```

Reference:  
https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html

---

## 18. Phase 14 — color and shape

Keep attributes separate:

```text
shape = cube
color = red
```

not:

```text
class = red_cube
```

For color:
1. use only pixels inside the detected mask,
2. convert to HSV,
3. use median HSV,
4. map to color label.

For simple shape baselines use:
- contour polygon,
- aspect ratio,
- circularity,
- convexity,
- Hu moments.

For complex shapes, use detector classes.

---

## 19. Phase 15 — digit recognition

If cubes contain one digit 0–9:

```text
detect cube
 ↓
find visible numbered face
 ↓
perspective rectify face
 ↓
crop digit
 ↓
10-class digit classifier
```

A small pretrained MobileNet/ResNet fine-tuned on your digit images is a good first choice.

Only use general OCR when labels become multi-character.

---

## 20. Phase 16 — moving objects

A 9-view sequential scan is not ideal for moving objects.

### Fixed scan pose

If the motion zone fits one view:

```text
YOLO
 ↓
tracker
 ↓
object ID
 ↓
X(t),Y(t)
 ↓
velocity
 ↓
future intercept position
```

ByteTrack is a simple baseline for a fixed camera.

### Moving eye-in-hand camera

When the camera itself moves:
- update camera pose using FK + hand-eye calibration,
- use a tracker with camera-motion compensation.

Current Ultralytics docs specifically point to BoT-SORT for moving-camera footage.

Tracking docs:  
https://docs.ultralytics.com/modes/track

Long-term ideal moving-platform setup:

```text
fixed global camera → continuous tracking
+
wrist camera → close-range correction
```

---

## 21. Active scanning / next-best-view upgrade

Initial system:

```text
fixed 3×3 scan
```

Later:

```text
scan current view
 ↓
confidence high?
object not clipped?
object large enough?
 ↓ yes → use
 ↓ no  → move to another view
```

Possible view-quality score:

```text
score =
 detection_confidence
 + normalized_object_area
 - image_border_penalty
 - occlusion_penalty
```

References:  
https://arxiv.org/abs/1809.08564  
https://github.com/ethz-asl/active_grasp

---

## 22. Log everything

For every detection save:

```text
timestamp
scan_pose_id
camera_resolution
camera_calibration_version
J1,J2,J3,J4
FK camera/gripper pose
object_id
class
confidence
pixel_u,pixel_v
bbox
mask
OBB angle
keypoints
predicted_x,y,z
known/verified_x,y,z
error_x,y,z
error_norm
inference_ms
total_latency_ms
```

---

## 23. Metrics

### Camera calibration
- reprojection error.

### Position
- X error,
- Y error,
- Z error,
- XY/XYZ norm.

### Detection
- precision,
- recall,
- mAP,
- segmentation mAP.

### Robot system
- grasp success,
- detection-to-grasp time,
- error by scan pose,
- error by workspace region.

### Moving object
- ID switches,
- lost tracks,
- velocity error,
- intercept error,
- pick success.

---

## 24. Recommended experiment order

1. **V1 — webcam characterization**
   - resolution/FPS,
   - measure FOV,
   - choose scan grid.

2. **V2 — intrinsic calibration**
   - ChArUco,
   - save intrinsics.

3. **V3 — click-to-XY**
   - one scan pose,
   - known points,
   - homography.

4. **V4 — ray-plane**
   - fit real table plane,
   - compare against homography.

5. **V5 — OpenCV black-cube detection**
   - contour centroid,
   - camera → robot XYZ.

6. **V6 — safe robot approach**
   - move above cube,
   - measure centering error.

7. **V7 — full multi-view scan**
   - cover 300 × 300 mm,
   - merge detections.

8. **V8 — YOLO transfer learning**
   - segmentation first.

9. **V9 — OBB / keypoints / PnP**
   - orientation and 6D pose.

10. **V10 — color / shape / digit**

11. **V11 — hand-eye calibration**

12. **V12 — moving-object tracking and intercept**

---

## 25. Immediate next code

Build:

```text
Experiment_CameraMapping_V1.py
```

It should:

1. open the Piranha webcam,
2. undistort the image,
3. detect ArUco references,
4. allow a mouse click,
5. display `(u,v)`,
6. convert to robot `(X,Y)`,
7. log the result,
8. show mapping error on known points.

After that works, add the black cube detector.

Only after geometry is validated should YOLO training begin.

---

## Key references

- OpenCV Camera Calibration  
  https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html

- OpenCV ChArUco  
  https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html

- OpenCV Homography  
  https://docs.opencv.org/4.13.0/d7/dff/tutorial_feature_homography.html

- OpenCV solvePnP  
  https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html

- OpenCV Hand-Eye  
  https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html

- OpenMANIPULATOR-X keypoint/IBVS paper  
  https://arxiv.org/abs/2409.13668

- OpenMANIPULATOR-X webcam/homography project  
  https://github.com/ShinyoengAn/omx-rl-grasp

- OpenMANIPULATOR-X ArUco/HSV pick-place  
  https://github.com/fengting70/omx_pick_place

- Multi-View Picking  
  https://arxiv.org/abs/1809.08564

- Active Grasp  
  https://github.com/ethz-asl/active_grasp
