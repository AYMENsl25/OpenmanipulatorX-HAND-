# OpenMANIPULATOR-X Vision Research Resources

This file collects the most useful papers, datasets, libraries, algorithms, repositories, and research strategies found for the next stage of the project.

Current setup:

- OpenMANIPULATOR-X.
- Piranha RGB webcam mounted eye-in-hand.
- Approx. **300 × 300 mm** workspace.
- Approx. **140 × 140 mm** visible region from one current scan pose.
- Initial object: **20 × 20 × 20 mm cube**.
- Existing FK/IK, teaching, repeatability, and robot-position logging.

---

# 1. Previous work closest to this project

## 1.1 OpenMANIPULATOR-X + end-effector camera + learned keypoints

**Paper:**  
https://arxiv.org/abs/2409.13668

**Title:** Keypoint Detection Technique for Image-Based Visual Servoing of Manipulators

Why it matters:

- uses OpenMANIPULATOR-X,
- camera is mounted on the manipulator tip,
- AprilTags are placed on a horizontal plane,
- validates image-based visual servoing,
- creates a dataset while the robot moves through the task space,
- automatically labels corner/keypoint targets,
- increases the dataset with flips and rotations,
- fine-tunes a VGG-19 model pretrained on ImageNet.

Strong idea to reuse:

```text
fiducial geometry
→ automatic dataset collection
→ learned corner/keypoint detector
→ visual correction
```

This is especially relevant to our later `cube corners → solvePnP → visual servoing` stage.

---

## 1.2 OpenMANIPULATOR-X + RGB webcam + homography

**Repository:**  
https://github.com/ShinyoengAn/omx-rl-grasp

Important details:

- OpenMANIPULATOR-X.
- Logitech C920 RGB camera.
- Classical-CV cube localization.
- Pixel-to-robot-base conversion using a homography.
- Perception is separated from control rather than learning one end-to-end system.

This strongly supports our first approach:

```text
detect cube
→ pixel center
→ calibrated mapping
→ robot X,Y
```

The project also notes limitations outside the calibrated region, which is a good reason for us to validate each scan region carefully.

---

## 1.3 OpenMANIPULATOR-X + ArUco + HSV + object localization

**Repository:**  
https://github.com/fengting70/omx_pick_place

Pipeline:

```text
camera
→ ArUco calibration
→ HSV color detector
→ contour centroid
→ 3D localization
→ pick/place
```

Their system uses RealSense depth, but the architecture is still very useful.

For our RGB-only Piranha:

```text
depth lookup
```

can be replaced with:

```text
ray + known work plane
```

---

## 1.4 OpenMANIPULATOR-X + ordinary Full-HD webcam

**Paper PDF:**  
https://sensors.myu-group.co.jp/sm_pdf/SM3232.pdf

The study describes an OpenMANIPULATOR-X RM-X52-TNM with a general Full-HD 1080p webcam placed above the robot gripper.

This is a useful mechanical reference showing that a standard webcam can be used on this exact robot.

---

# 2. Camera calibration resources

## 2.1 OpenCV camera calibration

**Official tutorial:**  
https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html

Provides:

- camera matrix,
- `fx`, `fy`,
- `cx`, `cy`,
- radial/tangential distortion,
- undistortion,
- reprojection error.

Important OpenCV recommendation:
multiple calibration-pattern views are needed; the official tutorial recommends at least 10 good patterns.

Use the exact camera resolution that will be used during experiments.

---

## 2.2 ChArUco calibration

**Official tutorial:**  
https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html

OpenCV recommends ChArUco corners when possible because:

- they are more accurate than plain ArUco marker corners,
- partial board views can be used,
- some occlusion is tolerated.

Good choice for the Piranha calibration.

---

## 2.3 Hand-eye calibration

**OpenCV Calib3D / `calibrateHandEye`:**  
https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html

Eye-in-hand inputs include:

- gripper-to-base transforms for multiple robot poses,
- calibration-target-to-camera transforms for the same poses.

Output:

```text
camera → gripper
```

This is the key transform needed for an arbitrary moving wrist camera.

---

# 3. Eye-in-hand calibration research

## Hand-eye calibration error analysis

**Paper:**  
https://www.mdpi.com/1424-8220/24/1/113

Useful points:

- explicitly studies eye-in-hand calibration,
- uses AR markers,
- compares calibration techniques,
- evaluates reprojection error,
- evaluates actual visual-positioning error.

Important lesson for our experiment:

Do not only save a hand-eye transform.

Also validate it using known robot-space points and report positioning error.

---

# 4. Position/localization algorithms

## 4.1 Homography

**OpenCV tutorial:**  
https://docs.opencv.org/4.13.0/d7/dff/tutorial_feature_homography.html

Good for:

- fixed scan pose,
- planar workspace,
- fast first implementation.

Mapping:

```text
(u,v) image
→
(X,Y) table/robot
```

Main limitation:
it describes one plane. A cube top 20 mm above the table can produce a parallax error.

Recommended role:
**baseline method**.

---

## 4.2 Ray-plane intersection

Best first physically grounded XYZ method for this project.

Inputs:

- calibrated camera matrix,
- undistorted pixel `(u,v)`,
- camera pose relative to robot,
- known physical table plane.

Procedure:

```text
pixel
→ camera ray
→ transform ray to robot frame
→ intersect ray with target plane
→ robot XYZ
```

For the 20 mm cube:

```text
target plane = table plane + 20 mm
```

This method does not need an RGB-D camera.

---

## 4.3 solvePnP

**Official OpenCV documentation:**  
https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html

Purpose:

Estimate object rotation and translation from:

- known 3D points,
- corresponding 2D image points,
- camera intrinsics.

For the cube top:

```text
known square = 20 × 20 mm
4 detected top-face corners
→ solvePnP
→ 3D translation + rotation
```

Recommended later comparison:

```text
homography
vs
ray-plane
vs
PnP
```

---

# 5. Multi-view / active camera research

## 5.1 Multi-View Picking

**Paper:**  
https://arxiv.org/abs/1809.08564

Main idea:

- eye-in-hand camera,
- choose informative views,
- reduce grasp uncertainty caused by occlusion/clutter.

The paper reports:
- 80% grasp success in its clutter test,
- 12 percentage-point improvement over the single-view baseline.

Why it matters here:

Our camera sees only ~14 × 14 cm of a 30 × 30 cm plate, so moving through multiple known views is a natural solution.

---

## 5.2 Active Grasp

**Repository:**  
https://github.com/ethz-asl/active_grasp

Implements closed-loop next-best-view planning for grasp detection.

Hardware in the project:
- Franka arm,
- wrist-mounted RealSense.

Long-term idea for us:

```text
look
→ evaluate confidence/visibility
→ move camera only if needed
→ grasp when uncertainty is low
```

---

# 6. Classical OpenCV baseline

For the first black cube, a neural model is not necessary.

Suggested baseline:

```text
grayscale or HSV
→ threshold
→ morphology
→ contours
→ filter by geometry
→ centroid / minAreaRect
```

Useful OpenCV operations:

```text
cvtColor
inRange
findContours
approxPolyDP
moments
minAreaRect
```

Reason to keep this baseline:

It allows camera geometry and robot motion to be verified independently from deep learning.

---

# 7. YOLO models for transfer learning

Do not train from random initialization.

Fine-tune pretrained weights on the custom Piranha dataset.

PyTorch transfer-learning reference:  
https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial

---

## 7.1 YOLO instance segmentation

**Official docs:**  
https://docs.ultralytics.com/tasks/segment

Outputs:

- class,
- confidence,
- bounding box,
- one binary mask per object,
- mask polygon.

Why it is my preferred first neural detector:

```text
mask
→ robust centroid
→ object pixels for color
→ contour for orientation
```

---

## 7.2 YOLO oriented bounding boxes

**Official docs:**  
https://docs.ultralytics.com/tasks/obb

Output representation includes:

- center,
- width,
- height,
- rotation,
- four box corners.

Useful for:

- rotated cubes,
- grasp orientation,
- comparison with WoodenCube-style annotations.

---

## 7.3 YOLO pose/keypoints

**Official docs:**  
https://docs.ultralytics.com/tasks/pose

Ultralytics supports custom keypoint layouts and recommends pretrained pose weights for training.

Potential cube keypoints:

```text
corner_0
corner_1
corner_2
corner_3
```

Output can feed directly to `solvePnP`.

---

# 8. Small-object detection

The physical cube size is 20 mm, but the detector sees pixels.

Measure cube width/height in pixels at each scan pose.

If the cube becomes too small after network resizing, consider sliced inference.

---

## SAHI

**Repository:**  
https://github.com/obss/sahi

**Quick start:**  
https://github.com/obss/sahi/blob/main/docs/quick-start.md

Concept:

```text
large image
→ overlapping tiles
→ detector per tile
→ merge predictions
```

SAHI is specifically designed for small objects in large images and can work with Ultralytics models.

Use only if normal high-resolution inference struggles.

---

# 9. Datasets

## 9.1 WoodenCube

**Paper/dataset description:**  
https://www.mdpi.com/1424-8220/24/18/5903

Very relevant to our cube problem.

Reported dataset:

- **5113 RGB images**,
- resolution **2448 × 2048**,
- **10 block classes**,
- 30 physical target wooden cubes,
- dense annotations,
- object categories,
- four-corner/rotated annotations,
- rotation information,
- industrial scenes with foreground/background similarity.

Reported annotation style:

```text
x1,y1,x2,y2,x3,y3,x4,y4,class,difficulty
```

The paper also proposes cube-specific evaluation ideas.

Best use for us:

- rotated-cube detection reference,
- annotation strategy,
- model comparison ideas.

Still collect our own Piranha dataset because the domain is very different.

---

## 9.2 HOPE

**Official repository:**  
https://github.com/swtyree/hope-dataset

Contains:

- 28 manipulation-sized objects,
- RGB-D,
- 6-DoF poses,
- textured 3D meshes.

### HOPE-Video

Especially relevant:

- 10 sequences,
- 2038 frames,
- camera mounted on a robotic arm,
- several views of tabletop objects,
- camera intrinsics/extrinsics,
- object pose annotations.

Useful to study:
- moving eye-in-hand vision,
- world/camera coordinate transforms,
- multi-view object pose.

---

## 9.3 BOP

**Official datasets:**  
https://bop.felk.cvut.cz/datasets/

BOP standardizes:

- 2D detection,
- segmentation,
- 6D object pose,
- object models,
- real/synthetic training data.

Datasets include:

- T-LESS,
- YCB-V,
- HOPE,
- LM-O,
- ITODD,
- IC-BIN,
- TUD-L.

Useful later for advanced pose-estimation benchmarking.

---

## 9.4 T-LESS

**Official site:**  
https://cmp.felk.cvut.cz/t-less/

**Download:**  
https://cmp.felk.cvut.cz/t-less/download.html

Contains:

- 30 industrial textureless objects,
- synchronized RGB-D/high-resolution RGB,
- ~38K training images per sensor,
- ~10K test images per sensor,
- clutter/occlusion,
- CAD and reconstructed models.

Why relevant:
plain cubes can also be texture-poor, and symmetry is important in cube pose.

---

## 9.5 YCB Object and Model Set

**Official data site:**  
https://ycb-benchmarks.s3.amazonaws.com/index.html

Provides, depending on object:

- high-resolution RGB,
- RGB-D,
- point clouds,
- textured meshes,
- object models.

This is a standard robotic manipulation reference.

---

# 10. Synthetic-data generation

Synthetic data is attractive because our target geometry is simple.

Randomize:

```text
cube position
cube rotation
cube color
digit
camera pose
lighting
shadows
background
motion blur
```

Generate automatic labels:

```text
mask
bbox
corners
depth
XYZ
rotation
digit
```

---

## 10.1 Kubric

**Repository:**  
https://github.com/google-research/kubric

Built on:

- Blender,
- PyBullet.

Generates rich annotations such as:

- instance segmentation,
- depth,
- optical flow,
- object properties.

Useful for synthetic cube experiments.

---

## 10.2 BlenderProc

**Repository:**  
https://github.com/DLR-RM/BlenderProc

Features:

- object pose sampling,
- physics/collision,
- randomized materials,
- randomized lighting,
- camera poses,
- RGB,
- stereo/depth,
- normals,
- segmentation,
- COCO/BOP output.

Potentially the strongest synthetic-data tool if we want to model the actual cube and workspace.

---

# 11. Advanced 6D pose systems

These are valuable research references but are unnecessary for the first 20 mm cube experiment.

---

## 11.1 NVIDIA DOPE

**Official repo:**  
https://github.com/NVlabs/Deep_Object_Pose

Purpose:

```text
RGB camera
→ detection
→ 6-DoF pose of known object
```

Repository contains:

- training,
- inference,
- evaluation,
- synthetic-data generation,
- ROS support.

Useful as an RGB-only 6D-pose benchmark.

---

## 11.2 MegaPose

**Official repo:**  
https://github.com/megapose6d/megapose6d

Inputs can include:

- RGB image,
- camera intrinsics,
- object mesh,
- object bounding box.

Output:

- 6D object pose.

Depth is optional for the RGB models.

Released synthetic training data:
- about 2 million images,
- more than 20,000 objects.

Useful if we later use CAD/STL models for several objects.

---

## 11.3 FoundationPose

**Project page:**  
https://nvlabs.github.io/FoundationPose/

CVPR 2024.

Supports:

- novel object 6D pose,
- pose tracking,
- CAD-model setup,
- reference-image setup.

This is a high-end reference, not the first implementation.

---

# 12. Annotation tools

## CVAT

**Official docs:**  
https://docs.cvat.ai/docs/

Supports:

- rectangles,
- polygons,
- points,
- tracking,
- cuboids,
- skeletons,
- brush/masks.

Recommended uses:

```text
YOLO segmentation → polygons
YOLO OBB          → rotated object annotation
YOLO pose         → four cube corners
video tracking    → track labels
```

---

# 13. Color and shape strategy

Keep attributes independent:

```text
shape  = cube
color  = red
number = 7
```

Avoid:

```text
class = red_cube_7
```

which creates too many classes.

Color:

```text
segmentation mask
→ HSV pixels inside object
→ median HSV
→ color label
```

Shape:

- classical contours as baseline,
- YOLO object class later.

---

# 14. Number recognition / OCR

## EasyOCR

**Official repo:**  
https://github.com/JaidedAI/EasyOCR

Provides:

- text detection,
- recognition,
- bounding boxes,
- confidence,
- custom recognition models.

For one digit on one cube face, a dedicated 10-class classifier is still simpler.

---

## Tesseract

**Official docs:**  
https://tesseract-ocr.github.io/tessdoc/

Tesseract 5.x is a mature OCR engine.

Recommended only when labels become more complex than one digit.

---

# 15. Tracking moving objects

## 15.1 ByteTrack

**Official repository:**  
https://github.com/FoundationVision/ByteTrack

Strong simple multi-object tracking baseline.

Good when the camera is fixed.

---

## 15.2 Ultralytics tracking

**Official docs:**  
https://docs.ultralytics.com/modes/track

Current docs support several trackers.

Important distinction:

### ByteTrack
- simple and fast,
- no camera-motion compensation.

### BoT-SORT
- camera-motion compensation can be enabled,
- optional ReID.

For this project:

```text
camera fixed in scan pose
→ ByteTrack is a reasonable baseline

camera moving with arm
→ use camera-motion-aware tracker or compensate motion geometrically
```

---

# 16. Recommended experiments for a strong research report

## A. Position algorithm comparison

Compare:

```text
homography
ray-plane
solvePnP
```

Metrics:

- X error,
- Y error,
- Z error,
- XYZ norm,
- runtime.

---

## B. Detector comparison

Compare:

```text
OpenCV contour
YOLO segmentation
YOLO OBB
```

Metrics:

- precision,
- recall,
- pixel-center error,
- robot-space error,
- runtime.

---

## C. Workspace scanning comparison

Compare:

```text
single view
fixed 3 × 3 scan
active/confidence-based scanning
```

Metrics:

- coverage,
- missed objects,
- scan time,
- duplicate detections,
- position accuracy.

---

## D. Static vs moving-camera tracking

Compare:

```text
fixed camera / fixed wrist pose
moving eye-in-hand camera
```

Metrics:

- ID switches,
- lost tracks,
- robot-space position error,
- latency.

---

# 17. Most useful links — quick index

## OpenMANIPULATOR-X

- Keypoint / visual servoing paper  
  https://arxiv.org/abs/2409.13668

- RGB webcam + homography cube project  
  https://github.com/ShinyoengAn/omx-rl-grasp

- ArUco + HSV pick/place  
  https://github.com/fengting70/omx_pick_place

- Full-HD webcam study  
  https://sensors.myu-group.co.jp/sm_pdf/SM3232.pdf

## OpenCV

- Camera calibration  
  https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html

- ChArUco  
  https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html

- Homography  
  https://docs.opencv.org/4.13.0/d7/dff/tutorial_feature_homography.html

- solvePnP  
  https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html

- Hand-eye calibration  
  https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html

## YOLO / training

- Segmentation  
  https://docs.ultralytics.com/tasks/segment

- OBB  
  https://docs.ultralytics.com/tasks/obb

- Pose/keypoints  
  https://docs.ultralytics.com/tasks/pose

- Tracking  
  https://docs.ultralytics.com/modes/track

- Transfer learning  
  https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial

## Small objects

- SAHI  
  https://github.com/obss/sahi

## Datasets

- WoodenCube  
  https://www.mdpi.com/1424-8220/24/18/5903

- HOPE  
  https://github.com/swtyree/hope-dataset

- BOP  
  https://bop.felk.cvut.cz/datasets/

- T-LESS  
  https://cmp.felk.cvut.cz/t-less/

- YCB  
  https://ycb-benchmarks.s3.amazonaws.com/index.html

## Synthetic data

- Kubric  
  https://github.com/google-research/kubric

- BlenderProc  
  https://github.com/DLR-RM/BlenderProc

## Advanced 6D pose

- DOPE  
  https://github.com/NVlabs/Deep_Object_Pose

- MegaPose  
  https://github.com/megapose6d/megapose6d

- FoundationPose  
  https://nvlabs.github.io/FoundationPose/

## Annotation

- CVAT  
  https://docs.cvat.ai/docs/

## OCR

- EasyOCR  
  https://github.com/JaidedAI/EasyOCR

- Tesseract  
  https://tesseract-ocr.github.io/tessdoc/

## Active perception

- Multi-View Picking  
  https://arxiv.org/abs/1809.08564

- Active Grasp  
  https://github.com/ethz-asl/active_grasp

## Tracking

- ByteTrack  
  https://github.com/FoundationVision/ByteTrack

---

# 18. Research conclusion

The strongest strategy for this setup is:

```text
ChArUco camera calibration
→ ArUco workspace references
→ homography baseline
→ real table-plane fit from robot touch data
→ ray-plane XYZ
→ OpenCV cube baseline
→ multi-view scan
→ custom YOLO segmentation transfer learning
→ OBB/keypoint model
→ solvePnP comparison
→ hand-eye calibration
→ color/shape/digit
→ moving-object tracking
```

The main principle is:

**Use machine learning to improve object perception, but use calibrated geometry to generate metric robot coordinates.**

This is easier to validate, easier to debug, and better connected to the FK/IK and repeatability experiments already completed.
