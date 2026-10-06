# Live camera and scan preview (Stages A-D)

This update extends the existing OpenCV detector and repeatability GUI. Grasping remains disabled. Discrete camera scanning is available only behind encoder, torque, configured-direction, endpoint-clearance, motor-limit, and confirmation gates.

## Why the old Y display was wrong

The earlier preview assigned guessed pixels to the four physical corners. That treated a convenient image footprint as the complete plate and produced plausible numbers even though the camera is off-centre. That mapping is removed. The program never uses image centre as robot `Y=0`, and never assumes equal left/right coverage.

Each fixed scan pose has measured correspondences:

```text
pixel (u, v) <-> physical plate (X mm, Y mm)
```

At four or more non-collinear points, `cv2.findHomography` creates that pose's mapping and `cv2.perspectiveTransform` maps a valid cube centre. Until then, the UI says `XY UNCALIBRATED`. After four clicks it says `PROVISIONAL XY`: this planar preview is not hand-eye calibration or a grasp target.

Configured physical references are measured from the **rear face of ID11**:

```text
P1 = (131, +130) mm    P2 = (295, +130) mm
P3 = (131, -92) mm     P4 = (295, -92) mm
P5 = (295, 0) mm       P6 = (180, +78) mm
P7 = (260, +78) mm     P8 = (180, -50) mm
P9 = (260, -50) mm
```

These are workspace facts, not image coordinates. The operator frame is
`ID11_REAR_FACE_ESTIMATED`; X in the unchanged robot-axis frame is `X_rear -
25 mm` (provisional housing-based distance). The GUI displays both. Never
click a point unless its matching physical mark is known. Extra measured
reference points can be added with `ADD EXTRA POINT`.

Each calibration is bound to the four encoder joint values at which the red marks were clicked. A fixed-pose mapping applies within 0.1° of that pose. With a measured CENTER mapping, views at other joint-1 angles use the encoder angle to rotate the plate-plane mapping about the robot base. This works only while the camera stays rigid and joints 2–4 remain within 1° of the CENTER calibration; the sweep is limited to 45° from CENTER. The UI and CSV label this `ROTATED_CENTER`, an estimate requiring physical validation. Cube height and parallax are not solved by this planar mapping.

In homogeneous XY coordinates the moving-view map is `H(q1) = T^-1 R(q1-q1_center) T H_center`, where `T` composes the rear-to-axis X translation with the unchanged axis-to-internal transform. This rotates around the ID11 axis, not around the rear-face origin. Physical Y is inverted in the internal frame, so positive joint 1 turns the mapped point toward negative physical Y. The transform uses the *measured* joint angle after each move, including intermediate scan steps.

## Run the integrated GUI

From the project root:

```powershell
cd .\openmanipulator_repeatability_project\python_app
..\..\venv\Scripts\python.exe .\main.py
```

Open tab **4 Live Camera and Scan Preview**.

1. Enter the Piranha camera index (default `1`) and click **START CAMERA**.
2. Connect to OpenCR and use **READ ANGLES NOW**. Position/FK information is now always visible beside the camera.
3. The CENTER/LEFT/RIGHT box under **Fixed-view XY calibration** selects a calibration profile only. It does not move the robot.
4. Select a physical red reference, then click its exact red mark in the image. Repeat for four non-collinear references without moving the hand. Clicking again replaces that point.
5. Confirm the camera footer changes from `XY UNCALIBRATED` to `PROVISIONAL XY`.
6. Click **SAVE CURRENT VIEW CSV** to append detections to `data/vision_scan_logs/vision_snapshots_v3_YYYYMMDD.csv`.

Pose-specific calibration clicks are stored in `data/vision_calibration/center.json`, `left.json`, or `right.json`. A fixed LEFT or RIGHT calibration takes precedence when its encoder pose matches. Otherwise the center yaw model supplies a clearly labelled plate-plane estimate.

## Object status

- `VALID`: entire detected box is inside the image. Provisional XY appears only with a current-pose homography or the center yaw model, and a mapped centre inside the reference bounds plus the 10 mm detection-only padding. The image ROI has no inset; a clipped box remains `EDGE_RESCAN`.
- `EDGE_RESCAN LEFT/RIGHT/TOP/BOTTOM`: cube exists but is at an unreliable boundary. X/Y remains `---`.
- `REJECTED`: candidate fails darkness, size, geometry, or clutter filtering. Rejections are counted in details but omitted from the table.

Distinct outlines identify temporary object IDs; they do not classify physical cube color.

## Discrete scan movement

The movement panel deliberately separates motion from calibration.

1. Use **MOVE TO CENTER SCAN** first.
2. Establish whether physical camera LEFT is `J1 +` or `J1 -`; select that measured convention. Do not guess it from the image.
3. Enter a small step, normally 10 degrees. **PREVIEW STEP** uses existing FK and cannot move.
4. Enter the verified half-span, physically clear the whole swept volume and endpoints, then check the clearance confirmation. Motion is blocked if J2-J4 are not near the named camera SCAN posture or a step would exceed that span.
5. **MOVE CAMERA LEFT/RIGHT** executes one discrete step using current encoders and the existing motor-limit checks. It asks for confirmation first.
6. Enter a physically verified half-span. **SCAN LEFT AREA**, **SCAN RIGHT AREA**, or **SCAN BOTH AREAS** moves in the entered angle steps from CENTER to the endpoint and back. At each step it stops, settles, captures five fresh frames, keeps detections present in at least three frames, and logs them.
7. **STOP AFTER CURRENT STEP** prevents the next movement; it does not pretend to interrupt a motor command already being completed by OpenCR.

Scan results are written to `data/vision_scan_logs/total_scan_TIMESTAMP.csv`; raw and annotated images are saved beside it. Every result stores pixels, joint angles, axis-frame FK hand XYZ, the provisional rear-frame plate X/Y, an axis-X preview, explicit frame names, and an `xy_method` field (`FIXED_POSE`, `ROTATED_CENTER`, or `NONE`). Provisional object X/Y is saved when the fixed-pose or center yaw map is applicable. If no applicable mapping exists, X/Y stays blank. A single LEFT calibration applies only near its recorded angle; the CENTER model handles intermediate steps when its posture checks pass.

The configured angular span is a sweep plan, not a proof that the camera has seen every square millimetre of the physical plate. Review the returned images for gaps before treating a run as full workspace coverage.

This is a joint-1 yaw transfer for a flat plate. Cube-top XYZ and a general moving-camera transform still require measured camera extrinsics, plate height, and cube height/depth evidence.

## Evidence to return after the first low-span scan

Please send:

1. A screenshot before reference clicks showing `XY UNCALIBRATED`.
2. A screenshot after all clicks showing reference labels and `PROVISIONAL XY`.
3. Six detector screenshots: center, left, right, top, bottom, and two cubes.
4. The **PREVIEW STEP** output and your confirmed `LEFT = J1 +` or `LEFT = J1 -` convention.
5. The CENTER, LEFT, and RIGHT calibration JSON files and total-scan CSV.
6. A held-out fifth point: its known physical X/Y and the displayed provisional X/Y. This is the real mapping-error check.
