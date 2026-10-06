# Rotation-trained digit models: camera experiment

The source is `results version 2 rotation traniing.zip`. The new `best.pt` files were extracted to `checkpoints/rotation_v2/{mobilenetv3_small,resnet18}/best.pt`; the older checkpoints under `checkpoints/{mobilenetv3_small,resnet18}/` remain available for comparison. `extraction_manifest.json` records sizes and SHA-256 hashes. Both new checkpoints identify a full, non-smoke rotation experiment with 128-pixel input and best epochs 10 and 19.

## What has been tested locally

The three archived digit crops were classified directly without YOLO. Both new models predicted `1` on all three. The additional `saved_crop_rotation_sweep.csv` contains predictions for 0°, 90°, 180°, and 270° software rotations of each crop; both models predicted `1` on all 12 transformed views. These are correlated views of one physical cube, with `true_digit` still blank in the capture CSV, so they are diagnostic observations rather than an accuracy estimate.

The four archived raw frames were then run through the existing YOLO cube model followed by the new classifiers. YOLO found one cube in each frame and both classifiers predicted `1` on every saved digit crop. The labeled overlays and `yolo_digit_predictions.csv` are in `yolo_saved_frames/`. The older classifiers predict `4` or `6`/`2` on two of these four new YOLO crops; see `yolo_saved_frames/prior_checkpoint_predictions.csv`. A later 28-frame live test exposed tracking and detector misses; see `camera_tests/rotation_v2_yolo/DETECTION_REVIEW.md`. The live script now defaults to direct per-frame detection.

## Stage 1: digit classifiers with a live camera

From the project root in PowerShell:

```powershell
& .\.venv\Scripts\python.exe digit_model_experiments\test_digit_camera.py `
  --index 1 --device auto `
  --mobile-checkpoint digit_model_experiments\checkpoints\rotation_v2\mobilenetv3_small\best.pt `
  --resnet-checkpoint digit_model_experiments\checkpoints\rotation_v2\resnet18\best.pt `
  --output-dir digit_model_experiments\camera_tests\rotation_v2_standalone
```

If camera 1 does not open, use `--index 0`. Drag the yellow box around one visible cube face with the entire digit and a little margin. The model receives the selected crop; there is no detector in this stage. Rotate the physical cube through several orientations, distances, and lighting conditions. Press the actual digit key `0`–`9` to save a labeled crop and frame. `R` rotates the selected image crop in software for a diagnostic check, `S` saves without a label, and `Q` quits. For model comparison, collect separate physical rolls; do not count repeated software rotations as independent scenes.

## Stage 2: YOLO cube boxes followed by the digit models

After reviewing Stage 1, run:

```powershell
& .\.venv\Scripts\python.exe digit_model_experiments\test_yolo_digit_camera.py `
  --yolo vision_experiments\checkpoints\robotic_E1_camera_finetune_best.pt `
  --index 1 --cube-class cube --detector-mode predict --yolo-confidence 0.25 --inner-scale 0.72 `
  --mobile-checkpoint digit_model_experiments\checkpoints\rotation_v2\mobilenetv3_small\best.pt `
  --resnet-checkpoint digit_model_experiments\checkpoints\rotation_v2\resnet18\best.pt `
  --output-dir digit_model_experiments\camera_tests\rotation_v2_yolo_predict
```

Start with one cube, then try four or five cubes at random positions and orientations. The green outer box is YOLO's cube detection; the cyan inner box is what the digit classifiers actually see. `S` saves the raw frame, labeled overlay, per-cube digit crops, and two CSV files. In `frames.csv`, fill `true_cube_count` and a `scene_id` for each saved frame, including frames with no detections. In `cube_crops.csv`, fill `true_digit` and the same `scene_id` for each detected cube. Direct detection numbers cubes from left to right within each frame, so these numbers are not persistent identities. Set `--detector-mode track` only when persistent IDs are needed, then check whether it suppresses visible cubes. Press `Q` to quit.

Judge the stages separately: whether YOLO finds every visible cube, whether the cyan crop contains the complete top-face digit, and whether each classifier reads that crop correctly. Repeated frames of one arrangement are correlated; use new rolls and include several digit classes before choosing a model. Prediction percentages are softmax scores, not calibrated guarantees. No robot command is sent by either test.

After collecting and labeling captures, summarize each stage:

```powershell
& .\.venv\Scripts\python.exe digit_model_experiments\summarize_camera_test.py `
  --log-dir digit_model_experiments\camera_tests\rotation_v2_standalone
& .\.venv\Scripts\python.exe digit_model_experiments\summarize_yolo_digit_camera.py `
  --log-dir digit_model_experiments\camera_tests\rotation_v2_yolo
```
