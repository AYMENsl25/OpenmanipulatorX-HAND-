# YOLO cube detection + digit reading

## Camera pipeline

1. YOLO detects each cube in the full camera image. The outer green/orange rectangle is its bounding box.
2. A cyan center rectangle inside each cube is cropped for digit classification. Its size is controlled by `--inner-scale`; it is not a second learned detector.
3. Both digit models read that crop. The display attaches the predictions to a YOLO track ID when available. If tracking is unavailable, `cube 1`, `cube 2`, etc. are only left-to-right order in that frame.
4. Green means the two digit models agree, not that the result is correct. Orange means they disagree. No robot action follows from this prototype.

The camera-fine-tuned Robotic Arm YOLO checkpoint is now at `../vision_experiments/checkpoints/robotic_E1_camera_finetune_best.pt`. Its classes include `cube`. It was fine-tuned on one camera session; its 24-image development validation is not an independent final camera test. See the [checkpoint provenance](../vision_experiments/checkpoints/README.md).

```powershell
& .\.venv\Scripts\python.exe -m pip install ultralytics
& .\.venv\Scripts\python.exe digit_model_experiments\test_yolo_digit_camera.py --yolo '.\vision_experiments\checkpoints\robotic_E1_camera_finetune_best.pt' --index 1 --yolo-confidence 0.25
```

If the cyan crop cuts off a digit, increase `--inner-scale` toward `0.9`. If it includes neighboring cube faces or too much background, decrease it toward `0.6`. `--rotation-search` tries four image orientations, but the highest softmax score can still be wrong. Press `S` to save the raw and labeled frame plus a separate digit crop for every cube in `camera_tests/yolo_digit/`. The `cube_crops.csv` file records predictions and an empty `true_digit` column for human labeling. Do not use model predictions as training labels without checking the image.

## Rotation retraining experiment

Fine-tune the existing checkpoints using **labeled real camera crops** from the numbered cubes. Fill `true_digit`, `scene_id`, and `split` in `camera_tests/yolo_digit/cube_crops.csv`. Use `train`, `val`, or `test` for the split. Assign the same `scene_id` to all images of one physical cube arrangement and keep each scene entirely in one split. The training script rejects a scene that crosses splits. Test rows are never opened during training.

The prepared `finetune_cube_digits.py` script trains both downloaded classifiers with random in-plane rotation from -180 to +180 degrees on **train only**. It also applies modest perspective and brightness/contrast changes to train. Validation is unaugmented. Once labeled scenes exist, compare an otherwise identical no-rotation run with the rotation run:

```powershell
& .\.venv\Scripts\python.exe digit_model_experiments\finetune_cube_digits.py --rotation-degrees 0
& .\.venv\Scripts\python.exe digit_model_experiments\finetune_cube_digits.py --rotation-degrees 180
```

Each run saves validation-selected `best.pt` checkpoints and a history under `camera_finetuning/rotation_0/` or `rotation_180/`. The script refuses to overwrite an existing run. Compare on untouched, manually labeled rotated and upright real-camera **test** scenes after model selection. Report per-digit accuracy and cube-level detection-plus-reading accuracy, not just crop accuracy.

For a first pilot, use unambiguous labels such as 1-5 on the cubes. A rotated 6 can become a 9 and vice versa, so a digit-only orientation-invariant classifier cannot resolve that pair without an orientation mark or another cue. Repeated frames of the same cube arrangement are not independent test examples.

The YOLO cube box can contain a side face, shadow, or more than one number. The center crop is a first test, not a final digit locator. If failures cluster around off-center digits, add a face/digit detector or face segmentation inside each YOLO cube box before robot integration. Validate track IDs after rolling; they can change when a cube is occluded or redetected.
