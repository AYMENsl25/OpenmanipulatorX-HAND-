# Digit classifier training and camera test

## Current cube detection and digit reading

The [camera integration guide](YOLO_DIGIT_INTEGRATION.md) now uses the [one-class cube YOLO26 checkpoint](../vision_experiments/checkpoints/cube_only_yolo26_v1/README.md) and the rotation-trained MobileNetV3-Small and ResNet18 digit checkpoints by default. The [saved-frame replay](results/cube_only_yolo26_v1/yolo_saved_frames/) contains four camera frames and prediction records; its digit labels are unverified, so it does not measure accuracy.

## Rotation v2 camera experiment

The downloaded rotation-trained MobileNetV3-Small and ResNet18 checkpoints are in [`checkpoints/rotation_v2/`](checkpoints/rotation_v2/). The completed saved-crop and saved-frame replays, plus the two live camera commands, are documented in [`results/rotation_v2/README.md`](results/rotation_v2/README.md). The earlier checkpoints remain separate.

## Spatial Transformer Network ablation

[`stn_digit_recognition_experiment.ipynb`](stn_digit_recognition_experiment.ipynb) is a separate Kaggle experiment with four arms: MobileNetV3-Small and ResNet18, each with and without a classic affine STN. It leaves the two earlier notebooks and their checkpoints alone. See [`STN_EXPERIMENT.md`](STN_EXPERIMENT.md) for run settings, outputs, and evaluation limits.

## Rotation retraining notebook

[`digit_rotation_experiment.ipynb`](digit_rotation_experiment.ipynb) is a separate Kaggle experiment derived from the original [`digit_classifier_comparison.ipynb`](digit_classifier_comparison.ipynb). It trains only MobileNetV3-Small and ResNet18. Training images for digits 1–5 receive random in-plane rotation from -180° to +180° on an expanded canvas, which preserves the full digit at diagonal angles. The rotation run also shifts crops up to 12% horizontally and vertically and varies scale from 0.85 to 1.15. Digits 6 and 9 keep the original mild rotation because their identity can swap under a half-turn. Checkpoint selection uses the mean of upright validation macro F1 and fixed-angle validation macro F1 for digits 1–5. The test split remains untouched. Outputs go to `/kaggle/working/digit_rotation_experiment/`, separate from prior results.

Attach the same complete `DIGIT_CLASSIFICATION_V1` Kaggle dataset and enable GPU and Internet for the pretrained weights. The rotation notebook now defaults to `SMOKE_TEST=False` for full training. Run it from top to bottom. For a short pipeline check first, temporarily set `SMOKE_TEST=True`; restart the session and restore `False` before full training. `RUN_FINAL_EVALUATION=False` keeps the original test split unopened in this experiment. Download `mobilenetv3_small/best.pt` and `resnet18/best.pt` after training, along with each `metrics/history.csv` and `rotation_validation_summary.csv`, and test both on independently labeled real-camera cube crops before selecting a model. The saved three cube-1 frames illustrate the rotation failure but are not enough to estimate general accuracy. In-plane rotation alone cannot solve cube tilt, blur, occlusion, or ambiguous 6/9 faces.

After downloading the rotation-trained weights into separate local folders, the YOLO camera test accepts them with `--mobile-checkpoint` and `--resnet-checkpoint`. Keep the original `checkpoints/` files so you can compare original and retrained models on the same saved scenes.

The rotation notebook retrains on the mostly SVHN/Printed public crop dataset. Fine-tuning on a larger labeled set of actual cube-camera crops is still likely to address the camera domain gap better; see [`YOLO_DIGIT_INTEGRATION.md`](YOLO_DIGIT_INTEGRATION.md).

## Test both checkpoints with the camera

From the workspace root, in a Python environment with `torch`, `torchvision`, `opencv-python`, `Pillow`, and `numpy` installed:

```powershell
python digit_model_experiments/test_digit_camera.py --index 0
```

Try `--index 1` if camera 0 is not the Piranha/robot camera. A window shows both predictions on the **same crop**. Drag the yellow box tightly around one digit; include the digit's full strokes and some surrounding paper, and exclude neighboring digits. Press `R` to rotate the selected crop by 90 degrees counterclockwise and test its orientation. Press a number key `0`–`9` to save the crop with its true label, `S` to save without a label, or `Q` to quit. No robot commands are sent. The cropped PNG, camera frame, both predictions, and selected rotation are saved under `camera_tests/`.

To examine several cubes across the **whole camera frame**, run the experimental detector:

```powershell
python digit_model_experiments/test_multi_cube_camera.py --index 1
```

It looks for bright, nearly white cube faces and classifies each candidate separately at 0, 90, 180, and 270 degrees. Green boxes mean both models predict the same digit; orange boxes mean they disagree. Press `S` to save raw and marked frames. If faces are missed, try `--white-threshold 125`; if background patches are detected, try `--white-threshold 170` or increase `--min-area`. These are provisional predictions: a high softmax score across four rotations can still be wrong, and 6/9 are intrinsically orientation ambiguous without a face orientation cue. Validate several frames manually before using any result to choose a robot action.

For the proposed **YOLO cube detector followed by digit classification**, see [`YOLO_DIGIT_INTEGRATION.md`](YOLO_DIGIT_INTEGRATION.md) and `test_yolo_digit_camera.py`. The camera-fine-tuned YOLO checkpoint is now stored in `../vision_experiments/checkpoints/`.

After capturing several examples of each digit under different lighting and positions, run:

```powershell
python digit_model_experiments/summarize_camera_test.py
```

This reports accuracy on labeled captures and lists model disagreements. The on-screen percentage is the model's softmax score, not a guarantee of correctness. The first test requires a person to select each digit crop; the wide-area script attempts automatic face detection but has not been validated on a full multi-cube scene. Neither script has been run against the user's physical camera in this workspace.

The completed 2026-10-04 Kaggle run and its model decision are in [`results/`](results/MODEL_DECISION.md). The clean notebook below is the reusable experiment source. Both trained checkpoints are now under `checkpoints/`.

`prepare_digit_dataset.py` is the dataset builder; from the workspace root run `py digit_model_experiments/prepare_digit_dataset.py`. Its prepared output is stored separately in `digit_classification_dataset/`. The builder refuses to overwrite the existing dataset.

Open [digit_classifier_comparison.ipynb](digit_classifier_comparison.ipynb) in Kaggle and attach the existing `DIGIT_CLASSIFICATION_V1` dataset. The notebook accepts either the dataset folder or a ZIP containing that folder. It does not build crops or change the source data.

## Run order

1. Enable a GPU. Enable Internet for the official ImageNet weights used by MobileNetV3-Small and ResNet18, or provide a Kaggle environment where the weights are cached. The notebook stops if weights are unavailable.
2. Run with `SMOKE_TEST=True`. This uses a seeded, stratified 10% of train, keeps validation intact, saves checkpoints, and does not evaluate test.
3. For full training, set `SMOKE_TEST=False`. The default `MODELS_TO_RUN` trains all three models and then evaluates test. If the session is too short, set `MODELS_TO_RUN` to one model and `RUN_FINAL_EVALUATION=False` for each training session. Save each run's `digit_model_experiments` output as a Kaggle Dataset.
4. For a separate final comparison session, attach those three output datasets, set `MODELS_TO_RUN=[]`, set `EXTERNAL_CHECKPOINT_ROOTS` to the three attached `digit_model_experiments` directories, and set `RUN_FINAL_EVALUATION=True`. The notebook rejects smoke checkpoints for this comparison.

The final output is under `/kaggle/working/digit_model_experiments`. It includes `best.pt` and `last.pt` for each model, preflight files, source-specific metrics with sample counts, predictions, confusion matrices, error galleries, latency, `model_comparison.csv`, and `final_model_report.md`.

The attached dataset has 357,793 metadata rows: 325,090 train, 6,950 validation, and 25,753 test. Only 27 test crops are from Printed Digit Detection. The earlier complete image-read validation was stopped at the user's request. The notebook performs a lightweight preflight and flags suspect crops without deleting them. No model has been trained locally.

The GitHub repository contains the preparation code, this notebook, and lightweight dataset summaries. It does not contain the 357,793 crop PNGs or the 64 MB `metadata.csv`. For Kaggle, upload the **complete** `DIGIT_CLASSIFICATION_V1` folder, including `train/`, `val/`, `test/`, and `metadata.csv`. An upload containing only the manifest and reports will fail preflight. A ZIP of the complete folder is supported by the notebook.
