# STN digit recognition experiment

Open [`stn_digit_recognition_experiment.ipynb`](stn_digit_recognition_experiment.ipynb) in Kaggle. Attach the **complete** `DIGIT_CLASSIFICATION_V1` dataset or its ZIP, enable a GPU, and enable Internet for the official torchvision ImageNet weights. The notebook accepts both ZIP layouts: files directly at the archive root and files wrapped in a `DIGIT_CLASSIFICATION_V1/` folder. Run all cells in order. The default settings train all four models on the full training split and then evaluate the existing test split.

| Arm | Classifier | STN |
|---|---|---|
| `mobilenet_baseline` | MobileNetV3-Small | No |
| `mobilenet_stn` | MobileNetV3-Small | Yes |
| `resnet18_baseline` | ResNet18 | No |
| `resnet18_stn` | ResNet18 | Yes |

The four arms use the same full-circle training augmentation, including digits 6 and 9, and the same validation selection rule. The STN starts as identity, transforms unnormalized image pixels, and is trained by classification loss. The notebook monitors its affine matrix each epoch and warns about possible collapse. The old comparison and rotation notebooks remain separate.

For a quick pipeline check, set `SMOKE_TEST=True` before starting; it uses 5% of train for two epochs and skips test. Restart the session and restore `False` for full training. Four full runs may exceed a Kaggle session. To train them separately, set `MODELS_TO_RUN` to one or two arm names and `RUN_FINAL_EVALUATION=False`. Save each session's `stn_digit_experiment` output as a Kaggle Dataset. In a final session, attach those outputs, set `MODELS_TO_RUN=[]`, populate `EXTERNAL_CHECKPOINT_ROOTS` with the attached `stn_digit_experiment` directories, and set `RUN_FINAL_EVALUATION=True`.

Final output is under `/kaggle/working/stn_digit_experiment/`. Save the four `best.pt` checkpoints, `stn_model_comparison.csv`, `rotation_robustness.csv`, `stn_theta_statistics.csv`, the before/after and failure images, and `final_stn_report.md`. Check `source_metrics.csv` for sample counts; the Printed test set is small. Fixed-angle testing repeats the same source images at multiple angles, so angle rows are correlated.

This experiment tests whether an affine STN helps classification on the prepared public digit crops. Affine correction cannot fully undo perspective distortion, and a half-turned 6 can resemble 9 without an orientation cue. Select a model for the robot only after testing on independently labeled cube-camera scenes.
