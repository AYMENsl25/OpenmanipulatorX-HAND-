# OpenMANIPULATOR-X XR Lab workspace

The root contains project folders only. Start with the workstream that matches your task.

| Tier | Workstream | Files and purpose |
| --- | --- | --- |
| 1 — digit recognition | [`digit_model_experiments/`](digit_model_experiments/README.md) | Dataset preparation, training notebooks, saved Kaggle runs, and model decisions. The local [`digit_classification_dataset/`](digit_classification_dataset/) holds the full prepared crops and upload ZIP; GitHub holds its summaries and galleries. |
| 2 — camera and object vision | [`vision_experiments/`](vision_experiments/README.md) | Camera checks, OpenCV cube detection, scan XYZ preview, shape capture and labeling, tests, and vision research. Captures and calibration live in [`data/`](data/), [`dataset/`](dataset/), and [`calibration/`](calibration/). |
| 3 — robot analysis | [`robot_experiments/`](robot_experiments/README.md) | Pick and place notes, box TCP error analysis, inverse/forward kinematics research. |
| 4 — robot software | [`openmanipulator_project/`](openmanipulator_project/), [`openmanipulator_repeatability_project/`](openmanipulator_repeatability_project/), [`openmanipulator_trajectory_project/`](openmanipulator_trajectory_project/), [`calibration_rebuild/`](calibration_rebuild/) | Control applications, motion experiments, repeatability work, and calibration rebuild. |
| 5 — reports and physical assets | [`weekly_project_reports/`](weekly_project_reports/), [`hardware_assets/`](hardware_assets/) | Weekly reports and camera stand model. |
| 6 — generated app files | `build/`, `dist/` (local only) | Build intermediates and packaged releases; these are excluded from Git. |

## Current digit decision

The completed run is [`digit_model_experiments/results/2026-10-04_kaggle_full_comparison.ipynb`](digit_model_experiments/results/2026-10-04_kaggle_full_comparison.ipynb). Its [model decision](digit_model_experiments/results/MODEL_DECISION.md) recommends the Kaggle `mobilenetv3_small/best.pt` checkpoint for first integration. ResNet18 has the highest validation score but was not the notebook's provisional deployment pick.

The later [rotation-v2 camera experiment](digit_model_experiments/results/rotation_v2/README.md) tests separate MobileNet and ResNet checkpoints with cube detection. The [cube-only dataset workspace](dataset/cube_only_v1/README.md) contains 400 captured images and model-proposed boxes that still require human review before training.
