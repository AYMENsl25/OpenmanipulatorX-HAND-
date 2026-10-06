# OpenMANIPULATOR-X XR Lab workspace

The root contains project folders only. Start with the workstream that matches your task.

| Tier | Workstream | Files and purpose |
| --- | --- | --- |
| 1 — digit recognition | [`digit_model_experiments/`](digit_model_experiments/README.md) | Dataset preparation, clean training notebook, saved Kaggle run, and model decision. Prepared crops and upload ZIP are in [`digit_classification_dataset/`](digit_classification_dataset/). |
| 2 — camera and object vision | [`vision_experiments/`](vision_experiments/README.md) | Camera checks, OpenCV cube detection, scan XYZ preview, shape capture and labeling, tests, and vision research. Captures and calibration live in [`data/`](data/), [`dataset/`](dataset/), and [`calibration/`](calibration/). |
| 3 — robot analysis | [`robot_experiments/`](robot_experiments/README.md) | Pick and place notes, box TCP error analysis, inverse/forward kinematics research. |
| 4 — robot software | [`openmanipulator_project/`](openmanipulator_project/), [`openmanipulator_repeatability_project/`](openmanipulator_repeatability_project/), [`openmanipulator_trajectory_project/`](openmanipulator_trajectory_project/), [`calibration_rebuild/`](calibration_rebuild/) | Control applications, motion experiments, repeatability work, and calibration rebuild. |
| 5 — reports and physical assets | [`weekly_project_reports/`](weekly_project_reports/), [`hardware_assets/`](hardware_assets/) | Weekly reports and camera stand model. |
| 6 — generated app files | [`build/`](build/), [`dist/`](dist/) | Build intermediates and packaged releases. |

## Current digit decision

The completed run is [`digit_model_experiments/results/2026-10-04_kaggle_full_comparison.ipynb`](digit_model_experiments/results/2026-10-04_kaggle_full_comparison.ipynb). Its [model decision](digit_model_experiments/results/MODEL_DECISION.md) recommends the Kaggle `mobilenetv3_small/best.pt` checkpoint for first integration. ResNet18 has the highest validation score but was not the notebook's provisional deployment pick.
