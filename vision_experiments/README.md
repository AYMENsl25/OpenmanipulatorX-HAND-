# Vision experiments

| Experiment | Source and notes | Existing output |
| --- | --- | --- |
| Camera and first cube detection | `camera_test_v1.py`, `cube_detection_opencv_v1.py` | `../data/camera_test/`, `../data/cube_detection/` |
| OpenCV cube revisions | `opencv_cube_test_v1.py`, `_v2.py`, `_v3.py`, `capture_cube_dataset_v1.py` | `../data/opencv_tests*/`, `../dataset/raw/` |
| Scan XYZ and moving camera geometry | `scan_cube_xyz_preview.py`, `scan_cube_xyz_config.json`, `generate_charuco_board.py`, `test_*geometry.py`, `test_*scan*.py`; `SCAN_CUBE_XYZ_PREVIEW.md`, `MOVING_CAMERA_GEOMETRY.md` | `../data/vision_scan_logs/`, `../data/vision_calibration/`, `../calibration/` |
| Real camera shape test | `capture_shape_test.py`, `label_shape_boxes.py`, `evaluate_shape_test.py`; `REAL_CAMERA_SHAPE_TEST.md`, `SHAPE_ANNOTATION_GUIDE.md` | `../dataset/real_camera_shape_test_v1/` |
| Research and planning | `openmanipulator_vision_experiment_plan.md`, `openmanipulator_vision_research_resources.md`, research pack ZIP | Source material only |

Run scripts from the workspace root using `python vision_experiments/<script>.py`. Data paths in the scripts point back to the existing workspace data and dataset folders.

The camera-fine-tuned YOLO26 cube/shape [checkpoint and provenance](checkpoints/README.md) are in `checkpoints/`. The complete supplied YOLO project ZIP is retained in `archives/`.
