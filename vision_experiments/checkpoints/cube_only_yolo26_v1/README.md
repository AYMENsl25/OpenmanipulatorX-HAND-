# YOLO26 cube-only checkpoint, version 1

`best.pt` was extracted without modification from
`vision_experiments/YOLO 26 cube fine tunned model.zip`, member
`cube_only_yolo26/camera_cubes_v1/weights/best.pt`.

SHA-256: `db29817dc34a5c97082d05a773463ef616be20abd3f972ed1ee1afad8278cbcc`

Ultralytics loads it as a detection model with one class: ID 0 `cube`.
The archive also contains `last.pt`; the live camera pipeline uses the
validation-selected `best.pt`. The older five-shape checkpoint remains at
`vision_experiments/checkpoints/robotic_E1_camera_finetune_best.pt`.

The training archive includes same-session validation plots and metrics.
This does not establish performance on new camera sessions. Test the complete
cube-detection and digit-reading pipeline on fresh rolls before robot use.
