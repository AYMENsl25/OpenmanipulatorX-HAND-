# YOLO26 cube fine-tuning export

Generated from `dataset/raw/numbered_cubes_day1/`,
`dataset/cube_only_v1/reviewed_labels/`, and `reviewed_manifest.csv` using
`vision_experiments/prepare_cube_yolo_finetune.py`. The source files were not
changed. Use `data.yaml` in `vision_experiments/yolo26_cube_finetune.ipynb`.

GitHub stores the raw images once, the reviewed source labels, and the split
manifest. The generated `images/`, `labels/`, and machine-specific `data.yaml`
are local training exports. Re-run the preparation script in a fresh output
directory to generate them for a new machine.

Current verified inventory: **400 images, 1,593 cube boxes**. The manifest
contains 99 `reviewed` images and 301 `ai_reviewed` images. The additional 100
images mentioned by the user are not yet located in this workspace and are not
included in this export.

The first 320 images in capture order are training; the last 80 are validation.
This keeps consecutive near-duplicate frames together better than a random
frame split, but both sets come from the same camera session. Treat validation
metrics as development feedback. Evaluate the final model on another session.

One class is defined: ID 0 = `cube`. The source YOLO26 checkpoint has five
shape classes; fine-tuning adapts its detection head to the one-class dataset.
The original checkpoint remains available for future five-shape work.
