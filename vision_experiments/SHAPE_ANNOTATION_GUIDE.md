# Shape box labeling — v1.0

## What to label

Label **every visible physical object** in each image. One object gets one tight, axis-aligned rectangle around its visible body, including visible top and side faces. Leave shadows, red tabletop marks, cables, and the table outside the box. Rotated objects still use an ordinary horizontal/vertical rectangle. For a partially cut-off object, box the visible portion up to the image edge. If an object's shape cannot be established from the photo **and** you do not know which physical object was placed, skip that whole image until its identity is confirmed; do not guess.

| ID | Shape | Use when | Do not use for |
|---:|---|---|---|
| 0 | cube | The physical cube, black or green, at any rotation | Pyramid, dark shadow |
| 1 | cylinder | The physical cylinder, including a dark overhead view | Sphere |
| 2 | sphere | The physical round ball | Cylinder |
| 3 | pyramid | The physical 3-D pyramid | The public dataset's `triangular` or `diamond` class without physical confirmation |

Color is a separate per-object field: currently **black** or **green**. The YOLO shape class does not change with color. The five example images include a clear green cube; the black shapes' identities should be confirmed from what you placed on the table.

## Run the manual labeler

From `C:\Users\slima\Downloads\The Robotic Hand ISU XR LAB`:

```powershell
& '.\venv\Scripts\python.exe' .\vision_experiments\label_shape_boxes.py --session day1_normal --start 77
```

`--start 77` opens the 77th image in filename order; omit it to start from the first image. The display zoom defaults to 1.5. The saved coordinates still refer to the original **640 × 480** pixels.

1. Press **1/2/3/4** for cube/cylinder/sphere/pyramid.
2. Press **B** for black or **G** for green.
3. Drag the mouse from one corner of the object to the opposite corner. Repeat for **each** object; switch class and color as needed.
4. Press **S** to save that image and advance. **U** undoes the last box; **C** clears boxes; **P** goes back; **K** skips an uncertain image; **Q** quits. Press **E** only for a confirmed empty scene.

The tool writes `labels/day1_normal/<same-image-stem>.txt` with YOLO `class x_center y_center width height` values normalized to 0–1. It also writes `object_annotations.csv` with per-object color and pixel corners, and corrects each saved image's `capture_manifest.csv` row. The original manifest is backed up once as `capture_manifest_original.csv`. The raw JPEGs are unchanged.

Example only: a cube whose visible box is `(225, 205)` to `(305, 315)` in a 640 × 480 image would produce approximately:

```text
0 0.41406250 0.54166667 0.12500000 0.22916667
```

The rectangle must include all visible faces of the cube but not its cast shadow. Draw the actual box in the labeler rather than copying these example coordinates.

## These five images

- `s001_080...jpg`: one visible green cube; draw one cube box around top and dark side face.
- `s001_081...jpg`: three visible objects. Draw **three** boxes, one for the green cube and one for each black object after confirming their physical shapes. The capture manifest currently says “single black cube” for this image; the labeler corrects that row when saved.
- `s001_077...jpg` and `s001_078...jpg`: draw one box per visible black object if you know which shape was placed.
- `s001_079...jpg`: the overhead black silhouette is ambiguous. Confirm the physical shape before labeling.

## Quality check before model comparison

Review each saved image for a missing object, wrong class, wrong color, overly broad box, or box around a shadow. Keep all 81 captured images in the held-out test set; do not fine-tune on them. The evaluation script requires a `.txt` label for every image, including a **zero-byte** file for a truly empty scene. Images with uncertain shapes must be resolved before running full quantitative scoring.

### Changelog

- v1.0: Four physical shape classes, independent black/green color field, manual bounding boxes, and per-image manifest correction.
