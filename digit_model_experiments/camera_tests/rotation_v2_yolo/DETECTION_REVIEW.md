# Cube detection review — 2026-10-06

The 28 saved `*_raw.jpg` frames are an informal camera check. The live log reports
zero cubes in 11 frames, but the raw images show cubes in those frames. The
`true_cube_count` and `scene_id` fields are blank, so this folder is not yet a
scored detection test set. The 21 saved digit crops only cover boxes that the
live pipeline returned.

## Diagnostic replay of the 11 zero-detection frames

The unchanged five-class checkpoint
`vision_experiments/checkpoints/robotic_E1_camera_finetune_best.pt` was run
with direct `YOLO.predict()` on each saved raw frame. Default image size was
640 pixels. This is a diagnostic replay, not an independent validation score.
At confidence 0.25, the live tracker had 21 boxes and 11 zero-box frames;
direct prediction had 29 boxes and 7 zero-box frames on the same 28 images.
Visual inspection confirms that several of the additional boxes cover real
second cubes, but manually reviewed labels are still needed for recall.

| Frame time | Direct detection finding |
| --- | --- |
| 05:46:49 | One cube at confidence 0.652, despite live tracking reporting zero. |
| 05:49:30 | One cube at confidence 0.757, despite live tracking reporting zero; another visible cube is missed. |
| 05:49:46 | One cube at confidence 0.526, despite live tracking reporting zero; another cube appears only at 0.119 confidence. |
| 05:48:13 | One cube appears at 0.291 confidence when threshold is 0.10, below the previous live threshold of 0.35. |
| 05:46:57 | One cube appears at 0.187 with image size 960 and threshold 0.10. |
| 05:48:43 | Both visible cubes appear at 0.275 and 0.208 with image size 960 and threshold 0.10. |
| 05:49:26 | One edge cube appears at 0.103 at image size 640; the other remains missed. |
| 05:46:45, 05:47:20, 05:47:28, 05:47:31 | No cube detections in the tested direct-detection settings, though cubes are visible. |

The first three rows show that tracking can suppress boxes produced by the
detector. The remaining cases show a detector/domain problem. Increasing image
size is not a universal fix: at 05:49:46 the 960-pixel replay returned no cube
while the 640-pixel replay returned one high-confidence cube.

## Next experiment

1. For the immediate camera check, use per-frame prediction. The camera script
   now defaults to `--detector-mode predict` and confidence 0.25. It still
   accepts `--detector-mode track` if persistent IDs are needed later. Compare
   both using new captures with manually filled `true_cube_count`.
2. Keep the five-class shape checkpoint unchanged. Create a separate one-class
   cube detector initialized from those weights, using *reviewed* YOLO boxes
   on new camera sessions with the actual numbered cubes. Include multiple
   cubes, all positions including table edges, orientations, lighting, and
   empty scenes. Annotate every visible cube, not only ones detected by the
   current model.
3. Keep these 28 frames as a held-out challenge set for both old and new
   detectors. Capture distinct sessions for training and validation. Do not
   train on these frames or count model-generated boxes as ground truth.
4. Compare cube recall, false positives per frame, and box quality on the same
   manually labeled held-out images at the same operating threshold. Check the
   end-to-end digit result only after cube detection is measured separately.

The existing `dataset/real_camera_shape_test_v1` 81 images were already used
to fine-tune the five-class model; they are not a fresh independent test.
