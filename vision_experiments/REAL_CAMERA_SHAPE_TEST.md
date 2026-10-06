# Real-camera shape test: capture and model choice

## Current notebook results

The two executed Kaggle notebooks trained official pretrained YOLO26n models. Their saved outputs report:

| Dataset/model | Native task | Test images | E0 box mAP50-95 | E1 box mAP50-95 | E0 mask mAP50-95 |
|---|---|---:|---:|---:|---:|
| Colored Block | segmentation | 414 | 0.8013 | 0.7848 | 0.7027 |
| Robotic Arm | detection | 9 | 0.9153 | 0.8382 | — |

E0 is the baseline. E1 used robot-oriented augmentation. These are **different datasets and tasks**, so do not compare their native mAP values to choose a robot model. The Robotic Arm test has no cylinder and only one pyramid. The Colored Block model has `triangular`, not `pyramid`. The notebooks have `REAL_TEST_ROOT=None`, so neither was tested on the robot camera. The Colored Block notebook logged perceptual cross-split duplicate warnings; inspect those before treating its native test score as independent.

## Capture plan

Keep the camera rigid at the operating scan pose for the main benchmark. Change the **object's** in-plane rotation and position; change lamp direction and intensity while recording the `--lighting` condition. Do not silently tilt the camera. Only add a separately tagged `slight_tilt` stress set if the real mount can move.

Start with the available black cube, cylinder, sphere and pyramid, plus the green cube. Suggested pilot: 8 distinct positions/rotations per black shape per normal, dim and side-lit condition (96 images); 4 green-cube scenes per lighting condition (12); 12 two-object and 12 three-object scenes; 12 empty/background scenes. Total: about 144 images. Use different object spacing, left/center/right and near/far positions, 0/45/90/135 degree rotations where shape orientation is visible, touching and separated examples, and some partly cut-off objects. Save one image per meaningful scene change. Record difficult examples such as shadows, glare, touching shapes, and robot/gripper intrusion as separate scene types. Do not manufacture extra frames by saving an unchanged scene repeatedly.

Capture all four shapes under **each** light condition even if they are only black now. The green cube tests color robustness but does not test unseen colors for the other shapes. Add the coming colors in a later batch and retain this first test set untouched.

Use `capture_shape_test.py` from the project root. It captures only camera frames and metadata; it sends no robot commands:

```powershell
& '.\venv\Scripts\python.exe' .\vision_experiments\capture_shape_test.py --index 1 --session day1_normal --scene-id s001 --scene-type single --lighting normal --shapes cube --colors black --scan-pose scan_primary --pose-note 'recorded fixed pose'
```

Press **S** once after setting each new arrangement, **Q** to close. For the next lighting condition or scene, relaunch with a new `--session`, `--scene-id`, and matching metadata. Example for two objects:

```powershell
& '.\venv\Scripts\python.exe' .\vision_experiments\capture_shape_test.py --index 1 --session day1_side_left --scene-id s002 --scene-type two --lighting side_left --shapes cube,cylinder --colors black,green --scan-pose scan_primary
```

Images and `capture_manifest.csv` are written under `dataset/real_camera_shape_test_v1/`. They are **held-out test data**. Keep future fine-tuning images in a different capture directory or session; do not train on these scenes or near-duplicate frames.

## Annotation and evaluation

Use the local [manual box labeler](label_shape_boxes.py) and [annotation guide](SHAPE_ANNOTATION_GUIDE.md) for the captured images. The labeler corrects the manifest's per-image shape/color metadata after each save; the initial `s001` session flags were reused for later multi-object and green-cube frames.

Label every visible object, including partial objects. Use canonical YOLO detection IDs `0 cube`, `1 cylinder`, `2 sphere`, `3 pyramid`. Annotate tight boxes on all images; create an empty `.txt` for true empty scenes. Put labels at `dataset/real_camera_shape_test_v1/labels/<session>/<image-stem>.txt`. Review labels manually. For precise mask-center assessment, add polygon annotations later as a separate ground-truth layer; the current comparison uses bbox center for both models.

Download `E0_baseline_best.pt` from **each** Kaggle notebook's `yolo26_export.zip`, then run:

```powershell
& '.\venv\Scripts\python.exe' -m pip install ultralytics
& '.\venv\Scripts\python.exe' .\vision_experiments\evaluate_shape_test.py --colored 'C:\path\colored\E0_baseline_best.pt' --robotic 'C:\path\robotic\E0_baseline_best.pt'
```

This writes per-image outcomes, confidence scores, bbox center errors in pixels, gallery images and `evaluation/scores.json`. It compares cube/cylinder/sphere on exactly the same camera frames with a 0.25 confidence threshold and 0.50 IoU match threshold. Pyramid is reported only for Robotic Arm. The script does not claim mAP: its precision and recall are threshold-specific. Inspect missed objects, false positives on empty scenes, per-class recall, center error and lighting sensitivity before deciding. If Colored Block wins common-class performance, it still needs a properly labeled pyramid fine-tune before serving all four classes. Reserve a **new**, separate real-camera validation/test group for checking any fine-tuned model.

No pixel-to-millimeter conversion or automatic picking follows from this test. Camera calibration and robot safety checks remain separate.
