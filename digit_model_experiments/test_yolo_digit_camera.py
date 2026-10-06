r"""Camera-only YOLO cube detection followed by two digit classifiers.

Run from the workspace root:
    python digit_model_experiments/test_yolo_digit_camera.py --yolo vision_experiments/checkpoints/robotic_E1_camera_finetune_best.pt --index 1

Direct detection is the default so the tracker cannot suppress a visible cube.
Use --detector-mode track when persistent IDs are needed. Without tracking, cube
numbers are left-to-right order for the current frame, not persistent identities.
No robot motion or pick command is sent.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import cv2
import torch
from ultralytics import YOLO

from test_digit_camera import load_models, predict
from test_multi_cube_camera import predict_rotations


WINDOW = "YOLO cubes + digit models | S: save frame | Q: quit"
OUTPUT = Path(__file__).resolve().parent / "camera_tests" / "yolo_digit"
LOG_FIELDS = ["capture_id", "crop_file", "track_id", "cube_order", "yolo_confidence",
              "cube_box_xyxy", "digit_box_xyxy", "mobile_prediction", "mobile_score",
              "resnet_prediction", "resnet_score", "true_digit", "scene_id", "split"]
FRAME_LOG_FIELDS = ["capture_id", "raw_frame", "labeled_frame", "detected_cubes",
                    "true_cube_count", "scene_id"]


def inner_box(box: tuple[int, int, int, int], scale: float):
    """Crop the middle of a YOLO cube box where a face digit is expected."""
    x1, y1, x2, y2 = box
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    half_w = (x2 - x1) * scale / 2
    half_h = (y2 - y1) * scale / 2
    return (int(center_x - half_w), int(center_y - half_h),
            int(center_x + half_w), int(center_y + half_h))


def cube_detections(result, frame_shape, cube_class):
    """Extract supported YOLO detection/segmentation boxes and optional track IDs."""
    height, width = frame_shape[:2]
    if result.boxes is None:
        return []
    boxes = result.boxes
    coords = boxes.xyxy.int().cpu().tolist()
    classes = boxes.cls.int().cpu().tolist()
    scores = boxes.conf.cpu().tolist()
    ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [None] * len(coords)
    found = []
    for (x1, y1, x2, y2), class_id, confidence, track_id in zip(coords, classes, scores, ids):
        class_name = str(result.names[class_id]).strip().lower()
        if class_name != cube_class.lower():
            continue
        box = (max(0, x1), max(0, y1), min(width, x2), min(height, y2))
        if box[2] - box[0] < 24 or box[3] - box[1] < 24:
            continue
        found.append((box, float(confidence), track_id))
    return sorted(found, key=lambda item: item[0][0])


def draw_label(frame, text, point, color):
    x, y = point
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                color, 2, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yolo", type=Path, required=True, help="Trained YOLO shape checkpoint")
    parser.add_argument("--mobile-checkpoint", type=Path,
                        help="Optional replacement MobileNet checkpoint for comparison")
    parser.add_argument("--resnet-checkpoint", type=Path,
                        help="Optional replacement ResNet18 checkpoint for comparison")
    parser.add_argument("--index", type=int, default=1, help="Camera index")
    parser.add_argument("--cube-class", default="cube", help="Cube class name in YOLO model")
    parser.add_argument("--yolo-confidence", type=float, default=0.25)
    parser.add_argument("--detector-mode", choices=("predict", "track"), default="predict",
                        help="Direct per-frame detection (default) or persistent tracking")
    parser.add_argument("--yolo-imgsz", type=int, default=640,
                        help="YOLO inference image size; larger values may help small cubes but run slower")
    parser.add_argument("--inner-scale", type=float, default=0.72,
                        help="Fraction of each cube box sent to the digit models")
    parser.add_argument("--rotation-search", action="store_true",
                        help="Try four rotations; experimental and ambiguous for 6/9")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT,
                        help="Directory for raw frames, labeled frames, cube crops, and CSV")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Minimum seconds between inference updates")
    args = parser.parse_args()
    if not args.yolo.is_file():
        parser.error(f"YOLO checkpoint not found: {args.yolo}")
    if not 0 < args.yolo_confidence < 1 or not 0.2 <= args.inner_scale <= 1 or args.interval <= 0 or args.yolo_imgsz < 32:
        parser.error("Confidence must be 0-1; inner scale 0.2-1; interval positive; YOLO image size at least 32")

    device = torch.device("cpu")
    replacements = {}
    if args.mobile_checkpoint:
        replacements["MobileNetV3-Small"] = args.mobile_checkpoint
    if args.resnet_checkpoint:
        replacements["ResNet18"] = args.resnet_checkpoint
    digit_models = load_models(device, replacements)
    yolo = YOLO(str(args.yolo))
    names = {str(name).strip().lower() for name in yolo.names.values()}
    print("YOLO classes:", yolo.names)
    if args.cube_class.lower() not in names:
        parser.error(f"Class {args.cube_class!r} is absent. Set --cube-class to a listed YOLO class.")

    camera = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise RuntimeError(f"Camera {args.index} did not open")
    print("Green: both digit models agree. Orange: they disagree. Numbers are provisional.")
    print("The inner cyan box is the actual digit-model input. Press S to save, Q to quit.")
    last_update = 0.0
    annotated = None
    sample_frame = None
    observations = []
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("Camera stopped returning frames")
            if annotated is None or time.monotonic() - last_update >= args.interval:
                annotated = frame.copy()
                sample_frame = frame.copy()
                observations = []
                if args.detector_mode == "track":
                    result = yolo.track(frame, persist=True, conf=args.yolo_confidence,
                                        imgsz=args.yolo_imgsz, verbose=False)[0]
                else:
                    result = yolo.predict(frame, conf=args.yolo_confidence,
                                          imgsz=args.yolo_imgsz, verbose=False)[0]
                cubes = cube_detections(result, frame.shape, args.cube_class)
                for order, (box, yolo_score, track_id) in enumerate(cubes, 1):
                    x1, y1, x2, y2 = box
                    ix1, iy1, ix2, iy2 = inner_box(box, args.inner_scale)
                    digit_crop = frame[iy1:iy2, ix1:ix2]
                    if digit_crop.size == 0:
                        continue
                    if args.rotation_search:
                        predictions = predict_rotations(digit_crop, digit_models, device)
                        mobile = predictions["MobileNetV3-Small"]
                        resnet = predictions["ResNet18"]
                    else:
                        predictions = predict(digit_crop, digit_models, device)
                        mobile = predictions["MobileNetV3-Small"]
                        resnet = predictions["ResNet18"]
                    agree = mobile[0] == resnet[0]
                    color = (0, 210, 0) if agree else (0, 165, 255)
                    cube_id = f"track {track_id}" if track_id is not None else f"cube {order}"
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.rectangle(annotated, (ix1, iy1), (ix2, iy2), (255, 255, 0), 2)
                    draw_label(annotated, f"{cube_id} YOLO {yolo_score:.0%}",
                               (x1, max(20, y1 - 8)), color)
                    text = f"M:{mobile[0]} {mobile[1]:.0%}  R:{resnet[0]} {resnet[1]:.0%}"
                    draw_label(annotated, text, (x1, min(frame.shape[0] - 8, y2 + 20)), color)
                    observations.append({
                        "track_id": track_id, "cube_order": order, "yolo_confidence": yolo_score,
                        "cube_box": box, "digit_box": (ix1, iy1, ix2, iy2),
                        "mobile": mobile, "resnet": resnet,
                    })
                draw_label(annotated, f"Cubes: {len(cubes)} | S save | Q quit",
                           (10, frame.shape[0] - 10), (255, 255, 255))
                last_update = time.monotonic()
            cv2.imshow(WINDOW, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):
                args.output_dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                raw_path = args.output_dir / f"{stamp}_raw.jpg"
                overlay_path = args.output_dir / f"{stamp}_labeled.jpg"
                if not cv2.imwrite(str(raw_path), sample_frame) or not cv2.imwrite(str(overlay_path), annotated):
                    raise OSError("Could not save camera frames")
                frame_log = args.output_dir / "frames.csv"
                new_frame_log = not frame_log.exists()
                with frame_log.open("a", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    if new_frame_log:
                        writer.writerow(FRAME_LOG_FIELDS)
                    writer.writerow([stamp, raw_path.name, overlay_path.name,
                                     len(observations), "", ""])
                log_path = args.output_dir / "cube_crops.csv"
                new_log = not log_path.exists()
                with log_path.open("a", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    if new_log:
                        writer.writerow(LOG_FIELDS)
                    for observation in observations:
                        ix1, iy1, ix2, iy2 = observation["digit_box"]
                        crop_file = f"{stamp}_cube{observation['cube_order']}_digit.png"
                        crop = sample_frame[iy1:iy2, ix1:ix2]
                        if not cv2.imwrite(str(args.output_dir / crop_file), crop):
                            raise OSError(f"Could not save {crop_file}")
                        mobile = observation["mobile"]
                        resnet = observation["resnet"]
                        writer.writerow([
                            stamp, crop_file, observation["track_id"], observation["cube_order"],
                            f"{observation['yolo_confidence']:.6f}", observation["cube_box"],
                            observation["digit_box"], mobile[0], f"{mobile[1]:.6f}",
                            resnet[0], f"{resnet[1]:.6f}", "", "", "",
                        ])
                print(f"Saved {raw_path.name}, {overlay_path.name}, and {len(observations)} cube crops")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError, OSError, cv2.error) as exc:
        raise SystemExit(f"YOLO + digit camera test error: {exc}") from exc
