"""Test YOLO cube boxes followed by rotation-trained digit models on saved camera frames.

No live camera or robot is used. The CSV leaves true_digit blank for manual labeling.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

from test_digit_camera import load_models, predict
from test_yolo_digit_camera import cube_detections, draw_label, inner_box


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_FRAMES = HERE / "camera_tests" / "yolo_digit"
DEFAULT_YOLO = ROOT / "vision_experiments" / "checkpoints" / "robotic_E1_camera_finetune_best.pt"
DEFAULT_MOBILE = HERE / "checkpoints" / "rotation_v2" / "mobilenetv3_small" / "best.pt"
DEFAULT_RESNET = HERE / "checkpoints" / "rotation_v2" / "resnet18" / "best.pt"
DEFAULT_OUTPUT = HERE / "results" / "rotation_v2" / "yolo_saved_frames"
FIELDS = ["frame_file", "cube_order", "detected", "yolo_confidence", "cube_box_xyxy",
          "digit_box_xyxy", "digit_crop_file", "mobile_prediction", "mobile_confidence",
          "resnet_prediction", "resnet_confidence", "true_digit"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-dir", type=Path, default=DEFAULT_FRAMES)
    parser.add_argument("--pattern", default="*_raw.jpg")
    parser.add_argument("--yolo", type=Path, default=DEFAULT_YOLO)
    parser.add_argument("--mobile-checkpoint", type=Path, default=DEFAULT_MOBILE)
    parser.add_argument("--resnet-checkpoint", type=Path, default=DEFAULT_RESNET)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--yolo-confidence", type=float, default=0.35)
    parser.add_argument("--inner-scale", type=float, default=0.72)
    parser.add_argument("--cube-class", default="cube")
    args = parser.parse_args()
    if not args.yolo.is_file():
        parser.error(f"Missing YOLO model: {args.yolo}")
    if not 0 < args.yolo_confidence < 1 or not 0.2 <= args.inner_scale <= 1:
        parser.error("Confidence must be between 0 and 1; inner scale between 0.2 and 1")
    frames = sorted(args.frames_dir.glob(args.pattern))
    if not frames:
        parser.error(f"No saved frames in {args.frames_dir}")

    device = torch.device("cpu")
    digit_models = load_models(device, {
        "MobileNetV3-Small": args.mobile_checkpoint,
        "ResNet18": args.resnet_checkpoint,
    })
    yolo = YOLO(str(args.yolo))
    if args.cube_class.lower() not in {str(name).lower() for name in yolo.names.values()}:
        parser.error(f"Cube class {args.cube_class!r} absent from YOLO classes {yolo.names}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for frame_path in frames:
        frame = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if frame is None:
            raise OSError(f"Cannot read {frame_path}")
        result = yolo.predict(frame, conf=args.yolo_confidence, device="cpu", verbose=False)[0]
        detections = cube_detections(result, frame.shape, args.cube_class)
        overlay = frame.copy()
        if not detections:
            rows.append({"frame_file": str(frame_path.resolve()), "cube_order": "",
                         "detected": 0, "true_digit": ""})
            draw_label(overlay, "No cube detected", (10, 30), (0, 165, 255))
        for order, (box, yolo_score, _track_id) in enumerate(detections, 1):
            x1, y1, x2, y2 = box
            ix1, iy1, ix2, iy2 = inner_box(box, args.inner_scale)
            crop = frame[iy1:iy2, ix1:ix2]
            if crop.size == 0:
                continue
            predictions = predict(crop, digit_models, device)
            mobile = predictions["MobileNetV3-Small"]
            resnet = predictions["ResNet18"]
            crop_path = args.output_dir / f"{frame_path.stem}_cube{order}_digit.png"
            if not cv2.imwrite(str(crop_path), crop):
                raise OSError(f"Cannot write {crop_path}")
            rows.append({
                "frame_file": str(frame_path.resolve()), "cube_order": order, "detected": 1,
                "yolo_confidence": f"{yolo_score:.6f}", "cube_box_xyxy": str(box),
                "digit_box_xyxy": str((ix1, iy1, ix2, iy2)),
                "digit_crop_file": str(crop_path.resolve()),
                "mobile_prediction": mobile[0], "mobile_confidence": f"{mobile[1]:.6f}",
                "resnet_prediction": resnet[0], "resnet_confidence": f"{resnet[1]:.6f}",
                "true_digit": "",
            })
            color = (0, 210, 0) if mobile[0] == resnet[0] else (0, 165, 255)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.rectangle(overlay, (ix1, iy1), (ix2, iy2), (255, 255, 0), 2)
            draw_label(overlay, f"cube {order} YOLO {yolo_score:.0%}",
                       (x1, max(20, y1 - 8)), color)
            draw_label(overlay, f"M:{mobile[0]} {mobile[1]:.0%} R:{resnet[0]} {resnet[1]:.0%}",
                       (x1, min(frame.shape[0] - 8, y2 + 20)), color)
        overlay_path = args.output_dir / f"{frame_path.stem}_rotation_v2_labeled.jpg"
        if not cv2.imwrite(str(overlay_path), overlay):
            raise OSError(f"Cannot write {overlay_path}")
        print(f"{frame_path.name}: {len(detections)} cube detection(s)")

    log = args.output_dir / "yolo_digit_predictions.csv"
    with log.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} rows to {log}; add true_digit labels before computing accuracy.")


if __name__ == "__main__":
    main()
