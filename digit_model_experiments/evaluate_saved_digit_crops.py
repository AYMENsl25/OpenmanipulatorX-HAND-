"""Run both trained digit classifiers on saved camera crops without YOLO or a live camera.

This is a diagnostic pass. Repeated rotations of one crop are correlated observations,
and accuracy is reported only for rows with a user-supplied true_digit label.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import torch

from test_digit_camera import load_models, predict


HERE = Path(__file__).resolve().parent
DEFAULT_CROPS = HERE / "camera_tests" / "yolo_digit"
DEFAULT_MOBILE = HERE / "checkpoints" / "rotation_v2" / "mobilenetv3_small" / "best.pt"
DEFAULT_RESNET = HERE / "checkpoints" / "rotation_v2" / "resnet18" / "best.pt"
DEFAULT_OUTPUT = HERE / "results" / "rotation_v2" / "saved_crop_predictions.csv"
FIELDS = ["crop_file", "rotation_ccw_degrees", "true_digit", "mobile_prediction",
          "mobile_confidence", "mobile_ms", "resnet_prediction", "resnet_confidence", "resnet_ms"]


def read_labels(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {"crop_file", "true_digit"} <= set(reader.fieldnames):
            raise ValueError("Labels CSV needs crop_file and true_digit columns")
        labels = {}
        for row in reader:
            label = row["true_digit"].strip()
            if label and label not in {str(digit) for digit in range(10)}:
                raise ValueError(f"Invalid digit label for {row['crop_file']}: {label}")
            labels[Path(row["crop_file"]).name] = label
        return labels


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_CROPS)
    parser.add_argument("--pattern", default="*_cube*_digit.png")
    parser.add_argument("--mobile-checkpoint", type=Path, default=DEFAULT_MOBILE)
    parser.add_argument("--resnet-checkpoint", type=Path, default=DEFAULT_RESNET)
    parser.add_argument("--labels-csv", type=Path,
                        help="Optional CSV with crop_file,true_digit; blank labels remain unscored")
    parser.add_argument("--rotations", default="0",
                        help="Comma-separated counterclockwise angles from 0,90,180,270")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    try:
        rotations = [int(part.strip()) for part in args.rotations.split(",")]
    except ValueError as exc:
        parser.error("--rotations must contain comma-separated integers")
        raise AssertionError from exc
    if not rotations or any(angle not in {0, 90, 180, 270} for angle in rotations):
        parser.error("--rotations accepts only 0,90,180,270")
    paths = sorted(args.input_dir.glob(args.pattern))
    if not paths:
        parser.error(f"No matching crops in {args.input_dir}")

    labels = read_labels(args.labels_csv)
    device = torch.device("cpu")
    models = load_models(device, {
        "MobileNetV3-Small": args.mobile_checkpoint,
        "ResNet18": args.resnet_checkpoint,
    })
    rows = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"Cannot read image: {path}")
        for angle in rotations:
            rotated = np.rot90(image, k=angle // 90).copy()
            results = predict(rotated, models, device)
            mobile = results["MobileNetV3-Small"]
            resnet = results["ResNet18"]
            row = {
                "crop_file": str(path.resolve()),
                "rotation_ccw_degrees": angle,
                "true_digit": labels.get(path.name, ""),
                "mobile_prediction": mobile[0],
                "mobile_confidence": f"{mobile[1]:.6f}",
                "mobile_ms": f"{mobile[2]:.2f}",
                "resnet_prediction": resnet[0],
                "resnet_confidence": f"{resnet[1]:.6f}",
                "resnet_ms": f"{resnet[2]:.2f}",
            }
            rows.append(row)
            print(f"{path.name} {angle:3d}° | MobileNet {mobile[0]} {mobile[1]:.1%} | "
                  f"ResNet18 {resnet[0]} {resnet[1]:.1%}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} predictions to {args.output}")
    labeled = [row for row in rows if row["true_digit"]]
    if labeled:
        for name, column in (("MobileNetV3-Small", "mobile_prediction"),
                             ("ResNet18", "resnet_prediction")):
            correct = sum(str(row[column]) == row["true_digit"] for row in labeled)
            print(f"{name}: {correct}/{len(labeled)} labeled rows correct "
                  "(rotated views of one crop are not independent samples)")
    else:
        print("No true_digit labels supplied; predictions are diagnostic, not accuracy metrics.")


if __name__ == "__main__":
    main()
