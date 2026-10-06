"""Summarize labeled real-camera crops collected by test_digit_camera.py."""

import argparse
import csv
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parent / "camera_tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=LOG_DIR)
    args = parser.parse_args()
    logs = [args.log_dir / name for name in ("predictions.csv", "predictions_rotations.csv")]
    rows = []
    for log in logs:
        if log.is_file():
            with log.open(newline="", encoding="utf-8") as stream:
                rows.extend(row for row in csv.DictReader(stream)
                            if len(row["true_digit"]) == 1 and row["true_digit"] in "0123456789")
    if not any(log.is_file() for log in logs):
        raise SystemExit(f"No camera log yet in {args.log_dir}")
    if not rows:
        raise SystemExit("No labeled crops yet. Press a digit key (0-9) while the camera test runs.")
    print(f"Labeled captures: {len(rows)}")
    for name, field in (("MobileNetV3-Small", "mobile_prediction"), ("ResNet18", "resnet_prediction")):
        correct = sum(row[field] == row["true_digit"] for row in rows)
        print(f"{name}: {correct}/{len(rows)} correct ({correct / len(rows):.1%})")
        for digit in sorted({row["true_digit"] for row in rows}):
            subset = [row for row in rows if row["true_digit"] == digit]
            digit_correct = sum(row[field] == digit for row in subset)
            print(f"  digit {digit}: {digit_correct}/{len(subset)} correct")
    disagreements = [row for row in rows if row["mobile_prediction"] != row["resnet_prediction"]]
    print(f"Model disagreements: {len(disagreements)}")
    for row in disagreements[:20]:
        print(f"  {row['crop_file']}: true={row['true_digit']}, "
              f"MobileNet={row['mobile_prediction']}, ResNet={row['resnet_prediction']}")


if __name__ == "__main__":
    main()
