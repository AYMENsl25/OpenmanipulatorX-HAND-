"""Summarize manually labeled live YOLO+digit captures without mixing detection and recognition."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = HERE / "camera_tests" / "rotation_v2_yolo"


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    args = parser.parse_args()
    frames = read_rows(args.log_dir / "frames.csv")
    crops = read_rows(args.log_dir / "cube_crops.csv")
    if not frames and not crops:
        parser.error(f"No saved frame or cube CSV in {args.log_dir}")

    print(f"Saved frames: {len(frames)}; detected cube crops: {len(crops)}")
    counted = [row for row in frames if row.get("true_cube_count", "").strip().isdigit()]
    if counted:
        visible = sum(int(row["true_cube_count"]) for row in counted)
        detected = sum(int(row["detected_cubes"]) for row in counted)
        short = [row for row in counted if int(row["detected_cubes"]) < int(row["true_cube_count"])]
        excess = [row for row in counted if int(row["detected_cubes"]) > int(row["true_cube_count"])]
        print(f"Frames with manual cube counts: {len(counted)}; visible cubes: {visible}; "
              f"YOLO boxes: {detected}; count shortfalls: {len(short)} frames; "
              f"excess boxes: {len(excess)} frames")
        print("Counts alone do not establish detection recall; inspect saved overlays for matches and duplicates.")
        for row in short[:20]:
            print(f"  missed-count frame {row['raw_frame']}: "
                  f"detected {row['detected_cubes']} / visible {row['true_cube_count']}")
    else:
        print("Fill true_cube_count in frames.csv to inspect missed cube counts.")

    labeled = [row for row in crops if row.get("true_digit", "").strip() in "0123456789"
               and len(row.get("true_digit", "").strip()) == 1]
    if not labeled:
        print("Fill true_digit in cube_crops.csv to score digit recognition on detected cubes.")
        return
    print(f"Labeled detected cube crops: {len(labeled)}")
    for name, field in (("MobileNetV3-Small", "mobile_prediction"),
                        ("ResNet18", "resnet_prediction")):
        correct = sum(row[field] == row["true_digit"] for row in labeled)
        print(f"{name}: {correct}/{len(labeled)} correct ({correct / len(labeled):.1%})")
        for digit in sorted({row["true_digit"] for row in labeled}):
            subset = [row for row in labeled if row["true_digit"] == digit]
            found = sum(row[field] == digit for row in subset)
            print(f"  digit {digit}: {found}/{len(subset)} correct")
    disagreements = [row for row in labeled if row["mobile_prediction"] != row["resnet_prediction"]]
    print(f"Model disagreements on labeled crops: {len(disagreements)}")
    for row in disagreements[:20]:
        print(f"  {row['crop_file']}: true={row['true_digit']}, "
              f"MobileNet={row['mobile_prediction']}, ResNet18={row['resnet_prediction']}")


if __name__ == "__main__":
    main()
