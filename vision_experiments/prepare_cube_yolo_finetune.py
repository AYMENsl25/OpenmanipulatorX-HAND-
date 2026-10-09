"""Export reviewed numbered-cube boxes to a one-class Ultralytics dataset.

The source images and labels are never modified. Validation is the last 20%
of the capture sequence, so consecutive near-duplicate frames are not randomly
scattered between train and validation. It is still a same-session estimate.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IMAGES = ROOT / "dataset/raw/numbered_cubes_day1"
DEFAULT_LABELS = ROOT / "dataset/cube_only_v1/reviewed_labels"
DEFAULT_MANIFEST = ROOT / "dataset/cube_only_v1/reviewed_manifest.csv"
DEFAULT_OUTPUT = ROOT / "dataset/cube_only_v1/yolo_cube_finetune"


def read_boxes(path: Path) -> int:
    count = 0
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        parts = line.split()
        if len(parts) != 5 or parts[0] != "0":
            raise ValueError(f"Invalid one-class YOLO label: {path}:{line_number}")
        values = [float(v) for v in parts[1:]]
        x, y, w, h = values
        if not all(math.isfinite(v) for v in values) or not (
            0 < w <= 1 and 0 < h <= 1 and
            0 <= x - w / 2 and x + w / 2 <= 1.000001 and
            0 <= y - h / 2 and y + h / 2 <= 1.000001
        ):
            raise ValueError(f"Out-of-bounds box: {path}:{line_number}")
        count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    args = parser.parse_args()
    if not 0 < args.val_fraction < 0.5:
        parser.error("--val-fraction must be between 0 and 0.5")

    with args.manifest.open(newline="", encoding="utf-8-sig") as stream:
        records = list(csv.DictReader(stream))
    if not records or len({row["image"] for row in records}) != len(records):
        raise ValueError("Manifest is empty or contains duplicate image names")
    image_files = {p.name for p in args.images.glob("*.jpg")}
    label_files = {p.stem for p in args.labels.glob("*.txt")}
    manifest_names = {row["image"] for row in records}
    if image_files != manifest_names or label_files != {Path(n).stem for n in manifest_names}:
        raise ValueError("Image, reviewed-label, and manifest inventories differ")

    status_counts = Counter()
    box_count = 0
    for row in records:
        image = args.images / row["image"]
        label = args.labels / f"{image.stem}.txt"
        if row["status"] not in {"reviewed", "ai_reviewed"}:
            raise ValueError(f"Unreviewed source: {image.name}")
        actual_count = read_boxes(label)
        if actual_count != int(row["cube_count"]):
            raise ValueError(f"Manifest box count differs: {image.name}")
        status_counts[row["status"]] += 1
        box_count += actual_count

    # A capture-order holdout is safer than random frame splitting for video bursts.
    validation_count = max(1, round(len(records) * args.val_fraction))
    split_at = len(records) - validation_count
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output is not empty; choose a new directory: {output}")
    rows = []
    for index, row in enumerate(records):
        split = "train" if index < split_at else "val"
        image_name = row["image"]
        image_dest = output / "images" / split / image_name
        label_dest = output / "labels" / split / f"{Path(image_name).stem}.txt"
        image_dest.parent.mkdir(parents=True, exist_ok=True)
        label_dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.images / image_name, image_dest)
        shutil.copy2(args.labels / label_dest.name, label_dest)
        rows.append({"image": image_name, "split": split, "status": row["status"],
                     "cube_count": row["cube_count"]})

    yaml_text = (
        f"path: {json.dumps(output.as_posix())}\n"
        "train: images/train\nval: images/val\n"
        "nc: 1\nnames:\n  0: cube\n"
    )
    (output / "data.yaml").write_text(yaml_text, encoding="utf-8")
    with (output / "split_manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"Exported {len(records)} images, {box_count} cube boxes: "
          f"{split_at} train / {validation_count} validation")
    print(f"Source statuses: {dict(status_counts)}")
    print(f"Dataset YAML: {output / 'data.yaml'}")
    print("Validation is from the same capture session; use a new session for final evaluation.")


if __name__ == "__main__":
    main()
