"""Read-only visual and structural check of numbered-cube YOLO labels.

Examples from the workspace root:
    python vision_experiments/inspect_numbered_cube_labels.py
    python vision_experiments/inspect_numbered_cube_labels.py --start 172
    python vision_experiments/inspect_numbered_cube_labels.py --status all --start 1
    python vision_experiments/inspect_numbered_cube_labels.py --audit-only
    python vision_experiments/inspect_numbered_cube_labels.py --export-sheets dataset/cube_only_v1/reviewed_overlays --audit-only

This script never writes to the raw images, YOLO labels, or review manifest.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IMAGES = ROOT / "dataset" / "raw" / "numbered_cubes_day1"
DEFAULT_LABELS = ROOT / "dataset" / "cube_only_v1" / "reviewed_labels"
DEFAULT_MANIFEST = ROOT / "dataset" / "cube_only_v1" / "reviewed_manifest.csv"
WINDOW = "Cube label inspection | N/Space next | P previous | Q quit"
VALID_STATUSES = {"reviewed", "ai_reviewed"}


def read_manifest(path: Path) -> dict[str, dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(f"Missing review manifest: {path}")
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"image", "status", "cube_count"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"Manifest must contain {', '.join(sorted(required))}: {path}")
        rows = {}
        for row in reader:
            name = row["image"]
            if name in rows:
                raise ValueError(f"Duplicate manifest image: {name}")
            rows[name] = row
        return rows


def read_yolo_boxes(path: Path, width: int, height: int) -> list[tuple[int, int, int, int]]:
    boxes = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 5 or parts[0] != "0":
            raise ValueError(f"{path.name}:{line_number}: expected '0 cx cy w h'")
        try:
            cx, cy, bw, bh = map(float, parts[1:])
        except ValueError as exc:
            raise ValueError(f"{path.name}:{line_number}: non-numeric box") from exc
        if not all(np.isfinite(v) for v in (cx, cy, bw, bh)):
            raise ValueError(f"{path.name}:{line_number}: non-finite box")
        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
            raise ValueError(f"{path.name}:{line_number}: out-of-range box")
        left, top, right, bottom = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        if left < -1e-5 or top < -1e-5 or right > 1 + 1e-5 or bottom > 1 + 1e-5:
            raise ValueError(f"{path.name}:{line_number}: box crosses image boundary")
        x1 = max(0, min(width - 1, round(left * width)))
        y1 = max(0, min(height - 1, round(top * height)))
        x2 = max(1, min(width, round(right * width)))
        y2 = max(1, min(height, round(bottom * height)))
        if x2 <= x1 or y2 <= y1:
            raise ValueError(f"{path.name}:{line_number}: zero-pixel box")
        boxes.append((x1, y1, x2, y2))
    return boxes


def audit(images_dir: Path, labels_dir: Path, manifest_path: Path):
    images = sorted(images_dir.glob("*.jpg"))
    if not images:
        raise ValueError(f"No JPG images in {images_dir}")
    manifest = read_manifest(manifest_path)
    image_names = {p.name for p in images}
    label_names = {p.stem for p in labels_dir.glob("*.txt")}
    issues = []
    for extra in sorted(label_names - {p.stem for p in images}):
        issues.append(f"Label without image: {extra}.txt")
    for extra in sorted(set(manifest) - image_names):
        issues.append(f"Manifest row without image: {extra}")
    records = []
    for global_index, image in enumerate(images, 1):
        label = labels_dir / f"{image.stem}.txt"
        if not label.is_file():
            issues.append(f"Missing label: {label.name}")
            continue
        row = manifest.get(image.name)
        if row is None:
            issues.append(f"Missing manifest row: {image.name}")
            continue
        frame = cv2.imread(str(image))
        if frame is None:
            issues.append(f"Unreadable image: {image.name}")
            continue
        height, width = frame.shape[:2]
        try:
            boxes = read_yolo_boxes(label, width, height)
        except ValueError as exc:
            issues.append(str(exc))
            continue
        status = row["status"]
        if status not in VALID_STATUSES:
            issues.append(f"Unexpected status {status!r}: {image.name}")
        try:
            expected_count = int(row["cube_count"])
        except ValueError:
            issues.append(f"Invalid manifest cube_count: {image.name}")
        else:
            if expected_count != len(boxes):
                issues.append(f"Count mismatch: {image.name}: manifest={expected_count}, labels={len(boxes)}")
        if len(set(boxes)) != len(boxes):
            issues.append(f"Duplicate identical boxes: {image.name}")
        records.append({"index": global_index, "image": image, "status": status, "boxes": boxes})
    print(f"Images: {len(images)} | readable labels: {len(records)} | issues: {len(issues)}")
    print("Status counts:", dict(sorted(Counter(r["status"] for r in records).items())))
    print("Cube-count distribution:", dict(sorted(Counter(len(r["boxes"]) for r in records).items())))
    print("Total cube boxes:", sum(len(r["boxes"]) for r in records))
    for issue in issues[:30]:
        print("ISSUE:", issue)
    if len(issues) > 30:
        print(f"... {len(issues) - 30} more issues")
    return records, issues


def draw_overlay(frame: np.ndarray, record: dict, scale: float = 1.0,
                 compact: bool = False) -> np.ndarray:
    view = frame.copy()
    color = (0, 215, 255) if record["status"] == "ai_reviewed" else (0, 220, 0)
    for number, (x1, y1, x2, y2) in enumerate(record["boxes"], 1):
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        cv2.putText(view, str(number), (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(view, str(number), (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2, cv2.LINE_AA)
    if scale != 1:
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    footer = 30 if compact else 62
    view = cv2.copyMakeBorder(view, 0, footer, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    y = view.shape[0] - footer
    if compact:
        cv2.putText(view, f"#{record['index']} {record['status']}  {len(record['boxes'])} cubes",
                    (6, y + 21), cv2.FONT_HERSHEY_SIMPLEX, .49,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return view
    lines = [f"#{record['index']}  {record['image'].name}  {record['status']}  cubes={len(record['boxes'])}",
             "Read-only: N/Space next | P previous | Q/Esc quit | Use --start NUMBER to jump"]
    for n, line in enumerate(lines):
        cv2.putText(view, line, (6, y + 22 + n * 25), cv2.FONT_HERSHEY_SIMPLEX,
                    .49, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def export_sheets(records: list[dict], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    columns, rows, tile_w, tile_h = 4, 4, 320, 270
    for page, start in enumerate(range(0, len(records), columns * rows), 1):
        canvas = np.full((rows * tile_h, columns * tile_w, 3), 245, dtype=np.uint8)
        for slot, record in enumerate(records[start:start + columns * rows]):
            frame = cv2.imread(str(record["image"]))
            view = draw_overlay(frame, record, .5, compact=True)
            x, y = (slot % columns) * tile_w, (slot // columns) * tile_h
            canvas[y:y + view.shape[0], x:x + view.shape[1]] = view
        path = output / f"labels_sheet_{page:03d}.jpg"
        if not cv2.imwrite(str(path), canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise OSError(f"Could not write {path}")
    print(f"Saved {(len(records) + 15) // 16} contact sheets to {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--status", choices=("all", "reviewed", "ai_reviewed"), default="ai_reviewed")
    parser.add_argument("--start", type=int, default=1, help="One-based index in the original 400-image sequence")
    parser.add_argument("--scale", type=float, default=1.4)
    parser.add_argument("--audit-only", action="store_true", help="Check label structure and counts without opening a window")
    parser.add_argument("--export-sheets", type=Path, help="Save read-only overlay contact sheets")
    args = parser.parse_args()
    if args.start < 1 or not .5 <= args.scale <= 2:
        parser.error("--start must be positive and --scale must be between 0.5 and 2")
    records, issues = audit(args.images, args.labels, args.manifest)
    selected = [r for r in records if (args.status == "all" or r["status"] == args.status)
                and r["index"] >= args.start]
    print(f"Selected {len(selected)} images (status={args.status}, starting at #{args.start}).")
    if args.export_sheets:
        export_sheets(selected, args.export_sheets)
    if issues:
        raise SystemExit("Audit found label/manifest issues. Fix them before relying on these labels.")
    if args.audit_only:
        return
    if not selected:
        raise SystemExit("No images match the selected status/start index.")
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    position = 0
    try:
        while True:
            record = selected[position]
            frame = cv2.imread(str(record["image"]))
            cv2.imshow(WINDOW, draw_overlay(frame, record, args.scale))
            key = cv2.waitKey(0) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("p"), ord("P"), ord("b"), ord("B")):
                position = max(0, position - 1)
            elif key in (ord("n"), ord("N"), 32):
                position = min(len(selected) - 1, position + 1)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
