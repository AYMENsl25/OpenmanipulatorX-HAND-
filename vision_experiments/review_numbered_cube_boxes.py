"""Review model-proposed boxes for the one-class numbered-cube dataset.

The proposals are not ground truth. Every image must be inspected and saved
before its label may be used for training. This program never changes raw images.
"""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent.parent
IMAGES = ROOT / "dataset" / "raw" / "numbered_cubes_day1"
PROPOSALS = ROOT / "dataset" / "cube_only_v1" / "review" / "agnostic_proposals.jsonl"
REVIEW_DIR = ROOT / "dataset" / "cube_only_v1" / "reviewed_labels"
MANIFEST = ROOT / "dataset" / "cube_only_v1" / "reviewed_manifest.csv"
WINDOW = "Review cube boxes | drag: add | right click: remove | S: save"
FIELDS = ("image", "status", "cube_count", "reviewed_at")


def read_proposals(path: Path) -> dict[str, list[tuple[int, int, int, int]]]:
    if not path.is_file():
        raise FileNotFoundError(f"Proposal file not found: {path}")
    data = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        data[record["image"]] = [tuple(round(v) for v in item["xyxy"])
                                 for item in record["boxes"]]
    return data


def load_reviewed(path: Path, width: int, height: int) -> list[tuple[int, int, int, int]]:
    boxes = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 5 or parts[0] != "0":
            raise ValueError(f"Invalid one-class YOLO label in {path}: {line}")
        cx, cy, bw, bh = map(float, parts[1:])
        box = (round((cx - bw / 2) * width), round((cy - bh / 2) * height),
               round((cx + bw / 2) * width), round((cy + bh / 2) * height))
        boxes.append(box)
    return boxes


def save_reviewed(image: Path, boxes: list[tuple[int, int, int, int]],
                  width: int, height: int, review_dir: Path, manifest: Path) -> None:
    review_dir.mkdir(parents=True, exist_ok=True)
    label = review_dir / f"{image.stem}.txt"
    lines = []
    for x1, y1, x2, y2 in boxes:
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError(f"Invalid box {image.name}: {(x1, y1, x2, y2)}")
        lines.append(f"0 {(x1+x2)/(2*width):.8f} {(y1+y2)/(2*height):.8f} "
                     f"{(x2-x1)/width:.8f} {(y2-y1)/height:.8f}")
    tmp = label.with_suffix(".txt.tmp")
    tmp.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    tmp.replace(label)

    rows = {}
    if manifest.exists():
        with manifest.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                rows[row["image"]] = row
    rows[image.name] = {"image": image.name, "status": "reviewed",
                        "cube_count": len(boxes),
                        "reviewed_at": datetime.now().astimezone().isoformat(timespec="seconds")}
    manifest.parent.mkdir(parents=True, exist_ok=True)
    tmp = manifest.with_suffix(".csv.tmp")
    with tmp.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows[key] for key in sorted(rows))
    tmp.replace(manifest)


def draw(frame, boxes, pending, index, total, name, already_reviewed, scale):
    view = frame.copy()
    for number, (x1, y1, x2, y2) in enumerate(boxes, 1):
        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(view, str(number), (x1, max(17, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, str(number), (x1, max(17, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 1, cv2.LINE_AA)
    if pending:
        cv2.rectangle(view, pending[:2], pending[2:], (0, 165, 255), 2)
    if scale != 1:
        view = cv2.resize(view, None, fx=scale, fy=scale)
    view = cv2.copyMakeBorder(view, 0, 88, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    y = round(frame.shape[0] * scale)
    status = "REVIEWED" if already_reviewed else "UNREVIEWED PROPOSALS"
    lines = [f"{index+1}/{total}  {name}  boxes={len(boxes)}  {status}",
             "Draw every visible cube, including white/green and partial edge cubes. Exclude shadows.",
             "Left drag: add | Right click: remove | U: undo | R: reset proposals",
             "S: save+next | E: confirm empty | K: skip | P: previous | Q: quit"]
    for j, line in enumerate(lines):
        cv2.putText(view, line, (5, y + 17 + j * 20), cv2.FONT_HERSHEY_SIMPLEX,
                    .44, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=IMAGES)
    parser.add_argument("--proposals", type=Path, default=PROPOSALS)
    parser.add_argument("--review-dir", type=Path, default=REVIEW_DIR)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--start", type=int, default=0,
                        help="One-based image index; 0 resumes at first unreviewed image")
    parser.add_argument("--scale", type=float, default=1.3)
    args = parser.parse_args()
    if not .5 <= args.scale <= 2:
        parser.error("--scale must be between 0.5 and 2")
    images = sorted(args.images.glob("*.jpg"))
    if not images:
        parser.error(f"No JPG images under {args.images}")
    proposals = read_proposals(args.proposals)
    missing = [p.name for p in images if p.name not in proposals]
    if missing:
        parser.error(f"No proposals for {len(missing)} images, first: {missing[0]}")
    if args.start:
        i = max(0, min(len(images)-1, args.start-1))
    else:
        i = next((n for n, p in enumerate(images)
                  if not (args.review_dir / f"{p.stem}.txt").exists()), len(images))
    if i == len(images):
        print(f"All {len(images)} images have reviewed label files.")
        return
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    try:
        while 0 <= i < len(images):
            image = images[i]
            frame = cv2.imread(str(image))
            if frame is None:
                raise RuntimeError(f"Could not read {image}")
            height, width = frame.shape[:2]
            label = args.review_dir / f"{image.stem}.txt"
            already_reviewed = label.exists()
            boxes = load_reviewed(label, width, height) if already_reviewed else [
                (max(0, x1), max(0, y1), min(width, x2), min(height, y2))
                for x1, y1, x2, y2 in proposals[image.name]
            ]
            boxes = [b for b in boxes if b[2] > b[0] and b[3] > b[1]]
            history = []
            drag_start = None
            pending = None

            def mouse(event, x, y, flags, userdata):
                nonlocal drag_start, pending, boxes
                px = min(width-1, max(0, round(x / args.scale)))
                py = min(height-1, max(0, round(y / args.scale)))
                if y >= round(height * args.scale):
                    return
                if event == cv2.EVENT_LBUTTONDOWN:
                    drag_start = (px, py)
                elif event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
                    pending = (min(drag_start[0], px), min(drag_start[1], py),
                               max(drag_start[0], px), max(drag_start[1], py))
                elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
                    x1, y1 = drag_start
                    box = (min(x1, px), min(y1, py), max(x1, px)+1, max(y1, py)+1)
                    drag_start = pending = None
                    if box[2]-box[0] >= 8 and box[3]-box[1] >= 8:
                        history.append(boxes.copy())
                        boxes.append(box)
                elif event == cv2.EVENT_RBUTTONDOWN:
                    containing = [(n, (b[2]-b[0])*(b[3]-b[1])) for n,b in enumerate(boxes)
                                  if b[0] <= px <= b[2] and b[1] <= py <= b[3]]
                    if containing:
                        history.append(boxes.copy())
                        boxes.pop(min(containing, key=lambda item: item[1])[0])

            cv2.setMouseCallback(WINDOW, mouse)
            while True:
                cv2.imshow(WINDOW, draw(frame, boxes, pending, i, len(images),
                                        image.name, already_reviewed, args.scale))
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    return
                if key in (ord("u"), ord("U")) and history:
                    boxes = history.pop()
                elif key in (ord("r"), ord("R")):
                    history.append(boxes.copy())
                    boxes = [tuple(round(v) for v in box) for box in proposals[image.name]]
                elif key in (ord("s"), ord("S")):
                    if not boxes:
                        print("No boxes: press E to confirm an empty scene, or draw cubes.")
                        continue
                    save_reviewed(image, boxes, width, height, args.review_dir, args.manifest)
                    print(f"Reviewed {i+1}/{len(images)}: {image.name}, {len(boxes)} cubes")
                    i += 1
                    break
                elif key in (ord("e"), ord("E")):
                    if boxes:
                        print("Remove proposal boxes before confirming an empty scene.")
                        continue
                    save_reviewed(image, [], width, height, args.review_dir, args.manifest)
                    print(f"Reviewed empty {i+1}/{len(images)}: {image.name}")
                    i += 1
                    break
                elif key in (ord("k"), ord("K")):
                    i += 1
                    break
                elif key in (ord("p"), ord("P")):
                    i = max(0, i-1)
                    break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
