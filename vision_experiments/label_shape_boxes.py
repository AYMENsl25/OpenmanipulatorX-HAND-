r"""Manually label fixed-camera test images with YOLO bounding boxes.

Run from the project folder with: .\venv\Scripts\python.exe .\vision_experiments\label_shape_boxes.py
No model predictions or robot commands are used.
"""
from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent / "dataset" / "real_camera_shape_test_v1"
CLASSES = ("cube", "cylinder", "sphere", "pyramid")
CLASS_COLORS = ((255, 128, 0), (0, 200, 255), (200, 50, 200), (0, 255, 0))
OBJECT_FIELDS = ("image_path", "object_index", "class_id", "class_name", "color", "x1", "y1", "x2", "y2")


def yolo_line(class_id: int, box: tuple[int, int, int, int], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    if not 0 <= class_id < len(CLASSES) or not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError(f"Invalid class or bbox: {class_id}, {box}, image {width}x{height}")
    cx, cy = ((x1 + x2) / 2 / width, (y1 + y2) / 2 / height)
    bw, bh = ((x2 - x1) / width, (y2 - y1) / height)
    return f"{class_id} {cx:.8f} {cy:.8f} {bw:.8f} {bh:.8f}"


def read_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_rows(path: Path, rows: list[dict], fields: tuple[str, ...] | list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temp.replace(path)


def image_key(root: Path, image: Path) -> str:
    return image.relative_to(root).as_posix()


def label_path(root: Path, image: Path) -> Path:
    return root / "labels" / image.relative_to(root / "images").with_suffix(".txt")


def load_boxes(root: Path, image: Path, width: int, height: int) -> list[dict]:
    path = label_path(root, image)
    colors = {int(row["object_index"]): row["color"] for row in read_rows(root / "object_annotations.csv") if row["image_path"] == image_key(root, image)}
    boxes = []
    if not path.exists():
        return boxes
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines()):
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 5:
            raise ValueError(f"Expected a YOLO box in {path}: {line}")
        cid = int(parts[0])
        cx, cy, bw, bh = map(float, parts[1:])
        box = (round((cx - bw / 2) * width), round((cy - bh / 2) * height), round((cx + bw / 2) * width), round((cy + bh / 2) * height))
        boxes.append({"class_id": cid, "color": colors.get(index, "unknown"), "box": box})
    return boxes


def save_annotation(root: Path, image: Path, width: int, height: int, boxes: list[dict]) -> Path:
    key = image_key(root, image)
    label = label_path(root, image)
    label.parent.mkdir(parents=True, exist_ok=True)
    lines = [yolo_line(item["class_id"], item["box"], width, height) for item in boxes]
    label.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    objects_path = root / "object_annotations.csv"
    object_rows = [row for row in read_rows(objects_path) if row["image_path"] != key]
    for index, item in enumerate(boxes):
        x1, y1, x2, y2 = item["box"]
        object_rows.append({"image_path": key, "object_index": index, "class_id": item["class_id"], "class_name": CLASSES[item["class_id"]], "color": item["color"], "x1": x1, "y1": y1, "x2": x2, "y2": y2})
    write_rows(objects_path, object_rows, OBJECT_FIELDS)

    manifest = root / "capture_manifest.csv"
    if manifest.exists():
        backup = root / "capture_manifest_original.csv"
        if not backup.exists():
            shutil.copy2(manifest, backup)
        with manifest.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            fields = reader.fieldnames or []
            rows = list(reader)
        for row in rows:
            if row.get("image_path") == key:
                row["annotated"] = "yes"
                row["shapes_present"] = ",".join(sorted({CLASSES[item["class_id"]] for item in boxes})) if boxes else "none"
                row["colors_present"] = ",".join(sorted({item["color"] for item in boxes})) if boxes else "none"
                row["scene_type"] = ("empty" if not boxes else "single" if len(boxes) == 1 else "two" if len(boxes) == 2 else "three" if len(boxes) == 3 else "four" if len(boxes) == 4 else "multi")
        write_rows(manifest, rows, fields)
    return label


def draw_view(frame, boxes, selected_class, selected_color, pending, scale, index, total, image_name):
    view = frame.copy()
    for n, item in enumerate(boxes, 1):
        x1, y1, x2, y2 = item["box"]
        color = CLASS_COLORS[item["class_id"]]
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        cv2.putText(view, f"{n} {CLASSES[item['class_id']]} {item['color']}", (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 2)
    if pending:
        x1, y1, x2, y2 = pending
        cv2.rectangle(view, (x1, y1), (x2, y2), (255, 255, 255), 1)
    if scale != 1:
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    footer = 92
    canvas = cv2.copyMakeBorder(view, 0, footer, 0, 0, cv2.BORDER_CONSTANT, value=(32, 32, 32))
    lines = [f"{index + 1}/{total}  {image_name}",
             f"Class: {CLASSES[selected_class] if selected_class is not None else 'SELECT 1-4'} | Color: {selected_color or 'SELECT B/G'} | Boxes: {len(boxes)}",
             "1 cube  2 cylinder  3 sphere  4 pyramid | B black  G green | mouse: draw box",
             "S save+next  E empty+next  U undo  C clear  K skip  P previous  Q quit"]
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (6, view.shape[0] + 18 + i * 21), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT)
    parser.add_argument("--session", default=None, help="One image session, e.g. day1_normal; default: all sessions")
    parser.add_argument("--start", type=int, default=1, help="One-based image number after filename sorting")
    parser.add_argument("--scale", type=float, default=1.5, help="Display zoom; boxes are saved in original pixels")
    args = parser.parse_args(argv)
    if not 1 <= args.scale <= 3:
        parser.error("--scale must be between 1 and 3")
    root = args.dataset.resolve()
    base = root / "images" / args.session if args.session else root / "images"
    images = sorted(p for p in base.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        parser.error(f"No images under {base}")
    i = max(0, min(args.start - 1, len(images) - 1))
    window = "Manual shape bounding boxes"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    try:
        while 0 <= i < len(images):
            image = images[i]
            frame = cv2.imread(str(image))
            if frame is None:
                print(f"Unreadable: {image}")
                i += 1
                continue
            height, width = frame.shape[:2]
            boxes = load_boxes(root, image, width, height)
            selected_class = None
            selected_color = None
            drag_start = None
            pending = None

            def mouse(event, x, y, flags, userdata):
                nonlocal drag_start, pending
                px, py = min(width - 1, max(0, round(x / args.scale))), min(height - 1, max(0, round(y / args.scale)))
                if y >= round(height * args.scale):
                    return
                if event == cv2.EVENT_LBUTTONDOWN:
                    drag_start = (px, py)
                elif event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
                    pending = (min(drag_start[0], px), min(drag_start[1], py), max(drag_start[0], px), max(drag_start[1], py))
                elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
                    x1, y1 = drag_start
                    box = (min(x1, px), min(y1, py), max(x1, px) + 1, max(y1, py) + 1)
                    drag_start = None
                    pending = None
                    if selected_class is None or selected_color is None:
                        print("Select class 1-4 and color B/G before drawing.")
                    elif box[2] - box[0] < 8 or box[3] - box[1] < 8:
                        print("Box too small; draw at least 8x8 pixels.")
                    else:
                        boxes.append({"class_id": selected_class, "color": selected_color, "box": box})

            cv2.setMouseCallback(window, mouse)
            while True:
                cv2.imshow(window, draw_view(frame, boxes, selected_class, selected_color, pending, args.scale, i, len(images), image.name))
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("1"), ord("2"), ord("3"), ord("4")):
                    selected_class = key - ord("1")
                elif key in (ord("b"), ord("B")):
                    selected_color = "black"
                elif key in (ord("g"), ord("G")):
                    selected_color = "green"
                elif key in (ord("u"), ord("U")) and boxes:
                    boxes.pop()
                elif key in (ord("c"), ord("C")):
                    boxes.clear()
                elif key in (ord("s"), ord("S")):
                    if not boxes:
                        print("No boxes. Press E to confirm a truly empty image, or draw every visible object.")
                        continue
                    if any(item["color"] not in ("black", "green") for item in boxes):
                        print("Existing box has unknown color; clear and redraw it before saving.")
                        continue
                    path = save_annotation(root, image, width, height, boxes)
                    print(f"Saved {len(boxes)} boxes: {path}")
                    i += 1
                    break
                elif key in (ord("e"), ord("E")):
                    if boxes:
                        print("Boxes are present. Press C first only if the scene really contains no objects.")
                        continue
                    path = save_annotation(root, image, width, height, [])
                    print(f"Saved empty scene: {path}")
                    i += 1
                    break
                elif key in (ord("k"), ord("K")):
                    i += 1
                    break
                elif key in (ord("p"), ord("P")):
                    i = max(0, i - 1)
                    break
                elif key in (ord("q"), ord("Q"), 27):
                    return
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
