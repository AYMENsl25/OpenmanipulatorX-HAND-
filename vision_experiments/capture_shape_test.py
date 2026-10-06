"""Capture a held-out, fixed-camera shape test set. No robot commands are sent.

Example:
  python vision_experiments/capture_shape_test.py --index 1 --session day1_normal --scene-id s001 \
      --scene-type single --lighting normal --shapes cube --colors black \
      --scan-pose scan_primary

Press S to save an image, Q to quit. Change the scene between saves.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import re
from datetime import datetime
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent / "dataset" / "real_camera_shape_test_v1"
MANIFEST = ROOT / "capture_manifest.csv"
FIELDS = [
    "image_path", "sha256", "timestamp", "session", "split", "scene_id",
    "scene_type", "lighting", "view_angle", "shapes_present", "colors_present",
    "occlusion", "background", "scan_pose", "pose_note", "camera_mount",
    "camera_index", "width_px", "height_px", "notes", "annotated",
]
SHAPES = {"cube", "cylinder", "sphere", "pyramid", "none"}
COLORS = {"black", "green", "mixed", "none"}


def slug(value: str) -> str:
    result = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip()).strip("_").lower()
    if not result:
        raise argparse.ArgumentTypeError("Enter a nonempty name")
    return result


def choices_csv(value: str, allowed: set[str]) -> str:
    entries = [x.strip().lower() for x in value.split(",")]
    if not entries or any(x not in allowed for x in entries):
        raise argparse.ArgumentTypeError(f"Use comma-separated values from: {sorted(allowed)}")
    return ",".join(sorted(set(entries)))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--index", type=int, default=1, help="Confirm USB camera index; prior Piranha index was 1")
    p.add_argument("--session", type=slug, required=True, help="One capture block, e.g. day1_normal")
    p.add_argument("--scene-id", type=slug, required=True, help="Unique physical arrangement, e.g. s001")
    p.add_argument("--scene-type", choices=["empty", "single", "two", "three", "four", "partial", "occluded"], required=True)
    p.add_argument("--lighting", choices=["normal", "dim", "bright", "side_left", "side_right", "mixed"], required=True)
    p.add_argument("--view-angle", choices=["fixed", "slight_tilt"], default="fixed", help="Record camera view; keep fixed for the main benchmark")
    p.add_argument("--shapes", required=True, help="Comma separated: cube,cylinder,sphere,pyramid or none")
    p.add_argument("--colors", required=True, help="Comma separated: black,green,mixed or none")
    p.add_argument("--occlusion", choices=["none", "touching", "partial", "overlap"], default="none")
    p.add_argument("--background", choices=["white_table", "other"], default="white_table")
    p.add_argument("--scan-pose", type=slug, required=True, help="Recorded camera/robot pose name; script does not move arm")
    p.add_argument("--pose-note", default="", help="Recorded joint angles or camera mount note")
    p.add_argument("--notes", default="")
    p.add_argument("--output", type=Path, default=ROOT)
    args = p.parse_args(argv)
    args.shapes = choices_csv(args.shapes, SHAPES)
    args.colors = choices_csv(args.colors, COLORS)
    if args.scene_type == "empty" and (args.shapes != "none" or args.colors != "none"):
        p.error("An empty scene must use --shapes none --colors none")
    if args.scene_type != "empty" and ("none" in args.shapes.split(",") or "none" in args.colors.split(",")):
        p.error("Nonempty scenes need actual shape and color values")
    return args


def save_frame(frame, args, sequence: int):
    output = args.output.resolve()
    folder = output / "images" / args.session
    folder.mkdir(parents=True, exist_ok=True)
    now = datetime.now().astimezone()
    name = f"{args.scene_id}_{sequence:03d}_{now.strftime('%Y%m%d_%H%M%S_%f')}.jpg"
    path = folder / name
    if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise RuntimeError(f"Could not save {path}")
    width, height = frame.shape[1], frame.shape[0]
    row = {
        "image_path": path.relative_to(output).as_posix(),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "timestamp": now.isoformat(timespec="milliseconds"),
        "session": args.session, "split": "test", "scene_id": args.scene_id,
        "scene_type": args.scene_type, "lighting": args.lighting,
        "view_angle": args.view_angle, "shapes_present": args.shapes,
        "colors_present": args.colors, "occlusion": args.occlusion,
        "background": args.background, "scan_pose": args.scan_pose,
        "pose_note": args.pose_note, "camera_mount": "fixed_manual",
        "camera_index": args.index, "width_px": width, "height_px": height,
        "notes": args.notes, "annotated": "no",
    }
    manifest = output / "capture_manifest.csv"
    new = not manifest.exists()
    with manifest.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            writer.writeheader()
        writer.writerow(row)
    return path


def main(argv=None):
    args = parse_args(argv)
    camera = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Camera {args.index} could not be opened. Check its index and other apps using it.")
    existing = args.output / "images" / args.session
    sequence = len(list(existing.glob(f"{args.scene_id}_*.jpg"))) + 1 if existing.exists() else 1
    print(f"Camera {args.index}; session {args.session}; scene {args.scene_id}; pose {args.scan_pose}")
    print("Press S once per meaningfully changed arrangement; Q quits. Raw frame is saved without overlay.")
    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("Camera frame read failed")
                break
            preview = frame.copy()
            cv2.putText(preview, f"{args.session} / {args.scene_id} | S save  Q quit", (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
            cv2.imshow("Shape test capture", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")): break
            if key in (ord("s"), ord("S")):
                path = save_frame(frame, args, sequence)
                print(f"Saved {path}")
                sequence += 1
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
