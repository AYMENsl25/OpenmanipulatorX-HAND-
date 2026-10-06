"""Capture clean Piranha-camera images for the cube dataset; no robot control."""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import re

import cv2


CAMERA_INDEX = 1
DATASET_ROOT = Path(__file__).resolve().parent.parent / "dataset" / "raw"
MANIFEST_PATH = DATASET_ROOT / "capture_manifest.csv"
WINDOW = "Cube dataset capture | S: save raw image  Q: quit"
MANIFEST_FIELDS = [
    "image_path",
    "timestamp",
    "session",
    "camera_mount",
    "scan_pose",
    "pose_note",
    "camera_index",
    "width_px",
    "height_px",
]


def safe_session_name(value):
    name = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip()).strip("_")
    if not name:
        raise argparse.ArgumentTypeError("session must contain letters or numbers")
    return name.lower()


def existing_count(session_dir):
    return sum(1 for path in session_dir.glob("*.jpg") if path.is_file())


def save_frame(frame, session, scan_pose, pose_note, camera_index):
    session_dir = DATASET_ROOT / session
    session_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone()
    filename = timestamp.strftime("cube_%Y%m%d_%H%M%S_%f.jpg")
    image_path = session_dir / filename
    if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise RuntimeError(f"Could not save {image_path}")

    DATASET_ROOT.mkdir(parents=True, exist_ok=True)
    new_manifest = not MANIFEST_PATH.exists()
    relative_path = image_path.relative_to(Path(__file__).resolve().parent.parent)
    height, width = frame.shape[:2]
    with MANIFEST_PATH.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=MANIFEST_FIELDS)
        if new_manifest:
            writer.writeheader()
        writer.writerow({
            "image_path": relative_path.as_posix(),
            "timestamp": timestamp.isoformat(timespec="milliseconds"),
            "session": session,
            "camera_mount": "fixed_rigid",
            "scan_pose": scan_pose,
            "pose_note": pose_note,
            "camera_index": camera_index,
            "width_px": width,
            "height_px": height,
        })
    return image_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--session",
        type=safe_session_name,
        required=True,
        help="Scene group, for example one_cube_normal_light",
    )
    parser.add_argument(
        "--scan-pose",
        type=safe_session_name,
        required=True,
        help="Repeatable robot camera pose, for example scan_primary",
    )
    parser.add_argument(
        "--pose-note",
        default="",
        help="Optional recorded joint angles or other pose reference",
    )
    parser.add_argument("--index", type=int, default=CAMERA_INDEX,
                        help=f"Camera index (default: {CAMERA_INDEX})")
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Could not open camera index {args.index}.")

    session_dir = DATASET_ROOT / args.session
    count = existing_count(session_dir)
    print(f"Camera index: {args.index}")
    print(f"Session: {args.session}")
    print("Camera mount: fixed_rigid")
    print(f"Scan pose: {args.scan_pose}")
    if args.pose_note:
        print(f"Pose note: {args.pose_note}")
    print(f"Existing images in session: {count}")
    print("Press S only when the scene has changed; press Q to quit.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("Camera frame read failed.")
                break

            preview = frame.copy()
            status = (f"Fixed mount | Pose: {args.scan_pose} | "
                      f"Saved: {count}")
            cv2.putText(preview, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(preview, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(preview, "S: save raw image   Q: quit", (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(preview, "S: save raw image   Q: quit", (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                        (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                path = save_frame(
                    frame,
                    args.session,
                    args.scan_pose,
                    args.pose_note,
                    args.index,
                )
                count += 1
                print(f"Saved {count}: {path}")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
