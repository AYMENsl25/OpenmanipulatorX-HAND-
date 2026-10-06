"""Step 1: inspect a USB camera without connecting to the robot."""

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2


SAVE_DIR = Path(__file__).resolve().parent.parent / "data" / "camera_test"
WINDOW_NAME = "Piranha camera test | S: save  Q: quit"


def open_camera(requested_index):
    indices = [requested_index] if requested_index is not None else range(6)
    for index in indices:
        capture = cv2.VideoCapture(index)
        if capture.isOpened():
            ok, frame = capture.read()
            if ok and frame is not None:
                return capture, index, frame
        capture.release()
    raise RuntimeError(
        "No usable camera found. Try --index 0, --index 1, etc.; "
        "close other applications using the webcam."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--index", type=int, default=None,
        help="Camera index (default: try 0 through 5).",
    )
    args = parser.parse_args()

    capture, index, frame = open_camera(args.index)
    reported_fps = capture.get(cv2.CAP_PROP_FPS)
    print(f"Camera index: {index}")
    print(f"First frame: {frame.shape[1]} x {frame.shape[0]} pixels")
    print(f"Camera-reported FPS: {reported_fps:.2f}" if reported_fps > 0 else
          "Camera-reported FPS: unavailable")
    print("Press S to save a raw frame; press Q to quit.")

    frames_in_window = 0
    fps_window_start = time.monotonic()
    measured_fps = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("Camera frame read failed; stopping.")
                break

            frames_in_window += 1
            now = time.monotonic()
            elapsed = now - fps_window_start
            if elapsed >= 2.0:
                measured_fps = frames_in_window / elapsed
                frames_in_window = 0
                fps_window_start = now

            height, width = frame.shape[:2]
            preview = frame.copy()
            lines = [
                f"Camera {index}  |  {width} x {height}",
                f"Reported FPS: {reported_fps:.1f}" if reported_fps > 0
                else "Reported FPS: unavailable",
                f"Measured FPS: {measured_fps:.1f}" if measured_fps is not None
                else "Measured FPS: measuring...",
                "S: save image   Q: quit",
            ]
            for row, line in enumerate(lines):
                y = 28 + row * 29
                cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW_NAME, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                SAVE_DIR.mkdir(parents=True, exist_ok=True)
                filename = datetime.now().strftime("camera_%Y%m%d_%H%M%S_%f.jpg")
                path = SAVE_DIR / filename
                if cv2.imwrite(str(path), frame):
                    print(f"Saved: {path}")
                else:
                    print(f"Failed to save: {path}")
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, cv2.error) as exc:
        raise SystemExit(f"Camera test error: {exc}") from exc
