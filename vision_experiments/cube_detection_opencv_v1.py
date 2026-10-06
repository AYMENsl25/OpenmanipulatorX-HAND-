"""Step 2: locate dark square candidates in camera pixels only."""

import argparse
from datetime import datetime
from pathlib import Path

import cv2


CAMERA_INDEX = 1
SAVE_DIR = Path(__file__).resolve().parent.parent / "data" / "cube_detection"
WINDOW = "Black cube candidates | S: save  Q: quit"


def nothing(_value):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=CAMERA_INDEX)
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Could not open camera index {args.index}.")

    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("Dark threshold", WINDOW, 90, 255, nothing)
    cv2.createTrackbar("Min area px", WINDOW, 200, 10000, nothing)
    cv2.createTrackbar("Min fill %", WINDOW, 60, 100, nothing)
    print(f"Camera index: {args.index}")
    print("Use the sliders to isolate the black cube. S saves a view; Q quits.")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("Frame read failed.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            threshold = cv2.getTrackbarPos("Dark threshold", WINDOW)
            min_area = cv2.getTrackbarPos("Min area px", WINDOW)
            min_fill = cv2.getTrackbarPos("Min fill %", WINDOW) / 100.0
            _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            preview = frame.copy()
            candidates = []
            height, width = gray.shape
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, box_w, box_h = cv2.boundingRect(contour)
                if box_w == 0 or box_h == 0 or area < min_area:
                    continue
                if x <= 0 or y <= 0 or x + box_w >= width or y + box_h >= height:
                    continue
                aspect = box_w / box_h
                fill = area / (box_w * box_h)
                if not (0.55 <= aspect <= 1.8 and fill >= min_fill):
                    continue

                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue
                u = round(moments["m10"] / moments["m00"])
                v = round(moments["m01"] / moments["m00"])
                candidates.append((u, v, box_w, box_h))
                cv2.drawContours(preview, [contour], -1, (0, 255, 0), 2)
                cv2.rectangle(preview, (x, y), (x + box_w, y + box_h),
                              (0, 255, 0), 2)
                cv2.circle(preview, (u, v), 4, (0, 0, 255), -1)
                label_y = y - 8 if y >= 20 else y + box_h + 20
                cv2.putText(preview, f"CUBE? u={u} v={v} {box_w}x{box_h}px",
                            (x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2, cv2.LINE_AA)

            cv2.putText(preview, f"Candidates: {len(candidates)}  S: save  Q: quit",
                        (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, preview)
            cv2.imshow("Dark mask", mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                SAVE_DIR.mkdir(parents=True, exist_ok=True)
                path = SAVE_DIR / datetime.now().strftime("cube_%Y%m%d_%H%M%S_%f.png")
                if cv2.imwrite(str(path), preview):
                    print(f"Saved: {path} | candidates: {candidates}")
                else:
                    print(f"Failed to save: {path}")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
