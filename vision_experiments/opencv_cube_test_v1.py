"""Benchmark black-cube detection in pixels; never commands the robot."""

import argparse
import csv
from datetime import datetime
from pathlib import Path

import cv2


CAMERA_INDEX = 1
WINDOW = "OpenCV cube benchmark | S: save test  Q: quit"
MASK_WINDOW = "Benchmark dark mask"
OUTPUT_ROOT = Path(__file__).resolve().parent.parent / "data" / "opencv_tests"
IMAGE_DIR = OUTPUT_ROOT / "images"
CSV_PATH = OUTPUT_ROOT / "opencv_test_results.csv"
CSV_FIELDS = [
    "test_id",
    "timestamp",
    "object_id",
    "center_u",
    "center_v",
    "width_px",
    "height_px",
    "area_px",
    "aspect_ratio",
    "detected_objects",
    "dark_threshold",
    "min_area_px",
    "min_fill_percent",
]


def noop(_value):
    """OpenCV trackbar callback."""


def find_cubes(frame, threshold, min_area, min_fill):
    """Return candidate measurements and the binary dark-pixel mask."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    frame_h, frame_w = gray.shape
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        if width == 0 or height == 0 or area < min_area:
            continue

        # Regions touching the image border are normally the table or scene edge.
        if x <= 0 or y <= 0 or x + width >= frame_w or y + height >= frame_h:
            continue

        aspect_ratio = width / height
        rectangularity = area / (width * height)
        if not (0.50 <= aspect_ratio <= 2.00 and rectangularity >= min_fill):
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center_u = round(moments["m10"] / moments["m00"])
        center_v = round(moments["m01"] / moments["m00"])
        candidates.append({
            "contour": contour,
            "x": x,
            "y": y,
            "center_u": center_u,
            "center_v": center_v,
            "width_px": width,
            "height_px": height,
            "area_px": area,
            "aspect_ratio": aspect_ratio,
        })

    # Stable IDs for a still scene: left-to-right, then top-to-bottom.
    candidates.sort(key=lambda item: (item["center_u"], item["center_v"]))
    for number, candidate in enumerate(candidates, start=1):
        candidate["object_id"] = f"cube_{number}"
    return candidates, mask


def annotate(frame, candidates, threshold, min_area, min_fill):
    view = frame.copy()
    for item in candidates:
        contour = item["contour"]
        x, y, width, height = cv2.boundingRect(contour)
        cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
        cv2.rectangle(view, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.circle(view, (item["center_u"], item["center_v"]),
                   5, (0, 0, 255), -1)
        lines = [
            item["object_id"],
            f"center=({item['center_u']},{item['center_v']})",
            f"size={item['width_px']}x{item['height_px']} px",
            f"area={item['area_px']:.0f}  aspect={item['aspect_ratio']:.2f}",
        ]
        text_x = max(5, min(x, frame.shape[1] - 245))
        text_y = max(22, y - 68)
        for row, line in enumerate(lines):
            position = (text_x, text_y + row * 18)
            cv2.putText(view, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(view, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (0, 255, 0), 1, cv2.LINE_AA)

    status = (
        f"Detected objects: {len(candidates)} | threshold={threshold} "
        f"min_area={min_area} min_fill={round(min_fill * 100)}%"
    )
    cv2.putText(view, status, (10, frame.shape[0] - 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, status, (10, frame.shape[0] - 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1,
                cv2.LINE_AA)
    cv2.putText(view, "S: save test   Q: quit", (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, "S: save test   Q: quit", (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1,
                cv2.LINE_AA)
    return view


def save_test(view, mask, candidates, threshold, min_area, min_fill):
    timestamp = datetime.now().astimezone()
    test_id = timestamp.strftime("test_%Y%m%d_%H%M%S_%f")
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    image_path = IMAGE_DIR / f"{test_id}.png"
    mask_path = IMAGE_DIR / f"{test_id}_mask.png"
    cv2.imwrite(str(image_path), view)
    cv2.imwrite(str(mask_path), mask)

    new_file = not CSV_PATH.exists()
    with CSV_PATH.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        rows = candidates or [{
            "object_id": "none",
            "center_u": "",
            "center_v": "",
            "width_px": "",
            "height_px": "",
            "area_px": "",
            "aspect_ratio": "",
        }]
        for item in rows:
            writer.writerow({
                "test_id": test_id,
                "timestamp": timestamp.isoformat(timespec="milliseconds"),
                "object_id": item["object_id"],
                "center_u": item["center_u"],
                "center_v": item["center_v"],
                "width_px": item["width_px"],
                "height_px": item["height_px"],
                "area_px": (f"{item['area_px']:.2f}"
                            if item["area_px"] != "" else ""),
                "aspect_ratio": (f"{item['aspect_ratio']:.4f}"
                                 if item["aspect_ratio"] != "" else ""),
                "detected_objects": len(candidates),
                "dark_threshold": threshold,
                "min_area_px": min_area,
                "min_fill_percent": round(min_fill * 100),
            })

    print(f"Saved {test_id}: {len(candidates)} object(s)")
    print(f"  image: {image_path}")
    print(f"  mask:  {mask_path}")
    if not candidates:
        print("  Miss recorded in CSV with object_id=none.")
    else:
        for item in candidates:
            print(
                f"  {item['object_id']}: center=({item['center_u']},"
                f"{item['center_v']}), size={item['width_px']}x"
                f"{item['height_px']} px, area={item['area_px']:.1f}, "
                f"aspect={item['aspect_ratio']:.3f}"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=CAMERA_INDEX,
                        help=f"Camera index (default: {CAMERA_INDEX})")
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Could not open camera index {args.index}.")

    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("Dark threshold", WINDOW, 85, 255, noop)
    cv2.createTrackbar("Min area px", WINDOW, 200, 10000, noop)
    cv2.createTrackbar("Min fill %", WINDOW, 55, 100, noop)
    print(f"Camera index: {args.index}")
    print("Press S once for each test scene; press Q to quit.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("Camera frame read failed.")
                break
            threshold = cv2.getTrackbarPos("Dark threshold", WINDOW)
            min_area = cv2.getTrackbarPos("Min area px", WINDOW)
            min_fill = cv2.getTrackbarPos("Min fill %", WINDOW) / 100.0
            candidates, mask = find_cubes(frame, threshold, min_area, min_fill)
            view = annotate(frame, candidates, threshold, min_area, min_fill)
            cv2.imshow(WINDOW, view)
            cv2.imshow(MASK_WINDOW, mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                save_test(view, mask, candidates, threshold, min_area, min_fill)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
