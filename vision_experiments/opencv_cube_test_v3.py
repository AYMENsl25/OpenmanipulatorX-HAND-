"""Automatically tune segmentation for one or two black cubes at SCAN_UP."""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np


CAMERA_INDEX = 1
SCAN_POSE = "scan_up"
# Provisional repeated reference. This program records it but never moves the robot.
SCAN_UP_MOTORS_DEG = (170.156, 161.895, 333.896, 125.420)
ROOT = Path(__file__).resolve().parent.parent / "data" / "opencv_tests_v3"
IMAGE_DIR = ROOT / "images"
CSV_PATH = ROOT / "opencv_test_results_v3.csv"
WINDOW = "OpenCV cube v3 | 0 manual 1 Otsu 2 auto sweep | S save Q quit"
MASK_WINDOW = "OpenCV cube v3 mask"
CSV_FIELDS = [
    "test_id", "timestamp", "scan_pose", "mount_status", "motor_angles_deg",
    "threshold_mode", "threshold_used", "object_id", "center_u", "center_v",
    "width_px", "height_px", "area_px", "aspect_ratio", "rectangularity",
    "solidity", "mean_gray", "processing_ms", "detected_objects",
]


def noop(_value):
    pass


def make_mask(gray, mode, manual_threshold):
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    if mode == 1:
        used, mask = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        mode_name = "otsu"
    else:
        used, mask = cv2.threshold(
            blurred, manual_threshold, 255, cv2.THRESH_BINARY_INV
        )
        mode_name = "manual"
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask, mode_name, round(float(used), 2)


def find_candidates(frame, mask, min_area, min_rectangularity, min_solidity,
                    *, include_edge=False):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_h, frame_w = gray.shape
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, axis_w, axis_h = cv2.boundingRect(contour)
        touches_edge = (x <= 0 or y <= 0 or x + axis_w >= frame_w
                        or y + axis_h >= frame_h)
        if touches_edge and not include_edge:
            continue

        rotated = cv2.minAreaRect(contour)
        width, height = rotated[1]
        if width <= 0 or height <= 0:
            continue
        long_side, short_side = max(width, height), min(width, height)
        aspect = long_side / short_side
        rectangularity = area / (width * height)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull_area if hull_area > 0 else 0
        if aspect > 2.0:
            continue
        if rectangularity < min_rectangularity or solidity < min_solidity:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center_u = round(moments["m10"] / moments["m00"])
        center_v = round(moments["m01"] / moments["m00"])
        contour_mask = np.zeros_like(gray)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        mean_gray = cv2.mean(gray, mask=contour_mask)[0]
        candidates.append({
            "contour": contour,
            "rotated_box": cv2.boxPoints(rotated).astype(np.int32),
            "center_u": center_u,
            "center_v": center_v,
            "width_px": round(short_side, 1),
            "height_px": round(long_side, 1),
            "area_px": area,
            "aspect_ratio": aspect,
            "rectangularity": rectangularity,
            "solidity": solidity,
            "mean_gray": mean_gray,
            "touches_image_edge": touches_edge,
            "bbox_xywh": (x, y, axis_w, axis_h),
        })
    candidates.sort(key=lambda item: (item["center_u"], item["center_v"]))
    for number, item in enumerate(candidates, start=1):
        item["object_id"] = f"cube_{number}"
    return candidates


def automatic_threshold_sweep(frame, min_area, min_rectangularity, min_solidity):
    """Choose the highest threshold producing the most valid objects, up to two."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    otsu_threshold, _ = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    thresholds = list(range(25, round(otsu_threshold) + 1, 5))
    if round(otsu_threshold) not in thresholds:
        thresholds.append(round(otsu_threshold))

    best = None
    for threshold in thresholds:
        mask, _, used = make_mask(gray, 0, threshold)
        candidates = find_candidates(
            frame, mask, min_area, min_rectangularity, min_solidity
        )
        # This experiment has at most two physical cubes. More regions indicate
        # that the threshold is admitting background noise.
        if len(candidates) > 2:
            continue
        score = (len(candidates), threshold)
        if best is None or score > best[0]:
            best = (score, mask, candidates, used)

    if best is None:
        mask, _, used = make_mask(gray, 1, 0)
        return mask, [], "auto_sweep", used
    _, mask, candidates, used = best
    return mask, candidates, "auto_sweep", used


def annotate(frame, candidates, mode_name, threshold_used, processing_ms):
    view = frame.copy()
    for item in candidates:
        cv2.drawContours(view, [item["contour"]], -1, (0, 255, 0), 2)
        cv2.polylines(view, [item["rotated_box"]], True, (255, 180, 0), 2)
        center = (item["center_u"], item["center_v"])
        cv2.circle(view, center, 5, (0, 0, 255), -1)
        label = (
            f"{item['object_id']} ({center[0]},{center[1]}) "
            f"{item['width_px']:.0f}x{item['height_px']:.0f}px"
        )
        cv2.putText(view, label, (max(5, center[0] - 80), max(20, center[1] - 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, label, (max(5, center[0] - 80), max(20, center[1] - 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA)
    status = (f"{SCAN_POSE} | {mode_name} threshold={threshold_used:g} | "
              f"objects={len(candidates)} | {processing_ms:.1f} ms")
    cv2.putText(view, status, (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, status, (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1,
                cv2.LINE_AA)
    return view


def save_test(raw, view, mask, candidates, mode_name, threshold_used,
              processing_ms):
    timestamp = datetime.now().astimezone()
    test_id = timestamp.strftime("test_%Y%m%d_%H%M%S_%f")
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    paths = {
        "raw": IMAGE_DIR / f"{test_id}_raw.png",
        "view": IMAGE_DIR / f"{test_id}_annotated.png",
        "mask": IMAGE_DIR / f"{test_id}_mask.png",
    }
    for key, image in (("raw", raw), ("view", view), ("mask", mask)):
        if not cv2.imwrite(str(paths[key]), image):
            raise RuntimeError(f"Could not save {paths[key]}")

    ROOT.mkdir(parents=True, exist_ok=True)
    new_file = not CSV_PATH.exists()
    rows = candidates or [{key: "" for key in (
        "center_u", "center_v", "width_px", "height_px", "area_px",
        "aspect_ratio", "rectangularity", "solidity", "mean_gray"
    )} | {"object_id": "none"}]
    with CSV_PATH.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        for item in rows:
            writer.writerow({
                "test_id": test_id,
                "timestamp": timestamp.isoformat(timespec="milliseconds"),
                "scan_pose": SCAN_POSE,
                "mount_status": "fixed_rigid",
                "motor_angles_deg": ";".join(map(str, SCAN_UP_MOTORS_DEG)),
                "threshold_mode": mode_name,
                "threshold_used": threshold_used,
                "object_id": item["object_id"],
                "center_u": item["center_u"],
                "center_v": item["center_v"],
                "width_px": item["width_px"],
                "height_px": item["height_px"],
                "area_px": item["area_px"],
                "aspect_ratio": item["aspect_ratio"],
                "rectangularity": item["rectangularity"],
                "solidity": item["solidity"],
                "mean_gray": item["mean_gray"],
                "processing_ms": round(processing_ms, 3),
                "detected_objects": len(candidates),
            })
    print(f"Saved {test_id}: {len(candidates)} object(s), "
          f"{mode_name} threshold={threshold_used:g}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=CAMERA_INDEX)
    args = parser.parse_args()
    camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Could not open camera index {args.index}.")

    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("Mode 0 manual 1 Otsu 2 auto", WINDOW, 2, 2, noop)
    cv2.createTrackbar("Manual threshold", WINDOW, 85, 255, noop)
    cv2.createTrackbar("Min area px", WINDOW, 200, 10000, noop)
    cv2.createTrackbar("Min rectangularity %", WINDOW, 55, 100, noop)
    cv2.createTrackbar("Min solidity %", WINDOW, 80, 100, noop)
    print(f"Camera {args.index}; fixed mount; pose {SCAN_POSE}")
    print(f"Provisional motor reference: {SCAN_UP_MOTORS_DEG}")
    print("Mode 2 automatically searches thresholds for up to two cubes.")
    print("Modes 0 and 1 remain available for comparison. S saves; Q quits.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("Camera frame read failed.")
                break
            started = time.perf_counter()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mode = cv2.getTrackbarPos("Mode 0 manual 1 Otsu 2 auto", WINDOW)
            manual = cv2.getTrackbarPos("Manual threshold", WINDOW)
            min_area = cv2.getTrackbarPos("Min area px", WINDOW)
            min_rect = cv2.getTrackbarPos("Min rectangularity %", WINDOW) / 100
            min_solidity = cv2.getTrackbarPos("Min solidity %", WINDOW) / 100
            if mode == 2:
                mask, candidates, mode_name, threshold_used = (
                    automatic_threshold_sweep(
                        frame, min_area, min_rect, min_solidity
                    )
                )
            else:
                mask, mode_name, threshold_used = make_mask(gray, mode, manual)
                candidates = find_candidates(
                    frame, mask, min_area, min_rect, min_solidity
                )
            processing_ms = (time.perf_counter() - started) * 1000
            view = annotate(frame, candidates, mode_name, threshold_used,
                            processing_ms)
            cv2.imshow(WINDOW, view)
            cv2.imshow(MASK_WINDOW, mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                save_test(frame, view, mask, candidates, mode_name,
                          threshold_used, processing_ms)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
