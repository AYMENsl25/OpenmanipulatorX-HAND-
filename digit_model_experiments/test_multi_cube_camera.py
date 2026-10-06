"""Experimental full-frame search for white cube faces and their visible digits.

This is a camera-only prototype. It does not command the robot or choose a grasp.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from PIL import Image

from test_digit_camera import PREPROCESS, load_models


WINDOW = "Wide cube digit search | S: save view | Q: quit"
OUTPUT = Path(__file__).resolve().parent / "camera_tests" / "wide_area"


def overlap_fraction(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    width = max(0, min(ax2, bx2) - max(ax1, bx1))
    height = max(0, min(ay2, by2) - max(ay1, by1))
    intersection = width * height
    smaller = min((ax2 - ax1) * (ay2 - ay1), (bx2 - bx1) * (by2 - by1))
    return intersection / smaller if smaller else 0.0


def find_white_faces(frame, threshold, min_area, max_faces):
    """Return candidate face boxes; lighting and background may require tuning."""
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    mask = np.where((value >= threshold) & (saturation <= 110), 255, 0).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    frame_area = height * width
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > frame_area * 0.45:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if min(w, h) < 25 or not 0.55 <= w / h <= 1.85:
            continue
        if area / (w * h) < 0.48:
            continue
        pad = max(3, int(min(w, h) * 0.04))
        box = (max(0, x - pad), max(0, y - pad), min(width, x + w + pad),
               min(height, y + h + pad))
        candidates.append((area, box))
    chosen = []
    for _area, box in sorted(candidates, reverse=True):
        if all(overlap_fraction(box, previous) < 0.6 for previous in chosen):
            chosen.append(box)
        if len(chosen) == max_faces:
            break
    return sorted(chosen, key=lambda box: (box[1], box[0])), mask


def predict_rotations(crop, models, device):
    """Search four orientations; each maximum score is a hypothesis, not a certainty."""
    inputs = []
    for turns in range(4):
        rgb = cv2.cvtColor(np.rot90(crop, k=turns).copy(), cv2.COLOR_BGR2RGB)
        inputs.append(PREPROCESS(Image.fromarray(rgb)))
    batch = torch.stack(inputs).to(device)
    results = {}
    with torch.inference_mode():
        for name, model in models.items():
            probabilities = model(batch).softmax(dim=1)
            scores, digits = probabilities.max(dim=1)
            best_turn = int(scores.argmax().item())
            results[name] = (int(digits[best_turn].item()),
                             float(scores[best_turn].item()), best_turn * 90)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=1, help="Camera index")
    parser.add_argument("--white-threshold", type=int, default=145,
                        help="Minimum HSV brightness for white cube faces (0-255)")
    parser.add_argument("--min-area", type=int, default=1200,
                        help="Minimum visible white-face area in pixels")
    parser.add_argument("--max-faces", type=int, default=8)
    parser.add_argument("--interval", type=float, default=0.7,
                        help="Seconds between detection updates")
    args = parser.parse_args()
    if not 0 <= args.white_threshold <= 255 or args.min_area <= 0 or args.max_faces <= 0 or args.interval <= 0:
        parser.error("Threshold must be 0-255 and area, face count, and interval must be positive")
    device = torch.device("cpu")
    models = load_models(device)
    camera = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise RuntimeError(f"Camera {args.index} did not open")
    last_scan = 0.0
    annotated = None
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("Camera stopped returning frames")
            if annotated is None or time.monotonic() - last_scan >= args.interval:
                annotated = frame.copy()
                boxes, _mask = find_white_faces(frame, args.white_threshold,
                                                 args.min_area, args.max_faces)
                for number, (x1, y1, x2, y2) in enumerate(boxes, 1):
                    crop = frame[y1:y2, x1:x2]
                    results = predict_rotations(crop, models, device)
                    mobile = results["MobileNetV3-Small"]
                    resnet = results["ResNet18"]
                    agree = mobile[0] == resnet[0]
                    color = (0, 210, 0) if agree else (0, 165, 255)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    line1 = f"#{number} M:{mobile[0]} {mobile[1]:.0%} @ {mobile[2]}deg"
                    line2 = f"R:{resnet[0]} {resnet[1]:.0%} @ {resnet[2]}deg"
                    label_y = max(20, y1 - 26)
                    cv2.putText(annotated, line1, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.47, color, 2, cv2.LINE_AA)
                    cv2.putText(annotated, line2, (x1, label_y + 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.47, color, 2, cv2.LINE_AA)
                cv2.putText(annotated, f"Candidates: {len(boxes)} | M=MobileNet R=ResNet | S save Q quit",
                            (10, annotated.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, (255, 255, 255), 2, cv2.LINE_AA)
                last_scan = time.monotonic()
            cv2.imshow(WINDOW, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):
                OUTPUT.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                raw_path = OUTPUT / f"{stamp}_raw.jpg"
                overlay_path = OUTPUT / f"{stamp}_detections.jpg"
                cv2.imwrite(str(raw_path), frame)
                cv2.imwrite(str(overlay_path), annotated)
                print(f"Saved {raw_path} and {overlay_path}")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError, cv2.error) as exc:
        raise SystemExit(f"Wide-area camera test error: {exc}") from exc
