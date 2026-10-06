r"""Score two YOLO checkpoints on the same labeled real-camera test images.

Labels: dataset/real_camera_shape_test_v1/labels/<session>/<image-stem>.txt
Canonical YOLO box IDs: 0 cube, 1 cylinder, 2 sphere, 3 pyramid.

Example:
  python vision_experiments/evaluate_shape_test.py --colored "C:\path\E0_baseline_best.pt" \
      --robotic "C:\path\E0_baseline_best.pt"
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

CLASSES = ("cube", "cylinder", "sphere", "pyramid")
COMMON = frozenset(("cube", "cylinder", "sphere"))


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    aa = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    bb = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return inter / (aa + bb - inter) if aa + bb > inter else 0.0


def read_truth(image, root):
    relative = image.relative_to(root / "images")
    label = root / "labels" / relative.with_suffix(".txt")
    if not label.exists():
        raise FileNotFoundError(f"Missing ground-truth label: {label}. Label every test image, including empty scenes with an empty file.")
    frame = cv2.imread(str(image))
    if frame is None:
        raise ValueError(f"Unreadable image: {image}")
    h, w = frame.shape[:2]
    truth = []
    for line in label.read_text(encoding="utf-8").splitlines():
        if not line.strip(): continue
        values = line.split()
        if len(values) != 5:
            raise ValueError(f"Expected YOLO bbox label in {label}: {line}")
        cid = int(values[0]); cx, cy, bw, bh = map(float, values[1:])
        if not 0 <= cid < len(CLASSES) or not 0 <= cx <= 1 or not 0 <= cy <= 1 or not 0 < bw <= 1 or not 0 < bh <= 1:
            raise ValueError(f"Invalid label: {label}: {line}")
        truth.append({"class": CLASSES[cid], "bbox": [(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h], "center": (cx * w, cy * h)})
    return truth, frame


def predict(model, image, conf):
    result = model.predict(str(image), conf=conf, verbose=False)[0]
    found = []
    for i, box in enumerate(result.boxes):
        name = model.names[int(box.cls.item())].lower().strip()
        xy = box.xyxy[0].cpu().numpy().tolist()
        center = ((xy[0] + xy[2]) / 2, (xy[1] + xy[3]) / 2)
        mask_center = None
        if result.masks is not None and i < len(result.masks.xy):
            polygon = np.asarray(result.masks.xy[i], np.float32)
            if len(polygon) >= 3:
                moments = cv2.moments(polygon)
                if moments["m00"]:
                    mask_center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        found.append({"class": name, "confidence": float(box.conf.item()), "bbox": xy, "center": center, "mask_center": mask_center})
    return found, result.plot()


def score_image(truth, predictions, allowed):
    # Confidence-ordered one-to-one matching of same-class boxes at IoU >= 0.50.
    gt = [(i, x) for i, x in enumerate(truth) if x["class"] in allowed]
    pr = [(i, x) for i, x in enumerate(predictions) if x["class"] in allowed]
    used = set(); rows = []; counters = Counter(); errors = defaultdict(list)
    for pi, p in sorted(pr, key=lambda pair: -pair[1]["confidence"]):
        options = [(iou(p["bbox"], g["bbox"]), gi, g) for gi, g in gt if gi not in used and g["class"] == p["class"]]
        overlap, gi, g = max(options, default=(0, None, None), key=lambda x: x[0])
        if overlap >= .5:
            used.add(gi); counters[(p["class"], "tp")] += 1
            error = float(np.hypot(p["center"][0] - g["center"][0], p["center"][1] - g["center"][1]))
            errors[p["class"]].append(error)
            rows.append({"outcome": "tp", "class": p["class"], "confidence": p["confidence"], "iou": overlap, "center_error_px": error})
        else:
            counters[(p["class"], "fp")] += 1
            rows.append({"outcome": "fp", "class": p["class"], "confidence": p["confidence"], "iou": overlap, "center_error_px": None})
    for gi, g in gt:
        if gi not in used:
            counters[(g["class"], "fn")] += 1
            rows.append({"outcome": "fn", "class": g["class"], "confidence": None, "iou": None, "center_error_px": None})
    return rows, counters, errors


def summarize(counts, errors, allowed):
    result = {}
    for name in allowed:
        tp, fp, fn = (counts[(name, key)] for key in ("tp", "fp", "fn"))
        es = errors[name]
        result[name] = {"tp": tp, "fp": fp, "fn": fn,
                        "precision_at_threshold": tp / (tp + fp) if tp + fp else None,
                        "recall_at_threshold": tp / (tp + fn) if tp + fn else None,
                        "mean_center_error_px": float(np.mean(es)) if es else None,
                        "median_center_error_px": float(np.median(es)) if es else None,
                        "p95_center_error_px": float(np.percentile(es, 95)) if es else None}
    tp = sum(counts[(n, "tp")] for n in allowed)
    fp = sum(counts[(n, "fp")] for n in allowed)
    fn = sum(counts[(n, "fn")] for n in allowed)
    result["micro"] = {"tp": tp, "fp": fp, "fn": fn,
                       "precision_at_threshold": tp / (tp + fp) if tp + fp else None,
                       "recall_at_threshold": tp / (tp + fn) if tp + fn else None}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path(__file__).resolve().parent.parent / "dataset" / "real_camera_shape_test_v1")
    parser.add_argument("--colored", type=Path, required=True, help="Colored Block segmentation best.pt")
    parser.add_argument("--robotic", type=Path, required=True, help="Robotic Arm detection best.pt")
    parser.add_argument("--confidence", type=float, default=.25)
    args = parser.parse_args()
    if not 0 < args.confidence < 1: parser.error("--confidence must be between 0 and 1")
    images = sorted(p for p in (args.dataset / "images").rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images: parser.error(f"No images under {args.dataset / 'images'}")
    from ultralytics import YOLO
    output = args.dataset / "evaluation"
    output.mkdir(exist_ok=True)
    all_results = {}
    for tag, path in (("colored", args.colored), ("robotic", args.robotic)):
        if not path.exists(): raise FileNotFoundError(path)
        model = YOLO(str(path))
        # Colored Block has no pyramid. Its `triangular` output is not silently mapped to pyramid.
        allowed = COMMON if tag == "colored" else frozenset(CLASSES)
        counts = Counter(); errors = defaultdict(list); details = []; out_of_scope = 0
        gallery = output / tag; gallery.mkdir(exist_ok=True)
        for image in images:
            truth, frame = read_truth(image, args.dataset)
            predicted, plotted = predict(model, image, args.confidence)
            out_of_scope += sum(p["class"] not in allowed for p in predicted)
            rows, one_count, one_errors = score_image(truth, predicted, allowed)
            counts.update(one_count)
            for key, value in one_errors.items(): errors[key].extend(value)
            for row in rows: details.append({"image": image.relative_to(args.dataset).as_posix(), **row})
            if len(list(gallery.glob("*.jpg"))) < 24:
                cv2.imwrite(str(gallery / (image.parent.name + "_" + image.name)), plotted)
        with (output / f"{tag}_detections.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["image", "outcome", "class", "confidence", "iou", "center_error_px"])
            writer.writeheader(); writer.writerows(details)
        all_results[tag] = {"common_classes": summarize(counts, errors, COMMON),
                            "all_supported_classes": summarize(counts, errors, allowed),
                            "out_of_scope_predictions": out_of_scope}
    comparison = {tag: all_results[tag]["common_classes"] for tag in all_results}
    (output / "scores.json").write_text(json.dumps({"confidence_threshold": args.confidence, "iou_match_threshold": .5, "scores": all_results, "comparison_note": "Compare shared cube/cylinder/sphere classes on identical images. Colored Block has no pyramid. Precision and recall here are at one confidence threshold, not mAP."}, indent=2), encoding="utf-8")
    print(json.dumps(comparison, indent=2))
    print(f"Results: {output}")


if __name__ == "__main__": main()
