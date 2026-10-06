"""Compare the two trained digit checkpoints on a manually selected camera crop.

Camera only: this script never connects to or moves the robot.
Run from the workspace root: python digit_model_experiments/test_digit_camera.py --index 1
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image
import torch
from torch import nn
from torchvision import models, transforms


HERE = Path(__file__).resolve().parent
CHECKPOINTS = {
    "MobileNetV3-Small": (HERE / "checkpoints" / "mobilenetv3_small" / "best.pt", "MOBILENETV3_SMALL"),
    "ResNet18": (HERE / "checkpoints" / "resnet18" / "best.pt", "RESNET18"),
}
OUTPUT = HERE / "camera_tests"
WINDOW = "Digit camera comparison | drag box around ONE digit | R: rotate | 0-9: label | Q: quit"
CSV_FIELDS = ["timestamp", "true_digit", "crop_file", "frame_file", "mobile_prediction",
              "mobile_confidence", "mobile_ms", "resnet_prediction", "resnet_confidence", "resnet_ms",
              "rotation_ccw_degrees"]


def pad_square(image: Image.Image) -> Image.Image:
    """The same border-color square padding used in the training notebook."""
    image = image.convert("RGB")
    array = np.asarray(image)
    border = np.concatenate([array[0], array[-1], array[:, 0], array[:, -1]], axis=0)
    fill = tuple(int(v) for v in np.median(border, axis=0))
    width, height = image.size
    side = max(width, height)
    canvas = Image.new("RGB", (side, side), fill)
    canvas.paste(image, ((side - width) // 2, (side - height) // 2))
    return canvas


PREPROCESS = transforms.Compose([
    transforms.Lambda(pad_square),
    transforms.Resize((128, 128)),
    transforms.ToTensor(),
    transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
])


def load_models(device: torch.device, checkpoint_paths: dict[str, Path] | None = None) -> dict[str, nn.Module]:
    loaded = {}
    for display_name, (default_path, expected_name) in CHECKPOINTS.items():
        path = checkpoint_paths.get(display_name, default_path) if checkpoint_paths else default_path
        if not path.is_file():
            raise FileNotFoundError(f"Missing checkpoint: {path}")
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
        if (checkpoint.get("model_name") != expected_name or
                checkpoint.get("input_size") != 128 or
                checkpoint.get("smoke_test") is not False):
            raise ValueError(f"Checkpoint metadata does not match the full {expected_name} run: {path}")
        if expected_name == "MOBILENETV3_SMALL":
            model = models.mobilenet_v3_small(weights=None)
            model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, 10)
        else:
            model = models.resnet18(weights=None)
            model.fc = nn.Linear(model.fc.in_features, 10)
        model.load_state_dict(checkpoint["state_dict"], strict=True)
        loaded[display_name] = model.to(device).eval()
        print(f"Loaded {display_name} from {path}: epoch {checkpoint['epoch']}, "
              f"validation macro F1 {checkpoint['validation_macro_f1']:.4f}, "
              f"rotation experiment={checkpoint.get('rotation_experiment', False)}")
    return loaded


def predict(crop_bgr: np.ndarray, loaded: dict[str, nn.Module], device: torch.device):
    rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    tensor = PREPROCESS(Image.fromarray(rgb)).unsqueeze(0).to(device)
    results = {}
    with torch.inference_mode():
        for name, model in loaded.items():
            start = time.perf_counter()
            probabilities = model(tensor).softmax(dim=1)[0]
            if device.type == "cuda":
                torch.cuda.synchronize()
            elapsed_ms = (time.perf_counter() - start) * 1000
            confidence, digit = probabilities.max(dim=0)
            results[name] = (int(digit.item()), float(confidence.item()), elapsed_ms)
    return results


def centered_box(width: int, height: int, size: int) -> tuple[int, int, int, int]:
    side = min(size, width, height)
    x = (width - side) // 2
    y = (height - side) // 2
    return x, y, x + side, y + side


def clamp_box(box, width: int, height: int):
    x1, y1, x2, y2 = box
    x1, x2 = sorted((max(0, min(width, x1)), max(0, min(width, x2))))
    y1, y2 = sorted((max(0, min(height, y1)), max(0, min(height, y2))))
    return x1, y1, x2, y2


def save_sample(frame, crop, results, label: str, rotation_degrees: int,
                output_dir: Path = OUTPUT):
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    crop_path = output_dir / f"{stamp}_crop.png"
    frame_path = output_dir / f"{stamp}_frame.jpg"
    if not cv2.imwrite(str(crop_path), crop) or not cv2.imwrite(str(frame_path), frame):
        raise OSError("Could not save camera image")
    mobile = results["MobileNetV3-Small"]
    resnet = results["ResNet18"]
    row = [datetime.now().isoformat(timespec="seconds"), label, crop_path.name, frame_path.name,
           mobile[0], f"{mobile[1]:.6f}", f"{mobile[2]:.3f}",
           resnet[0], f"{resnet[1]:.6f}", f"{resnet[2]:.3f}", rotation_degrees]
    log = output_dir / "predictions_rotations.csv"
    new_file = not log.exists()
    with log.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        if new_file:
            writer.writerow(CSV_FIELDS)
        writer.writerow(row)
    print(f"Saved {crop_path.name}: label={label or 'unknown'}, "
          f"MobileNet={mobile[0]} ({mobile[1]:.1%}), ResNet={resnet[0]} ({resnet[1]:.1%})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=0, help="Camera index, usually 0 or 1")
    parser.add_argument("--roi-size", type=int, default=160, help="Initial centered crop side in pixels")
    parser.add_argument("--device", choices=("cpu", "cuda", "auto"), default="cpu")
    parser.add_argument("--mobile-checkpoint", type=Path, help="Replacement MobileNet checkpoint")
    parser.add_argument("--resnet-checkpoint", type=Path, help="Replacement ResNet18 checkpoint")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT,
                        help="Directory for saved crops, frames, and prediction CSV")
    args = parser.parse_args()
    if args.roi_size < 16:
        parser.error("--roi-size must be at least 16 pixels")
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA was requested but is unavailable")
    device = torch.device("cuda" if args.device == "cuda" or
                          (args.device == "auto" and torch.cuda.is_available()) else "cpu")
    replacements = {}
    if args.mobile_checkpoint:
        replacements["MobileNetV3-Small"] = args.mobile_checkpoint
    if args.resnet_checkpoint:
        replacements["ResNet18"] = args.resnet_checkpoint
    loaded = load_models(device, replacements)
    camera = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise RuntimeError(f"Camera {args.index} did not open. Try --index 1 or close other camera apps.")

    state = {"box": None, "drag_start": None, "drawing": False, "shape": None,
             "rotation_steps": 0}

    def on_mouse(event, x, y, _flags, _userdata):
        if state["shape"] is None:
            return
        height, width = state["shape"]
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drag_start"] = (x, y)
            state["drawing"] = True
        elif event == cv2.EVENT_MOUSEMOVE and state["drawing"]:
            sx, sy = state["drag_start"]
            state["box"] = clamp_box((sx, sy, x, y), width, height)
        elif event == cv2.EVENT_LBUTTONUP and state["drawing"]:
            sx, sy = state["drag_start"]
            candidate = clamp_box((sx, sy, x, y), width, height)
            if candidate[2] - candidate[0] >= 16 and candidate[3] - candidate[1] >= 16:
                state["box"] = candidate
            state["drawing"] = False

    print("Drag a box around ONE digit. R rotates the crop 90 degrees counterclockwise.")
    print("Press 0-9 to save a labeled sample, S for an unlabeled sample, Q to quit.")
    print("Predictions are for the selected crop only; this script does not detect digit locations automatically.")
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, on_mouse)
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("Camera stopped returning frames")
            height, width = frame.shape[:2]
            state["shape"] = (height, width)
            if state["box"] is None:
                state["box"] = centered_box(width, height, args.roi_size)
            x1, y1, x2, y2 = clamp_box(state["box"], width, height)
            if x2 - x1 < 16 or y2 - y1 < 16:
                state["box"] = centered_box(width, height, args.roi_size)
                x1, y1, x2, y2 = state["box"]
            crop = frame[y1:y2, x1:x2].copy()
            rotated_crop = np.rot90(crop, k=state["rotation_steps"]).copy()
            results = predict(rotated_crop, loaded, device)
            display = frame.copy()
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.rectangle(display, (0, 0), (min(width, 700), 96), (0, 0, 0), -1)
            for row, name in enumerate(CHECKPOINTS):
                digit, confidence, elapsed = results[name]
                cv2.putText(display, f"{name}: {digit}  {confidence:.1%}  {elapsed:.1f} ms",
                            (10, 30 + row * 31), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(display, f"R: rotate {state['rotation_steps'] * 90} CCW   0-9: label/save   S: save   Q: quit",
                        (10, 91), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("r"), ord("R")):
                state["rotation_steps"] = (state["rotation_steps"] + 1) % 4
                continue
            if ord("0") <= key <= ord("9"):
                save_sample(frame, crop, results, chr(key), state["rotation_steps"] * 90,
                            args.output_dir)
            elif key in (ord("s"), ord("S")):
                save_sample(frame, crop, results, "", state["rotation_steps"] * 90,
                            args.output_dir)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError, OSError, cv2.error) as exc:
        raise SystemExit(f"Digit camera test error: {exc}") from exc
