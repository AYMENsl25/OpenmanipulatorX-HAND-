"""Fine-tune the downloaded classifiers on human-labeled cube-camera crops.

Fill true_digit, scene_id, and split (train/val/test) in
camera_tests/yolo_digit/cube_crops.csv before running. Test rows are never used here.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import random

from PIL import Image
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms

from test_digit_camera import HERE, load_models, pad_square


DEFAULT_CSV = HERE / "camera_tests" / "yolo_digit" / "cube_crops.csv"
MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)


class CameraCrops(Dataset):
    def __init__(self, rows, root: Path, train: bool, rotation_degrees: float):
        self.rows = rows
        self.root = root
        steps = [transforms.Lambda(pad_square), transforms.Resize((128, 128))]
        if train:
            if rotation_degrees:
                steps.append(transforms.RandomRotation(rotation_degrees, fill=127))
            steps.extend([
                transforms.RandomPerspective(distortion_scale=0.12, p=0.2, fill=127),
                transforms.ColorJitter(brightness=0.15, contrast=0.15),
            ])
        steps.extend([transforms.ToTensor(), transforms.Normalize(MEAN, STD)])
        self.transform = transforms.Compose(steps)

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        with Image.open(self.root / row["crop_file"]) as image:
            tensor = self.transform(image.convert("RGB"))
        return tensor, int(row["true_digit"])


def read_rows(csv_path: Path):
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"crop_file", "true_digit", "scene_id", "split"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"CSV needs columns {sorted(required)} and at least one row")
    selected = []
    for row in rows:
        label = row["true_digit"].strip()
        split = row["split"].strip().lower()
        scene = row["scene_id"].strip()
        if not label and not split and not scene:
            continue  # unlabeled capture, not yet prepared for training
        if len(label) != 1 or label not in "0123456789" or split not in {"train", "val", "test"} or not scene:
            raise ValueError(f"Complete true_digit, scene_id, split for {row['crop_file']}")
        if not (csv_path.parent / row["crop_file"]).is_file():
            raise FileNotFoundError(csv_path.parent / row["crop_file"])
        row["split"] = split
        row["scene_id"] = scene
        selected.append(row)
    scenes = {}
    for row in selected:
        scenes.setdefault(row["scene_id"], set()).add(row["split"])
    leaking = [scene for scene, splits in scenes.items() if len(splits) > 1]
    if leaking:
        raise ValueError(f"Scene IDs cross train/val/test splits: {leaking[:10]}")
    return selected


def macro_f1(truth, predicted, labels):
    scores = []
    for digit in labels:
        tp = sum(a == digit and b == digit for a, b in zip(truth, predicted))
        fp = sum(a != digit and b == digit for a, b in zip(truth, predicted))
        fn = sum(a == digit and b != digit for a, b in zip(truth, predicted))
        denominator = 2 * tp + fp + fn
        scores.append(2 * tp / denominator if denominator else 0.0)
    return sum(scores) / len(scores)


def train_one(name, model, train_loader, val_loader, labels, device, epochs, lr, output):
    model = model.to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss()
    best = -1.0
    output.mkdir(parents=True, exist_ok=True)
    history = []
    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        for images, targets in train_loader:
            images, targets = images.to(device), targets.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(images), targets)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(targets)
        model.eval()
        truth, predictions = [], []
        with torch.inference_mode():
            for images, targets in val_loader:
                predicted = model(images.to(device)).argmax(dim=1).cpu().tolist()
                truth.extend(targets.tolist())
                predictions.extend(predicted)
        score = macro_f1(truth, predictions, labels)
        accuracy = sum(a == b for a, b in zip(truth, predictions)) / len(truth)
        row = {"epoch": epoch, "train_loss": train_loss / len(train_loader.dataset),
               "val_accuracy": accuracy, "val_macro_f1": score}
        history.append(row)
        print(f"{name} epoch {epoch}: train loss {row['train_loss']:.4f}, "
              f"val accuracy {accuracy:.4f}, macro F1 {score:.4f}")
        if score > best:
            best = score
            torch.save({"state_dict": model.state_dict(), "model_name": name,
                        "input_size": 128, "smoke_test": False, "epoch": epoch,
                        "validation_macro_f1": score, "domain": "real_cube_camera"},
                       output / "best.pt")
    (output / "history.json").write_text(json.dumps(history, indent=2), encoding="utf-8")
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--rotation-degrees", type=float, default=180,
                        help="Train-only random rotation from -N to +N degrees")
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    args = parser.parse_args()
    if not 0 <= args.rotation_degrees <= 180 or args.epochs <= 0 or args.batch_size <= 0 or args.lr <= 0:
        parser.error("Rotation must be 0-180; epochs, batch size, and learning rate must be positive")
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA requested but unavailable")
    random.seed(42)
    torch.manual_seed(42)
    device = torch.device("cuda" if args.device == "cuda" or
                          (args.device == "auto" and torch.cuda.is_available()) else "cpu")
    rows = read_rows(args.csv)
    train = [row for row in rows if row["split"] == "train"]
    val = [row for row in rows if row["split"] == "val"]
    if not train or not val:
        parser.error("Label independent train and val scenes before fine-tuning")
    labels = sorted({int(row["true_digit"]) for row in train + val})
    if args.rotation_degrees > 45 and ({6, 9} & set(labels)):
        parser.error("Full rotation makes 6/9 ambiguous. Use an orientation marker or exclude those digits.")
    print(f"Device {device}; train {len(train)}, val {len(val)}, untouched test "
          f"{sum(row['split'] == 'test' for row in rows)}; labels {labels}")
    train_loader = DataLoader(CameraCrops(train, args.csv.parent, True, args.rotation_degrees),
                              batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(CameraCrops(val, args.csv.parent, False, 0),
                            batch_size=args.batch_size, shuffle=False)
    root = HERE / "camera_finetuning" / f"rotation_{args.rotation_degrees:g}"
    if root.exists():
        parser.error(f"Output already exists; preserve or move it before another run: {root}")
    models = load_models(device)
    names = {"MobileNetV3-Small": "MOBILENETV3_SMALL", "ResNet18": "RESNET18"}
    for display_name, model in models.items():
        best = train_one(names[display_name], model, train_loader, val_loader, labels,
                         device, args.epochs, args.lr, root / names[display_name].lower())
        print(f"Best {display_name} validation macro F1: {best:.4f}")
    (root / "run_config.json").write_text(json.dumps({
        "source_csv": str(args.csv), "rotation_degrees": args.rotation_degrees,
        "epochs": args.epochs, "batch_size": args.batch_size, "learning_rate": args.lr,
        "train_rows": len(train), "val_rows": len(val), "labels": labels,
    }, indent=2), encoding="utf-8")
    print(f"Fine-tuned checkpoints: {root}")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, ValueError) as exc:
        raise SystemExit(f"Cube digit fine-tuning error: {exc}") from exc
