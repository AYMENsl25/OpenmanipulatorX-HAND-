"""Cheap local structural and synthetic checks; never trains on project data."""
import ast
import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from sklearn.metrics import accuracy_score, precision_recall_fscore_support
from torch import nn
from torch.utils.data import DataLoader
from torchvision import models, transforms
from torchvision.models import MobileNet_V3_Small_Weights, ResNet18_Weights

path = Path(__file__).with_name("digit_classifier_comparison.ipynb")
notebook = json.loads(path.read_text(encoding="utf-8"))
sources = [cell["source"] for cell in notebook["cells"] if cell["cell_type"] == "code"]
for source in sources:
    compile(source, str(path), "exec")

wanted = {"pad_square", "CustomDigitCNN", "create_model", "freeze_backbone",
          "classification_metrics", "run_epoch"}
nodes = []
for source in sources:
    for node in ast.parse(source).body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in wanted:
            nodes.append(node)
namespace = {"np": np, "torch": torch, "Image": Image, "nn": nn, "models": models,
             "MobileNet_V3_Small_Weights": MobileNet_V3_Small_Weights,
             "ResNet18_Weights": ResNet18_Weights,
             "accuracy_score": accuracy_score,
             "precision_recall_fscore_support": precision_recall_fscore_support,
             "device": torch.device("cpu")}
module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
exec(compile(module, str(path), "exec"), namespace)

image = Image.new("RGB", (12, 20), "white")
assert namespace["pad_square"](image).size == (20, 20)
cnn = namespace["CustomDigitCNN"]()
assert sum(p.numel() for p in cnn.parameters()) < 1_000_000
assert cnn(torch.zeros(2, 3, 128, 128)).shape == (2, 10)

for name in ("MOBILENETV3_SMALL", "RESNET18"):
    model, pretrained = namespace["create_model"](name, load_imagenet_weights=False)
    assert pretrained and model(torch.zeros(1, 3, 128, 128)).shape == (1, 10)
    namespace["freeze_backbone"](model, name, True)
    assert any(p.requires_grad for p in model.parameters())
    assert any(not p.requires_grad for p in model.parameters())

batch = [(torch.rand(3, 32, 32), label, f"sample{label}.png", "SVHN") for label in (1, 2)]
loader = DataLoader(batch, batch_size=2)
optimizer = torch.optim.AdamW(cnn.parameters(), lr=1e-3)
scaler = torch.amp.GradScaler("cuda", enabled=False)
result = namespace["run_epoch"](cnn, loader, nn.CrossEntropyLoss(), optimizer, scaler)
assert result["n"] == 2 and np.isfinite(result["loss"])
print("Notebook syntax and synthetic model/training checks passed; no project data trained.")
