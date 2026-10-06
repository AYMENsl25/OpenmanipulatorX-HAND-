"""Build the independent Kaggle STN ablation notebook; never edits earlier notebooks."""
from __future__ import annotations

import json
import textwrap
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
cells = []


def md(value):
    cells.append({"cell_type": "markdown", "metadata": {},
                  "source": textwrap.dedent(value).strip() + "\n"})


def code(value):
    cells.append({"cell_type": "code", "execution_count": None, "metadata": {},
                  "outputs": [], "source": textwrap.dedent(value).strip() + "\n"})


md("""
# Spatial Transformer Network digit experiment

**Question:** Does a classic affine Spatial Transformer Network (STN) improve recognition of randomly oriented cube digits? This is an ablation of MobileNetV3-Small and ResNet18, each with and without an STN. It does not use a Vision Transformer, retrain YOLO, or control the robot.

**Run:** Attach the existing complete `DIGIT_CLASSIFICATION_V1` dataset or its ZIP in Kaggle, enable a GPU and Internet for official ImageNet weights, then run top to bottom. The original train/val/test splits are preserved. Set `SMOKE_TEST=True` for a short pipeline check; restore `False` and restart before the full experiment. Four full runs may exceed one Kaggle session; `MODELS_TO_RUN` and `EXTERNAL_CHECKPOINT_ROOTS` support separate sessions. Final test evaluation runs only when all four full checkpoints are available.

**Interpretation:** The supplied public crops are not a substitute for labeled robot-camera cube faces. Full rotation of 6 and 9 is deliberately included for measurement, but a half-turn can make them visually ambiguous. An orientation dot is disabled until its physical design is specified. STN output is optimized for classification and need not look upright.

STN method: [original paper](https://arxiv.org/abs/1506.02025), [PyTorch tutorial](https://docs.pytorch.org/tutorials/intermediate/spatial_transformer_tutorial.html).
""")

md("""## 1. Setup and experiment configuration

The same seed, input size, training augmentation, optimizer family, patience, and evaluation rules apply to all four variants. Test images are never used for checkpoint selection.
""")

code("""
import json, math, os, random, shutil, time, warnings, zipfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image, ImageOps
from IPython.display import display
from sklearn.metrics import (accuracy_score, precision_recall_fscore_support,
                             confusion_matrix)
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision import models, transforms
from torchvision.models import MobileNet_V3_Small_Weights, ResNet18_Weights

SEED = 42
INPUT_SIZE = 128
BATCH_SIZE = 128
NUM_WORKERS = min(4, os.cpu_count() or 1)
SMOKE_TEST = False
MODELS_TO_RUN = ['mobilenet_baseline', 'mobilenet_stn', 'resnet18_baseline', 'resnet18_stn']
RUN_FINAL_EVALUATION = True
EXTERNAL_CHECKPOINT_ROOTS = []  # Prior attached Kaggle output roots containing model folders
DATASET_ROOT_OVERRIDE = None
DATASET_ZIP_OVERRIDE = None
MAX_EPOCHS = 2 if SMOKE_TEST else 20
HEAD_EPOCHS = 1 if SMOKE_TEST else 3
EARLY_STOPPING_PATIENCE = 5
HEAD_LR = 8e-4
STN_LR = 3e-4
BACKBONE_LR = 1e-4
WEIGHT_DECAY = 1e-4
USE_CLASS_WEIGHTS = True
THETA_MONITOR_BATCHES = 4
ROTATION_TEST_PER_CLASS = 20
ROTATION_TEST_ANGLES = (0, 30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 270, 315)
USE_HORIZONTAL_FLIP = False
USE_VERTICAL_FLIP = False
USE_SYNTHETIC_ORIENTATION_DOT = False
if SMOKE_TEST:
    RUN_FINAL_EVALUATION = False
OUTPUT_ROOT = Path('/kaggle/working/stn_digit_experiment') if Path('/kaggle/working').exists() else Path('stn_digit_experiment')
OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

if USE_SYNTHETIC_ORIENTATION_DOT:
    raise ValueError('Specify the physical dot size and placement before enabling synthetic dots.')
random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(SEED)
torch.backends.cudnn.benchmark = True
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print('PyTorch', torch.__version__, 'torchvision', __import__('torchvision').__version__, 'device', device)
if device.type != 'cuda':
    warnings.warn('A GPU is recommended for four full runs.')
assert 0 < HEAD_EPOCHS < MAX_EPOCHS
""")

md("""## 2. Locate and audit the existing dataset

No crops are rebuilt and no split is changed. A ZIP can be extracted under this experiment's output folder.
""")

code("""
def require_dataset(root):
    root = Path(root)
    required = ['dataset_manifest.json', 'metadata.csv']
    required += [f'{split}/{digit}' for split in ('train', 'val', 'test') for digit in range(10)]
    missing = [name for name in required if not (root / name).exists()]
    if missing:
        raise FileNotFoundError(f'Incomplete DIGIT_CLASSIFICATION_V1 at {root}: {missing[:6]}')
    return root


def extract_dataset(archive_path):
    destination = OUTPUT_ROOT / 'dataset_unpacked'
    destination.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive_path) as archive:
        names = archive.namelist()
        root_level = 'dataset_manifest.json' in names and 'metadata.csv' in names
        wrapped = any(name.endswith('DIGIT_CLASSIFICATION_V1/dataset_manifest.json') for name in names)
        if not (root_level or wrapped):
            raise ValueError(f'{archive_path} does not contain the dataset manifest and metadata')
        unpack_root = destination / 'DIGIT_CLASSIFICATION_V1' if root_level else destination
        unpack_root.mkdir(parents=True, exist_ok=True)
        base = unpack_root.resolve()
        for member in archive.infolist():
            target = (unpack_root / member.filename).resolve()
            if target != base and base not in target.parents:
                raise ValueError('Unsafe ZIP entry: ' + member.filename)
        archive.extractall(unpack_root)
    matches = list(destination.rglob('dataset_manifest.json'))
    matches = [path for path in matches if path.parent.name == 'DIGIT_CLASSIFICATION_V1']
    if not matches:
        raise FileNotFoundError('Dataset manifest absent after ZIP extraction')
    return require_dataset(matches[0].parent)


def find_dataset():
    if DATASET_ROOT_OVERRIDE is not None:
        return require_dataset(DATASET_ROOT_OVERRIDE)
    if DATASET_ZIP_OVERRIDE is not None:
        return extract_dataset(DATASET_ZIP_OVERRIDE)
    for base in (Path('/kaggle/input'), Path.cwd(), Path.cwd().parent):
        if not base.exists():
            continue
        for manifest_path in sorted(base.rglob('dataset_manifest.json')):
            if manifest_path.parent.name != 'DIGIT_CLASSIFICATION_V1':
                continue
            try:
                return require_dataset(manifest_path.parent)
            except FileNotFoundError:
                pass
    for base in (Path('/kaggle/input'), Path.cwd(), Path.cwd().parent):
        if base.exists():
            for archive_path in sorted(base.rglob('*.zip')):
                try:
                    return extract_dataset(archive_path)
                except ValueError:
                    pass
    raise FileNotFoundError('Attach complete DIGIT_CLASSIFICATION_V1 or set DATASET_ROOT_OVERRIDE / DATASET_ZIP_OVERRIDE')


DATA_ROOT = find_dataset()
manifest = json.loads((DATA_ROOT / 'dataset_manifest.json').read_text(encoding='utf-8'))
metadata = pd.read_csv(DATA_ROOT / 'metadata.csv', dtype={'crop_file': str, 'source_image': str})
metadata['crop_file'] = metadata.crop_file.str.replace('\\\\', '/', regex=False)
metadata['digit'] = metadata.digit.astype(int)
assert set(metadata.unified_split) == {'train', 'val', 'test'}
assert set(metadata.digit) == set(range(10))
assert not metadata.crop_file.duplicated().any()
assert metadata.crop_file.str.split('/').str[0].eq(metadata.unified_split).all()
assert metadata.crop_file.str.split('/').str[1].astype(int).eq(metadata.digit).all()
assert len(metadata) == manifest['total_crops']
assert metadata.unified_split.value_counts().to_dict() == manifest['split_counts']
group_splits = metadata.groupby(['source_dataset', 'source_split', 'source_image']).unified_split.nunique()
assert group_splits.max() == 1, 'A source image crosses train/validation/test'
for rel in metadata.sample(n=min(100, len(metadata)), random_state=SEED).crop_file:
    with Image.open(DATA_ROOT / rel) as image:
        image.verify()
print('Dataset:', DATA_ROOT)
display(metadata.groupby(['unified_split', 'source_dataset']).size().unstack(fill_value=0))
train_frame = metadata.loc[metadata.unified_split == 'train'].reset_index(drop=True)
val_frame = metadata.loc[metadata.unified_split == 'val'].reset_index(drop=True)
test_frame = metadata.loc[metadata.unified_split == 'test'].reset_index(drop=True)
if SMOKE_TEST:
    train_frame = pd.concat([part.sample(frac=0.05, random_state=SEED) for _, part in train_frame.groupby('digit')]).reset_index(drop=True)
    print('Smoke test uses 5% of train; validation unchanged; test withheld.')
""")

md("""## 3. Shared augmentation and a ten-class preview

The same full-circle training augmentation is used for all four models, including 6 and 9. Rotations use an expanded canvas before resizing to avoid cutting off diagonal digits. No mirror flips are used. Validation and ordinary test images are unaugmented. The fixed-angle test is separate and runs only after training.
""")

code("""
def border_color(image):
    image = image.convert('RGB')
    array = np.asarray(image)
    border = np.concatenate((array[0], array[-1], array[:, 0], array[:, -1]), axis=0)
    return tuple(int(v) for v in np.median(border, axis=0))


def pad_square(image):
    image = image.convert('RGB')
    width, height = image.size
    side = max(width, height)
    canvas = Image.new('RGB', (side, side), border_color(image))
    canvas.paste(image, ((side - width) // 2, (side - height) // 2))
    return canvas


def expanded_rotation(image, angle):
    square = pad_square(image)
    return square.rotate(angle, resample=Image.Resampling.BILINEAR, expand=True,
                         fillcolor=border_color(square))


photo_augment = transforms.Compose([
    transforms.RandomPerspective(distortion_scale=0.30, p=0.50, fill=127),
    transforms.RandomAffine(degrees=0, translate=(0.10, 0.10),
                            scale=(0.80, 1.20), shear=(-5, 5), fill=127),
    transforms.ColorJitter(brightness=0.25, contrast=0.30, saturation=0.15, hue=0.02),
    transforms.RandomApply([transforms.GaussianBlur(3, sigma=(0.1, 1.0))], p=0.15),
    transforms.RandomGrayscale(p=0.10),
    transforms.RandomAutocontrast(p=0.20),
])


def image_to_tensor(image, training=False, fixed_angle=None):
    if fixed_angle is not None:
        image = expanded_rotation(image, fixed_angle)
    elif training:
        image = expanded_rotation(image, random.uniform(-180, 180))
    image = pad_square(image).resize((INPUT_SIZE, INPUT_SIZE), Image.Resampling.BILINEAR)
    if training:
        image = photo_augment(image)
        if USE_HORIZONTAL_FLIP and random.random() < 0.5:
            image = ImageOps.mirror(image)
        if USE_VERTICAL_FLIP and random.random() < 0.5:
            image = ImageOps.flip(image)
    return transforms.functional.to_tensor(image)


class DigitDataset(Dataset):
    def __init__(self, frame, training=False, fixed_angles=None):
        self.records = frame[['crop_file', 'digit', 'source_dataset']].to_records(index=False)
        self.training = training
        self.fixed_angles = fixed_angles

    def __len__(self):
        return len(self.records) * (len(self.fixed_angles) if self.fixed_angles else 1)

    def __getitem__(self, index):
        if self.fixed_angles:
            record_index, angle_index = divmod(index, len(self.fixed_angles))
            angle = self.fixed_angles[angle_index]
        else:
            record_index, angle = index, None
        rel, digit, source = self.records[record_index]
        with Image.open(DATA_ROOT / rel) as image:
            tensor = image_to_tensor(image.convert('RGB'), self.training, angle)
        return tensor, int(digit), str(rel), str(source), -1 if angle is None else int(angle)


def make_loader(frame, training=False, fixed_angles=None, batch_size=BATCH_SIZE):
    return DataLoader(DigitDataset(frame, training, fixed_angles), batch_size=batch_size,
                      shuffle=training, num_workers=NUM_WORKERS,
                      generator=torch.Generator().manual_seed(SEED) if training else None,
                      pin_memory=(device.type == 'cuda'), persistent_workers=(NUM_WORKERS > 0))


fig, axes = plt.subplots(10, 7, figsize=(12, 17))
for digit in range(10):
    row = train_frame.loc[train_frame.digit == digit].sample(n=1, random_state=SEED).iloc[0]
    with Image.open(DATA_ROOT / row.crop_file) as image:
        original = image.convert('RGB')
        axes[digit, 0].imshow(image_to_tensor(original).permute(1, 2, 0)); axes[digit, 0].set_title(f'Original {digit}')
        for column in range(1, 7):
            axes[digit, column].imshow(image_to_tensor(original, training=True).permute(1, 2, 0))
            axes[digit, column].set_title(f'Aug {column}')
    for axis in axes[digit]:
        axis.axis('off')
fig.suptitle('Ten classes: original and six random training augmentations')
fig.tight_layout(); fig.savefig(OUTPUT_ROOT / 'augmentation_preview.png', dpi=150); plt.show()
""")

md("""## 4. Four model variants

The localization CNN regresses six affine parameters and starts at identity. It samples unnormalized `[0,1]` input, then the wrapper applies ImageNet normalization before the pretrained classifier. Baselines use the same normalization. `align_corners=False` is set explicitly in both grid operations and border padding limits artificial empty regions.
""")

code("""
class SpatialTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        self.localization = nn.Sequential(
            nn.Conv2d(3, 16, 5, padding=2), nn.BatchNorm2d(16), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(64, 64), nn.ReLU(inplace=True),
            nn.Linear(64, 6))
        final = self.localization[-1]
        nn.init.zeros_(final.weight)
        final.bias.data.copy_(torch.tensor([1., 0., 0., 0., 1., 0.]))

    def forward(self, x):
        # Float32 grid construction avoids precision problems under mixed precision.
        with torch.autocast(device_type=x.device.type, enabled=False):
            x = x.float()
            theta = self.localization(x).view(-1, 2, 3)
            grid = F.affine_grid(theta, x.size(), align_corners=False)
            transformed = F.grid_sample(x, grid, mode='bilinear',
                                        padding_mode='border', align_corners=False)
        return transformed, theta


class DigitClassifier(nn.Module):
    def __init__(self, backbone, use_stn=False):
        super().__init__()
        self.stn = SpatialTransformer() if use_stn else None
        self.backbone = backbone
        self.register_buffer('image_mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer('image_std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))
        self._head_only = False

    def transform_input(self, x):
        if self.stn is None:
            theta = x.new_tensor([[1., 0., 0.], [0., 1., 0.]]).expand(x.size(0), -1, -1)
            return x, theta
        return self.stn(x)

    def forward(self, x):
        transformed, _ = self.transform_input(x)
        normalized = (transformed - self.image_mean) / self.image_std
        return self.backbone(normalized)


MODEL_SPECS = {
    'mobilenet_baseline': ('mobilenet', False),
    'mobilenet_stn': ('mobilenet', True),
    'resnet18_baseline': ('resnet18', False),
    'resnet18_stn': ('resnet18', True),
}
assert set(MODELS_TO_RUN) <= set(MODEL_SPECS)


def create_model(name, load_pretrained=True):
    family, use_stn = MODEL_SPECS[name]
    if family == 'mobilenet':
        weights = MobileNet_V3_Small_Weights.DEFAULT if load_pretrained else None
        backbone = models.mobilenet_v3_small(weights=weights)
        backbone.classifier[-1] = nn.Linear(backbone.classifier[-1].in_features, 10)
        head = backbone.classifier
    else:
        weights = ResNet18_Weights.DEFAULT if load_pretrained else None
        backbone = models.resnet18(weights=weights)
        backbone.fc = nn.Linear(backbone.fc.in_features, 10)
        head = backbone.fc
    return DigitClassifier(backbone, use_stn), head


def set_head_stage(model, name, head_only):
    model._head_only = head_only
    for parameter in model.backbone.parameters():
        parameter.requires_grad = not head_only
    family, _ = MODEL_SPECS[name]
    head = model.backbone.classifier if family == 'mobilenet' else model.backbone.fc
    for parameter in head.parameters():
        parameter.requires_grad = True
    return head


probe, _ = create_model('mobilenet_stn', load_pretrained=False)
with torch.no_grad():
    transformed, initial_theta = probe.transform_input(torch.rand(2, 3, INPUT_SIZE, INPUT_SIZE))
assert transformed.shape == (2, 3, INPUT_SIZE, INPUT_SIZE)
assert torch.allclose(initial_theta[0], torch.tensor([[1., 0., 0.], [0., 1., 0.]]))
assert probe(torch.rand(2, 3, INPUT_SIZE, INPUT_SIZE)).shape == (2, 10)
del probe
print('STN identity initialization and output shape verified.')
""")

md("""## 5. Train and select by validation macro F1

All four variants use official ImageNet weights. Stage 1 freezes the pretrained backbone and trains the head plus STN, where present. Stage 2 fine-tunes the backbone at a lower learning rate. The best checkpoint is selected using validation macro F1 only. The held-out test split is not read in this section.
""")

code("""
full_train_counts = metadata.loc[metadata.unified_split == 'train', 'digit'].value_counts().reindex(range(10), fill_value=0)
class_weight_values = 1 / np.sqrt(full_train_counts.to_numpy(dtype=np.float64))
class_weight_values /= class_weight_values.mean()
class_weights = torch.tensor(class_weight_values, dtype=torch.float32, device=device) if USE_CLASS_WEIGHTS else None


def metrics(true, predicted):
    precision, recall, macro_f1, _ = precision_recall_fscore_support(
        true, predicted, labels=list(range(10)), average='macro', zero_division=0)
    _, _, weighted_f1, _ = precision_recall_fscore_support(
        true, predicted, labels=list(range(10)), average='weighted', zero_division=0)
    return {'n': len(true), 'accuracy': float(accuracy_score(true, predicted)),
            'macro_precision': float(precision), 'macro_recall': float(recall),
            'macro_f1': float(macro_f1), 'weighted_f1': float(weighted_f1)}


def run_epoch(model, loader, criterion, optimizer=None, scaler=None):
    training = optimizer is not None
    model.train(training)
    if training and model._head_only:
        # Frozen pretrained BatchNorm statistics must not update in the head stage.
        model.backbone.eval()
    true, predicted = [], []
    total_loss = 0.0
    context = torch.enable_grad() if training else torch.inference_mode()
    with context:
        for images, labels, _, _, _ in loader:
            images = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            if training:
                optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type, enabled=(device.type == 'cuda')):
                logits = model(images)
                loss = criterion(logits, labels)
            if training:
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
            total_loss += float(loss.detach()) * len(labels)
            true.extend(labels.cpu().tolist())
            predicted.extend(logits.argmax(1).cpu().tolist())
    return {'loss': total_loss / len(true), **metrics(true, predicted)}


def theta_diagnostics(model, loader, name, epoch):
    if model.stn is None:
        return None
    model.eval()
    batches = []
    outside = []
    with torch.inference_mode():
        for batch_index, (images, _, _, _, _) in enumerate(loader):
            if batch_index >= THETA_MONITOR_BATCHES:
                break
            images = images.to(device, non_blocking=True)
            _, theta = model.transform_input(images)
            grid = F.affine_grid(theta, images.size(), align_corners=False)
            outside.append(float((grid.abs() > 1).any(dim=-1).float().mean()))
            batches.append(theta.detach().cpu().reshape(-1, 6))
    values = torch.cat(batches).numpy()
    determinants = values[:, 0] * values[:, 4] - values[:, 1] * values[:, 3]
    row = {'Model': name, 'Epoch': epoch, 'n': len(values),
           'determinant_min': float(determinants.min()),
           'determinant_mean': float(determinants.mean()),
           'outside_grid_fraction': float(np.mean(outside))}
    for index, label in enumerate(('a', 'b', 'tx', 'c', 'd', 'ty')):
        column = values[:, index]
        row.update({f'{label}_mean': float(column.mean()), f'{label}_std': float(column.std()),
                    f'{label}_min': float(column.min()), f'{label}_max': float(column.max())})
    row['collapse_warning'] = bool((determinants <= 0.05).any() or
                                   row['outside_grid_fraction'] > 0.50 or
                                   (epoch > HEAD_EPOCHS and values.std(axis=0).max() < 1e-4))
    if row['collapse_warning']:
        warnings.warn(f'{name} epoch {epoch}: possible STN collapse; inspect theta statistics and images.')
    return row


theta_rows = []


def train_one(name):
    # Reset initialization and data-worker seeds for every ablation arm.
    random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(SEED)
    model, _ = create_model(name, load_pretrained=True)
    model = model.to(device)
    train_loader = make_loader(train_frame, training=True)
    val_loader = make_loader(val_frame)
    criterion = nn.CrossEntropyLoss(weight=class_weights)
    scaler = torch.amp.GradScaler('cuda', enabled=(device.type == 'cuda'))
    output_dir = OUTPUT_ROOT / name
    (output_dir / 'metrics').mkdir(parents=True, exist_ok=True)
    history = []
    best_f1, best_epoch, epoch = -1., -1, 0
    for stage, stage_epochs in (('head', HEAD_EPOCHS), ('full', MAX_EPOCHS - HEAD_EPOCHS)):
        head = set_head_stage(model, name, stage == 'head')
        head_ids = {id(parameter) for parameter in head.parameters()}
        groups = [{'params': list(head.parameters()), 'lr': HEAD_LR}]
        if model.stn is not None:
            groups.append({'params': list(model.stn.parameters()), 'lr': STN_LR})
        if stage == 'full':
            groups.append({'params': [p for p in model.backbone.parameters() if id(p) not in head_ids],
                           'lr': BACKBONE_LR})
        optimizer = torch.optim.AdamW(groups, weight_decay=WEIGHT_DECAY)
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='max', factor=0.5, patience=2)
        patience = 0
        for _ in range(stage_epochs):
            epoch += 1
            train_result = run_epoch(model, train_loader, criterion, optimizer, scaler)
            val_result = run_epoch(model, val_loader, criterion)
            scheduler.step(val_result['macro_f1'])
            theta_row = theta_diagnostics(model, val_loader, name, epoch)
            if theta_row is not None:
                theta_rows.append(theta_row)
                pd.DataFrame(theta_rows).to_csv(OUTPUT_ROOT / 'stn_theta_statistics.csv', index=False)
            history.append({'epoch': epoch, 'stage': stage,
                            **{'train_' + key: value for key, value in train_result.items()},
                            **{'val_' + key: value for key, value in val_result.items()}})
            payload = {'model_name': name, 'state_dict': model.state_dict(), 'epoch': epoch,
                       'validation_macro_f1': val_result['macro_f1'], 'input_size': INPUT_SIZE,
                       'smoke_test': SMOKE_TEST, 'seed': SEED, 'full_rotation': True,
                       'synthetic_orientation_dot': USE_SYNTHETIC_ORIENTATION_DOT}
            torch.save(payload, output_dir / 'last.pt')
            if val_result['macro_f1'] > best_f1:
                best_f1, best_epoch, patience = val_result['macro_f1'], epoch, 0
                torch.save(payload, output_dir / 'best.pt')
            else:
                patience += 1
            pd.DataFrame(history).to_csv(output_dir / 'metrics' / 'history.csv', index=False)
            print(name, stage, epoch, 'train loss', round(train_result['loss'], 4),
                  'val macro F1', round(val_result['macro_f1'], 4), flush=True)
            if stage == 'full' and patience >= EARLY_STOPPING_PATIENCE:
                print('Early stopping:', name)
                break
    del model, train_loader, val_loader
    if device.type == 'cuda':
        torch.cuda.empty_cache()
    return {'model_name': name, 'output_dir': output_dir, 'best_epoch': best_epoch,
            'validation_macro_f1': best_f1}


trained = {name: train_one(name) for name in MODELS_TO_RUN}
print('Selected training runs complete. Test split remains unopened.')
""")

md("""## 6. Load four full checkpoints before opening test

For separate Kaggle sessions, attach earlier outputs and list their `stn_digit_experiment` roots in `EXTERNAL_CHECKPOINT_ROOTS`. Smoke checkpoints are rejected. Test evaluation begins only after all four checkpoints pass these checks.
""")

code("""
ALL_MODELS = list(MODEL_SPECS)


def checkpoint_for(name):
    local = OUTPUT_ROOT / name / 'best.pt'
    candidates = [local] + [Path(root) / name / 'best.pt' for root in EXTERNAL_CHECKPOINT_ROOTS]
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        raise FileNotFoundError(f'Missing {name}/best.pt. Finish this run or attach a prior output root.')
    payload = torch.load(path, map_location='cpu', weights_only=False)
    assert payload['model_name'] == name and payload['input_size'] == INPUT_SIZE
    assert not payload['smoke_test'], f'{name} is only a smoke checkpoint'
    assert payload.get('full_rotation') is True
    assert payload.get('synthetic_orientation_dot') is False
    if path != local:
        local.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, local)
        path = local
    return path, payload


def load_trained(name):
    path, payload = checkpoint_for(name)
    model, _ = create_model(name, load_pretrained=False)
    model.load_state_dict(payload['state_dict'])
    return model.to(device).eval(), path, payload


if RUN_FINAL_EVALUATION:
    assert not SMOKE_TEST, 'Full evaluation requires full-data checkpoints.'
    ready = {name: checkpoint_for(name)[0] for name in ALL_MODELS}
    theta_files = [OUTPUT_ROOT / 'stn_theta_statistics.csv']
    theta_files += [Path(root) / 'stn_theta_statistics.csv' for root in EXTERNAL_CHECKPOINT_ROOTS]
    available_theta = [pd.read_csv(path) for path in theta_files if path.is_file()]
    if available_theta:
        all_theta = pd.concat(available_theta, ignore_index=True)
        all_theta.drop_duplicates(['Model', 'Epoch'], keep='first', inplace=True)
        all_theta.to_csv(OUTPUT_ROOT / 'stn_theta_statistics.csv', index=False)
    else:
        warnings.warn('No theta statistics found. Review STN stability from the training session before deployment.')
    print('All four full checkpoints ready:', ready)
else:
    print('Final evaluation disabled. Keep all four full checkpoints for a later comparison session.')
""")

md("""## 7. Held-out accuracy, sources, errors, and deployment cost

Ordinary test images remain unrotated. Source counts are shown explicitly; the Printed test source is small and should not drive a deployment decision alone. CPU and GPU latency include the complete wrapper, including STN when present.
""")

code("""
def predict_frame(model, loader):
    rows = []
    model.eval()
    with torch.inference_mode():
        for images, labels, paths, sources, angles in loader:
            logits = model(images.to(device, non_blocking=True))
            probabilities = logits.softmax(1)
            confidence, predicted = probabilities.max(1)
            for index in range(len(labels)):
                rows.append({'crop_file': paths[index], 'source_dataset': sources[index],
                             'angle': int(angles[index]), 'true_digit': int(labels[index]),
                             'predicted_digit': int(predicted[index]),
                             'confidence': float(confidence[index])})
    return pd.DataFrame(rows)


def measured_latency(model, target_device, repetitions=40, warmup=10):
    model = model.to(target_device).eval()
    sample = torch.zeros(1, 3, INPUT_SIZE, INPUT_SIZE, device=target_device)
    with torch.inference_mode():
        for _ in range(warmup):
            model(sample)
        if target_device.type == 'cuda':
            torch.cuda.synchronize()
        started = time.perf_counter()
        for _ in range(repetitions):
            model(sample)
        if target_device.type == 'cuda':
            torch.cuda.synchronize()
    return model, (time.perf_counter() - started) * 1000 / repetitions


def per_class_table(frame, model_name, scope):
    precision, recall, f1, support = precision_recall_fscore_support(
        frame.true_digit, frame.predicted_digit, labels=list(range(10)), zero_division=0)
    return pd.DataFrame({'Model': model_name, 'Scope': scope, 'Digit': range(10),
                         'Precision': precision, 'Recall': recall, 'F1': f1, 'Support': support})


def save_confusions(frame, model_name):
    matrix = confusion_matrix(frame.true_digit, frame.predicted_digit, labels=list(range(10)))
    normalized = matrix / np.maximum(matrix.sum(axis=1, keepdims=True), 1)
    output_dir = OUTPUT_ROOT / model_name / 'metrics'
    np.savetxt(output_dir / 'confusion_matrix.csv', matrix, delimiter=',', fmt='%d')
    np.savetxt(output_dir / 'confusion_matrix_normalized.csv', normalized, delimiter=',', fmt='%.6f')
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for axis, values, title in zip(axes, (matrix, normalized), ('Count', 'Row-normalized')):
        image = axis.imshow(values, cmap='Blues')
        axis.set(title=f'{model_name}: {title}', xlabel='Predicted digit', ylabel='True digit',
                 xticks=range(10), yticks=range(10))
        fig.colorbar(image, ax=axis, fraction=0.046)
    fig.tight_layout(); fig.savefig(output_dir / 'confusion_matrices.png', dpi=150); plt.close(fig)
    return matrix


comparison_rows = []
source_rows = []
per_class_rows = []
if RUN_FINAL_EVALUATION:
    val_loader = make_loader(val_frame)
    test_loader = make_loader(test_frame)
    for name in ALL_MODELS:
        model, checkpoint_path, checkpoint = load_trained(name)
        validation = predict_frame(model, val_loader)
        test_predictions = predict_frame(model, test_loader)
        test_predictions.to_csv(OUTPUT_ROOT / name / 'metrics' / 'test_predictions.csv', index=False)
        matrix = save_confusions(test_predictions, name)
        for scope, subset in [('Overall', test_predictions),
                              ('SVHN', test_predictions.loc[test_predictions.source_dataset == 'SVHN']),
                              ('Printed', test_predictions.loc[test_predictions.source_dataset == 'Printed'])]:
            if subset.empty:
                source_rows.append({'Model': name, 'Source': scope, 'n': 0})
            else:
                source_rows.append({'Model': name, 'Source': scope,
                                    **metrics(subset.true_digit, subset.predicted_digit)})
                per_class_rows.append(per_class_table(subset, name, scope))
        val_metric = metrics(validation.true_digit, validation.predicted_digit)
        test_metric = metrics(test_predictions.true_digit, test_predictions.predicted_digit)
        gpu_ms = np.nan
        if device.type == 'cuda':
            model, gpu_ms = measured_latency(model, device)
        model, cpu_ms = measured_latency(model, torch.device('cpu'))
        latency = gpu_ms if device.type == 'cuda' else cpu_ms
        comparison_rows.append({'Model': name, 'STN': model.stn is not None,
            'Best Epoch': int(checkpoint['epoch']),
            'Parameters': sum(parameter.numel() for parameter in model.parameters()),
            'Model Size MB': checkpoint_path.stat().st_size / 1e6,
            'Validation Accuracy': val_metric['accuracy'], 'Validation Macro F1': val_metric['macro_f1'],
            'Test Accuracy': test_metric['accuracy'], 'Test Macro F1': test_metric['macro_f1'],
            'SVHN Accuracy': metrics(test_predictions.loc[test_predictions.source_dataset == 'SVHN', 'true_digit'],
                                     test_predictions.loc[test_predictions.source_dataset == 'SVHN', 'predicted_digit'])['accuracy'] if (test_predictions.source_dataset == 'SVHN').any() else np.nan,
            'Printed Accuracy': metrics(test_predictions.loc[test_predictions.source_dataset == 'Printed', 'true_digit'],
                                        test_predictions.loc[test_predictions.source_dataset == 'Printed', 'predicted_digit'])['accuracy'] if (test_predictions.source_dataset == 'Printed').any() else np.nan,
            'SVHN N': int((test_predictions.source_dataset == 'SVHN').sum()),
            'Printed N': int((test_predictions.source_dataset == 'Printed').sum()),
            '6→9 errors': int(matrix[6, 9]), '9→6 errors': int(matrix[9, 6]),
            '2→5 errors': int(matrix[2, 5]), '5→2 errors': int(matrix[5, 2]),
            '3→8 errors': int(matrix[3, 8]), '8→3 errors': int(matrix[8, 3]),
            '1→7 errors': int(matrix[1, 7]), '7→1 errors': int(matrix[7, 1]),
            'CPU Latency ms': cpu_ms, 'GPU Latency ms': gpu_ms,
            'Images/sec': 1000 / latency})
        del model
        if device.type == 'cuda':
            torch.cuda.empty_cache()
    comparison = pd.DataFrame(comparison_rows)
    comparison.to_csv(OUTPUT_ROOT / 'stn_model_comparison.csv', index=False)
    pd.DataFrame(source_rows).to_csv(OUTPUT_ROOT / 'source_metrics.csv', index=False)
    pd.concat(per_class_rows, ignore_index=True).to_csv(OUTPUT_ROOT / 'per_class_metrics.csv', index=False)
    display(comparison.round(4))
    display(pd.DataFrame(source_rows).round(4))
""")

md("""## 8. Fixed-angle robustness on a predeclared test subset

The same deterministically selected, basic-quality-filtered images are used for every model and angle. The fixed rotations are evaluation only. Class 6/9 ambiguity is retained and counted; the same source image appears at multiple angles, so these rows are correlated and are not independent test images.
""")

code("""
def select_rotation_test(frame):
    selected = []
    for digit in range(10):
        candidates = frame.loc[frame.digit == digit].sample(frac=1, random_state=SEED)
        acceptable = []
        for _, row in candidates.head(200).iterrows():
            with Image.open(DATA_ROOT / row.crop_file) as image:
                gray = np.asarray(image.convert('L').resize((32, 32)), dtype=np.float32)
                size_ok = min(image.size) >= 24
            if size_ok and gray.std() >= 12:
                acceptable.append(row)
            if len(acceptable) >= ROTATION_TEST_PER_CLASS:
                break
        if len(acceptable) < ROTATION_TEST_PER_CLASS:
            warnings.warn(f'Digit {digit}: only {len(acceptable)} clear test crops pass size/contrast filter')
        if not acceptable:
            raise ValueError(f'No suitable fixed-rotation test crop for digit {digit}')
        selected.extend(acceptable)
    return pd.DataFrame(selected).reset_index(drop=True)


rotation_subset = select_rotation_test(test_frame) if RUN_FINAL_EVALUATION else None
if RUN_FINAL_EVALUATION:
    print('Fixed-rotation unique source images:', len(rotation_subset))
    display(rotation_subset.groupby(['digit', 'source_dataset']).size().unstack(fill_value=0))
    rotation_loader = make_loader(rotation_subset, fixed_angles=ROTATION_TEST_ANGLES)
    rotation_predictions = []
    robustness_rows = []
    for name in ALL_MODELS:
        model, _, _ = load_trained(name)
        frame = predict_frame(model, rotation_loader)
        frame.insert(0, 'Model', name)
        rotation_predictions.append(frame)
        for angle, subset in frame.groupby('angle', sort=True):
            confusion = confusion_matrix(subset.true_digit, subset.predicted_digit, labels=list(range(10)))
            robustness_rows.append({'Model': name, 'STN': model.stn is not None, 'Angle': int(angle),
                                    'Accuracy': float(accuracy_score(subset.true_digit, subset.predicted_digit)),
                                    'n': len(subset), '6→9 errors': int(confusion[6, 9]),
                                    '9→6 errors': int(confusion[9, 6])})
        del model
        if device.type == 'cuda':
            torch.cuda.empty_cache()
    rotation_predictions = pd.concat(rotation_predictions, ignore_index=True)
    rotation_predictions.to_csv(OUTPUT_ROOT / 'rotation_predictions.csv', index=False)
    rotation_robustness = pd.DataFrame(robustness_rows)
    rotation_robustness.to_csv(OUTPUT_ROOT / 'rotation_robustness.csv', index=False)
    display(rotation_robustness.pivot(index='Angle', columns='Model', values='Accuracy').round(3))
    fig, ax = plt.subplots(figsize=(10, 5))
    for name, group in rotation_robustness.groupby('Model', sort=False):
        ax.plot(group.Angle, group.Accuracy, marker='o', label=name)
    ax.set(title='Digit accuracy by fixed in-plane rotation', xlabel='Rotation angle (degrees)',
           ylabel='Top-1 accuracy', xticks=ROTATION_TEST_ANGLES, ylim=(0, 1))
    ax.grid(alpha=0.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUTPUT_ROOT / 'rotation_robustness.png', dpi=160); plt.show()
""")

md("""## 9. STN before/after and affine diagnostics

The 32-row gallery uses two examples at each of eight angles for each STN architecture. The third column reports labels, prediction, confidence, and the learned affine matrix. `atan2(c, a)` is shown only as an approximate angle because scale, shear, and translation also affect the affine matrix. Correct, failure, and 6/9 galleries come from held-out rotated predictions. A visually upright output is not required for success.
""")

code("""
def gallery_rows(frame, category, limit=12):
    stn_rows = frame.loc[frame.Model.isin(['mobilenet_stn', 'resnet18_stn'])]
    if category == 'correct':
        stn_rows = stn_rows.loc[stn_rows.true_digit == stn_rows.predicted_digit]
    elif category == 'failure':
        stn_rows = stn_rows.loc[stn_rows.true_digit != stn_rows.predicted_digit]
    elif category == '6_9':
        stn_rows = stn_rows.loc[((stn_rows.true_digit == 6) & (stn_rows.predicted_digit == 9)) |
                                ((stn_rows.true_digit == 9) & (stn_rows.predicted_digit == 6))]
    return stn_rows.sample(n=min(limit, len(stn_rows)), random_state=SEED) if len(stn_rows) else stn_rows


def save_stn_gallery(rows, filename, heading):
    rows = rows.reset_index(drop=True)
    if rows.empty:
        fig, ax = plt.subplots(figsize=(8, 2))
        ax.text(0.5, 0.5, 'No matching held-out examples', ha='center', va='center')
        ax.axis('off'); fig.suptitle(heading)
        fig.savefig(OUTPUT_ROOT / filename, dpi=140); plt.close(fig)
        return
    fig, axes = plt.subplots(len(rows), 3, figsize=(9, 2.2 * len(rows)), squeeze=False)
    active_name, model = None, None
    for row_number, row in rows.iterrows():
        if row.Model != active_name:
            if model is not None:
                del model
            model, _, _ = load_trained(row.Model)
            active_name = row.Model
        with Image.open(DATA_ROOT / row.crop_file) as image:
            image_tensor = image_to_tensor(image.convert('RGB'), fixed_angle=int(row.angle)).unsqueeze(0).to(device)
        with torch.inference_mode():
            transformed, theta = model.transform_input(image_tensor)
        source_image = image_tensor[0].cpu().permute(1, 2, 0).numpy()
        transformed_image = transformed[0].cpu().clamp(0, 1).permute(1, 2, 0).numpy()
        affine = theta[0].cpu().numpy()
        approx_angle = math.degrees(math.atan2(float(affine[1, 0]), float(affine[0, 0])))
        axes[row_number, 0].imshow(source_image); axes[row_number, 0].set_title(f'{row.Model} input {int(row.angle)}°', fontsize=8)
        axes[row_number, 1].imshow(transformed_image); axes[row_number, 1].set_title('STN output', fontsize=8)
        description = (f'True {int(row.true_digit)}  Pred {int(row.predicted_digit)}  Confidence {row.confidence:.2f}\\n'
                       f'[{affine[0,0]:.2f} {affine[0,1]:.2f} {affine[0,2]:.2f}]\\n'
                       f'[{affine[1,0]:.2f} {affine[1,1]:.2f} {affine[1,2]:.2f}]\\n'
                       f'Approx. angle {approx_angle:.1f}°')
        axes[row_number, 2].text(0, 0.5, description, va='center', fontsize=8, family='monospace')
        for axis in axes[row_number]:
            axis.axis('off')
    fig.suptitle(heading)
    fig.tight_layout(rect=(0, 0, 1, 0.995))
    fig.savefig(OUTPUT_ROOT / filename, dpi=150)
    plt.close(fig)
    if model is not None:
        del model
    if device.type == 'cuda':
        torch.cuda.empty_cache()


if RUN_FINAL_EVALUATION:
    main_rows = []
    for name in ('mobilenet_stn', 'resnet18_stn'):
        for angle in (0, 45, 90, 135, 180, 225, 270, 315):
            candidates = rotation_predictions.loc[(rotation_predictions.Model == name) &
                                                  (rotation_predictions.angle == angle)]
            main_rows.append(candidates.sample(n=min(2, len(candidates)), random_state=SEED))
    main_rows = pd.concat(main_rows, ignore_index=True)
    assert len(main_rows) >= 30, 'Need at least 30 samples for the before/after visualization'
    save_stn_gallery(main_rows, 'stn_before_after.png', 'STN before / after (32 held-out rotated samples)')
    save_stn_gallery(gallery_rows(rotation_predictions, 'correct'), 'stn_correct_examples.png', 'STN correct examples')
    save_stn_gallery(gallery_rows(rotation_predictions, 'failure'), 'stn_failure_examples.png', 'STN failure examples')
    save_stn_gallery(gallery_rows(rotation_predictions, '6_9'), 'stn_6_9_examples.png', 'STN 6↔9 confusion examples')
""")

md("""## 10. Comparison and provisional research decision

The generated report answers the pairwise STN questions from actual held-out results. Its candidate is for a subsequent robot-camera trial; public SVHN/Printed performance alone cannot establish the best deployed classifier.
""")

code("""
def percentage(value):
    return f'{100 * value:.2f}%'


if RUN_FINAL_EVALUATION:
    lines = ['# STN digit recognition ablation', '',
             f'Dataset: `{DATA_ROOT.name}`; unchanged train/val/test splits. Seed {SEED}.',
             'All four variants used the same full-circle training augmentation and validation macro F1 for checkpoint selection.',
             f'Fixed-angle test used {len(rotation_subset)} unique test images, each at {len(ROTATION_TEST_ANGLES)} angles.',
             'The fixed-angle rows are correlated because each image is repeated.',
             'The Printed test source is small; source sample counts are in `source_metrics.csv`.',
             'A 180° rotation can make 6 and 9 indistinguishable without an orientation cue. No synthetic dot was used.', '',
             '## Paired results', '']
    for family, baseline_name, stn_name in (
        ('MobileNetV3-Small', 'mobilenet_baseline', 'mobilenet_stn'),
        ('ResNet18', 'resnet18_baseline', 'resnet18_stn')):
        baseline = comparison.loc[comparison.Model == baseline_name].iloc[0]
        stn = comparison.loc[comparison.Model == stn_name].iloc[0]
        base_rot = rotation_robustness.loc[rotation_robustness.Model == baseline_name].Accuracy.mean()
        stn_rot = rotation_robustness.loc[rotation_robustness.Model == stn_name].Accuracy.mean()
        base_69 = rotation_robustness.loc[rotation_robustness.Model == baseline_name, ['6→9 errors', '9→6 errors']].to_numpy().sum()
        stn_69 = rotation_robustness.loc[rotation_robustness.Model == stn_name, ['6→9 errors', '9→6 errors']].to_numpy().sum()
        pair = rotation_robustness.pivot(index='Angle', columns='Model', values='Accuracy')
        angle_gains = pair[stn_name] - pair[baseline_name]
        strongest_angle = int(angle_gains.idxmax())
        latency_column = 'GPU Latency ms' if device.type == 'cuda' else 'CPU Latency ms'
        lines += [f'### {family}',
                  f'- Overall test accuracy: baseline {percentage(baseline["Test Accuracy"])}; STN {percentage(stn["Test Accuracy"])}; change {100 * (stn["Test Accuracy"] - baseline["Test Accuracy"]):+.2f} percentage points.',
                  f'- Mean fixed-angle accuracy: baseline {percentage(base_rot)}; STN {percentage(stn_rot)}; change {100 * (stn_rot - base_rot):+.2f} percentage points.',
                  f'- Largest angle-specific STN gain: {strongest_angle}° ({100 * angle_gains.loc[strongest_angle]:+.2f} percentage points).',
                  f'- 6→9 errors: {int(baseline["6→9 errors"])} vs {int(stn["6→9 errors"])}; 9→6 errors: {int(baseline["9→6 errors"])} vs {int(stn["9→6 errors"])} on ordinary test.',
                  f'- Total 6↔9 errors across fixed angles: baseline {int(base_69)}; STN {int(stn_69)}.',
                  f'- Printed accuracy: {percentage(baseline["Printed Accuracy"])} vs {percentage(stn["Printed Accuracy"])} (n={int(stn["Printed N"])}).',
                  f'- Batch-1 {"GPU" if device.type == "cuda" else "CPU"} latency: {baseline[latency_column]:.2f} ms vs {stn[latency_column]:.2f} ms; STN adds {stn[latency_column] - baseline[latency_column]:+.2f} ms.',
                  f'- Model size: {baseline["Model Size MB"]:.1f} MB vs {stn["Model Size MB"]:.1f} MB.', '']
    theta_summary = pd.DataFrame(theta_rows) if theta_rows else pd.DataFrame()
    if (OUTPUT_ROOT / 'stn_theta_statistics.csv').is_file():
        theta_summary = pd.read_csv(OUTPUT_ROOT / 'stn_theta_statistics.csv')
    warnings_count = int(theta_summary.collapse_warning.fillna(False).sum()) if not theta_summary.empty else 0
    lines += ['## Stability and next decision',
              f'Theta diagnostic rows flagged for possible collapse: {warnings_count}. Inspect `stn_theta_statistics.csv` and before/after images; this flag is a warning, not proof of failure.',
              'Compare overall accuracy, fixed-angle accuracy, 6/9 errors, Printed support, latency, checkpoint size, and theta stability together. The provisional selection rule keeps models within 2 percentage points of both the strongest mean fixed-angle accuracy and the strongest ordinary-test macro F1, then picks the fastest eligible model. STNs flagged at their best epoch are excluded. Printed accuracy is reported but not a hard gate because its test support is small.',
              'Do not treat this as the final OpenMANIPULATOR-X choice. Test the candidates on a labeled, independently collected cube-camera set with different cube poses and lighting.',
              'An STN may improve classification without making every transformed image visually upright. Describe actual learned transforms after inspecting `stn_before_after.png`.',
              'No YOLO retraining, robot integration, or dot detection was performed.']
    best_epoch_by_model = comparison.set_index('Model')['Best Epoch'].to_dict()
    best_theta = theta_summary.loc[
        theta_summary.apply(lambda row: row.Epoch == best_epoch_by_model.get(row.Model), axis=1)
    ] if not theta_summary.empty else theta_summary
    theta_bad = set(best_theta.loc[best_theta.collapse_warning.fillna(False), 'Model']) if not best_theta.empty else set()
    eligible = comparison.loc[~comparison.Model.isin(theta_bad)]
    if eligible.empty:
        eligible = comparison.loc[~comparison.STN]
    mean_angle = rotation_robustness.groupby('Model').Accuracy.mean()
    eligible = eligible.assign(mean_fixed_angle_accuracy=eligible.Model.map(mean_angle))
    near_best = eligible.loc[(eligible.mean_fixed_angle_accuracy >= eligible.mean_fixed_angle_accuracy.max() - 0.02) &
                             (eligible['Test Macro F1'] >= eligible['Test Macro F1'].max() - 0.02)]
    if near_best.empty:
        near_best = eligible.sort_values(['mean_fixed_angle_accuracy', 'Test Macro F1'], ascending=False).head(1)
    latency_column = 'GPU Latency ms' if device.type == 'cuda' else 'CPU Latency ms'
    candidate = near_best.sort_values([latency_column, 'Model Size MB']).iloc[0].Model
    lines.insert(lines.index('## Stability and next decision') + 1,
                 f'**Provisional public-data candidate:** `{candidate}` (mean fixed-angle accuracy {percentage(mean_angle[candidate])}; ordinary-test macro F1 {percentage(comparison.set_index("Model").loc[candidate, "Test Macro F1"])}).')
    report_path = OUTPUT_ROOT / 'final_stn_report.md'
    report_path.write_text('\\n'.join(lines) + '\\n', encoding='utf-8')
    print(report_path.read_text(encoding='utf-8'))
else:
    print('Training-only session: final report requires all four full checkpoints and RUN_FINAL_EVALUATION=True.')
""")

for index, cell in enumerate(cells):
    cell['id'] = uuid.uuid5(uuid.NAMESPACE_URL, f'stn-digit-experiment-cell-{index}').hex[:8]

notebook = {'cells': cells,
            'metadata': {'kernelspec': {'display_name': 'Python 3', 'language': 'python', 'name': 'python3'},
                         'language_info': {'name': 'python'}},
            'nbformat': 4, 'nbformat_minor': 5}
target = HERE / 'stn_digit_recognition_experiment.ipynb'
target.write_text(json.dumps(notebook, indent=1, ensure_ascii=False) + '\n', encoding='utf-8')
print(target)
