"""Generate the self-contained Kaggle digit classifier comparison notebook."""
from __future__ import annotations

import json
import textwrap
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
cells = []


def md(source):
    cells.append({"cell_type": "markdown", "metadata": {}, "source": textwrap.dedent(source).strip() + "\n"})


def code(source):
    cells.append({"cell_type": "code", "execution_count": None, "metadata": {}, "outputs": [],
                  "source": textwrap.dedent(source).strip() + "\n"})


md("""
# Digit classifier comparison for OpenMANIPULATOR-X

**Experiment:** compare a small scratch CNN, ImageNet-pretrained MobileNetV3-Small, and ImageNet-pretrained ResNet18 on the existing `DIGIT_CLASSIFICATION_V1` crops. The dataset is not regenerated. No shape model or robot motion is used.

1. Attach the prepared dataset as a Kaggle Dataset. A folder containing `dataset_manifest.json` or a ZIP containing `DIGIT_CLASSIFICATION_V1` is supported.
2. Enable a GPU. Enable Internet for the official torchvision weights, or ensure those weights are already cached.
3. Run once with `SMOKE_TEST=True` to check the pipeline. This uses 10% of **train only** and does not evaluate test.
4. Set `SMOKE_TEST=False`, restart the session, and run top to bottom for the complete experiment. Three full models on 325,090 training crops may take more than one Kaggle session. For separate sessions, set `MODELS_TO_RUN` to one model and `RUN_FINAL_EVALUATION=False`; save that run's output as a Kaggle Dataset. In a final session attach the three output datasets, set `MODELS_TO_RUN=[]`, populate `EXTERNAL_CHECKPOINT_ROOTS`, and set `RUN_FINAL_EVALUATION=True`.

The earlier dataset build did not finish its exhaustive image-read validation. This notebook performs a bounded preflight and exposes suspect crops. Source-specific evaluation is essential: only **27** Printed Digit crops are in the current test split.
""")

code("""
import os, io, json, math, random, time, copy, shutil, zipfile, warnings
from pathlib import Path
from collections import Counter

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image, ImageOps, ImageEnhance
from IPython.display import display
from sklearn.metrics import (accuracy_score, precision_recall_fscore_support,
                             classification_report, confusion_matrix)
from sklearn.model_selection import train_test_split
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models
from torchvision.models import MobileNet_V3_Small_Weights, ResNet18_Weights

SEED = 42
INPUT_SIZE = 128                 # 96, 128, or 160 may be tested in a later experiment
BATCH_SIZE = 128                 # lower to 64 or 32 if CUDA reports out of memory
NUM_WORKERS = min(4, os.cpu_count() or 1)
SMOKE_TEST = True               # set False for the final full-data run
ROTATION_EXPERIMENT = False     # True in digit_rotation_experiment.ipynb
MODELS_TO_RUN = ["CNN_FROM_SCRATCH", "MOBILENETV3_SMALL", "RESNET18"]
RUN_FINAL_EVALUATION = True      # False for a single-model training session
EXTERNAL_CHECKPOINT_ROOTS = []   # Path('/kaggle/input/prior-run/digit_model_experiments'), etc.
MAX_EPOCHS_CNN = 30
MAX_EPOCHS_PRETRAINED = 20      # includes the head-only stage
HEAD_EPOCHS = 3
EARLY_STOPPING_PATIENCE = 5
USE_CLASS_WEIGHTS = True
RUN_PRINTED_ADAPTATION = False  # reserved; not part of this fair comparison
EVALUATE_TEST = (not SMOKE_TEST) and RUN_FINAL_EVALUATION
PREFLIGHT_SCAN_LIMIT = 2000
DATASET_ROOT_OVERRIDE = None    # e.g. Path('/kaggle/input/my-dataset/DIGIT_CLASSIFICATION_V1')
DATASET_ZIP_OVERRIDE = None     # e.g. Path('/kaggle/input/my-dataset/DIGIT_CLASSIFICATION_V1.zip')
OUTPUT_NAME = 'digit_rotation_experiment' if ROTATION_EXPERIMENT else 'digit_model_experiments'
OUTPUT_ROOT = Path('/kaggle/working') / OUTPUT_NAME if Path('/kaggle/working').exists() else Path(OUTPUT_NAME + '_output')
OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(SEED)
torch.backends.cudnn.benchmark = True
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print('torch', torch.__version__, 'torchvision', __import__('torchvision').__version__, 'device', device)
if device.type != 'cuda':
    warnings.warn('GPU is unavailable. Full training will be very slow; enable a Kaggle GPU.')
""")

md("""## 1. Locate the prepared dataset and audit its splits

The manifest, report, class table, and metadata are read from the attached dataset. The test split is never resplit or used for model selection. A folder count and a random image-read sample form the lightweight preflight; the earlier exhaustive validation gap remains documented.
""")

code("""
def require_image_splits(root):
    missing = [name for name in ('train', 'val', 'test') if not (root / name).is_dir()]
    if missing:
        visible = [p.name for p in root.iterdir()] if root.is_dir() else []
        raise FileNotFoundError(
            f'{root} contains the manifest but is missing image folders {missing}. '
            f'Visible entries: {visible}. Upload the complete DIGIT_CLASSIFICATION_V1 folder '
            'with train/0-9, val/0-9, and test/0-9, or attach its ZIP and set DATASET_ZIP_OVERRIDE.'
        )
    return root

def find_dataset():
    if DATASET_ROOT_OVERRIDE is not None:
        root = Path(DATASET_ROOT_OVERRIDE)
        if not (root / 'dataset_manifest.json').is_file():
            raise FileNotFoundError(root / 'dataset_manifest.json')
        return require_image_splits(root)
    if DATASET_ZIP_OVERRIDE is not None:
        zip_candidates = [Path(DATASET_ZIP_OVERRIDE)]
    else:
        zip_candidates = []
    if zip_candidates:
        return extract_dataset_zip(zip_candidates)
    search_roots = [Path('/kaggle/input'), Path.cwd(), Path.cwd().parent]
    for base in search_roots:
        if base.exists():
            hits = sorted(base.rglob('dataset_manifest.json'))
            hits = [p for p in hits if p.parent.name == 'DIGIT_CLASSIFICATION_V1'
                    and all((p.parent / split).is_dir() for split in ('train', 'val', 'test'))]
            if hits:
                return hits[0].parent
    if Path('/kaggle/input').exists():
        zip_candidates = sorted(Path('/kaggle/input').rglob('*.zip'))
    return extract_dataset_zip(zip_candidates)

def extract_dataset_zip(zip_candidates):
    for archive_path in zip_candidates:
        with zipfile.ZipFile(archive_path) as archive:
            names = archive.namelist()
            if not any('DIGIT_CLASSIFICATION_V1/dataset_manifest.json' in n for n in names):
                continue
            destination = OUTPUT_ROOT / 'dataset_unpacked'
            destination.mkdir(exist_ok=True)
            base = destination.resolve()
            for member in archive.infolist():
                target = (destination / member.filename).resolve()
                if target != base and base not in target.parents:
                    raise ValueError('Unsafe ZIP member path: ' + member.filename)
            archive.extractall(destination)
            hits = list(destination.rglob('dataset_manifest.json'))
            hits = [p for p in hits if p.parent.name == 'DIGIT_CLASSIFICATION_V1']
            if hits:
                return require_image_splits(hits[0].parent)
    raise FileNotFoundError('No complete DIGIT_CLASSIFICATION_V1 dataset found. Attach train/, val/, test/ with the manifest, or set DATASET_ZIP_OVERRIDE to a ZIP of the full dataset.')

DATA_ROOT = find_dataset()
manifest = json.loads((DATA_ROOT / 'dataset_manifest.json').read_text(encoding='utf-8'))
source_report = (DATA_ROOT / 'dataset_report.md').read_text(encoding='utf-8')
class_table = pd.read_csv(DATA_ROOT / 'class_distribution.csv')
metadata = pd.read_csv(DATA_ROOT / 'metadata.csv', dtype={'source_image': str, 'crop_file': str})
metadata['digit'] = metadata['digit'].astype(int)
metadata['crop_file'] = metadata['crop_file'].str.replace('\\\\', '/', regex=False)
metadata['source_dataset'] = metadata['source_dataset'].astype(str)
print('Dataset:', DATA_ROOT)
print('Manifest validation status:', manifest.get('full_validation_status', 'not recorded'))
print('Report excerpt:', source_report[-700:])
display(class_table)
""")

code("""
expected_splits = {'train', 'val', 'test'}
expected_classes = {str(i) for i in range(10)}
assert set(metadata['unified_split']) == expected_splits
assert set(metadata['digit']) == set(range(10))
assert not metadata['crop_file'].duplicated().any(), 'Duplicate metadata crop paths'
assert metadata['crop_file'].str.split('/').str[0].isin(expected_splits).all()
assert (metadata['crop_file'].str.split('/').str[1].astype(int) == metadata['digit']).all()

folder_counts = {}
split_filenames = {}
for split in ('train', 'val', 'test'):
    split_dir = DATA_ROOT / split
    assert split_dir.is_dir(), split_dir
    actual_classes = {p.name for p in split_dir.iterdir() if p.is_dir()}
    assert actual_classes == expected_classes, (split, actual_classes)
    names = set()
    for digit in range(10):
        folder = split_dir / str(digit)
        with os.scandir(folder) as entries:
            files = [entry.name for entry in entries if entry.is_file()]
        folder_counts[(split, digit)] = len(files)
        names.update(files)
    split_filenames[split] = names
assert not (split_filenames['train'] & split_filenames['val'])
assert not (split_filenames['train'] & split_filenames['test'])
assert not (split_filenames['val'] & split_filenames['test'])
metadata_counts = metadata.groupby(['unified_split', 'digit']).size().to_dict()
assert folder_counts == metadata_counts, 'Folder/metadata count mismatch'
assert len(metadata) == manifest['total_crops']
assert metadata['unified_split'].value_counts().to_dict() == manifest['split_counts']
assert metadata['source_dataset'].value_counts().to_dict() == manifest['source_counts']
group_splits = metadata.groupby(['source_dataset', 'source_split', 'source_image'])['unified_split'].nunique()
assert group_splits.max() == 1, 'Original source image crosses splits'

sample = metadata.sample(n=min(200, len(metadata)), random_state=SEED)
for rel in sample['crop_file']:
    with Image.open(DATA_ROOT / rel) as image:
        image.verify()
summary = metadata.groupby(['unified_split', 'source_dataset']).size().unstack(fill_value=0)
display(summary)
print('Lightweight preflight passed: all folder/metadata counts and 200 sampled images checked.')
print('This does not replace a full image-by-image readability scan.')
""")

md("""### Suspect crop report and galleries

Geometry flags use all metadata rows. Contrast/blank flags use a reproducible bounded image sample. Flags are for review; no crop is deleted. The training gallery shows 100 random train crops, as requested.
""")

code("""
geometry = metadata[['crop_file', 'digit', 'source_dataset', 'unified_split', 'crop_width', 'crop_height']].copy()
geometry['crop_width'] = pd.to_numeric(geometry['crop_width'])
geometry['crop_height'] = pd.to_numeric(geometry['crop_height'])
ratio = geometry['crop_width'] / geometry['crop_height']
geometry['flag_small'] = (geometry[['crop_width', 'crop_height']].min(axis=1) < 16)
geometry['flag_aspect'] = (ratio < 0.33) | (ratio > 3.0)
scan = metadata.sample(n=min(PREFLIGHT_SCAN_LIMIT, len(metadata)), random_state=SEED)
appearance = {}
for rel in scan['crop_file']:
    with Image.open(DATA_ROOT / rel) as image:
        gray = np.asarray(image.convert('L').resize((32, 32)), dtype=np.float32)
    appearance[rel] = (float(gray.std()), float(np.percentile(gray, 95) - np.percentile(gray, 5)))
geometry['gray_std'] = geometry['crop_file'].map(lambda x: appearance.get(x, (np.nan, np.nan))[0])
geometry['gray_p95_p5'] = geometry['crop_file'].map(lambda x: appearance.get(x, (np.nan, np.nan))[1])
geometry['flag_low_contrast'] = geometry['gray_std'] < 12
geometry['flag_possible_blank'] = geometry['gray_p95_p5'] < 18
flag_columns = ['flag_small', 'flag_aspect', 'flag_low_contrast', 'flag_possible_blank']
suspects = geometry.loc[geometry[flag_columns].any(axis=1)].copy()
suspects['flags'] = suspects[flag_columns].apply(lambda row: ','.join(c for c in flag_columns if row[c]), axis=1)
suspects.to_csv(OUTPUT_ROOT / 'suspect_crops.csv', index=False)

def crop_gallery(frame, output_file, title, n=100):
    chosen = frame.head(n)
    columns = 10
    rows = max(1, math.ceil(len(chosen) / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(16, 1.85 * rows))
    for ax in np.asarray(axes).reshape(-1):
        ax.axis('off')
    for ax, (_, row) in zip(np.asarray(axes).reshape(-1), chosen.iterrows()):
        with Image.open(DATA_ROOT / row['crop_file']) as image:
            ax.imshow(image.convert('RGB'))
        ax.set_title(f"{row['digit']} | {row['source_dataset']}", fontsize=7)
        ax.axis('off')
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_file, dpi=120)
    plt.show()

train_random_100 = metadata.loc[metadata['unified_split'] == 'train'].sample(n=100, random_state=SEED)
crop_gallery(train_random_100, OUTPUT_ROOT / 'random_100_train_crops.png', '100 random train crops')
suspects_gallery = suspects.sort_values(['flag_possible_blank', 'flag_low_contrast', 'flag_aspect', 'flag_small'], ascending=False)
crop_gallery(suspects_gallery, OUTPUT_ROOT / 'suspect_crops_gallery.png', 'Suspect crop examples')
print('Suspect count:', len(suspects), 'flags:', suspects[flag_columns].sum().to_dict())
""")

code("""
preflight_lines = [
    '# Dataset preflight report',
    f'Dataset: `{DATA_ROOT}`',
    f"Manifest full-validation status: {manifest.get('full_validation_status', 'not recorded')}",
    f'Metadata rows and folder crops: {len(metadata):,}',
    f"Split counts: {metadata['unified_split'].value_counts().to_dict()}",
    f"Source counts: {metadata['source_dataset'].value_counts().to_dict()}",
    f"Test source counts: {metadata.loc[metadata.unified_split == 'test', 'source_dataset'].value_counts().to_dict()}",
    f'Suspect rows (geometry complete; appearance sample {len(scan)}): {len(suspects)}',
    f'Suspect flags: {suspects[flag_columns].sum().to_dict()}',
    'No crops were removed. Filename overlap and source-image split overlap checks passed.',
    'A random 200-image readability check passed; this is not exhaustive validation.',
    'Printed test has a very small denominator, so its accuracy is directional only.',
    'Gallery review is required because source-edge truncation and neighboring SVHN digits can survive numeric checks.',
]
(OUTPUT_ROOT / 'dataset_preflight_report.md').write_text('\\n\\n'.join(preflight_lines) + '\\n', encoding='utf-8')
print((OUTPUT_ROOT / 'dataset_preflight_report.md').read_text())
""")

md("""## 2. Input processing and loaders

Every image is converted to RGB, padded to a centered square with its median border color, then resized to 128×128. Moderate augmentation is applied only to train. There are no flips or 180° rotations. ImageNet normalization is used for both pretrained models; the scratch CNN uses fixed RGB mean/std of 0.5 for a transparent baseline.
""")

md("""### Optional full rotation experiment for numbered cubes

The separate `digit_rotation_experiment.ipynb` enables this cell. On **training images only**, digits 1–5 receive a random in-plane rotation from -180° to +180°. The canvas expands before resizing so diagonal digits are not cut off. Digits 6 and 9 retain the original small rotation because rotating them can change their identity without an orientation mark. A fixed-angle subset of the existing validation split is used alongside upright validation to select checkpoints. The held-out test split remains untouched. This is a public-data rotation experiment; it cannot establish performance on the robot camera without a labeled real-camera test set. The baseline notebook keeps this switch off.
""")

code("""
ROTATION_AUGMENT_CLASSES = frozenset({1, 2, 3, 4, 5})
EXTRA_ROTATION_DEGREES = 180
ROTATED_VAL_ANGLES = (45, 90, 135, 180, 225, 270, 315)
ROTATED_VAL_PER_CLASS = 30


def rotate_on_expanded_canvas(image, angle):
    square = pad_square(image)
    fill = square.getpixel((0, 0))
    return transforms.functional.rotate(
        square, angle, interpolation=transforms.InterpolationMode.BILINEAR,
        expand=True, fill=fill)

def rotate_train_digit(image, digit):
    if not ROTATION_EXPERIMENT or int(digit) not in ROTATION_AUGMENT_CLASSES:
        return image
    return rotate_on_expanded_canvas(image, random.uniform(-EXTRA_ROTATION_DEGREES,
                                                          EXTRA_ROTATION_DEGREES))

print('Extra rotation experiment:', ROTATION_EXPERIMENT,
      'classes:', sorted(ROTATION_AUGMENT_CLASSES),
      'training-only angle range:', (-EXTRA_ROTATION_DEGREES, EXTRA_ROTATION_DEGREES))
""")

code("""
def pad_square(image):
    image = image.convert('RGB')
    array = np.asarray(image)
    border = np.concatenate([array[0], array[-1], array[:, 0], array[:, -1]], axis=0)
    fill = tuple(int(v) for v in np.median(border, axis=0))
    width, height = image.size
    side = max(width, height)
    left = (side - width) // 2
    top = (side - height) // 2
    canvas = Image.new('RGB', (side, side), fill)
    canvas.paste(image, (left, top))
    return canvas

augment = transforms.Compose([
    transforms.RandomRotation(20, fill=127),
    transforms.RandomPerspective(distortion_scale=0.15, p=0.20, fill=127),
    transforms.RandomAffine(degrees=0,
                            translate=(0.12, 0.12) if ROTATION_EXPERIMENT else (0.06, 0.06),
                            scale=(0.85, 1.15) if ROTATION_EXPERIMENT else (0.90, 1.10),
                            fill=127),
    transforms.ColorJitter(brightness=0.15, contrast=0.15, saturation=0.10),
    transforms.RandomApply([transforms.GaussianBlur(kernel_size=3, sigma=(0.1, 0.8))], p=0.10),
    transforms.RandomGrayscale(p=0.05),
])

def make_transform(train, pretrained):
    mean = (0.485, 0.456, 0.406) if pretrained else (0.5, 0.5, 0.5)
    std = (0.229, 0.224, 0.225) if pretrained else (0.5, 0.5, 0.5)
    steps = [transforms.Lambda(pad_square), transforms.Resize((INPUT_SIZE, INPUT_SIZE))]
    if train:
        steps.append(augment)
    steps.extend([transforms.ToTensor(), transforms.Normalize(mean, std)])
    return transforms.Compose(steps)

class DigitCrops(Dataset):
    def __init__(self, frame, transform, train=False):
        self.records = frame[['crop_file', 'digit', 'source_dataset']].to_records(index=False)
        self.transform = transform
        self.train = train
    def __len__(self):
        return len(self.records)
    def __getitem__(self, index):
        rel, digit, source = self.records[index]
        with Image.open(DATA_ROOT / rel) as image:
            rgb = image.convert('RGB')
        if self.train:
            rgb = rotate_train_digit(rgb, digit)
        return self.transform(rgb), int(digit), str(rel), str(source)

train_frame = metadata.loc[metadata.unified_split == 'train'].reset_index(drop=True)
val_frame = metadata.loc[metadata.unified_split == 'val'].reset_index(drop=True)
test_frame = metadata.loc[metadata.unified_split == 'test'].reset_index(drop=True)
if SMOKE_TEST:
    _, sample_indices = train_test_split(np.arange(len(train_frame)), test_size=0.10,
                                         stratify=train_frame['digit'], random_state=SEED)
    train_frame = train_frame.iloc[np.sort(sample_indices)].reset_index(drop=True)
    print('Smoke mode: stratified train subset', len(train_frame), 'validation unchanged', len(val_frame), 'test untouched')

full_train_counts = metadata.loc[metadata.unified_split == 'train', 'digit'].value_counts().reindex(range(10), fill_value=0)
weights = 1.0 / np.sqrt(full_train_counts.to_numpy(dtype=np.float64))
weights = weights / weights.mean()
class_weights = torch.tensor(weights, dtype=torch.float32, device=device) if USE_CLASS_WEIGHTS else None
print('Train-only class weights:', dict(zip(range(10), np.round(weights, 3))))

def loaders(pretrained):
    common = dict(batch_size=BATCH_SIZE, num_workers=NUM_WORKERS,
                  pin_memory=(device.type == 'cuda'), persistent_workers=(NUM_WORKERS > 0))
    return (
        DataLoader(DigitCrops(train_frame, make_transform(True, pretrained), train=True), shuffle=True, **common),
        DataLoader(DigitCrops(val_frame, make_transform(False, pretrained)), shuffle=False, **common),
        DataLoader(DigitCrops(test_frame, make_transform(False, pretrained)), shuffle=False, **common),
    )


class RotatedValidationCrops(Dataset):
    def __init__(self, frame, transform):
        selected = []
        for digit in sorted(ROTATION_AUGMENT_CLASSES):
            candidates = frame.loc[frame.digit == digit]
            selected.append(candidates.sample(n=min(ROTATED_VAL_PER_CLASS, len(candidates)),
                                              random_state=SEED))
        self.records = pd.concat(selected)[['crop_file', 'digit', 'source_dataset']].to_records(index=False)
        self.transform = transform

    def __len__(self):
        return len(self.records) * len(ROTATED_VAL_ANGLES)

    def __getitem__(self, index):
        record_index, angle_index = divmod(index, len(ROTATED_VAL_ANGLES))
        rel, digit, source = self.records[record_index]
        angle = ROTATED_VAL_ANGLES[angle_index]
        with Image.open(DATA_ROOT / rel) as image:
            rotated = rotate_on_expanded_canvas(image.convert('RGB'), angle)
        return self.transform(rotated), int(digit), str(rel) + f'@{angle}', str(source)


def rotated_validation_loader(pretrained):
    if not ROTATION_EXPERIMENT:
        return None
    common = dict(batch_size=BATCH_SIZE, num_workers=NUM_WORKERS,
                  pin_memory=(device.type == 'cuda'), persistent_workers=(NUM_WORKERS > 0))
    return DataLoader(RotatedValidationCrops(val_frame, make_transform(False, pretrained)),
                      shuffle=False, **common)

preview = train_frame.sample(n=12, random_state=SEED)
fig, axes = plt.subplots(2, 6, figsize=(13, 5))
for ax, (_, row) in zip(axes.flat, preview.iterrows()):
    with Image.open(DATA_ROOT / row.crop_file) as image:
        transformed = augment(pad_square(image).resize((INPUT_SIZE, INPUT_SIZE)))
    ax.imshow(transformed); ax.set_title(str(row.digit)); ax.axis('off')
fig.suptitle('Training-only augmentation preview (no flips)'); fig.tight_layout(); plt.show()
""")

code("""
if ROTATION_EXPERIMENT:
    examples = train_frame.loc[train_frame.digit == 1].sample(n=12, random_state=SEED)
    fig, axes = plt.subplots(2, 6, figsize=(12, 5))
    for ax, (_, row) in zip(axes.flat, examples.iterrows()):
        with Image.open(DATA_ROOT / row.crop_file) as image:
            rotated = rotate_train_digit(image.convert('RGB'), row.digit)
        ax.imshow(rotated)
        ax.set_title('label 1')
        ax.axis('off')
    fig.suptitle('Extra training-only rotation: digit 1 examples')
    fig.tight_layout()
    fig.savefig(OUTPUT_ROOT / 'rotation_preview_digit_1.png', dpi=140)
    plt.show()
""")

md("""## 3. Define the three models

The scratch CNN uses 16→32→64→96 channels and global pooling so it remains genuinely small. Pretrained torchvision weights are mandatory for MobileNetV3-Small and ResNet18; a failed download stops the run instead of silently using random weights.
""")

code("""
class CustomDigitCNN(nn.Module):
    def __init__(self, classes=10):
        super().__init__()
        def block(cin, cout, pool=True):
            layers = [nn.Conv2d(cin, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                      nn.Conv2d(cout, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True)]
            return nn.Sequential(*layers, *([nn.MaxPool2d(2)] if pool else []))
        self.features = nn.Sequential(block(3, 16), block(16, 32), block(32, 64),
                                      nn.Conv2d(64, 96, 3, padding=1, bias=False), nn.BatchNorm2d(96),
                                      nn.ReLU(inplace=True), nn.AdaptiveAvgPool2d(1))
        self.classifier = nn.Sequential(nn.Flatten(), nn.Dropout(0.2), nn.Linear(96, classes))
    def forward(self, x):
        return self.classifier(self.features(x))

def create_model(name, load_imagenet_weights=True):
    if name == 'CNN_FROM_SCRATCH':
        return CustomDigitCNN(), False
    if name == 'MOBILENETV3_SMALL':
        model = models.mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT if load_imagenet_weights else None)
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, 10)
        return model, True
    if name == 'RESNET18':
        model = models.resnet18(weights=ResNet18_Weights.DEFAULT if load_imagenet_weights else None)
        model.fc = nn.Linear(model.fc.in_features, 10)
        return model, True
    raise ValueError(name)

def freeze_backbone(model, name, frozen):
    model._head_only = frozen
    for parameter in model.parameters():
        parameter.requires_grad = not frozen
    if frozen:
        head = model.fc if name == 'RESNET18' else model.classifier
        for parameter in head.parameters():
            parameter.requires_grad = True

scratch_probe = CustomDigitCNN()
print('Custom CNN parameters:', sum(p.numel() for p in scratch_probe.parameters()))
del scratch_probe
""")

md("""## 4. Train with validation-based checkpoint selection

AdamW uses inverse-square-root class weights from full train only. In the rotation experiment, the scheduler, early stopping, and `best.pt` use the mean of upright and fixed-angle validation macro F1. The fixed-angle validation metric covers digits 1–5 only. The original comparison still uses upright validation macro F1. The test split is not opened during rotation training. Mixed precision and gradient scaling activate on CUDA. Lower `BATCH_SIZE` if CUDA OOM occurs, then restart the run.
""")

code("""
def classification_metrics(true, predicted, labels=None):
    labels = list(range(10)) if labels is None else list(labels)
    precision, recall, macro_f1, _ = precision_recall_fscore_support(
        true, predicted, labels=labels, average='macro', zero_division=0)
    _, _, weighted_f1, _ = precision_recall_fscore_support(
        true, predicted, labels=labels, average='weighted', zero_division=0)
    return {'accuracy': float(accuracy_score(true, predicted)), 'macro_precision': float(precision),
            'macro_recall': float(recall), 'macro_f1': float(macro_f1), 'weighted_f1': float(weighted_f1),
            'n': len(true)}

def run_epoch(model, loader, criterion, optimizer=None, scaler=None, metric_labels=None):
    training = optimizer is not None
    model.train(training)
    # Frozen backbone BatchNorm statistics stay fixed in the head-only stage.
    if training and getattr(model, '_head_only', False):
        for module in model.modules():
            if isinstance(module, nn.modules.batchnorm._BatchNorm):
                module.eval()
    true, predicted, losses = [], [], []
    context = torch.enable_grad() if training else torch.inference_mode()
    with context:
        for images, labels, _, _ in loader:
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
            losses.append(float(loss.detach()) * len(labels))
            true.extend(labels.cpu().tolist())
            predicted.extend(logits.argmax(1).cpu().tolist())
    return {'loss': sum(losses) / len(true), **classification_metrics(true, predicted, metric_labels)}

def train_one(name):
    model, pretrained = create_model(name)
    model = model.to(device)
    train_loader, val_loader, _ = loaders(pretrained)
    rotated_val_loader = rotated_validation_loader(pretrained)
    criterion = nn.CrossEntropyLoss(weight=class_weights)
    scaler = torch.amp.GradScaler('cuda', enabled=(device.type == 'cuda'))
    output_dir = OUTPUT_ROOT / name.lower()
    (output_dir / 'metrics').mkdir(parents=True, exist_ok=True)
    (output_dir / 'plots').mkdir(exist_ok=True)
    best_f1, best_epoch, epoch_number = -1.0, -1, 0
    history = []
    stages = [('scratch', MAX_EPOCHS_CNN, 1e-3)] if not pretrained else [
        ('head', HEAD_EPOCHS, 1e-3), ('full', MAX_EPOCHS_PRETRAINED - HEAD_EPOCHS, 2e-4)]
    for stage, stage_epochs, learning_rate in stages:
        freeze_backbone(model, name, stage == 'head')
        optimizer = torch.optim.AdamW((p for p in model.parameters() if p.requires_grad),
                                      lr=learning_rate, weight_decay=1e-4)
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='max', factor=0.5, patience=2)
        patience = 0
        for _ in range(stage_epochs):
            epoch_number += 1
            train_result = run_epoch(model, train_loader, criterion, optimizer, scaler)
            val_result = run_epoch(model, val_loader, criterion)
            rotated_val_result = (run_epoch(model, rotated_val_loader, criterion,
                                           metric_labels=sorted(ROTATION_AUGMENT_CLASSES))
                                  if rotated_val_loader is not None else None)
            selection_score = (val_result['macro_f1'] + rotated_val_result['macro_f1']) / 2 if rotated_val_result else val_result['macro_f1']
            scheduler.step(selection_score)
            history.append({'epoch': epoch_number, 'stage': stage, 'lr': optimizer.param_groups[0]['lr'],
                            **{'train_' + k: v for k, v in train_result.items()},
                            **{'val_' + k: v for k, v in val_result.items()},
                            'val_rotated_macro_f1': rotated_val_result['macro_f1'] if rotated_val_result else None,
                            'selection_score': selection_score})
            checkpoint = {'model_name': name, 'state_dict': model.state_dict(), 'epoch': epoch_number,
                          'input_size': INPUT_SIZE, 'smoke_test': SMOKE_TEST,
                          'validation_macro_f1': val_result['macro_f1'],
                          'validation_rotated_macro_f1': rotated_val_result['macro_f1'] if rotated_val_result else None,
                          'selection_score': selection_score,
                          'rotation_experiment': ROTATION_EXPERIMENT,
                          'rotation_classes': sorted(ROTATION_AUGMENT_CLASSES) if ROTATION_EXPERIMENT else [],
                          'rotation_degrees': EXTRA_ROTATION_DEGREES if ROTATION_EXPERIMENT else 0}
            torch.save(checkpoint, output_dir / 'last.pt')
            if selection_score > best_f1:
                best_f1, best_epoch, patience = selection_score, epoch_number, 0
                torch.save(checkpoint, output_dir / 'best.pt')
            else:
                patience += 1
            print(name, stage, epoch_number, 'train loss', round(train_result['loss'], 4),
                  'val upright F1', round(val_result['macro_f1'], 4),
                  'val rotated F1', round(rotated_val_result['macro_f1'], 4) if rotated_val_result else 'n/a',
                  'selection', round(selection_score, 4), flush=True)
            pd.DataFrame(history).to_csv(output_dir / 'metrics' / 'history.csv', index=False)
            if patience >= EARLY_STOPPING_PATIENCE and stage != 'head':
                print('Early stopping:', name, stage)
                break
    best = torch.load(output_dir / 'best.pt', map_location='cpu', weights_only=False)
    model.load_state_dict(best['state_dict'])
    val_result = run_epoch(model, val_loader, criterion)
    rotated_val_result = (run_epoch(model, rotated_val_loader, criterion,
                                   metric_labels=sorted(ROTATION_AUGMENT_CLASSES))
                          if rotated_val_loader is not None else None)
    del train_loader, val_loader, rotated_val_loader
    if device.type == 'cuda':
        torch.cuda.empty_cache()
    return {'model': model, 'pretrained': pretrained, 'best_epoch': best_epoch,
            'validation': val_result, 'rotated_validation': rotated_val_result,
            'selection_score': best_f1, 'output_dir': output_dir}

trained = {}
for model_name in MODELS_TO_RUN:
    trained[model_name] = train_one(model_name)
print('All selected models trained. Best checkpoints used validation data only.')
""")

md("""## 5. Test evaluation and source-specific metrics

This section runs only when `SMOKE_TEST=False`. It is deliberately below the training loop. Overall, SVHN, and Printed results are reported with sample counts. With 27 Printed test crops, small accuracy changes are not reliable evidence of general robot-camera performance.
""")

code("""
if EVALUATE_TEST:
    assert not SMOKE_TEST
    all_models = {'CNN_FROM_SCRATCH', 'MOBILENETV3_SMALL', 'RESNET18'}
    for missing_name in all_models - set(trained):
        candidates = [Path(root) / missing_name.lower() / 'best.pt' for root in EXTERNAL_CHECKPOINT_ROOTS]
        existing = next((p for p in candidates if p.is_file()), None)
        if existing is None:
            raise FileNotFoundError(f'Missing full-run checkpoint for {missing_name}. Attach prior Kaggle output and set EXTERNAL_CHECKPOINT_ROOTS, or train all three here.')
        checkpoint = torch.load(existing, map_location='cpu', weights_only=False)
        assert checkpoint['model_name'] == missing_name and checkpoint['input_size'] == INPUT_SIZE
        assert not checkpoint['smoke_test'], 'A smoke checkpoint cannot enter the final comparison.'
        model, pretrained = create_model(missing_name, load_imagenet_weights=False)
        model.load_state_dict(checkpoint['state_dict'])
        model = model.to(device)
        output_dir = OUTPUT_ROOT / missing_name.lower()
        (output_dir / 'metrics').mkdir(parents=True, exist_ok=True)
        (output_dir / 'plots').mkdir(exist_ok=True)
        shutil.copy2(existing, output_dir / 'best.pt')
        prior_last = existing.with_name('last.pt')
        if prior_last.is_file():
            shutil.copy2(prior_last, output_dir / 'last.pt')
        _, val_loader, _ = loaders(pretrained)
        validation = run_epoch(model, val_loader, nn.CrossEntropyLoss(weight=class_weights))
        trained[missing_name] = {'model': model, 'pretrained': pretrained,
                                 'best_epoch': checkpoint['epoch'], 'validation': validation,
                                 'output_dir': output_dir}
        del val_loader
    assert set(trained) == all_models, 'All three full-run checkpoints are required before test.'
    assert all(not torch.load(result['output_dir'] / 'best.pt', map_location='cpu', weights_only=False)['smoke_test']
               for result in trained.values())

def predict_frame(model, loader):
    model.eval()
    records = []
    with torch.inference_mode():
        for images, labels, paths, sources in loader:
            logits = model(images.to(device, non_blocking=True))
            probabilities = logits.softmax(1).cpu()
            predicted = probabilities.argmax(1)
            confidence = probabilities.max(1).values
            for path, source, true, pred, conf in zip(paths, sources, labels.tolist(), predicted.tolist(), confidence.tolist()):
                records.append({'filename': path, 'source_dataset': source, 'true_class': true,
                                'predicted_class': pred, 'confidence': conf, 'correct': true == pred})
    return pd.DataFrame.from_records(records)

def metrics_row(frame, model_name, source):
    row = classification_metrics(frame['true_class'].to_numpy(), frame['predicted_class'].to_numpy())
    return {'Model': model_name, 'Source': source, **row}

def make_confusion(predictions, output_dir, title):
    matrix = confusion_matrix(predictions.true_class, predictions.predicted_class, labels=list(range(10)))
    for normalized in (False, True):
        values = matrix.astype(float)
        if normalized:
            values /= np.maximum(values.sum(axis=1, keepdims=True), 1)
        fig, ax = plt.subplots(figsize=(7.5, 6.5))
        image = ax.imshow(values, cmap='Blues')
        ax.set(xticks=range(10), yticks=range(10), xlabel='Predicted digit', ylabel='True digit',
               title=title + (' (row normalized)' if normalized else ' (counts)'))
        fig.colorbar(image, ax=ax, shrink=0.8)
        fig.tight_layout()
        fig.savefig(output_dir / ('confusion_normalized.png' if normalized else 'confusion_counts.png'), dpi=150)
        plt.show()
    return matrix

def prediction_gallery(frame, path, title):
    if frame.empty:
        return
    n = min(50, len(frame))
    fig, axes = plt.subplots(math.ceil(n / 10), 10, figsize=(16, 2 * math.ceil(n / 10)))
    for ax in np.asarray(axes).reshape(-1): ax.axis('off')
    for ax, (_, row) in zip(np.asarray(axes).reshape(-1), frame.head(n).iterrows()):
        with Image.open(DATA_ROOT / row.filename) as image: ax.imshow(image.convert('RGB'))
        ax.set_title(f"{row.true_class}->{row.predicted_class} {row.confidence:.2f}", fontsize=7)
        ax.axis('off')
    fig.suptitle(title); fig.tight_layout(); fig.savefig(path, dpi=120); plt.show()

overall_rows, svhn_rows, printed_rows, per_class_rows, comparison_rows = [], [], [], [], []
validation_source_rows, validation_key_rates = [], {}
if EVALUATE_TEST:
    for model_name, result in trained.items():
        _, val_loader, test_loader = loaders(result['pretrained'])
        val_predictions = predict_frame(result['model'], val_loader)
        for source in ('Overall', 'SVHN', 'Printed'):
            subset = val_predictions if source == 'Overall' else val_predictions.loc[val_predictions.source_dataset == source]
            validation_source_rows.append(metrics_row(subset, model_name, source))
        validation_plots = result['output_dir'] / 'plots' / 'validation'
        validation_plots.mkdir(parents=True, exist_ok=True)
        val_matrix = make_confusion(val_predictions, validation_plots, model_name + ' validation')
        key_pairs = [(0, 8), (1, 7), (3, 8), (5, 6), (6, 9), (9, 6)]
        validation_key_rates[model_name] = sum(int(val_matrix[a, b]) for a, b in key_pairs) / len(val_predictions)
        val_class_report = classification_report(val_predictions.true_class, val_predictions.predicted_class,
                                                 labels=list(range(10)), output_dict=True, zero_division=0)
        pd.DataFrame([{'Digit': digit, 'Precision': val_class_report[str(digit)]['precision'],
                       'Recall': val_class_report[str(digit)]['recall'],
                       'F1': val_class_report[str(digit)]['f1-score'],
                       'Class Accuracy': val_class_report[str(digit)]['recall'],
                       'Support': val_class_report[str(digit)]['support']} for digit in range(10)]).to_csv(
            result['output_dir'] / 'metrics' / 'validation_per_class.csv', index=False)
        predictions = predict_frame(result['model'], test_loader)
        output_dir = result['output_dir']
        metrics_dir = output_dir / 'metrics'
        plots_dir = output_dir / 'plots'
        predictions.to_csv(metrics_dir / 'test_predictions.csv', index=False)
        overall = metrics_row(predictions, model_name, 'Overall')
        svhn = metrics_row(predictions.loc[predictions.source_dataset == 'SVHN'], model_name, 'SVHN')
        printed = metrics_row(predictions.loc[predictions.source_dataset == 'Printed'], model_name, 'Printed')
        overall_rows.append(overall); svhn_rows.append(svhn); printed_rows.append(printed)
        for source, subset in [('Overall', predictions), ('SVHN', predictions.loc[predictions.source_dataset == 'SVHN']),
                               ('Printed', predictions.loc[predictions.source_dataset == 'Printed'])]:
            report = classification_report(subset.true_class, subset.predicted_class, labels=list(range(10)),
                                           output_dict=True, zero_division=0)
            for digit in range(10):
                item = report[str(digit)]
                per_class_rows.append({'Model': model_name, 'Source': source, 'Digit': digit,
                                       'Precision': item['precision'], 'Recall': item['recall'],
                                       'F1': item['f1-score'], 'Class Accuracy': item['recall'],
                                       'Support': item['support']})
        matrix = make_confusion(predictions, plots_dir, model_name)
        predictions['correct'] = predictions.true_class == predictions.predicted_class
        prediction_gallery(predictions.loc[predictions.correct].sample(frac=1, random_state=SEED),
                           plots_dir / 'correct_50.png', model_name + ': correct')
        prediction_gallery(predictions.loc[~predictions.correct].sample(frac=1, random_state=SEED),
                           plots_dir / 'incorrect_50.png', model_name + ': incorrect')
        prediction_gallery(predictions.loc[~predictions.correct].sort_values('confidence', ascending=False),
                           plots_dir / 'high_confidence_wrong.png', model_name + ': confident errors')
        prediction_gallery(predictions.loc[predictions.correct].sort_values('confidence'),
                           plots_dir / 'low_confidence_correct.png', model_name + ': low confidence correct')
        for true_digit, pred_digit in ((6, 9), (9, 6), (0, 8), (1, 7), (3, 8), (5, 6)):
            subset = predictions.loc[(predictions.true_class == true_digit) & (predictions.predicted_class == pred_digit)]
            prediction_gallery(subset, plots_dir / f'{true_digit}_as_{pred_digit}.png',
                               f'{model_name}: {true_digit} predicted as {pred_digit}')
        major_pairs = sorted([(int(matrix[i, j]), i, j) for i in range(10) for j in range(10) if i != j], reverse=True)[:5]
        (metrics_dir / 'major_confusions.json').write_text(json.dumps(major_pairs, indent=2))
        print(model_name, 'overall n=', overall['n'], 'SVHN n=', svhn['n'], 'Printed n=', printed['n'],
              'major confusions=', major_pairs)
        del val_loader, test_loader
""")

md("""## 6. Deployment-oriented latency and comparison

Latency measures an already-preprocessed 1×3×128×128 tensor, excluding camera capture, crop extraction, and YOLO26/ROS work. CPU and GPU numbers are specific to the Kaggle machine. CPU and GPU use the same warmup and 100 timed batch-1 inferences. A later robot-host benchmark is still required.
""")

code("""
def latency_ms(model, target_device, repeats=100, warmup=20):
    candidate = copy.deepcopy(model).to(target_device).eval()
    sample = torch.zeros(1, 3, INPUT_SIZE, INPUT_SIZE, device=target_device)
    with torch.inference_mode():
        for _ in range(warmup): candidate(sample)
        if target_device.type == 'cuda': torch.cuda.synchronize()
        started = time.perf_counter()
        for _ in range(repeats): candidate(sample)
        if target_device.type == 'cuda': torch.cuda.synchronize()
    elapsed = (time.perf_counter() - started) * 1000 / repeats
    del candidate
    return elapsed

if EVALUATE_TEST:
    pd.DataFrame(overall_rows).to_csv(OUTPUT_ROOT / 'overall_metrics.csv', index=False)
    pd.DataFrame(svhn_rows).to_csv(OUTPUT_ROOT / 'svhn_metrics.csv', index=False)
    pd.DataFrame(printed_rows).to_csv(OUTPUT_ROOT / 'printed_metrics.csv', index=False)
    pd.DataFrame(per_class_rows).to_csv(OUTPUT_ROOT / 'per_class_metrics.csv', index=False)
    pd.DataFrame(validation_source_rows).to_csv(OUTPUT_ROOT / 'validation_source_metrics.csv', index=False)
    for name, result in trained.items():
        model = result['model']
        cpu_ms = latency_ms(model, torch.device('cpu'))
        gpu_ms = latency_ms(model, torch.device('cuda')) if device.type == 'cuda' else np.nan
        effective_ms = gpu_ms if device.type == 'cuda' else cpu_ms
        overall = next(row for row in overall_rows if row['Model'] == name)
        svhn = next(row for row in svhn_rows if row['Model'] == name)
        printed = next(row for row in printed_rows if row['Model'] == name)
        printed_val = next(row for row in validation_source_rows if row['Model'] == name and row['Source'] == 'Printed')
        comparison_rows.append({
            'Model': name, 'Pretraining': 'None' if name == 'CNN_FROM_SCRATCH' else 'ImageNet',
            'Parameters': sum(p.numel() for p in model.parameters()),
            'Model Size MB': (result['output_dir'] / 'best.pt').stat().st_size / 1e6,
            'Best Epoch': result['best_epoch'],
            'Validation Accuracy': result['validation']['accuracy'],
            'Validation Macro F1': result['validation']['macro_f1'],
            'Test Accuracy': overall['accuracy'], 'Test Macro F1': overall['macro_f1'],
            'SVHN Test Accuracy': svhn['accuracy'], 'SVHN Test N': svhn['n'],
            'Printed Test Accuracy': printed['accuracy'], 'Printed Test N': printed['n'],
            'Printed Val Accuracy': printed_val['accuracy'], 'Printed Val N': printed_val['n'],
            'Val Key Confusion Rate': validation_key_rates[name],
            'CPU Latency ms': cpu_ms, 'GPU Latency ms': gpu_ms, 'Images/sec': 1000 / effective_ms,
        })
    comparison = pd.DataFrame(comparison_rows)
    comparison.to_csv(OUTPUT_ROOT / 'model_comparison.csv', index=False)
    display(comparison.round(4))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].bar(comparison['Model'], comparison['Validation Macro F1'])
    axes[0].set(title='Validation macro F1', ylabel='F1', ylim=(0, 1))
    axes[1].bar(comparison['Model'], comparison['CPU Latency ms'])
    axes[1].set(title='Batch-1 CPU latency on Kaggle host', ylabel='Milliseconds')
    for ax in axes: ax.tick_params(axis='x', rotation=25)
    fig.tight_layout(); fig.savefig(OUTPUT_ROOT / 'model_comparison.png', dpi=150); plt.show()
else:
    print('No test evaluation in this run. For the final comparison, use full-data checkpoints for all three models and set RUN_FINAL_EVALUATION=True.')
""")

md("""## 7. Final report and next phase

The candidate is provisional: among models within one percentage point of the best validation macro F1, rank CPU latency, Printed **validation** accuracy, and key validation confusion rate. Printed **test** accuracy is reported only after checkpoint selection and does not change the selection. The final deployed classifier requires actual white-paper digits on cube faces from the robot camera; this public-data stage is a **general digit model**.
""")

code("""
if EVALUATE_TEST:
    best_validation = comparison['Validation Macro F1'].max()
    close = comparison.loc[comparison['Validation Macro F1'] >= best_validation - 0.01].copy()
    close['decision_score'] = (0.50 * close['CPU Latency ms'].rank(ascending=True) +
                               0.25 * close['Printed Val Accuracy'].rank(ascending=False) +
                               0.25 * close['Val Key Confusion Rate'].rank(ascending=True))
    provisional = close.sort_values('decision_score').iloc[0]['Model']
    lines = [
        '# Final digit model comparison',
        f'Dataset: {DATA_ROOT}; train/val/test = {len(metadata.loc[metadata.unified_split == "train"]):,}/'
        f'{len(val_frame):,}/{len(test_frame):,} crops.',
        f'Test sources: {test_frame.source_dataset.value_counts().to_dict()}. Printed test accuracy has a small denominator.',
        'All models use the same existing splits, input size, training augmentations, class weighting, and validation selection metric.',
        'The scratch CNN starts from random weights; MobileNetV3-Small and ResNet18 use official ImageNet weights.',
        f'Provisional candidate: **{provisional}**. Eligibility: validation macro F1 within 0.01 of best. '
        'Within that set: rank CPU latency (50%), Printed validation accuracy (25%), and key validation confusion rate (25%).',
        'Printed validation also has a small denominator; this ranking is a decision aid, not a statistical guarantee.',
        'Review model_comparison.csv, printed_metrics.csv, and each model’s major_confusions.json before committing to hardware.',
        'Kaggle batch-1 latency excludes camera capture, YOLO26, crop extraction, and ROS. Benchmark on the robot host next.',
        'Public-data performance does not establish accuracy on white-paper digits attached to cubes; collect real camera images and fine-tune later.',
        'Printed-domain adaptation is disabled in this baseline comparison.',
    ]
    for _, row in comparison.iterrows():
        confusion_file = OUTPUT_ROOT / row['Model'].lower() / 'metrics' / 'major_confusions.json'
        confusions = json.loads(confusion_file.read_text())
        lines.append(f"{row['Model']}: val macro F1={row['Validation Macro F1']:.4f}; test accuracy={row['Test Accuracy']:.4f}; "
                     f"Printed val accuracy={row['Printed Val Accuracy']:.4f} (n={int(row['Printed Val N'])}); "
                     f"Printed test accuracy={row['Printed Test Accuracy']:.4f} (n={int(row['Printed Test N'])}); "
                     f"CPU={row['CPU Latency ms']:.2f} ms; checkpoint={row['Model Size MB']:.2f} MB; "
                     f"major confusion counts={confusions}.")
    (OUTPUT_ROOT / 'final_model_report.md').write_text('\\n\\n'.join(lines) + '\\n', encoding='utf-8')
    print((OUTPUT_ROOT / 'final_model_report.md').read_text())
else:
    print('Check losses and checkpoints. Complete all three full-data runs before the final evaluation.')
print('Artifacts:', OUTPUT_ROOT)
""")

md("""### Optional later experiment: Printed Digit adaptation

Disabled by default. If explicitly enabled after the baseline comparison, this starts from the provisional full-data winner, uses every Printed train crop plus a seeded class-balanced SVHN train subset, and fine-tunes at a much lower learning rate. It writes to a separate directory and does not alter any baseline result.
""")

code("""
if RUN_PRINTED_ADAPTATION:
    assert EVALUATE_TEST and not SMOKE_TEST, 'Complete the full baseline comparison first.'
    printed_train = metadata.loc[(metadata.unified_split == 'train') & (metadata.source_dataset == 'Printed')]
    svhn_train = metadata.loc[(metadata.unified_split == 'train') & (metadata.source_dataset == 'SVHN')]
    svhn_balanced = pd.concat([
        svhn_train.loc[svhn_train.digit == digit].sample(n=min(50, (svhn_train.digit == digit).sum()), random_state=SEED)
        for digit in range(10)
    ], ignore_index=True)
    adaptation_frame = pd.concat([printed_train, svhn_balanced], ignore_index=True)
    winner_info = trained[provisional]
    adapted = copy.deepcopy(winner_info['model']).to(device)
    adapted._head_only = False
    adaptation_loader = DataLoader(DigitCrops(adaptation_frame, make_transform(True, winner_info['pretrained'])),
                                   batch_size=min(BATCH_SIZE, 64), shuffle=True, num_workers=NUM_WORKERS,
                                   pin_memory=(device.type == 'cuda'), persistent_workers=(NUM_WORKERS > 0))
    _, adaptation_val_loader, adaptation_test_loader = loaders(winner_info['pretrained'])
    adaptation_optimizer = torch.optim.AdamW(adapted.parameters(), lr=2e-5, weight_decay=1e-4)
    adaptation_scaler = torch.amp.GradScaler('cuda', enabled=(device.type == 'cuda'))
    adaptation_criterion = nn.CrossEntropyLoss()
    adaptation_dir = OUTPUT_ROOT / 'printed_adaptation'
    adaptation_dir.mkdir(exist_ok=True)
    best_val_f1 = -1.0
    for adaptation_epoch in range(1, 4):
        run_epoch(adapted, adaptation_loader, adaptation_criterion, adaptation_optimizer, adaptation_scaler)
        val_result = run_epoch(adapted, adaptation_val_loader, adaptation_criterion)
        payload = {'state_dict': adapted.state_dict(), 'source_model': provisional,
                   'epoch': adaptation_epoch, 'validation_macro_f1': val_result['macro_f1']}
        torch.save(payload, adaptation_dir / 'last.pt')
        if val_result['macro_f1'] > best_val_f1:
            best_val_f1 = val_result['macro_f1']
            torch.save(payload, adaptation_dir / 'best.pt')
    adapted.load_state_dict(torch.load(adaptation_dir / 'best.pt', map_location='cpu', weights_only=False)['state_dict'])
    adapted_predictions = predict_frame(adapted, adaptation_test_loader)
    adapted_predictions.to_csv(adaptation_dir / 'test_predictions.csv', index=False)
    adaptation_results = pd.DataFrame([
        metrics_row(adapted_predictions, provisional + '_ADAPTED', 'Overall'),
        metrics_row(adapted_predictions.loc[adapted_predictions.source_dataset == 'SVHN'], provisional + '_ADAPTED', 'SVHN'),
        metrics_row(adapted_predictions.loc[adapted_predictions.source_dataset == 'Printed'], provisional + '_ADAPTED', 'Printed'),
    ])
    adaptation_results.to_csv(adaptation_dir / 'source_metrics.csv', index=False)
    display(adaptation_results)
else:
    print('Printed adaptation remains disabled; baseline results are isolated.')
""")

for index, cell in enumerate(cells):
    cell["id"] = uuid.uuid5(uuid.NAMESPACE_URL, f"digit-classifier-comparison-{index}").hex[:8]

notebook = {"cells": cells, "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
                                        "language_info": {"name": "python"}}, "nbformat": 4, "nbformat_minor": 5}
target = HERE / "digit_classifier_comparison.ipynb"
target.write_text(json.dumps(notebook, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
print(target)

rotation_cells = json.loads(json.dumps(cells))
config = rotation_cells[1]["source"]
config = config.replace('SMOKE_TEST = True', 'SMOKE_TEST = False')
config = config.replace("ROTATION_EXPERIMENT = False", "ROTATION_EXPERIMENT = True")
config = config.replace('MODELS_TO_RUN = ["CNN_FROM_SCRATCH", "MOBILENETV3_SMALL", "RESNET18"]',
                        'MODELS_TO_RUN = ["MOBILENETV3_SMALL", "RESNET18"]')
config = config.replace('RUN_FINAL_EVALUATION = True', 'RUN_FINAL_EVALUATION = False')
rotation_cells[1]["source"] = config
rotation_cells[0]["source"] = (
    '# Rotation-augmented digit classifier experiment for OpenMANIPULATOR-X\n\n'
    'This is a separate follow-up to the completed baseline comparison. It uses the same '
    '`DIGIT_CLASSIFICATION_V1` train/validation/test split and trains MobileNetV3-Small and '
    'ResNet18 again. Training images for digits 1–5 receive random full-circle in-plane '
    'rotation on an expanded canvas; all other train classes keep the original mild '
    'augmentation. A fixed-angle subset of validation digits 1–5 checks rotated accuracy '
    'alongside the upright validation split. '
    'The baseline notebook and its saved results remain separate.\n\n'
    'Attach the complete prepared dataset as a Kaggle Dataset and enable a GPU. '
    '`SMOKE_TEST=False` is set for the requested full run. If you want a short pipeline check '
    'first, temporarily set it to `True`; restart and restore `False` before full training. '
    'The notebook selects checkpoints using mean upright and rotated validation macro F1 '
    'and leaves public test evaluation disabled. Afterward, compare '
    'downloaded `best.pt` checkpoints on manually labeled, rotated real cube-camera crops. '
    'A tiny three-frame cube-1 pilot is diagnostic only. In-plane rotation does not address '
    'cube tilt, blur, occlusion, or the ambiguity between 6 and 9.\n'
)
for cell in rotation_cells:
    if cell["cell_type"] == "code" and "print('All selected models trained. Best checkpoints used validation data only.')" in cell["source"]:
        cell["source"] += (
            "\nrotation_summary = pd.DataFrame([\n"
            "    {'model': name, 'best_epoch': result['best_epoch'],\n"
            "     'upright_val_macro_f1': result['validation']['macro_f1'],\n"
            "     'rotated_val_macro_f1': result['rotated_validation']['macro_f1'],\n"
            "     'selection_score': result['selection_score']}\n"
            "    for name, result in trained.items()\n"
            "])\n"
            "rotation_summary.to_csv(OUTPUT_ROOT / 'rotation_validation_summary.csv', index=False)\n"
            "display(rotation_summary)\n"
        )
    if cell["cell_type"] == "markdown" and cell["source"].startswith('## 5. Test evaluation'):
        cell["source"] = ('## 5. Test evaluation\n\n'
                          'Disabled for this rotation experiment. The notebook trains and selects '
                          'checkpoints by upright and fixed-angle samples from the validation split; do the real cube-camera '
                          'evaluation after training.\n')
    if cell["cell_type"] == "code" and "Complete all three full-data runs before the final evaluation." in cell["source"]:
        cell["source"] = cell["source"].replace(
            "Check losses and checkpoints. Complete all three full-data runs before the final evaluation.",
            "Rotation experiment: review validation history and test the saved best.pt files on labeled real cube-camera scenes.")
rotation_notebook = {**notebook, "cells": rotation_cells}
rotation_target = HERE / "digit_rotation_experiment.ipynb"
rotation_target.write_text(json.dumps(rotation_notebook, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
print(rotation_target)
