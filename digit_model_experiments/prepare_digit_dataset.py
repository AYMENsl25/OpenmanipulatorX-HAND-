"""Build DIGIT_CLASSIFICATION_V1 from the local SVHN ZIP and Printed Digit YOLO set.

Run from the workspace root with `py digit_model_experiments/prepare_digit_dataset.py`. Raw inputs are never modified.
The optional --parse-mat mode is invoked with an h5py-enabled Python interpreter.
"""
from __future__ import annotations

import argparse
import collections
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import zipfile

SEED = 42
CROP_PADDING_RATIO = 0.15
USE_SVHN_EXTRA = True
MAX_SVHN_EXTRA_TOTAL = 100000  # Set to None for all source images.
SVHN_VAL_RATIO = 0.10
PRINTED_VAL_RATIO = 0.10
MIN_CROP_WIDTH = 8
MIN_CROP_HEIGHT = 8

ROOT = Path(__file__).resolve().parent.parent
SOURCE_SEARCH_ROOTS = (ROOT.parent, ROOT, Path.home() / "Downloads")
OUTPUT = ROOT / "digit_classification_dataset" / "DIGIT_CLASSIFICATION_V1"
H5PY_PYTHON = Path(os.environ.get(
    "DIGIT_H5PY_PYTHON", str(Path.home() / "miniconda3" / "envs" / "vmtk" / "python.exe")
))
if not H5PY_PYTHON.is_file():
    H5PY_PYTHON = Path(sys.executable)
FIELDS = ["crop_file", "digit", "source_dataset", "source_split", "unified_split",
          "source_image", "source_image_path", "digit_index", "original_class_id",
          "bbox_left", "bbox_top", "bbox_width", "bbox_height", "padded_left",
          "padded_top", "padded_width", "padded_height", "crop_width", "crop_height"]


def parse_mat(mat_path: Path, out_path: Path) -> None:
    import h5py
    with h5py.File(mat_path, "r") as handle, out_path.open("w", encoding="utf-8") as out:
        names = handle["digitStruct/name"]
        bboxes = handle["digitStruct/bbox"]
        for i in range(len(names)):
            name = "".join(chr(int(x)) for x in handle[names[i, 0]][()].flatten())
            box = handle[bboxes[i, 0]]
            values = {}
            for field in ("label", "left", "top", "width", "height"):
                dataset = box[field]
                raw = dataset[()]
                if raw.dtype.kind == "O":
                    values[field] = [float(handle[ref][()].flatten()[0]) for ref in raw.flatten()]
                else:
                    values[field] = [float(x) for x in raw.flatten()]
            if len(set(map(len, values.values()))) != 1:
                raise ValueError(f"Inconsistent SVHN bbox lengths: {name}")
            out.write(json.dumps({"name": name, "boxes": [dict(zip(values, x)) for x in zip(*values.values())]}) + "\n")
            if (i + 1) % 20000 == 0:
                print(f"parsed {i+1}/{len(names)}: {mat_path.name}", flush=True)


def locate_inputs():
    svhn = sorted({p.resolve() for root in SOURCE_SEARCH_ROOTS for p in root.glob("*SVHN*.zip")})
    printed = sorted({p.resolve() for root in SOURCE_SEARCH_ROOTS for p in root.glob("*Printed*Digit*")
                      if p.is_dir() and (p / "data.yaml").exists()})
    if len(svhn) != 1 or len(printed) != 1:
        raise RuntimeError(f"Expected one SVHN ZIP and one Printed Digit folder; found {svhn}, {printed}")
    return svhn[0], printed[0]


def read_yaml_names(path):
    # The actual Roboflow YAML uses a simple inline list; reject unsupported formats.
    import ast
    line = next((x for x in path.read_text(encoding="utf-8").splitlines() if x.startswith("names:")), None)
    if line is None:
        raise ValueError("Missing class names in data.yaml")
    names = ast.literal_eval(line.split(":", 1)[1].strip())
    if isinstance(names, dict):
        names = [names[i] for i in range(len(names))]
    mapping = {i: int(value) for i, value in enumerate(names)}
    if sorted(mapping.values()) != list(range(10)):
        raise ValueError(f"Unexpected Printed Digit class mapping: {mapping}")
    return mapping


def crop_box(image, box):
    left, top, width, height = (float(box[k]) for k in ("left", "top", "width", "height"))
    if not all(math.isfinite(v) for v in (left, top, width, height)) or width <= 0 or height <= 0:
        raise ValueError("invalid_annotation")
    if left >= image.width or top >= image.height or left + width <= 0 or top + height <= 0:
        raise ValueError("outside_image")
    x0 = max(0, math.floor(left - CROP_PADDING_RATIO * width))
    y0 = max(0, math.floor(top - CROP_PADDING_RATIO * height))
    x1 = min(image.width, math.ceil(left + width + CROP_PADDING_RATIO * width))
    y1 = min(image.height, math.ceil(top + height + CROP_PADDING_RATIO * height))
    if x1 - x0 < MIN_CROP_WIDTH or y1 - y0 < MIN_CROP_HEIGHT:
        raise ValueError("tiny_crop")
    return image.crop((x0, y0, x1, y1)), (x0, y0, x1-x0, y1-y0)


def main():
    from PIL import Image, ImageDraw, ImageOps

    svhn_zip, printed = locate_inputs()
    mapping = read_yaml_names(printed / "data.yaml")
    if OUTPUT.exists():
        raise RuntimeError(f"Output already exists: {OUTPUT}. Move it aside before rebuilding.")
    with zipfile.ZipFile(svhn_zip) as archive:
        members = {s: {Path(info.filename).name: info for info in archive.infolist()
                       if info.filename.startswith(s + "/") and info.filename.lower().endswith(".png")}
                   for s in ("train", "extra", "test")}
        for split in ("train", "extra", "test"):
            if f"{split}_digitStruct.mat" not in archive.namelist():
                raise RuntimeError(f"Missing {split}_digitStruct.mat")
        printed_counts = {s: len(list((printed / s / "images").iterdir())) for s in ("train", "valid", "test")}
        estimate_sources = len(members["train"]) + len(members["test"]) + (min(len(members["extra"]), MAX_SVHN_EXTRA_TOTAL) if USE_SVHN_EXTRA and MAX_SVHN_EXTRA_TOTAL is not None else (len(members["extra"]) if USE_SVHN_EXTRA else 0)) + sum(printed_counts.values())
        print("SVHN source images:", {s: len(v) for s, v in members.items()}, flush=True)
        print("Printed source images:", printed_counts, "mapping:", mapping, flush=True)
        print(f"Estimated source images: {estimate_sources}; estimated crop storage: {estimate_sources * 1.5 * 5_000 / 1e9:.2f} GB (rough).", flush=True)
        if USE_SVHN_EXTRA and MAX_SVHN_EXTRA_TOTAL is None:
            print("WARNING: processing full SVHN extra may require several GB and hours.", flush=True)

        OUTPUT.mkdir(parents=True)
        for split in ("train", "val", "test"):
            for digit in range(10):
                (OUTPUT / split / str(digit)).mkdir(parents=True)
        (OUTPUT / "quarantine").mkdir()
        counts = collections.Counter()
        classes = collections.Counter()
        source_counts = collections.Counter()
        rejected = collections.Counter()
        resolutions = collections.Counter()
        crop_resolutions = collections.Counter()
        hashes = {}
        duplicates = collections.Counter()
        mult_digit_images = 0
        missing = []
        annotation_issues = []
        examples = collections.defaultdict(list)
        random_examples = []
        rng = random.Random(SEED)
        total_seen = 0
        report_rejections = (OUTPUT / "quarantine" / "rejected.csv").open("w", newline="", encoding="utf-8")
        rejection_writer = csv.writer(report_rejections)
        rejection_writer.writerow(["source_dataset", "source_split", "source_image", "digit_index", "reason"])
        metadata_file = (OUTPUT / "metadata.csv").open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(metadata_file, fieldnames=FIELDS)
        writer.writeheader()

        def reject(source, split, image, index, reason):
            rejected[reason] += 1
            rejection_writer.writerow([source, split, image, index, reason])

        def save_crop(image, box, digit, source, source_split, unified, name, source_path, index, original_id):
            nonlocal total_seen
            if digit not in range(10):
                reject(source, source_split, name, index, "impossible_label")
                return
            try:
                crop, padded = crop_box(image, box)
                buffer = io.BytesIO()
                crop.convert("RGB").save(buffer, format="PNG")
                data = buffer.getvalue()
                digest = hashlib.sha256(data).hexdigest()
            except Exception as exc:
                reject(source, source_split, name, index, str(exc))
                return
            stem = Path(name).stem
            safe_stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)
            filename = f"{source.lower()}_{source_split}_{safe_stem}_digit{index:02d}.png"
            rel = f"{unified}/{digit}/{filename}"
            if digest in hashes:
                prior = hashes[digest]
                duplicates[(prior["unified_split"], unified)] += 1
                priority = {"train": 0, "val": 1, "test": 2}
                if priority[unified] > priority[prior["unified_split"]]:
                    # A previous lower-priority crop is removed from generated output.
                    (OUTPUT / prior["crop_file"]).unlink()
                    prior["removed"] = True
                    counts[prior["unified_split"]] -= 1
                    classes[int(prior["digit"])] -= 1
                    source_counts[prior["source_dataset"]] -= 1
                    hashes[digest] = None
                elif prior["unified_split"] != unified:
                    reject(source, source_split, name, index, "exact_cross_split_duplicate")
                    return
            (OUTPUT / rel).write_bytes(data)
            row = {"crop_file": rel, "digit": digit, "source_dataset": source, "source_split": source_split,
                   "unified_split": unified, "source_image": name, "source_image_path": str(source_path),
                   "digit_index": index, "original_class_id": original_id,
                   "bbox_left": box["left"], "bbox_top": box["top"], "bbox_width": box["width"],
                   "bbox_height": box["height"], "padded_left": padded[0], "padded_top": padded[1],
                   "padded_width": padded[2], "padded_height": padded[3], "crop_width": crop.width,
                   "crop_height": crop.height}
            writer.writerow(row)
            hashes[digest] = row
            counts[unified] += 1
            classes[digit] += 1
            source_counts[source] += 1
            crop_resolutions[(crop.width, crop.height)] += 1
            if len(examples[(unified, digit, source)]) < 4:
                examples[(unified, digit, source)].append(rel)
            total_seen += 1
            if len(random_examples) < 100:
                random_examples.append(rel)
            else:
                j = rng.randrange(total_seen)
                if j < 100:
                    random_examples[j] = rel

        temp = OUTPUT / "_working"
        temp.mkdir()
        try:
            for split in ("train", "extra", "test"):
                if split == "extra" and not USE_SVHN_EXTRA:
                    continue
                mat = temp / f"{split}.mat"
                with archive.open(f"{split}_digitStruct.mat") as src, mat.open("wb") as dst:
                    shutil.copyfileobj(src, dst, 8*1024*1024)
                parsed = temp / f"{split}.jsonl"
                subprocess.run([str(H5PY_PYTHON), str(Path(__file__).resolve()), "--parse-mat", str(mat), str(parsed)], check=True)
                mat.unlink()
                # Group-safe, reproducible image sampling. Labels remain in their source-image groups.
                if split == "extra" and MAX_SVHN_EXTRA_TOTAL is not None:
                    selected = set(rng.sample(sorted(members[split]), min(MAX_SVHN_EXTRA_TOTAL, len(members[split]))))
                else:
                    selected = None
                with parsed.open(encoding="utf-8") as annotations:
                    for i, line in enumerate(annotations):
                        record = json.loads(line)
                        name = record["name"]
                        if selected is not None and name not in selected:
                            continue
                        info = members[split].get(name)
                        if not info:
                            missing.append(f"SVHN {split}/{name}")
                            continue
                        unified = "test" if split == "test" else "train" if split == "extra" else ("val" if int(hashlib.sha256(f'{SEED}:{name}'.encode()).hexdigest(), 16) % 10000 < SVHN_VAL_RATIO*10000 else "train")
                        try:
                            with archive.open(info) as stream:
                                image = Image.open(stream).convert("RGB")
                        except Exception as exc:
                            reject("SVHN", split, name, -1, f"unreadable_image:{exc}")
                            continue
                        resolutions[image.size] += 1
                        if len(record["boxes"]) > 1:
                            mult_digit_images += 1
                        for index, box in enumerate(record["boxes"]):
                            raw_label = int(box["label"])
                            digit = 0 if raw_label == 10 else raw_label
                            save_crop(image, box, digit, "SVHN", split, unified, name, f"{svhn_zip}!/{info.filename}", index, raw_label)
                        if (i+1) % 10000 == 0:
                            print(f"SVHN {split}: {i+1} annotations, {sum(counts.values())} crops", flush=True)
                parsed.unlink()
        finally:
            shutil.rmtree(temp)

        for split in ("train", "valid", "test"):
            images = sorted(p for p in (printed / split / "images").iterdir() if p.is_file())
            for image_path in images:
                unified = {"valid": "val", "test": "test"}.get(split, "train")
                label_path = printed / split / "labels" / (image_path.stem + ".txt")
                if not label_path.exists():
                    missing.append(str(label_path))
                    continue
                try:
                    image = Image.open(image_path).convert("RGB")
                except Exception as exc:
                    reject("Printed", split, image_path.name, -1, f"unreadable_image:{exc}")
                    continue
                resolutions[image.size] += 1
                lines = label_path.read_text(encoding="utf-8").splitlines()
                if len(lines) > 1:
                    mult_digit_images += 1
                for index, line in enumerate(lines):
                    try:
                        parts = line.split()
                        if len(parts) != 5:
                            raise ValueError("invalid_annotation")
                        class_id = int(parts[0])
                        xc, yc, w, h = map(float, parts[1:])
                        if not all(0 <= v <= 1 for v in (xc, yc, w, h)):
                            raise ValueError("invalid_annotation")
                        digit = mapping[class_id]
                        box = {"left": (xc-w/2)*image.width, "top": (yc-h/2)*image.height,
                               "width": w*image.width, "height": h*image.height}
                        save_crop(image, box, digit, "Printed", split, unified, image_path.name, image_path, index, class_id)
                    except Exception as exc:
                        reject("Printed", split, image_path.name, index, str(exc))

        metadata_file.close()
        report_rejections.close()
        # Remove metadata entries whose output was superseded by a higher-priority duplicate.
        rows = list(csv.DictReader((OUTPUT / "metadata.csv").open(newline="", encoding="utf-8")))
        rows = [row for row in rows if (OUTPUT / row["crop_file"]).exists()]
        with (OUTPUT / "metadata.csv").open("w", newline="", encoding="utf-8") as stream:
            final_writer = csv.DictWriter(stream, fieldnames=FIELDS)
            final_writer.writeheader()
            final_writer.writerows(rows)
        counts = collections.Counter(row["unified_split"] for row in rows)
        classes = collections.Counter(int(row["digit"]) for row in rows)
        source_counts = collections.Counter(row["source_dataset"] for row in rows)
        distribution = []
        for digit in range(10):
            subset = [r for r in rows if int(r["digit"]) == digit]
            distribution.append({"Digit": digit,
                "SVHN Train": sum(r["source_dataset"] == "SVHN" and r["source_split"] == "train" and r["unified_split"] == "train" for r in subset),
                "SVHN Extra": sum(r["source_split"] == "extra" for r in subset),
                "Printed Train": sum(r["source_dataset"] == "Printed" and r["unified_split"] == "train" for r in subset),
                "Val": sum(r["unified_split"] == "val" for r in subset),
                "Test": sum(r["unified_split"] == "test" for r in subset), "Total": len(subset)})
        with (OUTPUT / "class_distribution.csv").open("w", newline="", encoding="utf-8") as stream:
            cw = csv.DictWriter(stream, fieldnames=list(distribution[0]))
            cw.writeheader(); cw.writerows(distribution)

        def gallery(path, chosen, title):
            canvas = Image.new("RGB", (1000, 110*math.ceil(len(chosen)/10)+35), "white")
            draw = ImageDraw.Draw(canvas)
            draw.text((8, 8), title, fill="black")
            for n, rel in enumerate(chosen):
                with Image.open(OUTPUT / rel) as item:
                    tile = ImageOps.contain(item.convert("RGB"), (88, 82))
                x, y = (n%10)*100+6, (n//10)*110+28
                canvas.paste(tile, (x, y))
                draw.text((x, y+84), Path(rel).parent.name, fill="black")
            canvas.save(OUTPUT / path)

        for split in ("train", "val", "test"):
            chosen = []
            for source in ("SVHN", "Printed"):
                for digit in range(10):
                    chosen.extend(p for p in examples[(split, digit, source)] if (OUTPUT / p).exists())
            gallery(f"sample_gallery_{split}.png", chosen, f"{split}: SVHN then Printed, digits 0-9")
        gallery("random_100_crops.png", [p for p in random_examples if (OUTPUT/p).exists()], "Random sample of generated crops")

        manifest = {"dataset_name": "DIGIT_CLASSIFICATION_V1", "creation_date": "2026-10-02", "seed": SEED,
                    "svhn_paths": {"zip": str(svhn_zip)}, "printed_digit_paths": {"root": str(printed), "yaml": str(printed/"data.yaml")},
                    "crop_padding": CROP_PADDING_RATIO, "minimum_crop_size": [MIN_CROP_WIDTH, MIN_CROP_HEIGHT],
                    "svhn_extra_configuration": {"use": USE_SVHN_EXTRA, "max_source_images": MAX_SVHN_EXTRA_TOTAL},
                    "total_images": estimate_sources, "total_crops": len(rows), "split_counts": dict(counts),
                    "class_counts": dict(sorted(classes.items())), "source_counts": dict(source_counts),
                    "quarantine_counts": dict(rejected), "exact_duplicate_pairs": {str(k): v for k, v in duplicates.items()},
                    "missing_files": missing, "annotation_issues": annotation_issues}
        (OUTPUT / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        valid_counts = [classes[d] for d in range(10) if classes[d]]
        report = ["# DIGIT_CLASSIFICATION_V1", "", f"SVHN source: `{svhn_zip}`", f"Printed source: `{printed}`",
                  f"Printed YAML class mapping: `{mapping}`", "SVHN MATLAB 7.3 HDF5 references decoded with h5py; label 10 maps to digit 0.",
                  "Printed YOLO boxes converted from normalized center coordinates to pixel boxes.",
                  f"Source images: SVHN { {s: len(m) for s,m in members.items()} }; Printed {printed_counts}.",
                  f"Total crops: {len(rows)}. Splits: {dict(counts)}. Sources: {dict(source_counts)}.",
                  f"Class counts: {dict(sorted(classes.items()))}. Max/min nonempty class ratio: {max(valid_counts)/min(valid_counts):.2f}.",
                  f"Source image resolution distribution: {resolutions.most_common(20)}.",
                  f"Crop resolution distribution: {crop_resolutions.most_common(20)}.",
                  f"Smallest crop: {min((int(r['crop_width'])*int(r['crop_height']), r['crop_file']) for r in rows) if rows else None}.",
                  f"Largest crop: {max((int(r['crop_width'])*int(r['crop_height']), r['crop_file']) for r in rows) if rows else None}.",
                  f"Multi-digit source images: {mult_digit_images}.", f"Rejected crops: {sum(rejected.values())}; reasons: {dict(rejected)}.",
                  f"Exact duplicate findings by original/new split: {dict(duplicates)}. Cross-split copies are excluded by test > val > train priority.",
                  f"Missing files: {missing[:20]} (total {len(missing)}). Annotation problems: {annotation_issues[:20]}.",
                  "Near-duplicate perceptual hashing was not performed. Training has not started."]
        (OUTPUT / "dataset_report.md").write_text("\n\n".join(report)+"\n", encoding="utf-8")
        validate(OUTPUT, rows)
        print(f"DATASET VALIDATION PASSED: {OUTPUT} ({len(rows)} crops)", flush=True)


def validate(output, rows):
    from PIL import Image
    paths = set()
    groups = {}
    for row in rows:
        path = output / row["crop_file"]
        if not path.is_file() or row["crop_file"] in paths:
            raise AssertionError(f"Missing/duplicate metadata path: {path}")
        paths.add(row["crop_file"])
        key = (row["source_dataset"], row["source_split"], row["source_image"])
        if key in groups and groups[key] != row["unified_split"]:
            raise AssertionError(f"Source image crosses splits: {key}")
        groups[key] = row["unified_split"]
        if int(row["digit"]) not in range(10) or path.parent.name != row["digit"]:
            raise AssertionError(f"Invalid class: {path}")
        with Image.open(path) as image:
            image.verify()
    for split in ("train", "val", "test"):
        if sorted(p.name for p in (output / split).iterdir()) != [str(i) for i in range(10)]:
            raise AssertionError(f"Invalid class folders: {split}")
        for folder in (output / split).iterdir():
            for file in folder.iterdir():
                if str(file.relative_to(output)).replace("\\", "/") not in paths:
                    raise AssertionError(f"Unindexed crop: {file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--parse-mat", nargs=2, type=Path)
    args = parser.parse_args()
    if args.parse_mat:
        parse_mat(*args.parse_mat)
    else:
        main()
