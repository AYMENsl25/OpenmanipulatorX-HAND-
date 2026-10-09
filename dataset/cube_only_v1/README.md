# Numbered cube detection labels — version 1.0

## Label taxonomy

| ID | Label | Include | Exclude |
| --- | --- | --- | --- |
| 0 | `cube` | Every visible physical numbered or blank cube, whether white or green, including a cube partly outside the image. | Shadows, printed digits as separate objects, glare, hands, tools, tabletop marks. |

Draw one tight axis-aligned box around the **visible cube body**, including its
top and visible sides. Do not include the cast shadow. If cubes touch, give
each cube its own box. A cube partly outside the frame gets a box clipped to
the image border. For a severely occluded cube, include it only if enough of
its visible body remains to draw an unambiguous box. Confirm truly empty images
with zero boxes.

## Source and status

- Raw images: `../raw/numbered_cubes_day1/` (400 JPGs, 640×480). Raw images
  remain unchanged.
- `review/agnostic_proposals.jsonl`: 1,360 **model-generated proposals** from
  the existing five-shape checkpoint. Inference used confidence 0.10, image
  size 640, class-agnostic NMS at IoU 0.3. Every proposal was mapped to the
  single target class `cube`; the original predicted shape class and score are
  retained in JSON. This dataset contains only cubes, but the source model
  sometimes called them `pyramid` or `cylinder`.
- `unreviewed_proposals/labels/`: 400 YOLO text files generated from those
  proposals. **These are not ground truth and must not be used for training.**
- `reviewed_labels/` and `reviewed_manifest.csv`: created only when a person
  inspects each image and saves corrected boxes with the review tool.

Sample checks found both missed cubes and false positives, including a bright
reflection. The draft boxes can also be too large because they include a
shadow. Do not simply accept every proposal.

## Review

From the workspace root in PowerShell:

```powershell
& .\.venv\Scripts\python.exe .\vision_experiments\review_numbered_cube_boxes.py
```

The tool resumes at the first unreviewed image. To jump to a specific image,
use `--start 123` (one-based index). In the window:

- Left-drag to add a cube box; right-click inside a box to remove it.
- `U` undoes the last change; `R` restores the model proposals.
- `S` saves the corrected boxes and advances; `E` confirms an empty image.
- `K` skips without review; `P` returns to the previous image; `Q` quits.

Inspect the **whole image** before saving, including edges and cubes without
visible digits. Each green box should cover one cube body and no shadow.
`reviewed_manifest.csv` records which images were actually reviewed.

## Training split

This is one camera session with several near-duplicate bursts. Do not randomly
split its frames into training and validation; nearly identical arrangements
would leak across splits. After review, use this session for training and
capture a separate session for validation. The 28 earlier
`rotation_v2_yolo` frames should remain a held-out challenge set. Export to
the final YOLO `images/` and `labels/` train/val structure only after review.

## Read-only inspection after review

Use `vision_experiments/inspect_numbered_cube_labels.py` to display the saved
`reviewed_labels/` boxes on the raw images. Its default view filters to
`ai_reviewed` images 100–400. Press `N` or Space for next, `P` for previous,
and `Q` to quit. `--start 172` jumps to that original image number;
`--status all --start 1` also includes the 99 hand-reviewed images.
`--audit-only` checks that each image, label file, and manifest count agree,
and that YOLO coordinates are valid. These structural checks cannot determine
whether a box includes a shadow or misses a cube. Contact sheets of the 301
AI-reviewed images are in `reviewed_overlays/`; they are generated previews,
not training images.

## Changelog

- 1.0 (2026-10-06): Defined one-class cube boxes and created unreviewed
  model proposals for the first numbered-cube capture session.
- 1.1 (2026-10-07): Images 1-99 were reviewed by hand (`status=reviewed`).
  Images 100-400 were labelled by AI (Sonnet 5.5 draft, then a second-pass
  verification by Sonnet 5.5 or Opus 5.5) and are marked `status=ai_reviewed`
  in `reviewed_manifest.csv`. Boxes follow the same rule: cube body only,
  including sharp-edged dark side faces, excluding soft cast shadows. Dark
  side face vs. shadow is ambiguous in blurry frames, so spot-check
  `ai_reviewed` labels before training. Pre-export labels were backed up in
  `_backup_before_ai_review_*`.
