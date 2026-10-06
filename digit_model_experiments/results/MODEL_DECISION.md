# Digit recognition model decision — 2026-10-04

Source: saved outputs in `2026-10-04_kaggle_full_comparison.ipynb`, especially the final report cell. This is a completed Kaggle run, not a local retraining.

| Model | Validation macro F1 | Overall test accuracy | Printed validation accuracy (39) | Printed test accuracy (27) | Kaggle CPU latency, batch 1 | Checkpoint size |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Scratch CNN | 0.9448 | 0.9616 | 0.7179 | 0.8889 | 5.48 ms | 0.53 MB |
| MobileNetV3-Small | 0.9569 | 0.9705 | 0.8205 | 0.9630 | 7.80 ms | 6.24 MB |
| ResNet18 | **0.9637** | **0.9753** | **0.8718** | 0.9259 | 14.79 ms | 44.80 MB |

**Recommended checkpoint for the first robot-camera integration:** `digit_model_experiments/mobilenetv3_small/best.pt` from the Kaggle run's `/kaggle/working/digit_model_experiments/` output. The notebook explicitly names `MOBILENETV3_SMALL` as its provisional candidate. Its rule admits models within 0.01 validation macro F1 of the best, then ranks CPU latency (50%), Printed validation accuracy (25%), and a selected validation confusion rate (25%). MobileNet qualifies (0.0068 behind ResNet18) and is roughly 1.9 times faster on the measured Kaggle CPU with a checkpoint roughly one seventh the size.

**ResNet18 is the accuracy leader**, so download `digit_model_experiments/resnet18/best.pt` as a comparison checkpoint if storage allows. It is not the notebook's provisional deployment choice. If real robot-camera tests show ResNet18 clearly more accurate and its measured host latency is acceptable, revisit the choice.

`best.pt` is the fine-tuned classifier checkpoint. The torchvision `.pth` files downloaded during training are ImageNet initialization weights and are not the trained digit classifier. `last.pt` is the final epoch checkpoint, not the validation-selected checkpoint.

## Retrieval and validation

1. In the completed Kaggle notebook, open the **Output** tab and download the output files or the whole output ZIP. Find `digit_model_experiments/mobilenetv3_small/best.pt` inside it.
2. Both checkpoints are now saved locally under `digit_model_experiments/checkpoints/mobilenetv3_small/best.pt` and `digit_model_experiments/checkpoints/resnet18/best.pt`.
3. Keep the saved notebook and, preferably, the Kaggle `model_comparison.csv`, `final_model_report.md`, and per-model `metrics/` next to the run for audit. These were not present in this local workspace when this note was written; the numbers above are copied from notebook outputs.
4. Verify the checkpoint identifies `MOBILENETV3_SMALL`, has `input_size=128`, and `smoke_test=False` before use. The model expects a crop converted to RGB, padded to a centered square with median border color, resized to 128×128, and normalized using ImageNet mean/std. The notebook defines that preprocessing and model architecture.

The dataset has 357,793 crops, mostly SVHN. Only 397 crops are Printed Digit, including 39 validation and 27 test crops. The prior exhaustive image validation did not finish. The public-data result does not establish performance on white-paper digits attached to cubes. Collect and label real robot-camera crops, then compare both models on those images before a final hardware decision. Kaggle latency excludes camera capture, cropping, and ROS.
