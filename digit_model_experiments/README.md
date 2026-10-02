# Kaggle digit-classifier training

Open [digit_classifier_comparison.ipynb](digit_classifier_comparison.ipynb) in Kaggle and attach the existing `DIGIT_CLASSIFICATION_V1` dataset. The notebook accepts either the dataset folder or a ZIP containing that folder. It does not build crops or change the source data.

## Run order

1. Enable a GPU. Enable Internet for the official ImageNet weights used by MobileNetV3-Small and ResNet18, or provide a Kaggle environment where the weights are cached. The notebook stops if weights are unavailable.
2. Run with `SMOKE_TEST=True`. This uses a seeded, stratified 10% of train, keeps validation intact, saves checkpoints, and does not evaluate test.
3. For full training, set `SMOKE_TEST=False`. The default `MODELS_TO_RUN` trains all three models and then evaluates test. If the session is too short, set `MODELS_TO_RUN` to one model and `RUN_FINAL_EVALUATION=False` for each training session. Save each run's `digit_model_experiments` output as a Kaggle Dataset.
4. For a separate final comparison session, attach those three output datasets, set `MODELS_TO_RUN=[]`, set `EXTERNAL_CHECKPOINT_ROOTS` to the three attached `digit_model_experiments` directories, and set `RUN_FINAL_EVALUATION=True`. The notebook rejects smoke checkpoints for this comparison.

The final output is under `/kaggle/working/digit_model_experiments`. It includes `best.pt` and `last.pt` for each model, preflight files, source-specific metrics with sample counts, predictions, confusion matrices, error galleries, latency, `model_comparison.csv`, and `final_model_report.md`.

The attached dataset has 357,793 metadata rows: 325,090 train, 6,950 validation, and 25,753 test. Only 27 test crops are from Printed Digit Detection. The earlier complete image-read validation was stopped at the user's request. The notebook performs a lightweight preflight and flags suspect crops without deleting them. No model has been trained locally.

The GitHub repository contains the preparation code, this notebook, and lightweight dataset summaries. It does not contain the 357,793 crop PNGs or the 64 MB `metadata.csv`. For Kaggle, upload the **complete** `DIGIT_CLASSIFICATION_V1` folder, including `train/`, `val/`, `test/`, and `metadata.csv`. An upload containing only the manifest and reports will fail preflight. A ZIP of the complete folder is supported by the notebook.
