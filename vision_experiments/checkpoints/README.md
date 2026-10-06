# Camera-fine-tuned YOLO26 shape checkpoint

`robotic_E1_camera_finetune_best.pt` was extracted unchanged from `../archives/YOLO26_shape_detection_full_project.zip`, member `YOLO26 shape detection/camera_finetune_results V1/robotic_E1_camera_finetune_best.pt`.

The archive's `data.yaml` maps class 0 to `cube`, then `cylinder`, `diamond`, `pyramid`, and `sphere`. The archive's `validation_scores.json` reports development-validation precision 0.6009, recall 0.8825, mAP50 0.7633, and mAP50-95 0.5005 on 24 images at an operating confidence of 0.25. Its own notes say the images were from one capture session and this is not an independent estimate of future camera performance.

Checkpoint SHA-256: `865840096f956192ea5feaf280117964409358da13dfd44af97e7b2e749e7224`. The PyTorch ZIP container passed a CRC check. Loading the model and testing live camera predictions still requires the user's local Python environment and camera.
