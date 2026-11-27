# 3D Cup-Orientation YOLO Models — Consolidated Lineage & Training Metrics

- **Canonical 6-class task**: `Back, Front, left_side, right_side, upright, upside_down`
- **Paper core lineup (`paper5`)**: 5 models (`MARC-v5s-640`, `FT-MARC-v5s-416`, `YOLOv5s-416`, `YOLO11n-416`, `YOLO11n-640`)
- **Archived historical models**: 0 models (retained for traceability/ablation reference)
- **Caution for paper writing**: Validation metrics below come from **each run's own validation split** (`complete_v3` vs `2026_GreG-2/3/4`). Use `eval_oak_blob.py --models paper5` + `eval_metrics.py` on a single frozen independent test set for final thesis comparison.

## 1. Paper Core Lineup (`--models paper5`)

| Model ID | Group | Arch | Cls | Input | SHAVE | Dataset | Ep (done/plan) | Artifacts | Val P | Val R | Val mAP@50 | Val mAP@50:95 | Notes |
| :--- | :---: | :--- | :---: | :---: | :---: | :--- | :---: | :--- | ---: | ---: | ---: | ---: | :--- |
| `MARC-v5s-640` | marc | YOLOv5s | 8 | 640×640 | 5 | MARC legacy (undocumented) | -- | blob | -- | -- | -- | -- | Baseline inherited 8-class model (includes Gripper, handle); no .pt or training logs available locally. |
| `FT-MARC-v5s-416` | ft_marc | YOLOv5s | 7 | 416×416 | 5 | complete_v3 (aug_v1) | 150/150 | blob, csv | 0.9674 | 0.9522 | 0.9809 | 0.8710 | Fine-tuned MARC ablation matrix anchor with full .pt, results.csv, curves; also used as fine-tune base for G3 v5s. |
| `YOLOv5s-416` | custom | YOLOv5s | 6 | 416×416 | 5 | 2026_GreG-4 | 120/120 | blob, csv | 0.9010 | 0.9298 | 0.9280 | 0.7744 | Latest full 120-epoch YOLOv5s 416 trained on 2026_GreG-4 (from yolov5s.pt, patience=0, batch=8); directly pairs with yolo11n_416. |
| `YOLO11n-416` | custom | YOLO11n | 6 | 416×416 | 5 | 2026_GreG-4 | 120/120 | blob, csv | 0.9310 | 0.9372 | 0.9354 | 0.7782 | Round-3 full 120-epoch YOLO11n 416 on 2026_GreG-4 (patience=0, batch=8). Complete .pt/.onnx/.blob + Drive results.csv & test_evaluation. |
| `YOLO11n-640` | custom | YOLO11n | 6 | 640×640 | 5 | 2026_GreG-4 | 120/120 | blob, csv | 0.9197 | 0.8989 | 0.9413 | 0.7950 | 640x640 resolution control run on 2026_GreG-4 (120 ep, patience=0, batch=8); directly pairs with yolo11n_416. |

## 2. Archived Historical Models

| Model ID | Group | Arch | Cls | Input | SHAVE | Dataset | Ep (done/plan) | Artifacts | Val P | Val R | Val mAP@50 | Val mAP@50:95 | Notes |
| :--- | :---: | :--- | :---: | :---: | :---: | :--- | :---: | :--- | ---: | ---: | ---: | ---: | :--- |

## 3. Deployment Preprocessing & SHAVES Notes (for Method / Threats to Validity)

- **Class schema evolution**:
  - **MARC Baseline (`MARC-v5s-640`)**: 8 classes (`Back, Front, Gripper, handle, left_side, right_side, upright, upside_down`).
  - **Fine-Tuned MARC (`FT-MARC-v5s-416`)**: 7 classes (`handle` removed; `Gripper` kept).
  - **Custom 6-Class Models (`YOLOv5s-416`, `YOLO11n-416`, `YOLO11n-640`)**: 6 canonical cup-orientation classes (`Gripper` removed; robot-arm/joint background hard negatives added).
- **OAK-D Pro deployment uniformity**: All 5 `paper5` models are compiled to **5 SHAVES** (OpenVINO 2022.1 FP16), enabling fair on-device latency comparison.
