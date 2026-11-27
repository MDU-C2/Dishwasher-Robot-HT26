# Colab Notebook Evaluation Extract (Per-Class Metrics, Complexity & Test-Split Results)

All figures below are transcribed directly from the saved cell outputs of the 3 deployed Google Colab training notebooks in `Trained_Models/notebooks/`.

> **Scope & Validity Caveat**:
> - `2026_GreG-4` validation split has **26 images / 78 instances** (`Back: 8, Front: 8, left_side: 9, right_side: 10, upright: 11, upside_down: 32`).
> - `2026_GreG-4` test split has **22 images / 63 instances** (`Back: 9, Front: 5, left_side: 15, right_side: 4, upright: 17, upside_down: 13`).
> - Because the 22-image test split is small and was inspected during model iteration, treat these numbers as **development/ablation evidence**, and refer to `reports/eval_100_oak/` (the frozen 100-image on-device OAK-D Pro benchmark) for final evaluation claims.

---

## 1. Model Complexity (6-Class Fused Architecture Summary)

| Model ID | Architecture | Layers | Parameters | GFLOPs | OAK 5-SHAVE `.blob` Size | Training Notebook |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| `yolov5s_416` | **YOLOv5s** (6-cls, 416) | 68 | 7,026,307 (7.03M) | 15.8 | 13.50 MB | `train_yolov5s_416.ipynb` |
| `yolo11n_416` | **YOLO11n** (6-cls, 416) | 101 | 2,598,266 (2.60M) | 6.4 | 5.51 MB | `train_yolo11n_416.ipynb` |
| `yolo11n_640` | **YOLO11n** (6-cls, 640) | 101 | 2,598,266 (2.60M) | 6.4 | 5.55 MB | `train_yolo11n_640.ipynb` |

- YOLO11n uses **2.70× fewer parameters** and **2.47× fewer GFLOPs** than YOLOv5s, with a **2.45× smaller** RVC2 FP16 `.blob` footprint on OAK-D Pro.

---

## 2. Controlled `2026_GreG-4` Validation Comparison (26 Images, 78 Instances, Full 120 Epochs, `batch=8, patience=0`)

| Model ID | Arch | Input | Overall P | Overall R | Overall mAP@50 | Overall mAP@50:95 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| `yolov5s_416` | YOLOv5s | 416×416 | 0.915 | 0.899 | 0.934 | 0.775 |
| `yolo11n_416` | YOLO11n | 416×416 | **0.931** | **0.937** | 0.935 | 0.779 |
| `yolo11n_640` | YOLO11n | 640×640 | 0.920 | 0.899 | **0.941** | **0.796** |

### Per-Class Validation Breakdown (`2026_GreG-4`, 78 Instances)

| Class (Inst.) | `yolov5s_416` (P / R / mAP50 / 50:95) | `yolo11n_416` (P / R / mAP50 / 50:95) | `yolo11n_640` (P / R / mAP50 / 50:95) |
| :--- | :---: | :---: | :---: |
| `Back` (8) | 0.927 / 1.000 / 0.995 / 0.763 | 1.000 / 0.989 / 0.995 / 0.779 | 0.990 / 0.875 / 0.971 / 0.781 |
| `Front` (8) | 1.000 / 0.564 / 0.883 / 0.762 | 1.000 / 0.766 / 0.875 / 0.764 | 1.000 / 0.649 / 0.875 / 0.773 |
| `left_side` (9) | 0.730 / 1.000 / 0.855 / 0.790 | 0.732 / 1.000 / 0.895 / 0.824 | 0.746 / 1.000 / 0.955 / 0.891 |
| `right_side` (10) | 0.963 / 0.900 / 0.931 / 0.730 | 0.961 / 0.900 / 0.895 / 0.729 | 0.882 / 0.900 / 0.885 / 0.731 |
| `upright` (11) | 0.916 / 0.993 / 0.972 / 0.769 | 0.900 / 1.000 / 0.988 / 0.791 | 0.913 / 1.000 / 0.972 / 0.765 |
| `upside_down` (32) | 0.956 / 0.938 / 0.968 / 0.836 | 0.994 / 0.969 / 0.965 / 0.785 | 0.989 / 0.969 / 0.989 / 0.833 |

**Key takeaway**: Under identical `2026_GreG-4` 120-epoch training at 416×416, **`yolo11n_416` outperforms `yolov5s_416` across all four overall metrics** (`+1.6%` P, `+3.8%` R, `+0.1%` mAP@50, `+0.4%` mAP@50:95) while using 2.7× fewer parameters. Raising input resolution to 640×640 (`yolo11n_640`) improves box localization (`mAP@50:95` `+1.7%`), especially on `left_side` (`0.891`), at the cost of lower `Front` recall (`0.649` vs `0.766`).

---

## 3. Early-Stopping Ablation on `2026_GreG-4` Test Split (22 Images, `yolo11n_416`)

| Run | Dataset | Epochs | Test P | Test R | Test mAP@50 | Test mAP@50:95 | Training Notebook |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| `yolo11n_416` (early-stop run) | `2026_GreG-4` (63 inst.) | 37 (ES) | 0.908 | 0.919 | 0.951 | 0.788 | `train_yolo11n_416.ipynb` |
| `yolo11n_416` (deployed full) | `2026_GreG-4` (63 inst.) | 120 (Full) | **0.954** | **0.939** | **0.957** | **0.796** | `train_yolo11n_416.ipynb` |

**Key takeaway**: Removing early stopping (`37 ep` → `120 ep`) on `2026_GreG-4` improved 22-image test Precision by `+4.6%` (`0.908 → 0.954`), Recall by `+2.0%` (`0.919 → 0.939`), and mAP@50:95 by `+0.8%` (`0.788 → 0.796`).
