# 3D Cup-Orientation YOLO Training, Conversion & Benchmark (`Trained_Models/`)

Consolidated training provenance, Luxonis OAK-D Pro `.blob` conversion scripts, the **3 deployed Google Colab training notebooks**, and **100-image on-device RVC2 evaluation results** for the 5 unified 3D cup-orientation YOLO models (`paper5`).

> **Storage Optimization**: Only metadata, training curves, scripts, the 3 deployed training notebooks, JSONL predictions, confusion matrices, and LaTeX tables are stored here (**~10.9 MB total**). The single deduplicated copy of the 5 compiled 5-SHAVE `.blob` weights resides in [`../Vision_System/models/`](../Vision_System/models/), referenced directly by `registry.json`.

---

## Unified 5-Model Lineup (`paper5`)

Canonical 6 cup-orientation classes: `0: Back`, `1: Front`, `2: left_side`, `3: right_side`, `4: upright`, `5: upside_down`. (Legacy `Gripper` and `handle` classes map to `null` during unified 6-class evaluation and are filtered out at runtime.)

| # | Model ID | Report Name | Arch | Cls | Input | Dataset / Epochs | Training Notebook | Benchmark Role |
| :-: | :--- | :--- | :--- | :-: | :-: | :--- | :--- | :--- |
| 1 | `marc_v5s_640` | `MARC-v5s-640` | YOLOv5s | 8 | 640 | MARC-HT25 inherited | — | Legacy baseline |
| 2 | `ft_marc_v5s_416` | `FT-MARC-v5s-416` | YOLOv5s | 7 | 416 | `complete_v3` (150 ep) | — | Fine-tuned transitional baseline |
| 3 | `yolov5s_416` | `YOLOv5s-416` | YOLOv5s | 6 | 416 | `2026_GreG-4` (120 ep) | `train_yolov5s_416.ipynb` | Dataset/6-class control vs #2; architecture control vs #4 |
| 4 | **`yolo11n_416`** | **`YOLO11n-416`** | **YOLO11n** | **6** | **416** | **`2026_GreG-4` (120 ep)** | **`train_yolo11n_416.ipynb`** | **Primary lightweight candidate (2.7× fewer params, 2.47× fewer GFLOPs vs v5s)** |
| 5 | `yolo11n_640` | `YOLO11n-640` | YOLO11n | 6 | 640 | `2026_GreG-4` (120 ep) | `train_yolo11n_640.ipynb` | Input-resolution vs edge-latency control vs #4 |

Models 3–5 share the exact same controlled `2026_GreG-4` split (`train=1932` including 39 pure arm-joint/empty background negatives, `val=26`, `test=22`; `batch=8, epochs=120, patience=0, seed=42`).

---

## Public Datasets (Roboflow Universe, `CC BY 4.0`)

- **Training & Development Dataset (`2026_GreG`, Version 4 used for Models 3–5)**: https://universe.roboflow.com/sjp-gcfju/2026_greg
- **Frozen 100-Image Independent Test Set (`TEST / 100pics`, Version 1)**: https://universe.roboflow.com/sjp-gcfju/test-5pahh

> To reproduce the Colab notebooks in `notebooks/`, set your own free Roboflow API key in the environment variable `ROBOFLOW_API_KEY` (no API keys are stored in this repository).

---

## 100-Image Independent OAK-D Pro Evaluation Summary

Measured via XLinkIn replay on **OAK-D Pro (DepthAI 2.30.0.0, 5-SHAVE FP16 `.blob`)** over the frozen 100-image test set (`54` single-cup 3×3 grid × 6 orientations, `38` multi-cup with/without robot arm, `8` pure negative arm/empty scenes):

### Deployment-Matched `stretch` Preprocessing (`conf=0.25`, `IoU=0.50`)

| Model | Precision | Recall | F1 | mAP@50 | mAP@50:95 | Single (54) mAP@50 | Multi (38) mAP@50 | Neg (8) FPIR | Latency (ms) |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `MARC-v5s-640` | 0.7613 | 0.4701 | 0.5296 | 0.5833 | 0.3995 | 0.6742 | 0.5655 | 0.000 | 259.0 |
| `FT-MARC-v5s-416` | 0.7952 | 0.6052 | 0.6744 | 0.6672 | 0.4688 | 0.7506 | 0.6529 | 0.000 | 95.5 |
| `YOLOv5s-416` | 0.9627 | 0.9615 | 0.9608 | 0.9696 | 0.8500 | 1.0000 | 0.9625 | 0.125 | 94.3 |
| **`YOLO11n-416`** | **0.9592** | **0.9530** | **0.9555** | **0.9609** | **0.8574** | **1.0000** | **0.9498** | **0.125** | **75.6** |
| `YOLO11n-640` | 0.9834 | 0.9853 | 0.9842 | 0.9777 | 0.8794 | 1.0000 | 0.9759 | 0.250 | 155.0 |

### Aspect-Preserving `letterbox` Preprocessing (`conf=0.25`, `IoU=0.50`)

| Model | Precision | Recall | F1 | mAP@50 | mAP@50:95 | Neg (8) FPIR | Latency (ms) |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `MARC-v5s-640` | 0.7163 | 0.3793 | 0.4303 | 0.4253 | 0.2711 | 0.000 | 257.4 |
| `FT-MARC-v5s-416` | 0.9040 | 0.7782 | 0.8298 | 0.8022 | 0.6250 | 0.125 | 94.6 |
| `YOLOv5s-416` | 0.9172 | 0.7829 | 0.8414 | 0.8191 | 0.6182 | 0.000 | 94.9 |
| `YOLO11n-416` | 0.9399 | 0.8109 | 0.8670 | 0.8680 | 0.7240 | 0.000 | 75.9 |
| `YOLO11n-640` | 0.9502 | 0.8573 | 0.9009 | 0.9228 | 0.7688 | 0.000 | 154.9 |

---

## Directory Structure

```text
Trained_Models/
├── README.md
├── registry.json                     # Unified 5-model SHA256 registry & 6-class mapping
├── model_catalog.csv                 # Summary catalog for spreadsheet / paper import
├── data_capture/                     # 1080p OAK-D Pro dataset capture tool (keys 1-6 by subset)
├── notebooks/                        # 3 deployed Colab training notebooks (2026_GreG-4, 120 ep)
│   ├── train_yolov5s_416.ipynb
│   ├── train_yolo11n_416.ipynb
│   └── train_yolo11n_640.ipynb
├── models/<paper5>/                  # meta.json, DepthAI configs, training curves & confusion matrices
├── scripts/
│   ├── verify_registry.py            # Registry SHA256 + DepthAI 5-SHAVE blob checker
│   ├── convert_to_oak_blob.py        # Luxonis API .pt -> 5-SHAVE RVC2 .blob converter
│   ├── eval_oak_blob.py              # OAK-D Pro XLinkIn replay evaluator (accuracy + on-chip latency)
│   ├── eval_pt_onnx.py               # PC/Colab FP32 PyTorch reference evaluator
│   ├── eval_metrics.py               # Pure NumPy+OpenCV 6-class mAP, CM, FPIR & LaTeX table generator
│   ├── plot_training_curves.py       # Regenerates SVG training curves for 2026_GreG-4
│   ├── generate_paper_summary.py     # Regenerates catalog CSV & paper LaTeX tables
│   └── render_raw_overlays.py        # Optional detection overlay visualizer
└── reports/
    ├── model_lineage_and_metrics.md  # Unified 5-model lineage & per-run validation summary
    ├── colab_notebook_eval_extract.md# Per-class Colab metrics, params & GFLOPs
    ├── registry_verification.json    # 5/5 SHA256 + DepthAI 5-SHAVE blob verification report
    ├── table_paper5_overview.tex     # Ready-to-include LaTeX booktabs table (5 models)
    ├── table_greg4_ablation.tex      # Controlled 2026_GreG-4 ablation LaTeX table
    ├── fig_greg4_ablation_curves.svg # Vector training curves (YOLOv5s-416 / YOLO11n-416 / YOLO11n-640)
    └── eval_100_oak/                 # 100-image OAK-D Pro evaluation outputs
        ├── eval_stretch_c025/        # Comparison CSV/MD/TEX, JSON metrics, 5 CM PNGs
        ├── eval_stretch_c050/        # High-threshold (conf=0.50) comparison
        ├── eval_letterbox_c025/      # Letterbox preprocessing comparison
        ├── preds/                    # Raw JSONL predictions (10 files, 1.84 MB)
        └── subset_breakdowns/        # Single-cup / multi-cup / pure-negative breakdown tables
```

---

## Methodological Note

- Per-run validation numbers in `reports/model_lineage_and_metrics.md` originate from different splits (`complete_v3` vs `2026_GreG-4`) and should **not** be directly compared across datasets.
- Cross-model accuracy claims must rely on the frozen 100-image evaluation in `reports/eval_100_oak/`.
- High 2D detection mAP does not by itself certify end-to-end physical grasp success; see [`../Calibration/README.md`](../Calibration/README.md) and [`../Vision_System/README.md`](../Vision_System/README.md) for 3D coordinate calibration and runtime integration boundaries.
