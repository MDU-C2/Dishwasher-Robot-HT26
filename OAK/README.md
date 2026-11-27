# OAK-D Pro 3D Vision, Calibration & YOLO Benchmark (`OAK/`)

3D vision pipeline, hand–eye calibration, custom YOLO model training/conversion, and 100-image on-device OAK-D Pro evaluation developed by **Zhixiong Liu** for the **ABB YuMi (IRB 14000) Dishwasher Loading & Unloading Robot** project (MDU DVA490 / DVA474, HT2026).

> **Branch & Isolation Note**: All contributions in this directory are strictly self-contained under `OAK/` on branch `HT26/vision-integration`. Shared and teammate-owned folders (`Python/main_tray_vs/`, `Python/3D_Vision/`, `Python/A/`, `RAPID/`, `gripper-control/`) are intentionally left untouched to prevent merge conflicts or accidental overwrites.

<p align="center">
  <a href="#directory-overview">Directory Overview</a> •
  <a href="#system-pipeline">System Pipeline</a> •
  <a href="#public-datasets">Public Datasets</a> •
  <a href="#benchmark-highlights">Benchmark Highlights</a> •
  <a href="#requirements">Requirements</a> •
  <a href="#quick-start">Quick Start</a> •
  <a href="#authorship--integration-boundaries">Authorship & Boundaries</a>
</p>

---

## Directory Overview

Inspired by and refined from [MDU-C2/MARC-HT25](https://github.com/MDU-C2/MARC-HT25) (with virtual environments, caches, duplicate weights, and raw image dumps excluded):

| Folder | Role | Key Contents |
| :--- | :--- | :--- |
| [`Vision_System/`](Vision_System/README.md) | **Latest Runtime (1125)** | Unified 5-model 3D cup detection + 2D YOLO/ArUco tray integration (`main_tray1125.py`), standalone vision-only preview (`main_tray1125_vision_only.py`), `camera_setup.py`, `requirements.txt`, and the 5 unified 5-SHAVE `.blob` models. |
| [`Trained_Models/`](Trained_Models/README.md) | **Model & Paper Hub** | Unified 5-model SHA256 registry (`registry.json`), catalog CSV, the 3 deployed Colab training notebooks, 8 conversion/evaluation/plotting scripts, dataset capture utility (`data_capture/`), training curves, and 100-image OAK-D Pro benchmark reports & LaTeX tables. |
| [`Calibration/`](Calibration/README.md) | **Hand–Eye & Protocols** | SVD rigid transform fitting (`HTv3_20261005/`), 45-matrix comparison & empirical X/Z compensation verification (`Matrix_Selection_and_Compensation_20261013/`), and the frozen 100-image independent evaluation protocol (`Independent_Test_Protocol/`). |
| [`Vision/`](Vision/README.md) | **Initial Release (Archive)** | Original Sep–Oct 2026 ArUco sampling, 16-point calibration, and early YOLOv5 preview package preserved unchanged from PR #2. |

---

## System Pipeline

```text
OAK-D Pro (1080p RGB + 400p StereoDepth, ExtendedDisparity=True, Subpixel=False)
  → On-chip YoloSpatialDetectionNetwork (5-SHAVE OpenVINO 2022.1 FP16 .blob)
  → Camera-frame 3D XYZ (mm) + 6-class cup orientation
  → Rigid Hand–Eye Transform (HT_New_Run2_Mean) + Empirical X/Z Compensation
  → Target Selection, Hysteresis & 5-Frame Anti-Jitter Median
  → Raw TCP/IP Socket (192.168.125.1:1025) ↔ ABB YuMi RAPID (LeftArmMain / RightArmMain)
```

---

## Public Datasets

Both the training dataset and the frozen 100-image independent evaluation set are publicly hosted on **Roboflow Universe** (`CC BY 4.0`, canonical 6 cup-orientation classes: `Back`, `Front`, `left_side`, `right_side`, `upright`, `upside_down`):

- **Training Dataset (`2026_GreG`, Version 4 used for `yolov5s_416`, `yolo11n_416`, `yolo11n_640`)**: https://universe.roboflow.com/sjp-gcfju/2026_greg
- **100-Image Independent Test Set (`TEST / 100pics`, Version 1)**: https://universe.roboflow.com/sjp-gcfju/test-5pahh

---

## Benchmark Highlights

Evaluated directly on the **OAK-D Pro RVC2 VPU** (5-SHAVE FP16 `.blob`, `preprocess=stretch` matching runtime deployment, `conf=0.25`, `IoU=0.50`) over the frozen **100-image independent test set** (54 single-cup, 38 multi-cup, 8 pure-negative/arm scenes, unified 6 classes):

| Menu | Unified Model ID | Report Name | Arch / Cls / Input | mAP@50 | mAP@50:95 | Single (54) mAP@50 | Multi (38) mAP@50 | On-Chip Latency (ms) |
| :-: | :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: |
| 1 | `marc_v5s_640` | `MARC-v5s-640` | YOLOv5s / 8 / 640 | 0.5833 | 0.3995 | 0.6742 | 0.5655 | 259.0 |
| 2 | `ft_marc_v5s_416` | `FT-MARC-v5s-416` | YOLOv5s / 7 / 416 | 0.6672 | 0.4688 | 0.7506 | 0.6529 | 95.5 |
| 3 | `yolov5s_416` | `YOLOv5s-416` | YOLOv5s / 6 / 416 | 0.9696 | 0.8500 | 1.0000 | 0.9625 | 94.3 |
| **4** | **`yolo11n_416`** | **`YOLO11n-416` (Default)** | **YOLO11n / 6 / 416** | **0.9609** | **0.8574** | **1.0000** | **0.9498** | **75.6** |
| 5 | `yolo11n_640` | `YOLO11n-640` | YOLO11n / 6 / 640 | 0.9777 | 0.8794 | 1.0000 | 0.9759 | 155.0 |

Full `stretch` (`conf=0.25` / `0.50`) and `letterbox` (`conf=0.25`) tables, confusion matrices, JSON metrics, and JSONL predictions are stored in [`Trained_Models/reports/eval_100_oak/`](Trained_Models/reports/eval_100_oak/).

---

## Requirements

- **Python**: 3.11 (tested on Windows)
- **DepthAI**: **Pinned to `depthai==2.30.0.0`** (do not upgrade to DepthAI v3)
- **Core packages**: `numpy`, `opencv-python` (`ultralytics` optional for 2D tray YOLO; otherwise falls back to OpenCV diff)
- **Mandatory StereoDepth setting**: Since 2026-11-20, all DepthAI pipelines explicitly set `stereo.setSubpixel(False)` and `stereo.setExtendedDisparity(True)`.

---

## Quick Start

```powershell
# 1. Install pinned dependencies
cd OAK\Vision_System
pip install -r requirements.txt

# 2. Standalone 3D OAK vision preview & model switcher (hotkeys 1-5, no robot required)
python main_tray1125_vision_only.py

# 3. Full 3D cup + 2D tray + YuMi TCP/IP runtime
python main_tray1125.py
```

---

## Authorship & Integration Boundaries

- **Zhixiong Liu (this `OAK/` directory)**: OAK-D Pro 3D stereo/spatial pipeline, unified `stretched_1080p` geometry fix, 5-model runtime integration, `2026_GreG-4` YOLO training/conversion/100-image on-device benchmark, hand–eye calibration (`HT_New_Run2_Mean`) & empirical compensation verification, and vision-only live preview tool.
- **Zebastian Thorsén**: ABB YuMi RAPID motion planning (`RAPID/`), TCP/IP server/client protocol (`updated_communication.py` interface), and team runtime integration in `Python/main_tray_vs/`.
- **Arafat Sulaiman**: 2D overhead tray slot YOLO & ArUco tracking (`tray_camera_yolo.py`, `aruco_tray_tracker.py`, `best.pt`, maintained in `test2Division:Python/A/`). Copies are bundled inside `OAK/Vision_System/` solely so this release runs standalone without modifying teammate branches.
- **Simon Jansson Perho & Daniel Rytenberg**: Custom gripper hardware/firmware (`gripper-control/`), Lidar & slip detection.
