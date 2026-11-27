# OAK-D Pro 3D + 2D Integrated Vision System (`Vision_System/`, 1125 Release)

Self-contained runtime package for the **ABB YuMi Dishwasher Robot** integrating **5 selectable 3D cup-orientation YOLO `.blob` models** on OAK-D Pro with **2D overhead tray slot detection (YOLO + ArUco ID 24)** and **YuMi TCP/IP communication**.

> **Zero-Overwrite Packaging**: This directory is self-contained under `OAK/Vision_System/` so it can be run and tested independently without modifying `Python/main_tray_vs/`, `Python/3D_Vision/`, or `Python/A/`.

---

## Key Improvements Over Earlier `main_tray` Snapshots

1. **Unified `stretched_1080p` Geometry (`camera_setup.py`)**: All 5 models (`416×416` and `640×640`) share consistent 1080p stretched RGB preview, matching stretched `StereoDepth` output, and `keepAspectRatio=False` intrinsics so the projected `[DOT]` spatial marker stays aligned inside each cup bounding box.
2. **Mandatory DepthAI 2.30 Stereo Configuration**: Explicitly sets `stereo.setSubpixel(False)` and `stereo.setExtendedDisparity(True)` for stable close-range workspace depth (30–70 cm).
3. **5-Model Runtime Selector (`1–5`, default `4`)** using unified canonical model IDs:
   - `1`: `marc_v5s_640` — MARC inherited YOLOv5s 640×640 (8 cls, 5-SHAVE)
   - `2`: `ft_marc_v5s_416` — Fine-tuned MARC YOLOv5s 416×416 (7 cls, 5-SHAVE)
   - `3`: `yolov5s_416` — Custom `2026_GreG-4` YOLOv5s 416×416 (6 cls, 5-SHAVE)
   - `4`: **`yolo11n_416` (Default)** — Custom `2026_GreG-4` YOLO11n 416×416 (6 cls, 5-SHAVE)
   - `5`: `yolo11n_640` — Custom `2026_GreG-4` YOLO11n 640×640 (6 cls, 5-SHAVE)
4. **Non-Cup Class Filtering**: Legacy `Gripper` and `handle` detections from 7/8-class models are strictly excluded via `NON_CUP_LABELS` before target selection.
5. **Hand–Eye Transform & Filtering**: Uses `HT_New_Run2_Mean` with empirical X/Z compensation, pickup workspace bounds (`X <= 615 mm`, `Y >= -65 mm`), left-entry proximity priority (`[-302.0, -147.09, 407.35] mm`), hysteresis (40 mm hold / 30 mm switch margin), and 5-frame anti-jitter median.
6. **Standalone Vision-Only Preview**: Includes `main_tray1125_vision_only.py` for live OAK-D Pro testing with hotkeys `1–5` to switch models, `s` snapshot, and `r` reset — no RobotStudio or 2D tray camera required.

---

## Files & Ownership Attribution

| File(s) | Role | Origin / Attribution |
| :--- | :--- | :--- |
| `main_tray1125.py` | Full 3D + 2D + YuMi TCP/IP runtime entry | Zhixiong Liu (integrated with Zebastian/Arafat interfaces) |
| `main_tray1125_vision_only.py` | Standalone OAK-D Pro 3D vision tester (no RobotStudio) | Zhixiong Liu |
| `camera_setup.py` | DepthAI 2.30 pipeline, 5-model registry, unified geometry | Zhixiong Liu |
| `requirements.txt` | Pinned Python dependencies (`depthai==2.30.0.0`) | Zhixiong Liu |
| `models/<model_id>/*` | 5 compiled 5-SHAVE FP16 `.blob` files + DepthAI JSONs | Models 3–5 trained/compiled by Zhixiong; #1 MARC-HT25; #2 fine-tuned baseline |
| `updated_communication.py`, `test_protocol.py` | YuMi TCP/IP client (`192.168.125.1:1025`) & logging | Adapted from MARC-HT25 / Zebastian Thorsén |
| `tray_camera_yolo.py`, `aruco_tray_tracker.py`, `calibrate_aruco.py`, `calibrate_slots.py`, `best.pt`, `slot_config.json`, `aruco_reference.json`, `aruco_marker_id24.png`, `test_images/empty_tray.jpg` | 2D overhead tray slot YOLO & ArUco ID 24 tracking | Bundled from Arafat Sulaiman (`test2Division:Python/A/`) for standalone execution |

---

## Setup & Usage

Install dependencies in a Python 3.11 virtual environment:

```powershell
pip install -r requirements.txt
```

> **Note**: If `ultralytics` is not installed, `tray_camera_yolo.py` automatically falls back to OpenCV image-diff against `test_images/empty_tray.jpg`.

### 1. Vision-Only Live Preview (OAK-D Pro Only, No YuMi)
```powershell
python main_tray1125_vision_only.py
```
Hotkeys: `1`–`5` switch active 3D `.blob` model, `s` save snapshot, `r` reset target filter, `q` quit.

### 2. Full YuMi Runtime (OAK-D Pro + 2D Tray Camera + ABB YuMi at `192.168.125.1:1025`)
```powershell
python main_tray1125.py
```

---

## Known Boundaries & Next Improvements

- During robot `busy == True` cycles with ≥ 2 cups in view, a local spatial tracking gate around `active_target_xyz` is planned as a follow-up refinement so multi-cup backoff retries never jump to neighboring cups.
