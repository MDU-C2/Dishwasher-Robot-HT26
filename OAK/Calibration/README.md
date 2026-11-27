# Hand–Eye Calibration, Empirical Compensation & Test Protocols (`Calibration/`)

Calibration computation scripts, 45-matrix selection/regression verification data, empirical X/Z compensation analysis, and the independent 100-image vision test protocol for the fixed-mount (eye-to-hand) **OAK-D Pro → ABB YuMi** setup.

> The original September 2026 16-point ArUco sampling campaign and early SVD preview remain preserved unchanged in [`../Vision/`](../Vision/README.md).

---

## Contents

### 1. [`HTv3_20261005/`](HTv3_20261005/)
- `compute_transform.py`: Kabsch SVD 3D rigid transform estimation from paired camera–robot coordinates with residual & leave-one-out diagnostics.
- `paired_points.csv`, `paired_points.json`: Paired camera and robot calibration coordinates (mm).
- `transform.json`, `transform_camera_to_robot.json`, `T_camera_to_robot.txt`: Exported 4×4 homogeneous transforms.

### 2. [`Matrix_Selection_and_Compensation_20261013/`](Matrix_Selection_and_Compensation_20261013/)
Independent audit and numerical re-computation of 45 candidate HT matrices across 9 physical cup-grasp verification points (with raw mobile photos excluded to keep the repository lightweight):
- **Selected rigid transform**: `raw / HT_New_Run2_Mean` (9-point planar error mean **31.21 mm**, median **23.60 mm**, max **57.73 mm** before linear compensation — 5.34 mm lower mean error than `HT_New_Combined_CleanedSensitivity`).
- **Empirical X compensation (active in `Vision_System/main_tray1125.py`)**:
  $$X_{\text{robot}} = 1.401822 \cdot X' - 198.521$$
  Addresses the systematic depth-dependent X drift ($\Delta X = -0.2887 \cdot X_{\text{ctrl}} + 142.61$, $R^2 = 0.9825$).
- **Empirical Z compensation**:
  $$Z_{\text{robot}} = -0.024556 \cdot Z' + 53.397$$
  Present in `1125`; note that slope $\approx 0$ maps Z into a narrow ~51–54 mm table-height band, and YuMi RAPID `FetchMug` / `pickupSequence` also applies fixed grasping height overrides (`55` / `35` mm).
- **Files**: `ht45_selection.csv`, `controller_readings.csv`, `finalists_xy.csv`, `finalists_z_comparison.csv`, `finalists_z_analysis.json`, `z_compensation_validation.{csv,json}`, `verification.json`, `finalist_z_compensation.py` (standalone zero-dependency X-to-Z compensation module), `ht4-compensated-preview.png`, and `verification_audit_record.md` (detailed numerical audit record).

> **Calibration Safety Constraint**: Do **not** alter `tGripper` TCP (`[0, 0, 145]`) or manually add an extra ArUco-to-flange offset on top of `HT_New_Run2_Mean` (already absorbed in translation vector `t`).

### 3. [`Independent_Test_Protocol/`](Independent_Test_Protocol/)
Frozen capture and annotation specification for the **100-image independent evaluation dataset** (publicly hosted on Roboflow Universe under `CC BY 4.0`: https://universe.roboflow.com/sjp-gcfju/test-5pahh) used in [`../Trained_Models/reports/eval_100_oak/`](../Trained_Models/reports/eval_100_oak/):
- **Scene composition (100 images across 40×40 cm workspace, 30–70 cm depth)**:
  - `54` single-cup (3×3 workspace grid × 6 orientations)
  - `10` multi-cup same-orientation (no arm) + `12` multi-cup mixed-orientation (no arm)
  - `8` multi-cup same + YuMi arm + `8` multi-cup mixed + YuMi arm
  - `8` pure negative scenes (`5` arm-only + `3` empty table)
- **Files**: `capture_rgb.py`, `planned-scenes.jsonl`, `model-register.json`, `annotation-record.template.json`, `session.template.json`, `verification.json`, `capture_and_annotation_protocol.txt`, `capture_checklist.txt`.
