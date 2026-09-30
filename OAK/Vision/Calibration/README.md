# HT matrix estimation

This step follows Sampling: paired camera and robot coordinates -> one fitted HT matrix -> JSON report and TXT matrix. No camera or robot is connected.

## Files and requirements

- `estimate_transform.py`: estimate rotation R and translation t using SVD, assemble HT, and report fitting/leave-one-out errors.
- `requirements.txt`: NumPy 2.2.6. The existing sampling environment already includes it.

## Input

Keep this folder beside `Sampling`. The included session is `../Sampling/aruco_center_sessions/` and contains:
- `sample_0001/sample.json` through `sample_0016/sample.json`: camera XYZ in millimetres.
- `points.txt`: robot target definitions in millimetres.

This version expects exactly 16 samples with a unique ArUco ID 23. Pairing is sample 1 -> HOMEpos; samples 2-16 -> s1-s15. Confirm that order before running. Retain the other sampling files for traceability; they are not directly read by this script.

## Run

In an activated Python environment, open PowerShell in this folder:

```powershell
python .\estimate_transform.py "..\Sampling\aruco_center_sessions"
```

For a later run, replace the session path with its timestamped sampling folder.

## Outputs

Both files are written inside the supplied session directory; rerunning replaces files with these names:

- `camera_to_robot.json`: the 4x4 HT under `T_diagnostic`, paired coordinates, fitting errors, leave-one-out errors and limitations.
- `HT_matrix.txt`: the same 4x4 matrix as four rows of numbers, with explanatory comment lines. It is exported from the same calculation, not fitted again.

Existing `ht_diagnostic_preview.json` and `HT_16points_preview.txt` are historical results and are not overwritten.

## Reading HT

For column-vector coordinates, [robot_X, robot_Y, robot_Z, 1]^T = HT @ [camera_X, camera_Y, camera_Z, 1]^T.

- Upper-left 3x3 block: R, corresponding to `rotation_matrix` in main_tray.py.
- First three entries of the last column: t, corresponding to `translation_vector`.
- Last row: [0, 0, 0, 1].

main_tray.py currently embeds R and t in its code; it does not automatically read these output files. Applying HT to detected targets belongs to the downstream integration step.

## Current limitation

The original experiment paired marker-centre camera coordinates with recorded robot tool positions. Their physical offset and the tool/reference-frame definitions still need confirmation. Fitting residuals do not certify independent grasp accuracy; the JSON retains this limitation. This script does not perform robot motion.
