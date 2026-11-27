# Nine-Point Cup & 45-Matrix Verification Audit Record

Audit date: 2026-10-13. Scope: X and Y planar accuracy only; error is defined as matrix-predicted robot coordinate minus controller reading. Original records reside in `<local_calibration_archive>/outputs`. No source files were modified, and neither the camera nor the physical robot was operated during this audit.

## Conclusions

The main numerical conclusions of the handoff summary hold. The regression direction for the 1.06 slope must be corrected, and recomputed CSV values must be distinguished from verbatim screen lines visible in photos.

## Verified Scope

- Three matrix files of 15 candidates each (45 matrices total); 18 comparison HEIC photos forming 9 pairs; 405 unique (pair, matrix group, matrix name) rows in the detailed CSV.
- All photos were inspected to confirm that the 9 controller XYZ positions and quaternions match the summary CSV. Displayed coordinates use WorkObject; Gripper tool definitions match project notes. Actual cup reference positions follow the confirmed jogged TCP readings.
- Matrix data on disk were parsed independently and multiplied against the 9 camera points (45 x 9 evaluations). All CSV robot XYZ values and error columns agree with recomputation within 0.05 mm (consistent with 0.1 mm display rounding).
- All `recommended` XYZ rows in the summary correspond to `cleaned / HT_New_Combined_CleanedSensitivity`, not `raw / HT_New_Run2_Mean`.
- Static inspection of `ht45_preview.py`: loads `v5s`; reads RGB, depth, and detections sequentially; displays RGB and 45 transformed outputs; contains no robot socket connection or motion commands. Static check only; live camera streaming was not re-tested here.

## Recomputed Metrics

- `HT_New_Run2_Mean` (`raw`, numerically equivalent to `reach`) 9-point planar error: mean **31.2083 mm**, median **23.5959 mm**, max **57.7308 mm**.
- `HT_New_Combined_CleanedSensitivity`: mean **36.5532 mm**, median **28.0173 mm**, max **63.8157 mm**. The former has a **5.3449 mm** lower mean error.
- `HT_New_Run2_Mean`: delta-X ranges from **-47.8275 to +56.9018 mm**; delta-Y ranges from **+0.8362 to +13.4539 mm**. This Y range applies specifically to this matrix and does not generalize across all 45 candidates (original finalist delta-Y: **+1.3365 to +20.1798 mm**).
- Across all 45 matrices, the minimum X error span is **104.7294 mm**. Without additional translation/linear compensation, the tightest simultaneous +/-n bound on X and Y across all 9 points remains **56.9018 mm**.
- Linear fit for `HT_New_Run2_Mean`: `delta_X = -0.2887334 * X_ctrl + 142.60785`, `R^2 = 0.982473`, zero-crossing at **493.9083 mm**, residual range **-8.2638 to +7.0954 mm**. Full-precision reporting should cite `R^2 = 0.98247` (approx. `0.9825`).
- The original candidate has a delta-X slope of `-0.3152293`, showing a similar near-to-far drift trend.

## Regression Direction Correction

- `X_ctrl ~= 1.0631169 * Z_cam - 182.2891`, residual **-7.0033 to +10.2475 mm**.
- `Z_cam ~= 0.9388844 * X_ctrl + 172.2944`, residual **-9.5741 to +6.6551 mm**.

Therefore, the earlier note stating "slope of camera Z against controller X is ~1.06" reversed the independent and dependent variables; it should state **"slope of controller X against camera Z is ~1.06"**. These 9 points do not support the claim that camera depth only reads ~70% of distance, nor do they independently certify absolute depth accuracy.

Camera Y against camera Z has slope `0.424258`. The X row of `HT_New_Run2_Mean` is:

`X_pred = -0.002597731 * X_cam - 0.387631747 * Y_cam + 0.921810653 * Z_cam - 84.047694`

Along the camera-Z direction of these 9 points, the three slope contributions are `+0.0005589`, `-0.1644559`, and `+0.9218107`, summing to `0.7579136`. Divided by the `X_ctrl` vs `Z_cam` slope (`1.0631169`), the ratio is `0.7129166` (~0.71). Direct regression of `X_pred` on `X_ctrl` gives `0.7112666`. This provides a geometric explanation under the current matrix rather than an independent validation of compensation.

## Photo Discrepancies & Precision Limits

1. `IMG_0634` header Z digit ambiguity: using `770.5` matches all 45 CSV matrix outputs with XYZ RMSE `0.0294 mm`, whereas `780.5` gives RMSE `5.7680 mm` (max diff `9.7274 mm`). The readable photo line for `raw / HT_New_Run2_Mean` (~`(594.8, 60.0, -55.0)`) supports retaining `770.5`.
2. `IMG_0636` exhibits screen differences beyond 0.1 mm rounding: e.g., `HT_New_Combined` displays X ~608.4 in the `raw` block vs X ~610.7 in the `reach` block, even though both matrices on disk are numerically equivalent. Photos cannot guarantee simultaneous freeze across all screen panels (possible refresh timing). Thus the CSV represents recomputed outputs from recorded camera XYZ and disk matrices, not a verbatim photo transcription.
3. Numerical equivalence between `raw` and `reach`: max element difference is `3.55e-15` for `HT_New_Run2_Mean` and `5.55e-17` for `HT_New_Combined` (equivalent at practical precision, not bit-identical float arrays).

## Evidence Status

File and numerical audits are complete. Live camera streaming was not re-tested, and no physical robot motion or new calibration/compensation trials were performed. Full 45-matrix rankings and fit statistics are stored in `verification.json`.
