"""Experimental Z correction AFTER the current finalist-specific X correction.

Input: robot XYZ in mm, with X already corrected by ht45_preview's
2026-10-13 finalist X fit, Y/Z still from that same HT matrix.
Output: a new XYZ tuple; X/Y unchanged, Z corrected for the observed near/far bias.

Fits use nine compara cup positions (controller Z 37.90..47.33 mm).
They describe that sampled surface; height response is not independently validated.
Changing the HT matrix or the preceding X correction requires refitting these values.
No hardware, socket, camera, or third-party dependencies.
"""
from math import isfinite

# delta_Z = Z_HT - Z_controller = slope * X_corrected + intercept_mm.
Z_FITS = {
    ('raw', 'HT_New_Run2_Mean'): (-0.7390093740232737, 375.98240729742537),
    ('raw', 'HT_Global'): (-0.7586320459624486, 386.30334510458675),
    ('cleaned', 'HT_Old_Run1_Mean_CleanedSensitivity'): (-0.7461443673053619, 377.01579512916874),
    ('cleaned', 'HT_New_Combined_CleanedSensitivity'): (-0.755151241319786, 379.5733471258256),
}


def compensate_robot_z(robot_xyz, group, matrix_name):
    """Subtract fitted Z error using ALREADY X-compensated XYZ; apply only once."""
    key = (group, matrix_name)
    if key not in Z_FITS:
        raise ValueError(f'No finalist Z compensation for {group} / {matrix_name}')
    try:
        x_corrected, y_ht, z_ht = (float(value) for value in robot_xyz)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('Expected three finite robot coordinates in mm') from exc
    if not all(isfinite(value) for value in (x_corrected, y_ht, z_ht)):
        raise ValueError('Expected three finite robot coordinates in mm')
    slope, intercept_mm = Z_FITS[key]
    predicted_z_error = slope * x_corrected + intercept_mm
    return x_corrected, y_ht, z_ht - predicted_z_error
