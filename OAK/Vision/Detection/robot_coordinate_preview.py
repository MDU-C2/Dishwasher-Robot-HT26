"""Offline-only coordinate preview using the approximate 16-point fit."""
import json
import math
from pathlib import Path

MATRIX_PATH = Path(__file__).resolve().parent / 'calibration' / 'ht_diagnostic_preview.json'


def load_matrix(path=MATRIX_PATH):
    matrix = json.loads(Path(path).read_text(encoding='utf-8'))['T_diagnostic']
    if (len(matrix) != 4 or any(len(row) != 4 for row in matrix)
            or not all(math.isfinite(v) for row in matrix for v in row)
            or matrix[3] != [0, 0, 0, 1]):
        raise ValueError('Invalid 4x4 homogeneous preview matrix')
    return matrix


def transform_xyz(matrix, xyz):
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz) or xyz[2] <= 0:
        return None
    return tuple(sum(matrix[i][j] * xyz[j] for j in range(3)) + matrix[i][3]
                 for i in range(3))


def target_rows(detections, labels, matrix):
    """Rebuild every frame; IDs are frame-local, not tracking identities.

    Use spatialCoordinates directly, exactly as the magenta projection does.
    Do not sample depth again at its projected pixel or apply another 80 mm offset.
    """
    rows = []
    for number, det in enumerate(detections, 1):
        coords = det.spatialCoordinates
        xyz = (coords.x, coords.y, coords.z)
        robot = transform_xyz(matrix, xyz)
        rows.append({'number': number,
                     'label': labels[det.label] if 0 <= det.label < len(labels) else str(det.label),
                     'camera_xyz_mm': xyz if robot is not None else None,
                     'robot_xyz_estimate_mm': robot})
    return rows
