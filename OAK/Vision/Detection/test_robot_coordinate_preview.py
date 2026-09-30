"""Offline tests: no camera, DepthAI, OpenCV or robot connection required."""
import json
import math
from types import SimpleNamespace
import unittest

from robot_coordinate_preview import MATRIX_PATH, load_matrix, target_rows, transform_xyz


class CoordinatePreviewTests(unittest.TestCase):
    def test_recorded_campaign_residuals(self):
        data = json.loads(MATRIX_PATH.read_text(encoding='utf-8'))
        matrix = load_matrix()
        for sample in data['samples']:
            predicted = transform_xyz(matrix, sample['camera_xyz_mm'])
            error = math.dist(predicted, sample['robot_xyz_mm'])
            self.assertAlmostEqual(error, sample['fit_error_mm'], places=8)

    def test_multiple_targets_invalid_depth_and_disappearance(self):
        def detection(x, y, z, label):
            return SimpleNamespace(spatialCoordinates=SimpleNamespace(x=x, y=y, z=z), label=label)
        matrix = [[1, 0, 0, 10], [0, 1, 0, -20], [0, 0, 1, 30], [0, 0, 0, 1]]
        detections = [detection(1, 2, 100, 0), detection(9, 8, 200, 1),
                      detection(0, 0, 0, 0), detection(float('nan'), 0, 100, 9)]
        rows = target_rows(detections, ['upright', 'sideways'], matrix)
        self.assertEqual([row['number'] for row in rows], [1, 2, 3, 4])
        self.assertEqual(rows[0]['robot_xyz_estimate_mm'], (11, -18, 130))
        self.assertEqual(rows[1]['robot_xyz_estimate_mm'], (19, -12, 230))
        self.assertEqual(rows[1]['label'], 'sideways')
        self.assertIsNone(rows[2]['robot_xyz_estimate_mm'])
        self.assertIsNone(rows[3]['camera_xyz_mm'])
        self.assertEqual(rows[3]['label'], '9')
        self.assertEqual(target_rows([], ['upright'], matrix), [])


if __name__ == '__main__':
    unittest.main()
