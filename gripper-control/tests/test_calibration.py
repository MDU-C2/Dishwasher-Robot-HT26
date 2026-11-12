import unittest

from robot_control.calibration import ServoCalibration


class ServoCalibrationTests(unittest.TestCase):
    def test_piecewise_interpolation_uses_independent_center(self):
        calibration = ServoCalibration(900, 1470, 2150)

        self.assertEqual(calibration.pulse_for_angle(0), 900)
        self.assertEqual(calibration.pulse_for_angle(45), 1185)
        self.assertEqual(calibration.pulse_for_angle(90), 1470)
        self.assertEqual(calibration.pulse_for_angle(135), 1810)
        self.assertEqual(calibration.pulse_for_angle(180), 2150)

    def test_validation_rejects_bad_order_and_bounds(self):
        self.assertIsNone(ServoCalibration(900, 1500, 2200).validate())
        self.assertIsNotNone(ServoCalibration(1500, 1400, 2200).validate())
        self.assertIsNotNone(ServoCalibration(500, 1500, 2200).validate())
        self.assertIsNotNone(ServoCalibration(900, 1500, 2500).validate())

    def test_angle_is_limited_to_gui_range(self):
        calibration = ServoCalibration()

        with self.assertRaises(ValueError):
            calibration.pulse_for_angle(-1)
        with self.assertRaises(ValueError):
            calibration.pulse_for_angle(181)


if __name__ == "__main__":
    unittest.main()
