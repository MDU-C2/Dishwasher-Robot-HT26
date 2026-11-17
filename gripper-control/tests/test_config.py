import unittest

from robot_control.config import CONTINUOUS_SERVO_CHANNELS, SERVO_CHANNELS


class ServoConfigurationTests(unittest.TestCase):
    def test_configured_channels_are_zero_through_eight(self):
        self.assertEqual(list(SERVO_CHANNELS), list(range(9)))
        self.assertEqual(len(SERVO_CHANNELS), 9)

    def test_finger_servos_are_continuous_rotation(self):
        self.assertEqual(CONTINUOUS_SERVO_CHANNELS, (6, 7, 8))


if __name__ == "__main__":
    unittest.main()
