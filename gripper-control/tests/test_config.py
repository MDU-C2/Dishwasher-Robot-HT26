import unittest

from robot_control.config import CONTINUOUS_SERVO_CHANNELS, SERVO_CHANNELS


class ServoConfigurationTests(unittest.TestCase):
    def test_configured_channels_are_zero_through_seven(self):
        self.assertEqual(list(SERVO_CHANNELS), list(range(8)))
        self.assertEqual(len(SERVO_CHANNELS), 8)

    def test_finger_servos_are_continuous_rotation(self):
        self.assertEqual(CONTINUOUS_SERVO_CHANNELS, (5, 6, 7))


if __name__ == "__main__":
    unittest.main()
