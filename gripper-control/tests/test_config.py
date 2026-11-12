import unittest

from robot_control.config import JOYSTICK_GROUPS, SERVO_CHANNELS


class ServoConfigurationTests(unittest.TestCase):
    def test_configured_channels_are_zero_through_eight(self):
        self.assertEqual(list(SERVO_CHANNELS), list(range(9)))
        self.assertEqual(len(SERVO_CHANNELS), 9)

    def test_joystick_groups_cover_remaining_and_fingers(self):
        self.assertEqual(
            [
                (mode["x_channels"], mode["y_channels"])
                for mode in JOYSTICK_GROUPS[0]["modes"]
            ],
            [((0,), (1,)), ((2,), (3,)), ((4,), (5,))],
        )
        self.assertEqual(
            [
                (mode["x_channels"], mode["y_channels"])
                for mode in JOYSTICK_GROUPS[1]["modes"]
            ],
            [((6,), ()), ((7,), ()), ((8,), ()), ((6, 7, 8), ())],
        )


if __name__ == "__main__":
    unittest.main()
