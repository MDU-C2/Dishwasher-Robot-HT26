import unittest

from robot_control.protocol import parse_line, set_angle, set_calibration, set_motion_profile


class ProtocolTests(unittest.TestCase):
    def test_three_point_calibration_command(self):
        self.assertEqual(set_calibration(1, 910, 1490, 2190), "CAL,1,910,1490,2190\n")

    def test_angle_command_and_ack_parsing(self):
        self.assertEqual(set_angle(0, 90), "SET,0,90\n")
        self.assertEqual(parse_line("ACKCAL,0,900,1500,2100\n"),
                         ("ACKCAL", ["0", "900", "1500", "2100"]))
        self.assertEqual(parse_line("ACK,1,90\n"), ("ACK", ["1", "90"]))

    def test_motion_profile_command(self):
        self.assertEqual(set_motion_profile(4, True, 60, 120), "MOTION,4,1,60,120\n")

    def test_empty_line(self):
        self.assertEqual(parse_line("\n"), ("", []))


if __name__ == "__main__":
    unittest.main()
