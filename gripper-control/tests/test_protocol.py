import unittest

from robot_control.protocol import (
    parse_line,
    set_angle,
    set_calibration,
    set_joystick_area,
    set_joystick_enabled,
)


class ProtocolTests(unittest.TestCase):
    def test_three_point_calibration_command(self):
        self.assertEqual(set_calibration(1, 910, 1490, 2190), "CAL,1,910,1490,2190\n")

    def test_angle_command_and_ack_parsing(self):
        self.assertEqual(set_angle(0, 90), "SET,0,90\n")
        self.assertEqual(parse_line("ACKCAL,0,900,1500,2100\n"), ("ACKCAL", ["0", "900", "1500", "2100"]))
        self.assertEqual(parse_line("ACK,1,90\n"), ("ACK", ["1", "90"]))

    def test_joystick_enable_command_and_telemetry_parsing(self):
        self.assertEqual(set_joystick_enabled(True), "JOY,1\n")
        self.assertEqual(set_joystick_enabled(False), "JOY,0\n")
        self.assertEqual(set_joystick_area(0), "JOYAREA,0\n")
        self.assertEqual(set_joystick_area(1), "JOYAREA,1\n")
        self.assertEqual(
            parse_line("JOY,1,1,2,500,505,0,0,1\n"),
            ("JOY", ["1", "1", "2", "500", "505", "0", "0", "1"]),
        )
        self.assertEqual(
            parse_line("FSR,1,438,2143\n"),
            ("FSR", ["1", "438", "2143"]),
        )


if __name__ == "__main__":
    unittest.main()
