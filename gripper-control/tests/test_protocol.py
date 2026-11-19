import unittest

from robot_control.protocol import (
    grip_command, parse_line, set_angle, set_calibration, set_fsr_calibration,
    set_fsr_guard, set_motion_profile, set_optical_calibration, set_optical_led,
    tare_fsr, zero_optical, set_lidar_config, set_slip_lidar_threshold, set_adaptive_grip,
)


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

    def test_fsr_and_grip_commands(self):
        self.assertEqual(
            set_fsr_calibration(0, 150, 600, 1800, True),
            "FSRCAL,0,150,600,1800,1\n",
        )
        self.assertEqual(set_fsr_guard(True), "FSRGUARD,1\n")
        self.assertEqual(set_fsr_guard(False), "FSRGUARD,0\n")
        self.assertEqual(tare_fsr(), "FSRTARE\n")
        self.assertEqual(grip_command("start"), "GRIP,START\n")
        self.assertEqual(grip_command("close_hold"), "GRIP,CLOSE_HOLD\n")
        self.assertEqual(
            parse_line("FSR,10,200,650,15,205,655,0,1,2\n"),
            ("FSR", ["10", "200", "650", "15", "205", "655", "0", "1", "2"]),
        )

    def test_optical_commands_and_telemetry(self):
        self.assertEqual(set_optical_calibration(3, 0), "OPTCAL,3,0\n")
        self.assertEqual(set_optical_led(True), "OPTLED,1\n")
        self.assertEqual(set_optical_led(False), "OPTLED,0\n")
        self.assertEqual(zero_optical(), "OPTZERO\n")
        self.assertEqual(
            parse_line("OPT,-3,7,8,40,75,1\n"),
            ("OPT", ["-3", "7", "8", "40", "75", "1"]),
        )

    def test_lidar_configuration_command(self):
        self.assertEqual(set_lidar_config(True, 150, 50), "LIDARCFG,1,150,50\n")
        self.assertEqual(set_lidar_config(False, 250, 75), "LIDARCFG,0,250,75\n")
        self.assertEqual(
            parse_line("LIDAR,240,-125,1,1,1\n"),
            ("LIDAR", ["240", "-125", "1", "1", "1"]),
        )

    def test_adaptive_grip_command(self):
        self.assertEqual(set_adaptive_grip(True), "ADAPTIVE,1\n")
        self.assertEqual(set_adaptive_grip(False), "ADAPTIVE,0\n")
        self.assertEqual(parse_line("ACKADAPTIVE,1\n"), ("ACKADAPTIVE", ["1"]))

    def test_lidar_slip_sensitivity_command(self):
        self.assertEqual(set_slip_lidar_threshold(20), "SLIPCFG,20\n")
        self.assertEqual(
            parse_line("ACKSLIPCFG,20\n"),
            ("ACKSLIPCFG", ["20"]),
        )

    def test_empty_line(self):
        self.assertEqual(parse_line("\n"), ("", []))


if __name__ == "__main__":
    unittest.main()
