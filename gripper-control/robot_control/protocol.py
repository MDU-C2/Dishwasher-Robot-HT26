"""Small text protocol shared by the desktop UI and Arduino firmware."""


def set_angle(channel: int, angle: int) -> str:
    return f"SET,{channel},{angle}\n"


def set_calibration(channel: int, min_us: int, center_us: int, max_us: int) -> str:
    return f"CAL,{channel},{min_us},{center_us},{max_us}\n"


def set_motion_profile(
    channel: int,
    enabled: bool,
    max_speed_deg_s: int,
    acceleration_deg_s2: int,
) -> str:
    return f"MOTION,{channel},{int(enabled)},{max_speed_deg_s},{acceleration_deg_s2}\n"


def set_fsr_calibration(
    finger_index: int,
    contact_threshold: int,
    grip_threshold: int,
    max_threshold: int,
    invert_close: bool = False,
) -> str:
    return (
        f"FSRCAL,{finger_index},{contact_threshold},"
        f"{grip_threshold},{max_threshold},{int(invert_close)}\n"
    )


def set_fsr_guard(enabled: bool) -> str:
    return f"FSRGUARD,{int(enabled)}\n"


def tare_fsr() -> str:
    return "FSRTARE\n"


def grip_command(action: str) -> str:
    return f"GRIP,{action.upper()}\n"


def set_optical_calibration(motion_threshold: int, squal_threshold: int) -> str:
    return f"OPTCAL,{motion_threshold},{squal_threshold}\n"


def set_optical_led(enabled: bool) -> str:
    return f"OPTLED,{int(enabled)}\n"


def zero_optical() -> str:
    return "OPTZERO\n"


def set_lidar_and_loop_config(
    trigger_mm: int,
    dwell_ms: int,
    slip_vel_mm_s: int,
    release_ms: int,
    boost_step: int,
    hold_deadband: int,
    grip_timeout_ms: int = 4000,
) -> str:
    return (
        f"LOOPCAL,{trigger_mm},{dwell_ms},{slip_vel_mm_s},"
        f"{release_ms},{boost_step},{hold_deadband},{grip_timeout_ms}\n"
    )


def set_auto_modes(auto_lidar_grip: bool, closed_loop_hold: bool) -> str:
    return f"AUTOMODE,{int(auto_lidar_grip)},{int(closed_loop_hold)}\n"


def parse_line(line: str) -> tuple[str, list[str]]:
    parts = line.strip().split(",")
    return (parts[0].upper(), parts[1:]) if parts and parts[0] else ("", [])
