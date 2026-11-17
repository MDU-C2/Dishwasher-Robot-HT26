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


def parse_line(line: str) -> tuple[str, list[str]]:
    parts = line.strip().split(",")
    return (parts[0].upper(), parts[1:]) if parts and parts[0] else ("", [])
