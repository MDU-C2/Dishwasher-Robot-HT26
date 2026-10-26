"""Text protocol shared by the desktop and Arduino firmware.

Desktop -> Arduino:
    HELLO             request readiness
    SET,<channel>,<angle>
                                      for continuous channels 6–8: 0/180=direction,
                                      90=stop
    GET               request latest commanded positions
    CENTER            reset all configured channels to 90 degrees
    CAL,<channel>,<min_us>,<center_us>,<max_us>
                                      adjust the three servo PWM points
    JOY,<enabled>      enable/disable local joystick control
    JOYAREA,<area>     select 0=remaining joints or 1=fingers

Arduino -> Desktop:
    READY,<count>
    ACK,<channel>,<angle>       command accepted (not physical angle feedback)
    STATE,<channel>,<angle>     latest commanded target
    ACKCAL,<channel>,<min_us>,<center_us>,<max_us> calibration accepted
    JOY,<area>,<enabled>,<mode>,<raw_x>,<raw_y>,<speed_x>,<speed_y>,<armed>
                                      joystick status at a low telemetry rate
    ERR,<reason>

    FSR,<adc_channel>,<raw_value>,<millivolts>
                                      e.g. FSR,1,438,2143 for analog pin A1
    DIST,<millimeters>             future LiDAR telemetry
"""


def set_angle(channel: int, angle: int) -> str:
    return f"SET,{channel},{angle}\n"


def set_calibration(channel: int, min_us: int, center_us: int, max_us: int) -> str:
    return f"CAL,{channel},{min_us},{center_us},{max_us}\n"


def set_joystick_enabled(enabled: bool) -> str:
    return f"JOY,{1 if enabled else 0}\n"


def set_joystick_area(area: int) -> str:
    if area not in (0, 1):
        raise ValueError("joystick area must be 0 (remaining) or 1 (fingers)")
    return f"JOYAREA,{area}\n"


def parse_line(line: str) -> tuple[str, list[str]]:
    parts = line.strip().split(",")
    return (parts[0].upper(), parts[1:]) if parts and parts[0] else ("", [])
