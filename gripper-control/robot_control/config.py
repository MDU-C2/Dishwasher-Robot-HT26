"""Application-wide configuration.

Keep hardware-independent servo settings here so that future FSR/LiDAR
support can be added without coupling it to the GUI widgets.
"""

APP_TITLE = "Dishwasher Robot · Control"
BAUD_RATE = 115200
FSR_ADC_CHANNEL = 1
FSR_ADC_LABEL = "A1"
FSR_ADC_MAX = 1023
FSR_ADC_REFERENCE_MV = 5000
SERVO_CHANNELS = {
    0: "Servo 01",
    1: "Servo 02",
    2: "Servo 03",
    3: "Servo 04",
    4: "Servo 05",
    5: "Servo 06",
    6: "Finger 1",
    7: "Finger 2",
    8: "Finger 3",
}
CONTINUOUS_SERVO_CHANNELS = (6, 7, 8)
DEFAULT_ANGLE = 90
MIN_ANGLE = 0
MAX_ANGLE = 180
COMMAND_INTERVAL_MS = 50  # Send at most 20 updates/second per changed channel

# These are software safety limits, not manufacturer guarantees. The exact
# usable endpoints must be found on each physical servo without load. The
# Adafruit PCA9685 example uses 600..2400 us as a typical adjustable window;
# we keep that as the widest range exposed by this application and start each
# servo at the more conservative 1000..2000 us.
PULSE_MIN_ALLOWED_US = 600
PULSE_MAX_ALLOWED_US = 2400
CALIBRATION_STEP_US = 10
DEFAULT_CALIBRATION_US = (1000, 1500, 2000)
# A slightly wider, still bounded starting range for the modified continuous
# servos. This affects speed command range, not the electrical supply current.
DEFAULT_CONTINUOUS_CALIBRATION_US = (900, 1500, 2100)

# Joystick input is read by the Arduino so the control loop remains responsive
# even when the USB serial link is busy. Channel mappings are deliberately
# kept in one small structure so joint groupings can be changed later.
JOYSTICK_DEADZONE_ADC = 40
JOYSTICK_DEBOUNCE_MS = 200
JOYSTICK_GROUPS = (
    {
        "key": "remaining",
        "name": "Övriga",
        "modes": (
            {"name": "Gripper group", "x_channels": (0,), "y_channels": (1,)},
            {"name": "Middle group", "x_channels": (2,), "y_channels": (3,)},
            {"name": "End group", "x_channels": (4,), "y_channels": (5,)},
        ),
    },
    {
        "key": "fingers",
        "name": "Fingrar",
        "modes": (
            {"name": "Finger 1", "x_channels": (6,), "y_channels": ()},
            {"name": "Finger 2", "x_channels": (7,), "y_channels": ()},
            {"name": "Finger 3", "x_channels": (8,), "y_channels": ()},
            {"name": "Alla fingrar", "x_channels": (6, 7, 8), "y_channels": ()},
        ),
    },
)
