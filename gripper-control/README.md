# Dishwasher Robot – Gripper Control (version 1.4)

A modular controller for nine servos connected to PCA9685 channels **0–8**, using an Arduino UNO R4 WiFi and a PySide6 desktop application for macOS. Channels 0–5 are the existing servos; channels 6–8 are three continuous-rotation (360°) finger servos. The manual `SET` command retains a 0–180 command range, while joystick control provides proportional speed control.

## 1. Hardware connections

- Connect Arduino UNO R4 WiFi **SDA → SDA** and **SCL → SCL** on the PCA9685.
- Connect Arduino **5V → VCC (logic)** and **GND → GND** on the PCA9685.
- Connect the external servo supply to PCA9685 **V+** and **GND**, using a voltage suitable for the servos.
- Connect the grounds of the Arduino, PCA9685, and external servo supply.
- Connect servos 1–6 to PCA9685 channels 0–5. Connect signal to PWM, red to V+, and black/brown to GND; verify wire colors for your servo.
- Connect continuous finger servos 1–3 to PCA9685 channels 6–8. At stop, the PCA9685 output is fully disabled. Direction and speed are controlled relative to the calibrated center pulse.
- Connect the FSR through a suitable voltage-divider circuit to **A1**. The GUI displays voltage calculated from the ADC reading using a 5.00 V reference. Share ground and keep the Arduino input voltage within its allowed range.
- Connect the Arduino to the computer over USB. **Do not power the servos from the Arduino 5V pin.**
- Connect the joystick: **VRX → A2**, **VRY → A3**, **SW → D2**, VCC → 5V, and GND → GND.

### Safety and calibration notes

At startup, the Arduino sends all nine channels to the default 90 command. For channels 6–8, this disables the PCA9685 output and stops the continuous-rotation servo. Test without tendon lines or other mechanical load first; movement may be abrupt. Keep a physical way to disconnect servo power within reach. The desktop application sends stop commands to continuous servos when it disconnects normally. A position servo may continue holding its last commanded position, and an unexpected loss of USB connection may leave the PCA9685 producing its last output.

The GUI's 0–180 values are commands. The MG996R position servos do not report their actual angle, so the GUI cannot display measured position. Each channel has three calibration points: 0°/minimum, 90°/center, and 180°/maximum. The Arduino interpolates linearly between the points. Calibration values are saved per channel on the Mac using Qt `QSettings` and sent to the Arduino when the application connects.

The application limits calibration values to 600–2400 µs. These are software limits, not guaranteed safe endpoints from the servo manufacturer. The initial values are 1000/1500/2000 µs for position servos and 900/1500/2100 µs for continuous servos. Change one value at a time in 10 µs increments. Stop and reduce the range if a servo reaches a mechanical stop, buzzes, or draws noticeably more current. Use **Cancel / Stop** to leave a calibration test.

The [Waveshare MG996R product page](https://www.waveshare.com/product/robotics/motors-servos/servos/mg996r-servo.htm) and the [Electrokit page for the specified product](https://www.electrokit.com/analogt-servo-11kg-0.18s-metall-stort-moment) specify 4.8–6.0 V, 180° travel, and torque/speed ratings, but do not provide verified pulse widths for the 0° and 180° endpoints. Therefore, 500–2500 µs is not treated as a safe specification for these particular servos. The [Adafruit PCA9685 example](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino) uses 600–2400 µs as an adjustable example and notes that endpoints must be tuned for each servo.

The PCA9685 runs at 50 Hz. The Adafruit library's `writeMicroseconds()` converts microseconds to 12-bit ticks using the nominal 25 MHz oscillator setting. Actual oscillator frequency varies between boards. For higher precision, measure the PWM output with an oscilloscope and update `PCA9685_OSCILLATOR_HZ` in the firmware.

## 2. Upload the Arduino firmware

1. Open **Arduino IDE**, select **Arduino UNO R4 WiFi**, and select the correct port.
2. Install **Adafruit PWM Servo Driver Library** through Library Manager, including its dependencies.
3. Open `firmware/GripperController/GripperController.ino` and click **Upload**.
4. Close Arduino IDE's **Serial Monitor** before connecting from Python. Both applications cannot use the same serial port at the same time.

If older firmware is installed, upload the current `.ino` file. The current GUI sends three-point `CAL` commands and expects the updated firmware protocol.

## 3. Install and run the Python application on macOS

In Terminal, change to the project directory containing `main.py`, then run:

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Use a Python version supported by PySide6, such as Python 3.11–3.13. After installation, start the application with `python main.py` or double-click `start_mac.command` (macOS may ask you to confirm the first launch). **Arduino IDE is not needed when running the GUI after the firmware has been uploaded.**

## 4. Using the application

1. Select the USB port, usually `/dev/cu.usbmodem...` on macOS.
2. Click **Connect** and give the Arduino a moment to restart if the serial port causes a reset.
3. Use the sliders or angle fields for position servos on channels 0–5. For finger servos 1–3, hold **Reverse** or **Forward**; releasing the button sends stop.
4. Use **0° / 90° / 180°** quick controls or **Center all at 90°** for position-servo commands. For continuous servos, 90° means stop.
5. A green ACK message means the Arduino accepted the command; it does not indicate a measured servo angle.
6. The FSR tile under **READY FOR EXPANSION** displays the live voltage from A1 along with its underlying raw ADC reading. Voltage is not a force measurement.

## 4.1 Joystick control

The Arduino reads the joystick locally for low-latency control. On startup, it calibrates the X/Y neutral point; leave the joystick untouched during this process. The default dead zone is ±40 ADC counts, and the button debounce time is 200 ms. The button is configured as active-low with `INPUT_PULLUP`. If your wiring behaves as 0=released and 1=pressed, change `JOYSTICK_BUTTON_ACTIVE_LOW` to `false` in the firmware and upload it again.

Use the **Finger mode (channels 6–8)** toggle to select the joystick group. The **Other** group has three modes:

| Mode | X axis | Y axis |
|---|---|---|
| 1 · Gripper group | Channel 0 | Channel 1 |
| 2 · Middle group | Channel 2 | Channel 3 |
| 3 · End group | Channel 4 | Channel 5 |

In **Finger mode**, the joystick's X axis is used and its Y axis is intentionally inactive:

| Mode | X axis | Y axis |
|---|---|---|
| 1 · Finger 1 | Channel 6 | Not used |
| 2 · Finger 2 | Channel 7 | Not used |
| 3 · Finger 3 | Channel 8 | Not used |
| 4 · All fingers | Channels 6, 7, and 8 | Not used |

For continuous servos on channels 6–8, the PCA9685 output is fully disabled at joystick neutral. Speed increases proportionally as the stick moves farther from center. Maximum speed is based on each servo's calibrated minimum and maximum pulse widths. Adjust `JOYSTICK_MAX_SPEED` in the firmware to change the normalized speed range. When the mode changes, the previous group is stopped, and the joystick must return to neutral before the new mode is armed.

Press the joystick button to move to the next mode in the selected group. Manual control and joystick control do not compete: a manual slider or hold-button command disables joystick control. Turn **Joystick active** back on and release the joystick to neutral to resume joystick operation. Group mappings are defined by `JOYSTICK_GROUPS` in `robot_control/config.py` and the corresponding `joystickModes` table in the firmware.

## 4.2 Calibrating servo movement

The **SERVO CALIBRATION / PWM** panel provides three pulse-width values for each channel. Calibration changes are applied only when that channel is at its center/stop command.

1. If possible, disconnect mechanical loads and tendon lines.
2. Connect the Arduino. On a new connection, all channels receive the safe default command of **90°**; previous GUI targets are not replayed. For Finger 1–3, the PCA9685 output is disabled at this command.
3. Select a channel and check its starting values. Position servos normally use **1000 / 1500 / 2000 µs**; Finger 1–3 start at **900 / 1500 / 2100 µs** as a wider speed range.
4. Click **Test 90°** for a position servo or **Test Stop** for a continuous servo. Change one field at a time, for example the minimum by 10 µs, then click **Save / Apply**.
5. Test **0°** for a position servo or **Test Reverse** for a continuous servo. Check the motion without holding the servo against a mechanical stop. Adjust the minimum pulse in 10 µs steps as needed.
6. Use **Cancel / Stop** between endpoint tests. Repeat carefully for **180°** or **Test Forward**.
7. Adjust the center pulse independently if a position servo's center is off or if a continuous servo creeps when receiving its center pulse. For continuous servos, the center pulse is used as the speed reference; the output is disabled at stop.
8. Leave each channel at 90°/stop when finished, then calibrate the next channel. Values are saved independently for channels 0–8 and restored on the next application launch.

The Arduino accepts calibration commands such as `CAL,0,900,1500,2100` and replies with `ACKCAL,0,900,1500,2100`. Calibration values are held in Arduino RAM and reset when it restarts; the GUI sends the saved values again on connection.

## Troubleshooting

- **No serial port appears:** check the USB data cable and board, then click **Refresh ports**.
- **No READY message / connection stays pending:** check the I²C connections and confirm the PCA9685 responds at address `0x40`. The firmware reports an error and does not become ready if the driver is not found.
- **A servo does not move:** check external V+, common ground, and the signal wire. For channels 6–8, 90° stops the servo by disabling the PCA9685 output. Confirm the joystick is enabled and has been returned to neutral after a mode change.
- **A position servo does not reach 180°:** this may be normal for an individual MG996R or its mechanical linkage. Check the 50 Hz signal, power supply, and mechanical clearance, then calibrate the minimum and maximum carefully.
- **Calibration appears to have no effect:** upload the current `.ino` file, confirm the Arduino replies with four values in `ACKCAL` (`channel,min,center,max`), and apply calibration while the channel is at 90°/stop.
- **Python/Qt dialog appears on macOS:** the PySide6 application runs locally and does not require a server.

## Project structure

```text
gripper-control/
├── main.py                         starts the desktop GUI
├── start_mac.command               macOS launcher
├── requirements.txt
├── robot_control/
│   ├── config.py                   channels, names, and defaults
│   ├── calibration.py              three-point calibration model and validation
│   ├── protocol.py                 shared serial message format
│   ├── serial_worker.py            serial communication outside the GUI thread
│   ├── servo_card.py               reusable servo widget
│   ├── theme.py                    colors and visual style
│   └── window.py                   layout, controls, and sensor display
└── firmware/
    └── GripperController/
        └── GripperController.ino   Arduino/PCA9685 firmware
```

## Future expansion

- **Additional servos:** add the channel and name in `robot_control/config.py`, update `SERVO_COUNT` and `SERVO_CHANNELS`, and add calibration entries in the Arduino sketch. The GUI builds servo cards from the configuration and lays them out in two columns.
- **Additional FSR sensors:** the first FSR is already read from A1 through a suitable voltage divider and sent as `FSR,1,512,2501` (raw ADC value and millivolts). The 5.00 V reference is set by `FSR_ADC_REFERENCE_MV` in the firmware. Add force calibration if readings need to be expressed in grams or newtons.
- **LiDAR:** connect SDA/SCL to the same I²C bus as the PCA9685, check the breakout's voltage requirements, and use a unique I²C address. Add the sensor code to the Arduino sketch and send telemetry such as `DIST,43` for 43 mm. The library and sample rate depend on the exact VL53 model.
- **Slip detection:** use the same message-based approach as for other sensors.

For future safety-critical behavior that must work without USB, implement the corresponding limits and control logic on the Arduino. Use Python for higher-level control and visualization.
