# Dishwasher Robot · Gripper Control

Desktop app for manual control and calibration of eight servos using an Arduino UNO R4 WiFi and PCA9685. The interface uses a dark, monochrome brutalist style.

## Hardware

- Connect PCA9685 SDA/SCL to the Arduino board's SDA/SCL pins and provide logic power to the driver board.
- Connect Arduino, PCA9685, and servo supply grounds together.
- Do not power servos from the Arduino 5 V output. Use a separate, suitable servo supply.
- Connect the Arduino to the computer with a USB data cable.
- Connect the 3 finger FSR sensors (Joy-It SEN-Pressure10) to analog pins `A0` (Finger 1), `A1` (Finger 2), and `A2` (Finger 3).
- Connect the PAA5100JE optical flow sensor over SPI: `CS` → `D10`, `MOSI` → `D11`, `MISO` → `D12`, `SCK` → `D13`.
- Connect the SparkFun Qwiic VL53L4CD LiDAR to the same I²C bus as the PCA9685 (SDA/SCL); its default I²C address is `0x29`, separate from the PCA9685 at `0x40`.

Channels 0–4 control positional servos. Channels 5–7 control continuous-rotation servos: hold **Reverse** or **Forward** to rotate and release to stop. A 90° command disables the PCA9685 output on continuous channels.

## Upload the firmware

1. Open `firmware/GripperController/GripperController.ino` in Arduino IDE.
2. Install **Adafruit PWM Servo Driver Library**, **Optical_Flow_Sensor** (`Optical_Flow_Sensors_Arduino`), and **SparkFun VL53L1X 4m Laser Distance Sensor** through Library Manager. The SparkFun library also supports the VL53L4CD.
3. Select **Arduino UNO R4 WiFi**, choose the correct port, and upload.
4. Close Serial Monitor before connecting the desktop app.

## Run the desktop app

Use Python 3.11–3.13 and install the dependencies:

```sh
python -m pip install -r requirements.txt
python main.py
```

Select the Arduino port and click **CONNECT**. The app sends a PING every second. If the Arduino stops responding, the status changes to **NO RESPONSE** instead of appearing connected. Disconnect and reconnect after an interruption.

## Tabs

- **Servos:** channels 0–4 use a slider, numeric input, and quick positions. Channels 5–7 have hold-to-run direction buttons; releasing a button stops that servo. **Center all** moves the positional servos to 90° and stops the continuous servos.
- **Calibration:** each channel has its own motion-profile toggle, maximum speed and acceleration settings, minimum/center/maximum pulse widths, point-by-point test buttons, and **Save / Apply** controls. Settings are saved on the computer and sent to the Arduino when the app connects.

## LiDAR distance monitoring and automatic gripping

The VL53L4CD is polled by the Arduino over I²C. The desktop application displays distance in mm and filtered radial speed in mm/s. Negative speed means the measured distance is decreasing (approaching); positive speed means it is increasing (receding). The speed is an estimate from successive range readings, not a measurement of tangential motion.

The **Automatic grip on entry** option starts the existing `GRIP,START` routine after three consecutive valid readings at or below the configured threshold. The firmware will not arm immediately when the option is enabled: it first requires the field of view to clear beyond the threshold plus a 50 mm re-arm margin (or repeated signal-fail/no-return readings). It then triggers only once until the object leaves and the sensor re-arms. Automatic gripping requires **FSR Auto-Stop Guard** to be enabled; disabling the guard also disables LiDAR automatic gripping and stops an active automatic grip. The existing per-finger FSR thresholds and six-second backup timeout remain responsible for stopping the fingers.

Default LiDAR trigger threshold: 150 mm. Adjustable range: 30–1300 mm. The re-arm margin is currently fixed at 50 mm.

## Per-servo motion profiles

Each calibration card has an **Enabled** toggle and its own maximum speed in degrees per second and acceleration in degrees per second squared. When enabled, the Arduino uses those limits to ramp toward that servo's target and decelerate before reaching it. Defaults are 60°/s and 120°/s². Lower either value for gentler movement; profiles can be changed while connected. When disabled, the servo moves directly to each target, as in the original controller.

On continuous-rotation channels 5–7, the profile ramps the pulse command while a direction button is held. Releasing the button still stops the output immediately. The controller estimates motion from its commands; it does not measure actual servo position. Mechanical load, supply voltage, and each servo's response affect the physical movement.

Calibrate without mechanical load. Start cautiously and stop if a servo buzzes or reaches a mechanical limit. The 600–2400 µs software range does not guarantee that every servo can safely use the full range.

## Troubleshooting

- **No port appears:** check the USB data cable, click **Refresh ports**, and make sure Arduino IDE's Serial Monitor is closed.
- **The app waits for READY:** check the PCA9685 I²C wiring and confirm it responds at address `0x40`. Make sure the current firmware is uploaded.
- **NO RESPONSE after connecting:** disconnect and reconnect in the app. If it happens again, check the USB cable and power supplies.
- **A servo does not move:** check servo power, shared ground, and the signal wire. A 90° command stops channels 5–7.
- **LiDAR sensor offline:** check the Qwiic/I²C connection and install the SparkFun VL53L1X 4m library (which supports VL53L4CD); confirm the sensor and PCA9685 share SDA/SCL and have distinct addresses.
- **Automatic grip will not arm:** enable the FSR Auto-Stop Guard, then move the object outside the trigger threshold plus the 50 mm re-arm margin.

## Project structure

```text
main.py
robot_control/
  calibration.py
  config.py
  protocol.py
  serial_worker.py
  servo_card.py
  theme.py
  window.py
firmware/GripperController/GripperController.ino
```
