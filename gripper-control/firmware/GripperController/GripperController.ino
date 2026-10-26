// Dishwasher Robot HT26 - controller for nine PCA9685 servos.
// SET preserves the existing 0..180 manual path; JOY drives continuous
// rotation servos around their calibrated speed pulses. For channels 6..8,
// the PCA9685 output is disabled completely when the command is stopped.
// Board: Arduino UNO R4 WiFi
// PCA9685 default I2C address: 0x40
// Servo signal: PCA9685 channels 0 through 8.
//
// Computer -> Arduino over USB Serial 115200:
//   HELLO
//   SET,0,90
//   SET,1,120
//   SET,6,180              (continuous servo: direction/speed command)
//   GET
//   CENTER
//   CAL,0,1000,1500,2000  (0/90/180 pulse points in microseconds)
//   JOY,1                  (enable local joystick; JOY,0 disables it)
//   JOYAREA,0              (0=remaining joints, 1=fingers)
//   For channels 6..8, SET 0/180 means direction and SET 90 means stop.
//
// Arduino -> Computer:
//   READY,9
//   ACK,0,90       (accepted target, NOT measured angle)
//   STATE,0,90
//   ACKCAL,0,1000,1500,2000
//   JOY,0,1,500,505,0,0,1  (area, enabled, mode, ADC, speed, armed)
//   FSR,1,438,2143        (raw ADC value and millivolts from analog pin A1)
//   ERR,BAD_COMMAND

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint32_t PCA9685_OSCILLATOR_HZ = 25000000UL;
constexpr uint8_t SERVO_PWM_FREQUENCY_HZ = 50;

Adafruit_PWMServoDriver pca(PCA9685_ADDRESS);

constexpr uint8_t SERVO_COUNT = 9;
constexpr uint8_t SERVO_CHANNELS[SERVO_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t CONTINUOUS_SERVO_FIRST_CHANNEL = 6;
constexpr int DEFAULT_ANGLE = 90;

constexpr uint8_t JOYSTICK_X_PIN = A2;
constexpr uint8_t JOYSTICK_Y_PIN = A3;
constexpr uint8_t JOYSTICK_BUTTON_PIN = 2;
constexpr bool JOYSTICK_BUTTON_ACTIVE_LOW = true;
constexpr int JOYSTICK_ADC_MAX = 1023;
constexpr int JOYSTICK_DEADZONE_ADC = 40;
constexpr int JOYSTICK_MAX_SPEED = 1000;
constexpr unsigned long JOYSTICK_DEBOUNCE_MS = 200;
constexpr unsigned long JOYSTICK_UPDATE_MS = 20;
constexpr unsigned long JOYSTICK_TELEMETRY_MS = 100;
constexpr uint8_t FSR_PIN = A1;
constexpr uint8_t FSR_ADC_CHANNEL = 1;
constexpr int FSR_ADC_MAX = 1023;
constexpr int FSR_ADC_REFERENCE_MV = 5000;
constexpr unsigned long FSR_SAMPLE_MS = 20;
constexpr unsigned long FSR_TELEMETRY_MS = 100;

// Software safety envelope. This is deliberately narrower than the often
// repeated, but not Waveshare-guaranteed, 500..2500 us claim. Calibrate each
// physical servo inside this envelope and stop immediately at any mechanical
// resistance or buzzing. The endpoints are not a claim about safe mechanics.
constexpr int PULSE_MIN_ALLOWED_US = 600;
constexpr int PULSE_MAX_ALLOWED_US = 2400;

// Start conservatively. The servo-specific pulse points may differ, including
// the center. 1000/1500/2000 us is only a starting point, not a guarantee of
// exactly 0/90/180 degrees.
struct ServoConfig {
  uint16_t minUs;
  uint16_t centerUs;
  uint16_t maxUs;
};
ServoConfig servoConfig[SERVO_COUNT] = {
  {1000, 1500, 2000},  // PCA9685 channel 0
  {1000, 1500, 2000},  // PCA9685 channel 1
  {1000, 1500, 2000},  // PCA9685 channel 2
  {1000, 1500, 2000},  // PCA9685 channel 3
  {1000, 1500, 2000},  // PCA9685 channel 4
  {1000, 1500, 2000},  // PCA9685 channel 5
  {900, 1500, 2100},  // PCA9685 channel 6, continuous finger 1
  {900, 1500, 2100},  // PCA9685 channel 7, continuous finger 2
  {900, 1500, 2100},  // PCA9685 channel 8, continuous finger 3
};

int targets[SERVO_COUNT] = {
  DEFAULT_ANGLE, DEFAULT_ANGLE, DEFAULT_ANGLE,
  DEFAULT_ANGLE, DEFAULT_ANGLE, DEFAULT_ANGLE,
  DEFAULT_ANGLE, DEFAULT_ANGLE, DEFAULT_ANGLE,
};

constexpr uint8_t JOYSTICK_AREA_REMAINING = 0;
constexpr uint8_t JOYSTICK_AREA_FINGERS = 1;
constexpr uint8_t JOYSTICK_AREA_COUNT = 2;
constexpr uint8_t JOYSTICK_MAX_MODE_COUNT = 4;

// Each mode maps joystick X/Y to one or more logical servo indices. The
// finger modes use only X: one axis is enough to drive one finger or all three
// together, while Y is deliberately ignored and shown as unused in the GUI.
struct JoystickAxisConfig {
  uint8_t servoIndices[3];
  uint8_t count;
  bool inverted;
};
struct JoystickMode {
  JoystickAxisConfig x;
  JoystickAxisConfig y;
};
const JoystickMode joystickModes[JOYSTICK_AREA_COUNT][JOYSTICK_MAX_MODE_COUNT] = {
  {
    { {{0, 0, 0}, 1, false}, {{1, 0, 0}, 1, false} },  // channels 0 + 1
    { {{2, 0, 0}, 1, false}, {{3, 0, 0}, 1, false} },  // channels 2 + 3
    { {{4, 0, 0}, 1, false}, {{5, 0, 0}, 1, false} },  // channels 4 + 5
  },
  {
    { {{6, 0, 0}, 1, false}, {{0, 0, 0}, 0, false} },  // finger 1
    { {{7, 0, 0}, 1, false}, {{0, 0, 0}, 0, false} },  // finger 2
    { {{8, 0, 0}, 1, false}, {{0, 0, 0}, 0, false} },  // finger 3
    { {{6, 7, 8}, 3, false}, {{0, 0, 0}, 0, false} },  // all fingers
  },
};
const uint8_t joystickModeCounts[JOYSTICK_AREA_COUNT] = {3, 4};

int joystickCenterX = 512;
int joystickCenterY = 512;
uint8_t joystickArea = JOYSTICK_AREA_REMAINING;
uint8_t joystickMode = 0;
bool joystickEnabled = true;
bool joystickNeedsNeutral = false;
bool joystickButtonRaw = false;
bool joystickButtonStable = false;
unsigned long joystickButtonChangedAt = 0;
unsigned long lastJoystickUpdateAt = 0;
unsigned long lastJoystickTelemetryAt = 0;
unsigned long lastFsrSampleAt = 0;
unsigned long lastFsrTelemetryAt = 0;
int fsrRawValue = 0;
int lastJoystickXSpeed = 2001;
int lastJoystickYSpeed = 2001;
char commandBuffer[48];
uint8_t bufferIndex = 0;

bool parseNumber(const char* s, long& result) {
  if (!s || !*s) return false;
  char* endptr;
  result = strtol(s, &endptr, 10);
  return *endptr == '\0';
}

int findServoIndex(long channel) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (channel == SERVO_CHANNELS[i]) return i;
  }
  return -1;
}

bool isContinuousServoIndex(int index) {
  return index >= 0 && index < SERVO_COUNT &&
         SERVO_CHANNELS[index] >= CONTINUOUS_SERVO_FIRST_CHANNEL;
}

void stopContinuousServo(int index) {
  // PCA9685 full-off bit: no servo pulse is generated. This is intentional for
  // the modified 360-degree servos, whose center pulse can otherwise creep.
  pca.setPWM(SERVO_CHANNELS[index], 0, 4096);
}

void writePosition(int index, int angle) {
  const ServoConfig& cal = servoConfig[index];
  long microseconds;
  if (angle <= DEFAULT_ANGLE) {
    microseconds = cal.minUs +
                   ((long)(cal.centerUs - cal.minUs) * angle + 45) / 90;
  } else {
    microseconds = cal.centerUs +
                   ((long)(cal.maxUs - cal.centerUs) * (angle - DEFAULT_ANGLE) + 45) / 90;
  }
  if (isContinuousServoIndex(index) && angle == DEFAULT_ANGLE) {
    stopContinuousServo(index);
  } else {
    pca.writeMicroseconds(SERVO_CHANNELS[index], (uint16_t)microseconds);
  }
  targets[index] = angle;
}

int joystickSpeedFromRaw(int raw, int center) {
  const int delta = raw - center;
  if (abs(delta) <= JOYSTICK_DEADZONE_ADC) return 0;

  if (delta > 0) {
    const int span = JOYSTICK_ADC_MAX - center - JOYSTICK_DEADZONE_ADC;
    if (span <= 0) return JOYSTICK_MAX_SPEED;
    int speed = ((delta - JOYSTICK_DEADZONE_ADC) * JOYSTICK_MAX_SPEED) / span;
    if (speed < 1) speed = 1;
    if (speed > JOYSTICK_MAX_SPEED) speed = JOYSTICK_MAX_SPEED;
    return speed;
  }

  const int span = center - JOYSTICK_DEADZONE_ADC;
  if (span <= 0) return -JOYSTICK_MAX_SPEED;
  int speed = (((-delta) - JOYSTICK_DEADZONE_ADC) * JOYSTICK_MAX_SPEED) / span;
  if (speed < 1) speed = 1;
  if (speed > JOYSTICK_MAX_SPEED) speed = JOYSTICK_MAX_SPEED;
  return -speed;
}

void writeJoystickSpeed(int index, int speed) {
  if (speed < -JOYSTICK_MAX_SPEED) speed = -JOYSTICK_MAX_SPEED;
  if (speed > JOYSTICK_MAX_SPEED) speed = JOYSTICK_MAX_SPEED;

  if (isContinuousServoIndex(index) && speed == 0) {
    stopContinuousServo(index);
    return;
  }

  const ServoConfig& cal = servoConfig[index];
  long microseconds = cal.centerUs;
  if (speed < 0) {
    microseconds = cal.centerUs -
                   ((long)(cal.centerUs - cal.minUs) * (-speed) + JOYSTICK_MAX_SPEED / 2) /
                       JOYSTICK_MAX_SPEED;
  } else if (speed > 0) {
    microseconds = cal.centerUs +
                   ((long)(cal.maxUs - cal.centerUs) * speed + JOYSTICK_MAX_SPEED / 2) /
                       JOYSTICK_MAX_SPEED;
  }
  pca.writeMicroseconds(SERVO_CHANNELS[index], (uint16_t)microseconds);
}

const JoystickMode& currentJoystickMode() {
  return joystickModes[joystickArea][joystickMode];
}

void writeJoystickAxis(const JoystickAxisConfig& axis, int speed) {
  if (axis.inverted) speed = -speed;
  for (uint8_t i = 0; i < axis.count; i++) {
    writeJoystickSpeed(axis.servoIndices[i], speed);
  }
}

void stopJoystickMode(uint8_t area, uint8_t mode) {
  if (area >= JOYSTICK_AREA_COUNT || mode >= joystickModeCounts[area]) return;
  const JoystickMode& selectedMode = joystickModes[area][mode];
  writeJoystickAxis(selectedMode.x, 0);
  writeJoystickAxis(selectedMode.y, 0);
  lastJoystickXSpeed = 2001;
  lastJoystickYSpeed = 2001;
}

bool joystickIsNeutral(int rawX, int rawY) {
  return abs(rawX - joystickCenterX) <= JOYSTICK_DEADZONE_ADC &&
         abs(rawY - joystickCenterY) <= JOYSTICK_DEADZONE_ADC;
}

void readJoystickState(int& rawX, int& rawY, int& speedX, int& speedY) {
  rawX = analogRead(JOYSTICK_X_PIN);
  rawY = analogRead(JOYSTICK_Y_PIN);
  speedX = joystickSpeedFromRaw(rawX, joystickCenterX);
  speedY = joystickSpeedFromRaw(rawY, joystickCenterY);
  const JoystickMode& mode = currentJoystickMode();
  if (mode.x.inverted) speedX = -speedX;
  if (mode.y.inverted) speedY = -speedY;
  if (mode.y.count == 0) speedY = 0;
}

void printJoystickTelemetry() {
  int rawX, rawY, speedX, speedY;
  readJoystickState(rawX, rawY, speedX, speedY);
  Serial.print("JOY,");
  Serial.print(joystickArea);
  Serial.print(',');
  Serial.print(joystickEnabled ? 1 : 0);
  Serial.print(',');
  Serial.print(joystickMode);
  Serial.print(',');
  Serial.print(rawX);
  Serial.print(',');
  Serial.print(rawY);
  Serial.print(',');
  Serial.print(speedX);
  Serial.print(',');
  Serial.print(speedY);
  Serial.print(',');
  Serial.println(joystickNeedsNeutral ? 0 : 1);
}

void updateJoystickButton(unsigned long now) {
  const int buttonLevel = digitalRead(JOYSTICK_BUTTON_PIN);
  const bool rawPressed = JOYSTICK_BUTTON_ACTIVE_LOW
                              ? buttonLevel == LOW
                              : buttonLevel == HIGH;
  if (rawPressed != joystickButtonRaw) {
    joystickButtonRaw = rawPressed;
    joystickButtonChangedAt = now;
  }

  if ((now - joystickButtonChangedAt) >= JOYSTICK_DEBOUNCE_MS &&
      joystickButtonStable != joystickButtonRaw) {
    joystickButtonStable = joystickButtonRaw;
    if (joystickButtonStable) {
      const uint8_t previousMode = joystickMode;
      joystickMode = (joystickMode + 1) % joystickModeCounts[joystickArea];
      if (joystickEnabled) {
        stopJoystickMode(joystickArea, previousMode);
        stopJoystickMode(joystickArea, joystickMode);
        joystickNeedsNeutral = true;
      }
      printJoystickTelemetry();
    }
  }
}

void updateJoystickControl() {
  if (!joystickEnabled) return;

  int rawX, rawY, speedX, speedY;
  readJoystickState(rawX, rawY, speedX, speedY);
  if (joystickNeedsNeutral) {
    if (joystickIsNeutral(rawX, rawY)) {
      joystickNeedsNeutral = false;
      lastJoystickXSpeed = 2001;
      lastJoystickYSpeed = 2001;
    } else {
      stopJoystickMode(joystickArea, joystickMode);
      return;
    }
  }

  const JoystickMode& mode = currentJoystickMode();
  if (speedX != lastJoystickXSpeed) {
    writeJoystickAxis(mode.x, speedX);
    lastJoystickXSpeed = speedX;
  }
  if (speedY != lastJoystickYSpeed) {
    writeJoystickAxis(mode.y, speedY);
    lastJoystickYSpeed = speedY;
  }
}

void disableJoystickForManualControl() {
  if (!joystickEnabled) return;
  stopJoystickMode(joystickArea, joystickMode);
  joystickEnabled = false;
  joystickNeedsNeutral = true;
  printJoystickTelemetry();
}

void calibrateJoystickCenter() {
  long sumX = 0;
  long sumY = 0;
  constexpr int SAMPLE_COUNT = 32;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    delay(5);
  }
  joystickCenterX = sumX / SAMPLE_COUNT;
  joystickCenterY = sumY / SAMPLE_COUNT;
}

void updateFsr() {
  fsrRawValue = analogRead(FSR_PIN);
}

void printFsrTelemetry() {
  const int millivolts =
      ((long)fsrRawValue * FSR_ADC_REFERENCE_MV + FSR_ADC_MAX / 2) / FSR_ADC_MAX;
  Serial.print("FSR,");
  Serial.print(FSR_ADC_CHANNEL);
  Serial.print(',');
  Serial.print(fsrRawValue);
  Serial.print(',');
  Serial.println(millivolts);
}

void printPosition(const char* prefix, int index) {
  Serial.print(prefix);
  Serial.print(',');
  Serial.print(SERVO_CHANNELS[index]);
  Serial.print(',');
  Serial.println(targets[index]);
}

void centerAll() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    writePosition(i, DEFAULT_ANGLE);
  }
}

void handleCommand(char* command) {
  if (strcmp(command, "HELLO") == 0) {
    Serial.print("READY,");
    Serial.println(SERVO_COUNT);
    return;
  }
  if (strcmp(command, "GET") == 0) {
    for (int i = 0; i < SERVO_COUNT; i++) printPosition("STATE", i);
    return;
  }
  if (strcmp(command, "CENTER") == 0) {
    disableJoystickForManualControl();
    centerAll();
    for (int i = 0; i < SERVO_COUNT; i++) printPosition("ACK", i);
    return;
  }
  // Runtime pulse calibration. Python can set/save these values without
  // reflashing the Arduino each time. CAL itself does not move the servo;
  // the new mapping is used on the next SET.
  if (strncmp(command, "CAL,", 4) == 0) {
    char* channelStr = command + 4;
    char* comma1 = strchr(channelStr, ',');
    if (!comma1) {
      Serial.println("ERR,BAD_CAL_FORMAT");
      return;
    }
    *comma1 = '\0';
    char* minStr = comma1 + 1;
    char* comma2 = strchr(minStr, ',');
    if (!comma2) {
      Serial.println("ERR,BAD_CAL_FORMAT");
      return;
    }
    *comma2 = '\0';
    char* centerOrMaxStr = comma2 + 1;
    char* comma3 = strchr(centerOrMaxStr, ',');
    char* centerStr = NULL;
    char* maxStr = centerOrMaxStr;
    if (comma3) {
      *comma3 = '\0';
      centerStr = centerOrMaxStr;
      maxStr = comma3 + 1;
    }

    long channel = 0, minUs = 0, centerUs = 0, maxUs = 0;
    if (!parseNumber(channelStr, channel) ||
        !parseNumber(minStr, minUs) ||
        (centerStr && !parseNumber(centerStr, centerUs)) ||
        !parseNumber(maxStr, maxUs)) {
      Serial.println("ERR,BAD_CAL_NUMBER");
      return;
    }
    // Accept the old v1.1 two-point CAL form as a migration path. New GUI
    // versions always send the explicit center point.
    if (!centerStr) centerUs = (minUs + maxUs) / 2;

    const int index = findServoIndex(channel);
    if (index < 0 ||
        minUs < PULSE_MIN_ALLOWED_US || minUs > PULSE_MAX_ALLOWED_US ||
        centerUs < PULSE_MIN_ALLOWED_US || centerUs > PULSE_MAX_ALLOWED_US ||
        maxUs < PULSE_MIN_ALLOWED_US || maxUs > PULSE_MAX_ALLOWED_US ||
        !(minUs < centerUs && centerUs < maxUs)) {
      Serial.println("ERR,BAD_CAL_RANGE");
      return;
    }
    // Do not change the speed/stop mapping while joystick output is active.
    disableJoystickForManualControl();
    servoConfig[index].minUs = (uint16_t)minUs;
    servoConfig[index].centerUs = (uint16_t)centerUs;
    servoConfig[index].maxUs = (uint16_t)maxUs;
    Serial.print("ACKCAL,");
    Serial.print(SERVO_CHANNELS[index]);
    Serial.print(',');
    Serial.print(minUs);
    Serial.print(',');
    Serial.print(centerUs);
    Serial.print(',');
    Serial.println(maxUs);
    return;
  }

  if (strncmp(command, "JOYAREA,", 8) == 0) {
    long area = 0;
    if (!parseNumber(command + 8, area) ||
        area < JOYSTICK_AREA_REMAINING || area >= JOYSTICK_AREA_COUNT) {
      Serial.println("ERR,BAD_JOY_AREA");
      return;
    }
    if (joystickEnabled) stopJoystickMode(joystickArea, joystickMode);
    joystickArea = (uint8_t)area;
    joystickMode = 0;
    joystickNeedsNeutral = true;
    lastJoystickXSpeed = 2001;
    lastJoystickYSpeed = 2001;
    printJoystickTelemetry();
    return;
  }

  if (strncmp(command, "JOY,", 4) == 0) {
    long enabled = 0;
    if (!parseNumber(command + 4, enabled) || (enabled != 0 && enabled != 1)) {
      Serial.println("ERR,BAD_JOY_VALUE");
      return;
    }
    if (enabled == 0) {
      stopJoystickMode(joystickArea, joystickMode);
      joystickEnabled = false;
      joystickNeedsNeutral = true;
    } else {
      stopJoystickMode(joystickArea, joystickMode);
      joystickEnabled = true;
      joystickNeedsNeutral = true;
    }
    printJoystickTelemetry();
    return;
  }

  if (strncmp(command, "SET,", 4) != 0) {
    Serial.println("ERR,BAD_COMMAND");
    return;
  }

  char* chStr = command + 4;
  char* comma = strchr(chStr, ',');
  if (!comma) {
    Serial.println("ERR,BAD_FORMAT");
    return;
  }
  *comma = '\0';
  const char* angleStr = comma + 1;
  long channel = 0, angle = 0;
  if (!parseNumber(chStr, channel) || !parseNumber(angleStr, angle)) {
    Serial.println("ERR,BAD_NUMBER");
    return;
  }
  const int index = findServoIndex(channel);
  if (index < 0 || angle < 0 || angle > 180) {
    Serial.println("ERR,OUT_OF_RANGE");
    return;
  }

  disableJoystickForManualControl();
  writePosition(index, (int)angle);
  printPosition("ACK", index);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  analogReadResolution(10);
  pinMode(FSR_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  calibrateJoystickCenter();
  // Require the operator to release the stick once after power-up. This
  // prevents a physically deflected joystick from starting a servo at speed.
  joystickNeedsNeutral = true;
  joystickButtonRaw = false;
  joystickButtonStable = false;
  joystickButtonChangedAt = millis();
  if (!pca.begin()) {
    Serial.println("ERR,PCA9685_NOT_FOUND");
    // The controller must not pretend to be ready without the driver.
    while (true) delay(1000);
  }
  // The library converts microseconds to 12-bit ticks using this nominal
  // oscillator. A real PCA9685 oscillator can differ; for precision, measure
  // the 50 Hz output with an oscilloscope and replace this value accordingly.
  pca.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
  pca.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
  delay(15);

  // Required initial default position. Can move abruptly on power-up!
  centerAll();
  Serial.print("READY,");
  Serial.println(SERVO_COUNT);
  updateFsr();
  printJoystickTelemetry();
  printFsrTelemetry();
}

void loop() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n') {
      commandBuffer[bufferIndex] = '\0';
      if (bufferIndex > 0) handleCommand(commandBuffer);
      bufferIndex = 0;
    } else if (ch != '\r') {
      if (bufferIndex < sizeof(commandBuffer) - 1) {
        commandBuffer[bufferIndex++] = ch;
      } else {
        bufferIndex = 0;
        Serial.println("ERR,LINE_TOO_LONG");
      }
    }
  }

  const unsigned long now = millis();
  updateJoystickButton(now);
  if (now - lastJoystickUpdateAt >= JOYSTICK_UPDATE_MS) {
    lastJoystickUpdateAt = now;
    updateJoystickControl();
  }
  if (now - lastJoystickTelemetryAt >= JOYSTICK_TELEMETRY_MS) {
    lastJoystickTelemetryAt = now;
    printJoystickTelemetry();
  }
  if (now - lastFsrSampleAt >= FSR_SAMPLE_MS) {
    lastFsrSampleAt = now;
    updateFsr();
  }
  if (now - lastFsrTelemetryAt >= FSR_TELEMETRY_MS) {
    lastFsrTelemetryAt = now;
    printFsrTelemetry();
  }

  // Future feature: read a VL53-series LiDAR over the same I2C bus, at a
  // distinct address, and stream DIST,<millimeters> telemetry.
  // Avoid delay() and blocking sensor calls for responsive servo control.
}
