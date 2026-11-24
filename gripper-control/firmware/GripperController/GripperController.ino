// Dishwasher Robot HT26 - PCA9685 servo + FSR + PAA5100JE optical + VL53L4CD LiDAR controller.
// Board: Arduino UNO R4 WiFi; PCA9685 I2C (Wire): 0x40; VL53L4CD LiDAR I2C (Wire1 / Qwiic).
// FSR inputs (Joy-It SEN-Pressure10): A0 (Finger 1 / ch 5), A1 (Finger 2 / ch 6), A2 (Finger 3 / ch 7).
// Optical flow (PAA5100JE SPI): CS=D10, MOSI=D11, MISO=D12, SCK=D13.
// USB serial protocol at 115200 baud:
//   HELLO / PING
//   SET,<channel>,<angle>     channel 0..7, angle 0..180
//   CAL,<channel>,<min_us>,<center_us>,<max_us>
//   MOTION,<channel>,<enabled>,<max_speed_deg_s>,<acceleration_deg_s2>
//   FSRCAL,<finger_idx>,<contact_thr>,<grip_thr>,<max_thr>,<invert_close>
//   FSRGUARD,<0|1>
//   FSRTARE
//   GRIP,<START|RELEASE|CLOSE_HOLD|OPEN_HOLD|STOP>
//   OPTCAL,<motion_thr>,<squal_thr>
//   OPTLED,<0|1>
//   OPTZERO
//   LOOPCAL,<trigger_mm>,<dwell_ms>,<slip_vel_mm_s>,<release_ms>,<boost_step>,<hold_deadband>
//   AUTOMODE,<auto_lidar_grip>,<closed_loop_hold>
//   CENTER
// Telemetry emitted:
//   FSR,<net0..2>,<raw0..2>,<state0..2>
//   OPT,<dx>,<dy>,<mag>,<total_x>,<total_y>,<state>
//   LIDAR,<dist_mm>,<signed_vel_mm_s>,<lidar_state>,<ctrl_state>,<slip_boost>
//   SLIP,<confidence_0_100>,<slip_state>,<opt_active>,<lidar_active>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Optical_Flow_Sensor.h"
#include <vl53l4cd_class.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint32_t PCA9685_OSCILLATOR_HZ = 25000000UL;
constexpr uint8_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr uint8_t SERVO_COUNT = 8;
constexpr uint8_t SERVO_CHANNELS[SERVO_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7};
constexpr uint8_t CONTINUOUS_SERVO_FIRST_CHANNEL = 5;
constexpr int DEFAULT_ANGLE = 90;
constexpr unsigned long MOTION_UPDATE_MS = 20;
constexpr unsigned long SERIAL_LINE_TIMEOUT_MS = 500;
constexpr unsigned long HOST_INACTIVITY_TIMEOUT_MS = 5000;
constexpr int PULSE_MIN_ALLOWED_US = 600;
constexpr int PULSE_MAX_ALLOWED_US = 2400;
constexpr uint16_t PULSE_UNSET = 0;
constexpr uint16_t PULSE_FULL_OFF = 0xFFFF;

constexpr uint8_t FSR_COUNT = 3;
constexpr uint8_t FSR_PINS[FSR_COUNT] = {A0, A1, A2};
constexpr unsigned long FSR_SAMPLE_MS = 10;
constexpr unsigned long TELEMETRY_REPORT_MS = 100;
constexpr uint16_t DEFAULT_AUTO_GRIP_TIMEOUT_MS = 4000;
constexpr uint16_t MIN_AUTO_GRIP_TIMEOUT_MS = 1000;
constexpr uint16_t MAX_AUTO_GRIP_TIMEOUT_MS = 15000;
constexpr int32_t CLOSING_EQUALIZE_BAND_ADC = 90;
constexpr int32_t CLOSING_EQUALIZE_RESUME_ADC = 50;
constexpr unsigned long HOLD_TIGHTEN_TIMEOUT_MS = 2500;
constexpr unsigned long SLIP_SETTLE_MS = 220;
constexpr unsigned long SLIP_CONFIRM_MS = 120;
constexpr unsigned long SLIP_BOOST_INTERVAL_MS = 350;
constexpr float FSR_EMA_ALPHA = 0.25f;
constexpr int FSR_HYSTERESIS = 30;
constexpr int FSR_MIN_THR = 10;
constexpr int FSR_MAX_THR = 4000;

enum FsrState : uint8_t {
  FSR_NO_CONTACT = 0,
  FSR_CONTACT = 1,
  FSR_TARGET_REACHED = 2,
  FSR_HIGH_PRESSURE = 3,
  FSR_SENSOR_ERROR = 4,
};

constexpr uint8_t PAA5100_CS_PIN = 10;
constexpr unsigned long OPTICAL_SAMPLE_MS = 50;
constexpr unsigned long OPTICAL_HOLD_MS = 250;
constexpr unsigned long OPTICAL_SLIP_HOLD_MS = 120;
constexpr uint8_t OPTICAL_HIST_LEN = 4;
constexpr float OPTICAL_EMA_ALPHA = 0.4f;
constexpr int OPTICAL_MIN_MOTION_THR = 1;
constexpr int OPTICAL_MAX_MOTION_THR = 500;
constexpr int OPTICAL_MAX_SQUAL_THR = 255;

enum OpticalState : uint8_t {
  OPT_STILL = 0,
  OPT_MOVEMENT = 1,
  OPT_LOW_SQUAL = 2,
  OPT_OFFLINE = 3,
};

constexpr unsigned long LIDAR_POLL_MS = 15;
constexpr unsigned long LIDAR_SLIP_HOLD_MS = 300;
constexpr uint8_t LIDAR_SPEED_WINDOW = 8;
constexpr float LIDAR_SPEED_EMA_ALPHA = 0.35f;
constexpr float LIDAR_SPEED_DEADBAND_MM_S = 4.0f;
constexpr uint16_t LIDAR_MIN_VALID_MM = 5;
constexpr uint16_t LIDAR_MAX_VALID_MM = 1200;
constexpr int LIDAR_SLIP_MIN_DISP_MM = 4;
constexpr int LIDAR_REMOVAL_HYST_MM = 25;

enum LidarState : uint8_t {
  LIDAR_CLEAR = 0,
  LIDAR_IN_RANGE = 1,
  LIDAR_SLIP = 2,
  LIDAR_OFFLINE = 3,
};

enum SlipState : uint8_t {
  SLIP_UNAVAILABLE = 0,
  SLIP_STABLE = 1,
  SLIP_SUSPECTED = 2,
  SLIP_CORROBORATED = 3,
  SLIP_OPTICAL_CONFIRMED = 4,
  SLIP_LIDAR_CONFIRMED = 5,
  SLIP_UNLOADED_MOTION = 6,
};

enum GripControllerState : uint8_t {
  GRIP_IDLE = 0,
  GRIP_DWELL = 1,
  GRIP_CLOSING = 2,
  GRIP_HOLDING = 3,
  GRIP_RELEASING = 4,
};

const SPISettings paaSpiSettings(2000000, MSBFIRST, SPI_MODE3);
Optical_Flow_Sensor flow(PAA5100_CS_PIN, PAA5100);
Adafruit_PWMServoDriver pca(PCA9685_ADDRESS);
VL53L4CD lidar(&Wire1, -1);

struct ServoConfig {
  uint16_t minUs;
  uint16_t centerUs;
  uint16_t maxUs;
};

ServoConfig servoConfig[SERVO_COUNT] = {
  {1000, 1500, 2000}, {1000, 1500, 2000}, {1000, 1500, 2000},
  {1000, 1500, 2000}, {1000, 1500, 2000},
  {900, 1500, 2100}, {900, 1500, 2100}, {900, 1500, 2100},
};
int targets[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90};
float currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90};
float currentVelocities[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
bool motionProfileEnabled[SERVO_COUNT] = {true, true, true, true, true, true, true, true};
int maxSpeedDegPerSecond[SERVO_COUNT] = {60, 60, 60, 60, 60, 60, 60, 60};
int accelerationDegPerSecondSquared[SERVO_COUNT] = {120, 120, 120, 120, 120, 120, 120, 120};
uint16_t lastPulseUs[SERVO_COUNT] = {
  PULSE_UNSET, PULSE_UNSET, PULSE_UNSET, PULSE_UNSET,
  PULSE_UNSET, PULSE_UNSET, PULSE_UNSET, PULSE_UNSET,
};

uint16_t fsrRaw[FSR_COUNT] = {0, 0, 0};
float fsrFiltered[FSR_COUNT] = {0, 0, 0};
bool fsrInitialized[FSR_COUNT] = {false, false, false};
uint16_t fsrZeroOffset[FSR_COUNT] = {0, 0, 0};
uint16_t fsrNet[FSR_COUNT] = {0, 0, 0};
uint8_t fsrState[FSR_COUNT] = {FSR_NO_CONTACT, FSR_NO_CONTACT, FSR_NO_CONTACT};
uint16_t fsrContactThreshold[FSR_COUNT] = {150, 150, 150};
uint16_t fsrGripThreshold[FSR_COUNT] = {600, 600, 600};
uint16_t fsrMaxThreshold[FSR_COUNT] = {1800, 1800, 1800};
bool fingerInvertClose[FSR_COUNT] = {false, false, false};
bool fsrGuardEnabled = true;

bool opticalSensorOnline = false;
bool opticalLedEnabled = true;
int32_t optAccumX = 0;
int32_t optAccumY = 0;
long optTotalX = 0;
long optTotalY = 0;
float optFilteredMag = 0.0f;
uint16_t optMag = 0;
uint8_t optSqual = 0;
uint8_t optState = OPT_OFFLINE;
uint16_t optMotionThreshold = 3;
uint8_t optSqualThreshold = 0;
int16_t optHistDx[OPTICAL_HIST_LEN] = {0, 0, 0, 0};
int16_t optHistDy[OPTICAL_HIST_LEN] = {0, 0, 0, 0};
uint8_t optHistHead = 0;
uint8_t optHistCount = 0;
bool optMotionActiveNow = false;
unsigned long lastOpticalSampleAt = 0;
unsigned long lastOpticalMotionAt = 0;

bool lidarOnline = false;
bool lidarValid = false;
uint16_t lidarDistanceMm = 0;
float lidarFilteredMm = 0.0f;
float lidarVelEmaSignedMmS = 0.0f;
int16_t lidarSignedVelMmS = 0;
uint16_t lidarVelocityMmS = 0;
uint8_t lidarState = LIDAR_OFFLINE;
float lidarHistDist[LIDAR_SPEED_WINDOW] = {0};
unsigned long lidarHistTime[LIDAR_SPEED_WINDOW] = {0};
uint8_t lidarHistHead = 0;
uint8_t lidarHistCount = 0;
unsigned long lastLidarPollAt = 0;
unsigned long lastLidarSampleAt = 0;
unsigned long lastLidarSlipAt = 0;

// Slip sensor fusion state.
uint8_t slipConfidence = 0;
uint8_t slipState = SLIP_UNAVAILABLE;
bool slipOpticalActive = false;
bool slipLidarActive = false;
unsigned long slipCandidateSince = 0;

// Cascaded Two-Loop Smart Grip Controller state.
uint8_t gripControllerState = GRIP_IDLE;
bool autoLidarGripEnabled = false;
bool closedLoopHoldEnabled = true;
bool waitForObjectRemoval = false;
uint16_t lidarTriggerMm = 60;
uint16_t lidarDwellMs = 400;
uint16_t lidarSlipVelocityThr = 40;
uint16_t releaseDurationMs = 1200;
uint16_t slipBoostStep = 100;
uint16_t forceHoldDeadband = 60;
uint16_t slipForceBoost = 0;
uint16_t gripHoldLidarMm = 0;
uint16_t autoGripTimeoutMs = DEFAULT_AUTO_GRIP_TIMEOUT_MS;
unsigned long lidarDwellStartedAt = 0;
unsigned long autoGripStartedAt = 0;
unsigned long firstContactAt = 0;
unsigned long releaseStartedAt = 0;
unsigned long lastTightenAt = 0;
unsigned long lastSlipBoostAt = 0;
unsigned long lastMotorMoveAt = 0;
bool boostPendingTighten = false;
bool closingContactStage = true;
bool fingerAtTarget[FSR_COUNT] = {false, false, false};
bool fingerParticipating[FSR_COUNT] = {true, true, true};
bool fingerEqualizePaused[FSR_COUNT] = {false, false, false};
uint16_t gripHoldBaseTarget[FSR_COUNT] = {600, 600, 600};
unsigned long fingerTightenStartedAt[FSR_COUNT] = {0, 0, 0};

unsigned long lastMotionUpdateAt = 0;
unsigned long lastFsrSampleAt = 0;
unsigned long lastTelemetryReportAt = 0;
unsigned long lastSerialByteAt = 0;
bool hostConnected = false;
unsigned long lastHostActivityAt = 0;
char commandBuffer[96];
uint8_t bufferIndex = 0;

bool parseNumber(const char* text, long& value) {
  if (!text || !*text) return false;
  char* end = nullptr;
  value = strtol(text, &end, 10);
  return *end == '\0';
}

int findServoIndex(long channel) {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (channel == SERVO_CHANNELS[i]) return i;
  }
  return -1;
}

bool isContinuousServo(int index) {
  return index >= 0 && index < SERVO_COUNT &&
         SERVO_CHANNELS[index] >= CONTINUOUS_SERVO_FIRST_CHANNEL;
}

int fingerIndexForServo(int servoIndex) {
  if (!isContinuousServo(servoIndex)) return -1;
  const int f = (int)SERVO_CHANNELS[servoIndex] - (int)CONTINUOUS_SERVO_FIRST_CHANNEL;
  return (f >= 0 && f < FSR_COUNT) ? f : -1;
}

int closingAngleForFinger(int f) {
  return fingerInvertClose[f] ? 0 : 180;
}

int openingAngleForFinger(int f) {
  return fingerInvertClose[f] ? 180 : 0;
}

bool isFingerClosing(int f, int angle) {
  if (angle == DEFAULT_ANGLE) return false;
  return fingerInvertClose[f] ? (angle < DEFAULT_ANGLE) : (angle > DEFAULT_ANGLE);
}

uint16_t activeTargetForceForFinger(uint8_t f) {
  const uint16_t base = (gripControllerState == GRIP_HOLDING) ? gripHoldBaseTarget[f] : fsrGripThreshold[f];
  uint32_t target = (uint32_t)base + (uint32_t)slipForceBoost;
  if (target > fsrMaxThreshold[f]) target = fsrMaxThreshold[f];
  return (uint16_t)target;
}

int proportionalClosingAngleForFinger(int f, int32_t error, uint16_t refForce) {
  if (gripControllerState == GRIP_CLOSING && fsrNet[f] < fsrContactThreshold[f]) {
    return closingAngleForFinger(f);
  }
  float ratio = (refForce > 0) ? ((float)error / (float)refForce) : 1.0f;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  int offset = (int)lroundf(20.0f + 70.0f * ratio);
  if (offset < 22) offset = 22;
  if (offset > 90) offset = 90;
  return fingerInvertClose[f] ? (DEFAULT_ANGLE - offset) : (DEFAULT_ANGLE + offset);
}

bool isFingerAtGripLimit(int f) {
  const uint16_t targetForce = activeTargetForceForFinger((uint8_t)f);
  return fsrNet[f] >= targetForce ||
         fsrState[f] == FSR_HIGH_PRESSURE ||
         fsrState[f] == FSR_SENSOR_ERROR;
}

void recoverI2CBus() {
#if defined(SDA) && defined(SCL)
  // Clock out any stuck slave transaction before Wire takes over the pins.
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(SDA) == LOW) {
    for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; ++i) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(10);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(10);
    }
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
    delayMicroseconds(10);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(10);
    pinMode(SDA, INPUT_PULLUP);
    delayMicroseconds(10);
  }
#endif
}

void stopContinuousServo(int index) {
  // Full-off stops the continuous rotation servos without a center pulse.
  if (lastPulseUs[index] == PULSE_FULL_OFF) return;
  pca.setPWM(SERVO_CHANNELS[index], 0, 4096);
  lastPulseUs[index] = PULSE_FULL_OFF;
}

void writePulseForAngle(int index, float angle) {
  const ServoConfig& cal = servoConfig[index];
  float pulseUs;
  if (angle <= DEFAULT_ANGLE) {
    pulseUs = cal.minUs + ((float)(cal.centerUs - cal.minUs) * angle / 90.0f);
  } else {
    pulseUs = cal.centerUs +
              ((float)(cal.maxUs - cal.centerUs) * (angle - DEFAULT_ANGLE) / 90.0f);
  }
  const uint16_t targetUs = (uint16_t)lroundf(pulseUs);
  if (lastPulseUs[index] == targetUs) return;
  pca.writeMicroseconds(SERVO_CHANNELS[index], targetUs);
  lastPulseUs[index] = targetUs;
}

void writeTargetImmediately(int index) {
  currentAngles[index] = targets[index];
  currentVelocities[index] = 0;
  if (isContinuousServo(index) && targets[index] == DEFAULT_ANGLE) {
    stopContinuousServo(index);
  } else {
    writePulseForAngle(index, targets[index]);
  }
}

void printPosition(const char* prefix, int index) {
  Serial.print(prefix);
  Serial.print(',');
  Serial.print(SERVO_CHANNELS[index]);
  Serial.print(',');
  Serial.println(targets[index]);
}

void resetOpticalMotionWindow() {
  for (uint8_t i = 0; i < OPTICAL_HIST_LEN; ++i) {
    optHistDx[i] = 0;
    optHistDy[i] = 0;
  }
  optHistHead = 0;
  optHistCount = 0;
  optFilteredMag = 0.0f;
  optMag = 0;
  optMotionActiveNow = false;
  lastOpticalMotionAt = 0;
}

void setTarget(int index, int angle) {
  const int f = fingerIndexForServo(index);
  if (f >= 0 && fsrGuardEnabled && isFingerClosing(f, angle) && isFingerAtGripLimit(f)) {
    angle = DEFAULT_ANGLE;
    Serial.print("GRIPSTOP,");
    Serial.print(f);
    Serial.print(',');
    Serial.println(fsrNet[f]);
  }
  targets[index] = angle;
  if (f >= 0 && angle != DEFAULT_ANGLE) {
    lastMotorMoveAt = millis();
  }
  // Continuous servos always stop immediately when commanded to stop.
  if (isContinuousServo(index) && angle == DEFAULT_ANGLE) {
    writeTargetImmediately(index);
  } else if (!motionProfileEnabled[index]) {
    writeTargetImmediately(index);
  }
}

void cancelSmartController() {
  gripControllerState = GRIP_IDLE;
  slipForceBoost = 0;
  lastSlipBoostAt = 0;
  gripHoldLidarMm = 0;
  slipCandidateSince = 0;
  firstContactAt = 0;
  closingContactStage = true;
  boostPendingTighten = false;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    fingerAtTarget[f] = false;
    fingerParticipating[f] = true;
    fingerEqualizePaused[f] = false;
    gripHoldBaseTarget[f] = fsrGripThreshold[f];
    fingerTightenStartedAt[f] = 0;
  }
}

void recomputeFsrNet(uint8_t f) {
  if (fsrZeroOffset[f] > 2500) {
    fsrNet[f] = (fsrFiltered[f] < (float)fsrZeroOffset[f])
                    ? (uint16_t)lroundf((float)fsrZeroOffset[f] - fsrFiltered[f])
                    : 0;
  } else {
    fsrNet[f] = (fsrFiltered[f] > (float)fsrZeroOffset[f])
                    ? (uint16_t)lroundf(fsrFiltered[f] - (float)fsrZeroOffset[f])
                    : 0;
  }
}

void updateFsrState(uint8_t f) {
  const bool pullUpMode = fsrZeroOffset[f] > 2500;
  const bool invalidThresholds =
      fsrContactThreshold[f] >= fsrGripThreshold[f] ||
      fsrGripThreshold[f] >= fsrMaxThreshold[f];
  const bool railFault =
      fsrZeroOffset[f] >= 4090 ||
      (!pullUpMode && fsrRaw[f] >= 4090) ||
      (pullUpMode && fsrRaw[f] <= 5);
  if (invalidThresholds || railFault) {
    fsrState[f] = FSR_SENSOR_ERROR;
    return;
  }

  const int32_t net = fsrNet[f];
  const uint8_t prev = fsrState[f];
  const int32_t contactLow = (int32_t)fsrContactThreshold[f] - FSR_HYSTERESIS;
  const int32_t gripLow = (int32_t)fsrGripThreshold[f] - FSR_HYSTERESIS;
  const int32_t maxLow = (int32_t)fsrMaxThreshold[f] - FSR_HYSTERESIS;

  if (net >= fsrMaxThreshold[f] || (prev == FSR_HIGH_PRESSURE && net >= maxLow)) {
    fsrState[f] = FSR_HIGH_PRESSURE;
  } else if (net >= fsrGripThreshold[f] ||
             (prev >= FSR_TARGET_REACHED && prev != FSR_SENSOR_ERROR && net >= gripLow)) {
    fsrState[f] = FSR_TARGET_REACHED;
  } else if (net >= fsrContactThreshold[f] ||
             (prev >= FSR_CONTACT && prev != FSR_SENSOR_ERROR && net >= contactLow)) {
    fsrState[f] = FSR_CONTACT;
  } else {
    fsrState[f] = FSR_NO_CONTACT;
  }
}

void sampleFsrSensors(unsigned long now, bool forceImmediate) {
  if (!forceImmediate && (now - lastFsrSampleAt < FSR_SAMPLE_MS)) return;
  lastFsrSampleAt = now;

  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const uint16_t raw = (uint16_t)analogRead(FSR_PINS[f]);
    fsrRaw[f] = raw;
    if (!fsrInitialized[f]) {
      fsrFiltered[f] = (float)raw;
      fsrInitialized[f] = true;
    } else {
      fsrFiltered[f] = (FSR_EMA_ALPHA * (float)raw) +
                       ((1.0f - FSR_EMA_ALPHA) * fsrFiltered[f]);
    }
    recomputeFsrNet(f);
    updateFsrState(f);
  }

  if (forceImmediate) return;

  // In manual modes (IDLE), still enforce per-finger FSR safety stop when closing.
  if (gripControllerState == GRIP_IDLE && fsrGuardEnabled) {
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      if (isFingerClosing(f, targets[servoIdx]) && isFingerAtGripLimit(f)) {
        targets[servoIdx] = DEFAULT_ANGLE;
        writeTargetImmediately(servoIdx);
        printPosition("ACK", servoIdx);
        Serial.print("GRIPSTOP,");
        Serial.print(f);
        Serial.print(',');
        Serial.println(fsrNet[f]);
      }
    }
  }
}

void paaWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(paaSpiSettings);
  digitalWrite(PAA5100_CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  delayMicroseconds(50);
  digitalWrite(PAA5100_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
}

uint8_t paaReadReg(uint8_t reg) {
  SPI.beginTransaction(paaSpiSettings);
  digitalWrite(PAA5100_CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(reg & 0x7F);
  delayMicroseconds(50);
  uint8_t val = SPI.transfer(0x00);
  delayMicroseconds(50);
  digitalWrite(PAA5100_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
  return val;
}

void setOpticalLed(bool enabled) {
  if (!opticalSensorOnline) return;
  paaWriteReg(0x7F, 0x14);
  paaWriteReg(0x6F, enabled ? 0x1C : 0x00);
  paaWriteReg(0x7F, 0x00);
}

bool initOpticalSensor() {
  opticalSensorOnline = flow.begin();
  resetOpticalMotionWindow();
  if (!opticalSensorOnline) {
    optState = OPT_OFFLINE;
    optAccumX = 0;
    optAccumY = 0;
    optSqual = 0;
    return false;
  }
  if (!opticalLedEnabled) {
    setOpticalLed(false);
  }
  optState = OPT_STILL;
  return true;
}

void evaluateOpticalState(unsigned long now, float rawMag, float vecMag) {
  if (!opticalSensorOnline) {
    optState = OPT_OFFLINE;
    optMotionActiveNow = false;
    return;
  }
  if (optSqualThreshold > 0 && optSqual < optSqualThreshold) {
    optState = OPT_LOW_SQUAL;
    optMotionActiveNow = false;
    return;
  }
  // Directional vector accumulation rejects stationary back-and-forth jitter
  // while accumulating slow, consistent slip; fast single spikes trigger at +1 margin.
  const bool aboveThreshold =
      (vecMag >= (float)optMotionThreshold) ||
      (rawMag >= (float)(optMotionThreshold + 1));
  optMotionActiveNow = aboveThreshold;
  if (aboveThreshold) {
    lastOpticalMotionAt = now;
    optState = OPT_MOVEMENT;
  } else if (lastOpticalMotionAt != 0 && (now - lastOpticalMotionAt < OPTICAL_HOLD_MS)) {
    optState = OPT_MOVEMENT;
  } else {
    optState = OPT_STILL;
  }
}

void sampleOpticalSensor(unsigned long now) {
  if (now - lastOpticalSampleAt < OPTICAL_SAMPLE_MS) return;
  lastOpticalSampleAt = now;
  if (!opticalSensorOnline) {
    optState = OPT_OFFLINE;
    optMotionActiveNow = false;
    return;
  }

  int16_t dx = 0;
  int16_t dy = 0;
  flow.readMotionCount(&dx, &dy);
  if (optSqualThreshold > 0) {
    optSqual = paaReadReg(0x07);
  }

  optAccumX += dx;
  optAccumY += dy;
  optTotalX += dx;
  optTotalY += dy;

  optHistDx[optHistHead] = dx;
  optHistDy[optHistHead] = dy;
  optHistHead = (optHistHead + 1) % OPTICAL_HIST_LEN;
  if (optHistCount < OPTICAL_HIST_LEN) ++optHistCount;

  long sumX = 0;
  long sumY = 0;
  for (uint8_t i = 0; i < optHistCount; ++i) {
    sumX += optHistDx[i];
    sumY += optHistDy[i];
  }

  const float rawMag = sqrtf((float)dx * (float)dx + (float)dy * (float)dy);
  const float vecMag = sqrtf((float)sumX * (float)sumX + (float)sumY * (float)sumY);
  optFilteredMag = (OPTICAL_EMA_ALPHA * vecMag) +
                   ((1.0f - OPTICAL_EMA_ALPHA) * optFilteredMag);
  optMag = (uint16_t)lroundf(fmaxf(rawMag, fmaxf(vecMag, optFilteredMag)));
  evaluateOpticalState(now, rawMag, vecMag);
}

bool initLidarSensor() {
  Wire1.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire1.setWireTimeout(3000, true);
#endif
  lidar.begin();
  lidar.VL53L4CD_Off();
  if (lidar.InitSensor() != 0) {
    lidarOnline = false;
    lidarValid = false;
    lidarDistanceMm = 0;
    lidarSignedVelMmS = 0;
    lidarVelocityMmS = 0;
    lidarHistCount = 0;
    lidarHistHead = 0;
    lidarState = LIDAR_OFFLINE;
    return false;
  }
  lidar.VL53L4CD_SetRangeTiming(50, 0);
  lidar.VL53L4CD_StartRanging();
  lidarOnline = true;
  lidarValid = false;
  lidarDistanceMm = 0;
  lidarFilteredMm = 0.0f;
  lidarVelEmaSignedMmS = 0.0f;
  lidarSignedVelMmS = 0;
  lidarVelocityMmS = 0;
  lidarHistCount = 0;
  lidarHistHead = 0;
  lidarState = LIDAR_CLEAR;
  return true;
}

float lidarSlopeMmPerSecond() {
  if (lidarHistCount < 3) return 0.0f;
  const uint8_t newest = (lidarHistHead + LIDAR_SPEED_WINDOW - 1) % LIDAR_SPEED_WINDOW;
  const unsigned long newestT = lidarHistTime[newest];

  float sumT = 0.0f;
  float sumD = 0.0f;
  for (uint8_t i = 0; i < lidarHistCount; ++i) {
    const uint8_t idx = (newest + LIDAR_SPEED_WINDOW - i) % LIDAR_SPEED_WINDOW;
    sumT += -((float)(newestT - lidarHistTime[idx])) / 1000.0f;
    sumD += lidarHistDist[idx];
  }
  const float meanT = sumT / (float)lidarHistCount;
  const float meanD = sumD / (float)lidarHistCount;

  float num = 0.0f;
  float den = 0.0f;
  for (uint8_t i = 0; i < lidarHistCount; ++i) {
    const uint8_t idx = (newest + LIDAR_SPEED_WINDOW - i) % LIDAR_SPEED_WINDOW;
    const float t = -((float)(newestT - lidarHistTime[idx])) / 1000.0f;
    num += (t - meanT) * (lidarHistDist[idx] - meanD);
    den += (t - meanT) * (t - meanT);
  }
  return (den > 1e-6f) ? (num / den) : 0.0f;
}

void sampleLidarSensor(unsigned long now) {
  if (!lidarOnline) {
    lidarState = LIDAR_OFFLINE;
    return;
  }
  if (now - lastLidarPollAt < LIDAR_POLL_MS) return;
  lastLidarPollAt = now;

  uint8_t dataReady = 0;
  const uint8_t status = lidar.VL53L4CD_CheckForDataReady(&dataReady);
  if (status != 0 || !dataReady) return;

  VL53L4CD_Result_t results;
  lidar.VL53L4CD_ClearInterrupt();
  lidar.VL53L4CD_GetResult(&results);

  const bool ok = (results.range_status == 0) &&
                  (results.distance_mm >= LIDAR_MIN_VALID_MM) &&
                  (results.distance_mm <= LIDAR_MAX_VALID_MM);
  if (ok) {
    const float rawDist = (float)results.distance_mm;
    if (!lidarValid || lastLidarSampleAt == 0) {
      lidarFilteredMm = rawDist;
      lidarVelEmaSignedMmS = 0.0f;
      lidarHistCount = 0;
      lidarHistHead = 0;
    } else {
      lidarFilteredMm = (0.45f * rawDist) + (0.55f * lidarFilteredMm);
    }

    lidarHistDist[lidarHistHead] = rawDist;
    lidarHistTime[lidarHistHead] = now;
    lidarHistHead = (lidarHistHead + 1) % LIDAR_SPEED_WINDOW;
    if (lidarHistCount < LIDAR_SPEED_WINDOW) ++lidarHistCount;

    const float slope = lidarSlopeMmPerSecond();
    lidarVelEmaSignedMmS = (LIDAR_SPEED_EMA_ALPHA * slope) +
                           ((1.0f - LIDAR_SPEED_EMA_ALPHA) * lidarVelEmaSignedMmS);
    if (fabsf(lidarVelEmaSignedMmS) < LIDAR_SPEED_DEADBAND_MM_S) {
      lidarSignedVelMmS = 0;
    } else {
      lidarSignedVelMmS = (int16_t)lroundf(lidarVelEmaSignedMmS);
    }

    lidarValid = true;
    lidarDistanceMm = (uint16_t)lroundf(lidarFilteredMm);
    lidarVelocityMmS = (uint16_t)abs((int)lidarSignedVelMmS);
    lastLidarSampleAt = now;
  } else {
    lidarValid = false;
    lidarVelEmaSignedMmS *= 0.5f;
    lidarSignedVelMmS = 0;
    lidarVelocityMmS = 0;
    lidarHistCount = 0;
    lidarHistHead = 0;
  }

  if (lastLidarSlipAt != 0 && (now - lastLidarSlipAt < LIDAR_SLIP_HOLD_MS)) {
    lidarState = LIDAR_SLIP;
  } else if (lidarValid && lidarDistanceMm <= lidarTriggerMm) {
    lidarState = LIDAR_IN_RANGE;
  } else {
    lidarState = LIDAR_CLEAR;
  }
}

void updateSlipDetection(unsigned long now) {
  bool anyMoving = (gripControllerState == GRIP_CLOSING) || (gripControllerState == GRIP_RELEASING);
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) {
      anyMoving = true;
      break;
    }
  }
  if (anyMoving) {
    lastMotorMoveAt = now;
  }

  bool anyContact = false;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    if (fsrState[f] != FSR_SENSOR_ERROR && fsrNet[f] >= fsrContactThreshold[f]) {
      anyContact = true;
      break;
    }
  }

  const bool opticalAvailable = opticalSensorOnline && (optState != OPT_LOW_SQUAL) && (optState != OPT_OFFLINE);
  const bool lidarAvailable = lidarOnline && lidarValid;
  const bool optFreshSlip = opticalAvailable &&
                            (optMotionActiveNow || (lastOpticalMotionAt != 0 && (now - lastOpticalMotionAt <= OPTICAL_SLIP_HOLD_MS)));
  const bool lidarDispOk = !anyContact || (gripHoldLidarMm == 0) ||
                           (abs((int)lidarDistanceMm - (int)gripHoldLidarMm) >= LIDAR_SLIP_MIN_DISP_MM);
  const bool lidarFreshSlip = lidarAvailable &&
                              (lidarVelocityMmS >= lidarSlipVelocityThr) &&
                              lidarDispOk;

  const bool inSettle = anyMoving || (lastMotorMoveAt != 0 && (now - lastMotorMoveAt < SLIP_SETTLE_MS));
  if (inSettle) {
    if (lidarAvailable && gripControllerState == GRIP_HOLDING) {
      gripHoldLidarMm = lidarDistanceMm;
    }
    slipOpticalActive = false;
    slipLidarActive = false;
    slipConfidence = 0;
    slipState = (opticalAvailable || lidarAvailable) ? SLIP_STABLE : SLIP_UNAVAILABLE;
    slipCandidateSince = 0;
    return;
  }

  slipOpticalActive = optFreshSlip;
  slipLidarActive = lidarFreshSlip;

  if (!opticalAvailable && !lidarAvailable) {
    slipConfidence = 0;
    slipState = SLIP_UNAVAILABLE;
    slipCandidateSince = 0;
    return;
  }

  if (!anyContact) {
    slipConfidence = 0;
    slipState = (slipOpticalActive || slipLidarActive) ? SLIP_UNLOADED_MOTION : SLIP_STABLE;
    slipCandidateSince = 0;
    return;
  }

  slipConfidence = (slipOpticalActive ? 60 : 0) + (slipLidarActive ? 40 : 0);
  if (!slipOpticalActive && !slipLidarActive) {
    slipState = SLIP_STABLE;
    slipCandidateSince = 0;
    return;
  }

  if (slipCandidateSince == 0) {
    slipCandidateSince = now;
  }
  if (now - slipCandidateSince < SLIP_CONFIRM_MS) {
    slipState = SLIP_SUSPECTED;
  } else if (slipOpticalActive && slipLidarActive) {
    slipState = SLIP_CORROBORATED;
  } else if (slipOpticalActive) {
    slipState = SLIP_OPTICAL_CONFIRMED;
  } else {
    slipState = SLIP_LIDAR_CONFIRMED;
  }
}

void startClosingCycle(unsigned long now) {
  gripControllerState = GRIP_CLOSING;
  slipForceBoost = 0;
  lastSlipBoostAt = 0;
  boostPendingTighten = false;
  slipCandidateSince = 0;
  autoGripStartedAt = now;
  firstContactAt = 0;
  closingContactStage = true;
  bool anyActive = false;

  bool hasContact[FSR_COUNT] = {false, false, false};
  bool anyContact = false;
  bool allContact = true;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    fingerTightenStartedAt[f] = 0;
    fingerParticipating[f] = true;
    fingerEqualizePaused[f] = false;
    gripHoldBaseTarget[f] = fsrGripThreshold[f];
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx < 0) continue;
    hasContact[f] = (fsrState[f] != FSR_SENSOR_ERROR && fsrNet[f] >= fsrContactThreshold[f]);
    if (hasContact[f]) {
      anyContact = true;
    } else {
      allContact = false;
    }
  }
  if (anyContact) firstContactAt = now;
  if (allContact) closingContactStage = false;

  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx < 0) {
      fingerAtTarget[f] = true;
      continue;
    }
    const uint16_t refForce = activeTargetForceForFinger(f);
    if (fsrNet[f] >= refForce || fsrState[f] >= FSR_TARGET_REACHED) {
      fingerAtTarget[f] = true;
      setTarget(servoIdx, DEFAULT_ANGLE);
    } else if (closingContactStage && hasContact[f]) {
      // Wait at contact threshold until other fingers catch up or contact timeout elapses.
      fingerAtTarget[f] = false;
      anyActive = true;
      setTarget(servoIdx, DEFAULT_ANGLE);
    } else {
      fingerAtTarget[f] = false;
      anyActive = true;
      const int cmdAngle = closingContactStage
                               ? closingAngleForFinger(f)
                               : proportionalClosingAngleForFinger(
                                     f, (int32_t)refForce - (int32_t)fsrNet[f], refForce);
      setTarget(servoIdx, cmdAngle);
    }
    printPosition("ACK", servoIdx);
  }

  if (!anyActive) {
    lastTightenAt = now;
    lastSlipBoostAt = now;
    lastMotorMoveAt = now;
    resetOpticalMotionWindow();
    gripHoldLidarMm = lidarValid ? lidarDistanceMm : 0;
    if (closedLoopHoldEnabled) {
      gripControllerState = GRIP_HOLDING;
      Serial.println("GRIPSTATE,HOLDING");
    } else {
      gripControllerState = GRIP_IDLE;
      Serial.println("GRIPSTATE,DONE");
    }
  } else {
    Serial.println("GRIPSTATE,CLOSING");
  }
}

void startTimedRelease(unsigned long now) {
  cancelSmartController();
  gripControllerState = GRIP_RELEASING;
  releaseStartedAt = now;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx < 0) continue;
    setTarget(servoIdx, openingAngleForFinger(f));
    printPosition("ACK", servoIdx);
  }
  Serial.println("GRIPSTATE,RELEASING");
}

void updateSmartGripController(unsigned long now) {
  if (gripControllerState == GRIP_IDLE) {
    if (waitForObjectRemoval) {
      if (!lidarOnline || !lidarValid || lidarDistanceMm > (lidarTriggerMm + LIDAR_REMOVAL_HYST_MM)) {
        waitForObjectRemoval = false;
      }
    }
    if (autoLidarGripEnabled && !waitForObjectRemoval && lidarOnline && lidarValid &&
        lidarDistanceMm <= lidarTriggerMm) {
      gripControllerState = GRIP_DWELL;
      lidarDwellStartedAt = now;
      Serial.println("GRIPSTATE,DWELL");
    }
    return;
  }

  if (gripControllerState == GRIP_DWELL) {
    if (!autoLidarGripEnabled || !lidarOnline || !lidarValid || lidarDistanceMm > lidarTriggerMm) {
      gripControllerState = GRIP_IDLE;
      Serial.println("GRIPSTATE,IDLE");
      return;
    }
    if (now - lidarDwellStartedAt >= lidarDwellMs) {
      startClosingCycle(now);
    }
    return;
  }

  if (gripControllerState == GRIP_RELEASING) {
    if (now - releaseStartedAt >= releaseDurationMs) {
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx >= 0) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      }
      slipForceBoost = 0;
      boostPendingTighten = false;
      waitForObjectRemoval = true;
      gripControllerState = GRIP_IDLE;
      Serial.println("GRIPSTATE,RELEASED");
    }
    return;
  }

  if (gripControllerState == GRIP_CLOSING) {
    bool hasContact[FSR_COUNT] = {false, false, false};
    bool anyContact = false;
    bool allContact = true;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      hasContact[f] = (fsrState[f] != FSR_SENSOR_ERROR && fsrNet[f] >= fsrContactThreshold[f]);
      if (hasContact[f]) {
        anyContact = true;
      } else {
        allContact = false;
      }
    }
    if (anyContact && firstContactAt == 0) {
      firstContactAt = now;
    }

    if (now - autoGripStartedAt >= autoGripTimeoutMs) {
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx >= 0) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      }
      if (!anyContact) {
        cancelSmartController();
        Serial.println("GRIPSTATE,TIMEOUT");
        return;
      }
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        fingerParticipating[f] = hasContact[f];
        fingerAtTarget[f] = true;
        fingerEqualizePaused[f] = false;
        fingerTightenStartedAt[f] = 0;
        if (hasContact[f]) {
          gripHoldBaseTarget[f] =
              (fsrNet[f] < fsrGripThreshold[f]) ? fsrNet[f] : fsrGripThreshold[f];
        } else {
          gripHoldBaseTarget[f] = fsrGripThreshold[f];
        }
      }
      closingContactStage = false;
      lastTightenAt = now;
      lastSlipBoostAt = now;
      lastMotorMoveAt = now;
      resetOpticalMotionWindow();
      gripHoldLidarMm = lidarValid ? lidarDistanceMm : 0;
      if (closedLoopHoldEnabled) {
        gripControllerState = GRIP_HOLDING;
        Serial.println("GRIPSTATE,HOLDING");
      } else {
        cancelSmartController();
        Serial.println("GRIPSTATE,DONE");
      }
      return;
    }

    if (closingContactStage) {
      unsigned long contactWaitLimit = autoGripTimeoutMs / 2;
      if (contactWaitLimit > 1800UL) contactWaitLimit = 1800UL;
      if (contactWaitLimit < 450UL) contactWaitLimit = 450UL;
      const unsigned long stage1Deadline = ((unsigned long)autoGripTimeoutMs * 65UL) / 100UL;

      if (allContact) {
        closingContactStage = false;
      } else if (anyContact &&
                 ((now - firstContactAt >= contactWaitLimit) ||
                  (now - autoGripStartedAt >= stage1Deadline))) {
        closingContactStage = false;
        for (uint8_t f = 0; f < FSR_COUNT; ++f) {
          if (hasContact[f]) {
            fingerParticipating[f] = true;
          } else {
            fingerParticipating[f] = false;
            fingerAtTarget[f] = true;
            const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
            if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) {
              targets[servoIdx] = DEFAULT_ANGLE;
              writeTargetImmediately(servoIdx);
              printPosition("ACK", servoIdx);
            }
          }
        }
      }
    }

    if (closingContactStage) {
      // Stage 1: Bring all fingers to Contact Thr and wait briefly so forces stay balanced.
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx < 0) continue;
        const uint16_t contactResume = (fsrContactThreshold[f] > FSR_HYSTERESIS)
                                           ? (fsrContactThreshold[f] - FSR_HYSTERESIS)
                                           : (fsrContactThreshold[f] / 2);
        if (hasContact[f] || fsrState[f] == FSR_HIGH_PRESSURE || fsrState[f] == FSR_SENSOR_ERROR) {
          if (targets[servoIdx] != DEFAULT_ANGLE) {
            targets[servoIdx] = DEFAULT_ANGLE;
            writeTargetImmediately(servoIdx);
            printPosition("ACK", servoIdx);
          }
        } else if (targets[servoIdx] != DEFAULT_ANGLE || fsrNet[f] <= contactResume) {
          const int cmd = closingAngleForFinger(f);
          if (targets[servoIdx] != cmd) {
            setTarget(servoIdx, cmd);
            printPosition("ACK", servoIdx);
          }
        }
      }
      return;
    }

    // Stage 2: Tighten participating fingers evenly up to Grip Stop Thr.
    bool allDone = true;
    uint16_t minActiveForce = 0xFFFF;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (!fingerParticipating[f]) continue;
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      const uint16_t refForce = activeTargetForceForFinger(f);
      const bool reached = (fsrNet[f] >= refForce) ||
                           (fsrState[f] == FSR_HIGH_PRESSURE) ||
                           (fsrState[f] == FSR_SENSOR_ERROR);
      if (reached) {
        if (!fingerAtTarget[f] || targets[servoIdx] != DEFAULT_ANGLE) {
          fingerAtTarget[f] = true;
          fingerEqualizePaused[f] = false;
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
          Serial.print("GRIPSTOP,");
          Serial.print(f);
          Serial.print(',');
          Serial.println(fsrNet[f]);
        }
      } else if (!fingerAtTarget[f] && fsrNet[f] < minActiveForce) {
        minActiveForce = fsrNet[f];
      }
    }

    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (!fingerParticipating[f] || fingerAtTarget[f]) continue;
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      allDone = false;
      const uint16_t refForce = activeTargetForceForFinger(f);
      const int32_t lead = (minActiveForce == 0xFFFF)
                               ? 0
                               : ((int32_t)fsrNet[f] - (int32_t)minActiveForce);
      if (lead >= CLOSING_EQUALIZE_BAND_ADC) {
        fingerEqualizePaused[f] = true;
      } else if (lead <= CLOSING_EQUALIZE_RESUME_ADC) {
        fingerEqualizePaused[f] = false;
      }

      if (fingerEqualizePaused[f]) {
        if (targets[servoIdx] != DEFAULT_ANGLE) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      } else {
        int32_t effError = (int32_t)refForce - (int32_t)fsrNet[f] - (lead / 2);
        if (effError < 1) effError = 1;
        const int cmd = proportionalClosingAngleForFinger(f, effError, refForce);
        if (targets[servoIdx] != cmd) {
          setTarget(servoIdx, cmd);
          printPosition("ACK", servoIdx);
        }
      }
    }

    if (allDone) {
      lastTightenAt = now;
      lastSlipBoostAt = now;
      lastMotorMoveAt = now;
      resetOpticalMotionWindow();
      gripHoldLidarMm = lidarValid ? lidarDistanceMm : 0;
      if (closedLoopHoldEnabled) {
        gripControllerState = GRIP_HOLDING;
        Serial.println("GRIPSTATE,HOLDING");
      } else {
        cancelSmartController();
        Serial.println("GRIPSTATE,DONE");
      }
    }
    return;
  }

  if (gripControllerState == GRIP_HOLDING) {
    if (!closedLoopHoldEnabled) {
      cancelSmartController();
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      }
      Serial.println("GRIPSTATE,DONE");
      return;
    }

    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (!fingerParticipating[f] && fsrState[f] != FSR_SENSOR_ERROR &&
          fsrNet[f] >= fsrContactThreshold[f]) {
        fingerParticipating[f] = true;
      }
    }

    // Outer Slip Integrator driven by sensor-fused SlipState (Optical + LiDAR + FSR contact).
    const bool confirmedSlip = (slipState == SLIP_CORROBORATED) ||
                               (slipState == SLIP_OPTICAL_CONFIRMED) ||
                               (slipState == SLIP_LIDAR_CONFIRMED);
    if (confirmedSlip && (now - lastSlipBoostAt >= SLIP_BOOST_INTERVAL_MS)) {
      uint16_t maxPossibleBoost = 0;
      uint16_t maxOvershoot = 0;
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        if (!fingerParticipating[f]) continue;
        if (fsrMaxThreshold[f] > gripHoldBaseTarget[f]) {
          const uint16_t span = fsrMaxThreshold[f] - gripHoldBaseTarget[f];
          if (span > maxPossibleBoost) maxPossibleBoost = span;
        }
        const uint32_t curTarget = (uint32_t)gripHoldBaseTarget[f] + (uint32_t)slipForceBoost;
        if (fsrState[f] != FSR_SENSOR_ERROR &&
            fsrNet[f] >= fsrContactThreshold[f] &&
            fsrNet[f] < fsrMaxThreshold[f] &&
            fsrNet[f] > curTarget) {
          const uint16_t over = (uint16_t)(fsrNet[f] - curTarget);
          if (over > maxOvershoot) maxOvershoot = over;
        }
      }
      uint32_t nextBoost = (uint32_t)slipForceBoost + (uint32_t)maxOvershoot + (uint32_t)slipBoostStep;
      if (nextBoost > maxPossibleBoost) nextBoost = maxPossibleBoost;
      if (nextBoost > slipForceBoost) {
        slipForceBoost = (uint16_t)nextBoost;
        boostPendingTighten = true;
        lastSlipBoostAt = now;
        lastTightenAt = now;
        if (lidarValid) gripHoldLidarMm = lidarDistanceMm;
        if (slipLidarActive) {
          lastLidarSlipAt = now;
          lidarState = LIDAR_SLIP;
        }
      }
    }

    // Inner Force P-Loop: equalizes all participating fingers up to activeTargetForceForFinger(f)
    // and recovers force if any participating finger drops past forceHoldDeadband.
    bool anyTightening = false;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (!fingerParticipating[f]) continue;
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      const uint16_t refForce = activeTargetForceForFinger(f);
      const int32_t error = (int32_t)refForce - (int32_t)fsrNet[f];
      const bool shouldTighten =
          (boostPendingTighten && error > 0) ||
          (error > (int32_t)forceHoldDeadband) ||
          (targets[servoIdx] != DEFAULT_ANGLE && error > 0);

      if (fsrState[f] == FSR_SENSOR_ERROR || fsrState[f] == FSR_HIGH_PRESSURE || error <= 0) {
        fingerTightenStartedAt[f] = 0;
        if (targets[servoIdx] != DEFAULT_ANGLE) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
          lastTightenAt = now;
          lastMotorMoveAt = now;
        }
      } else if (shouldTighten) {
        anyTightening = true;
        if (fingerTightenStartedAt[f] == 0) {
          fingerTightenStartedAt[f] = now;
        } else if (now - fingerTightenStartedAt[f] >= HOLD_TIGHTEN_TIMEOUT_MS) {
          cancelSmartController();
          for (uint8_t k = 0; k < FSR_COUNT; ++k) {
            const int sIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + k);
            if (sIdx >= 0) {
              targets[sIdx] = DEFAULT_ANGLE;
              writeTargetImmediately(sIdx);
              printPosition("ACK", sIdx);
            }
          }
          Serial.println("GRIPSTATE,TIMEOUT");
          return;
        }
        const int cmd = proportionalClosingAngleForFinger(f, error, refForce);
        if (targets[servoIdx] != cmd) {
          setTarget(servoIdx, cmd);
          printPosition("ACK", servoIdx);
        }
      }
    }
    if (!anyTightening && boostPendingTighten) {
      boostPendingTighten = false;
    }
  }
}

void reportTelemetry(unsigned long now) {
  if (!hostConnected || (now - lastTelemetryReportAt < TELEMETRY_REPORT_MS)) return;
  lastTelemetryReportAt = now;

  Serial.print("FSR,");
  Serial.print(fsrNet[0]);
  Serial.print(',');
  Serial.print(fsrNet[1]);
  Serial.print(',');
  Serial.print(fsrNet[2]);
  Serial.print(',');
  Serial.print(fsrRaw[0]);
  Serial.print(',');
  Serial.print(fsrRaw[1]);
  Serial.print(',');
  Serial.print(fsrRaw[2]);
  Serial.print(',');
  Serial.print(fsrState[0]);
  Serial.print(',');
  Serial.print(fsrState[1]);
  Serial.print(',');
  Serial.println(fsrState[2]);

  Serial.print("OPT,");
  Serial.print(optAccumX);
  Serial.print(',');
  Serial.print(optAccumY);
  Serial.print(',');
  Serial.print(optMag);
  Serial.print(',');
  Serial.print(optTotalX);
  Serial.print(',');
  Serial.print(optTotalY);
  Serial.print(',');
  Serial.println(optState);
  optAccumX = 0;
  optAccumY = 0;

  Serial.print("LIDAR,");
  Serial.print(lidarDistanceMm);
  Serial.print(',');
  Serial.print(lidarSignedVelMmS);
  Serial.print(',');
  Serial.print(lidarState);
  Serial.print(',');
  Serial.print(gripControllerState);
  Serial.print(',');
  Serial.println(slipForceBoost);

  Serial.print("SLIP,");
  Serial.print(slipConfidence);
  Serial.print(',');
  Serial.print(slipState);
  Serial.print(',');
  Serial.print(slipOpticalActive ? 1 : 0);
  Serial.print(',');
  Serial.println(slipLidarActive ? 1 : 0);
}

void updateMotion(unsigned long now) {
  if (now - lastMotionUpdateAt < MOTION_UPDATE_MS) return;
  float dt = (now - lastMotionUpdateAt) / 1000.0f;
  lastMotionUpdateAt = now;
  if (dt > 0.1f) dt = 0.1f;

  for (int index = 0; index < SERVO_COUNT; ++index) {
    if (!motionProfileEnabled[index]) continue;
    const float delta = targets[index] - currentAngles[index];
    const float distance = fabsf(delta);
    if (distance < 0.02f && fabsf(currentVelocities[index]) < 0.1f) {
      currentAngles[index] = targets[index];
      currentVelocities[index] = 0;
      if (isContinuousServo(index) && targets[index] == DEFAULT_ANGLE) {
        stopContinuousServo(index);
      }
      continue;
    }

    const float direction = delta >= 0 ? 1.0f : -1.0f;
    float desiredSpeed = direction * (float)maxSpeedDegPerSecond[index];
    if (!isContinuousServo(index)) {
      const float brakingSpeed = sqrtf(
          2.0f * accelerationDegPerSecondSquared[index] * distance);
      desiredSpeed = direction * fminf((float)maxSpeedDegPerSecond[index], brakingSpeed);
    }
    const float maxVelocityChange = accelerationDegPerSecondSquared[index] * dt;
    float velocityChange = desiredSpeed - currentVelocities[index];
    if (velocityChange > maxVelocityChange) velocityChange = maxVelocityChange;
    if (velocityChange < -maxVelocityChange) velocityChange = -maxVelocityChange;
    currentVelocities[index] += velocityChange;

    const float nextAngle = currentAngles[index] + currentVelocities[index] * dt;
    if ((targets[index] - currentAngles[index]) * (targets[index] - nextAngle) <= 0) {
      currentAngles[index] = targets[index];
      currentVelocities[index] = 0;
    } else {
      currentAngles[index] = nextAngle;
    }
    if (isContinuousServo(index) && currentAngles[index] == DEFAULT_ANGLE) {
      stopContinuousServo(index);
    } else {
      writePulseForAngle(index, currentAngles[index]);
    }
  }
}

void centerAll() {
  cancelSmartController();
  for (int i = 0; i < SERVO_COUNT; ++i) {
    targets[i] = DEFAULT_ANGLE;
    currentAngles[i] = DEFAULT_ANGLE;
    currentVelocities[i] = 0;
    if (isContinuousServo(i)) stopContinuousServo(i);
    else writePulseForAngle(i, DEFAULT_ANGLE);
  }
}

void handleCalibration(char* command) {
  char* fields[4] = {nullptr, nullptr, nullptr, nullptr};
  fields[0] = command;
  for (int i = 1; i < 4; ++i) {
    char* comma = strchr(fields[i - 1], ',');
    if (!comma) {
      Serial.println("ERR,BAD_CAL_FORMAT");
      return;
    }
    *comma = '\0';
    fields[i] = comma + 1;
  }
  long channel = 0, minUs = 0, centerUs = 0, maxUs = 0;
  if (!parseNumber(fields[0], channel) || !parseNumber(fields[1], minUs) ||
      !parseNumber(fields[2], centerUs) || !parseNumber(fields[3], maxUs)) {
    Serial.println("ERR,BAD_CAL_NUMBER");
    return;
  }
  const int index = findServoIndex(channel);
  if (index < 0 || minUs < PULSE_MIN_ALLOWED_US || minUs > PULSE_MAX_ALLOWED_US ||
      centerUs < PULSE_MIN_ALLOWED_US || centerUs > PULSE_MAX_ALLOWED_US ||
      maxUs < PULSE_MIN_ALLOWED_US || maxUs > PULSE_MAX_ALLOWED_US ||
      !(minUs < centerUs && centerUs < maxUs)) {
    Serial.println("ERR,BAD_CAL_RANGE");
    return;
  }
  servoConfig[index] = {(uint16_t)minUs, (uint16_t)centerUs, (uint16_t)maxUs};
  lastPulseUs[index] = PULSE_UNSET;
  if (!isContinuousServo(index) || targets[index] != DEFAULT_ANGLE) {
    writePulseForAngle(index, currentAngles[index]);
  }
  Serial.print("ACKCAL,");
  Serial.print(channel);
  Serial.print(',');
  Serial.print(minUs);
  Serial.print(',');
  Serial.print(centerUs);
  Serial.print(',');
  Serial.println(maxUs);
}

void handleFsrCalibration(char* command) {
  char* fields[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  fields[0] = command;
  for (int i = 1; i < 5; ++i) {
    char* comma = strchr(fields[i - 1], ',');
    if (!comma) {
      Serial.println("ERR,BAD_FSRCAL_FORMAT");
      return;
    }
    *comma = '\0';
    fields[i] = comma + 1;
  }
  long finger = 0, contact = 0, grip = 0, maxThr = 0, invert = 0;
  if (!parseNumber(fields[0], finger) || !parseNumber(fields[1], contact) ||
      !parseNumber(fields[2], grip) || !parseNumber(fields[3], maxThr) ||
      !parseNumber(fields[4], invert)) {
    Serial.println("ERR,BAD_FSRCAL_NUMBER");
    return;
  }
  if (finger < 0 || finger >= FSR_COUNT ||
      contact < FSR_MIN_THR || maxThr > FSR_MAX_THR ||
      !(contact < grip && grip < maxThr) ||
      (invert != 0 && invert != 1)) {
    Serial.println("ERR,BAD_FSRCAL_RANGE");
    return;
  }
  fsrContactThreshold[finger] = (uint16_t)contact;
  fsrGripThreshold[finger] = (uint16_t)grip;
  gripHoldBaseTarget[finger] = (uint16_t)grip;
  fsrMaxThreshold[finger] = (uint16_t)maxThr;
  fingerInvertClose[finger] = (invert == 1);
  updateFsrState((uint8_t)finger);
  Serial.print("ACKFSRCAL,");
  Serial.print(finger);
  Serial.print(',');
  Serial.print(fsrContactThreshold[finger]);
  Serial.print(',');
  Serial.print(fsrGripThreshold[finger]);
  Serial.print(',');
  Serial.print(fsrMaxThreshold[finger]);
  Serial.print(',');
  Serial.println(fingerInvertClose[finger] ? 1 : 0);
}

void handleLoopCalibration(char* command) {
  char* fields[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  fields[0] = command;
  uint8_t fieldCount = 1;
  for (uint8_t i = 1; i < 7; ++i) {
    char* comma = strchr(fields[i - 1], ',');
    if (!comma) break;
    *comma = '\0';
    fields[i] = comma + 1;
    ++fieldCount;
  }
  if (fieldCount < 6) {
    Serial.println("ERR,BAD_LOOPCAL_FORMAT");
    return;
  }
  long trig = 0, dwell = 0, vel = 0, relMs = 0, boost = 0, deadband = 0;
  long gripTimeout = autoGripTimeoutMs;
  if (!parseNumber(fields[0], trig) || !parseNumber(fields[1], dwell) ||
      !parseNumber(fields[2], vel) || !parseNumber(fields[3], relMs) ||
      !parseNumber(fields[4], boost) || !parseNumber(fields[5], deadband) ||
      (fieldCount == 7 && !parseNumber(fields[6], gripTimeout))) {
    Serial.println("ERR,BAD_LOOPCAL_NUMBER");
    return;
  }
  if (trig < 15 || trig > 500 || dwell < 100 || dwell > 2000 ||
      vel < 10 || vel > 500 || relMs < 200 || relMs > 5000 ||
      boost < 20 || boost > 500 || deadband < 15 || deadband > 300 ||
      gripTimeout < MIN_AUTO_GRIP_TIMEOUT_MS || gripTimeout > MAX_AUTO_GRIP_TIMEOUT_MS) {
    Serial.println("ERR,BAD_LOOPCAL_RANGE");
    return;
  }
  lidarTriggerMm = (uint16_t)trig;
  lidarDwellMs = (uint16_t)dwell;
  lidarSlipVelocityThr = (uint16_t)vel;
  releaseDurationMs = (uint16_t)relMs;
  slipBoostStep = (uint16_t)boost;
  forceHoldDeadband = (uint16_t)deadband;
  autoGripTimeoutMs = (uint16_t)gripTimeout;
  Serial.print("ACKLOOPCAL,");
  Serial.print(lidarTriggerMm);
  Serial.print(',');
  Serial.print(lidarDwellMs);
  Serial.print(',');
  Serial.print(lidarSlipVelocityThr);
  Serial.print(',');
  Serial.print(releaseDurationMs);
  Serial.print(',');
  Serial.print(slipBoostStep);
  Serial.print(',');
  Serial.print(forceHoldDeadband);
  Serial.print(',');
  Serial.println(autoGripTimeoutMs);
}

void handleGripCommand(const char* action) {
  if (strcmp(action, "START") == 0) {
    startClosingCycle(millis());
    return;
  }
  if (strcmp(action, "RELEASE") == 0) {
    startTimedRelease(millis());
    return;
  }
  if (strcmp(action, "CLOSE_HOLD") == 0) {
    cancelSmartController();
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      setTarget(servoIdx, closingAngleForFinger(f));
      printPosition("ACK", servoIdx);
    }
    Serial.println("GRIPSTATE,CLOSE_HOLD");
    return;
  }
  if (strcmp(action, "OPEN_HOLD") == 0) {
    cancelSmartController();
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      setTarget(servoIdx, openingAngleForFinger(f));
      printPosition("ACK", servoIdx);
    }
    Serial.println("GRIPSTATE,OPEN_HOLD");
    return;
  }
  if (strcmp(action, "STOP") == 0) {
    cancelSmartController();
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      setTarget(servoIdx, DEFAULT_ANGLE);
      printPosition("ACK", servoIdx);
    }
    Serial.println("GRIPSTATE,STOPPED");
    return;
  }
  Serial.println("ERR,BAD_GRIP_ACTION");
}

void handleCommand(char* command) {
  hostConnected = true;
  lastHostActivityAt = millis();

  if (strcmp(command, "HELLO") == 0) {
    if (!opticalSensorOnline) initOpticalSensor();
    if (!lidarOnline) initLidarSensor();
    Serial.print("READY,");
    Serial.println(SERVO_COUNT);
    return;
  }
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(command, "CENTER") == 0) {
    cancelSmartController();
    for (int i = 0; i < SERVO_COUNT; ++i) setTarget(i, DEFAULT_ANGLE);
    for (int i = 0; i < SERVO_COUNT; ++i) printPosition("ACK", i);
    return;
  }
  if (strcmp(command, "FSRTARE") == 0) {
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      fsrZeroOffset[f] = (uint16_t)lroundf(fsrFiltered[f]);
      recomputeFsrNet(f);
      updateFsrState(f);
    }
    Serial.print("ACKFSRTARE,");
    Serial.print(fsrZeroOffset[0]);
    Serial.print(',');
    Serial.print(fsrZeroOffset[1]);
    Serial.print(',');
    Serial.println(fsrZeroOffset[2]);
    return;
  }
  if (strncmp(command, "FSRGUARD,", 9) == 0) {
    long enabled = 0;
    if (!parseNumber(command + 9, enabled) || (enabled != 0 && enabled != 1)) {
      Serial.println("ERR,BAD_FSRGUARD");
      return;
    }
    fsrGuardEnabled = (enabled == 1);
    Serial.print("ACKFSRGUARD,");
    Serial.println(fsrGuardEnabled ? 1 : 0);
    return;
  }
  if (strncmp(command, "FSRCAL,", 7) == 0) {
    handleFsrCalibration(command + 7);
    return;
  }
  if (strncmp(command, "LOOPCAL,", 8) == 0) {
    handleLoopCalibration(command + 8);
    return;
  }
  if (strncmp(command, "AUTOMODE,", 9) == 0) {
    char* first = command + 9;
    char* comma = strchr(first, ',');
    if (!comma) {
      Serial.println("ERR,BAD_AUTOMODE_FORMAT");
      return;
    }
    *comma = '\0';
    long autoLidar = 0, closedLoop = 0;
    if (!parseNumber(first, autoLidar) || !parseNumber(comma + 1, closedLoop) ||
        (autoLidar != 0 && autoLidar != 1) || (closedLoop != 0 && closedLoop != 1)) {
      Serial.println("ERR,BAD_AUTOMODE");
      return;
    }
    autoLidarGripEnabled = (autoLidar == 1);
    closedLoopHoldEnabled = (closedLoop == 1);
    if (!autoLidarGripEnabled && gripControllerState == GRIP_DWELL) {
      gripControllerState = GRIP_IDLE;
      Serial.println("GRIPSTATE,IDLE");
    }
    if (!closedLoopHoldEnabled && gripControllerState == GRIP_HOLDING) {
      cancelSmartController();
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      }
      Serial.println("GRIPSTATE,DONE");
    }
    Serial.print("ACKAUTOMODE,");
    Serial.print(autoLidarGripEnabled ? 1 : 0);
    Serial.print(',');
    Serial.println(closedLoopHoldEnabled ? 1 : 0);
    return;
  }
  if (strncmp(command, "GRIP,", 5) == 0) {
    handleGripCommand(command + 5);
    return;
  }
  if (strncmp(command, "OPTCAL,", 7) == 0) {
    char* motionText = command + 7;
    char* comma = strchr(motionText, ',');
    if (!comma) {
      Serial.println("ERR,BAD_OPTCAL_FORMAT");
      return;
    }
    *comma = '\0';
    long motionThr = 0, squalThr = 0;
    if (!parseNumber(motionText, motionThr) || !parseNumber(comma + 1, squalThr) ||
        motionThr < OPTICAL_MIN_MOTION_THR || motionThr > OPTICAL_MAX_MOTION_THR ||
        squalThr < 0 || squalThr > OPTICAL_MAX_SQUAL_THR) {
      Serial.println("ERR,BAD_OPTCAL_RANGE");
      return;
    }
    optMotionThreshold = (uint16_t)motionThr;
    optSqualThreshold = (uint8_t)squalThr;
    evaluateOpticalState(millis(), 0.0f, 0.0f);
    Serial.print("ACKOPTCAL,");
    Serial.print(optMotionThreshold);
    Serial.print(',');
    Serial.println(optSqualThreshold);
    return;
  }
  if (strncmp(command, "OPTLED,", 7) == 0) {
    long enabled = 0;
    if (!parseNumber(command + 7, enabled) || (enabled != 0 && enabled != 1)) {
      Serial.println("ERR,BAD_OPTLED");
      return;
    }
    opticalLedEnabled = (enabled == 1);
    if (!opticalSensorOnline) {
      initOpticalSensor();
    } else {
      setOpticalLed(opticalLedEnabled);
    }
    Serial.print("ACKOPTLED,");
    Serial.println(opticalLedEnabled ? 1 : 0);
    return;
  }
  if (strcmp(command, "OPTZERO") == 0) {
    optTotalX = 0;
    optTotalY = 0;
    optAccumX = 0;
    optAccumY = 0;
    resetOpticalMotionWindow();
    Serial.println("ACKOPTZERO");
    return;
  }
  if (strncmp(command, "MOTION,", 7) == 0) {
    char* channelText = command + 7;
    char* firstComma = strchr(channelText, ',');
    if (!firstComma) {
      Serial.println("ERR,BAD_MOTION_FORMAT");
      return;
    }
    *firstComma = '\0';
    char* enabledText = firstComma + 1;
    char* secondComma = strchr(enabledText, ',');
    if (!secondComma) {
      Serial.println("ERR,BAD_MOTION_FORMAT");
      return;
    }
    *secondComma = '\0';
    char* speedText = secondComma + 1;
    char* thirdComma = strchr(speedText, ',');
    if (!thirdComma) {
      Serial.println("ERR,BAD_MOTION_FORMAT");
      return;
    }
    *thirdComma = '\0';
    long channel = 0, enabled = 0, speed = 0, acceleration = 0;
    const int index = parseNumber(channelText, channel) ? findServoIndex(channel) : -1;
    if (index < 0 || !parseNumber(enabledText, enabled) ||
        (enabled != 0 && enabled != 1) || !parseNumber(speedText, speed) ||
        !parseNumber(thirdComma + 1, acceleration) ||
        speed < 5 || speed > 360 || acceleration < 10 || acceleration > 1440) {
      Serial.println("ERR,BAD_MOTION_PROFILE");
      return;
    }
    motionProfileEnabled[index] = enabled == 1;
    maxSpeedDegPerSecond[index] = speed;
    accelerationDegPerSecondSquared[index] = acceleration;
    if (!motionProfileEnabled[index]) writeTargetImmediately(index);
    Serial.print("ACKMOTION,");
    Serial.print(channel);
    Serial.print(',');
    Serial.print(motionProfileEnabled[index] ? 1 : 0);
    Serial.print(',');
    Serial.print(maxSpeedDegPerSecond[index]);
    Serial.print(',');
    Serial.println(accelerationDegPerSecondSquared[index]);
    return;
  }
  if (strncmp(command, "CAL,", 4) == 0) {
    handleCalibration(command + 4);
    return;
  }
  if (strncmp(command, "SET,", 4) != 0) {
    Serial.println("ERR,BAD_COMMAND");
    return;
  }
  char* channelText = command + 4;
  char* comma = strchr(channelText, ',');
  if (!comma) {
    Serial.println("ERR,BAD_FORMAT");
    return;
  }
  *comma = '\0';
  long channel = 0, angle = 0;
  if (!parseNumber(channelText, channel) || !parseNumber(comma + 1, angle)) {
    Serial.println("ERR,BAD_NUMBER");
    return;
  }
  const int index = findServoIndex(channel);
  if (index < 0 || angle < 0 || angle > 180) {
    Serial.println("ERR,OUT_OF_RANGE");
    return;
  }
  if (fingerIndexForServo(index) >= 0) {
    cancelSmartController();
  }
  setTarget(index, (int)angle);
  printPosition("ACK", index);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  recoverI2CBus();
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  if (!pca.begin()) {
    Serial.println("ERR,PCA9685_NOT_FOUND");
    while (true) delay(1000);
  }
  pca.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
  pca.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
  delay(15);
  centerAll();
  sampleFsrSensors(millis(), true);
  initOpticalSensor();
  initLidarSensor();
  Serial.print("READY,");
  Serial.println(SERVO_COUNT);
}

void loop() {
  const unsigned long now = millis();
  if (bufferIndex > 0 && (now - lastSerialByteAt > SERIAL_LINE_TIMEOUT_MS)) {
    bufferIndex = 0;
  }
  while (Serial.available()) {
    lastSerialByteAt = millis();
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

  if (hostConnected && (millis() - lastHostActivityAt > HOST_INACTIVITY_TIMEOUT_MS)) {
    hostConnected = false;
    cancelSmartController();
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) {
        targets[servoIdx] = DEFAULT_ANGLE;
        writeTargetImmediately(servoIdx);
      }
    }
  }

  const unsigned long tick = millis();
  sampleFsrSensors(tick, false);
  sampleOpticalSensor(tick);
  sampleLidarSensor(tick);
  updateSlipDetection(tick);
  updateSmartGripController(tick);
  updateMotion(tick);
  reportTelemetry(tick);
}
