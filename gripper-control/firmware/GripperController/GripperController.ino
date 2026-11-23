// Dishwasher Robot HT26 - manual PCA9685 servo + FSR pressure + PAA5100JE optical controller.
// Board: Arduino UNO R4 WiFi; PCA9685 default I2C address: 0x40 (Wire).
// VL53L4CD lidar on the Qwiic connector (Wire1), address 0x29, Pololu VL53L4CD library.
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
//   GRIP,<START|CLOSE_HOLD|OPEN_HOLD|STOP>
//   LIDARCFG,<enabled>,<threshold_mm>,<rearm_mm>
//   OPTCAL,<motion_thr>,<squal_thr>
//   SLIPCFG,<lidar_speed_threshold_mm_s>
//   ADAPTIVE,<0|1>
//   OPTLED,<0|1>
//   OPTZERO
//   CENTER

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Optical_Flow_Sensor.h>
#include <VL53L4CD.h>
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
constexpr unsigned long FSR_REPORT_MS = 100;
constexpr unsigned long AUTO_GRIP_TIMEOUT_MS = 6000;

// Lidar
constexpr unsigned long LIDAR_POLL_INTERVAL_MS = 5;
constexpr unsigned long LIDAR_REPORT_MS = 100;
constexpr unsigned long SLIP_REPORT_MS = 100;
constexpr unsigned long SLIP_CONFIRM_MS = 150;
constexpr unsigned long ADAPTIVE_SLIP_COOLDOWN_MS = 1200;
constexpr uint16_t ADAPTIVE_GRIP_STEP = 40;
constexpr int32_t SLIP_LIDAR_SPEED_DEFAULT_THR_MM_S = 20;
constexpr int32_t SLIP_LIDAR_SPEED_MIN_THR_MM_S = 5;
constexpr int32_t SLIP_LIDAR_SPEED_MAX_THR_MM_S = 200;
constexpr float LIDAR_SPEED_EMA_ALPHA = 0.35f;   // lower = smoother but laggier
constexpr uint8_t LIDAR_SPEED_WINDOW = 8;        // samples in the regression window
constexpr float LIDAR_SPEED_DEADBAND = 3.0f;    // mm/s, below this reports 0
constexpr uint8_t LIDAR_ENTRY_SAMPLES = 3;
constexpr uint16_t LIDAR_DEFAULT_THRESHOLD_MM = 150;
constexpr uint16_t LIDAR_DEFAULT_REARM_MM = 50;
constexpr uint16_t LIDAR_MIN_THRESHOLD_MM = 30;
constexpr uint16_t LIDAR_MAX_THRESHOLD_MM = 1300;

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

enum SlipState : uint8_t {
  SLIP_UNAVAILABLE = 0,
  SLIP_STABLE = 1,
  SLIP_SUSPECTED = 2,
  SLIP_CORROBORATED = 3,
  SLIP_OPTICAL_CONFIRMED = 4,
  SLIP_LIDAR_CONFIRMED = 5,
};

const SPISettings paaSpiSettings(2000000, MSBFIRST, SPI_MODE3);
Optical_Flow_Sensor flow(PAA5100_CS_PIN, PAA5100);
Adafruit_PWMServoDriver pca(PCA9685_ADDRESS);
VL53L4CD lidarSensor;

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
uint16_t fsrActiveGripThreshold[FSR_COUNT] = {600, 600, 600};
uint16_t fsrMaxThreshold[FSR_COUNT] = {1800, 1800, 1800};
bool fingerInvertClose[FSR_COUNT] = {false, false, false};
bool fsrGuardEnabled = true;
bool autoGripActive = false;
bool adaptiveGripEnabled = false;
bool adaptiveGripActive = false;
bool adaptiveGripReady = false;
bool adaptiveSlipLatched = false;
unsigned long lastAdaptiveStepAt = 0;
bool autoGripFingerActive[FSR_COUNT] = {false, false, false};
unsigned long autoGripStartedAt = 0;

unsigned long lastMotionUpdateAt = 0;
unsigned long lastFsrSampleAt = 0;
unsigned long lastFsrReportAt = 0;
unsigned long lastSerialByteAt = 0;
bool hostConnected = false;
unsigned long lastHostActivityAt = 0;
char commandBuffer[96];
uint8_t bufferIndex = 0;

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
unsigned long lastOpticalSampleAt = 0;
unsigned long lastOpticalMotionAt = 0;

bool lidarSensorOnline = false;
bool lidarAutoEnabled = false;
bool lidarArmed = false;
bool lidarHasMeasurement = false;
uint16_t lidarThresholdMm = LIDAR_DEFAULT_THRESHOLD_MM;
uint16_t lidarRearmMm = LIDAR_DEFAULT_REARM_MM;
int32_t lidarDistanceMm = -1;
int32_t lidarSpeedMmPerSecond = 0;
int32_t slipLidarSpeedThresholdMmPerSecond = SLIP_LIDAR_SPEED_DEFAULT_THR_MM_S;
float lidarFilteredSpeed = 0.0f;
int32_t lidarPreviousDistanceMm = -1;
unsigned long lidarPreviousSampleAt = 0;
unsigned long lastLidarSampleAt = 0;
unsigned long lastLidarReportAt = 0;
uint8_t lidarEntryCount = 0;
uint8_t lidarClearCount = 0;
uint8_t lidarState = 0;  // 0 offline, 1 valid measurement, 2 invalid/no target.
float lidarHistDist[LIDAR_SPEED_WINDOW];
unsigned long lidarHistTime[LIDAR_SPEED_WINDOW];
uint8_t lidarHistHead = 0;
uint8_t lidarHistCount = 0;

// Slip evidence fusion plus optional, FSR-limited adaptive tightening.
uint8_t slipConfidence = 0;
uint8_t slipState = SLIP_UNAVAILABLE;
bool slipOpticalActive = false;
bool slipLidarActive = false;
unsigned long slipCandidateSince = 0;
unsigned long lastSlipReportAt = 0;

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

int slowClosingAngleForFinger(int f) {
  return fingerInvertClose[f] ? 45 : 135;
}

int openingAngleForFinger(int f) {
  return fingerInvertClose[f] ? 180 : 0;
}

bool isFingerClosing(int f, int angle) {
  if (angle == DEFAULT_ANGLE) return false;
  return fingerInvertClose[f] ? (angle < DEFAULT_ANGLE) : (angle > DEFAULT_ANGLE);
}

bool isFingerAtGripLimit(int f) {
  return fsrNet[f] >= fsrActiveGripThreshold[f] || fsrState[f] >= FSR_TARGET_REACHED;
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
  // Continuous servos always stop immediately when the hold button is released.
  if (isContinuousServo(index) && angle == DEFAULT_ANGLE) {
    writeTargetImmediately(index);
  } else if (!motionProfileEnabled[index]) {
    writeTargetImmediately(index);
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
  const int32_t gripLow = (int32_t)fsrActiveGripThreshold[f] - FSR_HYSTERESIS;
  const int32_t maxLow = (int32_t)fsrMaxThreshold[f] - FSR_HYSTERESIS;

  if (net >= fsrMaxThreshold[f] || (prev == FSR_HIGH_PRESSURE && net >= maxLow)) {
    fsrState[f] = FSR_HIGH_PRESSURE;
  } else if (net >= fsrActiveGripThreshold[f] ||
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

  if (autoGripActive && (now - autoGripStartedAt >= AUTO_GRIP_TIMEOUT_MS)) {
    autoGripActive = false;
    adaptiveGripActive = false;
    adaptiveGripReady = false;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (autoGripFingerActive[f]) {
        autoGripFingerActive[f] = false;
        const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
        if (servoIdx >= 0) {
          targets[servoIdx] = DEFAULT_ANGLE;
          writeTargetImmediately(servoIdx);
          printPosition("ACK", servoIdx);
        }
      }
    }
    Serial.println("GRIPSTATE,TIMEOUT");
    return;
  }

  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx < 0) continue;

    if (fsrGuardEnabled && isFingerAtGripLimit(f) &&
        (isFingerClosing(f, targets[servoIdx]) || autoGripFingerActive[f])) {
      autoGripFingerActive[f] = false;
      targets[servoIdx] = DEFAULT_ANGLE;
      writeTargetImmediately(servoIdx);
      printPosition("ACK", servoIdx);
      Serial.print("GRIPSTOP,");
      Serial.print(f);
      Serial.print(',');
      Serial.println(fsrNet[f]);
    } else if (autoGripFingerActive[f] && fsrState[f] == FSR_CONTACT) {
      const int slowAngle = slowClosingAngleForFinger(f);
      if (targets[servoIdx] != slowAngle) {
        setTarget(servoIdx, slowAngle);
        printPosition("ACK", servoIdx);
      }
    }
  }

  if (autoGripActive) {
    bool anyActive = false;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      if (autoGripFingerActive[f]) {
        anyActive = true;
        break;
      }
    }
    if (!anyActive) {
      autoGripActive = false;
      if (adaptiveGripActive) {
        adaptiveGripReady = true;
        adaptiveSlipLatched = true;  // Require a stable interval after the initial close before reacting.
        Serial.println("GRIPSTATE,ADAPTIVE_READY");
      } else {
        Serial.println("GRIPSTATE,DONE");
      }
    }
  }
}

void reportTelemetry(unsigned long now) {
  if (!hostConnected || (now - lastFsrReportAt < FSR_REPORT_MS)) return;
  lastFsrReportAt = now;
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
  if (!opticalSensorOnline) {
    optState = OPT_OFFLINE;
    optAccumX = 0;
    optAccumY = 0;
    optFilteredMag = 0.0f;
    optMag = 0;
    optSqual = 0;
    return false;
  }
  if (!opticalLedEnabled) {
    setOpticalLed(false);
  }
  optState = OPT_STILL;
  return true;
}

void evaluateOpticalState(unsigned long now, float rawMag) {
  if (!opticalSensorOnline) {
    optState = OPT_OFFLINE;
    return;
  }
  if (optSqualThreshold > 0 && optSqual < optSqualThreshold) {
    optState = OPT_LOW_SQUAL;
    return;
  }
  const bool aboveThreshold =
      (rawMag >= (float)optMotionThreshold) || (optMag >= optMotionThreshold);
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

  const float rawMag = sqrtf((float)dx * (float)dx + (float)dy * (float)dy);
  optFilteredMag = (OPTICAL_EMA_ALPHA * rawMag) +
                   ((1.0f - OPTICAL_EMA_ALPHA) * optFilteredMag);
  optMag = (uint16_t)lroundf(fmaxf(rawMag, optFilteredMag));
  evaluateOpticalState(now, rawMag);
}

void handleGripCommand(const char* action);

bool initLidarSensor() {
  lidarSensor.setBus(&Wire1);
  lidarSensor.setTimeout(100);
  if (!lidarSensor.init()) {
    lidarSensorOnline = false;
    lidarState = 0;
    return false;
  }
  lidarSensor.setRangeTiming(20, 25);  // 20 ms budget, 25 ms between measurements
  lidarSensor.startContinuous();
  lidarSensorOnline = true;
  lidarState = 2;
  lidarHasMeasurement = false;
  lidarPreviousDistanceMm = -1;
  lidarPreviousSampleAt = 0;
  lidarFilteredSpeed = 0.0f;
  lidarSpeedMmPerSecond = 0;
  lidarEntryCount = 0;
  lidarHistCount = 0;
  lidarHistHead = 0;
  // Require the object to leave the re-arm zone before the first automatic trigger.
  lidarArmed = false;
  return true;
}

// Least-squares slope (mm/s) of the recent distance history.
float lidarSlopeMmPerSecond() {
  if (lidarHistCount < 3) return 0.0f;
  const uint8_t newest =
      (lidarHistHead + LIDAR_SPEED_WINDOW - 1) % LIDAR_SPEED_WINDOW;
  const unsigned long newestT = lidarHistTime[newest];

  float sumT = 0, sumD = 0;
  for (uint8_t i = 0; i < lidarHistCount; ++i) {
    const uint8_t idx = (newest + LIDAR_SPEED_WINDOW - i) % LIDAR_SPEED_WINDOW;
    sumT += -((float)(newestT - lidarHistTime[idx])) / 1000.0f;
    sumD += lidarHistDist[idx];
  }
  const float meanT = sumT / lidarHistCount;
  const float meanD = sumD / lidarHistCount;

  float num = 0, den = 0;
  for (uint8_t i = 0; i < lidarHistCount; ++i) {
    const uint8_t idx = (newest + LIDAR_SPEED_WINDOW - i) % LIDAR_SPEED_WINDOW;
    const float t = -((float)(newestT - lidarHistTime[idx])) / 1000.0f;
    num += (t - meanT) * (lidarHistDist[idx] - meanD);
    den += (t - meanT) * (t - meanT);
  }
  return (den > 1e-6f) ? (num / den) : 0.0f;
}

void sampleLidarSensor(unsigned long now) {
  if (!lidarSensorOnline) return;
  if (now - lastLidarSampleAt < LIDAR_POLL_INTERVAL_MS) return;
  lastLidarSampleAt = now;
  if (!lidarSensor.dataReady()) {
    if (lidarPreviousSampleAt > 0 && now - lidarPreviousSampleAt > 500) {
      lidarState = 2;
      lidarHasMeasurement = false;
      lidarSpeedMmPerSecond = 0;
      lidarFilteredSpeed = 0.0f;
      lidarEntryCount = 0;
      lidarHistCount = 0;
      lidarHistHead = 0;
    }
    return;
  }

  lidarSensor.read(false);  // data is already ready, so don't block
  const uint16_t measured = lidarSensor.ranging_data.range_mm;
  const uint8_t rangeStatus = (uint8_t)lidarSensor.ranging_data.range_status;
  if (rangeStatus != 0 || measured == 0 || measured > 1300) {
    lidarState = 2;
    lidarHasMeasurement = false;
    lidarPreviousDistanceMm = -1;
    lidarPreviousSampleAt = 0;
    lidarFilteredSpeed *= 0.5f;
    lidarSpeedMmPerSecond = (int32_t)lroundf(lidarFilteredSpeed);
    lidarEntryCount = 0;
    lidarHistCount = 0;
    lidarHistHead = 0;
    // On the VL53L4CD, range status 1 is a sigma failure and 2 is a signal failure.
    // Repeated no-return readings are treated as a clear field of view for re-arming.
    if (lidarAutoEnabled && (rangeStatus == 1 || rangeStatus == 2)) {
      if (lidarClearCount < LIDAR_ENTRY_SAMPLES) ++lidarClearCount;
      if (lidarClearCount >= LIDAR_ENTRY_SAMPLES && !lidarArmed) {
        lidarArmed = true;
        Serial.println("LIDARSTATE,ARMED");
      }
    } else {
      lidarClearCount = 0;
    }
    return;
  }

  lidarClearCount = 0;
  lidarDistanceMm = measured;
  lidarState = 1;
  lidarHasMeasurement = true;

  lidarHistDist[lidarHistHead] = (float)measured;
  lidarHistTime[lidarHistHead] = now;
  lidarHistHead = (lidarHistHead + 1) % LIDAR_SPEED_WINDOW;
  if (lidarHistCount < LIDAR_SPEED_WINDOW) ++lidarHistCount;

  const float slope = lidarSlopeMmPerSecond();
  lidarFilteredSpeed = (LIDAR_SPEED_EMA_ALPHA * slope) +
                       ((1.0f - LIDAR_SPEED_EMA_ALPHA) * lidarFilteredSpeed);
  lidarSpeedMmPerSecond = (fabsf(lidarFilteredSpeed) < LIDAR_SPEED_DEADBAND)
                              ? 0
                              : (int32_t)lroundf(lidarFilteredSpeed);
  lidarPreviousDistanceMm = measured;
  lidarPreviousSampleAt = now;

  if (lidarAutoEnabled && !lidarArmed && measured >= (uint32_t)lidarThresholdMm + lidarRearmMm) {
    lidarArmed = true;
    lidarEntryCount = 0;
    lidarClearCount = 0;
    Serial.println("LIDARSTATE,ARMED");
  }
}

void updateLidarAutoGrip() {
  if (!lidarAutoEnabled || !lidarSensorOnline || lidarState != 1 || !lidarHasMeasurement) return;
  if (!lidarArmed || autoGripActive) return;
  // Do not take control away from an active manual finger command.
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
    if (servoIdx >= 0 && targets[servoIdx] != DEFAULT_ANGLE) return;
  }

  if (lidarDistanceMm <= lidarThresholdMm) {
    if (lidarEntryCount < LIDAR_ENTRY_SAMPLES) ++lidarEntryCount;
  } else {
    lidarEntryCount = 0;
  }

  if (lidarEntryCount < LIDAR_ENTRY_SAMPLES) return;
  lidarArmed = false;
  lidarEntryCount = 0;
  Serial.print("LIDARTRIGGER,");
  Serial.println(lidarDistanceMm);
  // The existing FSR-controlled auto-grip routine owns all finger movement and stopping.
  handleGripCommand("START");
}

void updateSlipDetection(unsigned long now) {
  const bool opticalAvailable = opticalSensorOnline && optState != OPT_LOW_SQUAL &&
                                optState != OPT_OFFLINE;
  const bool lidarAvailable = lidarSensorOnline && lidarState == 1 && lidarHasMeasurement;

  slipOpticalActive = opticalAvailable && optState == OPT_MOVEMENT;
  slipLidarActive = lidarAvailable &&
                    labs((long)lidarSpeedMmPerSecond) >= slipLidarSpeedThresholdMmPerSecond;

  // Keep the score interpretable: optical skin motion contributes 60 points,
  // palm-to-object range motion contributes 40 points. Missing sensors do not
  // count as evidence and are identified separately in the state field.
  slipConfidence = (slipOpticalActive ? 60 : 0) + (slipLidarActive ? 40 : 0);

  if (!opticalAvailable && !lidarAvailable) {
    slipState = SLIP_UNAVAILABLE;
    slipCandidateSince = 0;
    return;
  }

  if (!slipOpticalActive && !slipLidarActive) {
    slipState = SLIP_STABLE;
    slipCandidateSince = 0;
    return;
  }

  if (slipCandidateSince == 0) slipCandidateSince = now;
  if (now - slipCandidateSince < SLIP_CONFIRM_MS) {
    slipState = SLIP_SUSPECTED;
  } else if (slipOpticalActive && slipLidarActive) {
    slipState = SLIP_CORROBORATED;
  } else if (slipOpticalActive) {
    // The tactile optical sensor is independently sufficient: the palm LiDAR
    // may not see range change when the object shears against the rubber skin.
    slipState = SLIP_OPTICAL_CONFIRMED;
  } else {
    slipState = SLIP_LIDAR_CONFIRMED;
  }
}

void reportSlipTelemetry(unsigned long now) {
  if (!hostConnected || now - lastSlipReportAt < SLIP_REPORT_MS) return;
  lastSlipReportAt = now;
  Serial.print("SLIP,");
  Serial.print(slipConfidence);
  Serial.print(',');
  Serial.print(slipState);
  Serial.print(',');
  Serial.print(slipOpticalActive ? 1 : 0);
  Serial.print(',');
  Serial.println(slipLidarActive ? 1 : 0);
}

// Adaptive tightening is deliberately incremental and only runs after the
// initial FSR-controlled grip has completed. Optical-only evidence maps to
// finger 1 (the tactile sensor location); LiDAR evidence is global.
void updateAdaptiveGrip(unsigned long now) {
  if (!adaptiveGripActive || !adaptiveGripReady || !fsrGuardEnabled || autoGripActive) return;

  const bool confirmedSlip = slipState == SLIP_CORROBORATED ||
                             slipState == SLIP_OPTICAL_CONFIRMED ||
                             slipState == SLIP_LIDAR_CONFIRMED;
  if (!confirmedSlip) {
    adaptiveSlipLatched = false;
    return;
  }
  if (adaptiveSlipLatched) return;
  adaptiveSlipLatched = true;
  if (lastAdaptiveStepAt != 0 && now - lastAdaptiveStepAt < ADAPTIVE_SLIP_COOLDOWN_MS) return;
  lastAdaptiveStepAt = now;

  bool changedAny = false;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    const bool affected = (slipState == SLIP_OPTICAL_CONFIRMED) ? (f == 0) : true;
    if (!affected || fsrState[f] == FSR_SENSOR_ERROR ||
        fsrNet[f] < fsrContactThreshold[f] ||
        fsrActiveGripThreshold[f] >= fsrMaxThreshold[f]) continue;

    uint32_t nextTarget = (uint32_t)fsrActiveGripThreshold[f] + ADAPTIVE_GRIP_STEP;
    if (nextTarget > fsrMaxThreshold[f]) nextTarget = fsrMaxThreshold[f];
    // Do not raise a target unless it leaves enough room beyond the current
    // reading to clear the FSR hysteresis and produce a real controlled move.
    if (nextTarget <= (uint32_t)fsrNet[f] + FSR_HYSTERESIS) continue;
    fsrActiveGripThreshold[f] = (uint16_t)nextTarget;
    updateFsrState(f);

    if (fsrNet[f] < fsrActiveGripThreshold[f] && fsrState[f] != FSR_SENSOR_ERROR) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx >= 0) {
        autoGripFingerActive[f] = true;
        setTarget(servoIdx, slowClosingAngleForFinger(f));
      }
    }
    Serial.print("ADAPTIVE,STEP,");
    Serial.print(f);
    Serial.print(',');
    Serial.println(fsrActiveGripThreshold[f]);
    changedAny = true;
  }

  if (!changedAny) Serial.println("ADAPTIVE,LIMITS_REACHED_OR_SENSOR_FAULT");
}

void reportLidarTelemetry(unsigned long now) {
  if (!hostConnected || now - lastLidarReportAt < LIDAR_REPORT_MS) return;
  lastLidarReportAt = now;
  Serial.print("LIDAR,");
  Serial.print(lidarState == 1 ? lidarDistanceMm : -1);
  Serial.print(',');
  Serial.print(lidarSpeedMmPerSecond);
  Serial.print(',');
  Serial.print(lidarState);
  Serial.print(',');
  Serial.print(lidarAutoEnabled ? 1 : 0);
  Serial.print(',');
  Serial.println(lidarArmed ? 1 : 0);
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

void cancelAutoGrip() {
  autoGripActive = false;
  adaptiveGripActive = false;
  adaptiveGripReady = false;
  adaptiveSlipLatched = false;
  for (uint8_t f = 0; f < FSR_COUNT; ++f) {
    autoGripFingerActive[f] = false;
    fsrActiveGripThreshold[f] = fsrGripThreshold[f];
    updateFsrState(f);
    if (hostConnected) {
      Serial.print("ADAPTIVE,RESET,");
      Serial.print(f);
      Serial.print(',');
      Serial.println(fsrActiveGripThreshold[f]);
    }
  }
}

void centerAll() {
  cancelAutoGrip();
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
  if (adaptiveGripActive || autoGripActive) handleGripCommand("STOP");
  fsrContactThreshold[finger] = (uint16_t)contact;
  fsrGripThreshold[finger] = (uint16_t)grip;
  fsrActiveGripThreshold[finger] = (uint16_t)grip;
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

void handleGripCommand(const char* action) {
  if (strcmp(action, "START") == 0) {
    if (!fsrGuardEnabled) {
      Serial.println("ERR,ADAPTIVE_REQUIRES_FSR_GUARD");
      return;
    }
    adaptiveGripActive = adaptiveGripEnabled;
    adaptiveGripReady = false;
    adaptiveSlipLatched = false;
    lastAdaptiveStepAt = 0;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) fsrActiveGripThreshold[f] = fsrGripThreshold[f];
    autoGripActive = true;
    autoGripStartedAt = millis();
    bool anyActive = false;
    for (uint8_t f = 0; f < FSR_COUNT; ++f) {
      const int servoIdx = findServoIndex(CONTINUOUS_SERVO_FIRST_CHANNEL + f);
      if (servoIdx < 0) continue;
      if (fsrGuardEnabled && isFingerAtGripLimit(f)) {
        autoGripFingerActive[f] = false;
        setTarget(servoIdx, DEFAULT_ANGLE);
      } else {
        autoGripFingerActive[f] = true;
        anyActive = true;
        const int cmdAngle = (fsrState[f] == FSR_CONTACT)
                                 ? slowClosingAngleForFinger(f)
                                 : closingAngleForFinger(f);
        setTarget(servoIdx, cmdAngle);
      }
      printPosition("ACK", servoIdx);
    }
    if (!anyActive) {
      autoGripActive = false;
      bool hasValidContact = false;
      for (uint8_t f = 0; f < FSR_COUNT; ++f) {
        if (fsrState[f] != FSR_SENSOR_ERROR && fsrNet[f] >= fsrContactThreshold[f]) {
          hasValidContact = true;
        }
      }
      if (adaptiveGripActive && hasValidContact) {
        adaptiveGripReady = true;
        adaptiveSlipLatched = true;  // Require a stable interval after the initial close before reacting.
        Serial.println("GRIPSTATE,ADAPTIVE_READY");
      } else {
        adaptiveGripActive = false;
        Serial.println("GRIPSTATE,DONE");
      }
    } else {
      Serial.println("GRIPSTATE,CLOSING");
    }
    return;
  }
  if (strcmp(action, "CLOSE_HOLD") == 0) {
    cancelAutoGrip();
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
    cancelAutoGrip();
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
    cancelAutoGrip();
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
    if (!lidarSensorOnline) initLidarSensor();
    Serial.print("READY,");
    Serial.println(SERVO_COUNT);
    return;
  }
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(command, "CENTER") == 0) {
    cancelAutoGrip();
    for (int i = 0; i < SERVO_COUNT; ++i) setTarget(i, DEFAULT_ANGLE);
    for (int i = 0; i < SERVO_COUNT; ++i) printPosition("ACK", i);
    return;
  }
  if (strcmp(command, "FSRTARE") == 0) {
    if (adaptiveGripActive || autoGripActive) handleGripCommand("STOP");
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
    if (!fsrGuardEnabled) {
      adaptiveGripEnabled = false;
      if (lidarAutoEnabled) {
        lidarAutoEnabled = false;
        lidarArmed = false;
        lidarEntryCount = 0;
        Serial.println("LIDARSTATE,DISABLED_FSR_GUARD");
      }
      // Never let an automatic or adaptive grip continue after its FSR stop guard is removed.
      if (autoGripActive || adaptiveGripActive) handleGripCommand("STOP");
    }
    Serial.print("ACKFSRGUARD,");
    Serial.println(fsrGuardEnabled ? 1 : 0);
    return;
  }
  if (strncmp(command, "FSRCAL,", 7) == 0) {
    handleFsrCalibration(command + 7);
    return;
  }
  if (strncmp(command, "GRIP,", 5) == 0) {
    handleGripCommand(command + 5);
    return;
  }
  if (strncmp(command, "LIDARCFG,", 9) == 0) {
    char* enabledText = command + 9;
    char* firstComma = strchr(enabledText, ',');
    if (!firstComma) {
      Serial.println("ERR,BAD_LIDARCFG_FORMAT");
      return;
    }
    *firstComma = '\0';
    char* thresholdText = firstComma + 1;
    char* secondComma = strchr(thresholdText, ',');
    if (!secondComma) {
      Serial.println("ERR,BAD_LIDARCFG_FORMAT");
      return;
    }
    *secondComma = '\0';
    long enabled = 0, threshold = 0, rearm = 0;
    if (!parseNumber(enabledText, enabled) || (enabled != 0 && enabled != 1) ||
        !parseNumber(thresholdText, threshold) || !parseNumber(secondComma + 1, rearm) ||
        threshold < LIDAR_MIN_THRESHOLD_MM || threshold > LIDAR_MAX_THRESHOLD_MM ||
        rearm < 10 || rearm > 500) {
      Serial.println("ERR,BAD_LIDARCFG_RANGE");
      return;
    }
    if (enabled == 1 && !lidarSensorOnline) {
      Serial.println("ERR,LIDAR_SENSOR_OFFLINE");
      return;
    }
    if (enabled == 1 && !fsrGuardEnabled) {
      Serial.println("ERR,LIDAR_REQUIRES_FSR_GUARD");
      return;
    }
    lidarThresholdMm = (uint16_t)threshold;
    lidarRearmMm = (uint16_t)rearm;
    lidarAutoEnabled = (enabled == 1);
    lidarArmed = false;
    lidarEntryCount = 0;
    lidarClearCount = 0;
    Serial.print("ACKLIDARCFG,");
    Serial.print(lidarAutoEnabled ? 1 : 0);
    Serial.print(',');
    Serial.print(lidarThresholdMm);
    Serial.print(',');
    Serial.println(lidarRearmMm);
    Serial.println(lidarAutoEnabled ? "LIDARSTATE,WAITING_FOR_CLEAR" : "LIDARSTATE,DISABLED");
    return;
  }
  if (strncmp(command, "ADAPTIVE,", 9) == 0) {
    long enabled = 0;
    if (!parseNumber(command + 9, enabled) || (enabled != 0 && enabled != 1)) {
      Serial.println("ERR,BAD_ADAPTIVE_CONFIG");
      return;
    }
    if (enabled == 1 && !fsrGuardEnabled) {
      Serial.println("ERR,ADAPTIVE_REQUIRES_FSR_GUARD");
      return;
    }
    adaptiveGripEnabled = (enabled == 1);
    if (!adaptiveGripEnabled && adaptiveGripActive) handleGripCommand("STOP");
    Serial.print("ACKADAPTIVE,");
    Serial.println(adaptiveGripEnabled ? 1 : 0);
    return;
  }
  if (strncmp(command, "SLIPCFG,", 8) == 0) {
    long speedThreshold = 0;
    if (!parseNumber(command + 8, speedThreshold) ||
        speedThreshold < SLIP_LIDAR_SPEED_MIN_THR_MM_S ||
        speedThreshold > SLIP_LIDAR_SPEED_MAX_THR_MM_S) {
      Serial.println("ERR,BAD_SLIPCFG_RANGE");
      return;
    }
    slipLidarSpeedThresholdMmPerSecond = (int32_t)speedThreshold;
    Serial.print("ACKSLIPCFG,");
    Serial.println(slipLidarSpeedThresholdMmPerSecond);
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
    evaluateOpticalState(millis(), 0.0f);
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
  const int f = fingerIndexForServo(index);
  if (f >= 0) {
    if (adaptiveGripActive || autoGripActive) handleGripCommand("STOP");
    autoGripFingerActive[f] = false;
  }
  setTarget(index, (int)angle);
  printPosition("ACK", index);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  recoverI2CBus();
  Wire.begin();
  Wire1.begin();
  Wire1.setClock(400000);
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
    cancelAutoGrip();
    lidarAutoEnabled = false;
    lidarArmed = false;
    lidarEntryCount = 0;
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
  updateAdaptiveGrip(tick);
  updateLidarAutoGrip();
  updateMotion(tick);
  reportTelemetry(tick);
  reportLidarTelemetry(tick);
  reportSlipTelemetry(tick);
}
