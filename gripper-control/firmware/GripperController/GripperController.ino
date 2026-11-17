// Dishwasher Robot HT26 - manual PCA9685 servo controller.
// Board: Arduino UNO R4 WiFi; PCA9685 default I2C address: 0x40.
// USB serial protocol at 115200 baud:
//   HELLO / PING
//   SET,<channel>,<angle>     channel 0..8, angle 0..180
//   CAL,<channel>,<min_us>,<center_us>,<max_us>
//   MOTION,<channel>,<enabled>,<max_speed_deg_s>,<acceleration_deg_s2>
//   CENTER

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint32_t PCA9685_OSCILLATOR_HZ = 25000000UL;
constexpr uint8_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr uint8_t SERVO_COUNT = 9;
constexpr uint8_t SERVO_CHANNELS[SERVO_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t CONTINUOUS_SERVO_FIRST_CHANNEL = 6;
constexpr int DEFAULT_ANGLE = 90;
constexpr unsigned long MOTION_UPDATE_MS = 20;
constexpr int PULSE_MIN_ALLOWED_US = 600;
constexpr int PULSE_MAX_ALLOWED_US = 2400;

Adafruit_PWMServoDriver pca(PCA9685_ADDRESS);

struct ServoConfig {
  uint16_t minUs;
  uint16_t centerUs;
  uint16_t maxUs;
};

ServoConfig servoConfig[SERVO_COUNT] = {
  {1000, 1500, 2000}, {1000, 1500, 2000}, {1000, 1500, 2000},
  {1000, 1500, 2000}, {1000, 1500, 2000}, {1000, 1500, 2000},
  {900, 1500, 2100}, {900, 1500, 2100}, {900, 1500, 2100},
};
int targets[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
float currentAngles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
float currentVelocities[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool motionProfileEnabled[SERVO_COUNT] = {true, true, true, true, true, true, true, true, true};
int maxSpeedDegPerSecond[SERVO_COUNT] = {60, 60, 60, 60, 60, 60, 60, 60, 60};
int accelerationDegPerSecondSquared[SERVO_COUNT] = {120, 120, 120, 120, 120, 120, 120, 120, 120};
unsigned long lastMotionUpdateAt = 0;
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

void stopContinuousServo(int index) {
  // Full-off stops the continuous rotation servos without a center pulse.
  pca.setPWM(SERVO_CHANNELS[index], 0, 4096);
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
  pca.writeMicroseconds(SERVO_CHANNELS[index], (uint16_t)lroundf(pulseUs));
}

void writeTargetImmediately(int index) {
  currentAngles[index] = targets[index];
  currentVelocities[index] = 0;
  if (index >= CONTINUOUS_SERVO_FIRST_CHANNEL && targets[index] == DEFAULT_ANGLE) {
    stopContinuousServo(index);
  } else {
    writePulseForAngle(index, targets[index]);
  }
}

void setTarget(int index, int angle) {
  targets[index] = angle;
  // Continuous servos always stop immediately when the hold button is released.
  if (index >= CONTINUOUS_SERVO_FIRST_CHANNEL && angle == DEFAULT_ANGLE) {
    writeTargetImmediately(index);
  } else if (!motionProfileEnabled[index]) {
    writeTargetImmediately(index);
  }
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
      continue;
    }

    const float direction = delta >= 0 ? 1.0f : -1.0f;
    const float brakingSpeed = sqrtf(
        2.0f * accelerationDegPerSecondSquared[index] * distance);
    const float desiredSpeed = direction * fminf(maxSpeedDegPerSecond[index], brakingSpeed);
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
    writePulseForAngle(index, currentAngles[index]);
  }
}

void printPosition(const char* prefix, int index) {
  Serial.print(prefix);
  Serial.print(',');
  Serial.print(SERVO_CHANNELS[index]);
  Serial.print(',');
  Serial.println(targets[index]);
}

void centerAll() {
  for (int i = 0; i < SERVO_COUNT; ++i) {
    targets[i] = DEFAULT_ANGLE;
    currentAngles[i] = DEFAULT_ANGLE;
    currentVelocities[i] = 0;
    if (i >= CONTINUOUS_SERVO_FIRST_CHANNEL) stopContinuousServo(i);
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
  Serial.print("ACKCAL,");
  Serial.print(channel);
  Serial.print(',');
  Serial.print(minUs);
  Serial.print(',');
  Serial.print(centerUs);
  Serial.print(',');
  Serial.println(maxUs);
}

void handleCommand(char* command) {
  if (strcmp(command, "HELLO") == 0) {
    Serial.print("READY,");
    Serial.println(SERVO_COUNT);
    return;
  }
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(command, "CENTER") == 0) {
    for (int i = 0; i < SERVO_COUNT; ++i) setTarget(i, DEFAULT_ANGLE);
    for (int i = 0; i < SERVO_COUNT; ++i) printPosition("ACK", i);
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
  setTarget(index, (int)angle);
  printPosition("ACK", index);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!pca.begin()) {
    Serial.println("ERR,PCA9685_NOT_FOUND");
    while (true) delay(1000);
  }
  pca.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
  pca.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
  delay(15);
  centerAll();
  Serial.print("READY,");
  Serial.println(SERVO_COUNT);
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
  updateMotion(millis());
}
