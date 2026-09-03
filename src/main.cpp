// Tobble - integrated control loop: sense -> think -> act, both motors.
//
//   sense  - fused tilt angle from the MPU6500 (complementary filter)
//   think  - PID turns tilt error into a signed motor command
//   act    - TB6612FNG drives BOTH motors with that command
//
// ORIENTATION: the IMU is mounted vertically on the standing bot. Measured raw
// accel showed forward/back tilt lives on the X-Z plane (ax swings, az ~ -g) and
// the Y gyro axis - NOT the Y-Z / X the flat code assumed. So the angle is
// atan2(ax, -az) and the gyro rate is gy. A small offset (BALANCE_OFFSET) is
// subtracted so upright reads ~0 instead of ~180 (which sat on the atan2
// wraparound and made the angle jump). Fine-tune BALANCE_OFFSET so it truly
// balances. Balancing itself is NOT yet tuned.
//
// IMU: MPU6500 (WHO_AM_I = 0x70), read directly over I2C. Gyro already deg/s
// (raw/131) - no rad/s conversion. atan2 returns radians, so * 180/PI.

#include <Arduino.h>
#include <Wire.h>

// ---------------- IMU (I2C) ----------------
const uint8_t MPU          = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

// ---------------- Motor driver (TB6612) ----------------
// Channel A (one wheel)
const int PWMA = 25;   // speed (PWM)
const int AIN1 = 26;   // direction
const int AIN2 = 27;   // direction
// Channel B (other wheel)
const int PWMB = 14;   // speed (PWM)
const int BIN1 = 32;   // direction
const int BIN2 = 13;   // direction

// The two motors sit mirrored on the chassis, so the same command can spin them
// opposite ways. If the bot spins in place instead of driving straight, flip
// this to true (or swap one motor's output wires).
const bool MOTOR_B_INVERT = true;   // motors are mirrored on the chassis

// ledc PWM - one channel per motor
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;
const int PWM_FREQ = 1000;   // Hz
const int PWM_RES  = 8;      // bits -> duty 0..255
const int MAX_CMD  = 180;    // command clamp - kept below 255 during testing so a
                             // stalled/oscillating motor can't overheat. Raise
                             // toward 255 once it balances reliably.

// ---------------- Complementary filter ----------------
const float ALPHA         = 0.98;   // 98% gyro, 2% accel
const float GYRO_SIGN     = 1.0;    // flip to -1.0 if fused angle diverges from accel
const float BALANCE_OFFSET = 7.0;   // deg subtracted so upright reads ~0 (from the
                                    // orientation diagnostic). Nudge to find the
                                    // true balance point during tuning.
float angle    = 0.0;               // fused tilt angle (deg), ~0 at upright
float gyroBias = 0.0;               // resting gyro offset (deg/s)
unsigned long lastTime = 0;

// ---------------- PID ----------------
// Gains are starting guesses - UNTUNED on hardware. Ki stays 0 until P and D
// are dialed in.
float Kp = 25.0, Ki = 0.0, Kd = 0.8;
const float TARGET_ANGLE = 0.0;     // upright (angle is offset so this is 0)
const float MAX_I        = 150.0;   // integral clamp (anti-windup, for when Ki>0)
const float FALL_LIMIT   = 45.0;    // deg past which the bot is "fallen" -> stop
float errorSum  = 0.0;
float lastError = 0.0;

// ---------------- IMU helpers ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Reads accel-x/z (m/s^2) and gyro-y (deg/s) - the axes that carry the bot's
// forward/back tilt in this mounting.
void readSensor(float &ax, float &az, float &gy) {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 14);

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t rawAx = (buf[0]  << 8) | buf[1];    // accel X
  int16_t rawAz = (buf[4]  << 8) | buf[5];    // accel Z
  int16_t rawGy = (buf[10] << 8) | buf[11];   // gyro Y

  ax = rawAx / 16384.0 * 9.81;   // +-2g  -> 16384 LSB/g
  az = rawAz / 16384.0 * 9.81;
  gy = rawGy / 131.0;            // +-250 dps -> 131 LSB/(deg/s), already deg/s
}

// tilt angle from accel: forward/back lives on the X-Z plane, upright offset
// removed so it reads ~0.
float accelTilt(float ax, float az) {
  return atan2(ax, -az) * 180.0 / PI - BALANCE_OFFSET;
}

// Average many still samples to learn the resting gyro offset.
float calibrateGyroBias(int samples) {
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    float ax, az, gy;
    readSensor(ax, az, gy);
    sum += gy;
    delay(3);
  }
  return sum / samples;
}

// ---------------- Motor ----------------
// Drive one channel: sign = direction, magnitude = speed.
void driveChannel(int in1, int in2, int ch, int command) {
  if (command >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    command = -command;
  }
  ledcWrite(ch, command);
}

// Drive both motors from one command (channel B mirrored if MOTOR_B_INVERT).
void driveMotors(int command) {
  driveChannel(AIN1, AIN2, PWM_CH_A, command);
  driveChannel(BIN1, BIN2, PWM_CH_B, MOTOR_B_INVERT ? -command : command);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);
  writeReg(PWR_MGMT_1, 0x00);   // wake the MPU6500
  delay(100);

  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);  ledcAttachPin(PWMA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);  ledcAttachPin(PWMB, PWM_CH_B);
  driveMotors(0);

  Serial.println("Calibrating gyro - keep still...");
  gyroBias = calibrateGyroBias(500);

  float ax, az, gy;                 // seed angle from the accelerometer
  readSensor(ax, az, gy);
  angle = accelTilt(ax, az);

  lastTime = millis();
  Serial.println("Control loop running.");
}

void loop() {
  float ax, az, gy;
  readSensor(ax, az, gy);

  // --- sense: fused tilt angle ---
  float accelAngle = accelTilt(ax, az);            // ~0 upright, +/- for back/forward
  float gyroRate   = GYRO_SIGN * (gy - gyroBias);  // already deg/s

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  angle = ALPHA * (angle + gyroRate * dt) + (1.0 - ALPHA) * accelAngle;

  // --- think: PID ---
  float error = TARGET_ANGLE - angle;
  errorSum += error * dt;
  errorSum = constrain(errorSum, -MAX_I, MAX_I);   // anti-windup
  float dError = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * errorSum + Kd * dError;
  output = constrain(output, -MAX_CMD, MAX_CMD);

  // --- act: drive both motors (stop if the bot has toppled) ---
  int command = (int)output;
  if (fabs(angle) > FALL_LIMIT) command = 0;
  driveMotors(command);

  // --- verify over serial (throttled so it's readable while tilting) ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.print("angle: ");   Serial.print(angle, 1);
    Serial.print("  cmd: ");   Serial.print(command);
    Serial.print("  ");
    Serial.println(command == 0 ? "STOP" : (command > 0 ? "FWD" : "REV"));
  }

  delay(10);
}
