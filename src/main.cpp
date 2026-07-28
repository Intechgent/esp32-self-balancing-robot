// Tobble - integrated control loop: sense -> think -> act.
//
// Combines the three pieces validated separately:
//   sense  - fused tilt angle from the MPU6500 (complementary filter)
//   think  - PID turns tilt error into a signed motor command
//   act    - TB6612FNG drives one motor with that command
//
// Bench check (over serial, no motor power needed): tilt by hand and watch the
// command flip direction and scale with lean. NOT yet verified on hardware.
// Physical actuation is also pending a reliable battery pack - the motor drive
// is wired and called, it just can't move until VM has power.
//
// IMU: this module is an MPU6500 (WHO_AM_I = 0x70), read directly over I2C.
// Gyro is already deg/s (raw/131) - no rad/s conversion. atan2 returns radians,
// so only the accel angle gets * 180/PI.

#include <Arduino.h>
#include <Wire.h>

// ---------------- IMU (I2C) ----------------
const uint8_t MPU          = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

// ---------------- Motor driver (TB6612, channel A) ----------------
const int PWMA = 25;   // speed (PWM)
const int AIN1 = 26;   // direction
const int AIN2 = 27;   // direction

const int PWM_CH  = 0;
const int PWM_FREQ = 1000;   // Hz
const int PWM_RES  = 8;      // bits -> duty 0..255
const int MAX_CMD  = 255;    // motor command clamp

// ---------------- Complementary filter ----------------
const float ALPHA     = 0.98;   // 98% gyro, 2% accel
const float GYRO_SIGN = 1.0;    // flip to -1.0 if fused angle diverges from accel
float angle    = 0.0;           // fused tilt angle (deg)
float gyroBias = 0.0;           // resting gyro offset (deg/s)
unsigned long lastTime = 0;

// ---------------- PID ----------------
// Gains are starting guesses from sim - UNTUNED on hardware (real tuning needs
// the robot physically balancing). Ki stays 0 until P and D are dialed in.
float Kp = 25.0, Ki = 0.0, Kd = 0.8;
const float TARGET_ANGLE = 0.0;     // upright
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

// Reads accel-y/z (m/s^2) and gyro-x (deg/s) in one burst.
void readSensor(float &ay, float &az, float &gx) {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 14);

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t rawAy = (buf[2] << 8) | buf[3];
  int16_t rawAz = (buf[4] << 8) | buf[5];
  int16_t rawGx = (buf[8] << 8) | buf[9];

  ay = rawAy / 16384.0 * 9.81;   // +-2g  -> 16384 LSB/g
  az = rawAz / 16384.0 * 9.81;
  gx = rawGx / 131.0;            // +-250 dps -> 131 LSB/(deg/s), already deg/s
}

// Average many still samples to learn the resting gyro offset.
float calibrateGyroBias(int samples) {
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    float ay, az, gx;
    readSensor(ay, az, gx);
    sum += gx;
    delay(3);
  }
  return sum / samples;
}

// ---------------- Motor ----------------
// command: -MAX_CMD..+MAX_CMD. Sign = direction, magnitude = speed.
void driveMotor(int command) {
  if (command >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    command = -command;
  }
  ledcWrite(PWM_CH, command);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);
  writeReg(PWR_MGMT_1, 0x00);   // wake the MPU6500
  delay(100);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH);
  driveMotor(0);

  Serial.println("Calibrating gyro - keep still...");
  gyroBias = calibrateGyroBias(500);

  float ay, az, gx;                 // seed angle from the accelerometer
  readSensor(ay, az, gx);
  angle = atan2(ay, az) * 180.0 / PI;

  lastTime = millis();
  Serial.println("Control loop running.");
}

void loop() {
  float ay, az, gx;
  readSensor(ay, az, gx);

  // --- sense: fused tilt angle ---
  float accelAngle = atan2(ay, az) * 180.0 / PI;   // radians -> deg
  float gyroRate   = GYRO_SIGN * (gx - gyroBias);  // already deg/s

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

  // --- act: drive the motor (stop if the bot has toppled) ---
  int command = (int)output;
  if (fabs(angle) > FALL_LIMIT) command = 0;
  driveMotor(command);

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
