// Tobble - hardware "sense" stage: complementary filter on the real IMU.
//
// This module reports WHO_AM_I = 0x70, so it is an MPU6500, not a genuine
// MPU6050. The Adafruit MPU6050 library rejects anything that isn't 0x68, so we
// read the registers directly - the two chips are register-compatible.
//
// TWO SEPARATE unit conversions:
//   * accelAngle uses atan2(), which returns RADIANS -> multiply by 180/PI to
//     get degrees. This is always needed.
//   * gyroRate is read as raw/131, which is ALREADY deg/s -> no 180/PI. The
//     Wokwi version used the Adafruit library (rad/s) and needed it; this does
//     not. Converting twice would make the gyro term too large.

#include <Arduino.h>
#include <Wire.h>

const uint8_t MPU = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

// --- complementary filter ---
const float alpha = 0.98;        // trust ratio: 98% gyro, 2% accel
float angle = 0.0;               // fused tilt angle (degrees)
float gyroBiasX = 0.0;           // resting gyro offset (deg/s), measured at boot
unsigned long lastTime = 0;

// If the fused angle runs AWAY from the accel angle when you rotate (instead of
// tracking it), the gyro spins opposite to how atan2 grows on this mounting.
// Flip this to -1.0 to fix it
const float GYRO_SIGN = 1.0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read all 14 sensor bytes in one burst; return scaled values by reference.
// ax/ay/az in m/s^2, gx in deg/s.
void readSensor(float &ax, float &ay, float &az, float &gx) {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 14);

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t rawAx = (buf[0] << 8) | buf[1];
  int16_t rawAy = (buf[2] << 8) | buf[3];
  int16_t rawAz = (buf[4] << 8) | buf[5];
  int16_t rawGx = (buf[8] << 8) | buf[9];   // buf[6..7] are temperature

  ax = rawAx / 16384.0 * 9.81;   
  ay = rawAy / 16384.0 * 9.81;
  az = rawAz / 16384.0 * 9.81;
  gx = rawGx / 131.0;            
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);

  writeReg(PWR_MGMT_1, 0x00);    // wake the chip (it boots asleep)
  delay(100);

  // --- measure gyro bias: average many samples while the board is held still ---
  Serial.println("Measuring gyro bias - keep the board STILL (~2s)...");
  const int N = 500;
  float sum = 0.0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx;
    readSensor(ax, ay, az, gx);
    sum += gx;
    delay(3);
  }
  gyroBiasX = sum / N;
  Serial.print("Gyro bias X = ");
  Serial.print(gyroBiasX, 3);
  Serial.println(" deg/s (will be subtracted)");

  // Seed the fused angle with the accel angle so it starts at the true tilt,
  // not at 0 - otherwise it takes a few seconds to converge on boot.
  float ax, ay, az, gx;
  readSensor(ax, ay, az, gx);
  angle = atan2(ay, az) * 180.0 / PI;

  lastTime = millis();
  Serial.println("Filter running.");
}

void loop() {
  float ax, ay, az, gx;
  readSensor(ax, ay, az, gx);

  // accelerometer tilt angle: atan2 returns RADIANS -> convert to degrees
  float accelAngle = atan2(ay, az) * 180.0 / PI;

  // gyro rate about X: bias-corrected, ALREADY deg/s (no 180/PI)
  float gyroRate = GYRO_SIGN * (gx - gyroBiasX);

  // dt in seconds since last loop
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // complementary filter: gyro for fast motion, accel slowly corrects drift
  angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;

  Serial.print("accelAngle: "); Serial.print(accelAngle, 1);
  Serial.print("   fused: ");   Serial.print(angle, 1);
  Serial.println(" deg");

  delay(10);
}
