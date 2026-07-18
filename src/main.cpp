// Tobble - hardware "sense" stage.
// Reads accelerometer + gyroscope from the IMU over I2C on real hardware.
//
// NOTE: this module reports WHO_AM_I = 0x70, so it is an MPU6500, not a genuine
// MPU6050. The Adafruit MPU6050 library checks WHO_AM_I and rejects anything
// that isn't 0x68, so we talk to the registers directly instead - the two chips
// are register-compatible for accel/gyro.
//
// UNITS: gyro here is already deg/s. The Wokwi version used the Adafruit
// library (rad/s) and needed `* 180/PI` - not carrying that over.

#include <Arduino.h>
#include <Wire.h>

const uint8_t MPU = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);

  writeReg(PWR_MGMT_1, 0x00);   // wake it up - the chip boots asleep
  delay(100);

  Serial.println("MPU6500 ready.");
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 14);

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  int16_t ax = (buf[0]  << 8) | buf[1];
  int16_t ay = (buf[2]  << 8) | buf[3];
  int16_t az = (buf[4]  << 8) | buf[5];
  int16_t gx = (buf[8]  << 8) | buf[9];

  // accel: +-2g -> 16384 LSB/g ; gyro: +-250dps -> 131 LSB/(deg/s)
  float accelX = ax / 16384.0 * 9.81;
  float accelY = ay / 16384.0 * 9.81;
  float accelZ = az / 16384.0 * 9.81;
  float gyroX  = gx / 131.0;                 // already deg/s

  Serial.print("accel x:"); Serial.print(accelX, 2);
  Serial.print("  y:");     Serial.print(accelY, 2);
  Serial.print("  z:");     Serial.print(accelZ, 2);
  Serial.print("   gyro x:"); Serial.println(gyroX, 2);

  delay(200);
}
