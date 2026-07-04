#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float angle = 0.0; //fused angle
unsigned long lastTime = 0; // when last measured - dt
const float alpha = 0.98; //trust ratio: 98% gyro, 2% accel

void setup(){
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found - check wiring");
    while (1) delay(10);
    }
Serial.println("MPU6050 ready. ");
lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // accelerometer
  float accelAngle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  
  // gyro rotation speed around X in deg/s
  float gyroRate = g.gyro.x * 180.0 / PI;

  // dt = seconds since last loop
  unsigned long now = millis(); // Time right now!
  float dt = (now - lastTime) / 1000.0; //gap since last check
  lastTime = now;

  //fusion
  angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;

  Serial.print("Accel "); Serial.print(accelAngle, 1);
  Serial.print("  Fused: ");Serial.print(angle, 1);
  Serial.println(" deg");

  delay(10);
}