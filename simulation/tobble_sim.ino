#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float angle = 0.0; //fused angle
unsigned long lastTime = 0; // when last measured - dt
const float alpha = 0.98; //trust ratio: 98% gyro, 2% accel

/*
P = push proportional to how far you're tilted
I = accumulate small persistent leans and correct them
D = brake based on how fast the angle is changing (kills wobble)

Output = Kp·error + Ki·(sum of error) + Kd·(change in error)  
*/

float Kp = 25.0, Ki = 0.0, Kd = 0.8;   // tuning knobs
float targetAngle = 0.0;               // upright = 0°
float errorSum = 0.0;                  // running total for I
float lastError = 0.0;                 // previous error for D

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
  if (dt <= 0) dt = 0.001;   // guard: avoid divide-by-zero on the very first loop
  lastTime = now;

  //fusion
  angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;

  //PID
  float error = targetAngle - angle;          // how far from upright, and which way
  errorSum += error * dt;                      // I: accumulate over time
  float dError = (error - lastError) / dt;     // D: how fast error is changing
  lastError = error;                           // remember for next loop

  float output = Kp * error + Ki * errorSum + Kd * dError;

  // clamp to motor range (-255 to 255 = full reverse to full forward)
  if (output > 255) output = 255;
  if (output < -255) output = -255;

  Serial.print("Angle: ");  Serial.print(angle, 1);
  Serial.print("  Output: "); Serial.println(output, 1);

  delay(10);
}
