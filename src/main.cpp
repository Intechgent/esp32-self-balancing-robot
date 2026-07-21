// Tobble - hardware "act" stage: TB6612FNG single-motor bring-up test.
//
// Deliberately dumb: spin one motor forward 2s, stop, reverse 2s, loop. No IMU,
// no PID - just proves the motor driver + motor + battery power path works
// before wiring control to it.
//
// The IMU complementary-filter sketch this replaces is in the previous commit
// ("Add complementary filter on hardware").
//
// WIRING (TB6612FNG channel A): VM->battery+, VCC->3V3, GND->common,
// STBY->3V3 (must be HIGH or the driver stays off), AO1/AO2->motor.

#include <Arduino.h>

// --- TB6612FNG channel A -> one motor ---
const int PWMA = 25;   // speed (PWM)
const int AIN1 = 26;   // direction
const int AIN2 = 27;   // direction
// STBY is wired to 3V3 (always enabled). If you move STBY to a GPIO instead,
// set that pin OUTPUT + HIGH in setup(), or the driver stays off.

// --- ledc PWM config (ESP32 core 2.x) ---
const int PWM_CH   = 0;      // ledc channel
const int PWM_FREQ = 1000;   // 1 kHz
const int PWM_RES  = 8;      // 8-bit -> duty 0..255
const int SPEED    = 150;    // start gentle, not full 255

void forward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CH, SPEED);
}

void reverse() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWM_CH, SPEED);
}

void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CH, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH);

  stopMotor();
  Serial.println("Motor test starting.");
}

void loop() {
  Serial.println("forward");
  forward();
  delay(2000);

  Serial.println("stop");
  stopMotor();
  delay(1000);

  Serial.println("reverse");
  reverse();
  delay(2000);

  Serial.println("stop");
  stopMotor();
  delay(1000);
}
