# Build Log

A running, honest record of what I built, what broke, and how I fixed it.
The point of this log is to document my *understanding* and problem-solving,
not just the final result.

---

## 2026-07-02 - Reading a stable tilt angle (in simulation)

**What I did:**
Started in the Wokwi simulator (before hardware arrived) to build the "sense"
stage of the balancing bot. Wired a virtual MPU6050 to the ESP32 over I2C
(SDA→GPIO21, SCL→GPIO22) and read a tilt angle from the accelerometer using
`atan2(y, z)`.

**What I observed / what went wrong:**
The angle only responded when I changed the sensor's *acceleration*, not when
I changed its *rotation*. At first this seemed wrong.

**Why (what I learned):**
An accelerometer doesn't measure rotation - it measures acceleration, and at
rest the only acceleration present is gravity. So the accelerometer angle is
really "which way is down," which is why tilting (not spinning) moved it. The
accelerometer angle is correct on average but noisy, and it can't tell gravity
apart from the robot's own motion - a problem, because a balancing bot
accelerates constantly.

**How I fixed it:**
Added the gyroscope (which measures rotation *rate*) and fused both sensors
with a **complementary filter**:
`angle = α·(angle + gyro_rate·dt) + (1−α)·accel_angle`, with α = 0.98.
The gyro handles fast changes; the accelerometer slowly corrects gyro drift.
After this, the fused angle was visibly smoother than the raw accelerometer
angle, and responded correctly to rotation.

**Takeaway:**
Understood *why* sensor fusion is necessary rather than just copying it - each
sensor covers the other's weakness. This is the foundation the whole control
loop sits on.

---

<!-- Copy this template for each new entry:

## YYYY-MM-DD - [Short title]

**What I did:**

**What went wrong:**

**How I fixed it:**

**What I learned:**

-->