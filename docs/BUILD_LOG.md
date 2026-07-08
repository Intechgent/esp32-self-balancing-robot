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

**The fix (what I understood I needed):**
Add the gyroscope (which measures rotation *rate*) and fuse both sensors with a
**complementary filter**:
`angle = α·(angle + gyro_rate·dt) + (1−α)·accel_angle`, with α = 0.98.
The gyro handles fast changes; the accelerometer slowly corrects gyro drift.
I implement and verify this in the next entry (2026-07-04).

**Takeaway:**
Understood *why* sensor fusion is necessary rather than just copying it - each
sensor covers the other's weakness. This is the foundation the whole control
loop sits on.

**Verified findings (accelerometer-only baseline):**
- Resting angle reads 0°; tilting changes it, rotation alone does not (as expected)
- X-axis acceleration has no effect - confirms only Y and Z are in the atan2 formula
- y=1g → 90°, z=1g → 0°, y=z=1g → 45°: atan2 computes the *direction* of gravity
  between the two axes, not just magnitudes

---

## 2026-07-04 - Verifying the complementary filter (step 2)

Re-ran the fused-angle code deliberately, testing each axis one at a time.

**Verified findings**
- X-axis *rotation* now moves the Fused angle but not Accel - the gyro is
  contributing, and Accel's blindness to rotation is confirmed
- Y/Z rotation affect nothing - we only read gyro.x, since a balancing bot
  tips around one axis only
- Accel jumps to ±90° when gravity is fully on y, and 180° when z is
  negative (sensor upside down)
- Fused follows motion quickly but settles toward Accel's value - the 2%
  accelerometer correction visibly anchoring the gyro

---

## 2026-07-04 - PID controller (step 3)

**What I did:**
Added a PID controller that converts the fused tilt angle into a motor
command: `output = Kp·error + Ki·errorSum + Kd·dError`, clamped to the motor
range (-255 to 255). Started with Kp=25, Ki=0, Kd=0.8.

**Why I set Ki = 0 to start:**
Deliberately disabled I so I can tune P and D first - the textbook PID tuning
order (get P and D stable, add I last to remove residual lean). Turning I on
too early makes it fight P and D before they're dialed in.

**What P, I, D each do:**
- P pushes harder the more I'm tilted, but alone it overshoots and wobbles.
- D brakes based on how fast the angle is changing, damping the wobble.
- I accumulates small persistent leans and corrects steady offset.

**Verified findings (via Wokwi sliders):**
- Positive Y acceleration → negative output, and vice versa: the controller
  pushes *against* the tilt, as intended.
- Output saturates to ±255 quickly with few in-between values - expected in
  sim, because Kp=25 is high and the sliders jump the angle in coarse steps,
  so even small errors exceed the clamp. Confirms the clamp works.
- All inputs zero → angle 0.0, output ~0: balanced state produces no command.

**What I learned:**
The clamp prevents meaningless out-of-range motor values, and slider-driven
testing proves the control *logic* even though the sim has no real physics to
"fall." Smooth in-between outputs will appear on real hardware with fine tilts
and proper tuning.

**Known issue (next):**
`errorSum` is currently unclamped, so once I enable Ki it can wind up during a
sustained lean and overshoot on recovery. I'll add anti-windup (clamp the
integral, or only integrate near upright) before turning Ki on.

---

<!-- Copy this template for each new entry:

## YYYY-MM-DD - [Short title]

**What I did:**

**What went wrong:**

**How I fixed it:**

**What I learned:**

-->