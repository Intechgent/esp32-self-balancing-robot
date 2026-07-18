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

## 2026-07-18 - First readings on real hardware (and an MPU6500 surprise)

**What I did:**
Moved off the simulator onto real hardware. Set up PlatformIO for the ESP32
DevKit V1, wired the IMU module over I2C (SDA→GPIO21, SCL→GPIO22), and got live
accelerometer and gyroscope readings from the physical sensor.

**What went wrong:**
Two unrelated problems that masked each other:
1. No COM port appeared, so every upload failed with *"Please specify
   `upload_port`"*. The board was fine - Windows had no driver bound to its
   CP2102 USB-to-UART chip (Device Manager showed it with an error and no class).
2. Once uploads worked, an I2C scan clearly found a device at `0x68`, but
   `mpu.begin()` from the Adafruit MPU6050 library still reported
   *"MPU6050 not found"*.

**How I fixed it:**
1. Installed the Silicon Labs CP210x VCP driver manually - Windows Update
   didn't have it. `COM5` appeared and uploads worked immediately.
2. For the sensor, instead of rewiring again I read the **WHO_AM_I** register
   (`0x75`) directly. It returned **`0x70`**, not `0x68` - so this module is an
   **MPU6500**, not a genuine MPU6050. The Adafruit library validates WHO_AM_I
   and rejects anything that isn't `0x68`, which is why a perfectly working
   sensor was reported as missing. I dropped the library and read the registers
   directly; the MPU6500 is register-compatible for accel/gyro.

**What I learned:**
- Test one layer at a time. The I2C scanner proved the wiring was good, so
  "not found" *had* to be software. Without that check I'd have kept rewiring a
  circuit that was already correct - which I did twice before thinking to ask
  the chip what it was.
- "Responds at address `0x68`" and "is the chip the library expects" are two
  different claims. One register read settled it.
- A serial port can only be held by one program: the monitor must be closed
  before uploading, or the upload silently doesn't happen and you end up
  testing stale firmware and drawing wrong conclusions.

**Units gotcha (and why it matters):**
The Adafruit library returned gyro rates in **rad/s**, so the Wokwi code
converted with `* 180/PI`. Reading the registers directly, I divide the raw
value by 131 LSB/(deg/s), so the result is **already in deg/s**. If I carried
the old conversion over as well, I'd multiply by 180/π (≈57.3) a second time
and the gyro term would be ~57× too large - the complementary filter would
swing wildly on the slightest rotation and the fused angle would be useless.
Same physical quantity, different units, silent failure: worth pinning down
now rather than debugging as "mystery drift" later.

**Verified findings (first real readings):**
- Resting roughly flat: `z ≈ 9.78`, `y ≈ 0.20`, `x ≈ -1.40` (the steady x offset
  is just the breadboard sitting ~8° off level).
- Vector magnitude √(x²+y²+z²) ≈ **9.88 m/s² ≈ 1g** - confirms the raw-to-units
  scaling (16384 LSB/g) is correct. If the conversion were wrong this wouldn't
  land on gravity.
- **Gyro reads ~+0.9 °/s while completely still, not 0** - this is gyro bias, a
  normal MEMS characteristic. Integrated on its own it would drift the angle by
  roughly 54° per minute, which is precisely what the complementary filter's
  accelerometer correction is there to cancel. Seeing the drift I'd only read
  about made the reason for sensor fusion concrete.

**Next:** port the complementary filter to hardware, then verify the gyro sign
agrees with the accelerometer angle on this physical mounting. Also worth
measuring the resting gyro bias and subtracting it, so the filter starts from a
cleaner signal.

---

<!-- Copy this template for each new entry:

## YYYY-MM-DD - [Short title]

**What I did:**

**What went wrong:**

**How I fixed it:**

**What I learned:**

-->