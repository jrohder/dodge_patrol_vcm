# Commissioning & Calibration

A freshly flashed VCM boots **NOT COMMISSIONED**: normal driving is locked
out and only calibration/diagnostic motion is allowed. Every calibration
stores its results, timestamp and firmware version in NVS and shows a
**VALID / NOT CALIBRATED** status in the web UI.

## Commissioning sequence

1. **Connect** — join the `DodgePatrol-VCM` AP, open `http://192.168.4.1`.
2. **Hardware check** — Dashboard/Diagnostics: Nano ONLINE, sensors OK,
   battery voltage plausible.
3. **Vehicle dimensions** — Configuration → vehicle: wheel diameter,
   wheelbase, track width. Configuration → drive: counts/revolution.
4. **Steering calibration** — wizard below (required for commissioning).
   Manual Left/Center/Right ADC remains available without current sensing.
5. **Smart characterization** (optional but recommended) — minimum PWM,
   velocity, backlash. See [STEERING_DIAGNOSTICS.md](STEERING_DIAGNOSTICS.md).
6. **Wheel/speed calibration** — motor test below.
7. **IMU calibration** — orientation + zero.
8. **Current sensor check** — zero offset capture (optional for steering).
9. **Drive limits** — Configuration → drive: max speed, acceleration,
   deceleration; Configuration → safety: battery cutoff, timeouts.
10. **Commission** — Calibration → Commissioning → *Mark Vehicle
   Commissioned*. The state machine moves to READY.

## Steering calibration (wizard)

**Setup: lift the front wheels off the ground and make sure steering travel
is unobstructed.** The wizard displays Firgelli raw/filtered ADC, position
%, estimated angle, PWM, motor current and the hard-stop detection state
live.

1. Open Calibration → Steering, review the parameters (default 25% PWM,
   3.0 A current threshold, 300 ms no-motion hold).
2. **Start Calibration.** The actuator drives slowly toward the left stop.
3. A hard stop is detected only when **current rises AND position stops
   changing** for the hold period — never from current alone. Power is
   removed immediately and the limit recorded.
4. The sequence repeats toward the right stop, the center is calculated,
   and soft limits are saved with the configurable margin
   (`steering.soft_limit_margin`, default 3%).
5. **Manual override** — you can enter Left/Center/Right ADC values
   directly if needed.
6. **Center fine-tune** — use the −5/−1/+1/+5 trim buttons while watching
   the wheels (`steering.center_trim`); APPLY is live, SAVE persists.
7. **Steering test** — LEFT / CENTER / RIGHT buttons and a 0–100% slider
   drive the actuator closed-loop at commissioning power.

Normal operation never drives into the calibrated hard stops — soft limits
keep the configured margin.

## Speed calibration

**Setup: lift the rear wheels.**

1. Open Calibration → Speed. Live values: per-wheel frequency, pulse
   counters, RPM, speed.
2. Pick a test level (10 / 20 / 30%) and run LEFT / RIGHT / BOTH motor
   tests (auto-stop after 5 s; commissioning power is limited).
3. Verify wheel direction and plausibility. Adjust
   `drive.counts_per_rev`, `drive.left_cal_factor`,
   `drive.right_cal_factor` in Configuration → drive until computed speed
   matches reality. The firmware only cares about counts/revolution, not
   how many magnets physically exist.

## IMU calibration

1. Configuration → imu: choose which sensor axis feeds each vehicle axis
   (`axis_x/y/z_source`) and inversions — the module can be mounted in any
   orientation.
2. Calibration → IMU shows raw and corrected X/Y/Z plus pitch/roll/yaw
   rate side by side.
3. With the vehicle stationary and level, press **ZERO IMU**.

## Current sensors

With everything idle, Calibration → Current Sensors → *Capture Zero
Offset*, then APPLY+SAVE `current.offset` in Configuration. Scale, warning,
limit and trip levels per role are in Configuration → current.

**Current sensing is optional for steering.** Manual Left/Center/Right
entry and Smart Characterization work when the INA3221 is not fitted or
not trusted.

## Smart actuator characterization

**Setup: vehicle supported, linkage clear, wheels free, people clear,
emergency disconnect reachable.** This wizard does **not** rediscover
travel limits and does **not** require current sensing. Run position
calibration first.

1. Calibration → **Smart Characterization**. Read the warning, tick the
   confirmation, then Start (a second confirm is required).
2. The actuator moves to center, then sweeps PWM left and right while
   measuring Firgelli motion (repeatable movement = minimum start PWM).
3. Review the results table, PWM vs velocity graphs, and recommended
   settings. **Apply Recommended** writes RAM only; **SAVE** persists.
   Feed-forward stays off until you enable it.
4. **Save as Baseline** after a known-good run. Later runs show
   **Performance Deviation** vs that baseline (not a specific failure).

Full procedure, graph interpretation and export: [STEERING_DIAGNOSTICS.md](STEERING_DIAGNOSTICS.md).

## After calibration

Configuration → Export JSON gives you a complete backup of the working
setup (import it later or onto a replacement board). Calibration data
itself survives reboots, OTA updates and configuration factory resets.
