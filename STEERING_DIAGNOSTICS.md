# Steering Diagnostics & Smart Characterization

This document covers the **Steering Diagnostics** page (live 60-second
graph) and **Smart Actuator Characterization**. It does not replace the
existing 200 Hz PID steering controller, travel calibration, or the
30-second pre-fault event recorder.

Related:

- [CALIBRATION.md](CALIBRATION.md) — travel limits, commissioning
- [CONFIGURATION.md](CONFIGURATION.md) — parameter keys
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — hunting, faults

## Sensor roles (do not confuse)

| Sensor | What it measures | Role |
| --- | --- | --- |
| **P3022** (SPI) | Steering **wheel** / driver input | Requested steering |
| **Firgelli pot** (GPIO4 ADC) | Actuator **position** | Closed-loop feedback |

```
P3022 → requested position → target → PID + compensation → BTS7960
                                                         ↓
                                              Firgelli actuator
                                                         ↓
                                         Firgelli pot → actual position
                                                         ↓
                                              back into the PID
```

The INA3221 current sensor is **optional**. Travel calibration, smart
characterization, minimum PWM, velocity mapping and PID tuning all work
without it. When the INA is healthy it can add a current trace and later
improve stall / hard-stop / load analysis. Current is never faked: CSV/JSON
leave the field blank when the sensor is absent or faulted.

## Commissioning workflow

1. Enter vehicle configuration.
2. Establish steering **limits** (wizard or manual Left / Center / Right ADC).
3. Center the steering (trim).
4. Run **Smart Characterization**.
5. Review graphs and recommended settings.
6. **Apply Recommended** (RAM only) or keep current.
7. Test steering; watch the 60-second graph.
8. Tune manually if needed; **SAVE** configuration.
9. Run characterization again and **Save as Baseline**.

Position calibration, smart characterization and current-sensor calibration
are independent. Characterization **requires** existing limits; it does not
rediscover hard stops.

## Steering Diagnostics page

Open **Steer** on the tab bar (or Diagnostics → *Open Steering Diagnostics*).

The graph is a rolling oscilloscope of the last **60 seconds**. New samples
append; samples older than 60 s drop off. It does **not** clear every minute.

| Layer | Rate | Notes |
| --- | --- | --- |
| Steering control | 200 Hz | Never blocked by the graph, WebSocket, or CSV |
| On-vehicle recorder | 200 Hz | Packed 28-byte samples, ~12 000 for 60 s (PSRAM if present; SRAM fallback shrinks the window, not the 200 Hz loop) |
| Browser display | ~20 FPS | Downsampled to the canvas width |
| Live numeric panel | 10–20 Hz | Same WebSocket telemetry as the rest of the dashboard |

### Traces

Default (Basic): **Setpoint**, **Filtered Feedback**, **PWM**.

| Trace | Units | Meaning |
| --- | --- | --- |
| Setpoint | % | Rate-limited actuator target |
| Filtered Feedback | % | Position the PID uses |
| Raw Feedback | ADC | Unfiltered Firgelli ADC (cursor shows counts) |
| Error | % | Setpoint − filtered |
| PWM | % | Commanded actuator PWM (signed) |
| P / I / D | % | PID terms |
| Feed Forward | % | Characterization-based term (0 unless enabled) |
| Velocity | %/s | Filtered d(position)/dt |
| Current | A | Only when INA3221 is OK |

Presets: **Basic**, **Control Loop**, **Mechanical**, **Characterization**,
**Everything**.

Position and PWM use **separate vertical scales**. PWM is not plotted on
0–100% position if that would hide the trace.

### Interaction

- **PAUSE** — freeze the view. The ESP32 recorder **keeps running**.
- **LIVE** — jump to the current 60-second window.
- **Reset View** — 60 s span, clear cursor.
- **CLEAR** — confirm, then wipe the diagnostic recorder only (not
  characterization history).
- Pinch-zoom and cursor inspection (nearest sample, ESP32 timestamps).
- Vertical event markers: calibration, direction change, limits, PWM
  ceiling, faults, characterization steps, PWM test levels.

### Export

| Button | Contents |
| --- | --- |
| Download CSV | Full 60 s high-resolution buffer (`timestamp_us`, `time_s`, …). Blank `current` if unavailable. |
| Download JSON | Firmware / schema / calibration version + samples + event markers |
| Download PNG | Browser canvas → iPhone Photos/Files |
| Save Session | JSON snapshot (firmware, cal, characterization, traces) |
| Export Steering Diagnostics | ZIP: `metadata.json`, `configuration.json`, `steering_calibration.json`, `characterization.json`, `diagnostic.csv` |

The 30-second **event recorder** on the Diagnostics page is unchanged
(pre-fault log). Do not confuse the two.

### Statistics and hunting

The footer shows RMS / max / average error, peak and average PWM, max
velocity, direction changes and PWM reversals for the **visible window**.

**Hunting Detected** (with severity) flags repeated PWM / error reversals.
It does **not** change PID gains.

A simple **health** number (0–100) is a hint only. Raw traces remain
authoritative.

## Smart Characterization

**Calibration → Smart Characterization**, separate from position
calibration and current-sensor calibration.

### Safety

The wizard will not start without:

1. The on-screen warning acknowledged.
2. JSON `confirm: true` and `ack: "ACTUATOR WILL MOVE"`.

The actuator then moves by itself. Support the vehicle, keep the linkage
clear, keep people away, keep the emergency disconnect reachable.

### Sequence

1. Verify Firgelli feedback is valid.
2. Verify travel limits exist — otherwise **stop** and tell you to run
   position calibration.
3. Move to center (existing PID).
4. Verify stationary.
5. LEFT PWM sweep (position-based movement, no current required).
6. Return to center.
7. RIGHT PWM sweep.
8. Return to center.
9. Settling / backlash measurements.
10. Analyze and display. Partial runs are saved as **FAILED**, not discarded.

Sweep parameters (start PWM, increment, max PWM, dwell, settle, motion
threshold) are configuration, not hardcoded.

**Minimum start PWM** is the first level that produces **repeatable**
motion (apply, stop, repeat). A single ADC count is not enough.

**Hold PWM** is measured after start PWM is found: PWM is walked downward
until motion is no longer repeatable, then velocity mapping continues upward.

**Preferred PWM** is a moderate, reliable speed — not the highest PWM.

LEFT and RIGHT are measured independently.

If feedback is lost, a fault occurs, the loop misses deadlines, or the
wizard times out, the actuator is stopped and partial data is kept.

### Results and recommendations

The results table shows start / hold / preferred / max-tested PWM, max
velocity and estimated backlash per side.

**Recommended** PID / PWM / deadband / slew values are shown next to
**current** settings. **Apply Recommended** writes them to RAM only
(then SAVE in Configuration). **Keep Current** leaves the controller
alone. Feed-forward stays **OFF** until you enable it.

Recommendations are conservative (I = 0, modest P). The goal is a
measurement system first, not a black-box auto-tuner.

### Baseline and wear

**Save as Baseline** stores the known-good actuator response on the
vehicle. A later run shows **CURRENT vs BASELINE** with percent
**Performance Deviation** (for example minimum PWM +25% or max velocity
−23%). That can mean friction, binding, wear or electrical change — the
firmware does **not** name a specific failure.

Graphs overlay baseline vs current PWM→velocity (dashed vs solid).

**Characterization History** lists previous NVS records.

### Graphs

1. PWM vs actuator velocity (left / right; baseline overlay when present)
2. Time vs position (command / actual from the 60 s recorder)
3. PWM vs position error
4. Direction reversal / backlash (position vs time)

Download PNG / CSV / JSON / a self-contained **HTML report** (graphs
embedded) for the phone.

## Control extensions (same PID)

These run inside the existing 200 Hz controller. New algorithms default
so an OTA does **not** suddenly retune a working vehicle:

| Feature | Default |
| --- | --- |
| Start / hold PWM | 0 (compensation off until characterization / manual set) |
| Far / near / hold P | 0 = inherit `steering.pid_kp` |
| Feed-forward | OFF |
| Integral | existing `steering.pid_ki` (recommendations suggest 0) |
| Derivative | from **filtered velocity**, not raw ADC |
| Anti-windup | freeze I in deadband / hold / limit / saturation |
| PWM hysteresis | 2% on direction reversal |
| PWM slew | 0 = unlimited until recommended / configured |

Hysteresis: if stopped and the PID asks for 10% but start PWM is 18%,
the motor stays **off** (the 10% is not boosted). Once moving, hold PWM
can floor the command until error is inside the hold band. That avoids
START/STOP chatter.

## Interpreting the graph

**Hunting** — PWM and error reverse many times around the setpoint while
the vehicle is still. Raise deadband, lower P near target (`hold_p_gain`),
apply start/hold PWM from characterization, keep I at 0 until stable.

**Sluggish large moves** — far P and rate limit; later, enable feed-forward
from a good PWM→velocity curve.

**Direction delay** — backlash traces; do not auto-compensate yet. Save
the number for comparison.

**No current trace** — INA not ready; ignore and characterize anyway.

## REST / WebSocket

```
GET  /api/steer/diag/status
GET  /api/steer/diag/csv
GET  /api/steer/diag/json
GET  /api/steer/diag.bin          packed snapshot for the graph
POST /api/steer/diag/clear
GET  /api/steer/diag/events
GET  /api/steer/export/metadata.json

POST /api/cal/char/start          {"confirm":true,"ack":"ACTUATOR WILL MOVE"}
POST /api/cal/char/abort
GET  /api/cal/char/status
POST /api/cal/char/apply-recommended
POST /api/cal/char/save-baseline
GET  /api/cal/char/json
GET  /api/cal/char/csv
GET  /api/cal/char/history
```

WebSocket `/ws` (existing): `{ "type": "sdiag", "on": true }` then binary
frames `SD` + packed samples. Steering control continues if Wi‑Fi or the
browser disconnects.
