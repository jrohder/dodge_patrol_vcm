# Dodge Patrol VCM

ESP32-S3 **Vehicle Control Module** firmware for a Kids Trax Dodge Charger
conversion — built like an automotive ECU, not an Arduino sketch.

The ESP32-S3 is the master computer: vehicle dynamics, steering control,
differential steering, speed calculation, safety monitoring, diagnostics,
calibration, configuration, logging, WiFi dashboard and OTA updates. An
Arduino Nano R4 acts purely as a real-time extended I/O processor (RC
decoding, hall decoding, raw ADC) and streams raw engineering values over
UART — it knows nothing about the vehicle.

**Core philosophy: hardware-specific things are fixed in firmware; vehicle
behavior is configurable from the web interface.** After the initial USB
flash, commissioning, calibration, tuning, diagnostics and firmware updates
are all done from a phone.

## Features

- **Safety state machine** — BOOT → INITIALIZING → NOT_CALIBRATED → READY →
  DRIVING (+ CALIBRATION / DIAGNOSTIC / OTA / FAULT / ESTOP), every
  transition logged. Motor outputs default OFF in every non-driving state.
- **Control arbitration** — RC remote, in-vehicle manual (steering wheel +
  pedal), web remote, calibration and diagnostics all produce the same
  standardized command; exactly one source owns the vehicle. RC override
  with configurable threshold and hold time.
- **Closed-loop steering** — Firgelli actuator feedback potentiometer + PID
  at 200 Hz with soft limits, rate limiting, deadband, overcurrent trip and
  feedback-loss detection. Optional start/hold PWM hysteresis, gain
  scheduling and feed-forward (off until enabled). The P3022 SPI encoder
  reads the *steering wheel* (driver input), not the actuator.
- **Steering diagnostics** — 200 Hz 60-second rolling recorder, live graph
  on iPhone, CSV/JSON/PNG export, smart actuator characterization and
  baseline comparison. Independent of the 30-second pre-fault event recorder.
- **Differential steering** — pluggable module with SIMPLE (percentage) and
  GEOMETRY (Ackermann) strategies, aggressive turn assist, inside-wheel
  braking, all configurable.
- **Configuration database** — 100+ parameters with metadata (name, units,
  min/max, default, description, basic/advanced/expert level, danger flags)
  stored in NVS with schema versioning, APPLY (RAM) vs SAVE (flash) vs
  REVERT semantics, JSON export/import and factory reset.
- **Web dashboard** — mobile-first (iPhone Safari) single-page app served
  from flash: dashboard, web remote, auto-generated configuration editor,
  calibration wizards, live diagnostics, log viewer, firmware manager.
- **Telemetry** — one canonical structure shared by all modules, WebSocket
  streaming at a configurable rate, pause/resume, 30-second pre-fault event
  recorder with CSV download.
- **Fault system** — coded faults (`STR-001`, `COM-001`, …) with severity,
  active/history tracking and automatic recovery.
- **OTA** — local `.bin` upload and one-click install from GitHub Releases,
  dual OTA partitions, failed images are discarded and the running firmware
  keeps working. Automatic GitHub polling is **off** (use the Firmware page).
- **Commissioning-first boot** — motors forced off, then Nano/RC and the
  200 Hz control loop, then the Wi-Fi AP and web UI. I²C sensors probe in
  the background and are allowed to be offline. The dashboard shows a live
  OK / OFFLINE / NOT INSTALLED table.

## Quick start

### Build and flash (first time, USB)

```bash
pip install platformio
pio run -e esp32s3 -t upload      # build + flash over USB
pio device monitor                # optional: watch the boot log
```

Or flash a release binary: download `dodge_patrol_vcm-<ver>-factory.bin`
from [Releases](https://github.com/jrohder/dodge_patrol_vcm/releases) and
write it at offset `0x0` with esptool:

```bash
python -m esptool --chip esp32s3 write_flash 0x0 dodge_patrol_vcm-<ver>-factory.bin
```

### First boot

1. The VCM starts an **open** WiFi access point **DodgePatrol-VCM** (no
   network password — iPhone/Mac WPA on ESP32 soft-AP is unreliable).
2. Connect and open **http://192.168.4.1**. Enter the dashboard PIN once
   on that phone or computer (default `dodgepatrol`). The device is then
   remembered.
3. The vehicle boots **NOT COMMISSIONED** — driving is locked out until
   steering calibration is complete. Follow [CALIBRATION.md](CALIBRATION.md).
4. Optionally join your home WiFi: Configuration → wifi → set mode/SSID,
   SAVE, reboot. The AP automatically returns if the network is unreachable.

### Tests

```bash
pio test -e native    # host-side unit tests (PID, vehicle model, CRC, ...)
```

## Repository layout

```
src/
  core/       pins, shared types, canonical telemetry structure
  config/     parameter table (params.def) + NVS-backed registry
  control/    PID, vehicle model, speed calc, differential steering,
              steering/drive controllers, arbiter, vehicle dynamics
  drivers/    BTS7960, P3022, Firgelli ADC, MPU6050, INA3221, I²C bus,
              Nano UART link, status LED, WiFi
              link, status LED, WiFi
  services/   logger, telemetry hub, safety state machine, diagnostics,
              event recorder, steering recorder, characterization, calibration, OTA
  web/        async web server, REST API, WebSocket
web/          dashboard sources (html/css/js), gzipped into flash at build
config/       defaults.json (factory defaults reference)
test/         native unit tests
tools/        build scripts (version stamping, web asset embedding)
```

## Documentation

| Document | Contents |
| --- | --- |
| [HARDWARE.md](HARDWARE.md) | GPIO map, wiring, sensor roles, expansion |
| [PROTOCOL.md](PROTOCOL.md) | Nano ↔ ESP32 UART protocol specification |
| [CALIBRATION.md](CALIBRATION.md) | Commissioning + calibration wizards |
| [CONFIGURATION.md](CONFIGURATION.md) | Configuration system and parameters |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Fault codes and recovery |
| [STEERING_DIAGNOSTICS.md](STEERING_DIAGNOSTICS.md) | 60 s steering graph, characterization, baseline |
| [CHANGELOG.md](CHANGELOG.md) | Release history |

## Releases and OTA

Every push builds firmware in CI; pushing a tag `vX.Y.Z` publishes a GitHub
Release with the OTA image and the merged factory image attached. The
vehicle's Firmware page checks this repository's releases, shows the notes,
and installs updates over WiFi with automatic rollback protection.

## License

MIT — see [LICENSE](LICENSE).
