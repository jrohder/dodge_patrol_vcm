# Changelog

All notable changes to the Dodge Patrol VCM firmware.
Format: [Keep a Changelog](https://keepachangelog.com), versioning: semver.

## [Unreleased]

### Added
- USB/UART console (`help`) so steering calibration and characterization
  can be run from the CH343 serial port without joining the AP.
- I2C boot scan; INA3221 probes 0x40–0x43; MPU6050 probes 0x68/0x69.

### Changed
- Rear-wheel stop: **BRAKE = RPWM and LPWM both high**, **COAST = both low**.
  The previous driver always wrote both-low at zero, so `drive.brake_mode`
  did nothing. Steering still coasts at rest (do not short the actuator).
- I2C bus 100 kHz with GPIO pull-ups (400 kHz was failing `requestFrom`).
- Firgelli feedback: 16× oversample + 2-pole LPF; default cutoff **8 Hz**
  (schema v5). The old 30 Hz 1-pole filter tracked ADC noise almost 1:1.
- Steering travel cal detects endstops from stall-on-position when the
  INA3221 is absent (previously required current AND stall, so it never
  finished).
- Calibration/diagnostic states are allowed from FAULT and are not yanked
  back by COM-001, so wizards work while the Nano is offline.

### Added
- **Dashboard PIN**: WiFi AP is open; each phone/browser enters
  `web.access_pin` (default `dodgepatrol`) once and is remembered via cookie.
  AP country IE forced to **US** (Arduino defaulted to CN, which made Apple
  clients AssocFail even on an open network). DHCPS is restarted after AP
  start (Apple was associating then leaving with no IP), DHCP offers DNS,
  and a captive DNS responder answers `*` → `192.168.4.1`.
- **Steering Diagnostics**: 200 Hz / 60 s rolling recorder, live canvas
  graph (Steer page), trace presets, pause/live/cursor, hunting indicator,
  CSV/JSON/PNG/ZIP export. Independent of the 30 s pre-fault event recorder.
- **Smart Characterization**: position-based PWM sweep (no INA3221 required)
  for left/right start/hold PWM (hold is measured by walking PWM down after
  start is proven), velocity, backlash and settling; graphs; conservative
  recommended settings (Apply vs Keep); baseline + history +
  performance-deviation comparison. E-stop / OTA / output-disable aborts
  the wizard and zeros the actuator.
- Steering control extensions on the existing 200 Hz PID: start/hold PWM
  hysteresis, gain scheduling (0 = inherit `pid_kp`), filtered-velocity D
  term, stronger anti-windup, optional feed-forward (**off by default**).
- Configuration schema **v3** with the new `steering.*` diagnostic and
  characterization parameters.

### Fixed
- Soft-AP is forced to WPA2-PSK / CCMP / HT20 on channel 6 so macOS/iOS
  stop reporting “incorrect password” for `dodgepatrol` after a USB factory
  flash (Arduino `WiFi.softAP()` left `wifi_config_t` uninitialized).
- **OTA appeared to succeed then still reported the previous version:**
  app rollback was only cancelled after USB CDC came up, so a host opening
  the serial port (DTR reset) reverted to the other OTA slot. The running
  image is now confirmed at the start of boot. Web/GitHub OTA also reject
  merged `*-factory.bin` images (bootloader magic, not an app image).
- Nano UART on ESP32-S3-DevKitC-1 moved from GPIO 43/44 (silkscreen TX/RX,
  hard-wired to the onboard CP2102) to **GPIO 1 (TX) / GPIO 2 (RX)** so
  telemetry can actually reach the VCM.
- `Serial1.setRxBufferSize(1024)` is now called *before* `begin()`; the
  previous order left the 256-byte default RX ring in place.
- USB console routed through USB-Serial-JTAG (`ARDUINO_USB_MODE=0`,
  `ARDUINO_USB_CDC_ON_BOOT=1`) so boot/NANO logs appear on the native USB
  port instead of UART0.

## [1.2.0] - 2026-08-12

### Added
- **`ui.units`** configuration (IMPERIAL | METRIC). Factory default is
  **IMPERIAL** (US customary): dashboards and the config editor show mph,
  inches, miles, pounds, and ft/s². Control math remains SI internally;
  the UI converts on display and on APPLY/SAVE.
- Telemetry `sys.units` and `/api/system` expose the active preference.

### Changed
- Factory vehicle/drive defaults rounded to US values: 10 in wheels,
  24 in wheelbase, 20 in track, 66 lb mass, 5 mph max, 3 mph reverse,
  2.5 / 5.0 ft/s² accel/decel.
- Configuration schema version bumped to 2.

## [1.1.1] - 2026-08-06

### Fixed
- **Verified against public `vcm_extended_io`**: replaced the reconstructed
  protocol with a **byte-identical** copy of Nano `src/proto/protocol.{h,cpp}`
  (`veio::proto`). Critical mismatches in the earlier reconstruction:
  - Sync is `0x55 0xAA` (not AA/55)
  - Protocol version is **1** (Nano’s `kProtocolVersion`)
  - CRC covers bytes after the sync preamble only
  - Telemetry field layout matches Nano (rcAgeMs, 12-byte WheelData with
    signed pulseCount + freqHzX10 + periodUs, motor/battery ADC pairs)
  - Commands are `kPktCommand` (0x40) with Nano’s CommandId set; keepalive
    is `kCmdPing` (Nano answers ACK; no ESP32→Nano heartbeat packet)
- Diagnostics consume HEARTBEAT / DIAGNOSTIC / VERSION / FAULT packets.
- `PROTOCOL.md` synced from Nano docs.

## [1.1.0] - 2026-08-06

### Changed
- First attempt at Nano UART protocol alignment (superseded by 1.1.1 once
  `vcm_extended_io` became public and could be verified byte-for-byte).

## [1.0.0] - 2026-08-06

Initial release.

### Added
- Safety state machine (BOOT / INITIALIZING / NOT_CALIBRATED / READY /
  DRIVING / CALIBRATION / DIAGNOSTIC / OTA / FAULT / ESTOP) with logged
  transitions and outputs-default-OFF policy.
- Control arbitration: RC remote, manual vehicle (P3022 steering wheel +
  pedal), web remote, calibration and diagnostic sources produce one
  standardized command; single-owner rule with configurable RC override.
- Closed-loop steering: 200 Hz PID on Firgelli feedback with soft limits,
  rate limiting, deadband, overcurrent trip, feedback-loss detection and
  live tuning.
- Vehicle dynamics: throttle shaping, speed/accel/decel limits, per-wheel
  speed targets; drive controller with stiction compensation and
  three-level current handling (warn/limit/trip).
- Differential steering module: SIMPLE and GEOMETRY (Ackermann)
  strategies, aggressive turn assist, inside-wheel braking, authority
  ramping — all configurable.
- Configuration database: ~100 metadata-rich parameters in NVS with schema
  versioning, APPLY/SAVE/REVERT, JSON export/import, factory reset (web +
  10 s boot button).
- Nano R4 UART link: versioned binary protocol with CRC16, sequence
  tracking, link statistics and watchdog fault (COM-001).
- Sensors: INA3221 (configurable channel roles), MPU6050 (configurable
  mounting orientation + zeroing), P3022 SPI encoder, Firgelli ADC with
  filtering and validity checks.
- Mobile-first web dashboard: live dashboard, WebSocket web remote with
  heartbeat failsafe, auto-generated configuration editor, calibration
  wizards (steering auto-cal, motor tests, IMU, current), diagnostics
  (faults, PID terms, UART monitor, task timing), log viewer, firmware
  manager.
- Telemetry: canonical shared structure, configurable WS rate,
  pause/resume, 30 s pre-fault event recorder with CSV download.

- OTA: local .bin upload + GitHub Releases check/install with semantic
  version comparison, dual OTA partitions and validation rollback.
- CI: native unit tests + firmware build on every push; tagged releases
  publish OTA and factory .bin assets automatically.
- Documentation: README, HARDWARE, PROTOCOL, CALIBRATION, CONFIGURATION,
  TROUBLESHOOTING.
