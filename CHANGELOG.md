# Changelog

All notable changes to the Dodge Patrol VCM firmware.
Format: [Keep a Changelog](https://keepachangelog.com), versioning: semver.

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
