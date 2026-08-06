# Changelog

All notable changes to the Dodge Patrol VCM firmware.
Format: [Keep a Changelog](https://keepachangelog.com), versioning: semver.

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
