# Changelog

All notable changes to the Dodge Patrol VCM firmware.
Format: [Keep a Changelog](https://keepachangelog.com), versioning: semver.

## [Unreleased]

## [1.6.0] - 2026-08-22

### Fixed
- Configuration / steering cal vanished on power cycle: the 20 kB NVS
  partition filled (`nvs_set_blob NOT_ENOUGH_SPACE`), the next boot erased
  it (`ESP_ERR_NVS_NO_FREE_PAGES`), and SAVE still logged success. Saves
  now detect write failures and compact (erase + rewrite from RAM).
  Characterization history stays in RAM so it cannot fill flash.
- Nano rate swinging 111 Hz → 3 Hz with CRC climbing: a single bad byte
  made the parser swallow the next real frames (false 0x55 0xAA sync).
  Headers now require the exact payload size for each packet type, and a
  CRC/header failure resyncs from the next byte so the following frame is
  not eaten. UART1 RX FIFO interrupts earlier (Wi-Fi can delay the ISR
  past 128 bytes at 460800).
- Nano link locked up when the dashboard or USB console was busy:
  AsyncTCP ran on core 1 with the UART/control loop, `nano.poll()` shared
  the 100 Hz dynamics task, and `Serial.print` blocked if USB CDC had no
  reader. UART drain is now a 1 kHz core-1 task, AsyncTCP is on core 0,
  pings are 2 Hz and non-blocking, and the logger never waits on serial.
- AP+STA forever-retrying a home SSID (or a trailing-space SSID) scans
  the radio and drops phone WebSocket clients. Failed STA now disconnects
  without restarting the AP. Prefer AP-only on the vehicle.
- Dashboard sends a WebSocket ping every 2 s so idle viewers are not
  marked disconnected.

### Changed
- Default `safety.nano_timeout` is 300 ms (was 100). Schema v7 raises a
  stored 100 ms value so a single late UART poll does not flash OFFLINE
  or trip COM-001.

### Added
- Factory steering limits for this vehicle (L=3738 C=2048 R=358,
  commissioned) when NVS is empty after a wipe.
- USB `dump`, `nvs`, `nano`, `reboot` console commands.
- COM-002 when Nano CRC errors burst (≥20/s).

## [1.5.0] - 2026-08-22

### Fixed
- Panic reboot loop on every boot after `nano UART`: 1.4.0 started the 200 Hz
  steering task before `ota.begin()`, so `ota.busy()` took a null FreeRTOS
  mutex (`xQueueSemaphoreTake` assert) and the chip never reached Wi-Fi.
  A power-on looked like a several-minute hang. The OTA mutex is created
  before control tasks; `busy()`/`status()` also tolerate a missing mutex.

## [1.4.0] - 2026-08-22

Boot is no longer a single blocking chain. RC/steering start before Wi-Fi,
Wi-Fi starts before I²C, and missing sensors are commissioning status — not
a reason to stall a kid’s vehicle.

### Added
- Staged boot with timing + heap/PSRAM logs at each step (GPIO safe → Nano/RC
  → control tasks → Wi-Fi AP → web server → I²C bus → background probes).
- Reset / brownout / panic / watchdog diagnostics and an RTC boot counter
  (helps tell a several-minute “hang” from a reboot loop).
- I²C bus manager: mutex, on-demand scan, stuck-bus recovery (clock SCL +
  STOP + re-init), device lifecycle UNKNOWN → PROBING → ONLINE → FAULT → RETRY.
- Commissioning status table on the serial log, Dash, and Diagnostics pages.
- Diagnostics I²C panel (GPIO9/10, 100 kHz, pin levels, INA/MPU state,
  Scan / Recover buttons — scan is **not** run at boot).
- Wi-Fi AP stress counters: uptime, assoc/leave, DHCP ok/fail, AP stops,
  radio resets, client RSSI, heap/PSRAM.
- `hw.wheel_speed_installed`, `hw.manual_steering_installed`,
  `hw.pedal_installed` (default off). Unwired inputs are **NOT INSTALLED**.
- USB/UART console (`help`, `status`, `i2c`, `recover`) so calibration and
  I²C debug work from the CH343/native USB port without joining the AP.

### Changed
- `ota.auto_check` factory default is **OFF** (schema v6 migrates existing
  NVS to off). Manual check stays on the Firmware page. If re-enabled, GitHub
  is only contacted while parked, after 5 minutes.
- I²C stays at **100 kHz** until both devices ACK; 400 kHz is a later test.
- INA3221 / MPU6050 probe in the sensor task with backoff. Control never waits.
- Missing INA/IMU do not raise SNS-001/SNS-002; only a device that was online
  then failed becomes a warning fault. Currents publish as 0 while INA is down
  (steering cal already falls back to stall-on-position).

### Fixed
- `pinMode(SDA/SCL, INPUT_PULLUP)` after `Wire.begin()` could detach the I²C
  peripheral so 0x40 and 0x68 never ACKed. Pull-ups are now set with
  `gpio_set_pull_mode` without changing the pin function.
- Full I²C address scan removed from production boot (up to ~2 s on a dead bus
  before RC/Wi-Fi).
- P3022 steering-wheel encoder stays `NOT PRESENT` until it has been seen;
  an unwired chip is not a FAULT.
- Rear-wheel stop: **BRAKE = RPWM and LPWM both high**, **COAST = both low**.
- Firgelli feedback: 16× oversample + 2-pole LPF; default cutoff **8 Hz**.
- Steering travel cal detects endstops from stall-on-position when the INA3221
  is absent.
- Soft-AP country IE **US**, `WIFI_PS_NONE`, HT20, DHCPS restart after AP
  start, captive DNS. AP is not periodically reconfigured once it is up.

## [1.3.0] - 2026-08-21

### Added
- **Steering Diagnostics**: 200 Hz / 60 s rolling recorder, live canvas
  graph (Steer page), CSV/JSON/PNG/ZIP export.
- **Smart Characterization**: position-based PWM sweep (no INA3221 required).
- Steering control extensions: start/hold PWM hysteresis, gain scheduling,
  filtered-velocity D term, optional feed-forward (**off by default**).
- Configuration schema **v5** steering diagnostic parameters.

### Fixed
- Nano UART on ESP32-S3-DevKitC-1 moved to **GPIO 1 (TX) / GPIO 2 (RX)**.
- `Serial1.setRxBufferSize(1024)` is called *before* `begin()`.
- USB console on USB-Serial-JTAG (`ARDUINO_USB_MODE=0`,
  `ARDUINO_USB_CDC_ON_BOOT=1`).
- OTA image confirmed at the start of boot so a DTR reset cannot roll back
  a just-flashed slot. Web/GitHub OTA reject merged `*-factory.bin` images.

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
