# Configuration System

Everything behavioral is configuration. The single source of truth is
`src/config/params.def` — an X-macro table defining ~100 parameters, each
with:

- **Key** (`drive.max_acceleration`), **Name**, **Description**
- **Units**, **Type** (float / int / bool / enum), **Default / Min / Max**
- **Level** — basic / advanced / expert (UI filtering)
- **Flags** — `danger` (UI warns + requires confirmation),
  `restart` (takes effect after reboot)

The firmware serves this metadata at `GET /api/config/schema`; the web UI
generates its editor from it, so firmware and UI can never drift apart.
Adding a parameter is one line in `params.def`.

## Semantics

| Action | Effect |
| --- | --- |
| **APPLY** | Change RAM only — live tuning (e.g. watch steering response while changing `steering.pid_kp`). Lost on reboot. |
| **SAVE** | Persist all current values to NVS. Survives reboot, power loss and OTA. |
| **REVERT** | Reload the last saved values. |
| **Factory reset** | Restore defaults (double confirmation; calibration data is kept separately). Also: hold the boot button 10 s. |

Configuration is versioned (`schema_version`, currently **3**); on boot an older stored
schema is migrated where compatible. New steering diagnostic parameters keep
factory defaults until APPLY/SAVE (feed-forward stays off; scheduled P gains
of 0 inherit `steering.pid_kp`).

## Backup / restore

- `GET /api/config/export` — download the full configuration as JSON.
- `POST /api/config/import` — restore a previously exported file (unknown
  keys ignored, values clamped to min/max).
- `config/defaults.json` in the repository documents factory defaults.

## Categories

| Category | Contents |
| --- | --- |
| `ui` | **Display units** (`ui.units`: IMPERIAL default, or METRIC) |
| `vehicle` | wheel diameter, wheelbase, track width, mass, CoG height, name |
| `drive` | counts/rev, gear ratio, per-wheel cal factors, speed/accel/decel limits, brake mode, throttle deadband/expo, PWM limits, slip threshold |
| `steering` | max angle, rate limit, PID gains, deadband, soft-limit margin, center trim, feedback filter/validity, start/hold PWM, hysteresis, gain scheduling, derivative filter, PWM slew, feed-forward (default OFF), smart-characterization sweep, wheel-input (P3022) endpoints |
| `diff` | enable, algorithm (OFF/SIMPLE/GEOMETRY), activation/full-effect angles, reduction/boost, inside-wheel brake, speed window, ramp, aggressive assist |
| `safety` | RC/Nano/web timeouts, battery cutoff/warning, heap floor |
| `current` | shunt value, channel role mapping, offset/scale, warning/limit/trip levels |
| `imu` | axis source mapping, inversions, filter |
| `rc` | channel assignment, pulse endpoints, deadband, inversion |
| `control` | source enables, RC override threshold/hold, manual throttle level, pedal sense threshold |
| `web` | telemetry rate |
| `ota` | auto-check, pre-releases, GitHub repository |
| `log` | runtime log level |
| `wifi` | mode (AP/STA/AP_STA), SSIDs, passwords, hostname |

## Display units

Factory default is **IMPERIAL** (US customary): mph, inches, miles, pounds,
ft/s². Switch to METRIC from Configuration → `ui` → Display Units.

- Stored NVS values remain SI (`m`, `m/s`, `kg`, `m/s²`) so physics and
  JSON import/export stay unambiguous.
- The web UI converts for display and converts back on APPLY/SAVE.
- Vehicle defaults are round US values (10 in wheels, 24 in wheelbase,
  5 mph max, etc.).
- Control loops always compute in SI regardless of the display setting.

## REST API

```
GET  /api/config/schema        full metadata + current values
POST /api/config/apply         {"key": value, ...}  -> RAM
POST /api/config/save          persist to NVS
POST /api/config/revert        reload last saved
POST /api/config/factory-reset restore defaults
GET  /api/config/export        download JSON backup
POST /api/config/import        restore JSON backup
```

## Implementation notes

- Values live in a fixed array indexed by a generated enum — O(1) reads
  from 100–200 Hz control tasks, no string lookups in hot paths.
- Control modules cache derived config and refresh when the registry's
  revision counter changes (any APPLY bumps it).
- NVS keys are FNV-1a hashes of the dotted key (NVS's 15-char limit).
- Strings are mutex-protected; numeric reads are lock-free.
