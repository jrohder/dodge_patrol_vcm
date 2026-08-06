# Troubleshooting

## Fault codes

Faults appear on the Diagnostics page with active state, occurrence count
and timestamps. **TRIP** faults force the FAULT state (propulsion commanded
safe) until the condition clears; **WARNING** faults are logged and shown.
*Clear Fault History* never erases active conditions.

| Code | Severity | Meaning | Recovery |
| --- | --- | --- | --- |
| COM-001 | TRIP | Nano communication timeout | Check UART wiring (GPIO43/44), Nano power, baud (460800). Clears automatically when packets resume. |
| COM-002 | WARNING | Excessive Nano CRC errors | Check wiring/shielding and shared ground. |
| COM-003 | TRIP | Nano protocol version mismatch | Flash matching Nano firmware (see PROTOCOL.md). |
| RC-001 | TRIP | RC signal lost while RC had control | Restore transmitter; clears on signal return. |
| WEB-001 | TRIP | Web control heartbeat lost while web had control | Vehicle stops automatically. Re-take control from the Remote page. |
| STR-001 | TRIP | Steering feedback lost | Firgelli potentiometer wiring to GPIO4; check `steering.feedback_min/max_valid`. |
| STR-002 | TRIP | Steering overcurrent | Mechanical obstruction or `current.steering_trip` too low. |
| STR-003 | WARNING | Steering not calibrated | Run the steering calibration wizard. |
| DRV-001/002 | TRIP | Left/right motor overcurrent | Obstruction, stall or `current.drive_trip` too low. |
| PWR-001 | TRIP | Battery under-voltage cutoff | Charge the battery. Clears with 0.5 V hysteresis. |
| PWR-002 | WARNING | Battery voltage low | Charge soon. |
| SNS-001 | WARNING | IMU failure | I2C wiring (GPIO9/10), MPU6050 at 0x68. |
| SNS-002 | WARNING | Current monitor failure | INA3221 at 0x40. |
| SYS-001 | WARNING | Control task overrun | Check task timing on the Diagnostics page. |
| SYS-002 | WARNING | Low free heap | Reduce telemetry rate; report if persistent. |

## Common situations

**Can't reach the dashboard**
- AP mode: join `DodgePatrol-VCM` (password `dodgepatrol`), open
  `http://192.168.4.1`.
- STA mode: try `http://dodge-patrol.local`. If the configured network is
  unreachable, the AP comes back automatically after ~20 s.
- Last resort: USB serial monitor at 115200 shows the IP and full log.

**Vehicle won't drive**
- Header badge shows the state: NOT CALIBRATED → run steering calibration
  and commission the vehicle; FAULT → see Diagnostics; ESTOP → clear from
  the dashboard.
- Check the active control source is enabled in Configuration → control.

**Steering oscillates or is sluggish**
- Tune `steering.pid_kp/ki/kd` live with APPLY while watching the PID
  terms on the Diagnostics page. Raise the deadband if it hunts at center.

**Speed reads wrong**
- Verify `drive.counts_per_rev` and wheel diameter; use the speed
  calibration wizard and per-wheel calibration factors.

**Wheels turn the wrong way / steering reversed**
- `steering.invert_output`, `steering.wheel_invert`,
  `rc.invert_steering/throttle` cover all combinations without rewiring.

**OTA failed**
- The previous firmware keeps running; the error is shown on the Firmware
  page and logged. Verify the .bin is an OTA image (not the factory merged
  image) built for ESP32-S3.

**Factory reset**
- Web: Configuration → Restore Factory Defaults (double confirmation), or
  hold the boot button 10 seconds. Calibration data is kept separately;
  re-commissioning is not required after a config-only reset.

## Diagnostics data to include in bug reports

Download from the web UI: Logs → Download, Diagnostics → event recorder
CSV, Configuration → Export JSON, plus firmware version from the System
page.
