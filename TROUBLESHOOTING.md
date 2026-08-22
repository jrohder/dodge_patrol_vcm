# Troubleshooting

## Fault codes

Faults appear on the Diagnostics page with active state, occurrence count
and timestamps. **TRIP** faults force the FAULT state (propulsion commanded
safe) until the condition clears; **WARNING** faults are logged and shown.
*Clear Fault History* never erases active conditions.

| Code | Severity | Meaning | Recovery |
| --- | --- | --- | --- |
| COM-001 | TRIP | Nano communication timeout | Check UART wiring (**GPIO1 TX / GPIO2 RX**, not silkscreen TX/RX), Nano power, baud (460800), 5 V→3.3 V divider on Nano TX. Clears automatically when packets resume. |
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
| SNS-001 | WARNING | IMU failure | Only after the MPU6050 was online then dropped. I2C GPIO9/10, 0x68. Not present at boot is OFFLINE, not this fault. |
| SNS-002 | WARNING | Current monitor failure | Only after the INA3221 was online then dropped. 0x40. Missing chip does not trip the vehicle. |
| SYS-001 | WARNING | Control task overrun | Check task timing on the Diagnostics page. |
| SYS-002 | WARNING | Low free heap | Reduce telemetry rate; report if persistent. |

## Common situations

**Takes minutes before RC or the AP works**
- That is not a normal boot. 1.4.0 starts motors-off, Nano/RC, then control
  tasks, then the AP — I²C is background. If it still takes minutes, the
  serial log is the first thing to capture. Look for `reset=`, `BROWNOUT`,
  `PANIC`, `WDT`, `boot#` climbing (reboot loop), and `BOOT +Nms` timings.
- A brownout on the 5 V / 3.3 V rail when BTS7960s or Wi-Fi come up will look
  like “it finally starts working” after several crash/reboot cycles.
- Diagnostics → ESP reset / boot shows the last reset reason, boot count, and
  milliseconds to control / Wi-Fi / web / ready.

**INA3221 or MPU6050 not communicating**
- Open Diagnostics → I²C bus. SDA should be GPIO9, SCL GPIO10, 100 kHz.
  Press **Scan I²C Bus** (never done automatically at boot).
- Need `0x40` (INA3221) and `0x68` (MPU6050, or `0x69` if AD0 is high).
  If neither ACKs, it is wiring/power, not the driver: SDA/SCL swap, no 3.3 V,
  no common ground, missing pull-ups, bus held low. **Recover Bus** bit-bangs
  SCL if a device is sitting on the lines.
- Current-sensor **power** wiring is separate: the INA3221 is high-side on
  each BTS7960 **supply**, not in the PWM motor leads. See HARDWARE.md.
- Missing INA/IMU is **OFFLINE**, not a trip. Steering and RC still run.

**Wi-Fi associates then drops / no IP**
- Join open `DodgePatrol-VCM`, then `http://192.168.4.1`. Diagnostics → Wi-Fi
  AP shows assoc vs leave vs DHCP fail vs min heap. If min heap is tiny or
  boot count climbs, it is power or memory, not the password.
- AP country is US, HT20, power-save off. The radio is not restarted in a loop
  once the AP is up.

**Can't reach the dashboard**
- AP mode: join **open** network `DodgePatrol-VCM` (no WiFi password), open
  `http://192.168.4.1`, then enter the dashboard PIN once (default
  `dodgepatrol`). Each phone/browser only needs the PIN the first time.
- After a USB factory flash, NVS (saved WiFi/config) is erased, so the AP
  password is the default again. If the Mac/phone says “incorrect
  password”, Forget `DodgePatrol-VCM` and reconnect with `dodgepatrol`.
- STA mode: try `http://dodge-patrol.local`. If the configured network is
  unreachable, the AP comes back automatically after ~20 s.
- Last resort: USB serial monitor at 115200 shows the IP and full log.

**Vehicle won't drive**
- Header badge shows the state: NOT CALIBRATED → run steering calibration
  and commission the vehicle; FAULT → see Diagnostics; ESTOP → clear from
  the dashboard.
- Check the active control source is enabled in Configuration → control.

**Steering oscillates or is sluggish**
- Open **Steer** (Steering Diagnostics) and watch setpoint vs filtered
  position vs PWM at 200 Hz. Hunting shows as repeated PWM/error reversals.
- Run **Smart Characterization** (does not require INA3221) for start/hold
  PWM. Apply recommended settings to RAM, then SAVE if they help.
- Tune `steering.pid_kp/ki/kd`, `steering.hold_p_gain` and deadband live
  with APPLY. Keep I at 0 until the loop is stable. Do not enable
  feed-forward until the PWM→velocity curve looks sane.
- See [STEERING_DIAGNOSTICS.md](STEERING_DIAGNOSTICS.md).

**Speed reads wrong**
- Verify `drive.counts_per_rev` and wheel diameter; use the speed
  calibration wizard and per-wheel calibration factors.

**Wheels turn the wrong way / steering reversed**
- `steering.invert_output`, `steering.wheel_invert`,
  `rc.invert_steering/throttle` cover all combinations without rewiring.

**OTA says successful, reboots, still the old version**
- Arduino-ESP32 enables app rollback. The new image must call
  `esp_ota_mark_app_valid_cancel_rollback()` *before* USB CDC enumerates.
  If a host (serial monitor, esptool, the Mac) opens the port, DTR resets
  the chip and the bootloader reverts to the previous OTA slot. Firmware
  after this fix confirms the image at the very start of boot.
- Do not upload `*-factory.bin` via the web UI. That file starts with a
  bootloader, Arduino Update can still report success, then the new slot
  does not run and rollback returns you to the old version. Use
  `dodge_patrol_vcm-<ver>.bin` for OTA / web upload.

**Can't USB-flash (Failed to connect / Device not configured)**
- `ARDUINO_USB_CDC_ON_BOOT=1` makes opening the serial port reset the
  chip, so the `/dev/cu.usbmodem*` node vanishes under esptool. Hold
  **BOOT**, tap **RESET**, keep holding BOOT until esptool says
  `Connecting...`. The device should enumerate as USB JTAG/serial debug,
  not "ESP32-S3-DevKitC-1-N8". Then flash
  `dodge_patrol_vcm-<ver>-factory.bin` at offset `0x0`.

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
CSV, **Steer → Export Steering Diagnostics** (ZIP of config, calibration,
characterization and the 60 s CSV), Configuration → Export JSON, plus
firmware version from the System page.
