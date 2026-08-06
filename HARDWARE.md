# Hardware

ESP32-S3 GPIO assignment for the Dodge Patrol VCM. Pins are fixed in
firmware (`src/core/pins.h`); everything behavioral is configuration.

## GPIO map

| GPIO | Function | Notes |
| --- | --- | --- |
| 43 | UART TX → Nano | commands/heartbeat, 460800 baud |
| 44 | UART RX ← Nano | telemetry packets ~100 Hz (primary sensor input) |
| 9 | I2C SDA | 400 kHz sensor bus |
| 10 | I2C SCL | |
| 11 | SPI CS — P3022 | steering **wheel** angle encoder, 4096 counts |
| 12 | SPI MOSI — P3022 | |
| 13 | SPI CLK — P3022 | |
| 14 | SPI MISO — P3022 | |
| 8 | Steering RPWM | BTS7960 → Firgelli actuator |
| 3 | Steering LPWM | |
| 15 | Left drive RPWM | BTS7960 → left rear motor |
| 16 | Left drive LPWM | |
| 17 | Right drive RPWM | BTS7960 → right rear motor |
| 18 | Right drive LPWM | |
| 4 | ADC1 — Firgelli feedback | built-in 10k potentiometer |
| 47 | WS2812 status LED | |
| 0 | Boot button | bootloader; held 10 s at runtime = factory reset |

All motor PWM runs at 20 kHz / 10-bit via LEDC channels 0–5.

## The two steering measurements (do not confuse!)

| Sensor | Measures | Used for |
| --- | --- | --- |
| **P3022** (SPI) | Driver's steering **wheel input** | What steering the child requests in manual mode |
| **Firgelli potentiometer** (GPIO4) | Actual **actuator position** | Primary closed-loop feedback for the steering PID |

```
Steering wheel → P3022 → requested steering ─┐
                                             ├─ error → PID → BTS7960 → actuator
Firgelli pot   → ADC   → actual position   ──┘
```

## I2C devices

| Device | Address | Role |
| --- | --- | --- |
| INA3221 | 0x40 | 3-channel current + bus voltage. Channel roles (left/right/steering) are **configured**, not hardwired (`current.chN_role`). |
| MPU6050 | 0x68 | Accel/gyro. Mounting orientation fully configurable (`imu.axis_*`, `imu.invert_*`). |

## Status LED

| Color | Meaning |
| --- | --- |
| Blue | Booting / initializing |
| Orange blinking | Not commissioned |
| Green | Ready / driving |
| Yellow | Calibration / diagnostic test |
| Purple blinking | OTA update |
| Red blinking | Fault / emergency stop |
| Fast blink (any color) | Nano link down |

## Nano R4 responsibilities (extended I/O processor only)

- RC receiver decoding (6 channels, pulse widths in µs)
- Hall wheel sensor decoding (frequency, direction, pulse counters)
- Raw ADC acquisition (battery divider, motor-wire sense A/B, spare)
- Local fault detection (hall fault, RC lost, ADC fault, brownout)

The Nano never sees wheel diameter, gearing, speed limits, PID gains or
vehicle modes. See [PROTOCOL.md](PROTOCOL.md).

## Outputs default OFF

Motor GPIOs are driven to a known inactive state at the earliest point in
boot, before any other subsystem starts (PCB pull-downs are the hardware
backup; firmware is the software backup). Outputs are also forced safe
during OTA, FAULT, ESTOP and while not commissioned.

## Free GPIO / future expansion

Remaining ESP32-S3 GPIOs are intentionally unused and documented here for
future modules (GPS, CAN transceiver, display, temperature sensing,
additional lights/actuators): 1, 2, 5, 6, 7, 21, 35–42, 45, 46, 48. The
modular driver/controller architecture allows adding these without
architectural rewrites.
