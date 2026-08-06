/**
 * @file pins.h
 * @brief ESP32-S3 GPIO assignment for the Dodge Patrol VCM.
 *
 * Hardware-specific pin mapping is fixed in firmware (see HARDWARE.md).
 * Vehicle *behavior* is configurable from the web interface; pins are not.
 *
 * Unused GPIOs are intentionally left free for future expansion
 * (GPS, CAN, display, temperature, additional lights/actuators).
 */
#pragma once

namespace pins {

// UART link to Arduino Nano R4 (extended IO processor)
constexpr int NANO_TX = 43;  ///< ESP32 TX -> Nano RX
constexpr int NANO_RX = 44;  ///< ESP32 RX <- Nano TX

// I2C sensor bus (INA3221 current monitor, MPU6050 IMU) @ 400 kHz
constexpr int I2C_SDA = 9;
constexpr int I2C_SCL = 10;

// SPI - P3022 absolute hall angle encoder (steering WHEEL input, 4096 counts)
constexpr int P3022_CS = 11;
constexpr int P3022_MOSI = 12;
constexpr int P3022_CLK = 13;
constexpr int P3022_MISO = 14;

// Steering actuator BTS7960 (Firgelli linear actuator)
constexpr int STEER_RPWM = 8;
constexpr int STEER_LPWM = 3;

// Left drive motor BTS7960
constexpr int LEFT_RPWM = 15;
constexpr int LEFT_LPWM = 16;

// Right drive motor BTS7960
constexpr int RIGHT_RPWM = 17;
constexpr int RIGHT_LPWM = 18;

// Firgelli actuator built-in 10k feedback potentiometer (ADC1).
// This is the PRIMARY closed-loop steering position feedback.
constexpr int FIRGELLI_ADC = 4;

// WS2812 RGB status LED
constexpr int STATUS_LED = 47;

// Boot button (bootloader; held 10 s at runtime = factory reset)
constexpr int BOOT_BUTTON = 0;

// LEDC PWM channel allocation (ESP32-S3 has 8 channels)
constexpr int CH_STEER_R = 0;
constexpr int CH_STEER_L = 1;
constexpr int CH_LEFT_R = 2;
constexpr int CH_LEFT_L = 3;
constexpr int CH_RIGHT_R = 4;
constexpr int CH_RIGHT_L = 5;

constexpr int PWM_FREQ_HZ = 20000;  ///< Above audible range for BTS7960
constexpr int PWM_RES_BITS = 10;    ///< 0..1023 duty

}  // namespace pins
