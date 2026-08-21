/**
 * @file bts7960.h
 * @brief BTS7960 dual half-bridge motor driver (RPWM/LPWM) via ESP32 LEDC.
 *
 * Outputs default OFF (both PWM low = coast) in begin() as early as
 * possible. At zero command: COAST = RPWM and LPWM low; BRAKE = both high
 * (IBT-2 / BTS7960 short-brake). Steering uses COAST; drive follows
 * drive.brake_mode.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

class Bts7960 {
 public:
  enum class StopMode { COAST, BRAKE };

  /**
   * @param name short name for logging ("LEFT", "RIGHT", "STEER")
   * @param rpwmPin,lpwmPin BTS7960 PWM inputs
   * @param chR,chL LEDC channels
   */
  Bts7960() = default;
  Bts7960(const char* name, int rpwmPin, int lpwmPin, int chR, int chL)
      : name_(name), rpwmPin_(rpwmPin), lpwmPin_(lpwmPin), chR_(chR), chL_(chL) {}

  /// Configure pins/PWM and force outputs off. Call early in boot.
  void begin();

  /**
   * @brief Drive the motor.
   * @param pct -100..+100; sign selects direction, magnitude sets duty.
   *            Values are clamped. 0 applies the configured stop mode.
   */
  void drive(float pct);

  /// Immediately stop with the given mode (overrides configured mode once).
  void stop(StopMode mode);

  void setStopMode(StopMode mode) { stopMode_ = mode; }
  void setInverted(bool inv) { inverted_ = inv; }
  void setMaxPct(float maxPct) { maxPct_ = constrain(maxPct, 0.0f, 100.0f); }

  float currentPct() const { return currentPct_; }

 private:
  void write(float rDuty, float lDuty);
  void applyStop(StopMode mode);

  const char* name_ = "?";
  int rpwmPin_ = -1, lpwmPin_ = -1, chR_ = -1, chL_ = -1;
  StopMode stopMode_ = StopMode::BRAKE;
  bool inverted_ = false;
  float maxPct_ = 100.0f;
  float currentPct_ = 0.0f;
};

}  // namespace vcm
