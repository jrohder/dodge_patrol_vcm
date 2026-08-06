/**
 * @file control_arbiter.h
 * @brief Selects which control source owns the vehicle.
 *
 * Every source (RC remote, manual steering wheel/pedal, web remote,
 * calibration, diagnostics) produces the same standardized ControlCommand.
 * Only ONE source owns propulsion and steering at a time - sources are
 * never blended. Priority (highest first):
 *
 *   FAULT/ESTOP > CALIBRATION/DIAGNOSTIC > RC REMOTE > WEB REMOTE > MANUAL
 *
 * The RC transmitter can override lower-priority sources when moved away
 * from neutral (configurable threshold + hold time).
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class ControlArbiter {
 public:
  /// Feed the latest web-remote command (from the WebSocket handler).
  void submitWebCommand(float throttle, float steering, bool brake,
                        bool lights, bool siren);
  /// Heartbeat keeping web control alive (sent with every ws control frame).
  void webHeartbeat() { lastWebMs_ = millis(); }
  /// Web client requested/released driving ownership.
  void setWebControlRequested(bool requested);
  bool webControlRequested() const { return webRequested_; }

  /**
   * @brief Arbitrate at 100 Hz.
   * @param rcThrottle,rcSteering -1..1 mapped RC inputs
   * @param rcValid RC signal present (from Nano)
   * @param manualThrottle,manualSteering -1..1 from pedal + P3022
   * @param manualActive pedal pressed / wheel active
   * @return the winning standardized command
   */
  ControlCommand arbitrate(float rcThrottle, float rcSteering, bool rcValid,
                           float manualThrottle, float manualSteering,
                           bool manualActive);

  ControlSource activeSource() const { return active_; }

 private:
  bool rcAwayFromNeutral(float throttle, float steering) const;

  ControlSource active_ = ControlSource::NONE;
  ControlCommand webCmd_;
  volatile uint32_t lastWebMs_ = 0;
  bool webRequested_ = false;
  uint32_t rcActiveUntil_ = 0;  ///< override hold window
};

extern ControlArbiter arbiter;

}  // namespace vcm
