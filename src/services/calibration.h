/**
 * @file calibration.h
 * @brief Guided calibration wizards and persistent calibration records.
 *
 * Every calibration has a status (VALID / NOT CALIBRATED / EXPIRED) stored
 * in NVS together with results, timestamp and the firmware version that
 * produced it. Driving is locked out (NOT_CALIBRATED / "NOT COMMISSIONED")
 * until steering calibration is valid.
 *
 * Steering auto-calibration: user-initiated only, starts at very low PWM,
 * and detects a hard stop when position stops changing. If the INA3221
 * is present, current must also be elevated (never current alone). Without
 * a current sensor, stall-on-position is used so travel calibration still
 * works.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

enum class SteeringCalState : uint8_t {
  IDLE = 0,
  MOVE_LEFT,      ///< driving toward the left hard stop
  CONFIRM_LEFT,   ///< current elevated + no motion; timing confirmation
  SETTLE_LEFT,    ///< power removed; recording limit
  MOVE_RIGHT,
  CONFIRM_RIGHT,
  SETTLE_RIGHT,
  DONE,
  ABORTED,
};

struct SteeringCalData {
  bool valid = false;
  float leftAdc = 0, rightAdc = 0, centerAdc = 0;  ///< hard stop positions
  uint32_t timestamp = 0;    ///< uptime seconds when calibrated
  char fwVersion[24] = {0};  ///< firmware that produced the calibration
};

/// Parameters for the steering auto-calibration run (sent by the web UI).
struct SteeringCalParams {
  float pwmPct = 25.0f;          ///< low commissioning PWM
  float currentThresholdA = 3.0f;///< current indicating a possible stop
  uint32_t noMotionMs = 300;     ///< position-unchanged confirmation time
  float motionEpsilon = 12.0f;   ///< ADC counts considered "no movement"
};

struct MotorTest {
  bool active = false;
  bool left = false, right = false;
  float pwmPct = 0.0f;      ///< limited commissioning power
  uint32_t endMs = 0;       ///< auto-stop deadline
};

class CalibrationService {
 public:
  void begin();  ///< load calibration records from NVS

  // -- steering auto-calibration wizard ------------------------------------
  bool startSteeringCal(const SteeringCalParams& params);
  void abortSteeringCal();
  /// One 200 Hz step; returns the PWM to apply to the steering actuator.
  /// Called by the steering controller while calibration owns the actuator.
  /// @param currentValid false when INA3221 is absent (stall-on-position).
  float steeringCalStep(float feedbackAdc, float currentA, bool currentValid);
  SteeringCalState steeringCalState() const { return steerState_; }
  const SteeringCalData& steeringCal() const { return steerData_; }
  bool steeringCalActive() const {
    return steerState_ != SteeringCalState::IDLE &&
           steerState_ != SteeringCalState::DONE &&
           steerState_ != SteeringCalState::ABORTED;
  }
  /// Manually set limits (manual override from the wizard).
  void setSteeringLimits(float leftAdc, float rightAdc, float centerAdc);

  /// Soft limits = hard stops pulled in by steering.soft_limit_margin.
  void softLimits(float& leftAdc, float& rightAdc) const;
  float centerAdc() const;  ///< includes steering.center_trim

  // -- motor test (speed calibration / commissioning) -----------------------
  bool startMotorTest(bool left, bool right, float pwmPct, uint32_t durationMs);
  void stopMotorTest();
  const MotorTest& motorTest() const { return motorTest_; }
  void tickMotorTest();  ///< auto-stop on deadline; call at >= 10 Hz

  // -- manual steering test (commissioning page) ----------------------------
  void setManualSteeringTarget(float pct, bool enabled);
  bool manualSteeringActive() const { return manualSteerActive_; }
  float manualSteeringTarget() const { return manualSteerTarget_; }

  /// USB/open-loop PWM, ±40%, works before travel calibration.
  void setOpenLoopSteer(float pwmPct, bool enabled);
  bool openLoopSteerActive() const { return openLoopActive_; }
  float openLoopSteerPwm() const { return openLoopPwm_; }

  // -- current sensor calibration -------------------------------------------
  /// Capture the present current readings as the zero offset.
  void captureCurrentOffset(float measuredOffsetA);

  // -- commissioning ---------------------------------------------------------
  bool commissioned() const;
  void setCommissioned(bool done);

  bool saveSteering();  ///< persist steering calibration to NVS
  void dumpToLog() const;
  void applyFactoryDefaults();  ///< RAM + NVS from vehicle_defaults.h

 private:
  SteeringCalState steerState_ = SteeringCalState::IDLE;
  SteeringCalParams params_;
  SteeringCalData steerData_;
  float calLeft_ = 0, calRight_ = 0;
  uint32_t confirmStartMs_ = 0, settleStartMs_ = 0, calStartMs_ = 0;
  float lastAdc_ = 0;
  uint32_t lastMotionMs_ = 0;

  MotorTest motorTest_;
  bool manualSteerActive_ = false;
  float manualSteerTarget_ = 50.0f;
  bool openLoopActive_ = false;
  float openLoopPwm_ = 0.0f;
  bool commissioned_ = false;
};

extern CalibrationService calibration;

}  // namespace vcm
