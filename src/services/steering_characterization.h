/**
 * @file steering_characterization.h
 * @brief Smart actuator characterization wizard (position-based, no INA).
 *
 * Independent of position-limit calibration and of current-sensor
 * calibration. Requires existing left/right/center limits. Owns the
 * actuator the same way the travel-calibration wizard does: the 200 Hz
 * steering task calls step() and applies the returned PWM. Analysis
 * feeds recommended settings into the existing config registry — it
 * never writes PID gains unless the user presses Apply Recommended.
 */
#pragma once

#include <Arduino.h>

#include "control/steering_types.h"

namespace vcm {

enum class SteerCharPhase : uint8_t {
  IDLE = 0,
  CHECK,
  MOVE_CENTER,
  VERIFY_STILL,
  SWEEP_LEFT,
  RETURN_CENTER_1,
  SWEEP_RIGHT,
  RETURN_CENTER_2,
  SETTLING,
  BACKLASH,
  ANALYZE,
  DONE,
  FAILED,
};

struct SteerCharLive {
  SteerCharPhase phase = SteerCharPhase::IDLE;
  const char* stepName = "IDLE";
  float pwmCmd = 0;
  float pwmLevel = 0;
  int direction = 0;  ///< -1 left, +1 right
  bool openLoop = false;
  float closedLoopTarget = 50.0f;
  uint8_t progressPct = 0;
  char message[96] = {0};
};

class SteeringCharacterizationService {
 public:
  void begin();

  /// Requires explicit confirm=true from the UI (safety interlock).
  bool start(bool confirmed);
  void abort(const char* reason);
  bool active() const;
  SteerCharPhase phase() const { return phase_; }

  /**
   * @brief 200 Hz wizard step.
   * @return PWM to apply when openLoop is true; ignored otherwise
   *         (controller tracks closedLoopTarget with the existing PID).
   */
  float step(float actualPct, float filteredAdc, float rawAdc, float dt,
             bool feedbackOk, bool fault, uint32_t loopMisses);

  const SteerCharLive& live() const { return live_; }
  const SteeringCharacterization& result() const { return current_; }
  const SteeringCharacterization& baseline() const { return baseline_; }
  const SteerRecommendations& recommended() const { return recommended_; }

  bool saveAsBaseline();
  bool loadHistorySlot(uint8_t index, SteeringCharacterization& out) const;
  uint8_t historyCount() const { return histCount_; }

  void persistCurrent();
  /// Write current + baseline only (not the 8-slot history). Used after compact.
  void persistEssential();

  static const char* phaseName(SteerCharPhase p);

 private:
  void fail(const char* reason);
  void finishOk();
  void setPhase(SteerCharPhase p, const char* msg);
  float driveToCenter(float actualPct);
  float sweepStep(float actualPct, float dt, int dirSign);
  void recordSweepPoint(int dirSign, float pwm, float vel);
  void analyze();
  void saveHistory();
  void loadFromNvs();
  void saveBlob(const char* key, const SteeringCharacterization& c);
  bool loadBlob(const char* key, SteeringCharacterization& c);

  SteerCharPhase phase_ = SteerCharPhase::IDLE;
  SteerCharLive live_;
  SteeringCharacterization current_;
  SteeringCharacterization baseline_;
  SteeringCharacterization history_[kSteerCharHistory];
  uint8_t histCount_ = 0;
  SteerRecommendations recommended_;

  uint32_t phaseStartMs_ = 0;
  uint32_t startMs_ = 0;
  uint32_t lastMisses_ = 0;
  float pwmLevel_ = 0;
  float dwellPos0_ = 0;
  float dwellPos1_ = 0;
  uint8_t dwellPass_ = 0;  ///< 0=apply, 1=repeat
  bool lookingForStart_ = true;
  bool foundStart_ = false;
  float minStartThisDir_ = 0;
  float holdThisDir_ = 0;
  bool holdSearch_ = false;
  float stillPos0_ = 50.0f;
  float sweepVel_ = 0;
  float centerTarget_ = 50.0f;

  // backlash
  float backlashStartPos_ = 0;
  uint32_t backlashMoveUs_ = 0;
  bool backlashMoved_ = false;

  // settling
  float settlePeak_ = 0;
  float settleTarget_ = 70.0f;
  uint8_t settleLeg_ = 0;
  uint32_t settleStartUs_ = 0;
  int settleReversals_ = 0;
  int8_t settleLastDir_ = 0;
};

extern SteeringCharacterizationService steerChar;

}  // namespace vcm
