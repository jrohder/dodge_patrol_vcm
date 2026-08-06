/**
 * @file safety.h
 * @brief Safety state machine and fault management.
 *
 * Explicit operating states (BOOT..ESTOP); every transition is logged and
 * visible on the diagnostics page. Faults carry codes (e.g. STR-001),
 * severities and history. TRIP faults force the FAULT state, which commands
 * propulsion safe. Outputs default OFF in every non-driving state.
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

struct FaultInfo {
  const char* code;  ///< e.g. "STR-001"
  const char* desc;
  FaultSeverity severity;
};

struct FaultRecord {
  FaultId id;
  uint32_t firstMs = 0;
  uint32_t lastMs = 0;
  uint16_t count = 0;
  bool active = false;
};

class SafetyManager {
 public:
  void begin();

  // -- state machine --------------------------------------------------------
  VehicleState state() const { return state_; }
  /// Request a state transition. Invalid transitions are rejected + logged.
  bool requestState(VehicleState next, const char* reason);
  /// Called at 100 Hz from the dynamics task to auto-manage READY/DRIVING
  /// and FAULT recovery.
  void tick(bool commandActive, bool commissioned);

  /// True when the state machine permits driving the propulsion motors.
  bool motionAllowed() const {
    return state_ == VehicleState::READY || state_ == VehicleState::DRIVING;
  }
  /// True when actuators may be moved by calibration/diagnostic pages.
  bool testMotionAllowed() const {
    return state_ == VehicleState::CALIBRATION ||
           state_ == VehicleState::DIAGNOSTIC;
  }

  // -- faults ----------------------------------------------------------------
  void raiseFault(FaultId id);
  void clearFault(FaultId id);
  bool faultActive(FaultId id) const { return records_[id].active; }
  uint16_t activeFaultMask() const;
  bool anyTripActive() const;
  void clearHistory();  ///< clears history but never active conditions

  const FaultRecord* records() const { return records_; }
  static const FaultInfo& info(FaultId id);

  // -- estop -----------------------------------------------------------------
  void estop(const char* reason);
  void clearEstop();

 private:
  bool transitionValid(VehicleState from, VehicleState to) const;

  volatile VehicleState state_ = VehicleState::BOOT;
  FaultRecord records_[FLT_COUNT];
  SemaphoreHandle_t mutex_ = nullptr;
};

extern SafetyManager safety;

}  // namespace vcm
