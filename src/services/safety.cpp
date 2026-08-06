#include "services/safety.h"

#include "services/logger.h"

namespace vcm {

SafetyManager safety;

static const FaultInfo kFaults[FLT_COUNT] = {
    {"COM-001", "Nano communication timeout", FaultSeverity::TRIP},
    {"COM-002", "Excessive Nano CRC errors", FaultSeverity::WARNING},
    {"COM-003", "Nano protocol version mismatch", FaultSeverity::TRIP},
    {"RC-001", "RC signal lost", FaultSeverity::TRIP},
    {"WEB-001", "Web control heartbeat lost", FaultSeverity::TRIP},
    {"STR-001", "Steering feedback lost", FaultSeverity::TRIP},
    {"STR-002", "Steering overcurrent", FaultSeverity::TRIP},
    {"STR-003", "Steering not calibrated", FaultSeverity::WARNING},
    {"DRV-001", "Left motor overcurrent", FaultSeverity::TRIP},
    {"DRV-002", "Right motor overcurrent", FaultSeverity::TRIP},
    {"PWR-001", "Battery under-voltage cutoff", FaultSeverity::TRIP},
    {"PWR-002", "Battery voltage low", FaultSeverity::WARNING},
    {"SNS-001", "IMU failure", FaultSeverity::WARNING},
    {"SNS-002", "Current monitor failure", FaultSeverity::WARNING},
    {"SYS-001", "Control task overrun", FaultSeverity::WARNING},
    {"SYS-002", "Low free heap", FaultSeverity::WARNING},
};

const FaultInfo& SafetyManager::info(FaultId id) { return kFaults[id]; }

void SafetyManager::begin() {
  mutex_ = xSemaphoreCreateMutex();
  for (int i = 0; i < FLT_COUNT; ++i) records_[i].id = (FaultId)i;
  state_ = VehicleState::INITIALIZING;
  LOGI("SAFETY", "State: BOOT -> INITIALIZING");
}

bool SafetyManager::transitionValid(VehicleState from, VehicleState to) const {
  if (from == to) return false;
  // ESTOP can be entered from anywhere and only leaves via clearEstop
  if (to == VehicleState::ESTOP) return true;
  if (from == VehicleState::ESTOP) return to == VehicleState::READY ||
                                          to == VehicleState::NOT_CALIBRATED ||
                                          to == VehicleState::FAULT;
  // FAULT can be entered from anywhere except boot-time states
  if (to == VehicleState::FAULT) return from != VehicleState::BOOT;
  switch (from) {
    case VehicleState::BOOT:
      return to == VehicleState::INITIALIZING;
    case VehicleState::INITIALIZING:
      return to == VehicleState::NOT_CALIBRATED || to == VehicleState::READY;
    case VehicleState::NOT_CALIBRATED:
      return to == VehicleState::READY || to == VehicleState::CALIBRATION ||
             to == VehicleState::DIAGNOSTIC || to == VehicleState::OTA_UPDATE;
    case VehicleState::READY:
      return to == VehicleState::DRIVING || to == VehicleState::CALIBRATION ||
             to == VehicleState::DIAGNOSTIC || to == VehicleState::OTA_UPDATE ||
             to == VehicleState::NOT_CALIBRATED;
    case VehicleState::DRIVING:
      return to == VehicleState::READY;
    case VehicleState::CALIBRATION:
    case VehicleState::DIAGNOSTIC:
      return to == VehicleState::READY || to == VehicleState::NOT_CALIBRATED;
    case VehicleState::OTA_UPDATE:
      return to == VehicleState::READY || to == VehicleState::NOT_CALIBRATED;
    case VehicleState::FAULT:
      return to == VehicleState::READY || to == VehicleState::NOT_CALIBRATED;
    default:
      return false;
  }
}

bool SafetyManager::requestState(VehicleState next, const char* reason) {
  const VehicleState cur = state_;
  if (!transitionValid(cur, next)) {
    LOGW("SAFETY", "Rejected transition %s -> %s (%s)", vehicleStateName(cur),
         vehicleStateName(next), reason);
    return false;
  }
  state_ = next;
  LOGI("SAFETY", "State: %s -> %s (%s)", vehicleStateName(cur),
       vehicleStateName(next), reason);
  return true;
}

void SafetyManager::tick(bool commandActive, bool commissioned) {
  const VehicleState cur = state_;
  if (anyTripActive() && cur != VehicleState::FAULT &&
      cur != VehicleState::ESTOP && cur != VehicleState::OTA_UPDATE) {
    requestState(VehicleState::FAULT, "trip fault active");
    return;
  }
  switch (cur) {
    case VehicleState::READY:
      if (commandActive) requestState(VehicleState::DRIVING, "command active");
      break;
    case VehicleState::DRIVING:
      if (!commandActive) requestState(VehicleState::READY, "command idle");
      break;
    case VehicleState::FAULT:
      if (!anyTripActive())
        requestState(commissioned ? VehicleState::READY
                                  : VehicleState::NOT_CALIBRATED,
                     "faults cleared");
      break;
    case VehicleState::NOT_CALIBRATED:
      if (commissioned) requestState(VehicleState::READY, "commissioned");
      break;
    default:
      break;
  }
}

void SafetyManager::raiseFault(FaultId id) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  FaultRecord& r = records_[id];
  const bool wasActive = r.active;
  r.active = true;
  r.lastMs = millis();
  if (!wasActive) {
    if (r.count == 0) r.firstMs = r.lastMs;
    if (r.count < 0xFFFF) r.count++;
  }
  xSemaphoreGive(mutex_);
  if (!wasActive) {
    const FaultInfo& fi = kFaults[id];
    if (fi.severity == FaultSeverity::TRIP)
      LOGE("FAULT", "%s %s", fi.code, fi.desc);
    else
      LOGW("FAULT", "%s %s", fi.code, fi.desc);
  }
}

void SafetyManager::clearFault(FaultId id) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const bool wasActive = records_[id].active;
  records_[id].active = false;
  xSemaphoreGive(mutex_);
  if (wasActive) LOGI("FAULT", "%s cleared", kFaults[id].code);
}

uint16_t SafetyManager::activeFaultMask() const {
  uint16_t mask = 0;
  for (int i = 0; i < FLT_COUNT; ++i)
    if (records_[i].active) mask |= (1u << i);
  return mask;
}

bool SafetyManager::anyTripActive() const {
  for (int i = 0; i < FLT_COUNT; ++i)
    if (records_[i].active && kFaults[i].severity == FaultSeverity::TRIP)
      return true;
  return false;
}

void SafetyManager::clearHistory() {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  for (int i = 0; i < FLT_COUNT; ++i) {
    if (!records_[i].active) {
      records_[i].count = 0;
      records_[i].firstMs = records_[i].lastMs = 0;
    }
  }
  xSemaphoreGive(mutex_);
  LOGI("FAULT", "Fault history cleared (active conditions retained)");
}

void SafetyManager::estop(const char* reason) {
  const VehicleState cur = state_;
  if (cur == VehicleState::ESTOP) return;
  state_ = VehicleState::ESTOP;
  LOGE("SAFETY", "EMERGENCY STOP (%s), was %s", reason, vehicleStateName(cur));
}

void SafetyManager::clearEstop() {
  if (state_ != VehicleState::ESTOP) return;
  requestState(anyTripActive() ? VehicleState::FAULT : VehicleState::READY,
               "estop cleared");
}

}  // namespace vcm
