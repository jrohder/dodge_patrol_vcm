#include "services/event_recorder.h"

#include "services/logger.h"
#include "services/safety.h"

namespace vcm {

EventRecorder recorder;

void EventRecorder::record(const VehicleTelemetry& t) {
  // Auto-freeze on a NEW trip fault so pre-fault data is preserved
  const uint16_t mask = t.system.activeFaultMask;
  if (!frozen_ && mask != 0 && (mask & ~lastFaultMask_) != 0 &&
      safety.anyTripActive()) {
    for (int i = 0; i < FLT_COUNT; ++i) {
      if ((mask & ~lastFaultMask_) & (1u << i)) {
        freeze(SafetyManager::info((FaultId)i).code);
        break;
      }
    }
  }
  lastFaultMask_ = mask;
  if (frozen_) return;

  RecorderSample s;
  s.ms = millis();
  s.speed = t.drive.vehicleSpeed;
  s.throttle = t.drive.throttleInput;
  s.steerReq = t.steering.requestedPct;
  s.steerAct = t.steering.actualPct;
  s.steerPwm = t.steering.pwmPct;
  s.leftPwm = t.drive.leftPwmPct;
  s.rightPwm = t.drive.rightPwmPct;
  s.leftCurrent = t.power.leftCurrentA;
  s.rightCurrent = t.power.rightCurrentA;
  s.steerCurrent = t.power.steeringCurrentA;
  s.battV = t.power.busVoltage;
  s.faultMask = mask;
  s.state = (uint8_t)t.system.state;

  xSemaphoreTake(mutex_, portMAX_DELAY);
  ring_[head_] = s;
  head_ = (head_ + 1) % CAPACITY;
  if (count_ < CAPACITY) count_++;
  xSemaphoreGive(mutex_);
}

void EventRecorder::freeze(const char* reason) {
  frozen_ = true;
  strncpy(freezeReason_, reason, sizeof(freezeReason_) - 1);
  LOGW("RECORD", "Event recorder frozen (%s) - snapshot preserved", reason);
}

void EventRecorder::unfreeze() {
  frozen_ = false;
  freezeReason_[0] = 0;
  LOGI("RECORD", "Event recorder resumed");
}

String EventRecorder::dumpCsv() const {
  String out;
  out.reserve(count_ * 96 + 128);
  out += "ms,speed_mps,throttle,steer_req_pct,steer_act_pct,steer_pwm,"
         "left_pwm,right_pwm,left_a,right_a,steer_a,batt_v,fault_mask,state\n";
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const size_t start = (head_ + CAPACITY - count_) % CAPACITY;
  for (size_t i = 0; i < count_; ++i) {
    const RecorderSample& s = ring_[(start + i) % CAPACITY];
    char line[160];
    snprintf(line, sizeof(line),
             "%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%u,%u\n",
             (unsigned long)s.ms, s.speed, s.throttle, s.steerReq, s.steerAct,
             s.steerPwm, s.leftPwm, s.rightPwm, s.leftCurrent, s.rightCurrent,
             s.steerCurrent, s.battV, (unsigned)s.faultMask, (unsigned)s.state);
    out += line;
  }
  xSemaphoreGive(mutex_);
  return out;
}

}  // namespace vcm
