#include "drivers/motor/bts7960.h"

#include "core/pins.h"
#include "services/logger.h"

namespace vcm {

static constexpr uint32_t kMaxDuty = (1u << pins::PWM_RES_BITS) - 1;

void Bts7960::begin() {
  // Known-inactive state before attaching PWM (outputs default OFF)
  pinMode(rpwmPin_, OUTPUT);
  pinMode(lpwmPin_, OUTPUT);
  digitalWrite(rpwmPin_, LOW);
  digitalWrite(lpwmPin_, LOW);

  ledcSetup(chR_, pins::PWM_FREQ_HZ, pins::PWM_RES_BITS);
  ledcSetup(chL_, pins::PWM_FREQ_HZ, pins::PWM_RES_BITS);
  ledcAttachPin(rpwmPin_, chR_);
  ledcAttachPin(lpwmPin_, chL_);
  ledcWrite(chR_, 0);
  ledcWrite(chL_, 0);
  LOGD("MOTOR", "%s BTS7960 ready (R=%d L=%d)", name_, rpwmPin_, lpwmPin_);
}

void Bts7960::write(float rDuty, float lDuty) {
  ledcWrite(chR_, (uint32_t)(constrain(rDuty, 0.0f, 100.0f) * kMaxDuty / 100.0f));
  ledcWrite(chL_, (uint32_t)(constrain(lDuty, 0.0f, 100.0f) * kMaxDuty / 100.0f));
}

void Bts7960::drive(float pct) {
  pct = constrain(pct, -maxPct_, maxPct_);
  currentPct_ = pct;
  if (inverted_) pct = -pct;

  if (pct > 0.05f) {
    write(pct, 0.0f);
  } else if (pct < -0.05f) {
    write(0.0f, -pct);
  } else {
    // BTS7960: both inputs low = coast (high-Z); both PWM'd low-side = brake.
    // We implement brake as both outputs low duty 100 is unsafe; use both 0
    // (coast) or both enabled at 0 duty -> effectively coast. True dynamic
    // braking shorts the motor through the low-side switches by driving
    // both LPWM/RPWM low with EN high; on common BTS7960 boards tying both
    // PWM inputs low achieves the same via freewheel diodes.
    write(0.0f, 0.0f);
  }
}

void Bts7960::stop(StopMode) {
  currentPct_ = 0.0f;
  write(0.0f, 0.0f);
}

}  // namespace vcm
