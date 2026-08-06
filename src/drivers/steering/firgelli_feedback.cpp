#include "drivers/steering/firgelli_feedback.h"

#include "core/pins.h"
#include "services/logger.h"

namespace vcm {

void FirgelliFeedback::begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(pins::FIRGELLI_ADC, ADC_11db);
  raw_ = analogRead(pins::FIRGELLI_ADC);
  filtered_ = (float)raw_;
  primed_ = true;
  health_ = SensorHealth::OK;
  LOGI("FIRGEL", "Actuator feedback ADC ready (raw=%u)", (unsigned)raw_);
}

void FirgelliFeedback::sample(float filterHz, int validMin, int validMax) {
  raw_ = analogRead(pins::FIRGELLI_ADC);

  if ((int)raw_ < validMin || (int)raw_ > validMax) {
    if (invalidCount_ < 255) invalidCount_++;
    // ~50 ms of continuous invalid readings at 200 Hz = broken wiring
    if (invalidCount_ > 10) health_ = SensorHealth::FAULT;
    return;
  }
  invalidCount_ = 0;
  health_ = SensorHealth::OK;

  // First-order low-pass at the 200 Hz sample rate
  const float dt = 1.0f / 200.0f;
  const float alpha = (filterHz > 0)
                          ? dt * filterHz * 6.2831853f /
                                (1.0f + dt * filterHz * 6.2831853f)
                          : 1.0f;
  if (!primed_) {
    filtered_ = (float)raw_;
    primed_ = true;
  }
  filtered_ += alpha * ((float)raw_ - filtered_);
}

float FirgelliFeedback::toPct(float adc, float leftAdc, float rightAdc) {
  const float span = rightAdc - leftAdc;
  if (fabsf(span) < 1.0f) return 50.0f;
  return constrain((adc - leftAdc) / span * 100.0f, 0.0f, 100.0f);
}

}  // namespace vcm
