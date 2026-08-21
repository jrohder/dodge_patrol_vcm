#include "drivers/steering/firgelli_feedback.h"

#include "core/pins.h"
#include "services/logger.h"

namespace vcm {

void FirgelliFeedback::begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(pins::FIRGELLI_ADC, ADC_11db);
  raw_ = analogRead(pins::FIRGELLI_ADC);
  stage1_ = (float)raw_;
  filtered_ = (float)raw_;
  primed_ = true;
  health_ = SensorHealth::OK;
  LOGI("FIRGEL", "Actuator feedback ADC ready (raw=%u)", (unsigned)raw_);
}

void FirgelliFeedback::sample(float filterHz, int validMin, int validMax) {
  // ESP32-S3 SAR ADC is noisy; oversample and drop outliers before the IIR.
  constexpr int kN = 16;
  uint32_t acc = 0;
  uint16_t mn = 4095, mx = 0;
  for (int i = 0; i < kN; ++i) {
    const uint16_t v = analogRead(pins::FIRGELLI_ADC);
    acc += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  raw_ = (uint16_t)((acc - mn - mx) / (kN - 2));

  if ((int)raw_ < validMin || (int)raw_ > validMax) {
    if (invalidCount_ < 255) invalidCount_++;
    if (invalidCount_ > 10) health_ = SensorHealth::FAULT;
    return;
  }
  invalidCount_ = 0;
  health_ = SensorHealth::OK;

  // Two cascaded first-order LPFs at 200 Hz ≈ 2-pole Butterworth-ish.
  const float dt = 1.0f / 200.0f;
  const float alpha = (filterHz > 0)
                          ? dt * filterHz * 6.2831853f /
                                (1.0f + dt * filterHz * 6.2831853f)
                          : 1.0f;
  if (!primed_) {
    stage1_ = (float)raw_;
    filtered_ = (float)raw_;
    primed_ = true;
  }
  const float x = (float)raw_;
  stage1_ += alpha * (x - stage1_);
  filtered_ += alpha * (stage1_ - filtered_);
}

float FirgelliFeedback::toPct(float adc, float leftAdc, float rightAdc) {
  const float span = rightAdc - leftAdc;
  if (fabsf(span) < 1.0f) return 50.0f;
  return constrain((adc - leftAdc) / span * 100.0f, 0.0f, 100.0f);
}

}  // namespace vcm
