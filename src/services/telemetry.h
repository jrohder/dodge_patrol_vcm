/**
 * @file telemetry.h
 * @brief TelemetryHub - thread-safe owner of the canonical VehicleTelemetry.
 *
 * All modules update the shared telemetry through update(); readers take
 * consistent snapshots. A short mutex hold keeps 100-200 Hz writers cheap.
 */
#pragma once

#include <Arduino.h>

#include "core/telemetry_data.h"

namespace vcm {

class TelemetryHub {
 public:
  void begin() { mutex_ = xSemaphoreCreateMutex(); }

  /// Mutate the shared telemetry under the lock.
  template <typename Fn>
  void update(Fn&& fn) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    fn(data_);
    xSemaphoreGive(mutex_);
  }

  /// Consistent copy for readers (WebSocket, logger, recorder, ...).
  VehicleTelemetry snapshot() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    VehicleTelemetry copy = data_;
    xSemaphoreGive(mutex_);
    return copy;
  }

 private:
  VehicleTelemetry data_;
  mutable SemaphoreHandle_t mutex_ = nullptr;
};

extern TelemetryHub telemetry;

}  // namespace vcm
