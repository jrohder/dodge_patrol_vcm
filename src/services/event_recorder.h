/**
 * @file event_recorder.h
 * @brief Rolling pre-fault flight recorder.
 *
 * Keeps ~30 s of high-value telemetry samples in a RAM ring buffer.
 * When a TRIP fault occurs the buffer is frozen (snapshot) so the events
 * leading up to the fault can be inspected/downloaded from the web UI.
 */
#pragma once

#include <Arduino.h>

#include "core/telemetry_data.h"

namespace vcm {

struct RecorderSample {
  uint32_t ms;
  float speed, throttle;
  float steerReq, steerAct, steerPwm;
  float leftPwm, rightPwm;
  float leftCurrent, rightCurrent, steerCurrent, battV;
  uint16_t faultMask;
  uint8_t state;
};

class EventRecorder {
 public:
  static constexpr size_t CAPACITY = 300;  ///< 30 s at 10 Hz

  /// Record one sample (call at 10 Hz).
  void record(const VehicleTelemetry& t);

  /// Freeze the buffer (called automatically when a trip fault appears).
  void freeze(const char* reason);
  void unfreeze();
  bool frozen() const { return frozen_; }
  const char* freezeReason() const { return freezeReason_; }

  /// Dump the buffer as CSV (oldest first) for download.
  String dumpCsv() const;

 private:
  RecorderSample ring_[CAPACITY];
  size_t head_ = 0, count_ = 0;
  bool frozen_ = false;
  char freezeReason_[32] = {0};
  uint16_t lastFaultMask_ = 0;
  mutable SemaphoreHandle_t mutex_ = nullptr;

 public:
  void begin() { mutex_ = xSemaphoreCreateMutex(); }
};

extern EventRecorder recorder;

}  // namespace vcm
