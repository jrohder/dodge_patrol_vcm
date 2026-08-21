/**
 * @file steering_recorder.h
 * @brief 200 Hz rolling steering diagnostic recorder (60 s).
 *
 * Separate from the 10 Hz pre-fault EventRecorder. The 200 Hz steering
 * task only pushes a packed sample (no locks, no String, no WS). Web
 * export and WebSocket batching run on the telemetry task.
 */
#pragma once

#include <Arduino.h>

#include "control/steering_types.h"

namespace vcm {

class SteeringRecorder {
 public:
  static constexpr size_t kTargetCapacity = 12000;  ///< 60 s × 200 Hz
  static constexpr size_t kEventCapacity = 128;
  static constexpr size_t kMinCapacity = 2000;

  void begin();  ///< allocate ring (PSRAM preferred, SRAM fallback)

  /// Push one 200 Hz sample. Safe from the steering task (single writer).
  void push(const SteeringDiagnosticSample& s);

  void pushEvent(SteerDiagEvent type, int16_t value_x10 = 0);

  /// Stop accepting samples (used only while generating a consistent dump).
  /// The control loop still runs; samples during a dump are dropped rather
  /// than blocking the 200 Hz task.
  void requestClear();
  bool recording() const { return recording_; }
  void setRecording(bool on) { recording_ = on; }

  size_t capacity() const { return capacity_; }
  size_t count() const { return count_; }
  uint32_t seq() const { return seq_; }
  bool usingPsram() const { return usingPsram_; }

  /// Copy packed samples with seq > afterSeq into out (oldest first).
  /// Returns number copied. Leaves a 32-sample gap so the writer can wrap.
  size_t copySince(uint32_t afterSeq, SteeringDiagPacked* out, size_t maxOut,
                   uint32_t& lastSeq) const;

  /// Snapshot the whole ring (oldest first). maxOut samples.
  size_t copyAll(SteeringDiagPacked* out, size_t maxOut) const;

  size_t copyEvents(SteerEventMarker* out, size_t maxOut) const;

  /// Hunting over the most recent ~2 s of the ring (called at 10–20 Hz).
  void computeHunting(int& pwmReversals, int& errCrossings, bool& hunting,
                      uint8_t& severity) const;

  /// Window statistics over the whole ring (telemetry-task / HTTP).
  struct Stats {
    float rmsError = 0, maxError = 0, avgError = 0;
    float peakPwm = 0, avgPwm = 0;
    float maxVelocity = 0;
    int dirChanges = 0, pwmReversals = 0;
    uint32_t samples = 0;
  };
  Stats stats() const;

  /// Packed sample by age (0 = oldest). Returns false if out of range.
  bool peekPacked(size_t oldestIndex, SteeringDiagPacked& out) const;

 private:
  SteeringDiagPacked* buf_ = nullptr;
  size_t capacity_ = 0;
  size_t head_ = 0;   ///< next write index
  size_t count_ = 0;
  volatile uint32_t seq_ = 0;
  bool usingPsram_ = false;
  volatile bool recording_ = true;

  SteerEventMarker events_[kEventCapacity];
  size_t evHead_ = 0, evCount_ = 0;
};

extern SteeringRecorder steerDiag;

}  // namespace vcm
