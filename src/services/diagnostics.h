/**
 * @file diagnostics.h
 * @brief Performance monitoring: task execution times, deadline misses,
 *        CPU load, heap. Exposed live on the diagnostics page.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

/// Tracks execution-time statistics for one periodic task.
class TaskMonitor {
 public:
  explicit TaskMonitor(uint32_t periodUs) : periodUs_(periodUs) {}

  void beginCycle() { startUs_ = micros(); }
  void endCycle() {
    const uint32_t elapsed = micros() - startUs_;
    sumUs_ += elapsed;
    samples_++;
    if (elapsed > maxUs_) maxUs_ = elapsed;
    if (elapsed > periodUs_) misses_++;
    if (samples_ >= 100) {  // publish a rolling average every 100 cycles
      avgUs_ = (uint32_t)(sumUs_ / samples_);
      sumUs_ = 0;
      samples_ = 0;
    }
  }

  uint32_t avgUs() const { return avgUs_; }
  uint32_t maxUs() const { return maxUs_; }
  uint32_t misses() const { return misses_; }
  void resetMax() { maxUs_ = 0; }

 private:
  const uint32_t periodUs_;
  uint32_t startUs_ = 0;
  uint64_t sumUs_ = 0;
  uint32_t samples_ = 0;
  uint32_t avgUs_ = 0, maxUs_ = 0, misses_ = 0;
};

class DiagnosticsService {
 public:
  /// Update system stats (heap, uptime, CPU estimate). Call at ~1 Hz.
  void tick();

  uint32_t freeHeap() const { return freeHeap_; }
  uint32_t minFreeHeap() const { return minFreeHeap_; }
  float cpuLoadPct() const { return cpuLoad_; }
  uint32_t uptimeS() const { return millis() / 1000; }

 private:
  uint32_t freeHeap_ = 0, minFreeHeap_ = 0;
  float cpuLoad_ = 0.0f;
};

extern DiagnosticsService diagnostics;

}  // namespace vcm
