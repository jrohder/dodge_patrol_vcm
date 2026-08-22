/**
 * @file boot_report.h
 * @brief Reset/brownout/reboot diagnostics and staged boot timings.
 *
 * Captured at the start of setup() and after each major boot stage so a
 * several-minute "nothing works" symptom can be distinguished from a
 * reboot loop, brownout, or a single slow subsystem.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

struct BootMemSnap {
  uint32_t heap = 0;
  uint32_t minHeap = 0;
  uint32_t largest = 0;
  uint32_t psram = 0;
};

struct BootReport {
  uint32_t t0Ms = 0;  ///< millis() at bootstrap start
  uint32_t tGpioMs = 0;
  uint32_t tNanoMs = 0;
  uint32_t tControlMs = 0;
  uint32_t tWifiMs = 0;
  uint32_t tWebMs = 0;
  uint32_t tI2cMs = 0;
  uint32_t tReadyMs = 0;

  uint32_t resetReason = 0;      ///< esp_reset_reason()
  uint32_t resetReasonCpu0 = 0;  ///< rtc_get_reset_reason(0)
  uint32_t resetReasonCpu1 = 0;
  uint32_t bootCount = 0;  ///< RTC counter across unexpected resets
  bool brownout = false;
  bool panic = false;
  bool watchdog = false;
  char resetName[24] = {};

  BootMemSnap memEarly{};
  BootMemSnap memAfterRecorders{};
  BootMemSnap memBeforeWifi{};
  BootMemSnap memAfterWifi{};
  BootMemSnap memReady{};
};

class BootReporter {
 public:
  void captureReset();
  void mark(const char* stage, uint32_t& destMs);
  void snap(BootMemSnap& out) const;
  void logMem(const char* stage, const BootMemSnap& snap) const;

  const BootReport& report() const { return r_; }
  BootReport& report() { return r_; }

  uint32_t elapsedMs() const { return millis() - r_.t0Ms; }

 private:
  BootReport r_{};
};

extern BootReporter bootReport;

const char* espResetReasonName(uint32_t reason);

}  // namespace vcm
