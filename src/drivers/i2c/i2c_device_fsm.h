/**
 * @file i2c_device_fsm.h
 * @brief Optional-I²C device lifecycle. Header-only so native tests can
 *        exercise the state machine without Wire/Arduino.
 *
 * UNKNOWN → PROBING → ONLINE
 *                ↘ RETRY ↻
 * ONLINE → FAULT → RETRY → PROBING
 *
 * Missing devices stay NOT_PRESENT / RETRY and never look like a hard
 * fault. A device that *was* online and then failed consecutive
 * transactions becomes FAULT, then RETRY with exponential backoff.
 */
#pragma once

#include <cstdint>

#include "core/types.h"

namespace vcm {

enum class I2cDevState : uint8_t {
  UNKNOWN = 0,
  PROBING,
  ONLINE,
  FAULT,
  RETRY,
};

inline const char* i2cDevStateName(I2cDevState s) {
  switch (s) {
    case I2cDevState::UNKNOWN: return "UNKNOWN";
    case I2cDevState::PROBING: return "PROBING";
    case I2cDevState::ONLINE: return "ONLINE";
    case I2cDevState::FAULT: return "FAULT";
    case I2cDevState::RETRY: return "RETRY";
  }
  return "?";
}

class I2cDeviceFsm {
 public:
  static constexpr uint8_t kFailTrip = 10;

  I2cDevState state() const { return state_; }
  bool everOnline() const { return everOnline_; }
  uint8_t failCount() const { return failCount_; }
  uint8_t retryCount() const { return retryCount_; }
  uint32_t lastOkMs() const { return lastOkMs_; }
  uint32_t lastAttemptMs() const { return lastAttemptMs_; }
  uint32_t nextRetryMs() const { return nextRetryMs_; }

  SensorHealth health() const {
    switch (state_) {
      case I2cDevState::ONLINE:
        return SensorHealth::OK;
      case I2cDevState::FAULT:
        return SensorHealth::FAULT;
      case I2cDevState::RETRY:
        return everOnline_ ? SensorHealth::STALE : SensorHealth::NOT_PRESENT;
      case I2cDevState::PROBING:
      case I2cDevState::UNKNOWN:
      default:
        return SensorHealth::NOT_PRESENT;
    }
  }

  /// True when the caller should run a (short) probe of the known address.
  bool shouldProbe(uint32_t nowMs) const {
    switch (state_) {
      case I2cDevState::UNKNOWN:
        return true;
      case I2cDevState::ONLINE:
      case I2cDevState::PROBING:
        return false;
      case I2cDevState::FAULT:
      case I2cDevState::RETRY:
        return nowMs >= nextRetryMs_;
    }
    return false;
  }

  bool online() const { return state_ == I2cDevState::ONLINE; }

  void onProbeStart(uint32_t nowMs) {
    state_ = I2cDevState::PROBING;
    lastAttemptMs_ = nowMs;
  }

  void onProbeOk(uint32_t nowMs) {
    state_ = I2cDevState::ONLINE;
    failCount_ = 0;
    retryCount_ = 0;
    lastOkMs_ = nowMs;
    lastAttemptMs_ = nowMs;
    everOnline_ = true;
    nextRetryMs_ = 0;
  }

  void onProbeFail(uint32_t nowMs) {
    lastAttemptMs_ = nowMs;
    scheduleRetry(nowMs, /*faulted=*/false);
  }

  void onTxnOk(uint32_t nowMs) {
    state_ = I2cDevState::ONLINE;
    failCount_ = 0;
    lastOkMs_ = nowMs;
    everOnline_ = true;
  }

  void onTxnFail(uint32_t nowMs) {
    if (failCount_ < 255) failCount_++;
    if (failCount_ >= kFailTrip) {
      state_ = I2cDevState::FAULT;
      scheduleRetry(nowMs, /*faulted=*/true);
    }
  }

  static uint32_t backoffMs(uint8_t retryCount) {
    if (retryCount == 0) return 500;
    if (retryCount == 1) return 1000;
    if (retryCount == 2) return 2000;
    if (retryCount == 3) return 5000;
    return 10000;
  }

 private:
  void scheduleRetry(uint32_t nowMs, bool faulted) {
    nextRetryMs_ = nowMs + backoffMs(retryCount_);
    if (retryCount_ < 255) retryCount_++;
    state_ = faulted ? I2cDevState::FAULT : I2cDevState::RETRY;
  }

  I2cDevState state_ = I2cDevState::UNKNOWN;
  uint8_t failCount_ = 0;
  uint8_t retryCount_ = 0;
  uint32_t lastOkMs_ = 0;
  uint32_t lastAttemptMs_ = 0;
  uint32_t nextRetryMs_ = 0;
  bool everOnline_ = false;
};

}  // namespace vcm
