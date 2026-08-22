/**
 * @file i2c_bus.h
 * @brief Shared I²C bus manager for the vehicle sensor bus.
 *
 * The control loops never wait on this bus. Wire is initialized once,
 * without a full address scan. Known devices (INA3221, MPU6050) are
 * probed from the sensor task. A full scan runs only on demand
 * (web button / USB `i2c` command).
 *
 * If SDA/SCL is stuck, recover() bit-bangs ~9 SCL clocks and a STOP
 * then re-inits the peripheral — typical vehicle EMI / long-wire
 * recovery, not something a bench Arduino sketch usually needs.
 */
#pragma once

#include <Arduino.h>

#include "drivers/i2c/i2c_device_fsm.h"

namespace vcm {

struct I2cScanResult {
  bool valid = false;
  bool inProgress = false;
  uint32_t startedMs = 0;
  uint32_t durationMs = 0;
  uint8_t count = 0;
  uint8_t addrs[16] = {};
};

struct I2cBusStats {
  uint32_t errors = 0;
  uint32_t timeouts = 0;
  uint32_t recoveries = 0;
  uint32_t scans = 0;
  uint32_t lastTxnMs = 0;
  bool lastTxnOk = true;
  char lastTxn[28] = {};
};

class I2cBus {
 public:
  static constexpr uint32_t kFreqHz = 100000;  ///< stay at 100 kHz until sensors are reliable
  static constexpr uint16_t kTimeoutMs = 20;

  /// Attach Wire to the vehicle I²C pins. Does **not** scan the bus.
  void begin();

  bool ready() const { return ready_; }

  /// Non-blocking-ish lock for the sensor task (fails fast so 100 Hz never stalls).
  bool tryLock(uint32_t waitMs = 2);
  void unlock();

  /// Disable the peripheral, clock SCL, generate STOP, re-init Wire.
  bool recover(const char* reason);

  void requestScan();
  void requestRecover();
  bool scanRequested() const { return scanRequested_; }
  bool recoverRequested() const { return recoverRequested_; }
  /// Run a pending scan. Call from the sensor task while holding the lock.
  /// Takes up to ~2 s on a dead bus — never call this from a control task.
  void runScanIfRequested();
  I2cScanResult lastScan() const;

  bool sdaHigh() const;
  bool sclHigh() const;
  const char* busStateName() const;  ///< OK / SDA_LOW / SCL_LOW / BOTH_LOW

  void noteTxn(const char* what, bool ok, bool timeout = false);
  I2cBusStats stats() const { return stats_; }

  int sdaPin() const;
  int sclPin() const;
  uint32_t freqHz() const { return kFreqHz; }

  I2cDeviceFsm& inaFsm() { return inaFsm_; }
  I2cDeviceFsm& imuFsm() { return imuFsm_; }
  const I2cDeviceFsm& inaFsm() const { return inaFsm_; }
  const I2cDeviceFsm& imuFsm() const { return imuFsm_; }

  static const char* nameForAddr(uint8_t addr);

 private:
  void attachWire();
  void enablePullups();

  SemaphoreHandle_t mutex_ = nullptr;
  bool ready_ = false;
  volatile bool scanRequested_ = false;
  volatile bool recoverRequested_ = false;
  I2cScanResult scan_{};
  I2cBusStats stats_{};
  I2cDeviceFsm inaFsm_;
  I2cDeviceFsm imuFsm_;
};

extern I2cBus i2cBus;

}  // namespace vcm
