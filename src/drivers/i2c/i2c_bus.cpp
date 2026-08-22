#include "drivers/i2c/i2c_bus.h"

#include <Wire.h>
#include <string.h>

#include "core/pins.h"
#include "driver/gpio.h"
#include "services/logger.h"

namespace vcm {

I2cBus i2cBus;

int I2cBus::sdaPin() const { return pins::I2C_SDA; }
int I2cBus::sclPin() const { return pins::I2C_SCL; }

const char* I2cBus::nameForAddr(uint8_t addr) {
  if (addr >= 0x40 && addr <= 0x43) return "INA3221";
  if (addr == 0x68 || addr == 0x69) return "MPU6050";
  return "unknown";
}

void I2cBus::enablePullups() {
  // Keep the I²C peripheral attached. pinMode() after Wire.begin() can
  // detach SDA/SCL from the I2C controller and the devices never ACK.
  gpio_set_pull_mode(static_cast<gpio_num_t>(pins::I2C_SDA), GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(static_cast<gpio_num_t>(pins::I2C_SCL), GPIO_PULLUP_ONLY);
}

void I2cBus::attachWire() {
  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, kFreqHz);
  Wire.setTimeOut(kTimeoutMs);
  enablePullups();
}

void I2cBus::begin() {
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
  attachWire();
  ready_ = true;
  LOGI("I2C", "bus GPIO%d/GPIO%d @ %lu Hz (no boot scan)", pins::I2C_SDA,
       pins::I2C_SCL, (unsigned long)kFreqHz);
}

bool I2cBus::tryLock(uint32_t waitMs) {
  if (!mutex_) return false;
  return xSemaphoreTake(mutex_, pdMS_TO_TICKS(waitMs)) == pdTRUE;
}

void I2cBus::unlock() {
  if (mutex_) xSemaphoreGive(mutex_);
}

bool I2cBus::sdaHigh() const {
  return gpio_get_level(static_cast<gpio_num_t>(pins::I2C_SDA)) != 0;
}

bool I2cBus::sclHigh() const {
  return gpio_get_level(static_cast<gpio_num_t>(pins::I2C_SCL)) != 0;
}

const char* I2cBus::busStateName() const {
  const bool sda = sdaHigh();
  const bool scl = sclHigh();
  if (sda && scl) return "OK";
  if (!sda && !scl) return "BOTH_LOW";
  if (!sda) return "SDA_LOW";
  return "SCL_LOW";
}

bool I2cBus::recover(const char* reason) {
  recoverRequested_ = false;
  LOGW("I2C", "bus recovery (%s) SDA=%d SCL=%d", reason ? reason : "?",
       sdaHigh() ? 1 : 0, sclHigh() ? 1 : 0);

  Wire.end();

  const gpio_num_t sda = static_cast<gpio_num_t>(pins::I2C_SDA);
  const gpio_num_t scl = static_cast<gpio_num_t>(pins::I2C_SCL);

  gpio_reset_pin(sda);
  gpio_reset_pin(scl);
  gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
  gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);

  // Clock SCL up to 9 times until SDA is released (device finishes its byte).
  for (int i = 0; i < 9; ++i) {
    gpio_set_level(scl, 0);
    delayMicroseconds(5);
    gpio_set_level(scl, 1);
    delayMicroseconds(5);
    if (gpio_get_level(sda)) break;
  }

  // STOP: SDA rising while SCL is high.
  gpio_set_level(sda, 0);
  delayMicroseconds(5);
  gpio_set_level(scl, 1);
  delayMicroseconds(5);
  gpio_set_level(sda, 1);
  delayMicroseconds(5);

  attachWire();
  stats_.recoveries++;
  noteTxn("recover", sdaHigh() && sclHigh(), false);
  LOGI("I2C", "bus reinit SDA=%d SCL=%d recoveries=%lu", sdaHigh() ? 1 : 0,
       sclHigh() ? 1 : 0, (unsigned long)stats_.recoveries);
  return sdaHigh() && sclHigh();
}

void I2cBus::requestScan() { scanRequested_ = true; }
void I2cBus::requestRecover() { recoverRequested_ = true; }

I2cScanResult I2cBus::lastScan() const { return scan_; }

void I2cBus::runScanIfRequested() {
  if (!scanRequested_) return;
  scanRequested_ = false;
  scan_.inProgress = true;
  scan_.startedMs = millis();
  scan_.count = 0;
  scan_.valid = false;

  const uint32_t t0 = millis();
  if (!sdaHigh() || !sclHigh()) {
    recover("scan: bus stuck");
  }

  for (uint8_t a = 0x08; a < 0x78; ++a) {
    Wire.beginTransmission(a);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      if (scan_.count < 16) scan_.addrs[scan_.count] = a;
      scan_.count++;
      LOGI("I2C", "ack 0x%02X %s", a, nameForAddr(a));
    } else if (err == 5) {
      stats_.timeouts++;
    }
  }

  scan_.durationMs = millis() - t0;
  scan_.valid = true;
  scan_.inProgress = false;
  stats_.scans++;
  noteTxn("scan", true, false);

  if (scan_.count == 0) {
    LOGW("I2C", "scan empty in %lums — SDA=GPIO%d SCL=GPIO%d, 3.3V, GND, pull-ups",
         (unsigned long)scan_.durationMs, pins::I2C_SDA, pins::I2C_SCL);
  } else {
    LOGI("I2C", "scan %u device(s) in %lums", scan_.count,
         (unsigned long)scan_.durationMs);
  }
}

void I2cBus::noteTxn(const char* what, bool ok, bool timeout) {
  stats_.lastTxnMs = millis();
  stats_.lastTxnOk = ok;
  if (what) {
    strncpy(stats_.lastTxn, what, sizeof(stats_.lastTxn) - 1);
    stats_.lastTxn[sizeof(stats_.lastTxn) - 1] = 0;
  }
  if (!ok) stats_.errors++;
  if (timeout) stats_.timeouts++;
}

}  // namespace vcm
