#include "drivers/steering/p3022.h"

#include <SPI.h>

#include "core/pins.h"
#include "services/logger.h"

namespace vcm {

static SPIClass spi(HSPI);
static const SPISettings kSettings(1000000, MSBFIRST, SPI_MODE0);

void P3022::begin() {
  pinMode(pins::P3022_CS, OUTPUT);
  digitalWrite(pins::P3022_CS, HIGH);
  spi.begin(pins::P3022_CLK, pins::P3022_MISO, pins::P3022_MOSI, pins::P3022_CS);
  uint16_t c;
  health_ = read(c) ? SensorHealth::OK : SensorHealth::NOT_PRESENT;
  LOGI("P3022", "Steering wheel encoder %s (raw=%u)",
       sensorHealthName(health_), (unsigned)lastCounts_);
}

bool P3022::read(uint16_t& counts) {
  spi.beginTransaction(kSettings);
  digitalWrite(pins::P3022_CS, LOW);
  delayMicroseconds(2);
  const uint16_t raw = spi.transfer16(0x0000);
  digitalWrite(pins::P3022_CS, HIGH);
  spi.endTransaction();

  // 12-bit angle in the upper bits of the 16-bit frame
  const uint16_t angle = (raw >> 4) & 0x0FFF;

  // All-zero or all-one frames indicate a missing/failed sensor.
  // Stay NOT_PRESENT if the encoder was never seen (not wired yet).
  // Only escalate to FAULT after a previously-good sensor drops out.
  if (raw == 0x0000 || raw == 0xFFFF) {
    if (badReads_ < 255) badReads_++;
    if (badReads_ > 10) {
      health_ = (health_ == SensorHealth::OK) ? SensorHealth::FAULT
                                              : SensorHealth::NOT_PRESENT;
    }
    return false;
  }
  badReads_ = 0;
  health_ = SensorHealth::OK;
  lastCounts_ = angle;
  counts = angle;
  return true;
}

float P3022::normalize(uint16_t counts, int minC, int centerC, int maxC,
                       float deadbandPct, bool invert) {
  float v;
  if ((int)counts >= centerC) {
    const float range = (float)(maxC - centerC);
    v = range > 1.0f ? ((float)counts - centerC) / range : 0.0f;
  } else {
    const float range = (float)(centerC - minC);
    v = range > 1.0f ? ((float)counts - centerC) / range : 0.0f;
  }
  v = constrain(v, -1.0f, 1.0f);
  const float db = deadbandPct / 100.0f;
  if (fabsf(v) < db) return 0.0f;
  // rescale so output is continuous past the deadband
  v = (v > 0 ? (v - db) : (v + db)) / (1.0f - db);
  return invert ? -v : v;
}

}  // namespace vcm
