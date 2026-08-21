#include "services/steering_recorder.h"

#include <esp_heap_caps.h>
#include <algorithm>
#include <cmath>
#include <cstring>

#include "control/steering_compensation.h"
#include "services/logger.h"

namespace vcm {

SteeringRecorder steerDiag;

void SteeringRecorder::begin() {
  const size_t tryCaps[] = {kTargetCapacity, 9000, 6000, 4000, kMinCapacity};
  for (size_t cap : tryCaps) {
    const size_t bytes = cap * sizeof(SteeringDiagPacked);
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) {
      buf_ = static_cast<SteeringDiagPacked*>(p);
      capacity_ = cap;
      usingPsram_ = true;
      break;
    }
    p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p) {
      buf_ = static_cast<SteeringDiagPacked*>(p);
      capacity_ = cap;
      usingPsram_ = false;
      break;
    }
  }
  if (!buf_) {
    LOGE("SDIAG", "Failed to allocate steering diagnostic buffer");
    capacity_ = 0;
    return;
  }
  memset(buf_, 0, capacity_ * sizeof(SteeringDiagPacked));
  LOGI("SDIAG", "Recorder %u samples (%u kB, %s)", (unsigned)capacity_,
       (unsigned)(capacity_ * sizeof(SteeringDiagPacked) / 1024),
       usingPsram_ ? "PSRAM" : "SRAM");
}

void SteeringRecorder::push(const SteeringDiagnosticSample& s) {
  if (!buf_ || !recording_ || capacity_ == 0) return;
  buf_[head_] = packSteerSample(s);
  head_ = (head_ + 1) % capacity_;
  if (count_ < capacity_) count_++;
  seq_++;
}

void SteeringRecorder::pushEvent(SteerDiagEvent type, int16_t value_x10) {
  SteerEventMarker m;
  m.t_us = micros();
  m.type = type;
  m.value_x10 = value_x10;
  events_[evHead_] = m;
  evHead_ = (evHead_ + 1) % kEventCapacity;
  if (evCount_ < kEventCapacity) evCount_++;
}

void SteeringRecorder::requestClear() {
  count_ = 0;
  head_ = 0;
  seq_ = 0;
  evCount_ = 0;
  evHead_ = 0;
}

size_t SteeringRecorder::copySince(uint32_t afterSeq, SteeringDiagPacked* out,
                                   size_t maxOut, uint32_t& lastSeq) const {
  lastSeq = seq_;
  if (!buf_ || !out || maxOut == 0 || count_ == 0) return 0;
  const uint32_t live = seq_;
  if (afterSeq >= live) return 0;

  uint32_t oldestSeq = live - (uint32_t)count_ + 1;
  // Writer overwrites the oldest slot when the ring is full — skip a few.
  if (count_ == capacity_) oldestSeq += 32;
  uint32_t first = afterSeq + 1;
  if (first < oldestSeq) first = oldestSeq;
  if (first > live) return 0;

  uint32_t want = live - first + 1;
  if (want > maxOut) {
    first = live - (uint32_t)maxOut + 1;
    want = (uint32_t)maxOut;
  }
  size_t idx = (head_ + capacity_ + (size_t)first - (size_t)live - 1) % capacity_;
  for (uint32_t i = 0; i < want; ++i) {
    out[i] = buf_[idx];
    idx = (idx + 1) % capacity_;
  }
  lastSeq = first + want - 1;
  return (size_t)want;
}

size_t SteeringRecorder::copyAll(SteeringDiagPacked* out, size_t maxOut) const {
  if (!buf_ || !out || count_ == 0) return 0;
  const size_t n = count_ < maxOut ? count_ : maxOut;
  const size_t gap = 16;
  const size_t take = n > gap ? n - gap : n;
  size_t idx = (head_ + capacity_ - count_) % capacity_;
  for (size_t i = 0; i < take; ++i) {
    out[i] = buf_[idx];
    idx = (idx + 1) % capacity_;
  }
  return take;
}

size_t SteeringRecorder::copyEvents(SteerEventMarker* out, size_t maxOut) const {
  if (!out || evCount_ == 0) return 0;
  const size_t n = evCount_ < maxOut ? evCount_ : maxOut;
  size_t idx = (evHead_ + kEventCapacity - evCount_) % kEventCapacity;
  for (size_t i = 0; i < n; ++i) {
    out[i] = events_[idx];
    idx = (idx + 1) % kEventCapacity;
  }
  return n;
}

void SteeringRecorder::computeHunting(int& pwmReversals, int& errCrossings,
                                      bool& hunting, uint8_t& severity) const {
  pwmReversals = 0;
  errCrossings = 0;
  hunting = false;
  severity = 0;
  if (!buf_ || count_ < 40) return;
  // ~2 s at 200 Hz = 400 samples
  const size_t n = count_ < 400 ? count_ : 400;
  size_t idx = (head_ + capacity_ - n) % capacity_;
  int lastPwm = 0, lastErr = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto& s = buf_[idx];
    const int ps = (s.pwm_x100 > 80) ? 1 : (s.pwm_x100 < -80) ? -1 : 0;
    const int es = (s.err_x100 > 20) ? 1 : (s.err_x100 < -20) ? -1 : 0;
    if (ps && lastPwm && ps != lastPwm) pwmReversals++;
    if (es && lastErr && es != lastErr) errCrossings++;
    if (ps) lastPwm = ps;
    if (es) lastErr = es;
    idx = (idx + 1) % capacity_;
  }
  // More than ~3 reversals/s over 2 s is hunting.
  if (pwmReversals >= 8 || errCrossings >= 8) {
    hunting = true;
    severity = (pwmReversals >= 20 || errCrossings >= 20) ? 3
               : (pwmReversals >= 12 || errCrossings >= 12) ? 2
                                                             : 1;
  }
}

bool SteeringRecorder::peekPacked(size_t oldestIndex,
                                  SteeringDiagPacked& out) const {
  if (!buf_ || oldestIndex >= count_) return false;
  const size_t idx = (head_ + capacity_ - count_ + oldestIndex) % capacity_;
  out = buf_[idx];
  return true;
}

SteeringRecorder::Stats SteeringRecorder::stats() const {
  Stats st;
  if (!buf_ || count_ == 0) return st;
  const size_t n = count_;
  size_t idx = (head_ + capacity_ - n) % capacity_;
  double errSq = 0, errSum = 0, pwmSum = 0;
  int lastDir = 0, lastPwm = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto& s = buf_[idx];
    const float err = steerFromQ100(s.err_x100);
    const float pwm = steerFromQ100(s.pwm_x100);
    const float vel = steerFromQ100(s.vel_x100);
    errSq += (double)err * err;
    errSum += err;
    pwmSum += std::fabs(pwm);
    st.maxError = std::max(st.maxError, std::fabs(err));
    st.peakPwm = std::max(st.peakPwm, std::fabs(pwm));
    st.maxVelocity = std::max(st.maxVelocity, std::fabs(vel));
    const int d = (s.flags & 0x03) - 1;
    const int ps = (s.pwm_x100 > 80) ? 1 : (s.pwm_x100 < -80) ? -1 : 0;
    if (d && lastDir && d != lastDir) st.dirChanges++;
    if (ps && lastPwm && ps != lastPwm) st.pwmReversals++;
    if (d) lastDir = d;
    if (ps) lastPwm = ps;
    idx = (idx + 1) % capacity_;
  }
  st.samples = (uint32_t)n;
  st.rmsError = (float)std::sqrt(errSq / (double)n);
  st.avgError = (float)(errSum / (double)n);
  st.avgPwm = (float)(pwmSum / (double)n);
  return st;
}

}  // namespace vcm
