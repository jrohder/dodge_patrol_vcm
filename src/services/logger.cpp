#include "services/logger.h"

#include <cstdarg>

namespace vcm {

Logger logger;

void Logger::begin() {
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
}

const char* Logger::levelName(LogLevel l) {
  switch (l) {
    case LogLevel::ERROR: return "ERROR";
    case LogLevel::WARN: return "WARN";
    case LogLevel::INFO: return "INFO";
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::TRACE: return "TRACE";
  }
  return "?";
}

void Logger::log(LogLevel level, const char* module, const char* fmt, ...) {
  if (level > level_) return;

  LogEntry e;
  e.ms = millis();
  e.level = level;
  strncpy(e.module, module, sizeof(e.module) - 1);
  va_list args;
  va_start(args, fmt);
  vsnprintf(e.message, sizeof(e.message), fmt, args);
  va_end(args);

  // USB CDC (Serial) and UART0 (Serial0 / DevKit TX-RX / USB-UART dongle)
  char line[192];
  snprintf(line, sizeof(line), "%8lu [%-5s] %-7s %s\n", (unsigned long)e.ms,
           levelName(level), e.module, e.message);
  Serial.print(line);
  Serial0.print(line);

  if (!mutex_) return;  // before begin(): serial only
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return;
  e.seq = nextSeq_++;
  ring_[head_] = e;
  head_ = (head_ + 1) % CAPACITY;
  if (count_ < CAPACITY) count_++;
  xSemaphoreGive(mutex_);
}

size_t Logger::fetch(uint32_t afterSeq, LogEntry* out, size_t maxCount) const {
  if (!mutex_) return 0;
  size_t n = 0;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const size_t start = (head_ + CAPACITY - count_) % CAPACITY;
  for (size_t i = 0; i < count_ && n < maxCount; ++i) {
    const LogEntry& e = ring_[(start + i) % CAPACITY];
    if (e.seq > afterSeq) out[n++] = e;
  }
  xSemaphoreGive(mutex_);
  return n;
}

String Logger::dumpText() const {
  String out;
  out.reserve(count_ * 64);
  if (!mutex_) return out;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const size_t start = (head_ + CAPACITY - count_) % CAPACITY;
  for (size_t i = 0; i < count_; ++i) {
    const LogEntry& e = ring_[(start + i) % CAPACITY];
    char line[160];
    const uint32_t s = e.ms / 1000, msRem = e.ms % 1000;
    snprintf(line, sizeof(line), "%02lu:%02lu:%02lu.%03lu [%-5s] %-7s %s\n",
             (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60),
             (unsigned long)(s % 60), (unsigned long)msRem,
             levelName(e.level), e.module, e.message);
    out += line;
  }
  xSemaphoreGive(mutex_);
  return out;
}

}  // namespace vcm
