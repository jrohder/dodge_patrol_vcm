/**
 * @file logger.h
 * @brief In-memory ring-buffer logger with severity levels and module tags.
 *
 * Log lines are kept in a fixed-size RAM ring buffer, mirrored to the USB
 * serial console, viewable/filterable in the web UI (Wi-Fi log viewer)
 * and downloadable as a plain text file. The runtime level is configurable
 * (log.level).
 */
#pragma once

#include <Arduino.h>

namespace vcm {

enum class LogLevel : uint8_t { ERROR = 0, WARN, INFO, DEBUG, TRACE };

struct LogEntry {
  uint32_t ms = 0;
  LogLevel level = LogLevel::INFO;
  char module[8] = {0};
  char message[96] = {0};
  uint32_t seq = 0;
};

class Logger {
 public:
  static constexpr size_t CAPACITY = 256;  ///< entries in the ring buffer

  void begin();
  void setLevel(LogLevel level) { level_ = level; }
  LogLevel level() const { return level_; }

  void log(LogLevel level, const char* module, const char* fmt, ...)
      __attribute__((format(printf, 4, 5)));

  /// Copy entries newer than `afterSeq` into `out` (up to maxCount).
  /// Returns number copied. Used by web UI incremental fetch.
  size_t fetch(uint32_t afterSeq, LogEntry* out, size_t maxCount) const;

  uint32_t latestSeq() const { return nextSeq_ ? nextSeq_ - 1 : 0; }

  /// Render the whole buffer as text (for log download).
  String dumpText() const;

  static const char* levelName(LogLevel l);

 private:
  LogEntry ring_[CAPACITY];
  size_t head_ = 0;   ///< next write slot
  size_t count_ = 0;  ///< valid entries
  uint32_t nextSeq_ = 1;
  volatile LogLevel level_ = LogLevel::INFO;
  mutable SemaphoreHandle_t mutex_ = nullptr;
};

extern Logger logger;

}  // namespace vcm

#define LOGE(mod, ...) vcm::logger.log(vcm::LogLevel::ERROR, mod, __VA_ARGS__)
#define LOGW(mod, ...) vcm::logger.log(vcm::LogLevel::WARN, mod, __VA_ARGS__)
#define LOGI(mod, ...) vcm::logger.log(vcm::LogLevel::INFO, mod, __VA_ARGS__)
#define LOGD(mod, ...) vcm::logger.log(vcm::LogLevel::DEBUG, mod, __VA_ARGS__)
#define LOGT(mod, ...) vcm::logger.log(vcm::LogLevel::TRACE, mod, __VA_ARGS__)
