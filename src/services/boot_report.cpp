#include "services/boot_report.h"

#include <esp_attr.h>
#include <esp_system.h>
#include <rom/rtc.h>
#include <string.h>

#include "services/logger.h"

namespace vcm {

BootReporter bootReport;

static constexpr uint32_t kBootMagic = 0xB007C0DEul;

RTC_NOINIT_ATTR static uint32_t s_bootMagic;
RTC_NOINIT_ATTR static uint32_t s_bootCount;

const char* espResetReasonName(uint32_t reason) {
  switch (static_cast<esp_reset_reason_t>(reason)) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT_PIN";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
#ifdef ESP_RST_USB
    case ESP_RST_USB: return "USB";
#endif
#ifdef ESP_RST_JTAG
    case ESP_RST_JTAG: return "JTAG";
#endif
#ifdef ESP_RST_EFUSE
    case ESP_RST_EFUSE: return "EFUSE";
#endif
#ifdef ESP_RST_PWR_GLITCH
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
#endif
#ifdef ESP_RST_CPU_LOCKUP
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
#endif
    default: return "?";
  }
}

void BootReporter::snap(BootMemSnap& out) const {
  out.heap = ESP.getFreeHeap();
  out.minHeap = ESP.getMinFreeHeap();
  out.largest = ESP.getMaxAllocHeap();
  out.psram = ESP.getFreePsram();
}

void BootReporter::logMem(const char* stage, const BootMemSnap& snap) const {
  LOGI("BOOT", "+%lums %s heap=%u min=%u largest=%u psram=%u",
       (unsigned long)elapsedMs(), stage, (unsigned)snap.heap,
       (unsigned)snap.minHeap, (unsigned)snap.largest, (unsigned)snap.psram);
}

void BootReporter::mark(const char* stage, uint32_t& destMs) {
  destMs = elapsedMs();
  BootMemSnap s;
  snap(s);
  logMem(stage, s);
}

void BootReporter::captureReset() {
  r_.t0Ms = millis();
  r_.resetReason = static_cast<uint32_t>(esp_reset_reason());
  r_.resetReasonCpu0 = static_cast<uint32_t>(rtc_get_reset_reason(0));
  r_.resetReasonCpu1 = static_cast<uint32_t>(rtc_get_reset_reason(1));
  strncpy(r_.resetName, espResetReasonName(r_.resetReason),
          sizeof(r_.resetName) - 1);
  r_.brownout = (r_.resetReason == ESP_RST_BROWNOUT);
  r_.panic = (r_.resetReason == ESP_RST_PANIC);
  r_.watchdog = (r_.resetReason == ESP_RST_INT_WDT ||
                 r_.resetReason == ESP_RST_TASK_WDT ||
                 r_.resetReason == ESP_RST_WDT);

  if (s_bootMagic != kBootMagic || r_.resetReason == ESP_RST_POWERON) {
    s_bootMagic = kBootMagic;
    s_bootCount = 1;
  } else {
    s_bootCount++;
  }
  r_.bootCount = s_bootCount;
  snap(r_.memEarly);

  LOGI("BOOT", "reset=%s (%lu) cpu0=%lu cpu1=%lu boot#%lu", r_.resetName,
       (unsigned long)r_.resetReason, (unsigned long)r_.resetReasonCpu0,
       (unsigned long)r_.resetReasonCpu1, (unsigned long)r_.bootCount);
  if (r_.brownout) {
    LOGE("BOOT", "Brownout detector was triggered — check 5V/3.3V under motor load");
  }
  if (r_.panic) {
    LOGE("BOOT", "Previous run aborted (Guru Meditation / panic)");
  }
  if (r_.watchdog) {
    LOGE("BOOT", "Previous run reset by watchdog");
  }
  if (r_.bootCount >= 5 && r_.resetReason != ESP_RST_POWERON &&
      r_.resetReason != ESP_RST_SW && r_.resetReason != ESP_RST_EXT) {
    LOGW("BOOT", "reboot loop suspected (%lu resets since last power-on)",
         (unsigned long)r_.bootCount);
  }
}

}  // namespace vcm
