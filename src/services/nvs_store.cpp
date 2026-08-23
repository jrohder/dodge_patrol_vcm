#include "services/nvs_store.h"

#include <nvs.h>
#include <nvs_flash.h>

#include "config/config_registry.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/steering_characterization.h"
#include "web/web_auth.h"

namespace vcm {

static bool s_compacting = false;

void nvsLogStats(const char* why) {
  nvs_stats_t st{};
  const esp_err_t err = nvs_get_stats(nullptr, &st);
  if (err != ESP_OK) {
    LOGW("NVS", "%s stats failed: %s", why ? why : "nvs", esp_err_to_name(err));
    return;
  }
  LOGI("NVS", "%s used=%u free=%u total=%u namespaces=%u", why ? why : "nvs",
       (unsigned)st.used_entries, (unsigned)st.free_entries,
       (unsigned)st.total_entries, (unsigned)st.namespace_count);
}

esp_err_t nvsBegin() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    LOGW("NVS", "init %s — erasing 20 kB partition (will restore factory cal)",
         esp_err_to_name(err));
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    LOGE("NVS", "nvs_flash_init failed: %s", esp_err_to_name(err));
  } else {
    nvsLogStats("boot");
  }
  return err;
}

bool nvsCompactFromRam() {
  if (s_compacting) {
    LOGE("NVS", "compact re-entered");
    return false;
  }
  s_compacting = true;
  LOGW("NVS", "partition full — compacting (erase + rewrite from RAM)");
  nvs_flash_erase();
  const esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    LOGE("NVS", "compact init failed: %s", esp_err_to_name(err));
    s_compacting = false;
    return false;
  }
  const bool okCfg = config.save();
  const bool okCal = calibration.saveSteering();
  steerChar.persistEssential();
  webAuth.persist();
  nvsLogStats("after compact");
  s_compacting = false;
  LOGI("NVS", "compact done cfg=%d cal=%d", okCfg ? 1 : 0, okCal ? 1 : 0);
  return okCfg && okCal;
}

bool nvsIsCompacting() { return s_compacting; }

}  // namespace vcm
