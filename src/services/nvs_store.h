/**
 * @file nvs_store.h
 * @brief NVS init, stats, and compact-rewrite of all RAM-backed namespaces.
 *
 * The default 8 MB table gives NVS only 20 kB. A few full config saves
 * plus characterization blobs fill it. The next boot then saw
 * ESP_ERR_NVS_NO_FREE_PAGES, erased the partition, and the car came up
 * on factory defaults. Compact = erase + rewrite from RAM while values
 * are still live.
 */
#pragma once

#include <esp_err.h>

namespace vcm {

/// Call once at boot (after Serial). Recovers a full/version-mismatched
/// partition, then logs used/free entries.
esp_err_t nvsBegin();

void nvsLogStats(const char* why);

/// Erase NVS and rewrite config, steering cal, characterization, and
/// dashboard auth tokens from RAM. Used when a save hits NOT_ENOUGH_SPACE.
bool nvsCompactFromRam();
bool nvsIsCompacting();

}  // namespace vcm
