#include "services/ota_service.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <string.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>

#include "config/config_registry.h"
#include "core/version.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"

namespace vcm {

OtaService ota;

// ESP_APP_DESC_MAGIC_WORD at offset 0x20. Merged *-factory.bin images start
// with a bootloader (also 0xE9) and fail this check, so Arduino Update must
// not be allowed to "succeed" them into an OTA app slot.
static bool looksLikeEspAppImage(const uint8_t* d, size_t n) {
  if (n < 36 || d[0] != ESP_IMAGE_HEADER_MAGIC) return false;
  uint32_t magic = 0;
  memcpy(&magic, d + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t),
         sizeof(magic));
  return magic == ESP_APP_DESC_MAGIC_WORD;
}

void OtaService::confirmRunningImage() {
  esp_ota_mark_app_valid_cancel_rollback();
}

void OtaService::begin() {
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
  const esp_partition_t* running = esp_ota_get_running_partition();
  LOGI("OTA", "Firmware %s (%s) running from %s", VCM_FW_VERSION,
       VCM_GIT_COMMIT, running ? running->label : "?");
}

bool OtaService::inspectAppImage(const uint8_t* data, size_t len) {
  if (appHdrChecked_) return true;
  const size_t room = sizeof(appHdr_) - appHdrLen_;
  const size_t n = len < room ? len : room;
  memcpy(appHdr_ + appHdrLen_, data, n);
  appHdrLen_ += n;
  if (appHdrLen_ < sizeof(appHdr_)) return true;
  appHdrChecked_ = true;
  if (looksLikeEspAppImage(appHdr_, appHdrLen_)) return true;
  setError("Not an OTA app image (use dodge_patrol_vcm-<ver>.bin, not *-factory.bin)");
  Update.abort();
  return false;
}

OtaStatus OtaService::status() const {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  OtaStatus copy = status_;
  xSemaphoreGive(mutex_);
  return copy;
}

bool OtaService::busy() const {
  const OtaState s = status().state;
  return s == OtaState::CHECKING || s == OtaState::DOWNLOADING ||
         s == OtaState::FLASHING;
}

void OtaService::setError(const char* msg) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.state = OtaState::FAILED;
  strncpy(status_.error, msg, sizeof(status_.error) - 1);
  xSemaphoreGive(mutex_);
  LOGE("OTA", "%s", msg);
}

int OtaService::compareVersions(const char* a, const char* b) {
  if (*a == 'v' || *a == 'V') a++;
  if (*b == 'v' || *b == 'V') b++;
  int pa[3] = {0, 0, 0}, pb[3] = {0, 0, 0};
  sscanf(a, "%d.%d.%d", &pa[0], &pa[1], &pa[2]);
  sscanf(b, "%d.%d.%d", &pb[0], &pb[1], &pb[2]);
  for (int i = 0; i < 3; ++i) {
    if (pa[i] != pb[i]) return pa[i] - pb[i];
  }
  return 0;
}

// ---------------------------------------------------------------- GitHub

bool OtaService::checkGitHub() {
  if (busy()) return false;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.state = OtaState::CHECKING;
  status_.error[0] = 0;
  xSemaphoreGive(mutex_);
  xTaskCreatePinnedToCore(checkTask, "ota_check", 12288, this, 1, nullptr, 0);
  return true;
}

void OtaService::checkTask(void* arg) {
  static_cast<OtaService*>(arg)->doCheck();
  vTaskDelete(nullptr);
}

void OtaService::doCheck() {
  const String repo = config.s(S_OTA_REPO);
  const String url = "https://api.github.com/repos/" + repo +
                     (config.b(OTA_PRERELEASE) ? "/releases" : "/releases/latest");
  LOGI("OTA", "Checking %s", url.c_str());

  WiFiClientSecure client;
  client.setInsecure();  // public release metadata; integrity via size checks
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("dodge-patrol-vcm");
  if (!http.begin(client, url)) {
    setError("HTTP begin failed");
    return;
  }
  const int code = http.GET();
  if (code != 200) {
    http.end();
    char msg[64];
    snprintf(msg, sizeof(msg), "GitHub API HTTP %d", code);
    setError(msg);
    return;
  }

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, http.getStream());
  http.end();
  if (err) {
    setError("Release JSON parse failed");
    return;
  }
  JsonObject rel = config.b(OTA_PRERELEASE) && doc.is<JsonArray>()
                       ? doc.as<JsonArray>()[0].as<JsonObject>()
                       : doc.as<JsonObject>();
  const char* tag = rel["tag_name"] | "";
  const char* body = rel["body"] | "";
  if (!*tag) {
    setError("No release found");
    return;
  }

  // Find the OTA .bin asset (skip merged "factory" images)
  String assetUrl;
  for (JsonObject asset : rel["assets"].as<JsonArray>()) {
    const char* name = asset["name"] | "";
    if (strstr(name, ".bin") && !strstr(name, "factory") &&
        !strstr(name, "merged")) {
      assetUrl = (const char*)(asset["browser_download_url"] | "");
      break;
    }
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);
  strncpy(status_.latestVersion, tag, sizeof(status_.latestVersion) - 1);
  strncpy(status_.releaseNotes, body, sizeof(status_.releaseNotes) - 1);
  strncpy(status_.assetUrl, assetUrl.c_str(), sizeof(status_.assetUrl) - 1);
  const bool newer = compareVersions(tag, VCM_FW_VERSION) > 0;
  status_.state = (newer && assetUrl.length())
                      ? OtaState::UPDATE_AVAILABLE
                      : OtaState::UP_TO_DATE;
  xSemaphoreGive(mutex_);
  LOGI("OTA", "Latest release %s (%s)", tag,
       newer ? "update available" : "up to date");
}

bool OtaService::installFromGitHub() {
  if (busy()) return false;
  if (status().state != OtaState::UPDATE_AVAILABLE) return false;
  xTaskCreatePinnedToCore(installTask, "ota_inst", 12288, this, 1, nullptr, 0);
  return true;
}

void OtaService::installTask(void* arg) {
  static_cast<OtaService*>(arg)->doInstall();
  vTaskDelete(nullptr);
}

bool OtaService::enterOtaState() {
  calibration.abortSteeringCal();
  calibration.stopMotorTest();
  if (!safety.requestState(VehicleState::OTA_UPDATE, "firmware update")) {
    setError("Vehicle must be stopped before updating");
    return false;
  }
  return true;
}

void OtaService::leaveOtaState(bool success) {
  if (!success && safety.state() == VehicleState::OTA_UPDATE) {
    safety.requestState(calibration.commissioned() ? VehicleState::READY
                                                   : VehicleState::NOT_CALIBRATED,
                        "OTA finished");
  }
}

void OtaService::doInstall() {
  if (!enterOtaState()) return;
  const OtaStatus st = status();

  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.state = OtaState::DOWNLOADING;
  status_.progressPct = 0;
  xSemaphoreGive(mutex_);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("dodge-patrol-vcm");
  http.setTimeout(30000);
  if (!http.begin(client, st.assetUrl)) {
    setError("Download begin failed");
    leaveOtaState(false);
    return;
  }
  const int code = http.GET();
  const int total = http.getSize();
  if (code != 200 || total <= 0) {
    http.end();
    setError("Asset download failed");
    leaveOtaState(false);
    return;
  }
  WiFiClient* stream = http.getStreamPtr();
  uint8_t first[64];
  size_t firstLen = 0;
  uint32_t waitStart = millis();
  while (firstLen < 36 && millis() - waitStart < 15000) {
    const size_t avail = stream->available();
    if (avail) {
      firstLen += stream->readBytes(first + firstLen,
                                    min(avail, sizeof(first) - firstLen));
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  if (!looksLikeEspAppImage(first, firstLen)) {
    http.end();
    setError("Not an OTA app image (use dodge_patrol_vcm-<ver>.bin, not *-factory.bin)");
    leaveOtaState(false);
    return;
  }

  if (!Update.begin(total)) {
    http.end();
    setError("Not enough OTA space");
    leaveOtaState(false);
    return;
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.state = OtaState::FLASHING;
  status_.totalBytes = total;
  xSemaphoreGive(mutex_);

  uint8_t buf[2048];
  size_t written = 0;
  uint32_t lastData = millis();
  if (Update.write(first, firstLen) != firstLen) {
    http.end();
    Update.abort();
    setError("Update failed validation; keeping current firmware");
    leaveOtaState(false);
    return;
  }
  written = firstLen;
  while (written < (size_t)total) {
    const size_t avail = stream->available();
    if (avail) {
      const size_t n = stream->readBytes(buf, min(avail, sizeof(buf)));
      if (Update.write(buf, n) != n) break;
      written += n;
      lastData = millis();
      xSemaphoreTake(mutex_, portMAX_DELAY);
      status_.doneBytes = written;
      status_.progressPct = (int)(written * 100 / total);
      xSemaphoreGive(mutex_);
    } else if (millis() - lastData > 15000) {
      break;  // stalled
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  http.end();

  if (written == (size_t)total && Update.end(true)) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    status_.state = OtaState::SUCCESS;
    status_.progressPct = 100;
    xSemaphoreGive(mutex_);
    LOGI("OTA", "Update verified; rebooting into %s", st.latestVersion);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
  } else {
    Update.abort();
    setError("Update failed validation; keeping current firmware");
    leaveOtaState(false);
  }
}

// ---------------------------------------------------------------- upload

bool OtaService::uploadBegin(size_t totalSize) {
  if (busy()) return false;
  if (!enterOtaState()) return false;
  appHdrLen_ = 0;
  appHdrChecked_ = false;
  memset(appHdr_, 0, sizeof(appHdr_));
  if (!Update.begin(totalSize ? totalSize : UPDATE_SIZE_UNKNOWN)) {
    setError("Not enough OTA space");
    leaveOtaState(false);
    return false;
  }
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.state = OtaState::FLASHING;
  status_.error[0] = 0;
  status_.progressPct = 0;
  status_.totalBytes = totalSize;
  status_.doneBytes = 0;
  xSemaphoreGive(mutex_);
  LOGI("OTA", "Local upload started (%u bytes)", (unsigned)totalSize);
  return true;
}

bool OtaService::uploadChunk(const uint8_t* data, size_t len) {
  if (!inspectAppImage(data, len)) return false;
  if (Update.write(const_cast<uint8_t*>(data), len) != len) return false;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_.doneBytes += len;
  if (status_.totalBytes)
    status_.progressPct = (int)(status_.doneBytes * 100 / status_.totalBytes);
  xSemaphoreGive(mutex_);
  return true;
}

bool OtaService::uploadEnd(bool ok) {
  if (ok && Update.end(true)) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    status_.state = OtaState::SUCCESS;
    status_.progressPct = 100;
    xSemaphoreGive(mutex_);
    LOGI("OTA", "Local upload verified; rebooting");
    return true;
  }
  Update.abort();
  setError("Uploaded image failed validation; keeping current firmware");
  leaveOtaState(false);
  return false;
}

}  // namespace vcm
