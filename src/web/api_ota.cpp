/**
 * @file api_ota.cpp
 * @brief OTA REST API: status, GitHub check/install, local .bin upload.
 */
#include <ESPAsyncWebServer.h>
#include <Update.h>

#include "core/version.h"
#include "services/logger.h"
#include "services/ota_service.h"
#include "web/web_server.h"

namespace vcm {

static const char* otaStateName(OtaState s) {
  switch (s) {
    case OtaState::IDLE: return "IDLE";
    case OtaState::CHECKING: return "CHECKING";
    case OtaState::UPDATE_AVAILABLE: return "UPDATE_AVAILABLE";
    case OtaState::UP_TO_DATE: return "UP_TO_DATE";
    case OtaState::DOWNLOADING: return "DOWNLOADING";
    case OtaState::FLASHING: return "FLASHING";
    case OtaState::SUCCESS: return "SUCCESS";
    case OtaState::FAILED: return "FAILED";
  }
  return "?";
}

void VcmWebServer::setupOtaApi() {
  server_.on("/api/ota/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    const OtaStatus st = ota.status();
    JsonDocument doc;
    doc["state"] = otaStateName(st.state);
    doc["current_version"] = VCM_FW_VERSION;
    doc["build_date"] = VCM_BUILD_DATE;
    doc["latest_version"] = st.latestVersion;
    doc["release_notes"] = st.releaseNotes;
    doc["progress"] = st.progressPct;
    doc["total"] = st.totalBytes;
    doc["done"] = st.doneBytes;
    doc["error"] = st.error;
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server_.on("/api/ota/check", HTTP_POST, [](AsyncWebServerRequest* req) {
    const bool ok = ota.checkGitHub();
    req->send(ok ? 200 : 409, "application/json",
              ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"busy\"}");
  });

  server_.on("/api/ota/install", HTTP_POST, [](AsyncWebServerRequest* req) {
    const bool ok = ota.installFromGitHub();
    req->send(ok ? 200 : 409, "application/json",
              ok ? "{\"ok\":true}"
                 : "{\"ok\":false,\"error\":\"no update available\"}");
  });

  // Local .bin upload (multipart/form-data)
  server_.on(
      "/api/ota/upload", HTTP_POST,
      [](AsyncWebServerRequest* req) {
        const bool ok = !Update.hasError() && ota.uploadEnd(true);
        req->send(ok ? 200 : 400, "application/json",
                  ok ? "{\"ok\":true,\"rebooting\":true}"
                     : "{\"ok\":false,\"error\":\"validation failed\"}");
        if (ok) {
          xTaskCreate([](void*) { vTaskDelay(pdMS_TO_TICKS(750));
                                  ESP.restart(); },
                      "ota_rb", 2048, nullptr, 1, nullptr);
        }
      },
      [](AsyncWebServerRequest* req, String filename, size_t index,
         uint8_t* data, size_t len, bool final) {
        if (index == 0) {
          const size_t total =
              req->contentLength();  // approximate (multipart overhead)
          if (!ota.uploadBegin(total)) {
            req->send(409, "application/json",
                      "{\"ok\":false,\"error\":\"cannot start update\"}");
            return;
          }
          LOGI("OTA", "Uploading %s", filename.c_str());
        }
        if (len && !ota.uploadChunk(data, len)) {
          LOGE("OTA", "Upload write failed at %u", (unsigned)index);
        }
        (void)final;
      });
}

}  // namespace vcm
