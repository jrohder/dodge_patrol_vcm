#include "web/web_auth.h"

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_random.h>
#include <string.h>

#include "config/config_registry.h"
#include "services/logger.h"

namespace vcm {

WebAuth webAuth;

static bool pinsEqual(const String& a, const char* b) {
  const size_t n = a.length();
  const size_t m = b ? strlen(b) : 0;
  const size_t len = n > m ? n : m;
  uint8_t diff = static_cast<uint8_t>(n != m);
  for (size_t i = 0; i < len; ++i) {
    const char ca = i < n ? a[i] : 0;
    const char cb = i < m ? b[i] : 0;
    diff |= static_cast<uint8_t>(ca ^ cb);
  }
  return diff == 0 && m > 0;
}

static String randomToken() {
  uint8_t raw[16];
  esp_fill_random(raw, sizeof(raw));
  char hex[33];
  for (int i = 0; i < 16; ++i) snprintf(hex + i * 2, 3, "%02x", raw[i]);
  return String(hex);
}

void WebAuth::begin() {
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
  load();
}

void WebAuth::attach(AsyncWebServer& server) {
  server.on("/api/auth/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["authed"] = allowed(req);
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/auth/unlock",
      [this](AsyncWebServerRequest* req, JsonVariant& json) {
        const char* pin = json["pin"] | "";
        const String tok = unlock(pin);
        if (!tok.length()) {
          req->send(403, "application/json",
                    "{\"ok\":false,\"error\":\"bad pin\"}");
          return;
        }
        char cookie[96];
        snprintf(cookie, sizeof(cookie),
                 "%s=%s; Path=/; Max-Age=31536000; SameSite=Lax", kCookie,
                 tok.c_str());
        AsyncWebServerResponse* res =
            req->beginResponse(200, "application/json", "{\"ok\":true}");
        res->addHeader("Set-Cookie", cookie);
        req->send(res);
      }));

  server.addMiddleware([this](AsyncWebServerRequest* req, ArMiddlewareNext next) {
    const String url = req->url();
    if (url == "/" || url == "/app.css" || url == "/app.js" ||
        url.startsWith("/api/auth/") || url.indexOf("hotspot-detect") >= 0 ||
        url.indexOf("generate_204") >= 0 || url.indexOf("success.html") >= 0 ||
        url.indexOf("ncsi.txt") >= 0 || url.indexOf("connecttest.txt") >= 0) {
      next();
      return;
    }
    if (allowed(req)) {
      next();
      return;
    }
    req->send(401, "application/json", "{\"ok\":false,\"error\":\"auth\"}");
  });
}

bool WebAuth::allowed(AsyncWebServerRequest* req) const {
  const String tok = parseCookie(req);
  if (!tok.length()) return false;
  return hasToken(tok.c_str());
}

String WebAuth::unlock(const char* pin) {
  const String expected = config.s(S_WEB_PIN);
  if (!pinsEqual(expected, pin ? pin : "")) {
    LOGW("AUTH", "Bad dashboard PIN");
    return String();
  }
  const String tok = randomToken();
  xSemaphoreTake(mutex_, portMAX_DELAY);
  tokens_[nextSlot_ % kMaxDevices] = tok;
  nextSlot_ = static_cast<uint8_t>((nextSlot_ + 1) % kMaxDevices);
  xSemaphoreGive(mutex_);
  save();
  LOGI("AUTH", "Device unlocked (slot rotated)");
  return tok;
}

void WebAuth::load() {
  Preferences prefs;
  if (!prefs.begin("vcm_auth", true)) return;
  nextSlot_ = prefs.getUChar("next", 0);
  for (int i = 0; i < kMaxDevices; ++i) {
    char key[4];
    snprintf(key, sizeof(key), "d%d", i);
    tokens_[i] = prefs.getString(key, "");
  }
  prefs.end();
}

void WebAuth::save() const {
  Preferences prefs;
  if (!prefs.begin("vcm_auth", false)) return;
  prefs.putUChar("next", nextSlot_);
  xSemaphoreTake(mutex_, portMAX_DELAY);
  for (int i = 0; i < kMaxDevices; ++i) {
    char key[4];
    snprintf(key, sizeof(key), "d%d", i);
    prefs.putString(key, tokens_[i]);
  }
  xSemaphoreGive(mutex_);
  prefs.end();
}

bool WebAuth::hasToken(const char* tok) const {
  if (!tok || !tok[0]) return false;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  bool ok = false;
  for (int i = 0; i < kMaxDevices; ++i) {
    if (tokens_[i].length() && tokens_[i] == tok) {
      ok = true;
      break;
    }
  }
  xSemaphoreGive(mutex_);
  return ok;
}

String WebAuth::parseCookie(AsyncWebServerRequest* req) const {
  if (!req->hasHeader("Cookie")) return String();
  const String cookie = req->getHeader("Cookie")->value();
  const String prefix = String(kCookie) + "=";
  int at = cookie.indexOf(prefix);
  if (at < 0) return String();
  at += prefix.length();
  int end = cookie.indexOf(';', at);
  if (end < 0) end = cookie.length();
  return cookie.substring(at, end);
}

}  // namespace vcm
