/**
 * @file web_auth.h
 * @brief Per-device dashboard PIN. The WiFi AP is open; each browser/phone
 *        enters the PIN once and receives a long-lived cookie.
 */
#pragma once

#include <ESPAsyncWebServer.h>

namespace vcm {

class WebAuth {
 public:
  void begin();
  void attach(AsyncWebServer& server);

  bool allowed(AsyncWebServerRequest* req) const;
  /// @return new cookie token, or empty on failure
  String unlock(const char* pin);

 private:
  static constexpr int kMaxDevices = 8;
  static constexpr char kCookie[] = "vcm_dev";

  void load();
  void save() const;
  bool hasToken(const char* tok) const;
  String parseCookie(AsyncWebServerRequest* req) const;

  String tokens_[kMaxDevices];
  uint8_t nextSlot_ = 0;
  mutable SemaphoreHandle_t mutex_ = nullptr;
};

extern WebAuth webAuth;

}  // namespace vcm
