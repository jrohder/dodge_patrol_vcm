#include "drivers/wifi/wifi_manager.h"

#include <ESPmDNS.h>
#include <WiFi.h>

#include "config/config_registry.h"
#include "services/logger.h"

namespace vcm {

WifiManager wifiManager;

static constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 20000;

void WifiManager::startAp() {
  String ssid = config.s(S_WIFI_AP_SSID);
  String pass = config.s(S_WIFI_AP_PASS);
  if (pass.length() > 0 && pass.length() < 8) {
    LOGW("WIFI", "AP password too short; starting open AP");
    pass = "";
  }
  WiFi.softAP(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
  apActive_ = true;
  LOGI("WIFI", "AP '%s' at %s", ssid.c_str(),
       WiFi.softAPIP().toString().c_str());
}

void WifiManager::begin() {
  WiFi.persistent(false);
  const int mode = config.i(WIFI_MODE);  // 0=AP 1=STA 2=AP_STA
  const String staSsid = config.s(S_WIFI_STA_SSID);

  const bool wantSta = (mode != 0) && staSsid.length() > 0;
  const bool wantAp = (mode != 1) || !wantSta;

  WiFi.mode(wantSta && wantAp ? WIFI_AP_STA : (wantSta ? WIFI_STA : WIFI_AP));
  WiFi.setHostname(config.s(S_HOSTNAME).c_str());

  if (wantAp) startAp();
  if (wantSta) {
    LOGI("WIFI", "Connecting to '%s'...", staSsid.c_str());
    WiFi.begin(staSsid.c_str(), config.s(S_WIFI_STA_PASS).c_str());
    staConnectStart_ = millis();
  }

  if (MDNS.begin(config.s(S_HOSTNAME).c_str())) {
    MDNS.addService("http", "tcp", 80);
    LOGI("WIFI", "mDNS: http://%s.local", config.s(S_HOSTNAME).c_str());
  }
}

void WifiManager::tick() {
  const int mode = config.i(WIFI_MODE);
  if (mode == 1 && !apActive_ && staConnectStart_ != 0 &&
      WiFi.status() != WL_CONNECTED &&
      millis() - staConnectStart_ > STA_CONNECT_TIMEOUT_MS) {
    // STA-only but unreachable: bring the AP back so we can't be locked out
    LOGW("WIFI", "STA connect timeout; enabling fallback AP");
    WiFi.mode(WIFI_AP_STA);
    startAp();
  }
  static bool wasConnected = false;
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    LOGI("WIFI", "STA connected: %s (RSSI %d)",
         WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else if (!connected && wasConnected) {
    LOGW("WIFI", "STA disconnected");
  }
  wasConnected = connected;
}

int8_t WifiManager::rssi() const {
  return WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
}

uint8_t WifiManager::clientCount() const {
  return apActive_ ? WiFi.softAPgetStationNum() : 0;
}

String WifiManager::ipAddress() const {
  if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
  if (apActive_) return WiFi.softAPIP().toString();
  return "0.0.0.0";
}

}  // namespace vcm
