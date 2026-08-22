#include "drivers/wifi/wifi_manager.h"

#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <string.h>

#include "dhcpserver/dhcpserver.h"
#include "dhcpserver/dhcpserver_options.h"

#include "config/config_registry.h"
#include "services/logger.h"

namespace vcm {

WifiManager wifiManager;

static constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 20000;
static const IPAddress kApIp(192, 168, 4, 1);
static const IPAddress kApMask(255, 255, 255, 0);

static DNSServer dnsServer;
static TaskHandle_t dnsTaskHandle = nullptr;

static void dnsTask(void*) {
  for (;;) {
    dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void configureApDhcp(const IPAddress& ip) {
  esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (!ap) {
    LOGW("WIFI", "no AP netif; DHCP not restarted");
    return;
  }

  esp_netif_dhcp_status_t st = ESP_NETIF_DHCP_INIT;
  esp_netif_dhcps_get_status(ap, &st);
  LOGI("WIFI", "dhcps before restart status=%d", static_cast<int>(st));

  // One-time DHCPS restart so iOS gets a lease. Do not repeat this while
  // the AP is already running.
  esp_netif_dhcps_stop(ap);

  dhcps_offer_t offer_dns = OFFER_DNS;
  const esp_err_t dns_opt = esp_netif_dhcps_option(
      ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer_dns,
      sizeof(offer_dns));
  esp_netif_dns_info_t dns{};
  dns.ip.type = ESP_IPADDR_TYPE_V4;
  dns.ip.u_addr.ip4.addr = static_cast<uint32_t>(ip);
  const esp_err_t dns_set = esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns);
  const esp_err_t start = esp_netif_dhcps_start(ap);
  esp_netif_dhcps_get_status(ap, &st);
  LOGI("WIFI", "dhcps restart start=%s dns_opt=%s dns_set=%s status=%d",
       esp_err_to_name(start), esp_err_to_name(dns_opt),
       esp_err_to_name(dns_set), static_cast<int>(st));
}

static void startCaptiveDns(const IPAddress& ip) {
  dnsServer.stop();
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  if (!dnsServer.start(53, "*", ip)) {
    LOGW("WIFI", "captive DNS bind failed");
    return;
  }
  if (!dnsTaskHandle) {
    xTaskCreatePinnedToCore(dnsTask, "dns_cap", 3072, nullptr, 2, &dnsTaskHandle,
                            1);
  }
  LOGI("WIFI", "captive DNS * -> %s", ip.toString().c_str());
}

static const char* authName(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    default: return "?";
  }
}

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_START:
      wifiManager.noteApStart();
      LOGI("WIFI", "event AP_START");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      wifiManager.noteApStop();
      LOGW("WIFI", "event AP_STOP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED: {
      const auto& s = info.wifi_ap_staconnected;
      wifiManager.noteStaAssoc();
      LOGI("WIFI", "STA assoc %02X:%02X:%02X:%02X:%02X:%02X aid=%u", s.mac[0],
           s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5], s.aid);
      break;
    }
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: {
      const auto& s = info.wifi_ap_stadisconnected;
      wifiManager.noteStaLeave(false);
      LOGW("WIFI", "STA leave  %02X:%02X:%02X:%02X:%02X:%02X aid=%u", s.mac[0],
           s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5], s.aid);
      break;
    }
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      wifiManager.noteDhcpAssign();
      LOGI("WIFI", "STA DHCP %s",
           IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str());
      break;
    default:
      break;
  }
}

void WifiManager::noteApStart() {
  if (stats_.apStartMs == 0) stats_.apStartMs = millis();
  apActive_ = true;
}

void WifiManager::noteApStop() {
  stats_.apStopCount++;
  apActive_ = false;
}

void WifiManager::noteStaAssoc() {
  stats_.assocCount++;
  gotIpSinceAssoc_ = false;
}

void WifiManager::noteStaLeave(bool /*hadIp*/) {
  stats_.disconnectCount++;
  if (!gotIpSinceAssoc_) stats_.dhcpFailCount++;
  gotIpSinceAssoc_ = false;
}

void WifiManager::noteDhcpAssign() { 
  stats_.dhcpAssignCount++;
  gotIpSinceAssoc_ = true;
}

uint32_t WifiManager::apUptimeS() const {
  if (!apActive_ || stats_.apStartMs == 0) return 0;
  return (millis() - stats_.apStartMs) / 1000;
}

void WifiManager::startAp() {
  if (apStartedOnce_) stats_.wifiResetCount++;

  String ssid = config.s(S_WIFI_AP_SSID);
  String pass = config.s(S_WIFI_AP_PASS);
  if (pass.length() > 0 && pass.length() < 8) {
    LOGW("WIFI", "AP password too short; starting open AP");
    pass = "";
  }

  // Country / PHY / power-save / HT20 before the AP is advertised. Doing
  // this after softAP() restarts the radio (AP_STOP) and kills DHCPS.
  const esp_err_t ctry = esp_wifi_set_country_code("US", false);
  LOGI("WIFI", "country US (%s)", esp_err_to_name(ctry));
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);

  if (!WiFi.softAPConfig(kApIp, kApIp, kApMask)) {
    LOGW("WIFI", "softAPConfig failed");
  }

  const bool ok = WiFi.softAP(ssid.c_str(), pass.length() ? pass.c_str() : nullptr,
                              6, 0, 4);
  delay(50);
  configureApDhcp(kApIp);
  startCaptiveDns(kApIp);

  wifi_config_t conf;
  memset(&conf, 0, sizeof(conf));
  esp_wifi_get_config(WIFI_IF_AP, &conf);
  apActive_ = true;
  apStartedOnce_ = true;
  if (stats_.apStartMs == 0) stats_.apStartMs = millis();
  LOGI("WIFI", "AP %s ssid='%s' auth=%s ch=%u ip=%s heap=%u", ok ? "ok" : "FAIL",
       reinterpret_cast<char*>(conf.ap.ssid), authName(conf.ap.authmode),
       conf.ap.channel, WiFi.softAPIP().toString().c_str(),
       static_cast<unsigned>(ESP.getFreeHeap()));
  if (!pass.length()) {
    LOGI("WIFI", "Open AP — enter dashboard PIN once per device in the web UI");
  }
}

void WifiManager::begin() {
  WiFi.persistent(false);
  WiFi.onEvent(onWifiEvent);
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

void WifiManager::sampleClientRssi() {
  wifi_sta_list_t list;
  memset(&list, 0, sizeof(list));
  if (esp_wifi_ap_get_sta_list(&list) != ESP_OK || list.num == 0) {
    stats_.lastClientRssi = 0;
    return;
  }
  stats_.lastClientRssi = list.sta[0].rssi;
}

void WifiManager::tick() {
  const int mode = config.i(WIFI_MODE);
  if (mode == 1 && !apActive_ && staConnectStart_ != 0 &&
      WiFi.status() != WL_CONNECTED &&
      millis() - staConnectStart_ > STA_CONNECT_TIMEOUT_MS) {
    LOGW("WIFI", "STA connect timeout; enabling fallback AP");
    WiFi.mode(WIFI_AP_STA);
    startAp();
    staConnectStart_ = 0;
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

  if (apActive_) {
    sampleClientRssi();
    static uint8_t lastClients = 0xFF;
    const uint8_t n = clientCount();
    if (n != lastClients) {
      lastClients = n;
      LOGI("WIFI", "AP clients=%u rssi=%d", n, stats_.lastClientRssi);
    }
  }
}

int8_t WifiManager::rssi() const {
  if (WiFi.status() == WL_CONNECTED) return WiFi.RSSI();
  return stats_.lastClientRssi;
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
