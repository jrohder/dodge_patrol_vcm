/**
 * @file wifi_manager.h
 * @brief WiFi commissioning and connectivity.
 *
 * First-boot experience: with no configuration, the VCM starts an access
 * point (default SSID "DodgePatrol-VCM") so a phone can connect and finish
 * commissioning from the browser - no code editing required. STA mode is
 * configurable; if the configured network is unreachable, the AP comes
 * back as a fallback so the vehicle can never become unreachable.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

class WifiManager {
 public:
  void begin();

  /// Housekeeping (reconnect / AP fallback). Call at ~1 Hz.
  void tick();

  int8_t rssi() const;
  uint8_t clientCount() const;
  String ipAddress() const;
  bool apActive() const { return apActive_; }

 private:
  void startAp();
  bool apActive_ = false;
  uint32_t staConnectStart_ = 0;
};

extern WifiManager wifiManager;

}  // namespace vcm
