/**
 * @file wifi_manager.h
 * @brief WiFi commissioning and connectivity.
 *
 * First-boot experience: with no configuration, the VCM starts an access
 * point (default SSID "DodgePatrol-VCM") so a phone can connect and finish
 * commissioning from the browser - no code editing required. STA mode is
 * configurable; if the configured network is unreachable, the AP comes
 * back as a fallback so the vehicle can never become unreachable.
 *
 * The radio is brought up independently of I²C and of the vehicle control
 * loop. Once the AP is running it is left alone (no periodic reconfigure).
 */
#pragma once

#include <Arduino.h>

namespace vcm {

struct WifiStats {
  uint32_t apStartMs = 0;
  uint32_t assocCount = 0;
  uint32_t disconnectCount = 0;
  uint32_t dhcpAssignCount = 0;
  uint32_t dhcpFailCount = 0;
  uint32_t apStopCount = 0;
  uint32_t wifiResetCount = 0;
  int8_t lastClientRssi = 0;
};

class WifiManager {
 public:
  void begin();

  /// Housekeeping (reconnect / AP fallback). Call at ~1 Hz.
  /// Does not restart a healthy AP.
  void tick();

  int8_t rssi() const;
  uint8_t clientCount() const;
  String ipAddress() const;
  bool apActive() const { return apActive_; }
  uint32_t apUptimeS() const;
  WifiStats stats() const { return stats_; }

  void noteApStart();
  void noteApStop();
  void noteStaAssoc();
  void noteStaLeave(bool hadIp);
  void noteDhcpAssign();

 private:
  void startAp();
  void sampleClientRssi();
  bool apActive_ = false;
  bool apStartedOnce_ = false;
  uint32_t staConnectStart_ = 0;
  bool gotIpSinceAssoc_ = false;
  WifiStats stats_{};
};

extern WifiManager wifiManager;

}  // namespace vcm
