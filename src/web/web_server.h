/**
 * @file web_server.h
 * @brief Async web server: embedded UI, REST API and WebSocket telemetry.
 *
 * The UI is a mobile-first single-page app served gzip-compressed from
 * flash. Continuous driving commands (web remote) use the WebSocket, never
 * HTTP requests; a missing control heartbeat stops the vehicle. Telemetry
 * can be paused/resumed per client if bandwidth becomes an issue.
 */
#pragma once

#include <ESPAsyncWebServer.h>

namespace vcm {

class VcmWebServer {
 public:
  void begin();

  /// Broadcast one telemetry frame to all WebSocket clients (20 Hz task).
  void broadcastTelemetry();

  /// Send a batch of packed 200 Hz steering samples to subscribed clients.
  /// Must NOT run inside the steering control task.
  void broadcastSteerDiag();

  /// Periodic WS housekeeping (client cleanup). Call at ~1 Hz.
  void tick();

 private:
  void setupStatic();
  void setupApi();          // api_routes.cpp
  void setupOtaApi();       // api_ota.cpp
  void setupSteeringApi();  // api_steering.cpp
  void setupWebSocket();
  void handleWsMessage(AsyncWebSocketClient* client, const char* data,
                       size_t len);
  void sdiagSubscribe(uint32_t clientId, bool on);
  String buildTelemetryJson();

  AsyncWebServer server_{80};
  AsyncWebSocket ws_{"/ws"};
  bool telemetryPaused_ = false;

  static constexpr int kMaxSdiagClients = 4;
  uint32_t sdiagIds_[kMaxSdiagClients] = {};
  uint32_t sdiagSeq_[kMaxSdiagClients] = {};
  uint8_t sdiagCount_ = 0;
};

extern VcmWebServer webServer;

}  // namespace vcm
