/**
 * @file ota_service.h
 * @brief OTA firmware updates: local .bin upload and GitHub Releases.
 *
 * Uses the ESP32 dual OTA partition scheme (ota_0/ota_1) so the running,
 * known-good image is never overwritten. If an update fails validation
 * the firmware remains on the existing version. Before flashing, the
 * safety manager puts the vehicle into OTA_UPDATE (vehicle stopped, motor
 * outputs disabled, calibration stopped, control disabled).
 *
 * GitHub path: queries /releases/latest of the configured repository,
 * compares semantic versions, downloads the .bin asset, verifies and
 * installs it, then reboots. No secrets are embedded; the repository is
 * public and accessed anonymously.
 */
#pragma once

#include <Arduino.h>

namespace vcm {

enum class OtaState : uint8_t {
  IDLE = 0,
  CHECKING,
  UPDATE_AVAILABLE,
  UP_TO_DATE,
  DOWNLOADING,
  FLASHING,
  SUCCESS,   ///< reboot imminent
  FAILED,
};

struct OtaStatus {
  OtaState state = OtaState::IDLE;
  char latestVersion[32] = {0};
  char releaseNotes[512] = {0};
  char assetUrl[256] = {0};
  char error[96] = {0};
  int progressPct = 0;
  size_t totalBytes = 0, doneBytes = 0;
};

class OtaService {
 public:
  /// Mark the running image valid (cancels rollback) - call once after boot
  /// when the system is healthy.
  void begin();

  /// Query GitHub Releases (async task). Result lands in status().
  bool checkGitHub();

  /// Download + flash the release found by checkGitHub() (async task).
  bool installFromGitHub();

  OtaStatus status() const;

  // Local upload hooks (called by the web server upload handler)
  bool uploadBegin(size_t totalSize);
  bool uploadChunk(const uint8_t* data, size_t len);
  bool uploadEnd(bool ok);

  /// True while an update is in progress (motors must stay disabled).
  bool busy() const;

  /**
   * @brief Compare semantic versions ("1.2.3", tolerates leading 'v').
   * @return >0 if a is newer, <0 if b is newer, 0 if equal.
   */
  static int compareVersions(const char* a, const char* b);

 private:
  static void checkTask(void* arg);
  static void installTask(void* arg);
  void doCheck();
  void doInstall();
  bool enterOtaState();
  void leaveOtaState(bool success);
  void setError(const char* msg);

  mutable SemaphoreHandle_t mutex_ = nullptr;
  OtaStatus status_;
};

extern OtaService ota;

}  // namespace vcm
