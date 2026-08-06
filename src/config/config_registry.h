/**
 * @file config_registry.h
 * @brief Central configuration database with metadata, NVS persistence,
 *        apply/save/revert semantics and JSON import/export.
 *
 * Semantics:
 *  - apply: change the RAM value only (live tuning; lost on reboot)
 *  - save:  persist all RAM values to NVS
 *  - revert: reload RAM values from NVS (last saved)
 *  - factoryReset: restore defaults (requires explicit confirmation in UI)
 *
 * A configuration schema version is stored alongside values; on boot,
 * older versions are migrated where compatible.
 */
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "config/param.h"

namespace vcm {

class ConfigRegistry {
 public:
  static constexpr uint16_t SCHEMA_VERSION = 1;

  /// Load saved values from NVS (missing keys keep defaults) and migrate
  /// older schema versions if needed. Call once at boot before tasks start.
  void begin();

  // -- fast typed access (O(1), safe from any task) ------------------------
  float f(ParamId id) const { return values_[id]; }
  int i(ParamId id) const { return (int)values_[id]; }
  bool b(ParamId id) const { return values_[id] != 0.0f; }
  String s(StrParamId id) const;

  const ParamMeta& meta(ParamId id) const;
  const StrParamMeta& strMeta(StrParamId id) const;

  // -- key-based access (web API) ------------------------------------------
  /// Apply a value by dotted key. Clamps to min/max. Returns false if the
  /// key is unknown or the value is invalid.
  bool applyByKey(const char* key, float value);
  bool applyStringByKey(const char* key, const char* value);
  bool getByKey(const char* key, float& out) const;

  // -- persistence ----------------------------------------------------------
  void save();          ///< persist all current values to NVS
  void revert();        ///< reload last-saved values into RAM
  void factoryReset();  ///< restore defaults in RAM and NVS
  bool dirty() const { return dirty_; }

  // -- JSON -----------------------------------------------------------------
  /// Serialize current values (and schema version) for export/backup.
  void exportJson(JsonDocument& doc) const;
  /// Import values from a previously exported document. Unknown keys are
  /// ignored; values are clamped. Returns number of applied parameters.
  int importJson(const JsonDocument& doc);
  /// Serialize the full parameter schema (metadata) for the web UI.
  void schemaJson(JsonDocument& doc) const;

  /// Monotonic counter bumped on every applied change, so control tasks
  /// can cheaply detect "configuration changed" and refresh cached configs.
  uint32_t revision() const { return revision_; }

  /// FNV-1a hash used to derive short NVS keys from dotted parameter keys.
  static uint32_t keyHash(const char* key);

 private:
  void loadFromNvs();

  float values_[PARAM_COUNT];
  String strValues_[SPARAM_COUNT];
  mutable SemaphoreHandle_t strMutex_ = nullptr;
  volatile uint32_t revision_ = 0;
  bool dirty_ = false;
};

extern ConfigRegistry config;

}  // namespace vcm
