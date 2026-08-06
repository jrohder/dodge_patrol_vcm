#include "config/config_registry.h"

#include <Preferences.h>

#include "services/logger.h"

namespace vcm {

ConfigRegistry config;

// Metadata tables generated from params.def -------------------------------
static const ParamMeta kParams[PARAM_COUNT] = {
#define PARAM(ID, key, name, units, type, def, mn, mx, level, flags, opts, desc) \
  {key, name, units, type, (float)(def), (float)(mn), (float)(mx), level,        \
   (uint8_t)(flags), opts, desc},
#define SPARAM(ID, key, name, def, level, flags, desc)
#include "config/params.def"
#undef PARAM
#undef SPARAM
};

static const StrParamMeta kStrParams[SPARAM_COUNT] = {
#define PARAM(ID, key, name, units, type, def, mn, mx, level, flags, opts, desc)
#define SPARAM(ID, key, name, def, level, flags, desc) \
  {key, name, def, level, (uint8_t)(flags), desc},
#include "config/params.def"
#undef PARAM
#undef SPARAM
};

static const char* kNamespace = "vcm_cfg";
static const char* kSchemaKey = "_schema";

// NVS keys are limited to 15 chars; use an FNV-1a hash of the dotted key.
uint32_t ConfigRegistry::keyHash(const char* key) {
  uint32_t h = 2166136261u;
  for (const char* p = key; *p; ++p) {
    h ^= (uint8_t)*p;
    h *= 16777619u;
  }
  return h;
}

static void nvsKeyFor(const char* key, char out[16]) {
  snprintf(out, 16, "p%08lx", (unsigned long)ConfigRegistry::keyHash(key));
}

void ConfigRegistry::begin() {
  strMutex_ = xSemaphoreCreateMutex();
  for (int i = 0; i < PARAM_COUNT; ++i) values_[i] = kParams[i].def;
  for (int i = 0; i < SPARAM_COUNT; ++i) strValues_[i] = kStrParams[i].def;
  loadFromNvs();
}

void ConfigRegistry::loadFromNvs() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) {
    LOGI("CONFIG", "No saved configuration; using factory defaults");
    return;
  }
  const uint16_t storedSchema = prefs.getUShort(kSchemaKey, 0);
  char nk[16];
  for (int i = 0; i < PARAM_COUNT; ++i) {
    nvsKeyFor(kParams[i].key, nk);
    if (prefs.isKey(nk)) {
      const float v = prefs.getFloat(nk, kParams[i].def);
      values_[i] = constrain(v, kParams[i].min, kParams[i].max);
    }
  }
  for (int i = 0; i < SPARAM_COUNT; ++i) {
    nvsKeyFor(kStrParams[i].key, nk);
    if (prefs.isKey(nk)) strValues_[i] = prefs.getString(nk, kStrParams[i].def);
  }
  prefs.end();

  if (storedSchema != 0 && storedSchema != SCHEMA_VERSION) {
    // Migration point: adjust/rename values here when SCHEMA_VERSION bumps.
    // Unknown keys are simply ignored, new keys keep their defaults.
    LOGW("CONFIG", "Migrated configuration schema v%u -> v%u", storedSchema,
         SCHEMA_VERSION);
    save();
  }
  LOGI("CONFIG", "Configuration loaded (schema v%u)", SCHEMA_VERSION);
  revision_++;
}

void ConfigRegistry::save() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) {
    LOGE("CONFIG", "Failed to open NVS for save");
    return;
  }
  prefs.putUShort(kSchemaKey, SCHEMA_VERSION);
  char nk[16];
  for (int i = 0; i < PARAM_COUNT; ++i) {
    nvsKeyFor(kParams[i].key, nk);
    prefs.putFloat(nk, values_[i]);
  }
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  for (int i = 0; i < SPARAM_COUNT; ++i) {
    nvsKeyFor(kStrParams[i].key, nk);
    prefs.putString(nk, strValues_[i]);
  }
  xSemaphoreGive(strMutex_);
  prefs.end();
  dirty_ = false;
  LOGI("CONFIG", "Configuration saved to NVS");
}

void ConfigRegistry::revert() {
  for (int i = 0; i < PARAM_COUNT; ++i) values_[i] = kParams[i].def;
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  for (int i = 0; i < SPARAM_COUNT; ++i) strValues_[i] = kStrParams[i].def;
  xSemaphoreGive(strMutex_);
  loadFromNvs();
  dirty_ = false;
  LOGI("CONFIG", "Configuration reverted to last saved values");
}

void ConfigRegistry::factoryReset() {
  Preferences prefs;
  if (prefs.begin(kNamespace, false)) {
    prefs.clear();
    prefs.end();
  }
  for (int i = 0; i < PARAM_COUNT; ++i) values_[i] = kParams[i].def;
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  for (int i = 0; i < SPARAM_COUNT; ++i) strValues_[i] = kStrParams[i].def;
  xSemaphoreGive(strMutex_);
  dirty_ = false;
  revision_++;
  LOGW("CONFIG", "FACTORY RESET - all configuration restored to defaults");
}

String ConfigRegistry::s(StrParamId id) const {
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  String v = strValues_[id];
  xSemaphoreGive(strMutex_);
  return v;
}

const ParamMeta& ConfigRegistry::meta(ParamId id) const { return kParams[id]; }
const StrParamMeta& ConfigRegistry::strMeta(StrParamId id) const {
  return kStrParams[id];
}

bool ConfigRegistry::applyByKey(const char* key, float value) {
  for (int i = 0; i < PARAM_COUNT; ++i) {
    if (strcmp(kParams[i].key, key) == 0) {
      if (isnan(value) || isinf(value)) return false;
      values_[i] = constrain(value, kParams[i].min, kParams[i].max);
      dirty_ = true;
      revision_++;
      return true;
    }
  }
  return false;
}

bool ConfigRegistry::applyStringByKey(const char* key, const char* value) {
  for (int i = 0; i < SPARAM_COUNT; ++i) {
    if (strcmp(kStrParams[i].key, key) == 0) {
      xSemaphoreTake(strMutex_, portMAX_DELAY);
      strValues_[i] = value;
      xSemaphoreGive(strMutex_);
      dirty_ = true;
      revision_++;
      return true;
    }
  }
  return false;
}

bool ConfigRegistry::getByKey(const char* key, float& out) const {
  for (int i = 0; i < PARAM_COUNT; ++i) {
    if (strcmp(kParams[i].key, key) == 0) {
      out = values_[i];
      return true;
    }
  }
  return false;
}

void ConfigRegistry::exportJson(JsonDocument& doc) const {
  doc["schema_version"] = SCHEMA_VERSION;
  JsonObject params = doc["params"].to<JsonObject>();
  for (int i = 0; i < PARAM_COUNT; ++i) {
    if (kParams[i].type == PT_FLOAT)
      params[kParams[i].key] = values_[i];
    else
      params[kParams[i].key] = (int)values_[i];
  }
  JsonObject strs = doc["strings"].to<JsonObject>();
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  for (int i = 0; i < SPARAM_COUNT; ++i) strs[kStrParams[i].key] = strValues_[i];
  xSemaphoreGive(strMutex_);
}

int ConfigRegistry::importJson(const JsonDocument& doc) {
  int applied = 0;
  JsonObjectConst params = doc["params"].as<JsonObjectConst>();
  for (JsonPairConst kv : params) {
    if (applyByKey(kv.key().c_str(), kv.value().as<float>())) applied++;
  }
  JsonObjectConst strs = doc["strings"].as<JsonObjectConst>();
  for (JsonPairConst kv : strs) {
    if (applyStringByKey(kv.key().c_str(), kv.value().as<const char*>()))
      applied++;
  }
  LOGI("CONFIG", "Imported %d parameters from JSON", applied);
  return applied;
}

static const char* levelName(ParamLevel l) {
  return l == PL_BASIC ? "basic" : (l == PL_ADVANCED ? "advanced" : "expert");
}

void ConfigRegistry::schemaJson(JsonDocument& doc) const {
  doc["schema_version"] = SCHEMA_VERSION;
  JsonArray arr = doc["params"].to<JsonArray>();
  for (int i = 0; i < PARAM_COUNT; ++i) {
    const ParamMeta& m = kParams[i];
    JsonObject o = arr.add<JsonObject>();
    o["key"] = m.key;
    o["name"] = m.name;
    o["units"] = m.units;
    o["type"] = m.type == PT_FLOAT   ? "float"
                : m.type == PT_INT   ? "int"
                : m.type == PT_BOOL  ? "bool"
                                     : "enum";
    o["def"] = m.def;
    o["min"] = m.min;
    o["max"] = m.max;
    o["level"] = levelName(m.level);
    o["danger"] = (m.flags & PF_DANGER) != 0;
    o["restart"] = (m.flags & PF_RESTART) != 0;
    if (m.type == PT_ENUM) o["options"] = m.options;
    o["desc"] = m.desc;
    o["value"] = (m.type == PT_FLOAT) ? values_[i] : (float)(int)values_[i];
  }
  JsonArray sarr = doc["strings"].to<JsonArray>();
  xSemaphoreTake(strMutex_, portMAX_DELAY);
  for (int i = 0; i < SPARAM_COUNT; ++i) {
    const StrParamMeta& m = kStrParams[i];
    JsonObject o = sarr.add<JsonObject>();
    o["key"] = m.key;
    o["name"] = m.name;
    o["def"] = m.def;
    o["level"] = levelName(m.level);
    o["danger"] = (m.flags & PF_DANGER) != 0;
    o["restart"] = (m.flags & PF_RESTART) != 0;
    o["desc"] = m.desc;
    o["value"] = strValues_[i];
  }
  xSemaphoreGive(strMutex_);
}

}  // namespace vcm
