/**
 * @file param.h
 * @brief Parameter metadata types for the configuration system.
 *
 * Every parameter has a name, description, units, default, minimum,
 * maximum and current value. The web UI generates its controls from
 * this metadata (served at /api/config/schema) so firmware and UI can
 * never drift apart.
 */
#pragma once

#include <cstdint>

namespace vcm {

enum ParamType : uint8_t { PT_FLOAT, PT_INT, PT_BOOL, PT_ENUM };
enum ParamLevel : uint8_t { PL_BASIC, PL_ADVANCED, PL_EXPERT };

enum ParamFlags : uint8_t {
  PF_NONE = 0,
  PF_DANGER = 1,   ///< UI shows a warning and requires confirmation
  PF_RESTART = 2,  ///< takes effect after reboot
};

/// Numeric parameter IDs, generated from params.def
enum ParamId : uint16_t {
#define PARAM(ID, key, name, units, type, def, mn, mx, level, flags, opts, desc) ID,
#define SPARAM(ID, key, name, def, level, flags, desc)
#include "config/params.def"
#undef PARAM
#undef SPARAM
  PARAM_COUNT
};

/// String parameter IDs, generated from params.def
enum StrParamId : uint16_t {
#define PARAM(ID, key, name, units, type, def, mn, mx, level, flags, opts, desc)
#define SPARAM(ID, key, name, def, level, flags, desc) ID,
#include "config/params.def"
#undef PARAM
#undef SPARAM
  SPARAM_COUNT
};

/// Compile-time metadata for a numeric parameter.
struct ParamMeta {
  const char* key;   ///< dotted key, e.g. "drive.max_acceleration"
  const char* name;  ///< human readable name
  const char* units;
  ParamType type;
  float def, min, max;
  ParamLevel level;
  uint8_t flags;
  const char* options;  ///< "A|B|C" for enums, "" otherwise
  const char* desc;
};

/// Compile-time metadata for a string parameter.
struct StrParamMeta {
  const char* key;
  const char* name;
  const char* def;
  ParamLevel level;
  uint8_t flags;
  const char* desc;
};

}  // namespace vcm
