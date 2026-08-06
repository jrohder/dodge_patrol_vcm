/**
 * @file version.h
 * @brief Firmware version metadata, injected by tools/version.py from git.
 */
#pragma once

#ifndef VCM_FW_VERSION
#define VCM_FW_VERSION "0.0.0-dev"
#endif
#ifndef VCM_GIT_COMMIT
#define VCM_GIT_COMMIT "unknown"
#endif
#ifndef VCM_BUILD_DATE
#define VCM_BUILD_DATE "unknown"
#endif

#define VCM_PROTOCOL_VERSION 1  ///< Nano <-> ESP32 UART protocol version
