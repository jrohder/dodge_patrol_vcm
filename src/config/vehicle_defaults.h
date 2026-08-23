/**
 * @file vehicle_defaults.h
 * @brief Factory steering calibration for this vehicle.
 *
 * Captured over USB on 2026-08-22 while the car was READY / commissioned
 * (Firgelli feedback ADC hard stops). Used when NVS is empty or was erased
 * after filling the 20 kB partition.
 */
#pragma once

namespace vcm {
namespace factory {

constexpr bool kSteeringCalValid = true;
constexpr float kSteerLeftAdc = 3738.0f;
constexpr float kSteerCenterAdc = 2048.0f;
constexpr float kSteerRightAdc = 358.0f;
constexpr bool kCommissioned = true;

}  // namespace factory
}  // namespace vcm
