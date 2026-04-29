// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_object.pb.h>

#include <cstdint>

namespace carla {
namespace osi {

/// Maps CARLA VehicleLightState bitflags to OSI light state fields.
/// Accepts uint32_t to avoid pulling in carla/rpc/VehicleLightState.h.
/// Bit positions match rpc::VehicleLightState::LightState.
struct VehicleLightConverter {

  // CARLA VehicleLightState bit positions
  static constexpr uint32_t kPosition     = 1u << 0;
  static constexpr uint32_t kLowBeam      = 1u << 1;
  static constexpr uint32_t kHighBeam     = 1u << 2;
  static constexpr uint32_t kBrake        = 1u << 3;
  static constexpr uint32_t kRightBlinker = 1u << 4;
  static constexpr uint32_t kLeftBlinker  = 1u << 5;
  static constexpr uint32_t kReverse      = 1u << 6;
  static constexpr uint32_t kFog          = 1u << 7;
  static constexpr uint32_t kInterior     = 1u << 8;
  static constexpr uint32_t kSpecial1     = 1u << 9;
  static constexpr uint32_t kSpecial2     = 1u << 10;

  using GenericState = osi3::MovingObject_VehicleClassification_LightState_GenericLightState;
  using BrakeState = osi3::MovingObject_VehicleClassification_LightState_BrakeLightState;
  using IndicatorState = osi3::MovingObject_VehicleClassification_LightState_IndicatorState;

  static void ToOSI(
      uint32_t light_flags,
      osi3::MovingObject_VehicleClassification_LightState &out) {

    // Head lights (low beam)
    out.set_head_light(IsSet(light_flags, kLowBeam)
        ? GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON
        : GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);

    // High beam
    out.set_high_beam(IsSet(light_flags, kHighBeam)
        ? GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON
        : GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);

    // Brake light
    if (IsSet(light_flags, kBrake)) {
      out.set_brake_light_state(
          BrakeState::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL);
    } else {
      out.set_brake_light_state(
          BrakeState::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OFF);
    }

    // Indicators (turn signals)
    bool right = IsSet(light_flags, kRightBlinker);
    bool left  = IsSet(light_flags, kLeftBlinker);
    if (right && left) {
      out.set_indicator_state(
          IndicatorState::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_WARNING);
    } else if (right) {
      out.set_indicator_state(
          IndicatorState::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_RIGHT);
    } else if (left) {
      out.set_indicator_state(
          IndicatorState::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_LEFT);
    } else {
      out.set_indicator_state(
          IndicatorState::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OFF);
    }

    // Reversing light
    out.set_reversing_light(IsSet(light_flags, kReverse)
        ? GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON
        : GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);

    // Fog light (front)
    out.set_front_fog_light(IsSet(light_flags, kFog)
        ? GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON
        : GenericState::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
  }

private:
  static bool IsSet(uint32_t flags, uint32_t bit) {
    return (flags & bit) != 0;
  }
};

} // namespace osi
} // namespace carla
