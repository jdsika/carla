// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_trafficlight.pb.h>

#include <cstdint>

namespace carla {
namespace osi {

/// Converts CARLA traffic light states to OSI TrafficLight messages.
/// Accepts uint8_t to avoid pulling in carla/rpc/TrafficLightState.h
/// (which drags in MsgPack → Boost). Values match rpc::TrafficLightState.
struct TrafficLightConverter {

  /// Map a CARLA TrafficLightState value to an OSI traffic light color.
  static osi3::TrafficLight::Classification::Color ToOSIColor(uint8_t state) {
    switch (state) {
      case 0: // Red
        return osi3::TrafficLight::Classification::COLOR_RED;
      case 1: // Yellow
        return osi3::TrafficLight::Classification::COLOR_YELLOW;
      case 2: // Green
        return osi3::TrafficLight::Classification::COLOR_GREEN;
      case 3: // Off
      case 4: // Unknown
      default:
        return osi3::TrafficLight::Classification::COLOR_UNKNOWN;
    }
  }

  /// Map a CARLA TrafficLightState value to an OSI traffic light mode.
  static osi3::TrafficLight::Classification::Mode ToOSIMode(uint8_t state) {
    switch (state) {
      case 0: // Red
      case 1: // Yellow
      case 2: // Green
        return osi3::TrafficLight::Classification::MODE_CONSTANT;
      case 3: // Off
        return osi3::TrafficLight::Classification::MODE_OFF;
      case 4: // Unknown
      default:
        return osi3::TrafficLight::Classification::MODE_UNKNOWN;
    }
  }
};

} // namespace osi
} // namespace carla
