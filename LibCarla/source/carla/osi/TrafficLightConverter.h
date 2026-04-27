// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/rpc/TrafficLightState.h"

#include <osi_trafficlight.pb.h>

namespace carla {
namespace osi {

/// Converts CARLA traffic light states to OSI TrafficLight messages.
struct TrafficLightConverter {

  /// Map a CARLA TrafficLightState enum to an OSI traffic light color.
  static osi3::TrafficLight_Classification_Color ToOSIColor(
      rpc::TrafficLightState state) {
    switch (state) {
      case rpc::TrafficLightState::Red:
        return osi3::TrafficLight_Classification_Color_COLOR_RED;
      case rpc::TrafficLightState::Yellow:
        return osi3::TrafficLight_Classification_Color_COLOR_YELLOW;
      case rpc::TrafficLightState::Green:
        return osi3::TrafficLight_Classification_Color_COLOR_GREEN;
      case rpc::TrafficLightState::Off:
      case rpc::TrafficLightState::Unknown:
      default:
        return osi3::TrafficLight_Classification_Color_COLOR_UNKNOWN;
    }
  }

  /// Map a CARLA TrafficLightState enum to an OSI traffic light mode.
  static osi3::TrafficLight_Classification_Mode ToOSIMode(
      rpc::TrafficLightState state) {
    switch (state) {
      case rpc::TrafficLightState::Red:
      case rpc::TrafficLightState::Yellow:
      case rpc::TrafficLightState::Green:
        return osi3::TrafficLight_Classification_Mode_MODE_CONSTANT;
      case rpc::TrafficLightState::Off:
        return osi3::TrafficLight_Classification_Mode_MODE_OFF;
      case rpc::TrafficLightState::Unknown:
      default:
        return osi3::TrafficLight_Classification_Mode_MODE_UNKNOWN;
    }
  }
};

} // namespace osi
} // namespace carla
