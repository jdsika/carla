// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_trafficsign.pb.h>

#include <string>
#include <string_view>

namespace carla {
namespace osi {

/// Maps CARLA/OpenDRIVE traffic sign types to OSI TrafficSign classification.
/// Accepts plain strings to avoid pulling in carla/road/Signal.h.
struct TrafficSignClassification {

  /// Map an OpenDRIVE sign type string to an OSI MainSign type.
  /// CARLA's Signal::GetType() returns the OpenDRIVE type code.
  static osi3::TrafficSign_MainSign_Classification_Type GetMainSignType(
      std::string_view type) {
    // OpenDRIVE type codes follow country-specific conventions.
    // Common DE (Germany) types used in CARLA maps:
    if (type == "206")  // Stop sign
      return osi3::TrafficSign_MainSign_Classification::TYPE_STOP;
    if (type == "205")  // Yield / Give way
      return osi3::TrafficSign_MainSign_Classification::TYPE_GIVE_WAY;
    if (type == "274")  // Speed limit
      return osi3::TrafficSign_MainSign_Classification::TYPE_SPEED_LIMIT_BEGIN;
    if (type == "278")  // End of speed limit
      return osi3::TrafficSign_MainSign_Classification::TYPE_SPEED_LIMIT_END;
    if (type == "306")  // Priority road
      return osi3::TrafficSign_MainSign_Classification::TYPE_RIGHT_OF_WAY_BEGIN;
    if (type == "307")  // End of priority road
      return osi3::TrafficSign_MainSign_Classification::TYPE_RIGHT_OF_WAY_END;
    if (type == "301")  // Right-of-way at next intersection
      return osi3::TrafficSign_MainSign_Classification::TYPE_RIGHT_OF_WAY_NEXT_INTERSECTION;
    if (type == "250")  // No vehicles
      return osi3::TrafficSign_MainSign_Classification::TYPE_DO_NOT_ENTER;
    if (type == "267")  // No entry
      return osi3::TrafficSign_MainSign_Classification::TYPE_DO_NOT_ENTER;
    if (type == "220")  // Roundabout
      return osi3::TrafficSign_MainSign_Classification::TYPE_ROUNDABOUT;
    if (type == "310" || type == "311")  // City begin / end
      return osi3::TrafficSign_MainSign_Classification::TYPE_TOWN_BEGIN;
    if (type == "350")  // Pedestrian crossing
      return osi3::TrafficSign_MainSign_Classification::TYPE_ZEBRA_CROSSING;

    return osi3::TrafficSign_MainSign_Classification::TYPE_OTHER;
  }

  /// Extract the speed limit value from a sign's value field.
  /// Returns 0 if not applicable.
  static double GetSpeedLimitValue(std::string_view type, double value) {
    if (type == "274" || type == "278") {
      // value is in km/h in OpenDRIVE, OSI expects m/s
      return value / 3.6;
    }
    return 0.0;
  }
};

} // namespace osi
} // namespace carla
