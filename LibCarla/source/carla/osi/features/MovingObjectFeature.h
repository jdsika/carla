// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_object.pb.h>

#include <string>
#include <string_view>

namespace carla {
namespace osi {

/// Maps CARLA blueprint type_id strings to OSI MovingObject and
/// VehicleClassification types.
struct ActorClassification {

  /// Determine the OSI MovingObject::Type from a CARLA blueprint type_id.
  static osi3::MovingObject::Type GetMovingObjectType(std::string_view type_id) {
    if (type_id.substr(0, 7) == "vehicle") {
      return osi3::MovingObject::TYPE_VEHICLE;
    }
    if (type_id.substr(0, 6) == "walker") {
      return osi3::MovingObject::TYPE_PEDESTRIAN;
    }
    return osi3::MovingObject::TYPE_OTHER;
  }

  /// Determine the OSI VehicleClassification::Type from a CARLA blueprint
  /// type_id.  Only meaningful when GetMovingObjectType returns TYPE_VEHICLE.
  static osi3::MovingObject_VehicleClassification_Type GetVehicleType(
      std::string_view type_id) {
    // CARLA blueprint ids follow the pattern "vehicle.<make>.<model>"
    // We classify based on known substrings in the second segment.

    if (Contains(type_id, "bicycle") || Contains(type_id, "crossbike")) {
      return osi3::MovingObject_VehicleClassification_TYPE_BICYCLE;
    }
    if (Contains(type_id, "motorcycle") || Contains(type_id, "harley")
        || Contains(type_id, "kawasaki") || Contains(type_id, "yamaha")
        || Contains(type_id, "vespa")) {
      return osi3::MovingObject_VehicleClassification_TYPE_MOTORBIKE;
    }
    if (Contains(type_id, "firetruck") || Contains(type_id, "ambulance")) {
      return osi3::MovingObject_VehicleClassification_TYPE_HEAVY_TRUCK;
    }
    if (Contains(type_id, "truck") || Contains(type_id, "carlamotors.european_hgv")
        || Contains(type_id, "daf")) {
      return osi3::MovingObject_VehicleClassification_TYPE_HEAVY_TRUCK;
    }
    if (Contains(type_id, "bus")) {
      return osi3::MovingObject_VehicleClassification_TYPE_BUS;
    }
    if (Contains(type_id, "van") || Contains(type_id, "sprinter")) {
      return osi3::MovingObject_VehicleClassification_TYPE_DELIVERY_VAN;
    }
    // Default: passenger car
    return osi3::MovingObject_VehicleClassification_TYPE_MEDIUM_CAR;
  }

private:
  static bool Contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
  }
};

} // namespace osi
} // namespace carla
