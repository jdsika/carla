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

/// Maps CARLA CityObjectLabel values to OSI StationaryObject classification.
/// Accepts uint8_t to avoid pulling in carla/rpc/ObjectLabel.h.
/// Values match rpc::CityObjectLabel.
struct StationaryObjectClassification {

  static osi3::StationaryObject::Classification::Type GetType(uint8_t label) {
    switch (label) {
      case 1:  // Buildings
        return osi3::StationaryObject::Classification::TYPE_BUILDING;
      case 5:  // Poles
        return osi3::StationaryObject::Classification::TYPE_POLE;
      case 9:  // Vegetation
        return osi3::StationaryObject::Classification::TYPE_VEGETATION;
      case 2:  // Fences
      case 17: // GuardRail
        return osi3::StationaryObject::Classification::TYPE_BARRIER;
      case 3:  // Other
        return osi3::StationaryObject::Classification::TYPE_OTHER;
      case 8:  // Walls
        return osi3::StationaryObject::Classification::TYPE_WALL;
      case 15: // Bridge
        return osi3::StationaryObject::Classification::TYPE_BRIDGE;
      // Roads, Sidewalks, RoadLines, Ground, Water, RailTrack, Terrain, Static
      default:
        return osi3::StationaryObject::Classification::TYPE_OTHER;
    }
  }
};

} // namespace osi
} // namespace carla
