// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_lane.pb.h>

#include <cstdint>

namespace carla {
namespace osi {

/// Maps CARLA Lane::LaneType flags to OSI Lane classification.
/// Accepts int32_t to avoid pulling in carla/road/Lane.h.
struct LaneClassification {

  // CARLA Lane::LaneType bit values
  static constexpr int32_t kDriving   = 0x1 << 1;
  static constexpr int32_t kSidewalk  = 0x1 << 5;
  static constexpr int32_t kBiking    = 0x1 << 4;
  static constexpr int32_t kParking   = 0x1 << 8;
  static constexpr int32_t kStop      = 0x1 << 2;
  static constexpr int32_t kShoulder  = 0x1 << 3;
  static constexpr int32_t kBorder    = 0x1 << 6;
  static constexpr int32_t kEntry     = 0x1 << 17;
  static constexpr int32_t kExit      = 0x1 << 18;
  static constexpr int32_t kOnRamp    = 0x1 << 20;
  static constexpr int32_t kOffRamp   = 0x1 << 19;

  static osi3::Lane::Classification::Type GetLaneType(
      int32_t lane_type,
      bool is_junction) {
    if (is_junction) {
      return osi3::Lane::Classification::TYPE_INTERSECTION;
    }
    if (lane_type & kDriving) {
      return osi3::Lane::Classification::TYPE_DRIVING;
    }
    return osi3::Lane::Classification::TYPE_NONDRIVING;
  }

  /// Map CARLA lane type to a more specific OSI subtype string for the
  /// Lane::Classification::subtype field (free-form in OSI 3.x).
  static const char* GetSubtype(int32_t lane_type) {
    if (lane_type & kSidewalk)  return "sidewalk";
    if (lane_type & kBiking)    return "biking";
    if (lane_type & kParking)   return "parking";
    if (lane_type & kStop)      return "stop";
    if (lane_type & kShoulder)  return "shoulder";
    if (lane_type & kEntry)     return "entry";
    if (lane_type & kExit)      return "exit";
    if (lane_type & kOnRamp)    return "on_ramp";
    if (lane_type & kOffRamp)   return "off_ramp";
    if (lane_type & kBorder)    return "border";
    if (lane_type & kDriving)   return "driving";
    return "other";
  }
};

/// Maps CARLA LaneMarking types and colors to OSI LaneBoundary classification.
/// Accepts enum values as uint8_t to avoid pulling in carla/road/element/LaneMarking.h.
struct LaneBoundaryClassification {

  /// Map CARLA LaneMarking::Type to OSI LaneBoundary type.
  /// Returns a pair: (primary type, secondary type for double lines).
  /// Secondary is TYPE_UNKNOWN if not a double line.
  static void GetBoundaryType(
      uint8_t marking_type,
      osi3::LaneBoundary::Classification::Type &primary,
      osi3::LaneBoundary::Classification::Type &secondary) {

    using T = osi3::LaneBoundary::Classification;
    secondary = T::TYPE_UNKNOWN;

    switch (marking_type) {
      case 0: // Other
        primary = T::TYPE_OTHER;
        break;
      case 1: // Broken
        primary = T::TYPE_DASHED_LINE;
        break;
      case 2: // Solid
        primary = T::TYPE_SOLID_LINE;
        break;
      case 3: // SolidSolid
        primary = T::TYPE_SOLID_LINE;
        secondary = T::TYPE_SOLID_LINE;
        break;
      case 4: // SolidBroken
        primary = T::TYPE_SOLID_LINE;
        secondary = T::TYPE_DASHED_LINE;
        break;
      case 5: // BrokenSolid
        primary = T::TYPE_DASHED_LINE;
        secondary = T::TYPE_SOLID_LINE;
        break;
      case 6: // BrokenBroken
        primary = T::TYPE_DASHED_LINE;
        secondary = T::TYPE_DASHED_LINE;
        break;
      case 7: // BottsDots
        primary = T::TYPE_BOTTS_DOTS;
        break;
      case 8: // Grass
        primary = T::TYPE_GRASS_EDGE;
        break;
      case 9: // Curb
        primary = T::TYPE_CURB;
        break;
      case 10: // None
      default:
        primary = T::TYPE_UNKNOWN;
        break;
    }
  }

  /// Map CARLA LaneMarking::Color to OSI LaneBoundary color.
  static osi3::LaneBoundary::Classification::Color GetColor(uint8_t color) {
    switch (color) {
      case 0: // Standard / White
        return osi3::LaneBoundary::Classification::COLOR_WHITE;
      case 1: // Blue
        return osi3::LaneBoundary::Classification::COLOR_BLUE;
      case 2: // Green
        return osi3::LaneBoundary::Classification::COLOR_GREEN;
      case 3: // Red
        return osi3::LaneBoundary::Classification::COLOR_RED;
      case 4: // Yellow
        return osi3::LaneBoundary::Classification::COLOR_YELLOW;
      case 5: // Other
      default:
        return osi3::LaneBoundary::Classification::COLOR_OTHER;
    }
  }
};

} // namespace osi
} // namespace carla
