// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"

#include <osi_common.pb.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace carla {
namespace osi {

/// Converts between CARLA's coordinate system (UE5, left-handed, Z-up,
/// meters, degrees for rotations) and OSI's coordinate system (ISO 8855,
/// right-handed, Z-up, meters, radians for rotations).
///
/// The conversion flips the Y-axis and converts angles from degrees to
/// radians with appropriate sign changes.
struct CoordinateTransform {

  static constexpr double kDegToRad = M_PI / 180.0;

  /// Convert a CARLA position to an OSI Vector3d (position).
  static void ToOSI(const geom::Location &in, osi3::Vector3d &out) {
    out.set_x(static_cast<double>(in.x));
    out.set_y(static_cast<double>(-in.y));
    out.set_z(static_cast<double>(in.z));
  }

  /// Convert a CARLA velocity/acceleration vector to an OSI Vector3d.
  static void ToOSI(const geom::Vector3D &in, osi3::Vector3d &out) {
    out.set_x(static_cast<double>(in.x));
    out.set_y(static_cast<double>(-in.y));
    out.set_z(static_cast<double>(in.z));
  }

  /// Convert a CARLA rotation (degrees) to an OSI Orientation3d (radians).
  /// CARLA: pitch (Y-axis), yaw (Z-axis), roll (X-axis) in degrees.
  /// OSI:   roll (X-axis), pitch (Y-axis), yaw (Z-axis) in radians (ISO 8855).
  static void ToOSI(const geom::Rotation &in, osi3::Orientation3d &out) {
    out.set_roll(static_cast<double>(in.roll) * kDegToRad);
    out.set_pitch(static_cast<double>(-in.pitch) * kDegToRad);
    out.set_yaw(static_cast<double>(-in.yaw) * kDegToRad);
  }

  /// Convert a CARLA angular velocity vector (degrees/s) to an OSI
  /// Orientation3d rate (radians/s).
  static void AngularVelocityToOSI(
      const geom::Vector3D &in,
      osi3::Orientation3d &out) {
    out.set_roll(static_cast<double>(in.x) * kDegToRad);
    out.set_pitch(static_cast<double>(-in.y) * kDegToRad);
    out.set_yaw(static_cast<double>(-in.z) * kDegToRad);
  }

  /// Convert a simulation timestamp (seconds as double) to an OSI Timestamp.
  static void ToOSI(double seconds, osi3::Timestamp &out) {
    auto secs = static_cast<int64_t>(seconds);
    auto nanos = static_cast<uint32_t>((seconds - static_cast<double>(secs)) * 1e9);
    out.set_seconds(secs);
    out.set_nanos(nanos);
  }

  /// Convert a CARLA bounding box extent (half-size) to an OSI Dimension3d.
  /// CARLA stores extent as half-sizes; OSI expects full dimensions.
  static void ExtentToOSIDimension(
      const geom::Vector3D &extent,
      osi3::Dimension3d &out) {
    out.set_length(static_cast<double>(extent.x) * 2.0);
    out.set_width(static_cast<double>(extent.y) * 2.0);
    out.set_height(static_cast<double>(extent.z) * 2.0);
  }
};

} // namespace osi
} // namespace carla
