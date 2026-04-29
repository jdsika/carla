// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_groundtruth.pb.h>

#include <cstdint>
#include <string>

namespace osi3 {
class MCAPTraceFileWriter;
} // namespace osi3

namespace carla {

namespace geom {
  class BoundingBox;
  class Location;
  class Rotation;
  class Transform;
  class Vector3D;
} // namespace geom

namespace osi {

/// Incrementally builds an osi3::GroundTruth message for a single simulation
/// frame.  Callers populate it by calling the Add* methods, then retrieve the
/// finished message with Build().
class GroundTruthBuilder {
public:
  GroundTruthBuilder();

  /// Reset the builder for a new frame.
  void Reset();

  /// Set the OSI interface version stamp.
  void SetVersion(uint32_t major, uint32_t minor, uint32_t patch);

  /// Set the simulation timestamp for this frame.
  void SetTimestamp(double elapsed_seconds);

  /// Set the host vehicle (ego) actor ID.
  void SetHostVehicleId(uint64_t actor_id);

  /// Set the OpenDRIVE map reference string.
  void SetMapReference(const std::string &opendrive_name);

  /// Add a moving object (vehicle or pedestrian).
  void AddMovingObject(
      uint64_t id,
      const std::string &type_id,
      const geom::Transform &transform,
      const geom::Vector3D &velocity,
      const geom::Vector3D &acceleration,
      const geom::Vector3D &angular_velocity,
      const geom::Vector3D &bbox_extent,
      const geom::Location &bbox_offset);

  /// Add a moving object with extended vehicle attributes.
  /// @param light_state   CARLA VehicleLightState bitflags (uint32_t).
  /// @param num_wheels    Number of wheels (0 to skip).
  /// @param wheel_radius  Wheel radius in meters (0 to skip).
  void AddMovingObjectExtended(
      uint64_t id,
      const std::string &type_id,
      const geom::Transform &transform,
      const geom::Vector3D &velocity,
      const geom::Vector3D &acceleration,
      const geom::Vector3D &angular_velocity,
      const geom::Vector3D &bbox_extent,
      const geom::Location &bbox_offset,
      uint32_t light_state,
      uint32_t num_wheels,
      float wheel_radius);

  /// Add a stationary object (building, barrier, pole, etc.).
  /// @param label  The underlying uint8_t of rpc::CityObjectLabel.
  void AddStationaryObject(
      uint64_t id,
      uint8_t label,
      const std::string &name,
      const geom::Transform &transform,
      const geom::Vector3D &bbox_extent);

  /// Add a traffic light with its current state.
  /// @param state_value  The underlying uint8_t of rpc::TrafficLightState.
  void AddTrafficLight(
      uint64_t id,
      const geom::Transform &transform,
      uint8_t state_value);

  /// Add a traffic sign.
  /// @param type       The OpenDRIVE sign type code (e.g. "274" for speed limit).
  /// @param value      The sign's value (e.g. speed limit in km/h).
  void AddTrafficSign(
      uint64_t id,
      const std::string &type,
      double value,
      const geom::Transform &transform,
      const geom::Vector3D &bbox_extent);

  /// Add a lane to the ground truth.
  /// @param id            Unique lane identifier.
  /// @param lane_type     The underlying int32_t of carla::road::Lane::LaneType.
  /// @param is_junction   True if the lane is inside a junction.
  /// @param centerline    Array of 3D points {{x,y,z},...} in UE coordinates.
  /// @param num_points    Number of centerline points.
  /// @param left_boundary_id   OSI ID of the left lane boundary (0 to skip).
  /// @param right_boundary_id  OSI ID of the right lane boundary (0 to skip).
  void AddLane(
      uint64_t id,
      int32_t lane_type,
      bool is_junction,
      const float *centerline_xyz,
      uint32_t num_points,
      uint64_t left_boundary_id,
      uint64_t right_boundary_id);

  /// Add a lane boundary to the ground truth.
  /// @param id             Unique boundary identifier.
  /// @param marking_type   The underlying uint8_t of LaneMarking::Type.
  /// @param marking_color  The underlying uint8_t of LaneMarking::Color.
  /// @param points_xyz     Array of 3D points {{x,y,z},...} in UE coordinates.
  /// @param num_points     Number of boundary points.
  void AddLaneBoundary(
      uint64_t id,
      uint8_t marking_type,
      uint8_t marking_color,
      const float *points_xyz,
      uint32_t num_points);

  /// Set the environmental conditions from weather parameters.
  void SetEnvironment(
      float precipitation,
      float fog_density,
      float sun_altitude_angle,
      float sun_azimuth_angle);

  /// Consume the built GroundTruth message.  The builder is left in a
  /// moved-from state; call Reset() before reusing.
  osi3::GroundTruth Build();

  /// Build the GroundTruth and write it directly to an MCAPRecorder.
  /// Convenience method so callers don't need to handle osi3::GroundTruth.
  void BuildAndWrite(class MCAPRecorder &recorder);

private:
  osi3::GroundTruth gt_;
};

} // namespace osi
} // namespace carla
