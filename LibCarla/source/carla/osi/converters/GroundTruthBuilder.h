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

namespace rpc {
  enum class TrafficLightState : uint8_t;
  class WeatherParameters;
} // namespace rpc

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

  /// Add a traffic light with its current state.
  void AddTrafficLight(
      uint64_t id,
      const geom::Transform &transform,
      rpc::TrafficLightState state);

  /// Set the environmental conditions from CARLA weather.
  void SetEnvironment(const rpc::WeatherParameters &weather);

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
