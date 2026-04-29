// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/osi/converters/GroundTruthBuilder.h"

#include "carla/osi/features/MovingObjectFeature.h"
#include "carla/osi/features/TrafficLightFeature.h"
#include "carla/osi/features/TrafficSignFeature.h"
#include "carla/osi/features/EnvironmentFeature.h"
#include "carla/osi/features/StationaryObjectFeature.h"
#include "carla/osi/features/VehicleLightFeature.h"
#include "carla/osi/io/MCAPRecorder.h"
#include "carla/osi/utils/CoordinateTransform.h"

#include "carla/geom/BoundingBox.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"

namespace carla {
namespace osi {

GroundTruthBuilder::GroundTruthBuilder() {
  SetVersion(3, 8, 0);
}

void GroundTruthBuilder::Reset() {
  gt_.Clear();
  SetVersion(3, 8, 0);
}

void GroundTruthBuilder::SetVersion(
    uint32_t major,
    uint32_t minor,
    uint32_t patch) {
  auto *v = gt_.mutable_version();
  v->set_version_major(major);
  v->set_version_minor(minor);
  v->set_version_patch(patch);
}

void GroundTruthBuilder::SetTimestamp(double elapsed_seconds) {
  CoordinateTransform::ToOSI(elapsed_seconds, *gt_.mutable_timestamp());
}

void GroundTruthBuilder::SetHostVehicleId(uint64_t actor_id) {
  gt_.mutable_host_vehicle_id()->set_value(actor_id);
}

void GroundTruthBuilder::SetMapReference(const std::string &opendrive_name) {
  gt_.set_map_reference(opendrive_name);
}

void GroundTruthBuilder::AddMovingObject(
    uint64_t id,
    const std::string &type_id,
    const geom::Transform &transform,
    const geom::Vector3D &velocity,
    const geom::Vector3D &acceleration,
    const geom::Vector3D &angular_velocity,
    const geom::Vector3D &bbox_extent,
    const geom::Location &bbox_offset) {

  AddMovingObjectExtended(
      id, type_id, transform, velocity, acceleration,
      angular_velocity, bbox_extent, bbox_offset,
      /*light_state=*/0, /*num_wheels=*/0, /*wheel_radius=*/0.0f);
}

void GroundTruthBuilder::AddMovingObjectExtended(
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
    float wheel_radius) {

  auto *obj = gt_.add_moving_object();

  // Identifier
  obj->mutable_id()->set_value(id);

  // Type classification
  auto osi_type = ActorClassification::GetMovingObjectType(type_id);
  obj->set_type(osi_type);

  if (osi_type == osi3::MovingObject::TYPE_VEHICLE) {
    auto *vc = obj->mutable_vehicle_classification();
    vc->set_type(ActorClassification::GetVehicleType(type_id));
    vc->set_has_trailer(false);

    // Vehicle light state
    if (light_state != 0) {
      VehicleLightConverter::ToOSI(light_state, *vc->mutable_light_state());
    }

    // Vehicle attributes
    if (num_wheels > 0 || wheel_radius > 0.0f) {
      auto *attrs = obj->mutable_vehicle_attributes();
      if (num_wheels > 0) {
        attrs->set_number_wheels(num_wheels);
      }
      if (wheel_radius > 0.0f) {
        attrs->set_radius_wheel(static_cast<double>(wheel_radius));
      }
    }
  }

  // Base parameters
  auto *base = obj->mutable_base();

  // Position (bounding box center in world coordinates)
  auto world_center = transform;
  world_center.location += geom::Location(
      transform.GetForwardVector() * bbox_offset.x +
      transform.GetRightVector() * bbox_offset.y +
      transform.GetUpVector() * bbox_offset.z);
  CoordinateTransform::ToOSI(world_center.location, *base->mutable_position());

  // Orientation
  CoordinateTransform::ToOSI(transform.rotation, *base->mutable_orientation());

  // Velocity
  CoordinateTransform::ToOSI(velocity, *base->mutable_velocity());

  // Acceleration
  CoordinateTransform::ToOSI(acceleration, *base->mutable_acceleration());

  // Angular velocity → orientation rate
  CoordinateTransform::AngularVelocityToOSI(
      angular_velocity, *base->mutable_orientation_rate());

  // Bounding box dimensions
  CoordinateTransform::ExtentToOSIDimension(
      bbox_extent, *base->mutable_dimension());

  // Model reference (the CARLA blueprint type_id)
  obj->set_model_reference(type_id);
}

void GroundTruthBuilder::AddStationaryObject(
    uint64_t id,
    uint8_t label,
    const std::string &name,
    const geom::Transform &transform,
    const geom::Vector3D &bbox_extent) {

  auto *obj = gt_.add_stationary_object();

  obj->mutable_id()->set_value(id);

  auto *classification = obj->mutable_classification();
  classification->set_type(StationaryObjectClassification::GetType(label));

  auto *base = obj->mutable_base();
  CoordinateTransform::ToOSI(transform.location, *base->mutable_position());
  CoordinateTransform::ToOSI(transform.rotation, *base->mutable_orientation());
  CoordinateTransform::ExtentToOSIDimension(
      bbox_extent, *base->mutable_dimension());

  obj->set_model_reference(name);
}

void GroundTruthBuilder::AddTrafficLight(
    uint64_t id,
    const geom::Transform &transform,
    uint8_t state) {

  auto *tl = gt_.add_traffic_light();

  tl->mutable_id()->set_value(id);

  auto *base = tl->mutable_base();
  CoordinateTransform::ToOSI(transform.location, *base->mutable_position());
  CoordinateTransform::ToOSI(transform.rotation, *base->mutable_orientation());

  auto *classification = tl->mutable_classification();
  classification->set_color(TrafficLightConverter::ToOSIColor(state));
  classification->set_mode(TrafficLightConverter::ToOSIMode(state));
}

void GroundTruthBuilder::AddTrafficSign(
    uint64_t id,
    const std::string &type,
    double value,
    const geom::Transform &transform,
    const geom::Vector3D &bbox_extent) {

  auto *sign = gt_.add_traffic_sign();

  sign->mutable_id()->set_value(id);

  auto *main_sign = sign->mutable_main_sign();
  auto *base = main_sign->mutable_base();
  CoordinateTransform::ToOSI(transform.location, *base->mutable_position());
  CoordinateTransform::ToOSI(transform.rotation, *base->mutable_orientation());
  CoordinateTransform::ExtentToOSIDimension(
      bbox_extent, *base->mutable_dimension());

  auto *classification = main_sign->mutable_classification();
  classification->set_type(TrafficSignClassification::GetMainSignType(type));

  double speed_ms = TrafficSignClassification::GetSpeedLimitValue(type, value);
  if (speed_ms > 0.0) {
    auto *speed_limit = classification->mutable_value();
    speed_limit->set_value(speed_ms);
  }
}

void GroundTruthBuilder::SetEnvironment(
    float precipitation,
    float fog_density,
    float sun_altitude_angle,
    float sun_azimuth_angle) {
  EnvironmentConverter::ToOSI(
      precipitation,
      fog_density,
      sun_altitude_angle,
      sun_azimuth_angle,
      *gt_.mutable_environmental_conditions());
}

osi3::GroundTruth GroundTruthBuilder::Build() {
  return std::move(gt_);
}

void GroundTruthBuilder::BuildAndWrite(MCAPRecorder &recorder) {
  recorder.WriteFrame(gt_);
  gt_.Clear();
  SetVersion(3, 8, 0);
}

} // namespace osi
} // namespace carla
