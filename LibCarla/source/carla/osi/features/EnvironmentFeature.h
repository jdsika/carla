// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <osi_environment.pb.h>

#include <cstdint>

namespace carla {
namespace osi {

/// Converts CARLA weather values to OSI EnvironmentalConditions.
/// Accepts plain floats to avoid pulling in carla/rpc/WeatherParameters.h
/// (which drags in MsgPack → Boost).
struct EnvironmentConverter {

  /// Populate an OSI EnvironmentalConditions message from CARLA weather values.
  static void ToOSI(
      float precipitation,
      float fog_density,
      float sun_altitude_angle,
      float sun_azimuth_angle,
      osi3::EnvironmentalConditions &out) {

    out.set_precipitation(MapPrecipitation(precipitation));
    out.set_fog(MapFog(fog_density));
    out.set_ambient_illumination(MapIllumination(sun_altitude_angle));

    auto *time_of_day = out.mutable_time_of_day();
    time_of_day->set_seconds_since_midnight(
        SunAngleToSecondsFromMidnight(sun_altitude_angle, sun_azimuth_angle));
  }

private:
  /// Map CARLA precipitation (0–100) to OSI Precipitation enum.
  static osi3::EnvironmentalConditions::Precipitation MapPrecipitation(float value) {
    if (value < 1.0f)  return osi3::EnvironmentalConditions::PRECIPITATION_NONE;
    if (value < 15.0f) return osi3::EnvironmentalConditions::PRECIPITATION_VERY_LIGHT;
    if (value < 30.0f) return osi3::EnvironmentalConditions::PRECIPITATION_LIGHT;
    if (value < 50.0f) return osi3::EnvironmentalConditions::PRECIPITATION_MODERATE;
    if (value < 75.0f) return osi3::EnvironmentalConditions::PRECIPITATION_HEAVY;
    if (value < 90.0f) return osi3::EnvironmentalConditions::PRECIPITATION_VERY_HEAVY;
    return osi3::EnvironmentalConditions::PRECIPITATION_EXTREME;
  }

  /// Map CARLA fog_density (0–100) to OSI Fog enum.
  static osi3::EnvironmentalConditions::Fog MapFog(float density) {
    if (density < 1.0f)  return osi3::EnvironmentalConditions::FOG_EXCELLENT_VISIBILITY;
    if (density < 10.0f) return osi3::EnvironmentalConditions::FOG_GOOD_VISIBILITY;
    if (density < 25.0f) return osi3::EnvironmentalConditions::FOG_MODERATE_VISIBILITY;
    if (density < 40.0f) return osi3::EnvironmentalConditions::FOG_POOR_VISIBILITY;
    if (density < 55.0f) return osi3::EnvironmentalConditions::FOG_MIST;
    if (density < 70.0f) return osi3::EnvironmentalConditions::FOG_LIGHT;
    if (density < 85.0f) return osi3::EnvironmentalConditions::FOG_THICK;
    return osi3::EnvironmentalConditions::FOG_DENSE;
  }

  /// Map sun altitude to OSI AmbientIllumination enum.
  static osi3::EnvironmentalConditions::AmbientIllumination MapIllumination(
      float sun_altitude) {
    if (sun_altitude < -18.0f) return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL1;
    if (sun_altitude < -12.0f) return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL2;
    if (sun_altitude < -6.0f)  return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL3;
    if (sun_altitude < 0.0f)   return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL4;
    if (sun_altitude < 6.0f)   return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL5;
    if (sun_altitude < 20.0f)  return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL6;
    if (sun_altitude < 40.0f)  return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL7;
    if (sun_altitude < 60.0f)  return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL8;
    return osi3::EnvironmentalConditions::AMBIENT_ILLUMINATION_LEVEL9;
  }

  /// Rough estimate of time-of-day from sun angles.
  static uint32_t SunAngleToSecondsFromMidnight(
      float altitude,
      float azimuth) {
    (void)altitude;
    float hour = 12.0f + (azimuth - 180.0f) / 15.0f;
    if (hour < 0.0f) hour += 24.0f;
    if (hour >= 24.0f) hour -= 24.0f;
    return static_cast<uint32_t>(hour * 3600.0f);
  }
};

} // namespace osi
} // namespace carla
