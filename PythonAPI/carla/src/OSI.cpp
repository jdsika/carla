// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PythonAPI.h>

#include <carla/osi/converters/GroundTruthBuilder.h>
#include <carla/osi/io/MCAPRecorder.h>

#include <vector>

static void OpenRecorder(
    carla::osi::MCAPRecorder &self,
    const std::string &path,
    const std::string &description) {
  carla::PythonUtil::ReleaseGIL unlock;
  self.Open(path, description);
}

static void CloseRecorder(carla::osi::MCAPRecorder &self) {
  carla::PythonUtil::ReleaseGIL unlock;
  self.Close();
}

static void BuilderBuildAndWrite(
    carla::osi::GroundTruthBuilder &self,
    carla::osi::MCAPRecorder &recorder) {
  carla::PythonUtil::ReleaseGIL unlock;
  self.BuildAndWrite(recorder);
}

/// Helper: extract a flat float list from a Python list of carla.Location or
/// 3-tuples, returning {x0,y0,z0, x1,y1,z1, ...}.
static std::vector<float> ExtractPoints(const boost::python::list &py_points) {
  namespace bp = boost::python;
  const auto n = bp::len(py_points);
  std::vector<float> pts;
  pts.reserve(static_cast<size_t>(n) * 3);
  for (bp::ssize_t i = 0; i < n; ++i) {
    bp::object pt = py_points[i];
    pts.push_back(bp::extract<float>(pt.attr("x")));
    pts.push_back(bp::extract<float>(pt.attr("y")));
    pts.push_back(bp::extract<float>(pt.attr("z")));
  }
  return pts;
}

static void BuilderAddLane(
    carla::osi::GroundTruthBuilder &self,
    uint64_t id,
    int32_t lane_type,
    bool is_junction,
    const boost::python::list &centerline,
    uint64_t left_boundary_id,
    uint64_t right_boundary_id) {
  auto pts = ExtractPoints(centerline);
  self.AddLane(id, lane_type, is_junction,
               pts.data(), static_cast<uint32_t>(pts.size() / 3),
               left_boundary_id, right_boundary_id);
}

static void BuilderAddLaneBoundary(
    carla::osi::GroundTruthBuilder &self,
    uint64_t id,
    uint8_t marking_type,
    uint8_t marking_color,
    const boost::python::list &points) {
  auto pts = ExtractPoints(points);
  self.AddLaneBoundary(id, marking_type, marking_color,
                       pts.data(), static_cast<uint32_t>(pts.size() / 3));
}

void export_osi() {
  using namespace boost::python;
  namespace co = carla::osi;

  class_<co::MCAPRecorder, boost::noncopyable>(
      "OsiRecorder",
      "Records OSI GroundTruth messages to an MCAP trace file.\n"
      "\n"
      "Usage:\n"
      "    recorder = carla.OsiRecorder()\n"
      "    recorder.open('output.mcap')\n"
      "    # ... per frame: use OsiGroundTruthBuilder to populate and write\n"
      "    recorder.close()\n",
      init<>())
    .def("open", &OpenRecorder,
         (arg("path"), arg("description") = ""),
         "Open an MCAP file for recording.")
    .def("close", &CloseRecorder,
         "Close the MCAP file and flush all data.")
    .def("is_open", &co::MCAPRecorder::IsOpen,
         "Whether the recorder is currently recording.")
    .add_property("frame_count", &co::MCAPRecorder::GetFrameCount,
         "Number of frames written so far.")
  ;

  class_<co::GroundTruthBuilder>(
      "OsiGroundTruthBuilder",
      "Incrementally builds an OSI GroundTruth message for a single frame.\n"
      "\n"
      "Populate with set_*/add_* methods, then call write_to(recorder) to\n"
      "serialize and write the frame to an OsiRecorder.\n",
      init<>())
    .def("reset", &co::GroundTruthBuilder::Reset,
         "Clear the builder for a new frame.")
    .def("set_timestamp", &co::GroundTruthBuilder::SetTimestamp,
         (arg("elapsed_seconds")),
         "Set the simulation timestamp (seconds since episode start).")
    .def("set_host_vehicle_id", &co::GroundTruthBuilder::SetHostVehicleId,
         (arg("actor_id")),
         "Set the ego-vehicle actor ID.")
    .def("set_map_reference", &co::GroundTruthBuilder::SetMapReference,
         (arg("opendrive_name")),
         "Set the OpenDRIVE map reference string.")
    .def("add_moving_object", &co::GroundTruthBuilder::AddMovingObject,
         (arg("id"), arg("type_id"), arg("transform"),
          arg("velocity"), arg("acceleration"),
          arg("angular_velocity"), arg("bbox_extent"),
          arg("bbox_offset")),
         "Add a vehicle or pedestrian to the ground truth.")
    .def("add_moving_object_extended", &co::GroundTruthBuilder::AddMovingObjectExtended,
         (arg("id"), arg("type_id"), arg("transform"),
          arg("velocity"), arg("acceleration"),
          arg("angular_velocity"), arg("bbox_extent"),
          arg("bbox_offset"), arg("light_state"),
          arg("num_wheels"), arg("wheel_radius")),
         "Add a vehicle with extended attributes (light state, wheel info).")
    .def("add_stationary_object", &co::GroundTruthBuilder::AddStationaryObject,
         (arg("id"), arg("label"), arg("name"),
          arg("transform"), arg("bbox_extent")),
         "Add a stationary object (building, barrier, pole, etc.).")
    .def("add_traffic_light", &co::GroundTruthBuilder::AddTrafficLight,
         (arg("id"), arg("transform"), arg("state_value")),
         "Add a traffic light with its current state (uint8 value of TrafficLightState).")
    .def("add_traffic_sign", &co::GroundTruthBuilder::AddTrafficSign,
         (arg("id"), arg("type"), arg("value"),
          arg("transform"), arg("bbox_extent")),
         "Add a traffic sign (OpenDRIVE type code and value).")
    .def("add_lane", &BuilderAddLane,
         (arg("id"), arg("lane_type"), arg("is_junction"),
          arg("centerline"), arg("left_boundary_id") = 0u,
          arg("right_boundary_id") = 0u),
         "Add a lane with centerline points (list of carla.Location).")
    .def("add_lane_boundary", &BuilderAddLaneBoundary,
         (arg("id"), arg("marking_type"), arg("marking_color"),
          arg("points")),
         "Add a lane boundary with geometry (list of carla.Location).")
    .def("set_environment", &co::GroundTruthBuilder::SetEnvironment,
         (arg("precipitation"), arg("fog_density"),
          arg("sun_altitude_angle"), arg("sun_azimuth_angle")),
         "Set environmental conditions from weather values.")
    .def("write_to", &BuilderBuildAndWrite,
         (arg("recorder")),
         "Build the GroundTruth and write it to the given OsiRecorder.")
  ;
}
