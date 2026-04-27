// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PythonAPI.h>

#include <carla/osi/GroundTruthBuilder.h>
#include <carla/osi/MCAPRecorder.h>

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
    .def("add_traffic_light", &co::GroundTruthBuilder::AddTrafficLight,
         (arg("id"), arg("transform"), arg("state")),
         "Add a traffic light with its current state.")
    .def("set_environment", &co::GroundTruthBuilder::SetEnvironment,
         (arg("weather")),
         "Set environmental conditions from a carla.WeatherParameters.")
    .def("write_to", &BuilderBuildAndWrite,
         (arg("recorder")),
         "Build the GroundTruth and write it to the given OsiRecorder.")
  ;
}
