// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/osi/MCAPRecorder.h"

#include "carla/Logging.h"

#include <osi-utilities/tracefile/writer/MCAPTraceFileWriter.h>
#include <osi_groundtruth.pb.h>

namespace carla {
namespace osi {

MCAPRecorder::MCAPRecorder()
  : writer_(std::make_unique<osi3::MCAPTraceFileWriter>()) {}

MCAPRecorder::~MCAPRecorder() {
  if (is_open_) {
    Close();
  }
}

MCAPRecorder::MCAPRecorder(MCAPRecorder &&other) noexcept
  : writer_(std::move(other.writer_)),
    is_open_(other.is_open_),
    channel_registered_(other.channel_registered_),
    frame_count_(other.frame_count_) {
  other.is_open_ = false;
  other.channel_registered_ = false;
  other.frame_count_ = 0;
}

MCAPRecorder &MCAPRecorder::operator=(MCAPRecorder &&other) noexcept {
  if (this != &other) {
    if (is_open_) {
      Close();
    }
    writer_ = std::move(other.writer_);
    is_open_ = other.is_open_;
    channel_registered_ = other.channel_registered_;
    frame_count_ = other.frame_count_;
    other.is_open_ = false;
    other.channel_registered_ = false;
    other.frame_count_ = 0;
  }
  return *this;
}

void MCAPRecorder::Open(
    const std::string &path,
    const std::string &description) {

  if (is_open_) {
    log_warning("OSI MCAPRecorder: already open, closing previous file first.");
    Close();
  }

  writer_ = std::make_unique<osi3::MCAPTraceFileWriter>();
  writer_->Open(path);

  // Add required OSI file-level metadata.
  auto metadata = osi3::MCAPTraceFileWriter::PrepareRequiredFileMetadata();
  if (!description.empty()) {
    metadata.metadata["description"] = description;
  }
  writer_->AddFileMetadata(metadata);

  // Register the GroundTruth channel.
  writer_->AddChannel(
      kGroundTruthTopic,
      osi3::GroundTruth::descriptor());
  channel_registered_ = true;

  is_open_ = true;
  frame_count_ = 0;

  log_info("OSI MCAPRecorder: opened ", path);
}

void MCAPRecorder::WriteFrame(const osi3::GroundTruth &ground_truth) {
  if (!is_open_) {
    log_warning("OSI MCAPRecorder: cannot write frame, recorder is not open.");
    return;
  }

  writer_->WriteMessage(ground_truth, kGroundTruthTopic);
  ++frame_count_;
}

void MCAPRecorder::Close() {
  if (!is_open_) {
    return;
  }

  writer_->Close();
  is_open_ = false;
  channel_registered_ = false;

  log_info("OSI MCAPRecorder: closed after ", frame_count_, " frames.");
}

bool MCAPRecorder::IsOpen() const {
  return is_open_;
}

} // namespace osi
} // namespace carla
