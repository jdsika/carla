// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/osi/GroundTruthBuilder.h"

#include <memory>
#include <string>

namespace osi3 {
class MCAPTraceFileWriter;
} // namespace osi3

namespace carla {
namespace osi {

/// Records OSI GroundTruth messages to an MCAP trace file.
///
/// Usage:
///   MCAPRecorder recorder;
///   recorder.Open("output.mcap");
///   // Per frame:
///   recorder.WriteFrame(ground_truth_message);
///   // When done:
///   recorder.Close();
class MCAPRecorder {
public:
  MCAPRecorder();
  ~MCAPRecorder();

  MCAPRecorder(const MCAPRecorder &) = delete;
  MCAPRecorder &operator=(const MCAPRecorder &) = delete;

  MCAPRecorder(MCAPRecorder &&) noexcept;
  MCAPRecorder &operator=(MCAPRecorder &&) noexcept;

  /// Open an MCAP file for writing.  Sets up the channel and required
  /// OSI file-level metadata.
  /// @param path  Output file path (should end in .mcap).
  /// @param description  Optional human-readable description for the file.
  void Open(const std::string &path, const std::string &description = "");

  /// Write a single GroundTruth frame to the trace file.
  void WriteFrame(const osi3::GroundTruth &ground_truth);

  /// Close the MCAP file and flush all data.
  void Close();

  /// Whether the recorder is currently recording.
  bool IsOpen() const;

  /// Number of frames written so far.
  uint64_t GetFrameCount() const { return frame_count_; }

private:
  static constexpr const char *kGroundTruthTopic = "GroundTruth";

  std::unique_ptr<osi3::MCAPTraceFileWriter> writer_;
  bool is_open_ = false;
  bool channel_registered_ = false;
  uint64_t frame_count_ = 0;
};

} // namespace osi
} // namespace carla
