#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/types.hpp>

#include "checkerboard_detector.h"
#include "types.hpp"

// One detected checkerboard's worth of corners, already mapped back to
// full-frame Y-plane pixel coordinates. `set_id` encodes which sub-frame the
// corners came from: 0 for `checkerboard` mode (whole frame); 0..3 for
// `checkerboard2x2` (row*2 + col, where row/col are 0 for top/left and 1 for
// bottom/right).
struct CornerSet {
  uint8_t set_id = 0;
  std::vector<cv::Point2f> corners;
};

// All corner sets detected on one frame, keyed back to that frame.
struct FrameDetection {
  uint32_t frame_id = 0;
  std::vector<CornerSet> sets;
};

class FrameSaver {
 private:
  SaveConfig config;
  std::atomic<bool> enabled{false};
  std::string actual_output_dir;  // The directory that will actually be used

  // For buffer mode
  std::vector<FrameData> buffered_frames;
  std::mutex buffer_mutex;

  // For batch and checkerboard modes
  struct WriteTask {
    std::string filename;
    std::vector<uint8_t> data;
    uint32_t camera_id;
    uint32_t frame_id;
  };

  std::queue<WriteTask> write_queue;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::vector<std::thread> writer_threads;
  std::atomic<bool> stop_threads{false};

  std::atomic<size_t> frames_saved{0};
  std::atomic<size_t> bytes_written{0};
  std::atomic<size_t> frames_checked{0};
  std::atomic<size_t> checkerboards_detected{0};

  // Per-camera frame saving counters
  std::unordered_map<uint32_t, std::atomic<uint32_t>> frames_saved_per_camera;
  // std::mutex is not used to protect the map because the numbers do not need
  // to be so precise

  // For checkerboard detection
  std::unique_ptr<CheckerboardDetector> checkerboard_detector;

  // Per-camera cache of the most recent detection, written by the saver and
  // read by the streamer. Only the latest frame_id per camera is retained.
  std::unordered_map<uint32_t, FrameDetection> latest_detection_per_camera;
  std::mutex detection_mutex;

  void writerThreadFunc();
  std::string generateFilename(uint32_t camera_id, uint32_t frame_id);
  // Detect on the whole Y plane (extracted per `checkerboard_full_res_detection`).
  // On success, populates `out_sets` with one CornerSet (set_id=0) whose
  // coordinates are translated to full-frame Y-plane pixel space.
  bool detectCheckerboard(const FrameData& frame,
                          std::vector<CornerSet>& out_sets);
  // CHECKERBOARD2X2: split the (already full-res or subsampled per
  // `checkerboard_full_res_detection`) Y plane into 4 equal quadrants and run
  // detection on each in parallel; returns true if any sub-frame detects.
  // `out_sets` receives one CornerSet per detecting quadrant, set_id = row*2+col,
  // coordinates translated to full-frame Y-plane pixel space.
  bool detectCheckerboard2x2(const FrameData& frame,
                             std::vector<CornerSet>& out_sets);
  bool createOutputDirectory();  // New method for directory creation

 public:
  FrameSaver() = default;
  ~FrameSaver();

  void configure(const SaveConfig& cfg);
  void start();
  void stop();

  // Called for each captured frame
  void saveFrame(const FrameData& frame);

  // For buffer mode - write all buffered frames to disk
  void flushBufferedFrames();

  size_t getFramesSaved() const { return frames_saved; }
  size_t getBytesWritten() const { return bytes_written; }
  size_t getFramesChecked() const { return frames_checked; }
  size_t getCheckerboardsDetected() const { return checkerboards_detected; }
  bool isEnabled() const { return enabled; }
  SaveMode getMode() const { return config.mode; }
  const std::string& getActualOutputDir() const { return actual_output_dir; }

  // Streamer-facing: latest cached detection for a camera. Returns nullopt if
  // no detection has run yet for that camera (e.g. save mode is NONE) or if
  // the cached entry's frame_id doesn't match the frame the caller is about
  // to send. Safe to call from any thread.
  std::optional<FrameDetection> getDetectionForFrame(uint32_t camera_id,
                                                     uint32_t frame_id) {
    std::lock_guard<std::mutex> lock(detection_mutex);
    auto it = latest_detection_per_camera.find(camera_id);
    if (it == latest_detection_per_camera.end()) return std::nullopt;
    if (it->second.frame_id != frame_id) return std::nullopt;
    return it->second;
  }

  // Get saved frame count for specific camera
  uint32_t getFramesSavedForCamera(uint32_t camera_id) {
    auto it = frames_saved_per_camera.find(camera_id);
    return it != frames_saved_per_camera.end() ? it->second.load() : 0;
  }

  // Reset saved frame counts for all cameras
  void resetFramesSavedCounts() {
    for (auto& pair : frames_saved_per_camera) {
      pair.second = 0;
    }
    // Per-camera frame_id counters reset alongside this, so the cached
    // detection's frame_id no longer matches any future stream frame —
    // drop it to avoid stale-cache hits.
    {
      std::lock_guard<std::mutex> lock(detection_mutex);
      latest_detection_per_camera.clear();
    }
    LOG_INFO("FrameSaver", "Reset per-camera saved frame counts");
  }
};
