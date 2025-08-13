#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "bayer_converter.h"
#include "checkerboard_detector.h"
#include "types.hpp"

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

  // For checkerboard detection
  std::unique_ptr<CheckerboardDetector> checkerboard_detector;

  void writerThreadFunc();
  std::string generateFilename(uint32_t camera_id, uint32_t frame_id);
  bool detectCheckerboard(const FrameData& frame);
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
};
