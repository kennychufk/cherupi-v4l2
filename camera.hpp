#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "awb_bayes.hpp"
#include "awb_processor.hpp"
#include "fe_configurator.hpp"
#include "frontend/pisp_statistics.h"
#include "media_device.hpp"
#include "types.hpp"
#include "v4l2_device.hpp"

class Camera {
 private:
  uint32_t camera_id;
  MediaDevice* media_device;  // Non-owning pointer
  std::unique_ptr<V4L2Device> video_device;
  // Bayes pipeline devices — only populated when AwbMode::BAYES is active
  // and the FE reroute in configure() succeeds. Otherwise null, and the
  // grey-world single-device path runs on video_device alone.
  std::unique_ptr<V4L2Device> stats_device;
  std::unique_ptr<V4L2Device> fe_config_device;
  std::unique_ptr<FeConfigurator> fe_configurator;
  bool bayes_pipeline_active = false;
  CameraConfig config;

  std::atomic<CameraState> state{CameraState::IDLE};
  std::atomic<uint32_t> frame_counter{0};
  std::atomic<uint32_t> frames_dropped{0};

  // Dual buffer system for streaming
  std::mutex streaming_frame_mutex;
  FrameData streaming_frame;  // Frame currently being streamed
  bool streaming_frame_in_use = false;

  std::mutex latest_frame_mutex;
  FrameData latest_frame;  // Most recent frame available for streaming
  bool has_new_frame = false;
  std::condition_variable new_frame_cv;

  // Capture thread
  std::unique_ptr<std::thread> capture_thread;
  std::atomic<bool> should_stop{false};

  // Per-camera AWB — grey-world by default, bayes when enabled.
  AwbProcessor awb;
  AwbBayes awb_bayes;

  // Sensor subdevice for reading exposure/gain controls (lux estimation).
  std::string sensor_subdev_path_;
  int sensor_subdev_fd_ = -1;
  double sensor_line_time_us_ = 0.0;  // µs per exposure line

  void captureLoop();
  void captureLoopBayes();
  bool configureBayesPipeline(size_t buffer_count);
  double readSensorLux(const pisp_statistics& stats) const;

 public:
  Camera(uint32_t id, MediaDevice* media_dev, const CameraConfig& cfg);
  ~Camera();

  bool configure(size_t buffer_count = 4);
  void setAwbConfig(const AwbConfig& cfg) {
    config.awb = cfg;
    awb.setConfig(cfg);
  }
  bool start();
  bool stop();

  // Get frame for streaming (returns false if no new frame)
  // This will copy the latest frame to streaming buffer if available
  bool getFrameForStreaming(FrameData& frame);

  // Wait for new frame with timeout (for efficient notification)
  bool waitForNewFrame(std::chrono::milliseconds timeout);

  // Mark streaming frame as no longer in use
  void releaseStreamingFrame();

  // For frame saving - callback for every captured frame
  std::function<void(const FrameData&)> onFrameCaptured;

  // For streaming notification - called when new frame available
  std::function<void()> onFrameAvailable;

  // Set the frame available callback
  void setFrameAvailableCallback(std::function<void()> callback);

  uint32_t getId() const { return camera_id; }
  CameraState getState() const { return state; }
  uint32_t getFramesCaptured() const { return frame_counter; }
  uint32_t getFramesDropped() const { return frames_dropped; }
  const CameraConfig& getConfig() const { return config; }
  void resetFrameCounts() {
    frame_counter = 0;
    frames_dropped = 0;
    LOG_INFO("Camera",
             "Reset frame counts for camera " + std::to_string(camera_id));
  }
};
