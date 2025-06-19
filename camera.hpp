#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "media_device.hpp"
#include "types.hpp"
#include "v4l2_device.hpp"

class Camera {
 private:
  uint32_t camera_id;
  MediaDevice* media_device;  // Non-owning pointer
  std::unique_ptr<V4L2Device> video_device;
  CameraConfig config;

  std::atomic<CameraState> state{CameraState::IDLE};
  std::atomic<uint32_t> frame_counter{0};
  std::atomic<uint32_t> frames_dropped{0};

  // Latest frame for streaming
  std::mutex latest_frame_mutex;
  FrameData latest_frame;
  bool has_new_frame = false;

  // Capture thread
  std::unique_ptr<std::thread> capture_thread;
  std::atomic<bool> should_stop{false};

  void captureLoop();

 public:
  Camera(uint32_t id, MediaDevice* media_dev, const CameraConfig& cfg);
  ~Camera();

  bool configure(size_t buffer_count = 4);
  bool start();
  bool stop();

  // Get latest frame for streaming (returns false if no new frame)
  bool getLatestFrame(FrameData& frame);

  // For frame saving - get a copy of every captured frame
  std::function<void(const FrameData&)> onFrameCaptured;

  uint32_t getId() const { return camera_id; }
  CameraState getState() const { return state; }
  uint32_t getFramesCaptured() const { return frame_counter; }
  uint32_t getFramesDropped() const { return frames_dropped; }
  const CameraConfig& getConfig() const { return config; }
};
