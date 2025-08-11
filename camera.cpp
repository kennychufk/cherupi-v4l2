#include "camera.hpp"

#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include <chrono>
#include <iostream>

Camera::Camera(uint32_t id, MediaDevice* media_dev, const CameraConfig& cfg)
    : camera_id(id), media_device(media_dev), config(cfg) {}

Camera::~Camera() { stop(); }

bool Camera::configure(size_t buffer_count) {
  if (state != CameraState::IDLE) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) + " is not idle");
    return false;
  }

  // Reset all links
  if (!media_device->reset()) {
    LOG_ERROR("Camera", "Failed to reset media device for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Set sensor crop
  if (!media_device->setCrop(config.sensor_entity, 0, config.crop_left,
                             config.crop_top, config.crop_width,
                             config.crop_height)) {
    LOG_ERROR("Camera", "Failed to set sensor crop for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Configure formats
  if (!media_device->setFormat(config.sensor_entity, 0, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set sensor format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 0, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set CSI2 pad0 format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 4, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set CSI2 pad4 format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Enable links
  if (!media_device->setLink(config.sensor_entity, 0, config.csi2_entity, 0,
                             true)) {
    LOG_ERROR("Camera", "Failed to enable sensor to CSI2 link for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setLink(config.csi2_entity, 4, config.video_entity, 0,
                             true)) {
    LOG_ERROR("Camera", "Failed to enable CSI2 to video link for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Get video device path
  std::string video_path =
      media_device->getVideoDevicePath(config.video_entity);
  if (video_path.empty()) {
    LOG_ERROR("Camera", "Failed to get video device path for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Open video device
  video_device = std::make_unique<V4L2Device>();
  if (!video_device->open(video_path)) {
    LOG_ERROR("Camera", "Failed to open video device for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!video_device->setFormat(config.crop_width, config.crop_height,
                               V4L2_PIX_FMT_SRGGB10P)) {
    LOG_ERROR("Camera", "Failed to set video format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!video_device->setupBuffers(buffer_count)) {
    LOG_ERROR("Camera", "Failed to setup buffers for camera " +
                            std::to_string(camera_id));
    return false;
  }

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera",
           "Camera " + std::to_string(camera_id) + " configured successfully");
  return true;
}

bool Camera::start() {
  if (state != CameraState::CONFIGURED) {
    LOG_ERROR("Camera",
              "Camera " + std::to_string(camera_id) + " is not configured");
    return false;
  }

  if (!video_device->startStreaming()) {
    LOG_ERROR("Camera", "Failed to start streaming for camera " +
                            std::to_string(camera_id));
    return false;
  }

  should_stop = false;
  capture_thread = std::make_unique<std::thread>(&Camera::captureLoop, this);

  state = CameraState::RUNNING;
  LOG_INFO("Camera",
           "Camera " + std::to_string(camera_id) + " started successfully");
  return true;
}

bool Camera::stop() {
  if (state != CameraState::RUNNING) {
    return true;
  }

  LOG_INFO("Camera", "Stopping camera " + std::to_string(camera_id));
  should_stop = true;

  // Wake up any waiting threads
  new_frame_cv.notify_all();

  if (capture_thread && capture_thread->joinable()) {
    capture_thread->join();
  }

  video_device->stopStreaming();

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera", "Camera " + std::to_string(camera_id) + " stopped");
  return true;
}

void Camera::captureLoop() {
  LOG_DEBUG("Camera",
            "Camera " + std::to_string(camera_id) + " capture loop started");

  // Track capture failures for logging
  int consecutive_failures = 0;
  bool first_frame = true;

  while (!should_stop) {
    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter;

    if (video_device->captureFrame(frame)) {
      frame_counter++;
      consecutive_failures = 0;  // Reset failure counter

      if (first_frame) {
        LOG_INFO("Camera",
                 "Camera " + std::to_string(camera_id) + " got first frame");
        first_frame = false;
      }

      // Update latest frame for streaming
      {
        std::lock_guard<std::mutex> lock(latest_frame_mutex);
        latest_frame = frame;  // Deep copy
        has_new_frame = true;
      }

      // Notify streaming thread that new frame is available
      new_frame_cv.notify_one();

      // Also notify through the callback if set
      if (onFrameAvailable) {
        onFrameAvailable();
      }

      // Notify frame saver if callback is set and saving is enabled
      if (onFrameCaptured) {
        onFrameCaptured(frame);
      }

      LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                              " captured frame " +
                              std::to_string(frame.frame_id));
    } else {
      consecutive_failures++;

      // Log warnings for extended capture failures
      if (consecutive_failures == 10) {
        LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                               " has failed to capture 10 consecutive frames");
      } else if (consecutive_failures == 100) {
        LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) +
                                " has failed to capture 100 consecutive frames "
                                "- check camera hardware");
      }

      // Small delay on capture failure to avoid busy loop
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  LOG_DEBUG("Camera",
            "Camera " + std::to_string(camera_id) + " capture loop ended");
}

bool Camera::getFrameForStreaming(FrameData& frame) {
  // First, check if streaming frame is still in use
  {
    std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
    if (streaming_frame_in_use) {
      LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                              " streaming frame still in use");
      return false;
    }
  }

  // Copy latest frame to streaming buffer
  {
    std::lock_guard<std::mutex> latest_lock(latest_frame_mutex);
    if (!has_new_frame) {
      return false;
    }

    // Copy to streaming buffer
    {
      std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
      streaming_frame = latest_frame;  // Deep copy
      streaming_frame_in_use = true;
      frame = streaming_frame;  // Copy to output
    }

    has_new_frame = false;
  }

  return true;
}

bool Camera::waitForNewFrame(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(latest_frame_mutex);
  return new_frame_cv.wait_for(
      lock, timeout, [this] { return has_new_frame || should_stop.load(); });
}

void Camera::releaseStreamingFrame() {
  std::lock_guard<std::mutex> lock(streaming_frame_mutex);
  streaming_frame_in_use = false;
  LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                          " streaming frame released");
}

void Camera::setFrameAvailableCallback(std::function<void()> callback) {
  onFrameAvailable = callback;
}
