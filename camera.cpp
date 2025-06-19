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
    std::cerr << "Camera " << camera_id << " is not idle" << std::endl;
    return false;
  }

  // Reset all links
  if (!media_device->reset()) {
    std::cerr << "Failed to reset media device for camera " << camera_id
              << std::endl;
    return false;
  }

  // Set sensor crop
  if (!media_device->setCrop(config.sensor_entity, 0, config.crop_left,
                             config.crop_top, config.crop_width,
                             config.crop_height)) {
    std::cerr << "Failed to set sensor crop for camera " << camera_id
              << std::endl;
    return false;
  }

  // Configure formats
  if (!media_device->setFormat(config.sensor_entity, 0, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set sensor format for camera " << camera_id
              << std::endl;
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 0, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set CSI2 pad0 format for camera " << camera_id
              << std::endl;
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 4, config.crop_width,
                               config.crop_height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set CSI2 pad4 format for camera " << camera_id
              << std::endl;
    return false;
  }

  // Enable links
  if (!media_device->setLink(config.sensor_entity, 0, config.csi2_entity, 0,
                             true)) {
    std::cerr << "Failed to enable sensor to CSI2 link for camera " << camera_id
              << std::endl;
    return false;
  }

  if (!media_device->setLink(config.csi2_entity, 4, config.video_entity, 0,
                             true)) {
    std::cerr << "Failed to enable CSI2 to video link for camera " << camera_id
              << std::endl;
    return false;
  }

  // Get video device path
  std::string video_path =
      media_device->getVideoDevicePath(config.video_entity);
  if (video_path.empty()) {
    std::cerr << "Failed to get video device path for camera " << camera_id
              << std::endl;
    return false;
  }

  // Open video device
  video_device = std::make_unique<V4L2Device>();
  if (!video_device->open(video_path)) {
    std::cerr << "Failed to open video device for camera " << camera_id
              << std::endl;
    return false;
  }

  if (!video_device->setFormat(config.crop_width, config.crop_height,
                               V4L2_PIX_FMT_SRGGB10P)) {
    std::cerr << "Failed to set video format for camera " << camera_id
              << std::endl;
    return false;
  }

  if (!video_device->setupBuffers(buffer_count)) {
    std::cerr << "Failed to setup buffers for camera " << camera_id
              << std::endl;
    return false;
  }

  state = CameraState::CONFIGURED;
  return true;
}

bool Camera::start() {
  if (state != CameraState::CONFIGURED) {
    std::cerr << "Camera " << camera_id << " is not configured" << std::endl;
    return false;
  }

  if (!video_device->startStreaming()) {
    std::cerr << "Failed to start streaming for camera " << camera_id
              << std::endl;
    return false;
  }

  should_stop = false;
  capture_thread = std::make_unique<std::thread>(&Camera::captureLoop, this);

  state = CameraState::RUNNING;
  return true;
}

bool Camera::stop() {
  if (state != CameraState::RUNNING) {
    return true;
  }

  should_stop = true;

  if (capture_thread && capture_thread->joinable()) {
    capture_thread->join();
  }

  video_device->stopStreaming();

  state = CameraState::CONFIGURED;
  return true;
}

void Camera::captureLoop() {
  while (!should_stop) {
    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter;

    if (video_device->captureFrame(frame)) {
      frame_counter++;

      // Store latest frame for streaming
      {
        std::lock_guard<std::mutex> lock(latest_frame_mutex);
        latest_frame = frame;
        has_new_frame = true;
      }

      // Notify frame saver if callback is set
      if (onFrameCaptured) {
        onFrameCaptured(frame);
      }
    } else {
      // Small delay on capture failure to avoid busy loop
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

bool Camera::getLatestFrame(FrameData& frame) {
  std::lock_guard<std::mutex> lock(latest_frame_mutex);
  if (!has_new_frame) {
    return false;
  }

  frame = latest_frame;
  has_new_frame = false;
  return true;
}
