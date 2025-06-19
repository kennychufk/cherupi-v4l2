#pragma once

#include <memory>
#include <vector>

#include "camera.hpp"
#include "media_device.hpp"
#include "types.hpp"

class CameraManager {
 private:
  std::vector<MediaDevice> media_devices;
  std::vector<std::unique_ptr<Camera>> cameras;
  CameraConfig default_config;

 public:
  CameraManager() = default;
  ~CameraManager() = default;

  // Discover all IMX296 cameras
  size_t discoverCameras();

  // Configure all cameras with the same config
  bool configureAll(const CameraConfig& config = {}, size_t buffer_count = 4);

  // Start/stop all cameras
  bool startAll();
  bool stopAll();

  // Get camera by ID
  Camera* getCamera(uint32_t camera_id);

  // Get all cameras
  std::vector<Camera*> getAllCameras();

  size_t getCameraCount() const { return cameras.size(); }

  // Set callback for frame saving
  void setFrameCallback(std::function<void(const FrameData&)> callback);
};
