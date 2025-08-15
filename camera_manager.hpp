#pragma once

#include <functional>
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

  // Callback to notify stream manager of new frames
  std::function<void()> stream_manager_notify;

 public:
  CameraManager();
  ~CameraManager();

  // Discover all IMX296 cameras (won't re-discover if already done)
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

  // Set callback to notify stream manager when frames are available
  void setStreamManagerNotify(std::function<void()> callback);

  // Check if any cameras are running
  bool areAnyRunning() const;

  void resetFrameCounts();
};
