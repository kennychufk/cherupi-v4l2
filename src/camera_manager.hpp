#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <libcamera/libcamera.h>

#include "camera.hpp"
#include "types.hpp"

class CameraManager {
 public:
  CameraManager();
  ~CameraManager();

  size_t discoverCameras();

  bool configureAll(const CameraConfig& config = {}, size_t buffer_count = 4);

  bool startAll();
  bool stopAll();
  // Release per-camera libcamera resources and return each camera to IDLE.
  // Caller must ensure cameras aren't RUNNING (stop() first).
  bool unconfigureAll();

  Camera* getCamera(uint32_t camera_id);
  std::vector<Camera*> getAllCameras();

  size_t getCameraCount() const { return cameras.size(); }

  void setFrameCallback(std::function<void(const FrameData&)> callback);
  void setStreamManagerNotify(std::function<void()> callback);

  bool areAnyRunning() const;
  void resetFrameCounts();

  // Apply the same focus setting to every discovered camera. lens_position
  // semantics: < 0 ⇒ continuous AF; ≥ 0 ⇒ manual at that dioptre value.
  void setLensPosition(float lens_position);

 private:
  std::unique_ptr<libcamera::CameraManager> lcam_manager;
  std::vector<std::unique_ptr<Camera>> cameras;
  CameraConfig default_config;
  std::function<void()> stream_manager_notify;
};
