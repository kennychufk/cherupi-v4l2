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
  // Apply the same exposure setting to every discovered camera.
  // exposure_time_us < 0 ⇒ auto AE; > 0 ⇒ manual shutter in microseconds.
  void setExposureTime(int32_t exposure_time_us);
  // Apply the same frame-duration lock to every discovered camera.
  // frame_duration_us > 0 ⇒ lock FrameDurationLimits to {x, x}; ≤ 0 ⇒ unset.
  void setFrameDuration(int64_t frame_duration_us);
  // Currently-applied frame duration in microseconds (0 ⇒ unset). Read from
  // the first camera since the setting is fanned out identically.
  int64_t getCurrentFrameDuration() const;
  // Hardware FrameDurationLimits {min, max} in microseconds, taken from the
  // first camera. Returns {0, 0} if cameras aren't acquired or the control
  // isn't advertised.
  std::pair<int64_t, int64_t> getFrameDurationLimitsHw() const;
  // Hardware LensPosition range {min, max, def} in dioptres, taken from the
  // first camera (all cameras run the same sensor). Fields are NaN if there
  // are no cameras, they aren't acquired, or the control isn't advertised.
  LensPositionLimits getLensPositionLimitsHw() const;

 private:
  std::unique_ptr<libcamera::CameraManager> lcam_manager;
  std::vector<std::unique_ptr<Camera>> cameras;
  CameraConfig default_config;
  std::function<void()> stream_manager_notify;
};
