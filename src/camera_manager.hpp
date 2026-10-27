#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <libcamera/libcamera.h>

#include "camera.hpp"
#include "types.hpp"

class CameraManager {
 public:
  CameraManager();
  ~CameraManager();

  // Enumerate libcamera cameras whose `properties::Model` contains `sensor`
  // as a case-insensitive substring (e.g. "imx519", "imx477"), numbered from
  // 0. A call for the sensor already discovered returns the same cameras; a
  // call for another one rebuilds the set, which requires every discovered
  // camera to be IDLE (otherwise the current set is kept and an error
  // logged). A Camera is created once per libcamera camera and kept for the
  // manager's life, so a pointer from getCamera() never dangles — a streamer
  // thread may hold one across a rediscovery. A sensor that matches nothing
  // leaves no cameras and no sensor, so the next call enumerates again.
  size_t discoverCameras(const std::string& sensor = "imx519");
  // The lower-cased sensor filter of the discovered cameras; empty if none.
  std::string activeSensor() const;

  bool configureAll(const CameraConfig& config = {}, size_t buffer_count = 4);

  bool startAll();
  bool stopAll();
  // Release per-camera libcamera resources and return each camera to IDLE.
  // Caller must ensure cameras aren't RUNNING (stop() first).
  bool unconfigureAll();

  Camera* getCamera(uint32_t camera_id);
  std::vector<Camera*> getAllCameras();

  size_t getCameraCount() const;

  void setFrameCallback(
      std::function<void(const std::shared_ptr<const FrameData>&)> callback);
  // Each camera's raw frames (Camera::onRawFrame); only cameras configured
  // with CameraConfig::raw deliver any.
  void setRawFrameCallback(std::function<void(const RawFrame&)> callback);
  // Camera::setLiveFrames on every camera.
  void setLiveFrames(bool enabled);
  void setStreamManagerNotify(std::function<void()> callback);

  bool areAnyRunning() const;
  void resetFrameCounts();

  // One camera whose capture has stopped delivering frames (see
  // Camera::captureStalledForUs), reported exactly once per camera.
  struct StalledCamera {
    uint32_t camera_id = 0;
    uint64_t stalled_for_us = 0;
  };
  // Sweep every camera and return the ones that have newly stalled, flagging
  // each so it is not reported again. Cheap enough to call on a timer; safe to
  // call from any thread.
  std::vector<StalledCamera> pollStalledCameras();

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
  // The block of the output frame averaged into FocusFoM, from the first
  // camera (all cameras share one CameraConfig). nullopt if there are no
  // cameras.
  std::optional<focus_region::Region> getFocusFomRegion() const;

 private:
  std::unique_ptr<libcamera::CameraManager> lcam_manager;
  bool lcam_started_ = false;
  // Every Camera ever discovered, by libcamera id; never shrinks.
  std::map<std::string, std::unique_ptr<Camera>> pool_;
  // The discovered set, indexed by camera id. Replaced only by
  // discoverCameras (on the uWS loop thread); the streamer and watchdog
  // threads read it, so they and the replacement take cameras_mutex_. The
  // loop thread's own reads need no lock.
  std::vector<Camera*> cameras;
  std::string active_sensor_;
  mutable std::shared_mutex cameras_mutex_;
  CameraConfig default_config;
  std::function<void()> stream_manager_notify;
};
