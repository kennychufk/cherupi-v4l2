#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <sys/mman.h>
#include <vector>

#include <libcamera/libcamera.h>

#include "types.hpp"

class Camera {
 public:
  Camera(uint32_t id, std::shared_ptr<libcamera::Camera> lcam,
         const CameraConfig& cfg, std::string model);
  ~Camera();

  bool configure(size_t buffer_count = 4);
  void setConfig(const CameraConfig& cfg) { config = cfg; }
  // Set focus. lens_position < 0 ⇒ continuous AF; ≥ 0 ⇒ manual at that lens
  // position in dioptres (~0 = infinity, ~10 = closest macro; per-tuning
  // map clamps the actual range). Thread-safe; takes effect on the next
  // queued request when RUNNING, or at the next start() when CONFIGURED.
  void setLensPosition(float lens_position);
  // Set exposure time. exposure_time_us < 0 ⇒ auto AE; > 0 ⇒ manual at that
  // value in microseconds. Thread-safe; same generation-counter pattern as
  // setLensPosition.
  void setExposureTime(int32_t exposure_time_us);
  // Set frame duration. frame_duration_us > 0 ⇒ lock FrameDurationLimits to
  // {x, x} (fixed frame interval / framerate). ≤ 0 ⇒ unset: at start() the
  // control is omitted (libcamera defaults); when transitioning to unset
  // while RUNNING the camera's HW max range is applied so any previous lock
  // is released. Thread-safe; same generation-counter pattern.
  void setFrameDuration(int64_t frame_duration_us);
  // Returns the currently-configured frame duration in microseconds, or 0 if
  // unset (no fixed FrameDurationLimits applied).
  int64_t getCurrentFrameDuration() const {
    return frame_duration_us_.load();
  }
  // Returns {min, max} hardware FrameDurationLimits in microseconds from
  // libcamera's ControlInfoMap. Returns {0, 0} if the control is not
  // advertised. Requires the camera to be acquired (CONFIGURED or RUNNING).
  std::pair<int64_t, int64_t> getFrameDurationLimitsHw() const;
  // Returns the hardware LensPosition range {min, max, def} in dioptres from
  // libcamera's ControlInfoMap. Fields are NaN if the control is not
  // advertised (fixed-focus module) or the camera is not acquired. Unlike
  // FrameDurationLimits, NaN (not 0) is the sentinel because 0 dioptres
  // (infinity) is a valid limit.
  LensPositionLimits getLensPositionLimitsHw() const;
  bool start();
  bool stop();
  // Release all libcamera resources acquired by configure() and transition
  // CONFIGURED → IDLE. No-op if already IDLE. Fails if RUNNING — caller must
  // stop() first. Idempotent against cameras whose resources were already
  // released by stop() (state flag vs. actual ownership).
  bool unconfigure();

  bool getFrameForStreaming(FrameData& frame);
  bool waitForNewFrame(std::chrono::milliseconds timeout);
  void releaseStreamingFrame();

  // Capture watchdog. libcamera's RPi pipeline handler gives up on a stalled
  // frontend after 5x the frame duration and logs "Camera frontend has timed
  // out!" — after which onRequestComplete() is never called again for the
  // life of the camera. Nothing in libcamera tells us that happened, so the
  // only signal is the absence of frames. These two let a supervisor notice.
  //
  // Returns how long ago the last frame completed, in µs, but only once that
  // exceeds the stall threshold — 0 means "not stalled" (including a camera
  // that isn't RUNNING, or one that hasn't produced its first frame yet).
  // The threshold is 10x the observed inter-frame interval, floored at 3 s,
  // so it always trips after libcamera's own watchdog rather than racing it.
  uint64_t captureStalledForUs() const;
  // Latch a stalled camera so a supervisor reports it exactly once per stall
  // episode. Returns true the first time; false while the latch is still set.
  // The next delivered frame clears it. This does NOT change the camera's
  // lifecycle state: it stays RUNNING, because stop() and unconfigure() must
  // still run to release the libcamera resources, and start() must be able to
  // bring it back — stop_cameras + start_cameras is the recovery path.
  bool markCaptureFailed();

  std::function<void(const FrameData&)> onFrameCaptured;
  std::function<void()> onFrameAvailable;
  void setFrameAvailableCallback(std::function<void()> callback);

  uint32_t getId() const { return camera_id; }
  // libcamera `properties::Model` string of the underlying sensor, e.g.
  // "imx519". Used to report the actual sensor type in the discover response.
  const std::string& getModel() const { return model; }
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

 private:
  uint32_t camera_id;
  std::string model;
  std::shared_ptr<libcamera::Camera> lcam;
  std::unique_ptr<libcamera::CameraConfiguration> cam_config;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::unique_ptr<libcamera::Request>> requests;

  struct MappedPlane {
    void* data = MAP_FAILED;
    size_t size = 0;
  };
  struct MmapRegion {
    void* base = MAP_FAILED;
    size_t size = 0;
  };
  // Indexed by request/buffer slot, then plane index.
  std::vector<std::vector<MappedPlane>> mapped_planes;
  // Actual mmap regions to unmap (one per unique fd per buffer slot).
  std::vector<std::vector<MmapRegion>> mmap_regions;

  CameraConfig config;
  std::atomic<CameraState> state{CameraState::IDLE};
  // Tracks whether lcam->acquire() has been called without a matching
  // release(). stop() releases and clears this; unconfigure() must not
  // double-release.
  bool lcam_acquired_ = false;
  std::atomic<uint32_t> frame_counter{0};
  std::atomic<uint32_t> frames_dropped{0};

  std::mutex latest_frame_mutex;
  FrameData latest_frame;
  bool has_new_frame = false;
  std::condition_variable new_frame_cv;

  // Gate only — there is deliberately no FrameData here. getFrameForStreaming
  // hands the caller the frame directly; a member copy would be a second
  // 17.9 MB memcpy per frame that nothing ever read.
  std::mutex streaming_frame_mutex;
  bool streaming_frame_in_use = false;

  libcamera::Stream* raw_stream_{nullptr};
  std::atomic<bool> should_stop{false};

  // Per-camera frame timing diagnostics (libcamera completion thread only).
  uint64_t last_frame_ts_ns_ = 0;  // hardware timestamp of previous frame (ns)
  uint32_t fps_log_counter_ = 0;   // counts frames between periodic FPS logs

  // Capture watchdog state, published by the libcamera completion thread and
  // read by whichever thread supervises the cameras. Wall-clock (steady) time
  // is what matters here, not the hardware timestamp: a stalled frontend stops
  // producing hardware timestamps altogether.
  std::atomic<uint64_t> last_frame_wall_us_{0};
  // Exponential moving average of the observed inter-frame interval (µs), so
  // the stall threshold tracks the cadence actually being achieved rather than
  // the one that was requested. 0 until two frames have arrived.
  std::atomic<uint64_t> observed_interval_us_{0};
  // Set by markCaptureFailed() so a stall is reported once, not every tick.
  std::atomic<bool> capture_failed_{false};

  // Focus state. af_continuous_ true ⇒ AfModeContinuous; false ⇒ AfModeManual
  // at lens_position_ dioptres. focus_generation_ is bumped on every
  // setLensPosition; the libcamera-completion thread compares it against
  // applied_focus_generation_ to decide whether to attach controls to the
  // next requeued request.
  std::atomic<bool> af_continuous_{true};
  std::atomic<float> lens_position_{0.0f};
  std::atomic<uint64_t> focus_generation_{0};
  uint64_t applied_focus_generation_ = 0;  // libcamera-completion thread only

  std::atomic<bool>     ae_auto_{true};
  std::atomic<int32_t>  exposure_time_us_{0};
  std::atomic<uint64_t> exposure_generation_{0};
  uint64_t              applied_exposure_generation_ = 0;  // libcamera-completion thread only

  // Frame duration state. 0 ⇒ unset (no FrameDurationLimits applied); > 0 ⇒
  // lock both min and max to this value. Same generation-counter pattern as
  // focus/exposure for runtime updates on the libcamera-completion thread.
  std::atomic<int64_t>  frame_duration_us_{0};
  std::atomic<uint64_t> frame_duration_generation_{0};
  uint64_t              applied_frame_duration_generation_ = 0;  // libcamera-completion thread only

  void onRequestComplete(libcamera::Request* request);
  // Shared teardown for stop() and unconfigure(): unmap buffers, free
  // allocator, drop requests, release lcam. Idempotent — safe to call even
  // if some or all resources were already released.
  void releaseConfiguredResources();
};
