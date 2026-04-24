#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <sys/mman.h>
#include <vector>

#include <libcamera/libcamera.h>

#include "types.hpp"

class Camera {
 public:
  Camera(uint32_t id, std::shared_ptr<libcamera::Camera> lcam,
         const CameraConfig& cfg);
  ~Camera();

  bool configure(size_t buffer_count = 4);
  void setAwbConfig(const AwbConfig& cfg) { config.awb = cfg; }
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

  std::function<void(const FrameData&)> onFrameCaptured;
  std::function<void()> onFrameAvailable;
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

 private:
  uint32_t camera_id;
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

  std::mutex streaming_frame_mutex;
  FrameData streaming_frame;
  bool streaming_frame_in_use = false;

  std::atomic<bool> should_stop{false};

  void onRequestComplete(libcamera::Request* request);
  // Shared teardown for stop() and unconfigure(): unmap buffers, free
  // allocator, drop requests, release lcam. Idempotent — safe to call even
  // if some or all resources were already released.
  void releaseConfiguredResources();
};
