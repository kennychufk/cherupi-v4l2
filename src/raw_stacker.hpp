#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "capture_frame.hpp"
#include "raw_stack.hpp"
#include "types.hpp"

// A stack_capture request as the client sent it (see command_parser).
struct StackRequest {
  std::optional<uint32_t> camera_id;  // absent ⇒ every running camera
  uint32_t num_frames = 16;
  // Pinned for the stack, then the preview settings come back. Gain defaults
  // to 1.0 when only an exposure is given: left on auto, the AGC would trade
  // the longer exposure straight back for less gain.
  std::optional<int32_t> exposure_time_us;
  std::optional<float> analogue_gain;
  int skip_frames = 0;  // after the settings have settled
  bool send_image = true;
};

// What the server resolved a request into.
struct StackPlan {
  std::vector<uint32_t> camera_ids;
  uint32_t num_frames = 16;
  std::optional<int32_t> exposure_time_us;
  std::optional<float> analogue_gain;
  int skip_frames = 0;
  // The process mode's detection, run on each stacked image.
  raw_stack::DetectConfig detect;
  bool send_image = true;
};

// Outcome of one stack, one entry per armed camera in camera_id order.
struct StackResult {
  struct Capture {
    uint32_t camera_id = 0;
    std::string error;  // empty on success
    uint32_t frames_stacked = 0;
    // Summed frames thrown away because the exposure/gain changed mid-stack
    // (the sum then restarts).
    uint32_t frames_rejected = 0;
    uint32_t settle_frames = 0;    // waited out before the settings applied
    uint32_t first_frame_id = 0, last_frame_id = 0;
    uint64_t first_timestamp_us = 0, last_timestamp_us = 0;
    int32_t exposure_time_us = -1;
    float analogue_gain = 0.f;
    float black_level = 0.f;
    unsigned bit_depth = 0;
    // Brightness, as fractions of the sensor's range above black: the
    // stacked image's 99.5th percentile, the brightest single sample, and the
    // largest fraction of samples clipped in any one frame.
    float white_fraction = 0.f;
    float peak_fraction = 0.f;
    float clipped_fraction = 0.f;
    uint32_t width = 0, height = 0;  // the stacked (2x2-binned) image
    double accumulate_ms = 0.0;  // mean per frame, on the capture thread
  };
  uint32_t stack_id = 0;
  bool cancelled = false;
  std::vector<Capture> captures;
};

// The `stack_result` message for `result`.
nlohmann::json stackResultJson(const StackResult& result);

// Runs stack_capture. A request arms every named camera: each one's exposure
// and gain are pinned (if asked), its raw frames are ignored until their
// metadata shows the new settings, `skip_frames` more are dropped, and the
// next `num_frames` taken with identical settings are summed (a change
// restarts the sum). A camera whose settings never arrive, or keep changing,
// fails with an error instead. Once every camera is done a worker thread
// turns the sums into images and acks, then runs the process mode's detection
// on each image — off the capture thread, since it takes hundreds of ms — and
// hands the images over for delivery. Nothing is saved. At most one stack is
// outstanding, from arm until its images have been handed over.
class RawStacker {
 public:
  using OverrideFn = std::function<void(uint32_t camera_id,
                                        std::optional<int32_t> exposure_time_us,
                                        std::optional<float> analogue_gain)>;
  using RestoreFn = std::function<void(uint32_t camera_id)>;

  // Frames a camera may take to show the requested settings. libcamera
  // applies a control a few frames after the request carrying it, and the
  // requests already queued go first.
  static constexpr uint32_t kSettleMaxFrames = 30;

  // `apply` pins a camera's exposure/gain for a stack and `restore` returns
  // it to the preview settings. Both are called on the capture thread or the
  // caller's, so they must be cheap and thread-safe.
  RawStacker(OverrideFn apply, RestoreFn restore);
  ~RawStacker();

  RawStacker(const RawStacker&) = delete;
  RawStacker& operator=(const RawStacker&) = delete;

  using AckFn = std::function<void(StackResult)>;
  using FramesFn = std::function<void(std::vector<CaptureFrame>)>;

  // Arm a stack. `on_ack` runs on the worker thread once the images are
  // formed (or, if cancelled, on the canceller's); `on_frames` runs exactly
  // once per stack after it, with one image per camera that succeeded, in
  // camera_id order — none when cancelled. Returns the stack id, or nullopt
  // when a stack is already outstanding or `plan` names no camera.
  std::optional<uint32_t> arm(StackPlan plan, AckFn on_ack, FramesFn on_frames);
  bool hasPending() const;

  // Capture-thread entry point: one raw frame from one camera.
  void onRawFrame(const RawFrame& frame);

  // Abandon a stack that is still capturing (stop_cameras): restore every
  // camera and deliver a `cancelled` result. A stack already being finished
  // runs to completion.
  void cancel();
  // start_cameras: restart stack ids at 1, as trigger ids do.
  void reset();

 private:
  enum class Phase { SETTLING, SKIPPING, STACKING, DONE };
  struct CameraJob {
    Phase phase = Phase::SETTLING;
    uint32_t settle_frames = 0;
    int skipped = 0;
    uint32_t rejected = 0;
    raw_stack::Accumulator acc;
    RawFrame first;  // metadata only; `data` is cleared
    RawFrame last;
    size_t clipped_max = 0;
    double accumulate_us = 0.0;
    std::string error;
  };
  struct Pending {
    uint32_t id = 0;
    StackPlan plan;
    std::map<uint32_t, CameraJob> jobs;
    AckFn on_ack;
    FramesFn on_frames;
    bool finishing = false;  // handed to the worker; frames are ignored
  };

  void advance(CameraJob& job, const RawFrame& frame);
  void workerLoop();
  // Form each camera's image (as a STACK frame, into `images`) and the ack.
  StackResult finish(Pending& p, std::vector<std::shared_ptr<FrameData>>& images);

  OverrideFn apply_;
  RestoreFn restore_;

  mutable std::mutex mutex_;
  std::unique_ptr<Pending> pending_;
  uint32_t next_id_ = 1;
  std::condition_variable work_cv_;
  bool stop_ = false;
  std::thread worker_;
};
