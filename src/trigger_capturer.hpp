#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "capture_frame.hpp"
#include "types.hpp"

// A trigger_capture request as the client sent it (see command_parser).
struct TriggerRequest {
  std::optional<uint32_t> camera_id;  // absent ⇒ every running camera
  int skip_frames = 0;  // frames discarded per camera before the kept one
  bool send_image = true;
};

// What the server resolved a request into.
struct TriggerPlan {
  std::vector<uint32_t> camera_ids;
  int skip_frames = 0;
  bool send_image = true;
  // What the process mode's detector emits, stamped on every frame's header.
  DetectionKind detection_kind = DetectionKind::NONE;
};

// The `trigger_result` ack: one entry per camera that delivered, in delivery
// order (not camera_id order).
struct TriggerResult {
  struct Capture {
    uint32_t camera_id = 0;
    uint32_t frame_id = 0;
  };
  uint32_t trigger_id = 0;
  // Capture stopped while cameras were still to deliver; `captures` then
  // holds only those that did, so a waiting client is never left hanging.
  bool cancelled = false;
  std::vector<Capture> captures;
};

// The `trigger_result` message for `result`.
nlohmann::json triggerResultJson(const TriggerResult& result);

// Runs trigger_capture. A request arms every named camera; each discards
// `skip_frames` frames and keeps a copy of the next one. Once every camera has
// delivered, the ack goes out from the capture thread — the frames exist, so
// a robot may move — and a worker thread runs the process mode's detection
// on each frame and hands them over for delivery. Nothing is saved. At most
// one trigger is outstanding, from arm until its frames have been handed over.
class TriggerCapturer {
 public:
  // The process mode's detection on one captured frame (may take hundreds of
  // ms; runs on the worker thread).
  using DetectFn = std::function<std::vector<CornerSet>(const FrameData&)>;
  using AckFn = std::function<void(TriggerResult)>;
  using FramesFn = std::function<void(std::vector<CaptureFrame>)>;

  explicit TriggerCapturer(DetectFn detect);
  ~TriggerCapturer();

  TriggerCapturer(const TriggerCapturer&) = delete;
  TriggerCapturer& operator=(const TriggerCapturer&) = delete;

  // Arm a trigger. `on_ack` runs on the capture thread of the last camera to
  // deliver (or the canceller's) and must be cheap; `on_frames` runs on the
  // worker thread exactly once per trigger, with the frames in delivery order
  // — possibly none, when it was cancelled first. Returns the trigger id, or
  // nullopt when one is already outstanding or `plan` names no camera.
  std::optional<uint32_t> arm(TriggerPlan plan, AckFn on_ack, FramesFn on_frames);
  // Armed and still waiting for a camera's frame.
  bool hasPending() const;

  // Capture-thread entry point: one live frame from one camera.
  void onFrame(const FrameData& frame);

  // stop_cameras: resolve a trigger still waiting on cameras as cancelled,
  // delivering the frames already taken.
  void cancel();
  // start_cameras: restart trigger ids at 1.
  void reset();

 private:
  struct Pending {
    uint32_t id = 0;
    TriggerPlan plan;
    // camera → frames still to discard; a camera leaves once it has delivered.
    std::map<uint32_t, int> waiting;
    // Frames claimed under the lock and being copied outside it. The trigger
    // completes only once these have landed, so a cancel never acks a frame
    // it then fails to deliver.
    int copies_in_flight = 0;
    bool cancelled = false;
    TriggerResult result;
    std::vector<std::shared_ptr<const FrameData>> frames;
    AckFn on_ack;
    FramesFn on_frames;
  };
  // Detection for one finished trigger, handed to the worker.
  struct Job {
    TriggerPlan plan;
    std::vector<std::shared_ptr<const FrameData>> frames;
    FramesFn on_frames;
  };

  // With mutex_ held: whether `pending_` is finished, and if so move it out.
  std::unique_ptr<Pending> takeIfDone();
  // Ack and queue the detection of a trigger taken by takeIfDone.
  void complete(std::unique_ptr<Pending> p);
  void workerLoop();

  DetectFn detect_;

  mutable std::mutex mutex_;
  std::unique_ptr<Pending> pending_;
  // Mirrors `pending_ != nullptr` so the per-frame call is lock-free while
  // nothing is armed — every live frame of every camera comes through here.
  std::atomic<bool> armed_{false};
  uint32_t next_id_ = 1;

  std::condition_variable work_cv_;
  std::unique_ptr<Job> job_;  // guarded by mutex_
  bool stop_ = false;
  std::thread worker_;
};
