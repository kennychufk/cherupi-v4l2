#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/types.hpp>

#include "aruco_detector.h"
#include "checkerboard_detector.h"
#include "types.hpp"

// One detection's worth of corners, already mapped back to full-frame Y-plane
// pixel coordinates. Shared by the checkerboard and aruco paths.
//
// `set_id` encodes which sub-frame the corners came from: 0 for the whole-frame
// modes (`checkerboard` / `aruco`); 0..3 for the 2x2 modes (row*2 + col, where
// row/col are 0 for top/left and 1 for bottom/right).
//
// `marker_id` is the ArUco/AprilTag dictionary id for `aruco` / `aruco2x2`
// (each detected marker is its own set of 4 corners); it is -1 for the
// checkerboard modes, which have no id (the set holds rows×cols corners).
struct CornerSet {
  uint8_t set_id = 0;
  int marker_id = -1;
  std::vector<cv::Point2f> corners;
};

// Outcome of one TRIGGER-mode capture, handed to the client once every armed
// camera has delivered its frame. One entry per camera, in the order the
// cameras delivered (not camera_id order).
struct TriggerResult {
  struct Capture {
    uint32_t camera_id = 0;
    uint32_t frame_id = 0;
    // Path the frame was queued to be written to; empty when the trigger mode
    // is running with save_frames off (capture acknowledged, nothing written).
    std::string filename;
  };
  uint32_t trigger_id = 0;
  std::vector<Capture> captures;
  // Set when the trigger was abandoned before every armed camera delivered
  // (capture stopped while it was pending). `captures` then holds only the
  // cameras that made it, so a waiting client is never left hanging.
  bool cancelled = false;
};

class FrameProcessor {
 private:
  ProcessConfig config;
  std::atomic<bool> enabled{false};
  std::string actual_output_dir;  // The directory that will actually be used

  // For buffer mode
  std::vector<FrameData> buffered_frames;
  std::mutex buffer_mutex;

  // For batch and checkerboard modes
  struct WriteTask {
    std::string filename;
    std::vector<uint8_t> data;
    uint32_t camera_id;
    uint32_t frame_id;
  };

  std::queue<WriteTask> write_queue;
  // BATCH mode: frames accumulate here and are moved into write_queue in
  // groups of config.batch_size (handed to the writer pool). Guarded by
  // queue_mutex; any partial batch is flushed in stop().
  std::vector<WriteTask> pending_batch;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::vector<std::thread> writer_threads;
  std::atomic<bool> stop_threads{false};

  std::atomic<size_t> frames_saved{0};
  std::atomic<size_t> bytes_written{0};
  std::atomic<size_t> frames_checked{0};
  std::atomic<size_t> frames_detected{0};

  // --- Backlog accounting -------------------------------------------------
  // Bytes currently held by frames that have been accepted but not yet written
  // out: BUFFER's `buffered_frames` plus the writer pool's `write_queue` and
  // `pending_batch`. Incremented when a frame is accepted, decremented when it
  // leaves. Atomic so the check on the capture thread needs no extra lock.
  std::atomic<size_t> backlog_bytes{0};
  // Budget resolved in start(): config.backlog_max_bytes, or a default derived
  // from MemAvailable when that is 0.
  std::atomic<size_t> backlog_budget{0};
  // Frames refused because accepting them would have exceeded the budget.
  std::atomic<size_t> frames_dropped_backlog{0};
  // Latches on the first refusal so the warning is logged once per session
  // rather than once per dropped frame.
  std::atomic<bool> backlog_warned{false};

  // Reserve `bytes` against the budget. Returns false when the frame does not
  // fit, in which case nothing is reserved, the drop is counted, and the first
  // such refusal is logged. Keeping the earliest contiguous run of frames (as
  // opposed to evicting the oldest) is what makes the flush on stop useful:
  // a prefix of a capture is a usable dataset, a ring of survivors is not.
  bool reserveBacklog(size_t bytes);
  // Return `bytes` to the budget once the frame has been written or dropped.
  void releaseBacklog(size_t bytes);

  // Per-camera frame saving counters
  std::unordered_map<uint32_t, std::atomic<uint32_t>> frames_saved_per_camera;
  // std::mutex is not used to protect the map because the numbers do not need
  // to be so precise

  // For checkerboard detection
  std::unique_ptr<CheckerboardDetector> checkerboard_detector;
  // For aruco detection (aruco / aruco2x2 modes)
  std::unique_ptr<ArucoDetector> aruco_detector;

  // Best-effort async checkerboard detection. Detection is CPU-intensive
  // (tens–hundreds of ms), so it cannot keep up with a high capture rate.
  // saveFrame() (called on the capture thread) drops the frame into
  // `pending_detection` — a per-camera latest-wins slot — and returns
  // immediately; a single worker thread pulls the most recent pending frame
  // per camera, runs detection, and drops the result into
  // `detected_for_stream`. Frames that arrive while the worker is busy
  // overwrite the pending slot and are silently skipped, exactly like the
  // streamer drops older frames under backpressure.
  std::map<uint32_t, FrameData> pending_detection;  // ordered → round-robin
  std::mutex pending_mutex;
  std::condition_variable pending_cv;
  std::thread detection_thread;
  bool stop_detection = false;  // guarded by pending_mutex
  // Round-robin cursor so no camera starves the others (worker thread only).
  uint32_t last_detected_camera_ = std::numeric_limits<uint32_t>::max();

  // Latest detector-processed frame per camera, awaiting streaming. Written by
  // the detection worker, read by the streamers via
  // getDetectedFrameForStreaming(). Multi-consumer: each streamer (commander,
  // observer) keeps its own cursor against `seq`, which is drawn from one
  // counter that is never reset — so clearing the map between sessions can
  // never rewind a cursor — and the frame is shared read-only rather than
  // moved out. Latest-wins: an earlier frame no streamer picked up is
  // overwritten (dropped).
  struct DetectedFrame {
    std::shared_ptr<const FrameData> frame;
    std::vector<CornerSet> sets;
    uint64_t seq = 0;
  };
  std::unordered_map<uint32_t, DetectedFrame> detected_for_stream;
  std::mutex detected_mutex;
  uint64_t detected_seq_counter_ = 0;  // guarded by detected_mutex

  // TRIGGER mode: a capture request arms every camera named in the request and
  // stays pending until each of them has delivered a frame; only those frames
  // are written. At most one trigger is outstanding at a time (a second
  // request while one is pending is rejected), which keeps the robot loop
  // request↔ack correspondence unambiguous.
  struct PendingTrigger {
    uint32_t id = 0;
    // camera_id → frames still to be discarded before this camera's capture
    // (from the request's skip_frames). A camera is done once it is erased.
    std::map<uint32_t, int> waiting;
    TriggerResult result;
    std::function<void(TriggerResult)> on_complete;
  };
  std::unique_ptr<PendingTrigger> pending_trigger;  // guarded by trigger_mutex
  std::mutex trigger_mutex;
  uint32_t next_trigger_id = 1;  // guarded by trigger_mutex

  // Capture-thread half of TRIGGER mode: consume one delivered frame against
  // the pending trigger, queueing a write task when the frame is the one that
  // was asked for. Fires the completion callback (outside the lock) once the
  // last armed camera has been served.
  void handleTriggerFrame(const FrameData& frame);

  void writerThreadFunc();
  // Worker-thread loop: pull pending frames (round-robin per camera), run
  // detection, and publish results. Drains all pending frames on stop.
  void detectionWorkerFunc();
  // Run detection on one frame, enqueue a write task if a detection was found
  // and config.save_frames is set, and publish {frame, corner sets} into
  // detected_for_stream (always, regardless of save_frames — this is what feeds
  // the streamer). Runs on the detection worker thread; takes the frame by
  // value so it can be moved into the streaming slot.
  void processDetection(FrameData frame);
  std::string generateFilename(uint32_t camera_id, uint32_t frame_id);
  // cornerSubPix-refine every set's corners on the full-res Y plane per
  // checkerboard_subpix_window. Returns the largest window half-size used.
  int refineCheckerboardCorners(const FrameData& frame,
                                std::vector<CornerSet>& sets);
  // Detect on the whole Y plane (extracted per `checkerboard_full_res_detection`).
  // On success, populates `out_sets` with one CornerSet (set_id=0) whose
  // coordinates are translated to full-frame Y-plane pixel space, then
  // refined by refineCheckerboardCorners.
  bool detectCheckerboard(const FrameData& frame,
                          std::vector<CornerSet>& out_sets);
  // CHECKERBOARD2X2: split the (already full-res or subsampled per
  // `checkerboard_full_res_detection`) Y plane into 4 equal quadrants and run
  // detection on each in parallel; returns true if any sub-frame detects.
  // `out_sets` receives one CornerSet per detecting quadrant, set_id = row*2+col,
  // coordinates translated to full-frame Y-plane pixel space.
  bool detectCheckerboard2x2(const FrameData& frame,
                             std::vector<CornerSet>& out_sets);
  // ARUCO: detect markers on the whole Y plane (extracted per
  // `aruco_full_res_detection`). `out_sets` receives one CornerSet per detected
  // marker (set_id=0, marker_id=the id, 4 corners), coordinates translated to
  // full-frame Y-plane pixel space. Returns true if any marker was found.
  bool detectAruco(const FrameData& frame, std::vector<CornerSet>& out_sets);
  // ARUCO2X2: split the Y plane into 4 equal quadrants and detect markers on
  // each in parallel (batched by `aruco_num_threads`, clamped to [1, 4]).
  // `out_sets` receives one CornerSet per detected marker (set_id = row*2+col,
  // marker_id = the id), coordinates translated to full-frame Y-plane pixels.
  // Returns true if any quadrant detected at least one marker.
  bool detectAruco2x2(const FrameData& frame, std::vector<CornerSet>& out_sets);
  bool createOutputDirectory();  // New method for directory creation

 public:
  FrameProcessor() = default;
  ~FrameProcessor();

  void configure(const ProcessConfig& cfg);
  void start();
  void stop();

  // Called for each captured frame
  void saveFrame(const FrameData& frame);

  // TRIGGER mode: arm a capture across `camera_ids` (all of them must deliver a
  // frame before the trigger completes). `skip_frames` frames are discarded per
  // camera first — 0 means "save the very next frame each camera delivers".
  // `on_complete` runs on the capture thread of the last camera to be served,
  // so it must be cheap and thread-safe (the server marshals the ack back onto
  // the uWS loop). Returns the new trigger's id, or std::nullopt when the
  // processor is not in an armable state: not enabled, not TRIGGER mode,
  // `camera_ids` empty, or a trigger is already pending.
  std::optional<uint32_t> armTrigger(const std::vector<uint32_t>& camera_ids,
                                     int skip_frames,
                                     std::function<void(TriggerResult)> on_complete);

  // Whether a trigger is currently armed and waiting for frames.
  bool hasPendingTrigger();

  // Default per-camera skip count from the configured ProcessConfig, used when
  // a trigger_capture request does not override it.
  int getTriggerSkipFrames() const { return config.trigger_skip_frames; }

  // For buffer mode - write all buffered frames to disk
  void flushBufferedFrames();

  size_t getFramesSaved() const { return frames_saved; }
  size_t getBytesWritten() const { return bytes_written; }
  size_t getFramesChecked() const { return frames_checked; }
  size_t getDetections() const { return frames_detected; }
  // Bytes of captured frames currently held in RAM awaiting write/flush.
  size_t getBacklogBytes() const { return backlog_bytes; }
  // The budget in force for this session (resolved in start()).
  size_t getBacklogBudgetBytes() const { return backlog_budget; }
  // Frames refused because the backlog budget was full. Non-zero means the
  // capture outran its sink and the recording has gaps.
  size_t getFramesDroppedForBacklog() const { return frames_dropped_backlog; }
  bool isEnabled() const { return enabled; }
  ProcessMode getMode() const { return config.mode; }
  // The mode's full configuration, for callers that need to reason about its
  // resource demands (the start_cameras feasibility check).
  const ProcessConfig& getConfig() const { return config; }
  const std::string& getActualOutputDir() const { return actual_output_dir; }

  // Streamer-facing: hand out the most recent detector-processed frame for
  // `camera_id` if it is newer than the caller's cursor `last_seen_seq` (0 for
  // a fresh consumer). On success fills `frame` (shared, read-only) and `sets`
  // (the corners found on it, possibly empty when detection ran but found no
  // board), advances the cursor, and returns true — so each consumer streams a
  // given processed frame at most once. Returns false when nothing newer is
  // available (no detection has completed since the caller's last take, or the
  // process mode isn't a detector mode). This is the ONLY source of streamable
  // frames in detector modes, so undetected frames never reach a client.
  // Safe to call from any streaming thread.
  bool getDetectedFrameForStreaming(uint32_t camera_id, uint64_t& last_seen_seq,
                                    std::shared_ptr<const FrameData>& frame,
                                    std::vector<CornerSet>& sets) {
    std::lock_guard<std::mutex> lock(detected_mutex);
    auto it = detected_for_stream.find(camera_id);
    if (it == detected_for_stream.end() || !it->second.frame ||
        it->second.seq <= last_seen_seq) {
      return false;
    }
    frame = it->second.frame;
    sets = it->second.sets;
    last_seen_seq = it->second.seq;
    return true;
  }

  // Get saved frame count for specific camera
  uint32_t getFramesSavedForCamera(uint32_t camera_id) {
    auto it = frames_saved_per_camera.find(camera_id);
    return it != frames_saved_per_camera.end() ? it->second.load() : 0;
  }

  // Reset saved frame counts for all cameras
  void resetFramesSavedCounts() {
    for (auto& pair : frames_saved_per_camera) {
      pair.second = 0;
    }
    // Per-camera frame_id counters reset alongside this. Drop any pending or
    // already-processed frame still carrying a pre-reset frame_id so the
    // streamer doesn't emit one after the counters have been zeroed.
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      pending_detection.clear();
    }
    {
      std::lock_guard<std::mutex> lock(detected_mutex);
      detected_for_stream.clear();
    }
    LOG_INFO("FrameProcessor", "Reset per-camera saved frame counts");
  }
};
