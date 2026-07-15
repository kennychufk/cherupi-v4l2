#pragma once

#include <atomic>
#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
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

class FrameSaver {
 private:
  SaveConfig config;
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
  std::atomic<size_t> checkerboards_detected{0};

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
  // the detection worker, drained by the streamer via
  // takeDetectedFrameForStreaming(). `fresh` is set on publish and cleared on
  // take so each processed frame streams at most once; latest-wins means an
  // unconsumed earlier frame is overwritten (dropped).
  struct DetectedFrame {
    FrameData frame;
    std::vector<CornerSet> sets;
    bool fresh = false;
  };
  std::unordered_map<uint32_t, DetectedFrame> detected_for_stream;
  std::mutex detected_mutex;

  void writerThreadFunc();
  // Worker-thread loop: pull pending frames (round-robin per camera), run
  // detection, and publish results. Drains all pending frames on stop.
  void detectionWorkerFunc();
  // Run detection on one frame, enqueue a write task if a board was found,
  // and publish {frame, corner sets} into detected_for_stream. Runs on the
  // detection worker thread; takes the frame by value so it can be moved into
  // the streaming slot.
  void processDetection(FrameData frame);
  std::string generateFilename(uint32_t camera_id, uint32_t frame_id);
  // Detect on the whole Y plane (extracted per `checkerboard_full_res_detection`).
  // On success, populates `out_sets` with one CornerSet (set_id=0) whose
  // coordinates are translated to full-frame Y-plane pixel space.
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
  FrameSaver() = default;
  ~FrameSaver();

  void configure(const SaveConfig& cfg);
  void start();
  void stop();

  // Called for each captured frame
  void saveFrame(const FrameData& frame);

  // For buffer mode - write all buffered frames to disk
  void flushBufferedFrames();

  size_t getFramesSaved() const { return frames_saved; }
  size_t getBytesWritten() const { return bytes_written; }
  size_t getFramesChecked() const { return frames_checked; }
  size_t getCheckerboardsDetected() const { return checkerboards_detected; }
  bool isEnabled() const { return enabled; }
  SaveMode getMode() const { return config.mode; }
  const std::string& getActualOutputDir() const { return actual_output_dir; }

  // Streamer-facing: hand off the most recent detector-processed frame for
  // `camera_id`, if one is waiting. On success fills `frame` and `sets` (the
  // corners found on it, possibly empty when detection ran but found no board)
  // and returns true; the slot is then marked consumed so the same frame is
  // not streamed twice. Returns false when no fresh processed frame is
  // available (no detection has completed since the last take, or the save
  // mode isn't a detector mode). This is the ONLY source of streamable frames
  // in detector modes, so undetected frames never reach the client.
  // Safe to call from the streaming thread.
  bool takeDetectedFrameForStreaming(uint32_t camera_id, FrameData& frame,
                                     std::vector<CornerSet>& sets) {
    std::lock_guard<std::mutex> lock(detected_mutex);
    auto it = detected_for_stream.find(camera_id);
    if (it == detected_for_stream.end() || !it->second.fresh) return false;
    // Consume: move the frame/corners out (the slot is marked stale, so the
    // moved-from state is never read before the worker republishes).
    frame = std::move(it->second.frame);
    sets = std::move(it->second.sets);
    it->second.fresh = false;
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
    LOG_INFO("FrameSaver", "Reset per-camera saved frame counts");
  }
};
