#include "frame_processor.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "capacity.hpp"
#include "frame_processor_helpers.hpp"

FrameProcessor::~FrameProcessor() { stop(); }

void FrameProcessor::configure(const ProcessConfig& cfg) {
  config = cfg;

  // Create the output directory only when frames will actually be written.
  // Skipped for NONE and for any mode with save_frames off (detection-only) —
  // no directory should appear on disk when nothing is saved.
  if (config.mode != ProcessMode::NONE && config.save_frames) {
    if (!createOutputDirectory()) {
      LOG_WARN("FrameProcessor",
               "Failed to create output directory, falling back to current "
               "directory");
      actual_output_dir = ".";
    }
  }

  // Initialize checkerboard detector if needed
  if (config.mode == ProcessMode::CHECKERBOARD ||
      config.mode == ProcessMode::CHECKERBOARD2X2) {
    checkerboard_detector = std::make_unique<CheckerboardDetector>(
        config.checkerboard_cols, config.checkerboard_rows);
  }

  // Initialize aruco detector if needed
  if (config.mode == ProcessMode::ARUCO || config.mode == ProcessMode::ARUCO2X2) {
    aruco_detector = std::make_unique<ArucoDetector>(
        config.aruco_corner_refine, config.aruco_edge_refine,
        config.aruco_edge_gamma);
  }
}

bool FrameProcessor::createOutputDirectory() {
  try {
    std::string trimmed_dir =
        frame_processor_helpers::normalizeBaseDir(config.output_dir);
    if (trimmed_dir == "." && config.output_dir.find_first_not_of(" \t\n\r") ==
                                  std::string::npos) {
      LOG_WARN("FrameProcessor",
               "Empty output directory specified, using current directory");
      actual_output_dir = ".";
      return true;
    }

    std::string final_dir = trimmed_dir;
    if (config.prepend_timestamp_to_dir) {
      final_dir = frame_processor_helpers::makeTimestampedDir(
          trimmed_dir, std::chrono::system_clock::now());
      LOG_INFO("FrameProcessor", "Timestamp prepended to directory: " +
                                 trimmed_dir + " -> " + final_dir);
    }

    // Use the filesystem library for robust directory creation
    std::filesystem::path dir_path(final_dir);

    // Check if directory already exists
    if (std::filesystem::exists(dir_path)) {
      if (std::filesystem::is_directory(dir_path)) {
        LOG_INFO("FrameProcessor", "Output directory already exists: " + final_dir);
        actual_output_dir = final_dir;
        return true;
      } else {
        LOG_ERROR("FrameProcessor",
                  "Output path exists but is not a directory: " + final_dir);
        return false;
      }
    }

    // Create the directory
    if (std::filesystem::create_directory(dir_path)) {
      LOG_INFO("FrameProcessor", "Created output directory: " + final_dir);

      // Set permissions to 0755 using chmod
      if (chmod(final_dir.c_str(),
                S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0) {
        LOG_WARN("FrameProcessor", "Failed to set directory permissions to 0755");
      }

      actual_output_dir = final_dir;
      return true;
    } else {
      LOG_ERROR("FrameProcessor",
                "Failed to create output directory: " + final_dir);
      return false;
    }
  } catch (const std::filesystem::filesystem_error& e) {
    LOG_ERROR("FrameProcessor",
              "Filesystem error creating directory: " + std::string(e.what()));
    return false;
  } catch (const std::exception& e) {
    LOG_ERROR("FrameProcessor",
              "Error creating output directory: " + std::string(e.what()));
    return false;
  }
}

void FrameProcessor::start() {
  if (config.mode == ProcessMode::NONE) {
    enabled = false;
    return;
  }

  enabled = true;
  stop_threads = false;
  frames_checked = 0;
  frames_detected = 0;

  // Resolve the backlog budget for this session. Without a bound here, a mode
  // that retains or writes every frame grows until the kernel OOM-kills the
  // server mid-capture; with one, it drops frames and keeps running.
  backlog_bytes = 0;
  frames_dropped_backlog = 0;
  backlog_warned = false;
  const size_t budget =
      config.backlog_max_bytes > 0
          ? config.backlog_max_bytes
          : capacity::defaultBacklogBudgetBytes(capacity::availableMemoryBytes());
  backlog_budget = budget;
  LOG_INFO("FrameProcessor",
           "Backlog budget: " + capacity::formatBytes(budget) +
               (config.backlog_max_bytes > 0 ? " (configured)"
                                             : " (derived from MemAvailable)"));

  // Reset the session totals along with the per-camera counters: the
  // frames_saved / bytes_written reported by stop_cameras describe this
  // capture session, not the life of the process.
  frames_saved = 0;
  bytes_written = 0;
  frames_saved_per_camera.clear();

  // Clear best-effort detection slots left over from a previous session.
  {
    std::lock_guard<std::mutex> lock(pending_mutex);
    pending_detection.clear();
    stop_detection = false;
    last_detected_camera_ = std::numeric_limits<uint32_t>::max();
  }
  {
    std::lock_guard<std::mutex> lock(detected_mutex);
    detected_for_stream.clear();
  }

  if (config.mode == ProcessMode::BUFFER) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.clear();
    // The previous flat reserve(10000) encoded an intent -- 10000 frames --
    // that is 179 GB at 4608x2592. The budget resolved above is the real
    // limit, so start small and let growth be bounded by it instead.
    buffered_frames.reserve(64);
  } else if (usesWriterPool(config.mode)) {
    // The writer pool exists only to persist frames. When save_frames is off we
    // skip it entirely: a detector mode then runs detection solely to stream
    // side-products, with no disk I/O and no per-detection frame copy.
    if (config.save_frames) {
      // Drop any partial batch left over from a previous session.
      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        pending_batch.clear();
      }
      // Start writer threads
      for (size_t i = 0; i < config.writer_threads; i++) {
        writer_threads.emplace_back(&FrameProcessor::writerThreadFunc, this);
      }
    }
    // Detector modes (checkerboard/aruco) run one async worker that does the
    // CPU-heavy detection off the capture thread. It runs whether or not the
    // detected frames are also written to disk, because it is the sole source
    // of streamable frames (and their corners/markers) in a detector mode.
    if (isDetectorMode(config.mode)) {
      detection_thread = std::thread(&FrameProcessor::detectionWorkerFunc, this);
    }
  }
}

void FrameProcessor::stop() {
  enabled = false;

  // Detector modes: shut down the detection worker FIRST. Its drain pass
  // detects every still-pending frame and enqueues any resulting write task,
  // so all detections land in write_queue before the writer pool below is
  // told to stop. Tearing the writers down first could lose a late detection.
  if (isDetectorMode(config.mode)) {
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      stop_detection = true;
    }
    pending_cv.notify_all();
    if (detection_thread.joinable()) detection_thread.join();
    // Drop the streamer-facing slots, as Camera::stop() does for its own: a
    // streamer that subscribes after the stop (an observer waiting for the
    // next start_cameras) must not be handed a stale pre-stop frame. The
    // sequence counter is left alone so existing cursors stay valid.
    {
      std::lock_guard<std::mutex> lock(detected_mutex);
      detected_for_stream.clear();
    }
  }

  if (usesWriterPool(config.mode) && config.save_frames) {
    // Flush any frames still held in a partial batch (BATCH mode) so they are
    // written before the workers exit, then signal threads to stop. Only
    // reached when save_frames is on, i.e. when the writer pool was started.
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      for (auto& t : pending_batch) write_queue.push(std::move(t));
      pending_batch.clear();
      stop_threads = true;
    }
    queue_cv.notify_all();

    // Wait for threads to finish
    for (auto& thread : writer_threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    writer_threads.clear();
  }

  // Log detection statistics (checkerboard/aruco modes)
  if (isDetectorMode(config.mode) && frames_checked > 0) {
    std::cout << "Detection stats: " << frames_detected
              << " detected out of " << frames_checked << " checked ("
              << (100.0 * frames_detected / frames_checked) << "%)"
              << std::endl;
  }
}

bool FrameProcessor::reserveBacklog(size_t bytes) {
  const size_t budget = backlog_budget.load();
  if (budget == 0) return true;  // unbounded (start() not run); nothing to gate

  // Compare-exchange rather than a plain fetch_add so two capture threads
  // cannot both see room and jointly overshoot the budget.
  size_t held = backlog_bytes.load(std::memory_order_relaxed);
  while (held + bytes <= budget) {
    if (backlog_bytes.compare_exchange_weak(held, held + bytes,
                                            std::memory_order_acq_rel,
                                            std::memory_order_relaxed)) {
      return true;
    }
  }

  frames_dropped_backlog.fetch_add(1, std::memory_order_relaxed);
  if (!backlog_warned.exchange(true)) {
    LOG_WARN("FrameProcessor",
             "Backlog budget of " + capacity::formatBytes(budget) +
                 " reached (" + capacity::formatBytes(held) + " held, " +
                 capacity::formatBytes(bytes) +
                 " per frame): capture is outrunning its sink. Keeping the "
                 "frames already accepted and dropping new ones from here on, "
                 "so the recording will have a gap. Further drops are counted, "
                 "not logged.");
  }
  return false;
}

void FrameProcessor::releaseBacklog(size_t bytes) {
  if (bytes == 0) return;
  // Saturating subtract via CAS, not load-then-fetch_sub: several writer
  // threads release concurrently, so a stale `held` could over-subtract and
  // wrap the counter — which would silently disable the budget (a huge value
  // never satisfies `held + bytes <= budget`, so every later frame is dropped)
  // for the rest of the session.
  size_t held = backlog_bytes.load(std::memory_order_relaxed);
  while (!backlog_bytes.compare_exchange_weak(
      held, held > bytes ? held - bytes : 0, std::memory_order_acq_rel,
      std::memory_order_relaxed)) {
  }
}

void FrameProcessor::saveFrame(const FrameData& frame) {
  saveFrame(std::make_shared<const FrameData>(frame));
}

void FrameProcessor::saveFrame(const std::shared_ptr<const FrameData>& shared) {
  if (!enabled) return;
  const FrameData& frame = *shared;

  if (config.mode == ProcessMode::BUFFER) {
    if (!config.save_frames) return;  // nothing to persist → nothing to buffer
    // BUFFER never drains before stop(), so the budget is its whole capacity.
    if (!reserveBacklog(frame.data.size())) return;
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.push_back(frame);
  } else if (config.mode == ProcessMode::BATCH) {
    if (!config.save_frames) return;  // no writer pool started
    // The writer pool drains continuously, so the budget only bites when the
    // disk cannot keep up with capture -- exactly the case that used to grow
    // write_queue without limit until the box died.
    if (!reserveBacklog(frame.data.size())) return;
    WriteTask task;
    task.filename = generateFilename(frame.camera_id, frame.frame_id);
    task.data = frame.data;
    task.camera_id = frame.camera_id;
    task.frame_id = frame.frame_id;

    // Accumulate frames and hand them to the writer pool in groups of
    // batch_size (at least 1). The partial final batch is flushed in stop().
    const size_t batch_size = std::max<size_t>(config.batch_size, 1);
    bool flushed = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      pending_batch.push_back(std::move(task));
      if (pending_batch.size() >= batch_size) {
        for (auto& t : pending_batch) write_queue.push(std::move(t));
        pending_batch.clear();
        flushed = true;
      }
    }
    if (flushed) queue_cv.notify_all();
  } else if (isDetectorMode(config.mode)) {
    // Best-effort: hand the frame to the async detection worker instead of
    // detecting inline. Detection takes tens–hundreds of ms — far longer than
    // the inter-frame interval — so running it on the capture thread would
    // stall the whole pipeline. Latest-wins per camera: if the worker is still
    // busy, the previously-pending frame for this camera is overwritten
    // (dropped) so the detector only ever works on the freshest frame. The
    // slot shares the camera's frame rather than copying it: this is
    // libcamera's completion thread (see Camera::onRequestComplete).
    std::shared_ptr<const FrameData> dropped;
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      dropped = std::exchange(pending_detection[frame.camera_id], shared);
    }
    pending_cv.notify_one();
  }
}

void FrameProcessor::detectionWorkerFunc() {
  while (true) {
    std::shared_ptr<const FrameData> frame;
    {
      std::unique_lock<std::mutex> lock(pending_mutex);
      pending_cv.wait(lock, [this] {
        return !pending_detection.empty() || stop_detection;
      });
      if (pending_detection.empty()) {
        // Nothing left to detect; only exit once a stop has been requested so
        // that stop() drains every frame queued before it was called.
        if (stop_detection) break;
        continue;
      }
      // Round-robin across cameras (ordered map) so a fast camera can't starve
      // the others of detection slots.
      auto it = pending_detection.upper_bound(last_detected_camera_);
      if (it == pending_detection.end()) it = pending_detection.begin();
      last_detected_camera_ = it->first;
      frame = std::move(it->second);
      pending_detection.erase(it);
    }
    processDetection(std::move(frame));
  }
}

void FrameProcessor::processDetection(std::shared_ptr<const FrameData> shared) {
  frames_checked++;
  const FrameData& frame = *shared;

  std::vector<CornerSet> sets;
  if (runDetector(frame, sets)) {
    frames_detected++;

    // Persist the detected frame only when saving is enabled. When it is off we
    // skip building the WriteTask entirely — that avoids copying the multi-MB
    // frame payload into the write queue on every detection, which is the main
    // win of the no-save detection pipeline (the frame itself is only shared
    // into the streaming slot below).
    if (config.save_frames && reserveBacklog(frame.data.size())) {
      WriteTask task;
      task.filename = generateFilename(frame.camera_id, frame.frame_id);
      task.data = frame.data;  // the frame itself is shared into the stream slot below
      task.camera_id = frame.camera_id;
      task.frame_id = frame.frame_id;

      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        write_queue.push(std::move(task));
      }
      queue_cv.notify_one();
    }
  }

  // Publish the processed frame plus its corners (possibly empty) for the
  // streamer. Only frames that reach this point — i.e. that the detector
  // actually ran on — are ever eligible to stream. Latest-wins per camera:
  // an earlier processed frame the streamer never picked up is overwritten.
  const uint32_t camera_id = frame.camera_id;
  {
    std::lock_guard<std::mutex> lock(detected_mutex);
    DetectedFrame& df = detected_for_stream[camera_id];
    df.frame = std::move(shared);
    df.sets = std::move(sets);
    df.seq = ++detected_seq_counter_;
  }
}

bool FrameProcessor::runDetector(const FrameData& frame,
                                 std::vector<CornerSet>& out_sets) {
  switch (config.mode) {
    case ProcessMode::CHECKERBOARD:
      return detectCheckerboard(frame, out_sets);
    case ProcessMode::CHECKERBOARD2X2:
      return detectCheckerboard2x2(frame, out_sets);
    case ProcessMode::ARUCO:
      return detectAruco(frame, out_sets);
    case ProcessMode::ARUCO2X2:
      return detectAruco2x2(frame, out_sets);
    default:
      out_sets.clear();
      return false;
  }
}

std::vector<CornerSet> FrameProcessor::detectCaptured(const FrameData& frame) {
  std::vector<CornerSet> sets;
  if (isDetectorMode(config.mode)) runDetector(frame, sets);
  return sets;
}

namespace {

int64_t elapsedUs(std::chrono::steady_clock::time_point from,
                  std::chrono::steady_clock::time_point to) {
  return std::chrono::duration_cast<std::chrono::microseconds>(to - from)
      .count();
}

}  // namespace

int FrameProcessor::refineCheckerboardCorners(const FrameData& frame,
                                              std::vector<CornerSet>& sets) {
  // Refine against the full-resolution Y plane — the first plane of the
  // YUV420 buffer — whatever resolution detection ran at, viewed in place.
  if (config.checkerboard_subpix_window <= 0 || frame.data.empty()) return 0;
  int window = 0;
  for (auto& set : sets) {
    window = std::max(window, checkerboard_detector->refine(
                                  frame.data.data(), static_cast<int>(frame.width),
                                  static_cast<int>(frame.height),
                                  frame.bytes_per_line, set.corners,
                                  config.checkerboard_subpix_window));
  }
  return window;
}

bool FrameProcessor::detectCheckerboard(const FrameData& frame,
                                    std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_processor_helpers::extractYFromYUV420(
      frame, config.checkerboard_full_res_detection);
  int out_width =
      config.checkerboard_full_res_detection ? frame.width : frame.width / 2;
  int out_height =
      config.checkerboard_full_res_detection ? frame.height : frame.height / 2;

  std::vector<cv::Point2f> corners;
  bool detected = checkerboard_detector->detect(grayscale_data.data(),
                                                out_width, out_height,
                                                /*stride=*/0, &corners);

  if (detected) {
    // When detection ran on the subsampled Y plane, corner coordinates are in
    // (W/2, H/2) space; multiply by 2 to map back to full-frame Y pixels.
    const float scale = config.checkerboard_full_res_detection ? 1.0f : 2.0f;
    if (scale != 1.0f) {
      for (auto& p : corners) {
        p.x *= scale;
        p.y *= scale;
      }
    }
    CornerSet set;
    set.set_id = 0;
    set.corners = std::move(corners);
    out_sets.push_back(std::move(set));
  }

  auto detect_tp = std::chrono::steady_clock::now();
  const int refine_window = refineCheckerboardCorners(frame, out_sets);
  auto end_tp = std::chrono::steady_clock::now();
  LOG_INFO("FrameProcessor",
           "Checkerboard detected: " + std::to_string(detected) +
               " Detect: " + std::to_string(elapsedUs(start_tp, detect_tp)) +
               "us Refine: " + std::to_string(elapsedUs(detect_tp, end_tp)) +
               "us (window " + std::to_string(refine_window) + ") Total: " +
               std::to_string(elapsedUs(start_tp, end_tp)) + "us");
  return detected;
}

bool FrameProcessor::detectCheckerboard2x2(const FrameData& frame,
                                       std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_processor_helpers::extractYFromYUV420(
      frame, config.checkerboard_full_res_detection);
  const int full_width = config.checkerboard_full_res_detection
                             ? static_cast<int>(frame.width)
                             : static_cast<int>(frame.width / 2);
  const int full_height = config.checkerboard_full_res_detection
                              ? static_cast<int>(frame.height)
                              : static_cast<int>(frame.height / 2);
  const int sub_w = full_width / 2;
  const int sub_h = full_height / 2;
  if (sub_w <= 0 || sub_h <= 0) return false;

  // Subsampled-mode coords need a final ×2 to land in full-frame Y pixels.
  const float scale = config.checkerboard_full_res_detection ? 1.0f : 2.0f;

  // Per-quadrant detection job. Each job writes its own slot in `results` so
  // we never share a mutable vector across threads.
  struct QuadrantResult {
    bool detected = false;
    std::vector<cv::Point2f> corners;
  };
  std::array<QuadrantResult, 4> results;

  const uint8_t* base = grayscale_data.data();
  auto run_detect = [&](int row, int col, QuadrantResult* out) {
    const uint8_t* src =
        base + static_cast<size_t>(row) * sub_h * full_width +
        static_cast<size_t>(col) * sub_w;
    out->detected = checkerboard_detector->detect(
        src, sub_w, sub_h, static_cast<size_t>(full_width), &out->corners);
  };

  constexpr std::array<std::pair<int, int>, 4> kQuadrants = {
      {{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
  const size_t pool_size =
      std::clamp<size_t>(config.checkerboard_num_threads, 1, 4);

  for (size_t i = 0; i < kQuadrants.size(); i += pool_size) {
    const size_t batch = std::min(pool_size, kQuadrants.size() - i);
    if (batch == 1) {
      run_detect(kQuadrants[i].first, kQuadrants[i].second, &results[i]);
    } else {
      std::vector<std::future<void>> futures;
      futures.reserve(batch);
      for (size_t k = 0; k < batch; ++k) {
        const auto& q = kQuadrants[i + k];
        QuadrantResult* slot = &results[i + k];
        futures.emplace_back(
            std::async(std::launch::async, run_detect, q.first, q.second, slot));
      }
      for (auto& f : futures) f.get();
    }
  }

  bool detected = false;
  for (size_t i = 0; i < kQuadrants.size(); ++i) {
    if (!results[i].detected) continue;
    detected = true;
    const int row = kQuadrants[i].first;
    const int col = kQuadrants[i].second;
    // Translate quadrant-local cv coords → full-Y-plane → full-frame pixels.
    const float ox = static_cast<float>(col * sub_w);
    const float oy = static_cast<float>(row * sub_h);
    CornerSet set;
    set.set_id = static_cast<uint8_t>(row * 2 + col);
    set.corners.reserve(results[i].corners.size());
    for (const auto& p : results[i].corners) {
      set.corners.emplace_back((p.x + ox) * scale, (p.y + oy) * scale);
    }
    out_sets.push_back(std::move(set));
  }

  auto detect_tp = std::chrono::steady_clock::now();
  const int refine_window = refineCheckerboardCorners(frame, out_sets);
  auto end_tp = std::chrono::steady_clock::now();
  LOG_INFO("FrameProcessor",
           "Checkerboard2x2 detected: " + std::to_string(detected) +
               " Detect: " + std::to_string(elapsedUs(start_tp, detect_tp)) +
               "us Refine: " + std::to_string(elapsedUs(detect_tp, end_tp)) +
               "us (window " + std::to_string(refine_window) + ") Total: " +
               std::to_string(elapsedUs(start_tp, end_tp)) + "us");
  return detected;
}

bool FrameProcessor::detectAruco(const FrameData& frame,
                             std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_processor_helpers::extractYFromYUV420(
      frame, config.aruco_full_res_detection);
  int out_width =
      config.aruco_full_res_detection ? frame.width : frame.width / 2;
  int out_height =
      config.aruco_full_res_detection ? frame.height : frame.height / 2;

  std::vector<ArucoDetector::Marker> markers;
  bool detected = aruco_detector->detect(grayscale_data.data(), out_width,
                                         out_height, /*stride=*/0, &markers);

  // When detection ran on the subsampled Y plane, corner coordinates are in
  // (W/2, H/2) space; multiply by 2 to map back to full-frame Y pixels.
  const float scale = config.aruco_full_res_detection ? 1.0f : 2.0f;
  for (auto& m : markers) {
    CornerSet set;
    set.set_id = 0;  // whole frame
    set.marker_id = m.id;
    set.corners.reserve(m.corners.size());
    for (const auto& p : m.corners) {
      set.corners.emplace_back(p.x * scale, p.y * scale);
    }
    out_sets.push_back(std::move(set));
  }

  auto end_tp = std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  LOG_INFO("FrameProcessor", "Aruco markers: " + std::to_string(markers.size()) +
                             " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

bool FrameProcessor::detectAruco2x2(const FrameData& frame,
                                std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_processor_helpers::extractYFromYUV420(
      frame, config.aruco_full_res_detection);
  const int full_width = config.aruco_full_res_detection
                             ? static_cast<int>(frame.width)
                             : static_cast<int>(frame.width / 2);
  const int full_height = config.aruco_full_res_detection
                              ? static_cast<int>(frame.height)
                              : static_cast<int>(frame.height / 2);
  const int sub_w = full_width / 2;
  const int sub_h = full_height / 2;
  if (sub_w <= 0 || sub_h <= 0) return false;

  // Subsampled-mode coords need a final ×2 to land in full-frame Y pixels.
  const float scale = config.aruco_full_res_detection ? 1.0f : 2.0f;

  // Per-quadrant detection job. Each job writes its own slot in `results` so
  // we never share a mutable vector across threads (the ArucoDetector itself
  // is read-only under detectMarkers, so a single instance is safe to share).
  struct QuadrantResult {
    std::vector<ArucoDetector::Marker> markers;
  };
  std::array<QuadrantResult, 4> results;

  const uint8_t* base = grayscale_data.data();
  auto run_detect = [&](int row, int col, QuadrantResult* out) {
    const uint8_t* src =
        base + static_cast<size_t>(row) * sub_h * full_width +
        static_cast<size_t>(col) * sub_w;
    aruco_detector->detect(src, sub_w, sub_h, static_cast<size_t>(full_width),
                           &out->markers);
  };

  constexpr std::array<std::pair<int, int>, 4> kQuadrants = {
      {{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
  const size_t pool_size = std::clamp<size_t>(config.aruco_num_threads, 1, 4);

  for (size_t i = 0; i < kQuadrants.size(); i += pool_size) {
    const size_t batch = std::min(pool_size, kQuadrants.size() - i);
    if (batch == 1) {
      run_detect(kQuadrants[i].first, kQuadrants[i].second, &results[i]);
    } else {
      std::vector<std::future<void>> futures;
      futures.reserve(batch);
      for (size_t k = 0; k < batch; ++k) {
        const auto& q = kQuadrants[i + k];
        QuadrantResult* slot = &results[i + k];
        futures.emplace_back(
            std::async(std::launch::async, run_detect, q.first, q.second, slot));
      }
      for (auto& f : futures) f.get();
    }
  }

  bool detected = false;
  for (size_t i = 0; i < kQuadrants.size(); ++i) {
    if (results[i].markers.empty()) continue;
    detected = true;
    const int row = kQuadrants[i].first;
    const int col = kQuadrants[i].second;
    // Translate quadrant-local cv coords → full-Y-plane → full-frame pixels.
    const float ox = static_cast<float>(col * sub_w);
    const float oy = static_cast<float>(row * sub_h);
    for (auto& m : results[i].markers) {
      CornerSet set;
      set.set_id = static_cast<uint8_t>(row * 2 + col);
      set.marker_id = m.id;
      set.corners.reserve(m.corners.size());
      for (const auto& p : m.corners) {
        set.corners.emplace_back((p.x + ox) * scale, (p.y + oy) * scale);
      }
      out_sets.push_back(std::move(set));
    }
  }

  auto end_tp = std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  LOG_INFO("FrameProcessor",
           "Aruco2x2 markers: " + std::to_string(out_sets.size()) +
               " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

void FrameProcessor::flushBufferedFrames() {
  if (config.mode != ProcessMode::BUFFER || !config.save_frames) return;

  std::lock_guard<std::mutex> lock(buffer_mutex);

  std::cout << "Writing " << buffered_frames.size()
            << " buffered frames to disk..." << std::endl;

  size_t flushed_bytes = 0;
  for (const auto& frame : buffered_frames) {
    flushed_bytes += frame.data.size();
    std::string filename = generateFilename(frame.camera_id, frame.frame_id);
    std::ofstream file(filename, std::ios::binary);
    if (file) {
      file.write(reinterpret_cast<const char*>(frame.data.data()),
                 frame.data.size());
      frames_saved++;
      bytes_written += frame.data.size();

      frames_saved_per_camera[frame.camera_id]++;
    }
  }

  buffered_frames.clear();
  // Hand the budget back so a subsequent session starts with a clear backlog
  // even if start() is never reached (e.g. stop_cameras without a restart).
  releaseBacklog(flushed_bytes);

  std::cout << "Finished writing buffered frames" << std::endl;
}

void FrameProcessor::writerThreadFunc() {
  while (true) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cv.wait(lock,
                  [this] { return !write_queue.empty() || stop_threads; });

    if (stop_threads && write_queue.empty()) {
      break;
    }

    if (!write_queue.empty()) {
      WriteTask task = std::move(write_queue.front());
      write_queue.pop();
      lock.unlock();

      // The frame's bytes stop counting against the backlog budget once this
      // task has been dealt with, written or not — holding them after the
      // buffer is gone would leak the budget away a frame at a time.
      const size_t task_bytes = task.data.size();

      // Write with O_DIRECT for better SSD performance
      int fd = ::open(task.filename.c_str(),
                      O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
      if (fd >= 0) {
        // Align buffer for O_DIRECT
        size_t alignment = 4096;
        size_t aligned_size =
            (task.data.size() + alignment - 1) & ~(alignment - 1);
        void* aligned_buffer;
        if (posix_memalign(&aligned_buffer, alignment, aligned_size) == 0) {
          memcpy(aligned_buffer, task.data.data(), task.data.size());
          ssize_t written = write(fd, aligned_buffer, aligned_size);
          if (written > 0) {
            bytes_written += task.data.size();
            frames_saved++;

            frames_saved_per_camera[task.camera_id]++;
          }
          free(aligned_buffer);
        }
        close(fd);
      }
      releaseBacklog(task_bytes);
    }
  }
}

std::string FrameProcessor::generateFilename(uint32_t camera_id,
                                         uint32_t frame_id) {
  return frame_processor_helpers::makeFilename(actual_output_dir, camera_id,
                                           frame_id);
}
