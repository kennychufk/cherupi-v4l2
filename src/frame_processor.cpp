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

  // Reset per-camera counters
  frames_saved_per_camera.clear();

  // Drop a trigger left armed by a previous session (its client is long gone).
  {
    std::lock_guard<std::mutex> lock(trigger_mutex);
    pending_trigger.reset();
  }

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

  // A trigger still waiting on cameras that will now never deliver: resolve it
  // as cancelled (with whatever was already captured) so the client isn't left
  // waiting on an ack that can no longer arrive.
  {
    std::unique_ptr<PendingTrigger> abandoned;
    {
      std::lock_guard<std::mutex> lock(trigger_mutex);
      abandoned = std::move(pending_trigger);
    }
    if (abandoned && abandoned->on_complete) {
      abandoned->result.cancelled = true;
      LOG_WARN("FrameProcessor",
               "Trigger " + std::to_string(abandoned->id) +
                   " cancelled: capture stopped while it was pending");
      abandoned->on_complete(std::move(abandoned->result));
    }
  }

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
  if (!enabled) return;

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
  } else if (config.mode == ProcessMode::TRIGGER) {
    // Nothing is written unless a trigger is armed; frames still stream live so
    // the operator (or the robot's supervisor) can see what is being framed.
    handleTriggerFrame(frame);
  } else if (isDetectorMode(config.mode)) {
    // Best-effort: hand the frame to the async detection worker instead of
    // detecting inline. Detection takes tens–hundreds of ms — far longer than
    // the inter-frame interval — so running it on the capture thread would
    // stall the whole pipeline. Latest-wins per camera: if the worker is still
    // busy, the previously-pending frame for this camera is overwritten
    // (dropped) so the detector only ever works on the freshest frame.
    // Copy the multi-MB frame outside the lock so the critical section is just
    // a vector move, keeping the worker's pop path responsive.
    FrameData pending = frame;
    {
      std::lock_guard<std::mutex> lock(pending_mutex);
      pending_detection[frame.camera_id] = std::move(pending);
    }
    pending_cv.notify_one();
  }
}

std::optional<uint32_t> FrameProcessor::armTrigger(
    const std::vector<uint32_t>& camera_ids, int skip_frames,
    std::function<void(TriggerResult)> on_complete) {
  if (!enabled || config.mode != ProcessMode::TRIGGER) return std::nullopt;
  if (camera_ids.empty()) return std::nullopt;

  std::lock_guard<std::mutex> lock(trigger_mutex);
  if (pending_trigger) return std::nullopt;  // one outstanding trigger at a time

  auto trigger = std::make_unique<PendingTrigger>();
  trigger->id = next_trigger_id++;
  const int skip = std::max(0, skip_frames);
  for (uint32_t camera_id : camera_ids) trigger->waiting[camera_id] = skip;
  trigger->result.trigger_id = trigger->id;
  trigger->result.captures.reserve(camera_ids.size());
  trigger->on_complete = std::move(on_complete);
  pending_trigger = std::move(trigger);

  LOG_INFO("FrameProcessor",
           "Trigger " + std::to_string(pending_trigger->id) + " armed for " +
               std::to_string(camera_ids.size()) + " camera(s), skipping " +
               std::to_string(skip) + " frame(s) each");
  return pending_trigger->id;
}

bool FrameProcessor::hasPendingTrigger() {
  std::lock_guard<std::mutex> lock(trigger_mutex);
  return pending_trigger != nullptr;
}

void FrameProcessor::handleTriggerFrame(const FrameData& frame) {
  WriteTask task;
  bool queue_write = false;
  std::function<void(TriggerResult)> on_complete;
  TriggerResult finished;

  {
    std::lock_guard<std::mutex> lock(trigger_mutex);
    if (!pending_trigger) return;  // not armed: this frame is preview only
    auto it = pending_trigger->waiting.find(frame.camera_id);
    // Camera not part of this trigger, or already served by an earlier frame.
    if (it == pending_trigger->waiting.end()) return;
    // Settling frames requested by the client: discard and keep waiting.
    if (it->second > 0) {
      it->second--;
      return;
    }

    TriggerResult::Capture capture;
    capture.camera_id = frame.camera_id;
    capture.frame_id = frame.frame_id;
    if (config.save_frames) {
      capture.filename = generateFilename(frame.camera_id, frame.frame_id);
      task.filename = capture.filename;
      task.camera_id = frame.camera_id;
      task.frame_id = frame.frame_id;
      queue_write = true;
    }
    pending_trigger->result.captures.push_back(std::move(capture));
    pending_trigger->waiting.erase(it);

    if (pending_trigger->waiting.empty()) {
      finished = std::move(pending_trigger->result);
      on_complete = std::move(pending_trigger->on_complete);
      pending_trigger.reset();
    }
  }

  if (queue_write && reserveBacklog(frame.data.size())) {
    // Copy the multi-MB payload outside trigger_mutex so a second camera's
    // callback isn't held up behind this one's memcpy.
    task.data = frame.data;
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      write_queue.push(std::move(task));
    }
    queue_cv.notify_one();
  }

  // Fires only after the frame is queued for writing, so the client's ack
  // means "captured and handed to the writer pool" — the robot can move on.
  if (on_complete) on_complete(std::move(finished));
}

void FrameProcessor::detectionWorkerFunc() {
  while (true) {
    FrameData frame;
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

void FrameProcessor::processDetection(FrameData frame) {
  frames_checked++;

  std::vector<CornerSet> sets;
  bool detected = false;
  switch (config.mode) {
    case ProcessMode::CHECKERBOARD:
      detected = detectCheckerboard(frame, sets);
      break;
    case ProcessMode::CHECKERBOARD2X2:
      detected = detectCheckerboard2x2(frame, sets);
      break;
    case ProcessMode::ARUCO:
      detected = detectAruco(frame, sets);
      break;
    case ProcessMode::ARUCO2X2:
      detected = detectAruco2x2(frame, sets);
      break;
    default:
      break;
  }

  if (detected) {
    frames_detected++;

    // Persist the detected frame only when saving is enabled. When it is off we
    // skip building the WriteTask entirely — that avoids copying the multi-MB
    // frame payload into the write queue on every detection, which is the main
    // win of the no-save detection pipeline (the frame is still moved once into
    // the streaming slot below).
    if (config.save_frames && reserveBacklog(frame.data.size())) {
      WriteTask task;
      task.filename = generateFilename(frame.camera_id, frame.frame_id);
      task.data = frame.data;  // frame is moved into the stream slot below
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
    df.frame = std::move(frame);
    df.sets = std::move(sets);
    df.fresh = true;
  }
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

  auto end_tp = std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  LOG_INFO("FrameProcessor", "Checkerboard detected: " + std::to_string(detected) +
                             " Total: " + std::to_string(total_time) + "ms");
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

  auto end_tp = std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  LOG_INFO("FrameProcessor",
           "Checkerboard2x2 detected: " + std::to_string(detected) +
               " Total: " + std::to_string(total_time) + "ms");
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
