#include "frame_saver.hpp"

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

#include "frame_saver_helpers.hpp"

FrameSaver::~FrameSaver() { stop(); }

void FrameSaver::configure(const SaveConfig& cfg) {
  config = cfg;

  // Create output directory when configuring (if mode is not NONE)
  if (config.mode != SaveMode::NONE) {
    if (!createOutputDirectory()) {
      LOG_WARN("FrameSaver",
               "Failed to create output directory, falling back to current "
               "directory");
      actual_output_dir = ".";
    }
  }

  // Initialize checkerboard detector if needed
  if (config.mode == SaveMode::CHECKERBOARD ||
      config.mode == SaveMode::CHECKERBOARD2X2) {
    checkerboard_detector = std::make_unique<CheckerboardDetector>(
        config.checkerboard_cols, config.checkerboard_rows);
  }

  // Initialize aruco detector if needed
  if (config.mode == SaveMode::ARUCO || config.mode == SaveMode::ARUCO2X2) {
    aruco_detector =
        std::make_unique<ArucoDetector>(config.aruco_corner_refine);
  }
}

bool FrameSaver::createOutputDirectory() {
  try {
    std::string trimmed_dir =
        frame_saver_helpers::normalizeBaseDir(config.output_dir);
    if (trimmed_dir == "." && config.output_dir.find_first_not_of(" \t\n\r") ==
                                  std::string::npos) {
      LOG_WARN("FrameSaver",
               "Empty output directory specified, using current directory");
      actual_output_dir = ".";
      return true;
    }

    std::string final_dir = trimmed_dir;
    if (config.prepend_timestamp_to_dir) {
      final_dir = frame_saver_helpers::makeTimestampedDir(
          trimmed_dir, std::chrono::system_clock::now());
      LOG_INFO("FrameSaver", "Timestamp prepended to directory: " +
                                 trimmed_dir + " -> " + final_dir);
    }

    // Use the filesystem library for robust directory creation
    std::filesystem::path dir_path(final_dir);

    // Check if directory already exists
    if (std::filesystem::exists(dir_path)) {
      if (std::filesystem::is_directory(dir_path)) {
        LOG_INFO("FrameSaver", "Output directory already exists: " + final_dir);
        actual_output_dir = final_dir;
        return true;
      } else {
        LOG_ERROR("FrameSaver",
                  "Output path exists but is not a directory: " + final_dir);
        return false;
      }
    }

    // Create the directory
    if (std::filesystem::create_directory(dir_path)) {
      LOG_INFO("FrameSaver", "Created output directory: " + final_dir);

      // Set permissions to 0755 using chmod
      if (chmod(final_dir.c_str(),
                S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0) {
        LOG_WARN("FrameSaver", "Failed to set directory permissions to 0755");
      }

      actual_output_dir = final_dir;
      return true;
    } else {
      LOG_ERROR("FrameSaver",
                "Failed to create output directory: " + final_dir);
      return false;
    }
  } catch (const std::filesystem::filesystem_error& e) {
    LOG_ERROR("FrameSaver",
              "Filesystem error creating directory: " + std::string(e.what()));
    return false;
  } catch (const std::exception& e) {
    LOG_ERROR("FrameSaver",
              "Error creating output directory: " + std::string(e.what()));
    return false;
  }
}

void FrameSaver::start() {
  if (config.mode == SaveMode::NONE) {
    enabled = false;
    return;
  }

  enabled = true;
  stop_threads = false;
  frames_checked = 0;
  checkerboards_detected = 0;

  // Reset per-camera counters
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

  if (config.mode == SaveMode::BUFFER) {
    // Reserve space for buffered frames
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.clear();
    buffered_frames.reserve(10000);
  } else if (config.mode == SaveMode::BATCH || isDetectorMode(config.mode)) {
    // Drop any partial batch left over from a previous session.
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      pending_batch.clear();
    }
    // Start writer threads
    for (size_t i = 0; i < config.writer_threads; i++) {
      writer_threads.emplace_back(&FrameSaver::writerThreadFunc, this);
    }
    // Detector modes (checkerboard/aruco) run one async worker that does the
    // CPU-heavy detection off the capture thread.
    if (isDetectorMode(config.mode)) {
      detection_thread = std::thread(&FrameSaver::detectionWorkerFunc, this);
    }
  }
}

void FrameSaver::stop() {
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
  }

  if (config.mode == SaveMode::BATCH || isDetectorMode(config.mode)) {
    // Flush any frames still held in a partial batch (BATCH mode) so they are
    // written before the workers exit, then signal threads to stop.
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
    std::cout << "Detection stats: " << checkerboards_detected
              << " detected out of " << frames_checked << " checked ("
              << (100.0 * checkerboards_detected / frames_checked) << "%)"
              << std::endl;
  }
}

void FrameSaver::saveFrame(const FrameData& frame) {
  if (!enabled) return;

  if (config.mode == SaveMode::BUFFER) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.push_back(frame);
  } else if (config.mode == SaveMode::BATCH) {
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

void FrameSaver::detectionWorkerFunc() {
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

void FrameSaver::processDetection(FrameData frame) {
  frames_checked++;

  std::vector<CornerSet> sets;
  bool detected = false;
  switch (config.mode) {
    case SaveMode::CHECKERBOARD:
      detected = detectCheckerboard(frame, sets);
      break;
    case SaveMode::CHECKERBOARD2X2:
      detected = detectCheckerboard2x2(frame, sets);
      break;
    case SaveMode::ARUCO:
      detected = detectAruco(frame, sets);
      break;
    case SaveMode::ARUCO2X2:
      detected = detectAruco2x2(frame, sets);
      break;
    default:
      break;
  }

  if (detected) {
    checkerboards_detected++;

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

bool FrameSaver::detectCheckerboard(const FrameData& frame,
                                    std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_saver_helpers::extractYFromYUV420(
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
  LOG_INFO("FrameSaver", "Checkerboard detected: " + std::to_string(detected) +
                             " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

bool FrameSaver::detectCheckerboard2x2(const FrameData& frame,
                                       std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_saver_helpers::extractYFromYUV420(
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
  LOG_INFO("FrameSaver",
           "Checkerboard2x2 detected: " + std::to_string(detected) +
               " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

bool FrameSaver::detectAruco(const FrameData& frame,
                             std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_saver_helpers::extractYFromYUV420(
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
  LOG_INFO("FrameSaver", "Aruco markers: " + std::to_string(markers.size()) +
                             " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

bool FrameSaver::detectAruco2x2(const FrameData& frame,
                                std::vector<CornerSet>& out_sets) {
  auto start_tp = std::chrono::steady_clock::now();
  out_sets.clear();

  std::vector<uint8_t> grayscale_data = frame_saver_helpers::extractYFromYUV420(
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
  LOG_INFO("FrameSaver",
           "Aruco2x2 markers: " + std::to_string(out_sets.size()) +
               " Total: " + std::to_string(total_time) + "ms");
  return detected;
}

void FrameSaver::flushBufferedFrames() {
  if (config.mode != SaveMode::BUFFER) return;

  std::lock_guard<std::mutex> lock(buffer_mutex);

  std::cout << "Writing " << buffered_frames.size()
            << " buffered frames to disk..." << std::endl;

  for (const auto& frame : buffered_frames) {
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

  std::cout << "Finished writing buffered frames" << std::endl;
}

void FrameSaver::writerThreadFunc() {
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
    }
  }
}

std::string FrameSaver::generateFilename(uint32_t camera_id,
                                         uint32_t frame_id) {
  return frame_saver_helpers::makeFilename(actual_output_dir, camera_id,
                                           frame_id);
}
