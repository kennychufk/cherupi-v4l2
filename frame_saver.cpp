#include "frame_saver.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

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
  if (config.mode == SaveMode::CHECKERBOARD) {
    checkerboard_detector = std::make_unique<CheckerboardDetector>(
        config.checkerboard_cols, config.checkerboard_rows);
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

  if (config.mode == SaveMode::BUFFER) {
    // Reserve space for buffered frames
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.clear();
    buffered_frames.reserve(10000);
  } else if (config.mode == SaveMode::BATCH ||
             config.mode == SaveMode::CHECKERBOARD) {
    // Start writer threads
    for (size_t i = 0; i < config.writer_threads; i++) {
      writer_threads.emplace_back(&FrameSaver::writerThreadFunc, this);
    }
  }
}

void FrameSaver::stop() {
  enabled = false;

  if (config.mode == SaveMode::BATCH || config.mode == SaveMode::CHECKERBOARD) {
    // Signal threads to stop
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
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

  // Log checkerboard detection statistics
  if (config.mode == SaveMode::CHECKERBOARD && frames_checked > 0) {
    std::cout << "Checkerboard detection stats: " << checkerboards_detected
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

    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      write_queue.push(std::move(task));
    }
    queue_cv.notify_one();
  } else if (config.mode == SaveMode::CHECKERBOARD) {
    frames_checked++;

    if (detectCheckerboard(frame)) {
      checkerboards_detected++;

      WriteTask task;
      task.filename = generateFilename(frame.camera_id, frame.frame_id);
      task.data = frame.data;
      task.camera_id = frame.camera_id;
      task.frame_id = frame.frame_id;

      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        write_queue.push(std::move(task));
      }
      queue_cv.notify_one();
    }
  }
}

bool FrameSaver::detectCheckerboard(const FrameData& frame) {
  auto start_tp = std::chrono::steady_clock::now();

  std::vector<uint8_t> grayscale_data = frame_saver_helpers::extractYFromYUV420(
      frame, config.checkerboard_full_res_detection);
  int out_width =
      config.checkerboard_full_res_detection ? frame.width : frame.width / 2;
  int out_height =
      config.checkerboard_full_res_detection ? frame.height : frame.height / 2;

  bool detected = checkerboard_detector->detect(grayscale_data.data(),
                                                out_width, out_height);

  auto end_tp = std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  LOG_INFO("FrameSaver", "Checkerboard detected: " + std::to_string(detected) +
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
