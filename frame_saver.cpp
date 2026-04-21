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
    // Check if output_dir is empty or just whitespace
    std::string trimmed_dir = config.output_dir;
    trimmed_dir.erase(0, trimmed_dir.find_first_not_of(" \t\n\r"));
    trimmed_dir.erase(trimmed_dir.find_last_not_of(" \t\n\r") + 1);

    if (trimmed_dir.empty()) {
      LOG_WARN("FrameSaver",
               "Empty output directory specified, using current directory");
      actual_output_dir = ".";
      return true;
    }

    // Apply timestamp prefix if requested
    std::string final_dir = trimmed_dir;
    if (config.prepend_timestamp_to_dir) {
      std::filesystem::path dir_path(trimmed_dir);
      std::filesystem::path parent_path = dir_path.parent_path();
      std::string dir_name = dir_path.filename().string();

      // Generate timestamp string (YYYYMMDD-HHMM-)
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::tm* tm_ptr = std::localtime(&time_t);

      std::stringstream timestamp_ss;
      timestamp_ss << std::put_time(tm_ptr, "%Y%m%d-%H%M-");
      std::string timestamp = timestamp_ss.str();

      // Construct the new directory name with timestamp
      std::string timestamped_name = timestamp + dir_name;

      if (parent_path.empty() || parent_path == ".") {
        final_dir = timestamped_name;
      } else {
        final_dir = (parent_path / timestamped_name).string();
      }

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
    buffered_frames.reserve(10000);  // Reserve space for many frames
    buffered_awb_records.clear();
    buffered_awb_records.reserve(10000);
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

  closeAwbSidecars();

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
    buffered_awb_records.push_back({frame.frame_id, frame.camera_id,
                                    frame.awb_gain_r, frame.awb_gain_b,
                                    frame.awb_cct});
  } else if (config.mode == SaveMode::BATCH) {
    WriteTask task;
    task.filename = generateFilename(frame.camera_id, frame.frame_id);
    task.data = frame.data;
    task.camera_id = frame.camera_id;
    task.frame_id = frame.frame_id;
    task.awb_gain_r = frame.awb_gain_r;
    task.awb_gain_b = frame.awb_gain_b;
    task.awb_cct = frame.awb_cct;

    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      write_queue.push(std::move(task));
    }
    queue_cv.notify_one();
  } else if (config.mode == SaveMode::CHECKERBOARD) {
    frames_checked++;

    // Check if frame contains checkerboard
    if (detectCheckerboard(frame)) {
      checkerboards_detected++;

      // Save the original raw frame
      WriteTask task;
      task.filename = generateFilename(frame.camera_id, frame.frame_id);
      task.data = frame.data;
      task.camera_id = frame.camera_id;
      task.frame_id = frame.frame_id;
      task.awb_gain_r = frame.awb_gain_r;
      task.awb_gain_b = frame.awb_gain_b;
      task.awb_cct = frame.awb_cct;

      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        write_queue.push(std::move(task));
      }
      queue_cv.notify_one();
    }
  }
}

bool FrameSaver::detectCheckerboard(const FrameData& frame) {
  std::chrono::steady_clock::time_point start_tp =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::now();

  // Configure BayerConverter
  Config bayer_config;
  bayer_config.width = frame.width;
  bayer_config.height = frame.height;
  bayer_config.stride = frame.bytes_per_line;
  bayer_config.full_res = config.checkerboard_full_res_detection;
  bayer_config.num_threads = config.checkerboard_num_threads;
  bayer_config.save_ppm = false;  // We don't need to save PPM

  // Create BayerConverter instance
  BayerConverter converter(bayer_config);

  // Set the input data directly (avoiding file I/O)
  // We need to modify BayerConverter to accept raw data, but for now,
  // we'll create a temporary converter that processes the data

  // Since BayerConverter expects packed 10-bit data and our frame.data
  // is already in the correct format, we can process it directly

  // Create a temporary processing pipeline
  std::vector<uint16_t> bayer_data(frame.width * frame.height);

  // Unpack 10-bit data
  unpack_10bit_neon(frame.data.data(), bayer_data.data(), frame.width,
                    frame.height, frame.bytes_per_line);
  std::chrono::steady_clock::time_point unpack_tp =
      std::chrono::steady_clock::now();

  // Prepare output buffer
  int out_width =
      config.checkerboard_full_res_detection ? frame.width : frame.width / 2;
  int out_height =
      config.checkerboard_full_res_detection ? frame.height : frame.height / 2;
  std::vector<uint8_t> grayscale_data(out_width * out_height);

  // Demosaic
  demosaic_threaded(bayer_data.data(), grayscale_data.data(), frame.width,
                    frame.height, config.checkerboard_full_res_detection,
                    config.checkerboard_num_threads);
  std::chrono::steady_clock::time_point demosaic_tp =
      std::chrono::steady_clock::now();

  // Detect checkerboard
  bool detected = checkerboard_detector->detect(grayscale_data.data(),
                                                out_width, out_height);

  std::chrono::steady_clock::time_point end_tp =
      std::chrono::steady_clock::now();
  auto total_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_tp - start_tp)
          .count();
  auto unpack_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                         unpack_tp - start_tp)
                         .count();
  auto demosaic_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                           demosaic_tp - unpack_tp)
                           .count();
  auto detect_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_tp - demosaic_tp)
                         .count();
  LOG_INFO("FrameSaver", "Checkerboard detected: " + std::to_string(detected) +
                             " Total: " + std::to_string(total_time) +
                             "ms (Unpack " + std::to_string(unpack_time) +
                             " Demosaic " + std::to_string(demosaic_time) +
                             " Detect " + std::to_string(detect_time) + ")");
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

  // Flush per-camera AWB sidecars from the buffered records.
  std::unordered_map<uint32_t, std::ofstream> awb_files;
  for (const auto& rec : buffered_awb_records) {
    auto it = awb_files.find(rec.camera_id);
    if (it == awb_files.end()) {
      std::stringstream ss;
      ss << actual_output_dir << "/cam" << rec.camera_id << "_awb.bin";
      auto [ins, _] = awb_files.emplace(
          rec.camera_id,
          std::ofstream(ss.str(), std::ios::binary | std::ios::app));
      it = ins;
    }
    if (it->second) {
      it->second.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }
  }
  buffered_awb_records.clear();

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
            writeAwbRecord(task);
          }
          free(aligned_buffer);
        }
        close(fd);
      }
    }
  }
}

void FrameSaver::writeAwbRecord(const WriteTask& task) {
  AwbRecord rec{task.frame_id, task.camera_id, task.awb_gain_r,
                task.awb_gain_b, task.awb_cct};

  std::lock_guard<std::mutex> lock(awb_sidecar_mutex);
  auto it = awb_sidecar_fds.find(task.camera_id);
  int fd = -1;
  if (it == awb_sidecar_fds.end()) {
    std::stringstream ss;
    ss << actual_output_dir << "/cam" << task.camera_id << "_awb.bin";
    fd = ::open(ss.str().c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd < 0) {
      LOG_WARN("FrameSaver", "Failed to open AWB sidecar: " + ss.str());
      return;
    }
    awb_sidecar_fds[task.camera_id] = fd;
  } else {
    fd = it->second;
  }

  ssize_t n = write(fd, &rec, sizeof(rec));
  if (n != static_cast<ssize_t>(sizeof(rec))) {
    LOG_WARN("FrameSaver", "Short write on AWB sidecar");
  }
}

void FrameSaver::closeAwbSidecars() {
  std::lock_guard<std::mutex> lock(awb_sidecar_mutex);
  for (auto& kv : awb_sidecar_fds) {
    if (kv.second >= 0) close(kv.second);
  }
  awb_sidecar_fds.clear();
}

std::string FrameSaver::generateFilename(uint32_t camera_id,
                                         uint32_t frame_id) {
  std::stringstream ss;
  ss << actual_output_dir << "/cam" << camera_id << "-" << frame_id << ".raw";
  return ss.str();
}
