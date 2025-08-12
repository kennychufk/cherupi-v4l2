#include "frame_saver.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

FrameSaver::~FrameSaver() { stop(); }

void FrameSaver::configure(const SaveConfig& cfg) {
  config = cfg;

  // Initialize checkerboard detector if needed
  if (config.mode == SaveMode::CHECKERBOARD) {
    checkerboard_detector = std::make_unique<CheckerboardDetector>(
        config.checkerboard_cols, config.checkerboard_rows);

    // Configure detector for RPi5 optimization
    checkerboard_detector->setAdaptiveThreshWinSize(11);
    checkerboard_detector->setAdaptiveThreshC(2);
    checkerboard_detector->setNormalizeImage(true);
    checkerboard_detector->setFastCheck(true);
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

  if (config.mode == SaveMode::BUFFER) {
    // Reserve space for buffered frames
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.clear();
    buffered_frames.reserve(10000);  // Reserve space for many frames
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
  std::stringstream ss;
  ss << config.prefix << "_cam" << camera_id << "_frame" << std::setfill('0')
     << std::setw(6) << frame_id << ".raw";
  return ss.str();
}
