#include "frame_saver.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

FrameSaver::~FrameSaver() { stop(); }

void FrameSaver::configure(const SaveConfig& cfg) { config = cfg; }

void FrameSaver::start() {
  if (config.mode == SaveMode::NONE) {
    enabled = false;
    return;
  }

  enabled = true;
  stop_threads = false;

  if (config.mode == SaveMode::BUFFER) {
    // Reserve space for buffered frames
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffered_frames.clear();
    buffered_frames.reserve(10000);  // Reserve space for many frames
  } else if (config.mode == SaveMode::BATCH) {
    // Start writer threads
    for (size_t i = 0; i < config.writer_threads; i++) {
      writer_threads.emplace_back(&FrameSaver::writerThreadFunc, this);
    }
  }
}

void FrameSaver::stop() {
  enabled = false;

  if (config.mode == SaveMode::BATCH) {
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
  }
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
