#include "stream_manager.hpp"

#include <random>
#include <sstream>

StreamManager::StreamManager(CameraManager* mgr, FrameSaver* saver)
    : camera_manager(mgr), frame_saver(saver) {
  LOG_INFO("StreamManager",
           "Initialized with adaptive rate control, chunk size: " +
               std::to_string(CHUNK_SIZE) + " bytes");
}

StreamManager::~StreamManager() {
  LOG_INFO("StreamManager", "Shutting down");
  stopAllStreaming();

  // Wait for any in-progress sends to complete
  auto start = std::chrono::steady_clock::now();
  while (send_in_progress.load() &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  logStats();
}

void StreamManager::setSink(FrameSink* new_sink) {
  std::lock_guard<std::mutex> lock(sink_mutex);
  sink = new_sink;
  has_backpressure = false;
  rate_controller.reset();

  LOG_INFO("StreamManager", "Sink attached");

  if (!streaming_active && !streaming_cameras.empty()) {
    should_stop = false;
    streaming_active = true;
    streaming_thread =
        std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
    LOG_INFO("StreamManager", "Streaming thread started");
  }
}

void StreamManager::clearSink() {
  LOG_INFO("StreamManager", "Clearing sink");

  stopAllStreaming();

  {
    std::lock_guard<std::mutex> lock(sink_mutex);
    sink = nullptr;
  }

  cleanupTransfer();
  rate_controller.reset();
}

bool StreamManager::startStreamingCamera(uint32_t camera_id) {
  if (camera_id >= camera_manager->getCameraCount()) {
    LOG_ERROR("StreamManager",
              "Invalid camera ID: " + std::to_string(camera_id));
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.insert(camera_id);
  }

  LOG_INFO("StreamManager",
           "Started streaming camera " + std::to_string(camera_id));

  if (!streaming_active) {
    std::lock_guard<std::mutex> lock(sink_mutex);
    if (sink) {
      should_stop = false;
      streaming_active = true;
      streaming_thread =
          std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
      LOG_INFO("StreamManager", "Streaming thread started");
    }
  }

  return true;
}

bool StreamManager::stopStreamingCamera(uint32_t camera_id) {
  bool removed = false;
  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    removed = streaming_cameras.erase(camera_id) > 0;
  }

  if (removed) {
    LOG_INFO("StreamManager",
             "Stopped streaming camera " + std::to_string(camera_id));

    {
      std::lock_guard<std::mutex> lock(transfer_mutex);
      if (current_transfer && current_transfer->frame.camera_id == camera_id) {
        current_transfer->in_progress = false;
      }
    }
  }

  return removed;
}

void StreamManager::stopAllStreaming() {
  LOG_INFO("StreamManager", "Stopping all streaming");

  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.clear();
  }

  if (streaming_active) {
    should_stop = true;
    frame_available_cv.notify_all();

    if (streaming_thread && streaming_thread->joinable()) {
      LOG_DEBUG("StreamManager", "Waiting for streaming thread to finish");
      streaming_thread->join();
    }

    streaming_active = false;
  }

  cleanupTransfer();

  LOG_INFO("StreamManager", "All streaming stopped");
}

bool StreamManager::isStreamingCamera(uint32_t camera_id) {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  return streaming_cameras.find(camera_id) != streaming_cameras.end();
}

size_t StreamManager::getStreamingCameraCount() {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  return streaming_cameras.size();
}

void StreamManager::notifyBackpressure(bool has_pressure) {
  bool prev = has_backpressure.exchange(has_pressure);
  if (has_pressure && !prev) {
    stats.backpressure_pauses++;
    rate_controller.recordBackpressure();
    LOG_WARN("StreamManager",
             "Backpressure detected - adjusting rate to " +
                 std::to_string(rate_controller.getCurrentRate()) +
                 " chunks/sec");
  } else if (!has_pressure && prev) {
    LOG_INFO("StreamManager",
             "Backpressure cleared - current rate: " +
                 std::to_string(rate_controller.getCurrentRate()) +
                 " chunks/sec");
    frame_available_cv.notify_one();
  }
}

void StreamManager::streamingLoop() {
  LOG_INFO("StreamManager", "Streaming loop started");

  std::vector<uint32_t> camera_order;
  size_t current_index = 0;

  while (!should_stop) {
    if (has_backpressure) {
      LOG_DEBUG("StreamManager", "Waiting for backpressure to clear");
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return !has_backpressure || should_stop.load();
      });
      continue;
    }

    // Snapshot sink/camera state under the locks, then release them BEFORE
    // sleeping. Holding sink_mutex or streaming_mutex across cv.wait_for
    // starves the loop thread on the WS message handler — stopStreamingCamera
    // / clearSink lock the same mutexes, and on a hot streaming path the
    // streaming thread would re-acquire the lock between wait_for ticks
    // faster than the kernel could schedule the loop-thread waiter.
    bool no_sink = false;
    {
      std::lock_guard<std::mutex> lock(sink_mutex);
      no_sink = (sink == nullptr);
    }
    if (no_sink) {
      LOG_DEBUG("StreamManager", "No sink attached, waiting");
      std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
      frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
      continue;
    }

    bool no_cameras = false;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      no_cameras = streaming_cameras.empty();
      if (!no_cameras && camera_order.size() != streaming_cameras.size()) {
        camera_order.clear();
        for (uint32_t cam_id : streaming_cameras) {
          camera_order.push_back(cam_id);
        }
        current_index = 0;
        LOG_DEBUG("StreamManager", "Updated camera order, " +
                                       std::to_string(camera_order.size()) +
                                       " cameras");
      }
    }
    if (no_cameras) {
      LOG_DEBUG("StreamManager", "No cameras streaming, waiting");
      std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
      frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
      continue;
    }

    if (isTransferTimedOut()) {
      LOG_WARN("StreamManager", "Transfer timed out, cleaning up");
      cleanupTransfer();
    }

    // Don't start a new frame while the WS outgoing buffer is already over
    // the soft cap — otherwise we widen the gap any TEXT control reply
    // (status / error) has to drain past on its way to the client.
    if (!waitForBufferDrain()) continue;

    {
      std::lock_guard<std::mutex> lock(transfer_mutex);
      if (current_transfer && current_transfer->in_progress) {
        LOG_DEBUG("StreamManager",
                  "Transfer in progress, skipping frame check");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
    }

    bool found_frame = false;
    size_t cameras_checked = 0;

    while (cameras_checked < camera_order.size() && !found_frame &&
           !should_stop) {
      uint32_t camera_id = camera_order[current_index];
      current_index = (current_index + 1) % camera_order.size();
      cameras_checked++;

      if (!isStreamingCamera(camera_id)) {
        continue;
      }

      Camera* camera = camera_manager->getCamera(camera_id);
      if (!camera) {
        LOG_WARN("StreamManager",
                 "Camera " + std::to_string(camera_id) + " not found");
        continue;
      }

      FrameData frame;
      if (camera->getFrameForStreaming(frame)) {
        LOG_DEBUG("StreamManager",
                  "Got frame " + std::to_string(frame.frame_id) +
                      " from camera " + std::to_string(camera_id));

        bool success = false;
        if (header_only_mode) {
          success = sendHeaderOnlyFrame(frame, camera_id);
          if (success) {
            stats.header_only_frames++;
          }
        } else {
          success = sendChunkedFrame(frame, camera_id);
        }

        if (success) {
          found_frame = true;
          stats.frames_sent++;
        } else {
          stats.frames_dropped++;
          LOG_WARN("StreamManager", "Failed to send frame from camera " +
                                        std::to_string(camera_id));
        }

        camera->releaseStreamingFrame();
      }
    }

    if (!found_frame && !should_stop) {
      LOG_DEBUG("StreamManager", "No frames available, waiting");
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(10),
                                  [this] { return should_stop.load(); });
    }
  }

  LOG_INFO("StreamManager", "Streaming loop ended");
}

bool StreamManager::waitForBufferDrain() {
  // Snapshot the sink under sink_mutex, but release the mutex before
  // sleeping — otherwise clearSink() (which also takes sink_mutex on the
  // loop thread during cleanupConnection) would block until we return.
  FrameSink* s = nullptr;
  {
    std::lock_guard<std::mutex> lock(sink_mutex);
    s = sink;
  }
  if (!s) return true;
  if (s->bufferedAmount() <= SOFT_BUFFER_LIMIT) return true;

  // Cap the wait: at LAN speeds 512 KB drains in ~5 ms, so 2 s of
  // ~5 ms ticks is ample. If we hit the cap something else is wrong
  // (network stalled, client gone) and the rest of the loop will pick
  // it up via the timeout / sink-cleared paths.
  for (int i = 0; i < 400; ++i) {
    if (should_stop.load()) return false;
    {
      std::lock_guard<std::mutex> lock(sink_mutex);
      if (!sink) return true;
      if (sink->bufferedAmount() <= SOFT_BUFFER_LIMIT) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;  // give up waiting; let the rate controller / drain handle it
}

bool StreamManager::sendFrameForTest(const FrameData& frame,
                                     uint32_t camera_id) {
  if (header_only_mode) {
    return sendHeaderOnlyFrame(frame, camera_id);
  }
  return sendChunkedFrame(frame, camera_id);
}

bool StreamManager::sendHeaderOnlyFrame(const FrameData& frame,
                                        uint32_t camera_id) {
  ChunkedTransfer transfer;
  transfer.frame = frame;
  transfer.frame_uuid = generateFrameUUID();
  transfer.total_chunks = 0;
  transfer.current_chunk = 0;
  transfer.start_time = std::chrono::steady_clock::now();
  transfer.in_progress = false;

  LOG_DEBUG("StreamManager", "Sending header only for frame " +
                                 std::to_string(frame.frame_id) +
                                 " from camera " + std::to_string(camera_id));

  return sendChunkHeader(transfer, true);
}

bool StreamManager::sendChunkedFrame(const FrameData& frame,
                                     uint32_t camera_id) {
  {
    std::lock_guard<std::mutex> lock(transfer_mutex);
    current_transfer = std::make_unique<ChunkedTransfer>();
    current_transfer->frame = frame;
    current_transfer->frame_uuid = generateFrameUUID();
    current_transfer->total_chunks =
        (frame.data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    current_transfer->current_chunk = 0;
    current_transfer->start_time = std::chrono::steady_clock::now();
    current_transfer->in_progress = true;
  }

  LOG_DEBUG("StreamManager",
            "Starting chunked transfer for frame " +
                std::to_string(frame.frame_id) + " from camera " +
                std::to_string(camera_id) + ", " +
                std::to_string(current_transfer->total_chunks) + " chunks");

  if (!sendChunkHeader(*current_transfer, false)) {
    LOG_ERROR("StreamManager", "Failed to send chunk header");
    cleanupTransfer();
    return false;
  }

  auto last_chunk_time = std::chrono::steady_clock::now();

  for (size_t chunk_idx = 0; chunk_idx < current_transfer->total_chunks;
       chunk_idx++) {
    if (should_stop) {
      cleanupTransfer();
      return false;
    }

    // Only enforce the per-camera streaming check when the streaming loop is
    // driving the send. `sendFrameForTest` skips this so chunking can be
    // exercised without entering the streaming state.
    if (streaming_active && !isStreamingCamera(camera_id)) {
      LOG_INFO("StreamManager", "Camera " + std::to_string(camera_id) +
                                    " stopped during transfer");
      cleanupTransfer();
      return false;
    }

    int backpressure_wait_count = 0;
    while (has_backpressure && backpressure_wait_count < 50) {
      LOG_DEBUG("StreamManager", "Backpressure during chunk " +
                                     std::to_string(chunk_idx) + ", waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      backpressure_wait_count++;

      if (should_stop) {
        cleanupTransfer();
        return false;
      }
    }

    if (has_backpressure) {
      LOG_ERROR("StreamManager", "Backpressure timeout at chunk " +
                                     std::to_string(chunk_idx) +
                                     ", aborting transfer");
      cleanupTransfer();
      return false;
    }

    // Mid-frame cap: a 6 MB IMX519 frame is ~200 chunks; without this an
    // in-flight frame can balloon the buffer well past SOFT_BUFFER_LIMIT
    // before the next frame's check runs.
    if (!waitForBufferDrain()) {
      cleanupTransfer();
      return false;
    }

    auto chunk_delay = rate_controller.getChunkDelay();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_chunk_time;

    if (elapsed < chunk_delay) {
      std::this_thread::sleep_for(chunk_delay - elapsed);
    }

    last_chunk_time = std::chrono::steady_clock::now();

    if (!sendChunkData(*current_transfer, chunk_idx)) {
      LOG_ERROR("StreamManager",
                "Failed to send chunk " + std::to_string(chunk_idx) + "/" +
                    std::to_string(current_transfer->total_chunks));
      cleanupTransfer();
      return false;
    }

    current_transfer->current_chunk = chunk_idx + 1;
    stats.chunks_sent++;

    if (!has_backpressure) {
      rate_controller.recordChunkSent();
    }

    if ((chunk_idx + 1) % 20 == 0 ||
        chunk_idx == current_transfer->total_chunks - 1) {
      LOG_DEBUG(
          "StreamManager",
          "Sent chunk " + std::to_string(chunk_idx + 1) + "/" +
              std::to_string(current_transfer->total_chunks) + " at rate: " +
              std::to_string(rate_controller.getCurrentRate()) + " chunks/sec");
    }
  }

  LOG_DEBUG(
      "StreamManager",
      "Successfully sent frame " + std::to_string(frame.frame_id) + " in " +
          std::to_string(current_transfer->total_chunks) + " chunks at rate: " +
          std::to_string(rate_controller.getCurrentRate()) + " chunks/sec");

  cleanupTransfer();
  return true;
}

bool StreamManager::sendChunkHeader(const ChunkedTransfer& transfer,
                                    bool header_only) {
  std::lock_guard<std::mutex> lock(sink_mutex);
  if (!sink) return false;

  // Pull a cached detection result (if any) for this exact frame. The saver
  // writes the cache synchronously in the capture-thread path before this
  // streaming-thread sees the frame (camera.cpp orders onFrameCaptured ahead
  // of latest_frame publish), so the cache is populated by the time we get
  // here in checkerboard / checkerboard2x2 modes.
  std::vector<CornerSet> corner_sets;
  if (frame_saver) {
    const SaveMode mode = frame_saver->getMode();
    if (mode == SaveMode::CHECKERBOARD || mode == SaveMode::CHECKERBOARD2X2) {
      auto cached = frame_saver->getDetectionForFrame(
          transfer.frame.camera_id, transfer.frame.frame_id);
      if (cached) corner_sets = std::move(cached->sets);
    }
  }

  // Compute the variable corner-block size up front so we can encode it in
  // ChunkHeader before the bytes go out.
  uint32_t corner_block_size = 0;
  for (const auto& set : corner_sets) {
    corner_block_size += static_cast<uint32_t>(sizeof(CornerSetHeader));
    corner_block_size += static_cast<uint32_t>(set.corners.size() *
                                               2 * sizeof(float));
  }

  std::vector<uint8_t> message;
  message.reserve(sizeof(ChunkStartMarker) + sizeof(ChunkHeader) +
                  corner_block_size);

  ChunkStartMarker start_marker;
  const uint8_t* start_bytes = reinterpret_cast<const uint8_t*>(&start_marker);
  message.insert(message.end(), start_bytes,
                 start_bytes + sizeof(ChunkStartMarker));

  ChunkHeader header{};
  header.frame_uuid = transfer.frame_uuid;
  header.frame_id = transfer.frame.frame_id;
  header.camera_id = transfer.frame.camera_id;

  if (header_only) {
    header.total_chunks = 0;
    header.total_size = 0;
  } else {
    header.total_chunks = transfer.total_chunks;
    header.total_size = transfer.frame.data.size();
  }

  header.bytes_per_line = transfer.frame.bytes_per_line;
  header.width = transfer.frame.width;
  header.height = transfer.frame.height;
  header.pixel_format = transfer.frame.pixel_format;
  header.frames_saved =
      frame_saver
          ? frame_saver->getFramesSavedForCamera(transfer.frame.camera_id)
          : 0;
  header.timestamp_us = transfer.frame.timestamp_us;
  header.frame_duration_us = transfer.frame.frame_duration_us;
  header.corner_block_size = corner_block_size;
  header.num_corner_sets = static_cast<uint16_t>(corner_sets.size());
  header.reserved = 0;

  const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&header);
  message.insert(message.end(), header_bytes,
                 header_bytes + sizeof(ChunkHeader));

  // Append the variable-size CornerBlock: { CornerSetHeader, corner xy floats }
  // per set, packed back-to-back.
  for (const auto& set : corner_sets) {
    CornerSetHeader sh{};
    sh.set_id = set.set_id;
    sh.flags = 0x01;  // bit 0: full-frame Y-plane pixel coords
    sh.num_corners = static_cast<uint16_t>(set.corners.size());
    const uint8_t* sh_bytes = reinterpret_cast<const uint8_t*>(&sh);
    message.insert(message.end(), sh_bytes, sh_bytes + sizeof(CornerSetHeader));
    for (const auto& p : set.corners) {
      float xy[2] = {p.x, p.y};
      const uint8_t* xy_bytes = reinterpret_cast<const uint8_t*>(xy);
      message.insert(message.end(), xy_bytes, xy_bytes + sizeof(xy));
    }
  }

  send_in_progress = true;
  bool ok = sink->send(message.data(), message.size());
  send_in_progress = false;
  if (ok) stats.bytes_sent += message.size();
  return ok;
}

bool StreamManager::sendChunkData(const ChunkedTransfer& transfer,
                                  size_t chunk_index) {
  std::lock_guard<std::mutex> lock(sink_mutex);
  if (!sink) return false;

  size_t offset = chunk_index * CHUNK_SIZE;
  size_t chunk_size = std::min(CHUNK_SIZE, transfer.frame.data.size() - offset);

  std::vector<uint8_t> message;
  message.reserve(sizeof(ChunkData) + chunk_size);

  ChunkData chunk_data;
  chunk_data.frame_uuid = transfer.frame_uuid;
  chunk_data.chunk_index = chunk_index;
  chunk_data.chunk_size = chunk_size;

  const uint8_t* chunk_header_bytes =
      reinterpret_cast<const uint8_t*>(&chunk_data);
  message.insert(message.end(), chunk_header_bytes,
                 chunk_header_bytes + sizeof(ChunkData));

  message.insert(message.end(), transfer.frame.data.begin() + offset,
                 transfer.frame.data.begin() + offset + chunk_size);

  send_in_progress = true;
  bool ok = sink->send(message.data(), message.size());
  send_in_progress = false;
  if (ok) stats.bytes_sent += message.size();
  return ok;
}

void StreamManager::cleanupTransfer() {
  std::lock_guard<std::mutex> lock(transfer_mutex);
  if (current_transfer) {
    current_transfer->in_progress = false;
    current_transfer.reset();
    LOG_DEBUG("StreamManager", "Cleaned up transfer");
  }
}

bool StreamManager::isTransferTimedOut() {
  std::lock_guard<std::mutex> lock(transfer_mutex);
  if (!current_transfer || !current_transfer->in_progress) {
    return false;
  }

  auto elapsed =
      std::chrono::steady_clock::now() - current_transfer->start_time;
  return elapsed > CHUNK_TIMEOUT;
}

uint32_t StreamManager::generateFrameUUID() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint32_t> dis;
  return dis(gen);
}

void StreamManager::logStats() const {
  std::stringstream ss;
  ss << "StreamManager Statistics:\n"
     << "  Frames sent: " << stats.frames_sent.load() << "\n"
     << "  Header only frames: " << stats.header_only_frames.load() << "\n"
     << "  Frames dropped: " << stats.frames_dropped.load() << "\n"
     << "  Chunks sent: " << stats.chunks_sent.load() << "\n"
     << "  Bytes sent: " << stats.bytes_sent.load() << "\n"
     << "  Backpressure pauses: " << stats.backpressure_pauses.load() << "\n"
     << "  Final chunk rate: " << rate_controller.getCurrentRate()
     << " chunks/sec\n"
     << "  Mode: " << (header_only_mode ? "Header Only" : "Full Frame");
  LOG_INFO("StreamManager", ss.str());
}
