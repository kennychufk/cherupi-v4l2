#include "stream_manager.hpp"

#include <uWS/App.h>

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

void StreamManager::setWebSocket(uWS::WebSocket<false, true, int>* ws) {
  std::lock_guard<std::mutex> lock(ws_mutex);
  ws_connection = ws;
  has_backpressure = false;
  rate_controller.reset();

  LOG_INFO("StreamManager", "WebSocket connection set");

  // Start streaming thread if we have cameras to stream
  if (!streaming_active && !streaming_cameras.empty()) {
    should_stop = false;
    streaming_active = true;
    streaming_thread =
        std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
    LOG_INFO("StreamManager", "Streaming thread started");
  }
}

void StreamManager::clearWebSocket() {
  LOG_INFO("StreamManager", "Clearing WebSocket connection");

  // Stop streaming first
  stopAllStreaming();

  // Clear the WebSocket connection
  {
    std::lock_guard<std::mutex> lock(ws_mutex);
    ws_connection = nullptr;
  }

  // Clean up any in-progress transfer
  cleanupTransfer();

  // Reset rate controller
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

  // Start streaming thread if not already running
  if (!streaming_active) {
    std::lock_guard<std::mutex> lock(ws_mutex);
    if (ws_connection) {
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

    // If this camera is currently being transferred, mark it for cleanup
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

  // Clear streaming cameras
  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.clear();
  }

  // Stop streaming thread
  if (streaming_active) {
    should_stop = true;
    frame_available_cv.notify_all();  // Wake up the streaming thread

    if (streaming_thread && streaming_thread->joinable()) {
      LOG_DEBUG("StreamManager", "Waiting for streaming thread to finish");
      streaming_thread->join();
    }

    streaming_active = false;
  }

  // Clean up any in-progress transfer
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
    frame_available_cv.notify_one();  // Wake up streaming thread
  }
}

void StreamManager::streamingLoop() {
  LOG_INFO("StreamManager", "Streaming loop started");

  std::vector<uint32_t> camera_order;
  size_t current_index = 0;

  while (!should_stop) {
    // Check if we have backpressure
    if (has_backpressure) {
      LOG_DEBUG("StreamManager", "Waiting for backpressure to clear");
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return !has_backpressure || should_stop.load();
      });
      continue;
    }

    // Check WebSocket connection
    {
      std::lock_guard<std::mutex> lock(ws_mutex);
      if (!ws_connection) {
        LOG_DEBUG("StreamManager", "No WebSocket connection, waiting");
        std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
        frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
        continue;
      }
    }

    // Update camera list if needed
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_cameras.empty()) {
        LOG_DEBUG("StreamManager", "No cameras streaming, waiting");
        std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
        frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
        continue;
      }

      // Rebuild camera order if it changed
      if (camera_order.size() != streaming_cameras.size()) {
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

    // Check for timeout on current transfer
    if (isTransferTimedOut()) {
      LOG_WARN("StreamManager", "Transfer timed out, cleaning up");
      cleanupTransfer();
    }

    // Skip to next camera if transfer is in progress
    {
      std::lock_guard<std::mutex> lock(transfer_mutex);
      if (current_transfer && current_transfer->in_progress) {
        LOG_DEBUG("StreamManager",
                  "Transfer in progress, skipping frame check");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
    }

    // Try to get a frame from cameras (round-robin with skipping)
    bool found_frame = false;
    size_t cameras_checked = 0;

    while (cameras_checked < camera_order.size() && !found_frame &&
           !should_stop) {
      uint32_t camera_id = camera_order[current_index];
      current_index = (current_index + 1) % camera_order.size();
      cameras_checked++;

      // Double-check camera is still streaming
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

        // Check if we're in header only mode
        bool success = false;
        if (header_only_mode) {
          // Send only the header, no frame data
          success = sendHeaderOnlyFrame(frame, camera_id);
          if (success) {
            stats.header_only_frames++;
          }
        } else {
          // Send the full frame with chunks
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

        // Release the streaming frame after send attempt
        camera->releaseStreamingFrame();
      }
    }

    // If no frames found, wait efficiently for new frames
    if (!found_frame && !should_stop) {
      LOG_DEBUG("StreamManager", "No frames available, waiting");

      // Wait for any camera to signal new frame available
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(10),
                                  [this] { return should_stop.load(); });
    }
  }

  LOG_INFO("StreamManager", "Streaming loop ended");
}

bool StreamManager::sendHeaderOnlyFrame(const FrameData& frame,
                                        uint32_t camera_id) {
  // For header only mode, we send just the chunk header with total_chunks = 0
  // and total_size = 0, but preserve width, height, and bytes_per_line

  // Create a minimal transfer structure for the header
  ChunkedTransfer transfer;
  transfer.frame = frame;
  transfer.frame_uuid = generateFrameUUID();
  transfer.total_chunks = 0;  // Indicates header only mode
  transfer.current_chunk = 0;
  transfer.start_time = std::chrono::steady_clock::now();
  transfer.in_progress = false;  // No actual transfer

  LOG_DEBUG("StreamManager", "Sending header only for frame " +
                                 std::to_string(frame.frame_id) +
                                 " from camera " + std::to_string(camera_id));

  // Send chunk header with header_only flag
  return sendChunkHeader(transfer, true);
}

bool StreamManager::sendChunkedFrame(const FrameData& frame,
                                     uint32_t camera_id) {
  // Prepare chunked transfer
  {
    std::lock_guard<std::mutex> lock(transfer_mutex);
    current_transfer = std::make_unique<ChunkedTransfer>();
    current_transfer->frame = frame;  // Deep copy
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

  // Send chunk header
  if (!sendChunkHeader(*current_transfer,
                       false)) {  // Pass false for normal mode
    LOG_ERROR("StreamManager", "Failed to send chunk header");
    cleanupTransfer();
    return false;
  }

  // Track timing for adaptive rate control
  auto last_chunk_time = std::chrono::steady_clock::now();

  // Send all chunks with adaptive pacing
  for (size_t chunk_idx = 0; chunk_idx < current_transfer->total_chunks;
       chunk_idx++) {
    // Check if we should stop or if transfer was cancelled
    if (should_stop) {
      cleanupTransfer();
      return false;
    }

    // Check if camera is still streaming
    if (!isStreamingCamera(camera_id)) {
      LOG_INFO("StreamManager", "Camera " + std::to_string(camera_id) +
                                    " stopped during transfer");
      cleanupTransfer();
      return false;
    }

    // Check for backpressure and wait if necessary
    int backpressure_wait_count = 0;
    while (has_backpressure && backpressure_wait_count < 50) {  // Max 5 seconds
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

    // Apply adaptive delay based on rate controller
    auto chunk_delay = rate_controller.getChunkDelay();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_chunk_time;

    if (elapsed < chunk_delay) {
      std::this_thread::sleep_for(chunk_delay - elapsed);
    }

    last_chunk_time = std::chrono::steady_clock::now();

    // Send chunk
    if (!sendChunkData(*current_transfer, chunk_idx)) {
      LOG_ERROR("StreamManager",
                "Failed to send chunk " + std::to_string(chunk_idx) + "/" +
                    std::to_string(current_transfer->total_chunks));
      cleanupTransfer();
      return false;
    }

    current_transfer->current_chunk = chunk_idx + 1;
    stats.chunks_sent++;

    // Record successful chunk send for rate adaptation
    if (!has_backpressure) {
      rate_controller.recordChunkSent();
    }

    // Log progress periodically for debugging
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

  // Clean up completed transfer
  cleanupTransfer();
  return true;
}

bool StreamManager::sendChunkHeader(const ChunkedTransfer& transfer,
                                    bool header_only) {
  std::lock_guard<std::mutex> lock(ws_mutex);
  if (!ws_connection) return false;

  // Prepare chunk header message
  std::vector<uint8_t> message;
  message.reserve(sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkStartMarker start_marker;
  const uint8_t* start_bytes = reinterpret_cast<const uint8_t*>(&start_marker);
  message.insert(message.end(), start_bytes,
                 start_bytes + sizeof(ChunkStartMarker));

  ChunkHeader header;
  header.frame_uuid = transfer.frame_uuid;
  header.frame_id = transfer.frame.frame_id;
  header.camera_id = transfer.frame.camera_id;

  // For header only mode, set total_chunks and total_size to 0
  if (header_only) {
    header.total_chunks = 0;
    header.total_size = 0;
  } else {
    header.total_chunks = transfer.total_chunks;
    header.total_size = transfer.frame.data.size();
  }

  // Always preserve these fields regardless of mode
  header.bytes_per_line = transfer.frame.bytes_per_line;
  header.width = transfer.frame.width;
  header.height = transfer.frame.height;

  header.pixel_format = transfer.frame.pixel_format;
  header.frames_saved =
      frame_saver
          ? frame_saver->getFramesSavedForCamera(transfer.frame.camera_id)
          : 0;

  const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&header);
  message.insert(message.end(), header_bytes,
                 header_bytes + sizeof(ChunkHeader));

  try {
    send_in_progress = true;
    std::string_view message_view(reinterpret_cast<const char*>(message.data()),
                                  message.size());
    ws_connection->send(message_view, uWS::OpCode::BINARY);
    send_in_progress = false;

    stats.bytes_sent += message.size();
    return true;
  } catch (...) {
    send_in_progress = false;
    LOG_ERROR("StreamManager", "Exception while sending chunk header");
    return false;
  }
}

bool StreamManager::sendChunkData(const ChunkedTransfer& transfer,
                                  size_t chunk_index) {
  std::lock_guard<std::mutex> lock(ws_mutex);
  if (!ws_connection) return false;

  size_t offset = chunk_index * CHUNK_SIZE;
  size_t chunk_size = std::min(CHUNK_SIZE, transfer.frame.data.size() - offset);

  // Prepare chunk data message
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

  // Add chunk payload
  message.insert(message.end(), transfer.frame.data.begin() + offset,
                 transfer.frame.data.begin() + offset + chunk_size);

  try {
    send_in_progress = true;
    std::string_view message_view(reinterpret_cast<const char*>(message.data()),
                                  message.size());
    ws_connection->send(message_view, uWS::OpCode::BINARY);
    send_in_progress = false;

    stats.bytes_sent += message.size();
    return true;
  } catch (...) {
    send_in_progress = false;
    LOG_ERROR("StreamManager", "Exception while sending chunk data");
    return false;
  }
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
