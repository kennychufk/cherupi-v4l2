#include "stream_manager.hpp"

#include <uWS/App.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>

uint32_t generate_random_uint32() {
  static std::random_device rd;
  static std::mt19937 engine(rd());
  static std::uniform_int_distribution<uint32_t> dist(
      std::numeric_limits<uint32_t>::min(),
      std::numeric_limits<uint32_t>::max());
  return dist(engine);
}

StreamManager::StreamManager(CameraManager* mgr) : camera_manager(mgr) {
  // Set chunk size based on environment variable or use default
  const char* chunk_size_env = std::getenv("CHUNK_SIZE");
  if (chunk_size_env) {
    CHUNK_SIZE = std::stoul(chunk_size_env);
    std::cout << "Using chunk size from environment: " << CHUNK_SIZE << " bytes"
              << std::endl;
  } else {
    std::cout << "Using default chunk size: " << CHUNK_SIZE << " bytes"
              << std::endl;
  }
}

StreamManager::~StreamManager() { stopAllStreaming(); }

void StreamManager::setWebSocket(uWS::WebSocket<false, true, int>* ws) {
  ws_connection = ws;
  has_backpressure = false;

  if (!streaming_active && !streaming_cameras.empty()) {
    // Start streaming thread if we have cameras to stream
    streaming_active = true;
    streaming_thread =
        std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
  }
}

void StreamManager::clearWebSocket() {
  ws_connection = nullptr;
  stopAllStreaming();
}

bool StreamManager::startStreamingCamera(uint32_t camera_id) {
  if (camera_id >= camera_manager->getCameraCount()) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.insert(camera_id);
  }

  // Start streaming thread if not already running
  if (!streaming_active && ws_connection) {
    streaming_active = true;
    streaming_thread =
        std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
  }

  return true;
}

bool StreamManager::stopStreamingCamera(uint32_t camera_id) {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  return streaming_cameras.erase(camera_id) > 0;
}

void StreamManager::stopAllStreaming() {
  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.clear();
  }

  // Stop streaming thread
  if (streaming_active) {
    streaming_active = false;
    if (streaming_thread && streaming_thread->joinable()) {
      streaming_thread->join();
    }
  }
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
  has_backpressure = has_pressure;
}

void StreamManager::streamingLoop() {
  std::vector<uint32_t> camera_order;
  size_t current_index = 0;

  while (streaming_active) {
    // If no WebSocket connection or backpressure, wait
    if (!ws_connection || has_backpressure) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Update camera list if needed
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_cameras.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // Rebuild camera order if it changed
      if (camera_order.size() != streaming_cameras.size()) {
        camera_order.clear();
        for (uint32_t cam_id : streaming_cameras) {
          camera_order.push_back(cam_id);
        }
        current_index = 0;
      }
    }

    // Round-robin through cameras
    uint32_t camera_id = camera_order[current_index];
    current_index = (current_index + 1) % camera_order.size();

    Camera* camera = camera_manager->getCamera(camera_id);
    if (!camera) continue;

    FrameData frame;
    if (camera->getLatestFrame(frame)) {
      // Send frame with chunking if necessary
      sendFrameChunked(frame);
    } else {
      // No new frame from this camera, continue to next
      // Small delay to avoid busy loop when no frames are available
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}

void StreamManager::sendFrameChunked(const FrameData& frame) {
  if (!ws_connection) return;

  // Prepare frame header
  FrameHeader header;
  header.frame_id = frame.frame_id;
  header.camera_id = frame.camera_id;
  header.bytes_per_line = frame.bytes_per_line;
  header.width = frame.width;
  header.height = frame.height;

  // Calculate total message size
  size_t header_size = sizeof(FrameHeader);
  size_t total_size = header_size + frame.data.size();

  // If frame is small enough, send as single message
  if (total_size <= CHUNK_SIZE) {
    // Create a single buffer with header + data
    std::vector<uint8_t> message;
    message.reserve(total_size);

    // Copy header
    const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&header);
    message.insert(message.end(), header_bytes, header_bytes + header_size);

    // Copy frame data
    message.insert(message.end(), frame.data.begin(), frame.data.end());

    // Send as binary message
    try {
      std::string_view message_view(
          reinterpret_cast<const char*>(message.data()), message.size());
      ws_connection->send(message_view, uWS::OpCode::BINARY);
    } catch (...) {
      // WebSocket might have disconnected
      ws_connection = nullptr;
    }
    return;
  }

  // Frame is too large, use chunked transfer
  std::cout << "Frame " << frame.frame_id << " from camera " << frame.camera_id
            << " is " << frame.data.size()
            << " bytes, using chunked transfer with " << CHUNK_SIZE
            << " byte chunks" << std::endl;

  uint32_t frame_uuid = generate_random_uint32();
  // Send chunk header first
  ChunkHeader chunk_header;
  chunk_header.frame_uuid = frame_uuid;
  chunk_header.frame_id = frame.frame_id;
  chunk_header.camera_id = frame.camera_id;
  chunk_header.total_chunks = (frame.data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
  chunk_header.total_size = frame.data.size();
  chunk_header.bytes_per_line = frame.bytes_per_line;
  chunk_header.width = frame.width;
  chunk_header.height = frame.height;

  // Send initial chunk header message
  std::vector<uint8_t> chunk_start_msg;
  chunk_start_msg.reserve(sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkStartMarker start_marker;
  const uint8_t* start_bytes = reinterpret_cast<const uint8_t*>(&start_marker);
  chunk_start_msg.insert(chunk_start_msg.end(), start_bytes,
                         start_bytes + sizeof(ChunkStartMarker));

  const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&chunk_header);
  chunk_start_msg.insert(chunk_start_msg.end(), header_bytes,
                         header_bytes + sizeof(ChunkHeader));

  try {
    std::string_view start_view(
        reinterpret_cast<const char*>(chunk_start_msg.data()),
        chunk_start_msg.size());
    ws_connection->send(start_view, uWS::OpCode::BINARY);
  } catch (...) {
    ws_connection = nullptr;
    return;
  }

  // Send data chunks
  size_t offset = 0;
  uint32_t chunk_index = 0;

  while (offset < frame.data.size() && ws_connection) {
    // Check if this camera should still be streaming
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_cameras.find(frame.camera_id) == streaming_cameras.end()) {
        // Camera was stopped mid-transfer, abort sending remaining chunks
        std::cout << "Camera " << frame.camera_id
                  << " stopped mid-transfer, aborting remaining chunks"
                  << std::endl;
        return;
      }
    }

    size_t chunk_size = std::min(CHUNK_SIZE, frame.data.size() - offset);

    // Create chunk message
    std::vector<uint8_t> chunk_msg;
    chunk_msg.reserve(sizeof(ChunkData) + chunk_size);

    ChunkData chunk_data;
    chunk_data.frame_uuid = frame_uuid;
    chunk_data.chunk_index = chunk_index;
    chunk_data.chunk_size = chunk_size;

    // Debug: Log chunk details
    if (chunk_index == 0) {
      std::cout << "Sending chunk " << chunk_index << " with magic 0x"
                << std::hex << chunk_data.magic << std::dec << " for frame "
                << frame.frame_id << " camera " << frame.camera_id << std::endl;
    }

    const uint8_t* chunk_header_bytes =
        reinterpret_cast<const uint8_t*>(&chunk_data);
    chunk_msg.insert(chunk_msg.end(), chunk_header_bytes,
                     chunk_header_bytes + sizeof(ChunkData));

    // Add chunk data
    chunk_msg.insert(chunk_msg.end(), frame.data.begin() + offset,
                     frame.data.begin() + offset + chunk_size);

    try {
      std::string_view chunk_view(
          reinterpret_cast<const char*>(chunk_msg.data()), chunk_msg.size());
      ws_connection->send(chunk_view, uWS::OpCode::BINARY);
    } catch (...) {
      ws_connection = nullptr;
      return;
    }

    offset += chunk_size;
    chunk_index++;

    // Add small delay between chunks to avoid overwhelming the connection
    if (chunk_index < chunk_header.total_chunks) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}
