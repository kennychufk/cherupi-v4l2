#include "stream_manager.hpp"

#include <uWS/App.h>

#include <chrono>
#include <iostream>

StreamManager::StreamManager(CameraManager* mgr) : camera_manager(mgr) {}

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
      // Prepare frame header
      FrameHeader header;
      header.frame_id = frame.frame_id;
      header.camera_id = frame.camera_id;
      header.bytes_per_line = frame.bytes_per_line;
      header.width = frame.width;
      header.height = frame.height;

      // Create a single buffer with header + data
      std::vector<uint8_t> message;
      message.reserve(sizeof(header) + frame.data.size());

      // Copy header
      const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&header);
      message.insert(message.end(), header_bytes,
                     header_bytes + sizeof(header));

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
        break;
      }
    } else {
      // No new frame from this camera, continue to next
      // Small delay to avoid busy loop when no frames are available
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}
