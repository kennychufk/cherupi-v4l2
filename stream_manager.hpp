#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

#include "camera_manager.hpp"
#include "types.hpp"

// Forward declaration for uWebSockets
namespace uWS {
template <bool SSL, bool isServer, typename USERDATA>
struct WebSocket;
}

class StreamManager {
 private:
  CameraManager* camera_manager;
  std::set<uint32_t> streaming_cameras;
  std::mutex streaming_mutex;

  std::atomic<bool> streaming_active{false};
  std::unique_ptr<std::thread> streaming_thread;

  // WebSocket connection
  uWS::WebSocket<false, true, int>* ws_connection = nullptr;
  std::atomic<bool> has_backpressure{false};

  void streamingLoop();
  void sendFrameChunked(const FrameData& frame);

 public:
  StreamManager(CameraManager* mgr);
  ~StreamManager();

  void setWebSocket(uWS::WebSocket<false, true, int>* ws);
  void clearWebSocket();

  bool startStreamingCamera(uint32_t camera_id);
  bool stopStreamingCamera(uint32_t camera_id);
  void stopAllStreaming();

  bool isStreamingCamera(uint32_t camera_id);
  size_t getStreamingCameraCount();

  void notifyBackpressure(bool has_pressure);

  // Allow chunk size to be configured for testing
  void setChunkSize(size_t size) { CHUNK_SIZE = size; }

 private:
  size_t CHUNK_SIZE = 32768;  // Make it non-const so it can be changed
};
