#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <thread>

#include "camera_manager.hpp"
#include "types.hpp"

// Forward declaration for uWebSockets
namespace uWS {
template <bool SSL, bool isServer, typename USERDATA>
struct WebSocket;
}

// Chunked transfer state for a single frame
struct ChunkedTransfer {
  FrameData frame;
  uint32_t frame_uuid;
  size_t total_chunks;
  size_t current_chunk;
  std::chrono::steady_clock::time_point start_time;
  bool in_progress;
};

class StreamManager {
 private:
  static constexpr size_t CHUNK_SIZE = 32768;  // Fixed 32KB chunks
  static constexpr auto CHUNK_TIMEOUT = std::chrono::seconds(5);

  CameraManager* camera_manager;

  // Streaming state
  std::set<uint32_t> streaming_cameras;
  std::mutex streaming_mutex;

  // Streaming thread and control
  std::unique_ptr<std::thread> streaming_thread;
  std::atomic<bool> should_stop{false};
  std::atomic<bool> streaming_active{false};

  // Condition variable for efficient frame waiting
  std::condition_variable frame_available_cv;
  std::mutex frame_wait_mutex;

  // WebSocket connection
  std::mutex ws_mutex;
  uWS::WebSocket<false, true, int>* ws_connection = nullptr;
  std::atomic<bool> has_backpressure{false};
  std::atomic<bool> send_in_progress{false};

  // Current chunked transfer state
  std::unique_ptr<ChunkedTransfer> current_transfer;
  std::mutex transfer_mutex;

  // Statistics
  struct StreamStats {
    std::atomic<uint64_t> frames_sent{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> chunks_sent{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> backpressure_pauses{0};
  } stats;

  // Internal methods
  void streamingLoop();
  bool sendChunkedFrame(const FrameData& frame, uint32_t camera_id);
  bool sendChunkHeader(const ChunkedTransfer& transfer);
  bool sendChunkData(const ChunkedTransfer& transfer, size_t chunk_index);
  void cleanupTransfer();
  bool isTransferTimedOut();
  uint32_t generateFrameUUID();

 public:
  StreamManager(CameraManager* mgr);
  ~StreamManager();

  // WebSocket management
  void setWebSocket(uWS::WebSocket<false, true, int>* ws);
  void clearWebSocket();

  // Stream control
  bool startStreamingCamera(uint32_t camera_id);
  bool stopStreamingCamera(uint32_t camera_id);
  void stopAllStreaming();

  // Query methods
  bool isStreamingCamera(uint32_t camera_id);
  size_t getStreamingCameraCount();

  // Backpressure notification from WebSocket
  void notifyBackpressure(bool has_pressure);

  // Statistics
  void logStats() const;

  // Notify that new frames might be available
  void notifyFrameAvailable() { frame_available_cv.notify_one(); }
};
