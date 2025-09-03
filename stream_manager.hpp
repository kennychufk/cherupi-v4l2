// stream_manager.hpp - Updated with header only mode support
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
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

// Adaptive rate control for chunk sending
class AdaptiveRateController {
 private:
  static constexpr size_t WINDOW_SIZE = 20;  // Number of chunks to track
  static constexpr double MIN_CHUNKS_PER_SECOND = 50.0;    // Minimum rate
  static constexpr double MAX_CHUNKS_PER_SECOND = 1000.0;  // Maximum rate
  static constexpr double RATE_INCREASE_FACTOR = 1.1;      // Increase by 10%
  static constexpr double RATE_DECREASE_FACTOR = 0.5;      // Decrease by 50%

  std::deque<std::chrono::steady_clock::time_point> chunk_send_times;
  std::chrono::steady_clock::time_point last_backpressure_time;
  double current_rate{200.0};   // Start with moderate rate
  double smoothed_rate{200.0};  // Exponentially weighted moving average
  std::atomic<int> consecutive_successes{0};
  std::atomic<int> backpressure_events{0};
  std::mutex rate_mutex;

 public:
  AdaptiveRateController() {
    last_backpressure_time =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }

  // Calculate delay before sending next chunk
  std::chrono::microseconds getChunkDelay() {
    std::lock_guard<std::mutex> lock(rate_mutex);

    // Calculate delay based on current rate
    double delay_ms = 1000.0 / smoothed_rate;
    return std::chrono::microseconds(static_cast<int64_t>(delay_ms * 1000));
  }

  // Record successful chunk send
  void recordChunkSent() {
    std::lock_guard<std::mutex> lock(rate_mutex);

    auto now = std::chrono::steady_clock::now();
    chunk_send_times.push_back(now);

    // Keep only recent history
    while (chunk_send_times.size() > WINDOW_SIZE) {
      chunk_send_times.pop_front();
    }

    consecutive_successes++;

    // Gradually increase rate if we've had many successes
    if (consecutive_successes > 10) {
      adjustRate(RATE_INCREASE_FACTOR);
      consecutive_successes = 0;
    }
  }

  // Record backpressure event
  void recordBackpressure() {
    std::lock_guard<std::mutex> lock(rate_mutex);

    auto now = std::chrono::steady_clock::now();
    auto time_since_last =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_backpressure_time)
            .count();

    last_backpressure_time = now;
    backpressure_events++;
    consecutive_successes = 0;

    // Aggressively reduce rate on backpressure
    if (time_since_last < 1000) {
      // Multiple backpressure events in quick succession - reduce more
      adjustRate(RATE_DECREASE_FACTOR * 0.7);
    } else {
      adjustRate(RATE_DECREASE_FACTOR);
    }
  }

  // Reset controller state
  void reset() {
    std::lock_guard<std::mutex> lock(rate_mutex);
    chunk_send_times.clear();
    current_rate = 200.0;
    smoothed_rate = 200.0;
    consecutive_successes = 0;
    backpressure_events = 0;
  }

  // Get current sending rate
  double getCurrentRate() const { return smoothed_rate; }

 private:
  void adjustRate(double factor) {
    current_rate *= factor;
    current_rate = std::max(MIN_CHUNKS_PER_SECOND,
                            std::min(MAX_CHUNKS_PER_SECOND, current_rate));

    // Apply exponential smoothing to avoid rapid changes
    smoothed_rate = 0.7 * smoothed_rate + 0.3 * current_rate;
  }
};

class StreamManager {
 private:
  static constexpr size_t CHUNK_SIZE = 32768;  // Fixed 32KB chunks
  static constexpr auto CHUNK_TIMEOUT = std::chrono::seconds(5);
  CameraManager* camera_manager;

  // Streaming state
  std::set<uint32_t> streaming_cameras;
  std::mutex streaming_mutex;

  // Header only mode
  std::atomic<bool> header_only_mode{false};

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

  // Adaptive rate control
  AdaptiveRateController rate_controller;

  // Statistics
  struct StreamStats {
    std::atomic<uint64_t> frames_sent{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> chunks_sent{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> backpressure_pauses{0};
    std::atomic<uint64_t> header_only_frames{0};
  } stats;

  // Internal methods
  void streamingLoop();
  bool sendChunkedFrame(const FrameData& frame, uint32_t camera_id);
  bool sendHeaderOnlyFrame(const FrameData& frame, uint32_t camera_id);
  bool sendChunkHeader(const ChunkedTransfer& transfer,
                       bool header_only = false);
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

  // Header only mode control
  void setHeaderOnlyMode(bool enabled) {
    LOG_INFO("StreamManager",
             "Setting header only mode to " +
                 std::string(enabled ? "enabled" : "disabled"));
    header_only_mode = enabled;
  }
  bool isHeaderOnlyMode() const { return header_only_mode; }

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
