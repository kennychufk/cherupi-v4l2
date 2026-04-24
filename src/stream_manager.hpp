#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

#include "camera_manager.hpp"
#include "frame_saver.hpp"
#include "frame_sink.hpp"
#include "rate_controller.hpp"
#include "types.hpp"

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
 public:
  static constexpr size_t CHUNK_SIZE = 32768;  // Fixed 32KB chunks

  StreamManager(CameraManager* mgr, FrameSaver* saver);
  ~StreamManager();

  // Connect/disconnect a FrameSink (owned by the caller).
  void setSink(FrameSink* sink);
  void clearSink();

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

  // Backpressure notification from the sink owner.
  void notifyBackpressure(bool has_pressure);

  // Statistics
  void logStats() const;

  // Notify that new frames might be available
  void notifyFrameAvailable() { frame_available_cv.notify_one(); }

  // Send a single frame synchronously on the calling thread.
  // Exposed for unit testing the chunking path without the streaming loop.
  // Returns true if every chunk was successfully written to the sink.
  bool sendFrameForTest(const FrameData& frame, uint32_t camera_id);

 private:
  static constexpr auto CHUNK_TIMEOUT = std::chrono::seconds(5);

  CameraManager* camera_manager;
  FrameSaver* frame_saver;

  // Streaming state
  std::set<uint32_t> streaming_cameras;
  std::mutex streaming_mutex;

  std::atomic<bool> header_only_mode{false};

  std::unique_ptr<std::thread> streaming_thread;
  std::atomic<bool> should_stop{false};
  std::atomic<bool> streaming_active{false};

  std::condition_variable frame_available_cv;
  std::mutex frame_wait_mutex;

  // Sink (abstract transport)
  std::mutex sink_mutex;
  FrameSink* sink = nullptr;
  std::atomic<bool> has_backpressure{false};
  std::atomic<bool> send_in_progress{false};

  std::unique_ptr<ChunkedTransfer> current_transfer;
  std::mutex transfer_mutex;

  AdaptiveRateController rate_controller;

  struct StreamStats {
    std::atomic<uint64_t> frames_sent{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> chunks_sent{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> backpressure_pauses{0};
    std::atomic<uint64_t> header_only_frames{0};
  } stats;

  void streamingLoop();
  bool sendChunkedFrame(const FrameData& frame, uint32_t camera_id);
  bool sendHeaderOnlyFrame(const FrameData& frame, uint32_t camera_id);
  bool sendChunkHeader(const ChunkedTransfer& transfer, bool header_only);
  bool sendChunkData(const ChunkedTransfer& transfer, size_t chunk_index);
  void cleanupTransfer();
  bool isTransferTimedOut();
  uint32_t generateFrameUUID();
};
