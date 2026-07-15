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
  // Detection corners for this frame (empty in non-detector modes, or when
  // detection ran but found nothing). Travels with the frame instead of being
  // looked up: in detector modes the streamer only ever sends frames the
  // detector already processed, so the corners are known up front.
  std::vector<CornerSet> corner_sets;
  // How to serialize corner_sets into the frame header's detection block:
  // DetectionKind cast to uint8_t (NONE/CHECKERBOARD/ARUCO). Determines whether
  // each record is a CornerSetHeader or a MarkerSetHeader.
  uint8_t detection_kind = static_cast<uint8_t>(DetectionKind::NONE);
  uint32_t frame_uuid;
  size_t total_chunks;
  size_t current_chunk;
  std::chrono::steady_clock::time_point start_time;
  bool in_progress;
};

class StreamManager {
 public:
  static constexpr size_t CHUNK_SIZE = 32768;  // Fixed 32KB chunks
  // Soft cap on the sink's bufferedAmount() that the streaming thread
  // enforces proactively. Without this, on a real LAN the server can
  // queue several MB of in-flight binary chunks before the network drains
  // them; a TEXT control reply that lands behind that backlog reaches the
  // client late, and a Node client whose event loop is swamped processing
  // the burst of binary 'message' events can take >30 s to issue its
  // next outbound command — long enough to trip test timeouts.
  // 512 KB ≈ ~5 ms drain at 1 Gbps wired LAN, which keeps text replies
  // responsive without choking binary throughput.
  static constexpr size_t SOFT_BUFFER_LIMIT = 512 * 1024;

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
  // `corner_sets` are encoded into the header's detection block as if they came
  // from the detector; the block format (checkerboard vs aruco) is inferred
  // from the sets (any set with marker_id >= 0 ⇒ aruco). Returns true if every
  // chunk was written to the sink.
  bool sendFrameForTest(const FrameData& frame, uint32_t camera_id,
                        std::vector<CornerSet> corner_sets = {});

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
  // Block the streaming thread until sink->bufferedAmount() drops below
  // SOFT_BUFFER_LIMIT, or `should_stop` fires, or we've waited too long.
  // Returns true if the caller should proceed, false if it should bail.
  bool waitForBufferDrain();
  bool sendChunkedFrame(const FrameData& frame, uint32_t camera_id,
                        std::vector<CornerSet> corner_sets,
                        uint8_t detection_kind);
  bool sendHeaderOnlyFrame(const FrameData& frame, uint32_t camera_id,
                           std::vector<CornerSet> corner_sets,
                           uint8_t detection_kind);
  bool sendChunkHeader(const ChunkedTransfer& transfer, bool header_only);
  bool sendChunkData(const ChunkedTransfer& transfer, size_t chunk_index);
  void cleanupTransfer();
  bool isTransferTimedOut();
  uint32_t generateFrameUUID();
};
