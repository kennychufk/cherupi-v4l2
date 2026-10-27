#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "camera_manager.hpp"
#include "capture_frame.hpp"
#include "frame_processor.hpp"
#include "frame_sink.hpp"
#include "rate_controller.hpp"
#include "stream_priority.hpp"
#include "types.hpp"

// Chunked transfer state for a single frame
struct ChunkedTransfer {
  // Shared read-only with the camera's / detector's publication slot and,
  // possibly, the other role's streamer — never copied. A subsampled stream
  // points at its own decimated frame instead.
  std::shared_ptr<const FrameData> frame;
  // Detection corners for this frame (empty in non-detector modes, or when
  // detection ran but found nothing). Travels with the frame instead of being
  // looked up: in detector modes the streamer only ever sends frames the
  // detector already processed, so the corners are known up front.
  std::vector<CornerSet> corner_sets;
  // How to serialize corner_sets into the frame header's detection block:
  // DetectionKind cast to uint8_t (NONE/CHECKERBOARD/ARUCO). Determines whether
  // each record is a CornerSetHeader or a MarkerSetHeader.
  uint8_t detection_kind = static_cast<uint8_t>(DetectionKind::NONE);
  uint32_t frame_uuid = 0;
  size_t total_chunks = 0;
  size_t current_chunk = 0;
  std::chrono::steady_clock::time_point start_time;
  bool in_progress = false;
  // An on-demand capture frame: not tied to a subscription, so unsubscribing
  // its camera does not abort it.
  bool capture = false;
};

// One client connection's streaming session: the cameras it subscribed to
// (with per-camera options), its header-only flag, its rate control, and the
// thread that pulls frames and chunks them into its FrameSink. The server owns
// one per client role — they outlive individual connections, which merely
// attach and detach a sink — and tells the observer's session to yield to the
// commander's (setYieldTo).
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
  // Longest an in-flight lower-priority frame keeps waiting for the
  // higher-priority connection to clear before being abandoned. Matches
  // CHUNK_TIMEOUT and the clients' incomplete-frame garbage collection (5 s).
  static constexpr auto PRIORITY_WAIT_LIMIT = std::chrono::seconds(5);

  StreamManager(CameraManager* mgr, FrameProcessor* saver,
                ClientRole role = ClientRole::COMMANDER);
  ~StreamManager();

  ClientRole getRole() const { return role_; }

  // Make this session yield to `higher` (see stream_priority.hpp): frames and
  // chunks are handed to the sink only while `higher` is clear. nullptr (the
  // default) never yields. `higher` must outlive this object.
  void setYieldTo(const StreamManager* higher) { yield_to_ = higher; }

  // Connect/disconnect a FrameSink (owned by the caller). clearSink also
  // drops every subscription: a subscription belongs to a connection.
  void setSink(FrameSink* sink);
  void clearSink();

  // Subscribe `camera_id` (must be a discovered camera) with `options`;
  // re-issuing for an already-subscribed camera updates its options. Whether
  // frames actually flow depends on the cameras running — a subscription
  // made while they are stopped simply waits.
  bool startStreamingCamera(uint32_t camera_id, StreamOptions options = {});
  bool stopStreamingCamera(uint32_t camera_id);
  // Unsubscribe every camera whose id is `camera_count` or above: after a
  // discover for another sensor, ids past the new set name no camera.
  void stopStreamingCamerasFrom(uint32_t camera_count);
  // Drop every subscription and stop the streaming thread; queued capture
  // frames go with it.
  void stopAllStreaming();
  // Drop every subscription but keep the thread, so capture frames already
  // queued still reach the client (stop_cameras).
  void clearSubscriptions();

  // Queue the frames of an on-demand capture (trigger_capture,
  // stack_capture) for this session's client. Each goes out once, in order
  // and ahead of any live frame, whatever the subscriptions, max_fps,
  // subsample or header-only state; a transfer cut short (backpressure) is
  // retried, not dropped. Dropped only when no client is attached, now or by
  // the time it would be sent.
  void enqueueCaptures(std::vector<CaptureFrame> frames);
  // Capture frames still queued or being sent.
  bool hasPendingCaptures() const;

  // Header only mode control
  void setHeaderOnlyMode(bool enabled) {
    LOG_INFO("StreamManager",
             std::string(clientRoleName(role_)) + ": setting header only mode to " +
                 std::string(enabled ? "enabled" : "disabled"));
    header_only_mode = enabled;
  }
  bool isHeaderOnlyMode() const { return header_only_mode; }

  // Query methods
  bool isStreamingCamera(uint32_t camera_id) const;
  size_t getStreamingCameraCount() const;
  std::optional<StreamOptions> getStreamOptions(uint32_t camera_id) const;

  // Backpressure notification from the sink owner.
  void notifyBackpressure(bool has_pressure);

  // What a lower-priority session sees of this one, and the verdict it draws
  // from it (stream_priority::isClear against SOFT_BUFFER_LIMIT).
  stream_priority::Snapshot prioritySnapshot() const;
  bool isClearForLowerPriority() const;

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

  // Per-subscribed-camera state.
  struct CameraStream {
    StreamOptions options;
    // Cursors into the two frame sources (Camera::getFrameForStreaming and
    // FrameProcessor::getDetectedFrameForStreaming). Per session, so the two
    // roles consume the shared slots independently.
    uint64_t camera_cursor = 0;
    uint64_t detected_cursor = 0;
    // When this camera's last frame went out, for max_fps pacing.
    std::chrono::steady_clock::time_point last_sent{};
  };

  CameraManager* camera_manager;
  FrameProcessor* frame_processor;
  const ClientRole role_;
  const StreamManager* yield_to_ = nullptr;

  // Streaming state
  std::map<uint32_t, CameraStream> streaming_cameras;
  mutable std::mutex streaming_mutex;

  std::atomic<bool> header_only_mode{false};

  std::unique_ptr<std::thread> streaming_thread;
  std::atomic<bool> should_stop{false};
  std::atomic<bool> streaming_active{false};

  std::condition_variable frame_available_cv;
  std::mutex frame_wait_mutex;

  // Sink (abstract transport)
  mutable std::mutex sink_mutex;
  FrameSink* sink = nullptr;
  std::atomic<bool> has_backpressure{false};
  // steady_clock microseconds of the last entry into backpressure; 0 = never.
  std::atomic<int64_t> last_backpressure_us{0};
  std::atomic<bool> send_in_progress{false};

  std::unique_ptr<ChunkedTransfer> current_transfer;
  std::mutex transfer_mutex;

  // On-demand capture frames awaiting the sink, oldest first, and whether one
  // has been taken off the queue and is being sent.
  std::deque<CaptureFrame> captures_;
  bool capture_in_flight_ = false;  // guarded by captures_mutex_
  mutable std::mutex captures_mutex_;

  AdaptiveRateController rate_controller;

  struct StreamStats {
    std::atomic<uint64_t> frames_sent{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> chunks_sent{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> backpressure_pauses{0};
    std::atomic<uint64_t> header_only_frames{0};
    // Frames abandoned mid-transfer because the higher-priority connection
    // stayed congested past PRIORITY_WAIT_LIMIT.
    std::atomic<uint64_t> priority_abandoned{0};
  } stats;

  std::string tag() const;  // log prefix carrying the role
  // Start the streaming thread unless it is running; needs sink_mutex held
  // and a sink attached.
  void startThreadLocked();
  void streamingLoop();
  // Send the oldest queued capture frame, if any. Returns whether one was
  // taken off the queue (sent, or dropped because the client went away).
  bool sendNextCapture();
  // Block the streaming thread until sink->bufferedAmount() drops below
  // SOFT_BUFFER_LIMIT, or `should_stop` fires, or we've waited too long.
  // Returns true if the caller should proceed, false if it should bail.
  bool waitForBufferDrain();
  // Lower-priority sessions only: block until the higher-priority connection
  // is clear. Returns false if `should_stop` fired or `deadline` passed.
  bool waitForPriorityClear(std::chrono::steady_clock::time_point deadline);
  bool sendChunkedFrame(std::shared_ptr<const FrameData> frame,
                        uint32_t camera_id,
                        std::vector<CornerSet> corner_sets,
                        uint8_t detection_kind, bool capture = false);
  bool sendHeaderOnlyFrame(std::shared_ptr<const FrameData> frame,
                           uint32_t camera_id,
                           std::vector<CornerSet> corner_sets,
                           uint8_t detection_kind);
  bool sendChunkHeader(const ChunkedTransfer& transfer, bool header_only);
  bool sendChunkData(const ChunkedTransfer& transfer, size_t chunk_index);
  void cleanupTransfer();
  bool isTransferTimedOut();
  uint32_t generateFrameUUID();
};
