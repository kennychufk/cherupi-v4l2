#include "stream_manager.hpp"

#include <algorithm>
#include <random>
#include <sstream>

#include "frame_subsample.hpp"

namespace {

int64_t nowSteadyUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

StreamManager::StreamManager(CameraManager* mgr, FrameProcessor* saver,
                             ClientRole role)
    : camera_manager(mgr), frame_processor(saver), role_(role) {
  LOG_INFO("StreamManager",
           tag() + "Initialized with adaptive rate control, chunk size: " +
               std::to_string(CHUNK_SIZE) + " bytes");
}

StreamManager::~StreamManager() {
  LOG_INFO("StreamManager", tag() + "Shutting down");
  stopAllStreaming();

  // Wait for any in-progress sends to complete
  auto start = std::chrono::steady_clock::now();
  while (send_in_progress.load() &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  logStats();
}

std::string StreamManager::tag() const {
  return std::string("[") + clientRoleName(role_) + "] ";
}

void StreamManager::setSink(FrameSink* new_sink) {
  std::lock_guard<std::mutex> lock(sink_mutex);
  sink = new_sink;
  has_backpressure = false;
  last_backpressure_us = 0;
  rate_controller.reset();

  LOG_INFO("StreamManager", tag() + "Sink attached");

  if (!streaming_active && getStreamingCameraCount() > 0) {
    should_stop = false;
    streaming_active = true;
    streaming_thread =
        std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
    LOG_INFO("StreamManager", tag() + "Streaming thread started");
  }
}

void StreamManager::clearSink() {
  LOG_INFO("StreamManager", tag() + "Clearing sink");

  stopAllStreaming();

  {
    std::lock_guard<std::mutex> lock(sink_mutex);
    sink = nullptr;
  }

  cleanupTransfer();
  rate_controller.reset();
}

bool StreamManager::startStreamingCamera(uint32_t camera_id,
                                         StreamOptions options) {
  if (camera_id >= camera_manager->getCameraCount()) {
    LOG_ERROR("StreamManager",
              tag() + "Invalid camera ID: " + std::to_string(camera_id));
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    // Keep the cursors of an existing subscription so an options update
    // doesn't re-send the frame the client already has.
    streaming_cameras[camera_id].options = options;
  }

  std::ostringstream detail;
  if (options.subsample > 1) detail << ", subsample " << options.subsample;
  if (options.max_fps > 0.0) detail << ", max " << options.max_fps << " fps";
  LOG_INFO("StreamManager", tag() + "Started streaming camera " +
                                std::to_string(camera_id) + detail.str());

  if (!streaming_active) {
    std::lock_guard<std::mutex> lock(sink_mutex);
    if (sink) {
      should_stop = false;
      streaming_active = true;
      streaming_thread =
          std::make_unique<std::thread>(&StreamManager::streamingLoop, this);
      LOG_INFO("StreamManager", tag() + "Streaming thread started");
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
             tag() + "Stopped streaming camera " + std::to_string(camera_id));

    {
      std::lock_guard<std::mutex> lock(transfer_mutex);
      if (current_transfer && current_transfer->frame &&
          current_transfer->frame->camera_id == camera_id) {
        current_transfer->in_progress = false;
      }
    }
  }

  return removed;
}

void StreamManager::stopAllStreaming() {
  LOG_INFO("StreamManager", tag() + "Stopping all streaming");

  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    streaming_cameras.clear();
  }

  if (streaming_active) {
    should_stop = true;
    frame_available_cv.notify_all();

    if (streaming_thread && streaming_thread->joinable()) {
      LOG_DEBUG("StreamManager", tag() + "Waiting for streaming thread to finish");
      streaming_thread->join();
    }

    streaming_active = false;
  }

  cleanupTransfer();

  LOG_INFO("StreamManager", tag() + "All streaming stopped");
}

bool StreamManager::isStreamingCamera(uint32_t camera_id) const {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  return streaming_cameras.find(camera_id) != streaming_cameras.end();
}

size_t StreamManager::getStreamingCameraCount() const {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  return streaming_cameras.size();
}

std::optional<StreamOptions> StreamManager::getStreamOptions(
    uint32_t camera_id) const {
  std::lock_guard<std::mutex> lock(streaming_mutex);
  auto it = streaming_cameras.find(camera_id);
  if (it == streaming_cameras.end()) return std::nullopt;
  return it->second.options;
}

void StreamManager::notifyBackpressure(bool has_pressure) {
  bool prev = has_backpressure.exchange(has_pressure);
  if (has_pressure && !prev) {
    last_backpressure_us = nowSteadyUs();
    stats.backpressure_pauses++;
    rate_controller.recordBackpressure();
    LOG_WARN("StreamManager",
             tag() + "Backpressure detected - adjusting rate to " +
                 std::to_string(rate_controller.getCurrentRate()) +
                 " chunks/sec");
  } else if (!has_pressure && prev) {
    LOG_INFO("StreamManager",
             tag() + "Backpressure cleared - current rate: " +
                 std::to_string(rate_controller.getCurrentRate()) +
                 " chunks/sec");
    frame_available_cv.notify_one();
  }
}

stream_priority::Snapshot StreamManager::prioritySnapshot() const {
  stream_priority::Snapshot s;
  {
    std::lock_guard<std::mutex> lock(sink_mutex);
    s.has_sink = sink != nullptr;
    s.buffered_bytes = sink ? sink->bufferedAmount() : 0;
  }
  {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    s.streaming_payload =
        !streaming_cameras.empty() && !header_only_mode.load();
  }
  s.in_backpressure = has_backpressure.load();
  const int64_t last_us = last_backpressure_us.load();
  if (last_us > 0) {
    s.since_last_backpressure = std::chrono::microseconds(
        std::max<int64_t>(0, nowSteadyUs() - last_us));
  }
  return s;
}

bool StreamManager::isClearForLowerPriority() const {
  return stream_priority::isClear(prioritySnapshot(), SOFT_BUFFER_LIMIT);
}

void StreamManager::streamingLoop() {
  LOG_INFO("StreamManager", tag() + "Streaming loop started");

  std::vector<uint32_t> camera_order;
  size_t current_index = 0;

  while (!should_stop) {
    if (has_backpressure) {
      LOG_DEBUG("StreamManager", tag() + "Waiting for backpressure to clear");
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
      LOG_DEBUG("StreamManager", tag() + "No sink attached, waiting");
      std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
      frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
      continue;
    }

    bool no_cameras = false;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      no_cameras = streaming_cameras.empty();
      std::vector<uint32_t> keys;
      keys.reserve(streaming_cameras.size());
      for (const auto& entry : streaming_cameras) keys.push_back(entry.first);
      if (keys != camera_order) {
        camera_order = std::move(keys);
        current_index = 0;
        LOG_DEBUG("StreamManager", tag() + "Updated camera order, " +
                                       std::to_string(camera_order.size()) +
                                       " cameras");
      }
    }
    if (no_cameras) {
      LOG_DEBUG("StreamManager", tag() + "No cameras streaming, waiting");
      std::unique_lock<std::mutex> wait_lock(frame_wait_mutex);
      frame_available_cv.wait_for(wait_lock, std::chrono::milliseconds(100));
      continue;
    }

    if (isTransferTimedOut()) {
      LOG_WARN("StreamManager", tag() + "Transfer timed out, cleaning up");
      cleanupTransfer();
    }

    // Don't start a new frame while the WS outgoing buffer is already over
    // the soft cap — otherwise we widen the gap any TEXT control reply
    // (status / error) has to drain past on its way to the client.
    if (!waitForBufferDrain()) continue;

    // Lower priority: withhold new frames while the higher-priority connection
    // is congested. Nothing is fetched, so once it clears the newest frame is
    // the one that goes out (latest-wins) rather than a stale one.
    if (yield_to_ && !yield_to_->isClearForLowerPriority()) {
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(5),
                                  [this] { return should_stop.load(); });
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(transfer_mutex);
      if (current_transfer && current_transfer->in_progress) {
        LOG_DEBUG("StreamManager",
                  tag() + "Transfer in progress, skipping frame check");
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

      CameraStream cs;
      {
        std::lock_guard<std::mutex> lock(streaming_mutex);
        auto it = streaming_cameras.find(camera_id);
        if (it == streaming_cameras.end()) continue;
        cs = it->second;
      }

      // max_fps pacing: when the next send isn't due yet, leave the newest
      // frame where it is. Nothing is fetched now, so the send that eventually
      // happens takes whatever is newest then.
      if (cs.options.max_fps > 0.0) {
        const auto period =
            std::chrono::duration<double>(1.0 / cs.options.max_fps);
        if (std::chrono::steady_clock::now() - cs.last_sent < period) continue;
      }

      Camera* camera = camera_manager->getCamera(camera_id);
      if (!camera) {
        LOG_WARN("StreamManager",
                 tag() + "Camera " + std::to_string(camera_id) + " not found");
        continue;
      }

      // Frame source depends on the process mode. In detector modes the only
      // streamable frames are the ones the detector has already processed, so
      // we pull from the processor's detected-frame slot (which carries the
      // corners); otherwise we pull the latest captured frame from the camera.
      // Either way the frame is shared, not copied, and the session's own
      // cursor decides what counts as new.
      // In STACK mode the only streamable frames are stacked images, each
      // carrying the detection made on it.
      const ProcessMode mode = frame_processor ? frame_processor->getMode() : ProcessMode::NONE;
      const bool detector_gated = isDetectorMode(mode);
      uint8_t detection_kind =
          static_cast<uint8_t>(detectionKindForMode(mode));

      std::shared_ptr<const FrameData> frame;
      std::vector<CornerSet> corner_sets;
      bool got_frame = false;
      if (mode == ProcessMode::STACK) {
        got_frame = stacker_ && stacker_->getStackedFrameForStreaming(
                                    camera_id, cs.stack_cursor, frame,
                                    corner_sets, detection_kind);
      } else if (detector_gated) {
        got_frame = frame_processor->getDetectedFrameForStreaming(
            camera_id, cs.detected_cursor, frame, corner_sets);
      } else {
        got_frame = camera->getFrameForStreaming(cs.camera_cursor, frame);
      }
      if (!got_frame || !frame) continue;

      // Persist the advanced cursor (unless the camera was unsubscribed in
      // the meantime, in which case there is nothing to update).
      {
        std::lock_guard<std::mutex> lock(streaming_mutex);
        auto it = streaming_cameras.find(camera_id);
        if (it != streaming_cameras.end()) {
          it->second.camera_cursor = cs.camera_cursor;
          it->second.detected_cursor = cs.detected_cursor;
          it->second.stack_cursor = cs.stack_cursor;
        }
      }

      LOG_DEBUG("StreamManager",
                tag() + "Got frame " + std::to_string(frame->frame_id) +
                    " from camera " + std::to_string(camera_id));

      // Subsampled streams get their own decimated frame; the header and any
      // detection block then describe that frame's pixel space.
      // A stacked image is float32, which the YUV420 decimation cannot read;
      // it always goes out whole.
      const uint32_t factor =
          frame->frame_kind == FrameKind::STACK ? 1 : cs.options.subsample;
      if (factor > 1) {
        if (header_only_mode) {
          // Geometry only — no point decimating pixels nobody will receive.
          auto meta = std::make_shared<FrameData>();
          meta->frame_id = frame->frame_id;
          meta->camera_id = frame->camera_id;
          meta->pixel_format = frame->pixel_format;
          meta->timestamp_us = frame->timestamp_us;
          meta->frame_duration_us = frame->frame_duration_us;
          meta->lens_position = frame->lens_position;
          meta->af_state = frame->af_state;
          meta->focus_fom = frame->focus_fom;
          const auto g = frame_subsample::subsampledGeometry(
              frame->width, frame->height, factor);
          meta->width = g.width;
          meta->height = g.height;
          meta->bytes_per_line = g.width;
          frame = meta;
        } else {
          frame = std::make_shared<FrameData>(
              frame_subsample::subsampleYuv420(*frame, factor));
        }
        frame_subsample::scaleCornerSets(corner_sets, factor);
      }

      bool success = false;
      if (header_only_mode) {
        success = sendHeaderOnlyFrame(frame, camera_id,
                                      std::move(corner_sets), detection_kind);
        if (success) {
          stats.header_only_frames++;
        }
      } else {
        success = sendChunkedFrame(frame, camera_id, std::move(corner_sets),
                                   detection_kind);
      }

      if (success) {
        found_frame = true;
        stats.frames_sent++;
        std::lock_guard<std::mutex> lock(streaming_mutex);
        auto it = streaming_cameras.find(camera_id);
        if (it != streaming_cameras.end()) {
          it->second.last_sent = std::chrono::steady_clock::now();
        }
      } else {
        stats.frames_dropped++;
        LOG_WARN("StreamManager", tag() + "Failed to send frame from camera " +
                                      std::to_string(camera_id));
      }
    }

    if (!found_frame && !should_stop) {
      LOG_DEBUG("StreamManager", tag() + "No frames available, waiting");
      std::unique_lock<std::mutex> lock(frame_wait_mutex);
      frame_available_cv.wait_for(lock, std::chrono::milliseconds(10),
                                  [this] { return should_stop.load(); });
    }
  }

  LOG_INFO("StreamManager", tag() + "Streaming loop ended");
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

bool StreamManager::waitForPriorityClear(
    std::chrono::steady_clock::time_point deadline) {
  while (yield_to_ && !yield_to_->isClearForLowerPriority()) {
    if (should_stop.load()) return false;
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}

bool StreamManager::sendFrameForTest(const FrameData& frame,
                                     uint32_t camera_id,
                                     std::vector<CornerSet> corner_sets) {
  // Infer the block format from the sets so tests exercise either path without
  // a live save mode: any marker id present ⇒ aruco, else checkerboard (an
  // empty set list carries no block, so NONE is fine).
  DetectionKind kind = DetectionKind::NONE;
  for (const auto& set : corner_sets) {
    if (set.marker_id >= 0) {
      kind = DetectionKind::ARUCO;
      break;
    }
    kind = DetectionKind::CHECKERBOARD;
  }
  const uint8_t detection_kind = static_cast<uint8_t>(kind);
  std::shared_ptr<const FrameData> shared = std::make_shared<FrameData>(frame);
  if (header_only_mode) {
    return sendHeaderOnlyFrame(shared, camera_id, std::move(corner_sets),
                               detection_kind);
  }
  return sendChunkedFrame(shared, camera_id, std::move(corner_sets),
                          detection_kind);
}

bool StreamManager::sendHeaderOnlyFrame(std::shared_ptr<const FrameData> frame,
                                        uint32_t camera_id,
                                        std::vector<CornerSet> corner_sets,
                                        uint8_t detection_kind) {
  ChunkedTransfer transfer;
  transfer.frame = std::move(frame);
  transfer.corner_sets = std::move(corner_sets);
  transfer.detection_kind = detection_kind;
  transfer.frame_uuid = generateFrameUUID();
  transfer.total_chunks = 0;
  transfer.current_chunk = 0;
  transfer.start_time = std::chrono::steady_clock::now();
  transfer.in_progress = false;

  LOG_DEBUG("StreamManager", tag() + "Sending header only for frame " +
                                 std::to_string(transfer.frame->frame_id) +
                                 " from camera " + std::to_string(camera_id));

  return sendChunkHeader(transfer, true);
}

bool StreamManager::sendChunkedFrame(std::shared_ptr<const FrameData> frame,
                                     uint32_t camera_id,
                                     std::vector<CornerSet> corner_sets,
                                     uint8_t detection_kind) {
  const uint32_t frame_id = frame->frame_id;
  {
    std::lock_guard<std::mutex> lock(transfer_mutex);
    current_transfer = std::make_unique<ChunkedTransfer>();
    current_transfer->frame = std::move(frame);
    current_transfer->corner_sets = std::move(corner_sets);
    current_transfer->detection_kind = detection_kind;
    current_transfer->frame_uuid = generateFrameUUID();
    current_transfer->total_chunks =
        (current_transfer->frame->data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    current_transfer->current_chunk = 0;
    current_transfer->start_time = std::chrono::steady_clock::now();
    current_transfer->in_progress = true;
  }

  LOG_DEBUG("StreamManager",
            tag() + "Starting chunked transfer for frame " +
                std::to_string(frame_id) + " from camera " +
                std::to_string(camera_id) + ", " +
                std::to_string(current_transfer->total_chunks) + " chunks");

  if (!sendChunkHeader(*current_transfer, false)) {
    LOG_ERROR("StreamManager", tag() + "Failed to send chunk header");
    cleanupTransfer();
    return false;
  }

  auto last_chunk_time = std::chrono::steady_clock::now();
  const auto priority_deadline =
      current_transfer->start_time + PRIORITY_WAIT_LIMIT;

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
      LOG_INFO("StreamManager", tag() + "Camera " + std::to_string(camera_id) +
                                    " stopped during transfer");
      cleanupTransfer();
      return false;
    }

    int backpressure_wait_count = 0;
    while (has_backpressure && backpressure_wait_count < 50) {
      LOG_DEBUG("StreamManager", tag() + "Backpressure during chunk " +
                                     std::to_string(chunk_idx) + ", waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      backpressure_wait_count++;

      if (should_stop) {
        cleanupTransfer();
        return false;
      }
    }

    if (has_backpressure) {
      LOG_ERROR("StreamManager", tag() + "Backpressure timeout at chunk " +
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

    // Lower priority: pause mid-frame while the higher-priority connection is
    // congested, and abandon the frame if that lasts past the deadline (the
    // client garbage-collects the incomplete frame on its side).
    if (yield_to_ && !waitForPriorityClear(priority_deadline)) {
      if (!should_stop) {
        stats.priority_abandoned++;
        LOG_INFO("StreamManager",
                 tag() + "Abandoned frame " + std::to_string(frame_id) +
                     " at chunk " + std::to_string(chunk_idx) + "/" +
                     std::to_string(current_transfer->total_chunks) +
                     ": yielding to the commander's stream");
      }
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
                tag() + "Failed to send chunk " + std::to_string(chunk_idx) +
                    "/" + std::to_string(current_transfer->total_chunks));
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
          tag() + "Sent chunk " + std::to_string(chunk_idx + 1) + "/" +
              std::to_string(current_transfer->total_chunks) + " at rate: " +
              std::to_string(rate_controller.getCurrentRate()) + " chunks/sec");
    }
  }

  LOG_DEBUG(
      "StreamManager",
      tag() + "Successfully sent frame " + std::to_string(frame_id) + " in " +
          std::to_string(current_transfer->total_chunks) + " chunks at rate: " +
          std::to_string(rate_controller.getCurrentRate()) + " chunks/sec");

  cleanupTransfer();
  return true;
}

bool StreamManager::sendChunkHeader(const ChunkedTransfer& transfer,
                                    bool header_only) {
  std::lock_guard<std::mutex> lock(sink_mutex);
  if (!sink) return false;
  if (!transfer.frame) return false;
  const FrameData& frame = *transfer.frame;

  // Corners were resolved when the frame was pulled (detector modes hand the
  // streamer only detector-processed frames, each carrying its own corner
  // set), so no lookup is needed here.
  const std::vector<CornerSet>& corner_sets = transfer.corner_sets;

  // The per-record header differs by detection kind: CornerSetHeader (4 bytes)
  // for checkerboard, MarkerSetHeader (8 bytes, carries the marker id) for
  // aruco. Compute the variable block size up front so it can be encoded in
  // ChunkHeader before the bytes go out.
  const bool is_aruco =
      transfer.detection_kind == static_cast<uint8_t>(DetectionKind::ARUCO);
  const size_t record_header_size =
      is_aruco ? sizeof(MarkerSetHeader) : sizeof(CornerSetHeader);
  uint32_t corner_block_size = 0;
  for (const auto& set : corner_sets) {
    corner_block_size += static_cast<uint32_t>(record_header_size);
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
  header.frame_id = frame.frame_id;
  header.camera_id = frame.camera_id;

  if (header_only) {
    header.total_chunks = 0;
    header.total_size = 0;
  } else {
    header.total_chunks = transfer.total_chunks;
    header.total_size = frame.data.size();
  }

  header.bytes_per_line = frame.bytes_per_line;
  header.width = frame.width;
  header.height = frame.height;
  header.pixel_format = frame.pixel_format;
  header.frames_saved =
      frame_processor
          ? frame_processor->getFramesSavedForCamera(frame.camera_id)
          : 0;
  header.timestamp_us = frame.timestamp_us;
  header.frame_duration_us = frame.frame_duration_us;
  header.corner_block_size = corner_block_size;
  header.num_corner_sets = static_cast<uint16_t>(corner_sets.size());
  header.reserved = 0;
  header.lens_position = frame.lens_position;
  header.af_state = frame.af_state;
  header.detection_kind = transfer.detection_kind;
  header.frame_kind = static_cast<uint8_t>(frame.frame_kind);
  header.reserved2 = 0;
  header.focus_fom = frame.focus_fom;

  const uint8_t* header_bytes = reinterpret_cast<const uint8_t*>(&header);
  message.insert(message.end(), header_bytes,
                 header_bytes + sizeof(ChunkHeader));

  // Append the variable-size detection block: one record per set, packed
  // back-to-back. The record header is a MarkerSetHeader (with the marker id)
  // for aruco, or a CornerSetHeader for checkerboard; the corner xy floats that
  // follow are identical in both. Empty corner_sets ⇒ no bytes appended.
  for (const auto& set : corner_sets) {
    if (is_aruco) {
      MarkerSetHeader mh{};
      mh.marker_id = set.marker_id;
      mh.quadrant = set.set_id;
      mh.flags = 0x01;  // bit 0: coordinates are in the streamed frame's Y-plane pixels
      mh.num_corners = static_cast<uint16_t>(set.corners.size());
      const uint8_t* mh_bytes = reinterpret_cast<const uint8_t*>(&mh);
      message.insert(message.end(), mh_bytes,
                     mh_bytes + sizeof(MarkerSetHeader));
    } else {
      CornerSetHeader sh{};
      sh.set_id = set.set_id;
      sh.flags = 0x01;  // bit 0: coordinates are in the streamed frame's Y-plane pixels
      sh.num_corners = static_cast<uint16_t>(set.corners.size());
      const uint8_t* sh_bytes = reinterpret_cast<const uint8_t*>(&sh);
      message.insert(message.end(), sh_bytes,
                     sh_bytes + sizeof(CornerSetHeader));
    }
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
  if (!transfer.frame) return false;
  const std::vector<uint8_t>& data = transfer.frame->data;

  size_t offset = chunk_index * CHUNK_SIZE;
  size_t chunk_size = std::min(CHUNK_SIZE, data.size() - offset);

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

  message.insert(message.end(), data.begin() + offset,
                 data.begin() + offset + chunk_size);

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
    LOG_DEBUG("StreamManager", tag() + "Cleaned up transfer");
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
  static std::mutex gen_mutex;  // two sessions share the generator
  std::lock_guard<std::mutex> lock(gen_mutex);
  return dis(gen);
}

void StreamManager::logStats() const {
  std::stringstream ss;
  ss << tag() << "StreamManager Statistics:\n"
     << "  Frames sent: " << stats.frames_sent.load() << "\n"
     << "  Header only frames: " << stats.header_only_frames.load() << "\n"
     << "  Frames dropped: " << stats.frames_dropped.load() << "\n"
     << "  Chunks sent: " << stats.chunks_sent.load() << "\n"
     << "  Bytes sent: " << stats.bytes_sent.load() << "\n"
     << "  Backpressure pauses: " << stats.backpressure_pauses.load() << "\n"
     << "  Frames abandoned for priority: " << stats.priority_abandoned.load()
     << "\n"
     << "  Final chunk rate: " << rate_controller.getCurrentRate()
     << " chunks/sec\n"
     << "  Mode: " << (header_only_mode ? "Header Only" : "Full Frame");
  LOG_INFO("StreamManager", ss.str());
}
