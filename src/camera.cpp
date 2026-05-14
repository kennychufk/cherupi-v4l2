#include "camera.hpp"

#include <sys/mman.h>
#include <algorithm>
#include <cstring>
#include <map>

Camera::Camera(uint32_t id, std::shared_ptr<libcamera::Camera> cam,
               const CameraConfig& cfg)
    : camera_id(id), lcam(std::move(cam)), config(cfg) {}

Camera::~Camera() {
  stop();
  unconfigure();
}

bool Camera::configure(size_t buffer_count) {
  if (state != CameraState::IDLE) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) + " is not idle");
    return false;
  }

  if (lcam->acquire() < 0) {
    LOG_ERROR("Camera",
              "Failed to acquire camera " + std::to_string(camera_id));
    return false;
  }
  lcam_acquired_ = true;

  cam_config = lcam->generateConfiguration(
      {libcamera::StreamRole::VideoRecording, libcamera::StreamRole::Raw});
  if (!cam_config) {
    LOG_ERROR("Camera", "Failed to generate configuration for camera " +
                            std::to_string(camera_id));
    releaseConfiguredResources();
    return false;
  }

  libcamera::StreamConfiguration& stream_cfg = cam_config->at(0);
  stream_cfg.pixelFormat = libcamera::formats::YUV420;
  stream_cfg.size = libcamera::Size(config.width, config.height);
  stream_cfg.bufferCount = static_cast<unsigned int>(buffer_count);

  // Raw stream + sensorConfig together force the PiSP backend to select the
  // native sensor mode for this resolution. pixelFormat Bayer order is ignored
  // by the pipeline handler (it substitutes the correct order); bit depth and
  // packing are what matter for mode selection.
  if (cam_config->size() > 1) {
    libcamera::StreamConfiguration& raw_cfg = cam_config->at(1);
    raw_cfg.size = libcamera::Size(config.width, config.height);
    raw_cfg.pixelFormat = libcamera::formats::SBGGR10_CSI2P;
    raw_cfg.bufferCount = static_cast<unsigned int>(buffer_count);

    cam_config->sensorConfig = libcamera::SensorConfiguration();
    cam_config->sensorConfig->outputSize = libcamera::Size(config.width, config.height);
    cam_config->sensorConfig->bitDepth = 10;
  }

  libcamera::CameraConfiguration::Status status = cam_config->validate();
  if (status == libcamera::CameraConfiguration::Invalid) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) +
                            " configuration is invalid");
    releaseConfiguredResources();
    return false;
  }
  if (status == libcamera::CameraConfiguration::Adjusted) {
    LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                           " configuration was adjusted: format=" +
                           stream_cfg.pixelFormat.toString() + " size=" +
                           std::to_string(stream_cfg.size.width) + "x" +
                           std::to_string(stream_cfg.size.height));
    config.width = stream_cfg.size.width;
    config.height = stream_cfg.size.height;
  }

  if (lcam->configure(cam_config.get()) < 0) {
    LOG_ERROR("Camera",
              "Failed to configure camera " + std::to_string(camera_id));
    releaseConfiguredResources();
    return false;
  }

  allocator = std::make_unique<libcamera::FrameBufferAllocator>(lcam);
  libcamera::Stream* stream = stream_cfg.stream();
  if (allocator->allocate(stream) < 0) {
    LOG_ERROR("Camera", "Failed to allocate buffers for camera " +
                            std::to_string(camera_id));
    releaseConfiguredResources();
    return false;
  }

  raw_stream_ = (cam_config->size() > 1) ? cam_config->at(1).stream() : nullptr;
  if (raw_stream_ && allocator->allocate(raw_stream_) < 0) {
    LOG_ERROR("Camera", "Failed to allocate raw buffers for camera " +
                            std::to_string(camera_id));
    releaseConfiguredResources();
    return false;
  }

  const auto& buffers = allocator->buffers(stream);
  mapped_planes.resize(buffers.size());
  mmap_regions.resize(buffers.size());
  requests.reserve(buffers.size());

  for (size_t i = 0; i < buffers.size(); ++i) {
    libcamera::FrameBuffer* buf = buffers[i].get();
    mapped_planes[i].resize(buf->planes().size());

    // Determine total mapped size required for each unique fd.
    // mmap requires page-aligned offsets, so we map each fd from offset 0
    // with enough length to cover all planes that share it, then derive
    // per-plane pointers as base + plane.offset.
    std::map<int, size_t> fd_total_size;
    for (const auto& plane : buf->planes()) {
      int fd = plane.fd.get();
      size_t needed = static_cast<size_t>(plane.offset) + plane.length;
      fd_total_size[fd] = std::max(fd_total_size[fd], needed);
    }

    std::map<int, void*> fd_base;
    for (auto& [fd, total] : fd_total_size) {
      void* base = ::mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);
      if (base == MAP_FAILED) {
        LOG_ERROR("Camera",
                  "mmap failed for camera " + std::to_string(camera_id));
        // Unmap any already-mapped fds for this buffer.
        for (auto& [mapped_fd, ptr] : fd_base)
          ::munmap(ptr, fd_total_size[mapped_fd]);
        releaseConfiguredResources();
        return false;
      }
      fd_base[fd] = base;
      mmap_regions[i].push_back({base, total});
    }

    for (size_t p = 0; p < buf->planes().size(); ++p) {
      const libcamera::FrameBuffer::Plane& plane = buf->planes()[p];
      uint8_t* ptr = static_cast<uint8_t*>(fd_base[plane.fd.get()]) + plane.offset;
      mapped_planes[i][p] = {ptr, plane.length};
    }

    auto req = lcam->createRequest(static_cast<uint64_t>(i));
    if (!req) {
      LOG_ERROR("Camera", "Failed to create request for camera " +
                              std::to_string(camera_id));
      releaseConfiguredResources();
      return false;
    }
    if (req->addBuffer(stream, buf) < 0) {
      LOG_ERROR("Camera", "Failed to add buffer to request for camera " +
                              std::to_string(camera_id));
      releaseConfiguredResources();
      return false;
    }
    if (raw_stream_) {
      const auto& raw_bufs = allocator->buffers(raw_stream_);
      if (i < raw_bufs.size() && req->addBuffer(raw_stream_, raw_bufs[i].get()) < 0) {
        LOG_ERROR("Camera", "Failed to add raw buffer to request for camera " +
                                std::to_string(camera_id));
        releaseConfiguredResources();
        return false;
      }
    }
    requests.push_back(std::move(req));
  }

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera",
           "Camera " + std::to_string(camera_id) + " configured: format=" +
               stream_cfg.pixelFormat.toString() + " " +
               std::to_string(stream_cfg.size.width) + "x" +
               std::to_string(stream_cfg.size.height) + " stride=" +
               std::to_string(stream_cfg.stride));

  // Log the sensor's hardware FrameDurationLimits so callers can see what
  // FPS range libcamera advertises for this camera in isolation.
  auto [hw_min, hw_max] = getFrameDurationLimitsHw();
  if (hw_min > 0 || hw_max > 0) {
    double max_fps = hw_min > 0 ? (1e6 / hw_min) : 0.0;
    double min_fps = hw_max > 0 ? (1e6 / hw_max) : 0.0;
    LOG_INFO("Camera",
             "Camera " + std::to_string(camera_id) +
                 " HW FrameDurationLimits: min=" + std::to_string(hw_min) +
                 " µs (" + std::to_string(max_fps).substr(0, 5) +
                 " fps max), max=" + std::to_string(hw_max) + " µs (" +
                 std::to_string(min_fps).substr(0, 4) + " fps min)");
    LOG_INFO("Camera",
             "NOTE: HW limits reflect the underlying IMX519 sensor spec. "
             "Multi-sensor boards (e.g. Arducam quad-kit) consolidate "
             "physical sensors into one logical camera and may achieve "
             "lower fps than advertised due to internal driver overhead.");
  }
  return true;
}

bool Camera::start() {
  if (state != CameraState::CONFIGURED) {
    LOG_ERROR("Camera",
              "Camera " + std::to_string(camera_id) + " is not configured");
    return false;
  }

  lcam->requestCompleted.connect(this, &Camera::onRequestComplete);

  libcamera::ControlList controls(lcam->controls());
  controls.set(libcamera::controls::AwbMode,
               static_cast<int32_t>(libcamera::controls::AwbAuto));
  if (af_continuous_.load()) {
    controls.set(libcamera::controls::AfMode,
                 static_cast<int32_t>(libcamera::controls::AfModeContinuous));
  } else {
    controls.set(libcamera::controls::AfMode,
                 static_cast<int32_t>(libcamera::controls::AfModeManual));
    controls.set(libcamera::controls::LensPosition, lens_position_.load());
  }
  applied_focus_generation_ = focus_generation_.load();

  if (ae_auto_.load()) {
    controls.set(libcamera::controls::ExposureTimeMode,
                 static_cast<int32_t>(libcamera::controls::ExposureTimeModeAuto));
  } else {
    controls.set(libcamera::controls::ExposureTimeMode,
                 static_cast<int32_t>(libcamera::controls::ExposureTimeModeManual));
    controls.set(libcamera::controls::ExposureTime, exposure_time_us_.load());
  }
  applied_exposure_generation_ = exposure_generation_.load();

  int64_t fd_initial = frame_duration_us_.load();
  if (fd_initial > 0) {
    int64_t v[2] = {fd_initial, fd_initial};
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(v));
    LOG_INFO("Camera",
             "Camera " + std::to_string(camera_id) +
                 " start: applying FrameDurationLimits lock @ " +
                 std::to_string(fd_initial) + " µs (" +
                 std::to_string(1e6 / fd_initial).substr(0, 5) + " fps)");
  } else {
    LOG_INFO("Camera",
             "Camera " + std::to_string(camera_id) +
                 " start: no FrameDurationLimits applied (libcamera default)");
  }
  applied_frame_duration_generation_ = frame_duration_generation_.load();

  should_stop = false;
  if (lcam->start(&controls) < 0) {
    LOG_ERROR("Camera",
              "Failed to start camera " + std::to_string(camera_id));
    lcam->requestCompleted.disconnect(this);
    return false;
  }

  for (auto& req : requests) {
    req->reuse(libcamera::Request::ReuseBuffers);
    if (lcam->queueRequest(req.get()) < 0) {
      LOG_ERROR("Camera", "Failed to queue initial request for camera " +
                              std::to_string(camera_id));
      lcam->stop();
      lcam->requestCompleted.disconnect(this);
      return false;
    }
  }

  state = CameraState::RUNNING;
  LOG_INFO("Camera",
           "Camera " + std::to_string(camera_id) + " started successfully");
  return true;
}

bool Camera::stop() {
  if (state != CameraState::RUNNING) {
    return true;
  }

  LOG_INFO("Camera", "Stopping camera " + std::to_string(camera_id));
  should_stop = true;
  new_frame_cv.notify_all();

  lcam->stop();
  lcam->requestCompleted.disconnect(this);

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera", "Camera " + std::to_string(camera_id) + " stopped");
  return true;
}

bool Camera::unconfigure() {
  if (state == CameraState::RUNNING) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) +
                            ": unconfigure rejected while RUNNING");
    return false;
  }
  if (state == CameraState::IDLE) {
    return true;  // already there
  }
  LOG_INFO("Camera",
           "Unconfiguring camera " + std::to_string(camera_id));
  releaseConfiguredResources();
  state = CameraState::IDLE;
  return true;
}

void Camera::releaseConfiguredResources() {
  for (auto& regions : mmap_regions) {
    for (auto& r : regions) {
      if (r.base != MAP_FAILED) {
        ::munmap(r.base, r.size);
        r.base = MAP_FAILED;
      }
    }
  }
  mmap_regions.clear();
  mapped_planes.clear();
  requests.clear();
  if (allocator && cam_config) {
    allocator->free(cam_config->at(0).stream());
    if (raw_stream_) allocator->free(raw_stream_);
  }
  raw_stream_ = nullptr;
  allocator.reset();
  cam_config.reset();
  if (lcam_acquired_) {
    lcam->release();
    lcam_acquired_ = false;
  }
}

void Camera::onRequestComplete(libcamera::Request* request) {
  if (should_stop) return;
  if (request->status() == libcamera::Request::RequestCancelled) return;

  uint64_t slot = request->cookie();
  libcamera::Stream* stream = cam_config->at(0).stream();
  const libcamera::StreamConfiguration& stream_cfg = cam_config->at(0);

  libcamera::FrameBuffer* buf = request->findBuffer(stream);
  if (!buf) {
    LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                           " requestCompleted: no buffer found");
    goto requeue;
  }

  {
    // Extract hardware timestamp and actual frame duration from libcamera
    // metadata. These reflect the real ISP-level cadence rather than the
    // sensor's theoretical limits, so they expose any multi-camera ISP
    // bandwidth sharing overhead.
    uint64_t hw_ts_ns = buf->metadata().timestamp;  // nanoseconds
    uint64_t hw_ts_us = hw_ts_ns / 1000;            // microseconds

    int64_t actual_fd_us = 0;
    auto fd_meta = request->metadata().get(libcamera::controls::FrameDuration);
    if (fd_meta) actual_fd_us = *fd_meta;

    // Periodic FPS diagnostics at INFO level (every 30 frames per camera).
    if (last_frame_ts_ns_ > 0) {
      uint64_t interval_ns = hw_ts_ns - last_frame_ts_ns_;
      ++fps_log_counter_;
      if (fps_log_counter_ >= 30) {
        double actual_fps = 1e9 / static_cast<double>(interval_ns);
        LOG_INFO("Camera",
                 "Camera " + std::to_string(camera_id) +
                     " frame#" + std::to_string(frame_counter) +
                     " hw_interval=" + std::to_string(interval_ns / 1000) +
                     " µs  actual_fps=" +
                     std::to_string(actual_fps).substr(0, 5) +
                     "  IPA_frame_duration=" + std::to_string(actual_fd_us) +
                     " µs");
        fps_log_counter_ = 0;
      }
    }
    last_frame_ts_ns_ = hw_ts_ns;

    LOG_DEBUG("Camera",
              "Camera " + std::to_string(camera_id) +
                  " frame#" + std::to_string(frame_counter) +
                  " ts=" + std::to_string(hw_ts_us) +
                  " µs  IPA_fd=" + std::to_string(actual_fd_us) + " µs");

    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter++;
    frame.width = stream_cfg.size.width;
    frame.height = stream_cfg.size.height;
    frame.bytes_per_line = stream_cfg.stride;
    frame.pixel_format = stream_cfg.pixelFormat.fourcc();
    frame.timestamp_us = hw_ts_us;
    frame.frame_duration_us = static_cast<uint32_t>(actual_fd_us > 0 ? actual_fd_us : 0);

    size_t total = 0;
    for (const auto& plane : buf->planes()) total += plane.length;
    frame.data.resize(total);

    size_t offset = 0;
    for (size_t p = 0; p < buf->planes().size(); ++p) {
      size_t len = buf->planes()[p].length;
      if (slot < mapped_planes.size() && p < mapped_planes[slot].size() &&
          mapped_planes[slot][p].data != MAP_FAILED) {
        std::memcpy(frame.data.data() + offset,
                    mapped_planes[slot][p].data, len);
      }
      offset += len;
    }

    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex);
      latest_frame = frame;
      has_new_frame = true;
    }
    new_frame_cv.notify_one();
    if (onFrameAvailable) onFrameAvailable();
    if (onFrameCaptured) onFrameCaptured(frame);
  }

requeue:
  if (!should_stop) {
    request->reuse(libcamera::Request::ReuseBuffers);
    uint64_t gen = focus_generation_.load();
    if (gen != applied_focus_generation_) {
      libcamera::ControlList& reqctrls = request->controls();
      if (af_continuous_.load()) {
        reqctrls.set(
            libcamera::controls::AfMode,
            static_cast<int32_t>(libcamera::controls::AfModeContinuous));
      } else {
        reqctrls.set(libcamera::controls::AfMode,
                     static_cast<int32_t>(libcamera::controls::AfModeManual));
        reqctrls.set(libcamera::controls::LensPosition, lens_position_.load());
      }
      applied_focus_generation_ = gen;
    }
    uint64_t exp_gen = exposure_generation_.load();
    if (exp_gen != applied_exposure_generation_) {
      libcamera::ControlList& reqctrls = request->controls();
      if (ae_auto_.load()) {
        reqctrls.set(libcamera::controls::ExposureTimeMode,
                     static_cast<int32_t>(libcamera::controls::ExposureTimeModeAuto));
      } else {
        reqctrls.set(libcamera::controls::ExposureTimeMode,
                     static_cast<int32_t>(libcamera::controls::ExposureTimeModeManual));
        reqctrls.set(libcamera::controls::ExposureTime, exposure_time_us_.load());
      }
      applied_exposure_generation_ = exp_gen;
    }
    uint64_t fd_gen = frame_duration_generation_.load();
    if (fd_gen != applied_frame_duration_generation_) {
      libcamera::ControlList& reqctrls = request->controls();
      int64_t fd = frame_duration_us_.load();
      if (fd > 0) {
        int64_t v[2] = {fd, fd};
        reqctrls.set(libcamera::controls::FrameDurationLimits,
                     libcamera::Span<const int64_t, 2>(v));
      } else {
        // Runtime "unset": release the lock by applying the HW max range
        // (omitting the control would just leave the previous lock in place).
        auto [hw_min, hw_max] = getFrameDurationLimitsHw();
        if (hw_max > 0) {
          int64_t v[2] = {hw_min, hw_max};
          reqctrls.set(libcamera::controls::FrameDurationLimits,
                       libcamera::Span<const int64_t, 2>(v));
        }
      }
      applied_frame_duration_generation_ = fd_gen;
    }
    lcam->queueRequest(request);
  }
}

void Camera::setLensPosition(float lens_position) {
  if (lens_position < 0.0f) {
    af_continuous_.store(true);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setLensPosition: continuous AF");
  } else {
    lens_position_.store(lens_position);
    af_continuous_.store(false);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setLensPosition: manual @ " +
                           std::to_string(lens_position) + " dioptres");
  }
  focus_generation_.fetch_add(1);
}

void Camera::setExposureTime(int32_t exposure_time_us) {
  if (exposure_time_us < 0) {
    ae_auto_.store(true);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setExposureTime: auto AE");
  } else {
    exposure_time_us_.store(exposure_time_us);
    ae_auto_.store(false);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setExposureTime: manual @ " +
                           std::to_string(exposure_time_us) + " \xc2\xb5s");
  }
  exposure_generation_.fetch_add(1);
}

void Camera::setFrameDuration(int64_t frame_duration_us) {
  if (frame_duration_us <= 0) {
    frame_duration_us_.store(0);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setFrameDuration: unset");
  } else {
    frame_duration_us_.store(frame_duration_us);
    LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                           " setFrameDuration: locked @ " +
                           std::to_string(frame_duration_us) + " \xc2\xb5s");
  }
  frame_duration_generation_.fetch_add(1);
}

std::pair<int64_t, int64_t> Camera::getFrameDurationLimitsHw() const {
  if (!lcam_acquired_) return {0, 0};
  const auto& cmap = lcam->controls();
  auto it = cmap.find(&libcamera::controls::FrameDurationLimits);
  if (it == cmap.end()) return {0, 0};
  // The Raspberry Pi pipeline handler registers FrameDurationLimits with
  // scalar int64_t min/max bounds (despite the control type being
  // Span<int64_t, 2>).
  const libcamera::ControlInfo& info = it->second;
  return {info.min().get<int64_t>(), info.max().get<int64_t>()};
}

bool Camera::getFrameForStreaming(FrameData& frame) {
  {
    std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
    if (streaming_frame_in_use) return false;
  }

  std::lock_guard<std::mutex> latest_lock(latest_frame_mutex);
  if (!has_new_frame) return false;

  {
    std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
    streaming_frame = latest_frame;
    streaming_frame_in_use = true;
    frame = streaming_frame;
  }
  has_new_frame = false;
  return true;
}

bool Camera::waitForNewFrame(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(latest_frame_mutex);
  return new_frame_cv.wait_for(
      lock, timeout,
      [this] { return has_new_frame || should_stop.load(); });
}

void Camera::releaseStreamingFrame() {
  std::lock_guard<std::mutex> lock(streaming_frame_mutex);
  streaming_frame_in_use = false;
  LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                          " streaming frame released");
}

void Camera::setFrameAvailableCallback(std::function<void()> callback) {
  onFrameAvailable = callback;
}
