#include "camera.hpp"

#include <sys/mman.h>
#include <algorithm>
#include <cstring>

Camera::Camera(uint32_t id, std::shared_ptr<libcamera::Camera> cam,
               const CameraConfig& cfg)
    : camera_id(id), lcam(std::move(cam)), config(cfg) {}

Camera::~Camera() { stop(); }

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

  cam_config = lcam->generateConfiguration({libcamera::StreamRole::Raw});
  if (!cam_config) {
    LOG_ERROR("Camera", "Failed to generate configuration for camera " +
                            std::to_string(camera_id));
    lcam->release();
    return false;
  }

  libcamera::StreamConfiguration& stream_cfg = cam_config->at(0);
  stream_cfg.pixelFormat = libcamera::formats::RGGB_PISP_COMP1;
  stream_cfg.size = libcamera::Size(config.width, config.height);
  stream_cfg.bufferCount = static_cast<unsigned int>(buffer_count);

  libcamera::CameraConfiguration::Status status = cam_config->validate();
  if (status == libcamera::CameraConfiguration::Invalid) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) +
                            " configuration is invalid");
    lcam->release();
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
    lcam->release();
    return false;
  }

  allocator = std::make_unique<libcamera::FrameBufferAllocator>(lcam);
  libcamera::Stream* stream = stream_cfg.stream();
  if (allocator->allocate(stream) < 0) {
    LOG_ERROR("Camera", "Failed to allocate buffers for camera " +
                            std::to_string(camera_id));
    lcam->release();
    return false;
  }

  const auto& buffers = allocator->buffers(stream);
  mapped_planes.resize(buffers.size());
  requests.reserve(buffers.size());

  for (size_t i = 0; i < buffers.size(); ++i) {
    libcamera::FrameBuffer* buf = buffers[i].get();

    mapped_planes[i].resize(buf->planes().size());
    for (size_t p = 0; p < buf->planes().size(); ++p) {
      const libcamera::FrameBuffer::Plane& plane = buf->planes()[p];
      size_t len = plane.length;
      void* ptr = ::mmap(nullptr, len, PROT_READ, MAP_SHARED,
                         plane.fd.get(), plane.offset);
      if (ptr == MAP_FAILED) {
        LOG_ERROR("Camera",
                  "mmap failed for camera " + std::to_string(camera_id));
        lcam->release();
        return false;
      }
      mapped_planes[i][p] = {ptr, len};
    }

    auto req = lcam->createRequest(static_cast<uint64_t>(i));
    if (!req) {
      LOG_ERROR("Camera", "Failed to create request for camera " +
                              std::to_string(camera_id));
      lcam->release();
      return false;
    }
    if (req->addBuffer(stream, buf) < 0) {
      LOG_ERROR("Camera", "Failed to add buffer to request for camera " +
                              std::to_string(camera_id));
      lcam->release();
      return false;
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

  should_stop = false;
  if (lcam->start(&controls) < 0) {
    LOG_ERROR("Camera",
              "Failed to start camera " + std::to_string(camera_id));
    lcam->requestCompleted.disconnect(this);
    return false;
  }

  for (auto& req : requests) {
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

  libcamera::Stream* stream = cam_config->at(0).stream();
  for (auto& planes : mapped_planes) {
    for (auto& mp : planes) {
      if (mp.data != MAP_FAILED) {
        ::munmap(mp.data, mp.size);
        mp.data = MAP_FAILED;
      }
    }
  }
  mapped_planes.clear();
  requests.clear();
  allocator->free(stream);
  allocator.reset();
  lcam->release();

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera", "Camera " + std::to_string(camera_id) + " stopped");
  return true;
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
    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter++;
    frame.width = stream_cfg.size.width;
    frame.height = stream_cfg.size.height;
    frame.bytes_per_line = stream_cfg.stride;

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

    const libcamera::ControlList& meta = request->metadata();
    if (auto gains = meta.get(libcamera::controls::ColourGains)) {
      frame.awb_gain_r = (*gains)[0];
      frame.awb_gain_b = (*gains)[1];
    }
    if (auto cct = meta.get(libcamera::controls::ColourTemperature)) {
      frame.awb_cct = static_cast<float>(*cct);
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
    lcam->queueRequest(request);
  }
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
