#include "camera.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

#include "frontend/pisp_statistics.h"

namespace {

// IMX519 lux estimation calibration (from imx519.json rpi.lux block).
constexpr double kLuxRefExposureUs = 13841.0;
constexpr double kLuxRefGain       = 2.0;
constexpr double kLuxRefY          = 12064.0;  // 16-bit scale
constexpr double kLuxRefLux        = 900.0;

// Reads a single V4L2 integer control; returns -1 on error.
static int32_t readV4l2Ctrl(int fd, uint32_t cid) {
  struct v4l2_control ctrl{};
  ctrl.id = cid;
  if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) < 0) return -1;
  return ctrl.value;
}

}  // namespace

Camera::Camera(uint32_t id, MediaDevice* media_dev, const CameraConfig& cfg)
    : camera_id(id), media_device(media_dev), config(cfg) {
  awb.setConfig(config.awb);
  awb_bayes.setConfig(config.awb);
}

Camera::~Camera() { stop(); }

bool Camera::configure(size_t buffer_count) {
  if (state != CameraState::IDLE) {
    LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) + " is not idle");
    return false;
  }

  // Reset all links
  if (!media_device->reset()) {
    LOG_ERROR("Camera", "Failed to reset media device for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Set sensor crop (optional - IMX519 does not support S_SELECTION,
  // it selects mode based on format resolution instead)
  if (!media_device->setCrop(config.sensor_entity, 0, config.crop_left,
                             config.crop_top, config.crop_width,
                             config.crop_height)) {
    LOG_WARN("Camera", "Sensor crop not supported for camera " +
                           std::to_string(camera_id) +
                           ", skipping (sensor selects mode via format)");
  }

  // Configure formats (use output width/height - sensor handles binning)
  if (!media_device->setFormat(config.sensor_entity, 0, config.width,
                               config.height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set sensor format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 0, config.width,
                               config.height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set CSI2 pad0 format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setFormat(config.csi2_entity, 4, config.width,
                               config.height,
                               MEDIA_BUS_FMT_SRGGB10_1X10)) {
    LOG_ERROR("Camera", "Failed to set CSI2 pad4 format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Enable links
  if (!media_device->setLink(config.sensor_entity, 0, config.csi2_entity, 0,
                             true)) {
    LOG_ERROR("Camera", "Failed to enable sensor to CSI2 link for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!media_device->setLink(config.csi2_entity, 4, config.video_entity, 0,
                             true)) {
    LOG_ERROR("Camera", "Failed to enable CSI2 to video link for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Get video device path
  std::string video_path =
      media_device->getVideoDevicePath(config.video_entity);
  if (video_path.empty()) {
    LOG_ERROR("Camera", "Failed to get video device path for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Open video device
  video_device = std::make_unique<V4L2Device>();
  if (!video_device->open(video_path)) {
    LOG_ERROR("Camera", "Failed to open video device for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!video_device->setFormat(config.width, config.height,
                               V4L2_PIX_FMT_SRGGB10P)) {
    LOG_ERROR("Camera", "Failed to set video format for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (!video_device->setupBuffers(buffer_count)) {
    LOG_ERROR("Camera", "Failed to setup buffers for camera " +
                            std::to_string(camera_id));
    return false;
  }

  // Bayes pipeline is opt-in. If requested but setup fails we keep the
  // single-device grey-world path we just built above.
  if (config.awb.mode == AwbMode::BAYES) {
    if (configureBayesPipeline(buffer_count)) {
      bayes_pipeline_active = true;
      LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                             " Bayes FE pipeline active");
    } else {
      LOG_WARN("Camera",
               "Camera " + std::to_string(camera_id) +
                   " Bayes FE setup failed, falling back to grey-world");
      stats_device.reset();
      fe_config_device.reset();
      fe_configurator.reset();
    }
  }

  // Compute sensor line time for lux estimation. Read pixel_rate and
  // horizontal_blanking from the sensor subdevice (both are read-only controls
  // that are fixed per mode).
  sensor_subdev_path_ = media_device->getSubdevPath(config.sensor_entity);
  if (!sensor_subdev_path_.empty()) {
    int sfd = ::open(sensor_subdev_path_.c_str(), O_RDONLY);
    if (sfd >= 0) {
      int32_t hblank = readV4l2Ctrl(sfd, V4L2_CID_HBLANK);
      int64_t prate = 0;
      struct v4l2_ext_controls ecs{};
      struct v4l2_ext_control ec{};
      ec.id = V4L2_CID_PIXEL_RATE;
      ecs.controls = &ec;
      ecs.count = 1;
      ioctl(sfd, VIDIOC_G_EXT_CTRLS, &ecs);
      prate = ec.value64;
      ::close(sfd);
      if (hblank > 0 && prate > 0) {
        sensor_line_time_us_ =
            static_cast<double>(config.width + hblank) / prate * 1e6;
      }
    }
  }
  if (sensor_line_time_us_ <= 0.0) {
    // IMX519 at 2328x1748: (2328+4184)/426666667*1e6 = 15.264 µs — safe default.
    sensor_line_time_us_ = 15.264;
    LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                           " could not read line time, using default " +
                           std::to_string(sensor_line_time_us_) + " µs");
  }

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera",
           "Camera " + std::to_string(camera_id) + " configured successfully");
  return true;
}

bool Camera::configureBayesPipeline(size_t buffer_count) {
  // Swap the media graph from csi2→csi2_ch0 (grey-world bypass) to the full
  // PiSP FE chain. The earlier configure() step set up the bypass; undo
  // that first link, then wire the FE.
  const std::string fe_entity = "pisp-fe";
  const std::string fe_image_entity = "rp1-cfe-fe_image0";
  const std::string fe_stats_entity = "rp1-cfe-fe_stats";
  const std::string fe_cfg_entity = "rp1-cfe-fe_config";

  media_device->setLink(config.csi2_entity, 4, config.video_entity, 0, false);
  if (!media_device->setLink(config.csi2_entity, 4, fe_entity, 0, true) ||
      !media_device->setLink(fe_entity, 2, fe_image_entity, 0, true) ||
      !media_device->setLink(fe_entity, 4, fe_stats_entity, 0, true) ||
      !media_device->setLink(fe_cfg_entity, 0, fe_entity, 1, true)) {
    LOG_ERROR("Camera", "Failed to enable pisp-fe links");
    return false;
  }

  // csi2 and pisp-fe both expect SRGGB16 across the FE. csi2 pad 4 was set
  // to SRGGB10 by the grey-world path — override here.
  media_device->setFormat(config.csi2_entity, 4, config.width, config.height,
                          MEDIA_BUS_FMT_SRGGB16_1X16);
  media_device->setFormat(fe_entity, 0, config.width, config.height,
                          MEDIA_BUS_FMT_SRGGB16_1X16);
  media_device->setFormat(fe_entity, 2, config.width, config.height,
                          MEDIA_BUS_FMT_SRGGB16_1X16);

  std::string image_path = media_device->getVideoNodeForPad(fe_entity, 2);
  std::string stats_path = media_device->getVideoNodeForPad(fe_entity, 4);
  std::string cfg_path = media_device->getVideoNodeForPad(fe_entity, 1);
  LOG_INFO("Camera", "pisp-fe nodes image='" + image_path + "' stats='" +
                         stats_path + "' cfg='" + cfg_path + "'");
  if (image_path.empty() || stats_path.empty() || cfg_path.empty()) {
    LOG_ERROR("Camera", "Could not resolve pisp-fe video nodes");
    return false;
  }

  // Re-open the image device against /dev/videoN on pisp-fe:2. The earlier
  // video_device (csi2_ch0 bypass) is replaced so callers see FE-processed
  // output. Geometry is unchanged; format is SRGGB16 on fe_image0.
  auto image_dev = std::make_unique<V4L2Device>();
  if (!image_dev->open(image_path)) {
    LOG_ERROR("Camera", "Failed to open fe_image0 (" + image_path + ")");
    return false;
  }
  if (!image_dev->setFormat(config.width, config.height,
                            V4L2_PIX_FMT_SRGGB16)) {
    LOG_ERROR("Camera", "Failed to set SRGGB16 on fe_image0");
    return false;
  }
  if (!image_dev->setupBuffers(buffer_count)) {
    LOG_ERROR("Camera", "Failed to alloc fe_image0 buffers");
    return false;
  }

  auto stats_dev = std::make_unique<V4L2Device>();
  if (!stats_dev->open(stats_path)) {
    LOG_ERROR("Camera", "Failed to open fe_stats (" + stats_path + ")");
    return false;
  }
  // v4l2_fourcc('P','I','S','P') is the format exposed by the fe_stats node.
  constexpr uint32_t kFmtPispStats = v4l2_fourcc('P', 'I', 'S', 'P');
  if (!stats_dev->setMetaFormat(kFmtPispStats, sizeof(pisp_statistics),
                                /*output=*/false)) {
    LOG_ERROR("Camera", "Failed to set META_CAPTURE on fe_stats");
    return false;
  }
  if (!stats_dev->setupBuffers(buffer_count)) {
    LOG_ERROR("Camera", "Failed to alloc fe_stats buffers");
    return false;
  }

  auto cfg_dev = std::make_unique<V4L2Device>();
  if (!cfg_dev->open(cfg_path)) {
    LOG_ERROR("Camera", "Failed to open fe_config (" + cfg_path + ")");
    return false;
  }
  constexpr uint32_t kFmtPispCfg = v4l2_fourcc('P', 'I', 'S', 'P');
  if (!cfg_dev->setMetaFormat(kFmtPispCfg, sizeof(pisp_fe_config),
                              /*output=*/true)) {
    LOG_ERROR("Camera", "Failed to set META_OUTPUT on fe_config");
    return false;
  }
  if (!cfg_dev->setupBuffers(buffer_count)) {
    LOG_ERROR("Camera", "Failed to alloc fe_config buffers");
    return false;
  }

  auto fe_cfg = std::make_unique<FeConfigurator>();
  if (!fe_cfg->init(config.width, config.height)) {
    LOG_ERROR("Camera", "FeConfigurator::init failed");
    return false;
  }

  // Swap the capture device to fe_image0 and install the FE-side nodes.
  video_device = std::move(image_dev);
  stats_device = std::move(stats_dev);
  fe_config_device = std::move(cfg_dev);
  fe_configurator = std::move(fe_cfg);
  return true;
}

bool Camera::start() {
  if (state != CameraState::CONFIGURED) {
    LOG_ERROR("Camera",
              "Camera " + std::to_string(camera_id) + " is not configured");
    return false;
  }

  if (bayes_pipeline_active) {
    // Prime the config queue before STREAMON so the FE has a valid config
    // the moment it wakes up.
    if (!fe_config_device->queueMetaOutputBuffer(
            fe_configurator->config(), fe_configurator->configSize())) {
      LOG_ERROR("Camera", "Failed to queue initial fe_config");
      return false;
    }
    if (!fe_config_device->startStreaming() ||
        !stats_device->startStreaming() || !video_device->startStreaming()) {
      LOG_ERROR("Camera", "Failed to start Bayes triple-device stream");
      return false;
    }
  } else if (!video_device->startStreaming()) {
    LOG_ERROR("Camera", "Failed to start streaming for camera " +
                            std::to_string(camera_id));
    return false;
  }

  if (bayes_pipeline_active && !sensor_subdev_path_.empty()) {
    sensor_subdev_fd_ = ::open(sensor_subdev_path_.c_str(), O_RDONLY);
    if (sensor_subdev_fd_ < 0)
      LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                             " could not open sensor subdev for lux reads");
  }

  should_stop = false;
  capture_thread = std::make_unique<std::thread>(
      bayes_pipeline_active ? &Camera::captureLoopBayes : &Camera::captureLoop,
      this);

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

  // Wake up any waiting threads
  new_frame_cv.notify_all();

  if (capture_thread && capture_thread->joinable()) {
    capture_thread->join();
  }

  if (sensor_subdev_fd_ >= 0) {
    ::close(sensor_subdev_fd_);
    sensor_subdev_fd_ = -1;
  }

  video_device->stopStreaming();
  if (bayes_pipeline_active) {
    if (stats_device) stats_device->stopStreaming();
    if (fe_config_device) fe_config_device->stopStreaming();
  }

  state = CameraState::CONFIGURED;
  LOG_INFO("Camera", "Camera " + std::to_string(camera_id) + " stopped");
  return true;
}

void Camera::captureLoop() {
  LOG_DEBUG("Camera",
            "Camera " + std::to_string(camera_id) + " capture loop started");

  // Track capture failures for logging
  int consecutive_failures = 0;
  bool first_frame = true;

  while (!should_stop) {
    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter;

    if (video_device->captureFrame(frame)) {
      frame_counter++;
      consecutive_failures = 0;  // Reset failure counter

      if (first_frame) {
        LOG_INFO("Camera",
                 "Camera " + std::to_string(camera_id) + " got first frame");
        first_frame = false;
      }

      // Stamp AWB gains + CCT on the frame before downstream consumers see it.
      awb.update(frame);

      // Update latest frame for streaming
      {
        std::lock_guard<std::mutex> lock(latest_frame_mutex);
        latest_frame = frame;  // Deep copy
        has_new_frame = true;
      }

      // Notify streaming thread that new frame is available
      new_frame_cv.notify_one();

      // Also notify through the callback if set
      if (onFrameAvailable) {
        onFrameAvailable();
      }

      // Notify frame saver if callback is set and saving is enabled
      if (onFrameCaptured) {
        onFrameCaptured(frame);
      }

      LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                              " captured frame " +
                              std::to_string(frame.frame_id));
    } else {
      consecutive_failures++;

      // Log warnings for extended capture failures
      if (consecutive_failures == 10) {
        LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                               " has failed to capture 10 consecutive frames");
      } else if (consecutive_failures == 100) {
        LOG_ERROR("Camera", "Camera " + std::to_string(camera_id) +
                                " has failed to capture 100 consecutive frames "
                                "- check camera hardware");
      }

      // Small delay on capture failure to avoid busy loop
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  LOG_DEBUG("Camera",
            "Camera " + std::to_string(camera_id) + " capture loop ended");
}

bool Camera::getFrameForStreaming(FrameData& frame) {
  // First, check if streaming frame is still in use
  {
    std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
    if (streaming_frame_in_use) {
      LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                              " streaming frame still in use");
      return false;
    }
  }

  // Copy latest frame to streaming buffer
  {
    std::lock_guard<std::mutex> latest_lock(latest_frame_mutex);
    if (!has_new_frame) {
      return false;
    }

    // Copy to streaming buffer
    {
      std::lock_guard<std::mutex> stream_lock(streaming_frame_mutex);
      streaming_frame = latest_frame;  // Deep copy
      streaming_frame_in_use = true;
      frame = streaming_frame;  // Copy to output
    }

    has_new_frame = false;
  }

  return true;
}

bool Camera::waitForNewFrame(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(latest_frame_mutex);
  return new_frame_cv.wait_for(
      lock, timeout, [this] { return has_new_frame || should_stop.load(); });
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

double Camera::readSensorLux(const pisp_statistics& stats) const {
  if (sensor_subdev_fd_ < 0 || sensor_line_time_us_ <= 0.0) return 0.0;

  const auto& zone = stats.agc.floating[0];
  if (zone.counted == 0) return 0.0;
  double current_y = static_cast<double>(zone.Y_sum) / zone.counted;

  int32_t exp_lines = readV4l2Ctrl(sensor_subdev_fd_, V4L2_CID_EXPOSURE);
  int32_t gain_reg  = readV4l2Ctrl(sensor_subdev_fd_, V4L2_CID_ANALOGUE_GAIN);
  if (exp_lines <= 0 || gain_reg < 0) return 0.0;

  double exposure_us   = exp_lines * sensor_line_time_us_;
  // IMX519 gain encoding: linear = 1024 / (1024 - reg)
  double analogue_gain = 1024.0 / std::max(1.0, 1024.0 - gain_reg);

  double lux = (kLuxRefExposureUs / exposure_us) *
               (kLuxRefGain / analogue_gain) *
               (current_y / kLuxRefY) * kLuxRefLux;
  return std::max(0.0, lux);
}

void Camera::captureLoopBayes() {
  LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                          " Bayes capture loop started");

  int consecutive_failures = 0;
  bool first_frame = true;
  pisp_statistics stats{};

  while (!should_stop) {
    struct pollfd fds[2];
    fds[0] = {video_device->getFd(), POLLIN, 0};
    fds[1] = {stats_device->getFd(), POLLIN, 0};
    int r = ::poll(fds, 2, 200);
    if (r <= 0) continue;

    // Drain any ready stats buffers; take the most recent for the AWB feed.
    // One-frame lag between image and stats is expected and acceptable.
    bool have_stats = false;
    size_t stats_used = 0;
    if (fds[1].revents & POLLIN) {
      if (stats_device->dequeueMetaCaptureBuffer(&stats, sizeof(stats),
                                                 stats_used)) {
        have_stats = stats_used >= sizeof(pisp_awb_statistics);
      }
    }

    if (!(fds[0].revents & POLLIN)) {
      // Keep the FE fed even when no image is ready — otherwise the
      // hardware stalls waiting for its next config.
      fe_config_device->queueMetaOutputBuffer(fe_configurator->config(),
                                              fe_configurator->configSize());
      continue;
    }

    FrameData frame;
    frame.camera_id = camera_id;
    frame.frame_id = frame_counter;
    if (!video_device->captureFrame(frame)) {
      consecutive_failures++;
      if (consecutive_failures == 10) {
        LOG_WARN("Camera", "Camera " + std::to_string(camera_id) +
                               " (bayes) 10 consecutive capture failures");
      }
      continue;
    }
    consecutive_failures = 0;
    frame_counter++;
    if (first_frame) {
      LOG_INFO("Camera", "Camera " + std::to_string(camera_id) +
                             " got first FE frame");
      first_frame = false;
    }

    if (have_stats) {
      double lux = readSensorLux(stats);
      awb_bayes.update(frame, stats, lux);
    } else {
      awb_bayes.update(frame);
    }

    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex);
      latest_frame = frame;
      has_new_frame = true;
    }
    new_frame_cv.notify_one();
    if (onFrameAvailable) onFrameAvailable();
    if (onFrameCaptured) onFrameCaptured(frame);

    // Re-queue a fresh FE config for the next frame.
    fe_config_device->queueMetaOutputBuffer(fe_configurator->config(),
                                            fe_configurator->configSize());
  }

  LOG_DEBUG("Camera", "Camera " + std::to_string(camera_id) +
                          " Bayes capture loop ended");
}
