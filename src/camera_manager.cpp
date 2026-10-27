#include "camera_manager.hpp"

#include <algorithm>
#include <limits>

CameraManager::CameraManager() {
  lcam_manager = std::make_unique<libcamera::CameraManager>();
}

CameraManager::~CameraManager() {
  stopAll();
  cameras.clear();
  pool_.clear();
  if (lcam_manager) lcam_manager->stop();
}

namespace {

std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

}  // namespace

size_t CameraManager::discoverCameras(const std::string& sensor) {
  const std::string sensor_lower = toLower(sensor);
  if (!cameras.empty() && sensor_lower == active_sensor_) {
    LOG_INFO("CameraManager",
             "Cameras already discovered: " + std::to_string(cameras.size()));
    return cameras.size();
  }
  for (Camera* camera : cameras) {
    if (camera->getState() != CameraState::IDLE) {
      LOG_ERROR("CameraManager",
                "Cannot discover '" + sensor + "': camera " +
                    std::to_string(camera->getId()) +
                    " is configured; keeping the '" + active_sensor_ + "' cameras");
      return cameras.size();
    }
  }

  if (!lcam_started_) {
    if (lcam_manager->start() < 0) {
      LOG_ERROR("CameraManager", "Failed to start libcamera CameraManager");
      return 0;
    }
    lcam_started_ = true;
  }

  std::vector<Camera*> found;
  for (auto& lcam : lcam_manager->cameras()) {
    // Filter to cameras whose sensor model matches the requested substring.
    auto model = lcam->properties().get(libcamera::properties::Model);
    if (!model) continue;
    std::string model_str(model->begin(), model->end());
    if (toLower(model_str).find(sensor_lower) == std::string::npos) continue;

    const uint32_t id = static_cast<uint32_t>(found.size());
    LOG_INFO("CameraManager", "Found camera: id=" + lcam->id() + " model=" +
                                  model_str + " -> camera " + std::to_string(id));

    std::unique_ptr<Camera>& cam = pool_[lcam->id()];
    if (!cam) {
      cam = std::make_unique<Camera>(id, lcam, default_config, model_str);
      if (stream_manager_notify) {
        cam->setFrameAvailableCallback(stream_manager_notify);
      }
    } else {
      cam->setId(id);
    }
    found.push_back(cam.get());
  }

  {
    std::unique_lock<std::shared_mutex> lock(cameras_mutex_);
    cameras = std::move(found);
    active_sensor_ = cameras.empty() ? std::string() : sensor_lower;
  }

  LOG_INFO("CameraManager",
           "Discovered " + std::to_string(cameras.size()) +
               " camera(s) matching sensor '" + sensor + "'");
  return cameras.size();
}

std::string CameraManager::activeSensor() const {
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  return active_sensor_;
}

size_t CameraManager::getCameraCount() const {
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  return cameras.size();
}

bool CameraManager::configureAll(const CameraConfig& config,
                                 size_t buffer_count) {
  if (cameras.empty()) {
    LOG_ERROR("CameraManager", "No cameras to configure");
    return false;
  }

  for (const auto& camera : cameras) {
    if (camera->getState() == CameraState::RUNNING) {
      LOG_WARN("CameraManager",
               "Cannot configure cameras while they are running");
      return false;
    }
  }

  default_config = config;

  for (auto& camera : cameras) {
    camera->setConfig(config);
    LOG_INFO("CameraManager",
             "Configuring camera " + std::to_string(camera->getId()));
    if (!camera->configure(buffer_count)) {
      LOG_ERROR("CameraManager", "Failed to configure camera " +
                                     std::to_string(camera->getId()));
      return false;
    }
  }
  return true;
}

bool CameraManager::startAll() {
  for (auto& camera : cameras) {
    LOG_INFO("CameraManager",
             "Starting camera " + std::to_string(camera->getId()));
    if (!camera->start()) {
      LOG_ERROR("CameraManager",
                "Failed to start camera " + std::to_string(camera->getId()));
      stopAll();
      return false;
    }
  }
  LOG_INFO("CameraManager", "All cameras started successfully");
  return true;
}

bool CameraManager::stopAll() {
  bool all_ok = true;
  for (auto& camera : cameras) {
    LOG_INFO("CameraManager",
             "Stopping camera " + std::to_string(camera->getId()));
    if (!camera->stop()) {
      LOG_ERROR("CameraManager",
                "Failed to stop camera " + std::to_string(camera->getId()));
      all_ok = false;
    }
  }
  return all_ok;
}

bool CameraManager::unconfigureAll() {
  bool all_ok = true;
  for (auto& camera : cameras) {
    if (!camera->unconfigure()) {
      LOG_ERROR("CameraManager", "Failed to unconfigure camera " +
                                     std::to_string(camera->getId()));
      all_ok = false;
    }
  }
  return all_ok;
}

Camera* CameraManager::getCamera(uint32_t camera_id) {
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  if (camera_id >= cameras.size()) return nullptr;
  return cameras[camera_id];
}

std::vector<Camera*> CameraManager::getAllCameras() {
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  return cameras;
}

void CameraManager::setFrameCallback(
    std::function<void(const std::shared_ptr<const FrameData>&)> callback) {
  for (auto& camera : cameras) camera->onFrameCaptured = callback;
}

void CameraManager::setRawFrameCallback(
    std::function<void(const RawFrame&)> callback) {
  for (auto& camera : cameras) camera->onRawFrame = callback;
}

void CameraManager::setLiveFrames(bool enabled) {
  for (auto& camera : cameras) camera->setLiveFrames(enabled);
}

void CameraManager::setStreamManagerNotify(std::function<void()> callback) {
  stream_manager_notify = callback;
  for (auto& [id, camera] : pool_) camera->setFrameAvailableCallback(callback);
}

std::vector<CameraManager::StalledCamera> CameraManager::pollStalledCameras() {
  std::vector<StalledCamera> stalled;
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  for (Camera* camera : cameras) {
    const uint64_t stalled_for = camera->captureStalledForUs();
    if (stalled_for == 0) continue;
    // markCaptureFailed() is the once-only gate: a camera already reported
    // stays silent, so a supervisor emits one error per failure, not per tick.
    if (!camera->markCaptureFailed()) continue;
    stalled.push_back({camera->getId(), stalled_for});
  }
  return stalled;
}

bool CameraManager::areAnyRunning() const {
  std::shared_lock<std::shared_mutex> lock(cameras_mutex_);
  for (const Camera* camera : cameras) {
    if (camera->getState() == CameraState::RUNNING) return true;
  }
  return false;
}

void CameraManager::resetFrameCounts() {
  for (auto& camera : cameras) camera->resetFrameCounts();
  LOG_INFO("CameraManager", "Reset frame counts for all " +
                                std::to_string(cameras.size()) + " cameras");
}

void CameraManager::setLensPosition(float lens_position) {
  for (auto& camera : cameras) camera->setLensPosition(lens_position);
  LOG_INFO("CameraManager",
           "setLensPosition(" + std::to_string(lens_position) +
               ") fanned out to " + std::to_string(cameras.size()) +
               " camera(s)");
}

void CameraManager::setExposureTime(int32_t exposure_time_us) {
  for (auto& camera : cameras) camera->setExposureTime(exposure_time_us);
  LOG_INFO("CameraManager", "setExposureTime fanned out to " +
                                std::to_string(cameras.size()) + " camera(s)");
}

void CameraManager::setFrameDuration(int64_t frame_duration_us) {
  for (auto& camera : cameras) camera->setFrameDuration(frame_duration_us);
  LOG_INFO("CameraManager", "setFrameDuration fanned out to " +
                                std::to_string(cameras.size()) + " camera(s)");
}

int64_t CameraManager::getCurrentFrameDuration() const {
  if (cameras.empty()) return 0;
  return cameras.front()->getCurrentFrameDuration();
}

std::pair<int64_t, int64_t> CameraManager::getFrameDurationLimitsHw() const {
  if (cameras.empty()) return {0, 0};
  return cameras.front()->getFrameDurationLimitsHw();
}

std::optional<focus_region::Region> CameraManager::getFocusFomRegion() const {
  if (cameras.empty()) return std::nullopt;
  return cameras.front()->getFocusFomRegion();
}

LensPositionLimits CameraManager::getLensPositionLimitsHw() const {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  if (cameras.empty()) return {nan, nan, nan};
  return cameras.front()->getLensPositionLimitsHw();
}
