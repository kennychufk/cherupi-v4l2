#include "camera_manager.hpp"

#include <algorithm>

CameraManager::CameraManager() {
  lcam_manager = std::make_unique<libcamera::CameraManager>();
}

CameraManager::~CameraManager() {
  stopAll();
  cameras.clear();
  if (lcam_manager) lcam_manager->stop();
}

size_t CameraManager::discoverCameras() {
  if (!cameras.empty()) {
    LOG_INFO("CameraManager",
             "Cameras already discovered: " + std::to_string(cameras.size()));
    return cameras.size();
  }

  if (lcam_manager->start() < 0) {
    LOG_ERROR("CameraManager", "Failed to start libcamera CameraManager");
    return 0;
  }

  uint32_t id = 0;
  for (auto& lcam : lcam_manager->cameras()) {
    // Filter to IMX519 cameras by model property.
    auto model = lcam->properties().get(libcamera::properties::Model);
    if (!model) continue;
    std::string model_str(model->begin(), model->end());
    // Convert to lower-case for comparison.
    std::string lower = model_str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower.find("imx519") == std::string::npos) continue;

    LOG_INFO("CameraManager",
             "Found IMX519 camera: id=" + lcam->id() + " model=" + model_str);

    auto cam = std::make_unique<Camera>(id, lcam, default_config);
    if (stream_manager_notify) {
      cam->setFrameAvailableCallback(stream_manager_notify);
    }
    cameras.push_back(std::move(cam));
    ++id;
  }

  LOG_INFO("CameraManager",
           "Discovered " + std::to_string(cameras.size()) + " IMX519 camera(s)");
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
    camera->setAwbConfig(config.awb);
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
  if (camera_id >= cameras.size()) return nullptr;
  return cameras[camera_id].get();
}

std::vector<Camera*> CameraManager::getAllCameras() {
  std::vector<Camera*> result;
  for (auto& cam : cameras) result.push_back(cam.get());
  return result;
}

void CameraManager::setFrameCallback(
    std::function<void(const FrameData&)> callback) {
  for (auto& camera : cameras) camera->onFrameCaptured = callback;
}

void CameraManager::setStreamManagerNotify(std::function<void()> callback) {
  stream_manager_notify = callback;
  for (auto& camera : cameras) camera->setFrameAvailableCallback(callback);
}

bool CameraManager::areAnyRunning() const {
  for (const auto& camera : cameras) {
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
