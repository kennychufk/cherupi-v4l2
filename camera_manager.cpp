#include "camera_manager.hpp"

#include <dirent.h>

#include <cstring>
#include <iostream>

CameraManager::CameraManager() {
  // Constructor
}

CameraManager::~CameraManager() {
  // Destructor - ensure cleanup
  stopAll();
}

size_t CameraManager::discoverCameras() {
  // If cameras are already discovered and running, just return the count
  if (!cameras.empty()) {
    LOG_INFO("CameraManager",
             "Cameras already discovered, returning existing count: " +
                 std::to_string(cameras.size()));
    return cameras.size();
  }

  // Only do actual discovery if no cameras exist
  cameras.clear();
  media_devices.clear();

  DIR* dir = opendir("/dev");
  if (!dir) return 0;

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "media", 5) != 0) continue;

    std::string path = "/dev/" + std::string(entry->d_name);
    MediaDevice media;
    if (media.open(path)) {
      // Check if this media device has an IMX296 sensor
      std::string sensor_entity = media.findIMX296SensorEntity();
      if (!sensor_entity.empty()) {
        LOG_INFO("CameraManager", "Found IMX296 camera at " + path +
                                      " with sensor entity: " + sensor_entity);
        media_devices.push_back(std::move(media));
      }
    }
  }
  closedir(dir);

  // Create Camera objects for each discovered device
  for (size_t i = 0; i < media_devices.size(); i++) {
    CameraConfig config = default_config;
    config.sensor_entity = media_devices[i].findIMX296SensorEntity();

    auto camera = std::make_unique<Camera>(i, &media_devices[i], config);

    // Set frame available callback to notify stream manager
    if (stream_manager_notify) {
      camera->setFrameAvailableCallback(stream_manager_notify);
    }

    cameras.push_back(std::move(camera));
  }

  LOG_INFO("CameraManager", "Discovered " + std::to_string(cameras.size()) +
                                " IMX296 camera(s)");
  return cameras.size();
}

bool CameraManager::configureAll(const CameraConfig& config,
                                 size_t buffer_count) {
  if (cameras.empty()) {
    LOG_ERROR("CameraManager", "No cameras to configure");
    return false;
  }

  // Check if cameras are already running
  bool any_running = false;
  for (const auto& camera : cameras) {
    if (camera->getState() == CameraState::RUNNING) {
      any_running = true;
      break;
    }
  }

  if (any_running) {
    LOG_WARN("CameraManager",
             "Cannot configure cameras while they are running");
    return false;
  }

  // Update default config
  default_config = config;

  for (auto& camera : cameras) {
    CameraConfig cam_config = config;
    cam_config.sensor_entity = camera->getConfig().sensor_entity;

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
      // Stop all cameras if one fails
      stopAll();
      return false;
    }
  }

  LOG_INFO("CameraManager", "All cameras started successfully");
  return true;
}

bool CameraManager::stopAll() {
  bool all_success = true;

  for (auto& camera : cameras) {
    LOG_INFO("CameraManager",
             "Stopping camera " + std::to_string(camera->getId()));
    if (!camera->stop()) {
      LOG_ERROR("CameraManager",
                "Failed to stop camera " + std::to_string(camera->getId()));
      all_success = false;
    }
  }

  return all_success;
}

Camera* CameraManager::getCamera(uint32_t camera_id) {
  if (camera_id >= cameras.size()) {
    return nullptr;
  }
  return cameras[camera_id].get();
}

std::vector<Camera*> CameraManager::getAllCameras() {
  std::vector<Camera*> result;
  for (auto& camera : cameras) {
    result.push_back(camera.get());
  }
  return result;
}

void CameraManager::setFrameCallback(
    std::function<void(const FrameData&)> callback) {
  for (auto& camera : cameras) {
    camera->onFrameCaptured = callback;
  }

  if (callback) {
    LOG_INFO("CameraManager", "Frame callback set for all cameras");
  } else {
    LOG_INFO("CameraManager", "Frame callback cleared for all cameras");
  }
}

void CameraManager::setStreamManagerNotify(std::function<void()> callback) {
  stream_manager_notify = callback;

  // Update existing cameras
  for (auto& camera : cameras) {
    camera->setFrameAvailableCallback(callback);
  }
}

bool CameraManager::areAnyRunning() const {
  for (const auto& camera : cameras) {
    if (camera->getState() == CameraState::RUNNING) {
      return true;
    }
  }
  return false;
}
