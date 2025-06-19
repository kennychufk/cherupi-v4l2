#include "camera_manager.hpp"

#include <dirent.h>

#include <cstring>
#include <iostream>

size_t CameraManager::discoverCameras() {
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
        std::cout << "Found IMX296 camera at " << path
                  << " with sensor entity: " << sensor_entity << std::endl;
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
    cameras.push_back(std::move(camera));
  }

  std::cout << "Discovered " << cameras.size() << " IMX296 camera(s)"
            << std::endl;
  return cameras.size();
}

bool CameraManager::configureAll(const CameraConfig& config,
                                 size_t buffer_count) {
  if (cameras.empty()) {
    std::cerr << "No cameras to configure" << std::endl;
    return false;
  }

  // Update default config
  default_config = config;

  for (auto& camera : cameras) {
    CameraConfig cam_config = config;
    cam_config.sensor_entity = camera->getConfig().sensor_entity;

    std::cout << "Configuring camera " << camera->getId() << "..." << std::endl;
    if (!camera->configure(buffer_count)) {
      std::cerr << "Failed to configure camera " << camera->getId()
                << std::endl;
      return false;
    }
  }

  return true;
}

bool CameraManager::startAll() {
  for (auto& camera : cameras) {
    std::cout << "Starting camera " << camera->getId() << "..." << std::endl;
    if (!camera->start()) {
      std::cerr << "Failed to start camera " << camera->getId() << std::endl;
      // Stop all cameras if one fails
      stopAll();
      return false;
    }
  }

  std::cout << "All cameras started successfully" << std::endl;
  return true;
}

bool CameraManager::stopAll() {
  bool all_success = true;

  for (auto& camera : cameras) {
    std::cout << "Stopping camera " << camera->getId() << "..." << std::endl;
    if (!camera->stop()) {
      std::cerr << "Failed to stop camera " << camera->getId() << std::endl;
      all_success = false;
    }
    std::cout << "Stopped camera " << camera->getId() << "..." << std::endl;
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
}
