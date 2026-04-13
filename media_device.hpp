#pragma once

#include <linux/media.h>

#include <cstdint>
#include <map>
#include <string>

class MediaDevice {
 private:
  int fd = -1;
  std::string device_path;
  std::map<std::string, media_entity_desc> entities;
  std::map<uint32_t, media_entity_desc> entities_by_id;

  void enumerateEntities();
  std::string findDeviceNode(uint32_t major, uint32_t minor);

 public:
  MediaDevice() = default;
  ~MediaDevice();

  // Move semantics
  MediaDevice(MediaDevice&& other) noexcept;
  MediaDevice& operator=(MediaDevice&& other) noexcept;

  // Delete copy semantics
  MediaDevice(const MediaDevice&) = delete;
  MediaDevice& operator=(const MediaDevice&) = delete;

  bool open(const std::string& path);
  bool findAndOpen(const std::string& driver_name,
                   const std::string& model_name);
  void listEntities();
  bool getEntityByName(const std::string& name, media_entity_desc& entity);
  bool setCrop(const std::string& entity_name, uint32_t pad, uint32_t left,
               uint32_t top, uint32_t width, uint32_t height);
  bool setFormat(const std::string& entity_name, uint32_t pad, uint32_t width,
                 uint32_t height, uint32_t code);
  bool setLink(const std::string& source_entity, uint32_t source_pad,
               const std::string& sink_entity, uint32_t sink_pad, bool enable);
  bool reset();
  std::string getVideoDevicePath(const std::string& entity_name);
  std::string findIMX519SensorEntity();
  std::string getDevicePath() const { return device_path; }
};
