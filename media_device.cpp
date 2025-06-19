#include "media_device.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <vector>

#ifndef MEDIA_ENT_T_V4L2_SUBDEV_SENSOR
#define MEDIA_ENT_T_V4L2_SUBDEV_SENSOR (MEDIA_ENT_T_V4L2_SUBDEV + 1)
#endif

MediaDevice::~MediaDevice() {
  if (fd >= 0) close(fd);
}

MediaDevice::MediaDevice(MediaDevice&& other) noexcept
    : fd(other.fd),
      device_path(std::move(other.device_path)),
      entities(std::move(other.entities)),
      entities_by_id(std::move(other.entities_by_id)) {
  other.fd = -1;
}

MediaDevice& MediaDevice::operator=(MediaDevice&& other) noexcept {
  if (this != &other) {
    if (fd >= 0) close(fd);
    fd = other.fd;
    device_path = std::move(other.device_path);
    entities = std::move(other.entities);
    entities_by_id = std::move(other.entities_by_id);
    other.fd = -1;
  }
  return *this;
}

bool MediaDevice::open(const std::string& path) {
  fd = ::open(path.c_str(), O_RDWR);
  if (fd < 0) return false;

  device_path = path;
  enumerateEntities();
  return true;
}

bool MediaDevice::findAndOpen(const std::string& driver_name,
                              const std::string& model_name) {
  DIR* dir = opendir("/dev");
  if (!dir) return false;

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "media", 5) != 0) continue;

    std::string path = "/dev/" + std::string(entry->d_name);
    int test_fd = ::open(path.c_str(), O_RDWR);
    if (test_fd < 0) continue;

    struct media_device_info info;
    if (ioctl(test_fd, MEDIA_IOC_DEVICE_INFO, &info) == 0) {
      if (std::string(info.driver) == driver_name &&
          std::string(info.model) == model_name) {
        fd = test_fd;
        device_path = path;
        closedir(dir);
        enumerateEntities();
        return true;
      }
    }
    close(test_fd);
  }
  closedir(dir);
  return false;
}

void MediaDevice::enumerateEntities() {
  media_entity_desc entity;
  memset(&entity, 0, sizeof(entity));
  entity.id = MEDIA_ENT_ID_FLAG_NEXT;

  while (ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &entity) == 0) {
    entities[entity.name] = entity;
    entities_by_id[entity.id] = entity;
    entity.id |= MEDIA_ENT_ID_FLAG_NEXT;
  }
}

void MediaDevice::listEntities() {
  std::cout << "Available entities:" << std::endl;
  for (const auto& pair : entities) {
    std::cout << "  - " << pair.first << " (id: " << pair.second.id
              << ", type: " << pair.second.type << ")" << std::endl;
  }
}

bool MediaDevice::getEntityByName(const std::string& name,
                                  media_entity_desc& entity) {
  auto it = entities.find(name);
  if (it != entities.end()) {
    entity = it->second;
    return true;
  }
  return false;
}

std::string MediaDevice::findDeviceNode(uint32_t major, uint32_t minor) {
  DIR* dir = opendir("/dev");
  if (!dir) return "";

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "v4l-subdev", 10) == 0) {
      std::string path = "/dev/" + std::string(entry->d_name);
      struct stat st;
      if (stat(path.c_str(), &st) == 0) {
        if (::major(st.st_rdev) == major && ::minor(st.st_rdev) == minor) {
          closedir(dir);
          return path;
        }
      }
    }
  }
  closedir(dir);
  return "";
}

bool MediaDevice::setCrop(const std::string& entity_name, uint32_t pad,
                          uint32_t left, uint32_t top, uint32_t width,
                          uint32_t height) {
  media_entity_desc entity;
  if (!getEntityByName(entity_name, entity)) {
    std::cerr << "Entity not found: " << entity_name << std::endl;
    return false;
  }

  std::string devpath = findDeviceNode(entity.v4l.major, entity.v4l.minor);
  if (devpath.empty()) {
    std::cerr << "Failed to find device node for " << entity_name << std::endl;
    return false;
  }

  int subdev_fd = ::open(devpath.c_str(), O_RDWR);
  if (subdev_fd < 0) {
    std::cerr << "Failed to open " << devpath << ": " << strerror(errno)
              << std::endl;
    return false;
  }

  struct v4l2_subdev_selection sel;
  memset(&sel, 0, sizeof(sel));
  sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  sel.pad = pad;
  sel.target = V4L2_SEL_TGT_CROP;
  sel.r.left = left;
  sel.r.top = top;
  sel.r.width = width;
  sel.r.height = height;

  int ret = ioctl(subdev_fd, VIDIOC_SUBDEV_S_SELECTION, &sel);
  close(subdev_fd);
  return ret == 0;
}

bool MediaDevice::setFormat(const std::string& entity_name, uint32_t pad,
                            uint32_t width, uint32_t height, uint32_t code) {
  media_entity_desc entity;
  if (!getEntityByName(entity_name, entity)) {
    std::cerr << "Entity not found: " << entity_name << std::endl;
    return false;
  }

  std::string devpath = findDeviceNode(entity.v4l.major, entity.v4l.minor);
  if (devpath.empty()) {
    std::cerr << "Failed to find device node for " << entity_name << std::endl;
    return false;
  }

  int subdev_fd = ::open(devpath.c_str(), O_RDWR);
  if (subdev_fd < 0) {
    std::cerr << "Failed to open " << devpath << ": " << strerror(errno)
              << std::endl;
    return false;
  }

  struct v4l2_subdev_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  fmt.pad = pad;
  fmt.format.width = width;
  fmt.format.height = height;
  fmt.format.code = code;
  fmt.format.field = V4L2_FIELD_NONE;
  fmt.format.colorspace = V4L2_COLORSPACE_RAW;
  fmt.format.xfer_func = V4L2_XFER_FUNC_NONE;
  fmt.format.ycbcr_enc = V4L2_YCBCR_ENC_601;
  fmt.format.quantization = V4L2_QUANTIZATION_FULL_RANGE;

  int ret = ioctl(subdev_fd, VIDIOC_SUBDEV_S_FMT, &fmt);
  close(subdev_fd);
  return ret == 0;
}

bool MediaDevice::setLink(const std::string& source_entity, uint32_t source_pad,
                          const std::string& sink_entity, uint32_t sink_pad,
                          bool enable) {
  media_entity_desc source, sink;
  if (!getEntityByName(source_entity, source)) {
    std::cerr << "Source entity not found: " << source_entity << std::endl;
    return false;
  }
  if (!getEntityByName(sink_entity, sink)) {
    std::cerr << "Sink entity not found: " << sink_entity << std::endl;
    return false;
  }

  // Try to find the link by enumerating from both source and sink entities
  bool link_found = false;
  bool link_is_immutable = false;
  bool link_is_enabled = false;

  // First try enumerating from the source entity
  for (int attempt = 0; attempt < 2; attempt++) {
    uint32_t entity_to_check = (attempt == 0) ? source.id : sink.id;
    media_entity_desc& check_entity = (attempt == 0) ? source : sink;

    struct media_links_enum links_enum;
    memset(&links_enum, 0, sizeof(links_enum));
    links_enum.entity = entity_to_check;

    // Get link count
    if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) < 0) {
      continue;  // Try the other entity
    }

    __u32 pads_count = check_entity.pads;
    __u32 links_count = check_entity.links;

    if (links_count > 0) {
      std::vector<media_pad_desc> pad_descs(pads_count);
      std::vector<media_link_desc> link_descs(links_count);

      links_enum.pads = pad_descs.data();
      links_enum.links = link_descs.data();

      // Get actual links
      if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) == 0) {
        // Check if the link exists
        for (const auto& link : link_descs) {
          if (link.source.entity == source.id &&
              link.source.index == source_pad && link.sink.entity == sink.id &&
              link.sink.index == sink_pad) {
            link_found = true;
            link_is_immutable = (link.flags & MEDIA_LNK_FL_IMMUTABLE) != 0;
            link_is_enabled = (link.flags & MEDIA_LNK_FL_ENABLED) != 0;

            std::cout << "Link " << source_entity << ":" << source_pad << " -> "
                      << sink_entity << ":" << sink_pad
                      << " found with flags: 0x" << std::hex << link.flags
                      << std::dec;
            if (link_is_immutable) std::cout << " (IMMUTABLE)";
            if (link_is_enabled) std::cout << " (ENABLED)";
            std::cout << std::endl;

            break;
          }
        }
      }
    }
    if (link_found) break;
  }

  // If link is immutable, check if it's already in the desired state
  if (link_found && link_is_immutable) {
    if (link_is_enabled == enable) {
      std::cout << "Link is immutable and already "
                << (enable ? "enabled" : "disabled") << std::endl;
      return true;
    } else {
      std::cerr << "Link is immutable and cannot be changed" << std::endl;
      return false;
    }
  }

  // If link already exists in desired state, we're done
  if (link_found && link_is_enabled == enable) {
    std::cout << "Link is already " << (enable ? "enabled" : "disabled")
              << std::endl;
    return true;
  }

  // Try to set the link
  struct media_link_desc link;
  memset(&link, 0, sizeof(link));
  link.source.entity = source.id;
  link.source.index = source_pad;
  link.sink.entity = sink.id;
  link.sink.index = sink_pad;
  link.flags = enable ? MEDIA_LNK_FL_ENABLED : 0;

  if (ioctl(fd, MEDIA_IOC_SETUP_LINK, &link) < 0) {
    std::cerr << "MEDIA_IOC_SETUP_LINK failed: " << strerror(errno)
              << std::endl;
    return false;
  }

  std::cout << "Successfully " << (enable ? "enabled" : "disabled") << " link"
            << std::endl;
  return true;
}

bool MediaDevice::reset() {
  struct media_entity_desc entity;
  memset(&entity, 0, sizeof(entity));
  entity.id = MEDIA_ENT_ID_FLAG_NEXT;

  while (ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &entity) == 0) {
    struct media_links_enum links_enum;
    memset(&links_enum, 0, sizeof(links_enum));
    links_enum.entity = entity.id;

    if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) < 0) {
      entity.id |= MEDIA_ENT_ID_FLAG_NEXT;
      continue;
    }

    __u32 pads_count = entity.pads;
    __u32 links_count = entity.links;

    if (links_count > 0) {
      std::vector<media_pad_desc> pad_descs(pads_count);
      std::vector<media_link_desc> link_descs(links_count);

      links_enum.pads = pad_descs.data();
      links_enum.links = link_descs.data();

      if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) == 0) {
        for (size_t i = 0; i < links_count; i++) {
          if (!(link_descs[i].flags & MEDIA_LNK_FL_IMMUTABLE) &&
              (link_descs[i].flags & MEDIA_LNK_FL_ENABLED)) {
            link_descs[i].flags &= ~MEDIA_LNK_FL_ENABLED;
            ioctl(fd, MEDIA_IOC_SETUP_LINK, &link_descs[i]);
          }
        }
      }
    }

    entity.id |= MEDIA_ENT_ID_FLAG_NEXT;
  }
  return true;
}

std::string MediaDevice::getVideoDevicePath(const std::string& entity_name) {
  media_entity_desc entity;
  if (getEntityByName(entity_name, entity)) {
    DIR* dir = opendir("/dev");
    if (!dir) return "";

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strncmp(entry->d_name, "video", 5) == 0) {
        std::string path = "/dev/" + std::string(entry->d_name);
        struct stat st;
        if (stat(path.c_str(), &st) == 0) {
          if (::major(st.st_rdev) == entity.v4l.major &&
              ::minor(st.st_rdev) == entity.v4l.minor) {
            closedir(dir);
            return path;
          }
        }
      }
    }
    closedir(dir);
  }
  return "";
}

std::string MediaDevice::findIMX296SensorEntity() {
  for (const auto& pair : entities) {
    if (pair.first.find("imx296") != std::string::npos) {
      return pair.first;
    }
  }
  return "";
}
