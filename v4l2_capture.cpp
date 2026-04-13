// a monolithic capture program that demonstrates how cameras can be controlled
// using v4l2. camera_ws_server is built upon this program
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Define the sensor type if not already defined
#ifndef MEDIA_ENT_T_V4L2_SUBDEV_SENSOR
#define MEDIA_ENT_T_V4L2_SUBDEV_SENSOR (MEDIA_ENT_T_V4L2_SUBDEV + 1)
#endif

// Capture pipeline modes
enum class CaptureMode {
  BUFFER_ALL,  // Buffer all frames in memory before writing
  BATCH_WRITE  // Write frames in batches while capturing
};

// Frame data structure
struct FrameData {
  std::vector<uint8_t> data;
  size_t frame_number;
  size_t camera_index;
  std::chrono::steady_clock::time_point timestamp;
};

// Write task for batch writing
struct WriteTask {
  std::string filename;
  std::vector<uint8_t> data;
};

class MediaDevice {
 private:
  int fd = -1;
  std::string device_path;
  std::map<std::string, media_entity_desc> entities;
  std::map<uint32_t, media_entity_desc> entities_by_id;

 public:
  MediaDevice() = default;

  // Move constructor
  MediaDevice(MediaDevice&& other) noexcept
      : fd(other.fd),
        device_path(std::move(other.device_path)),
        entities(std::move(other.entities)),
        entities_by_id(std::move(other.entities_by_id)) {
    other.fd = -1;  // Prevent the moved-from object from closing the fd
  }

  // Move assignment operator
  MediaDevice& operator=(MediaDevice&& other) noexcept {
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

  // Delete copy operations
  MediaDevice(const MediaDevice&) = delete;
  MediaDevice& operator=(const MediaDevice&) = delete;

  ~MediaDevice() {
    if (fd >= 0) close(fd);
  }

  bool open(const std::string& path) {
    fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) return false;

    device_path = path;
    enumerateEntities();
    return true;
  }

  bool findAndOpen(const std::string& driver_name,
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

  void enumerateEntities() {
    media_entity_desc entity;
    memset(&entity, 0, sizeof(entity));
    entity.id = MEDIA_ENT_ID_FLAG_NEXT;

    while (ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &entity) == 0) {
      entities[entity.name] = entity;
      entities_by_id[entity.id] = entity;
      entity.id |= MEDIA_ENT_ID_FLAG_NEXT;
    }
  }

  void listEntities() {
    std::cout << "Available entities:" << std::endl;
    for (const auto& pair : entities) {
      std::cout << "  - " << pair.first << " (id: " << pair.second.id
                << ", type: " << pair.second.type << ")" << std::endl;
    }
  }

  bool getEntityByName(const std::string& name, media_entity_desc& entity) {
    auto it = entities.find(name);
    if (it != entities.end()) {
      entity = it->second;
      return true;
    }
    return false;
  }

  std::string findDeviceNode(uint32_t major, uint32_t minor) {
    DIR* dir = opendir("/dev");
    if (!dir) return "";

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strncmp(entry->d_name, "v4l-subdev", 10) == 0) {
        std::string path = "/dev/" + std::string(entry->d_name);
        struct stat st;
        if (stat(path.c_str(), &st) == 0) {
          if (major(st.st_rdev) == major && minor(st.st_rdev) == minor) {
            closedir(dir);
            return path;
          }
        }
      }
    }
    closedir(dir);
    return "";
  }

  bool setCrop(const std::string& entity_name, uint32_t pad, uint32_t left,
               uint32_t top, uint32_t width, uint32_t height) {
    media_entity_desc entity;
    if (!getEntityByName(entity_name, entity)) {
      std::cerr << "Entity not found: " << entity_name << std::endl;
      return false;
    }

    // Find the actual device node by major/minor
    std::string devpath = findDeviceNode(entity.v4l.major, entity.v4l.minor);
    if (devpath.empty()) {
      std::cerr << "Failed to find device node for " << entity_name
                << " (major=" << entity.v4l.major
                << ", minor=" << entity.v4l.minor << ")" << std::endl;
      return false;
    }

    std::cout << "Setting crop on " << devpath << " for " << entity_name
              << std::endl;

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
    if (ret < 0) {
      std::cerr << "VIDIOC_SUBDEV_S_SELECTION failed on " << entity_name
                << " pad " << pad << ": " << strerror(errno) << std::endl;

      // Try to get the current crop to see what's supported
      struct v4l2_subdev_selection get_sel;
      memset(&get_sel, 0, sizeof(get_sel));
      get_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
      get_sel.pad = pad;
      get_sel.target = V4L2_SEL_TGT_CROP;

      if (ioctl(subdev_fd, VIDIOC_SUBDEV_G_SELECTION, &get_sel) == 0) {
        std::cerr << "Current crop: " << get_sel.r.left << "," << get_sel.r.top
                  << " " << get_sel.r.width << "x" << get_sel.r.height
                  << std::endl;
      }
    } else {
      std::cout << "Set crop to (" << left << "," << top << ")/" << width << "x"
                << height << std::endl;
    }
    close(subdev_fd);
    return ret == 0;
  }

  bool setFormat(const std::string& entity_name, uint32_t pad, uint32_t width,
                 uint32_t height, uint32_t code) {
    media_entity_desc entity;
    if (!getEntityByName(entity_name, entity)) {
      std::cerr << "Entity not found: " << entity_name << std::endl;
      return false;
    }

    // Find the actual device node by major/minor
    std::string devpath = findDeviceNode(entity.v4l.major, entity.v4l.minor);
    if (devpath.empty()) {
      std::cerr << "Failed to find device node for " << entity_name
                << " (major=" << entity.v4l.major
                << ", minor=" << entity.v4l.minor << ")" << std::endl;
      return false;
    }

    std::cout << "Setting format on " << devpath << " for " << entity_name
              << std::endl;

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
    if (ret < 0) {
      std::cerr << "VIDIOC_SUBDEV_S_FMT failed on " << entity_name << " pad "
                << pad << ": " << strerror(errno) << std::endl;

      // Try to get the current format to see what's supported
      struct v4l2_subdev_format get_fmt;
      memset(&get_fmt, 0, sizeof(get_fmt));
      get_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
      get_fmt.pad = pad;

      if (ioctl(subdev_fd, VIDIOC_SUBDEV_G_FMT, &get_fmt) == 0) {
        std::cerr << "Current format: " << get_fmt.format.width << "x"
                  << get_fmt.format.height << " code: 0x" << std::hex
                  << get_fmt.format.code << std::dec << std::endl;
      }
    }
    close(subdev_fd);
    return ret == 0;
  }

  bool setLink(const std::string& source_entity, uint32_t source_pad,
               const std::string& sink_entity, uint32_t sink_pad, bool enable) {
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
                link.source.index == source_pad &&
                link.sink.entity == sink.id && link.sink.index == sink_pad) {
              link_found = true;
              link_is_immutable = (link.flags & MEDIA_LNK_FL_IMMUTABLE) != 0;
              link_is_enabled = (link.flags & MEDIA_LNK_FL_ENABLED) != 0;

              std::cout << "Link " << source_entity << ":" << source_pad
                        << " -> " << sink_entity << ":" << sink_pad
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

  bool reset() {
    struct media_entity_desc entity;
    memset(&entity, 0, sizeof(entity));
    entity.id = MEDIA_ENT_ID_FLAG_NEXT;

    while (ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &entity) == 0) {
      // For each entity, enumerate and disable its links
      struct media_links_enum links_enum;
      memset(&links_enum, 0, sizeof(links_enum));
      links_enum.entity = entity.id;

      // First call to get counts
      if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) < 0) {
        entity.id |= MEDIA_ENT_ID_FLAG_NEXT;
        continue;
      }

      // The media_links_enum structure stores counts in reserved fields after
      // first call We need to extract the actual counts from the structure
      __u32 pads_count = entity.pads;
      __u32 links_count = entity.links;

      if (links_count > 0) {
        std::vector<media_pad_desc> pad_descs(pads_count);
        std::vector<media_link_desc> link_descs(links_count);

        // Setup the pointers for the second call
        links_enum.pads = pad_descs.data();
        links_enum.links = link_descs.data();

        // Second call to get the actual link data
        if (ioctl(fd, MEDIA_IOC_ENUM_LINKS, &links_enum) == 0) {
          // Disable all non-immutable links
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

  std::string getVideoDevicePath(const std::string& entity_name) {
    media_entity_desc entity;
    if (getEntityByName(entity_name, entity)) {
      // Find video device node by major/minor
      DIR* dir = opendir("/dev");
      if (!dir) return "";

      struct dirent* entry;
      while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "video", 5) == 0) {
          std::string path = "/dev/" + std::string(entry->d_name);
          struct stat st;
          if (stat(path.c_str(), &st) == 0) {
            if (major(st.st_rdev) == entity.v4l.major &&
                minor(st.st_rdev) == entity.v4l.minor) {
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

  std::string findIMX519SensorEntity() {
    for (const auto& pair : entities) {
      if (pair.first.find("imx519") != std::string::npos) {
        return pair.first;
      }
    }
    return "";
  }

  std::string getDevicePath() const { return device_path; }
};

// Writer thread pool for batch writing
class WriterThreadPool {
 private:
  std::vector<std::thread> threads;
  std::queue<WriteTask> task_queue;
  std::mutex queue_mutex;
  std::condition_variable cv;
  std::atomic<bool> stop_flag{false};
  std::atomic<size_t> bytes_written{0};
  std::atomic<size_t> files_written{0};

  void worker() {
    while (true) {
      std::unique_lock<std::mutex> lock(queue_mutex);
      cv.wait(lock, [this] { return !task_queue.empty() || stop_flag; });

      if (stop_flag && task_queue.empty()) {
        break;
      }

      if (!task_queue.empty()) {
        WriteTask task = std::move(task_queue.front());
        task_queue.pop();
        lock.unlock();

        // Write with O_DIRECT for better SSD performance
        int fd = ::open(task.filename.c_str(),
                        O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
        if (fd >= 0) {
          // Align buffer for O_DIRECT
          size_t alignment = 4096;
          size_t aligned_size =
              (task.data.size() + alignment - 1) & ~(alignment - 1);
          void* aligned_buffer;
          if (posix_memalign(&aligned_buffer, alignment, aligned_size) == 0) {
            memcpy(aligned_buffer, task.data.data(), task.data.size());
            ssize_t written = write(fd, aligned_buffer, aligned_size);
            if (written > 0) {
              bytes_written += task.data.size();
              files_written++;
            }
            free(aligned_buffer);
          }
          close(fd);
        }
      }
    }
  }

 public:
  WriterThreadPool(size_t num_threads = 4) {
    for (size_t i = 0; i < num_threads; i++) {
      threads.emplace_back(&WriterThreadPool::worker, this);
    }
  }

  ~WriterThreadPool() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      stop_flag = true;
    }
    cv.notify_all();

    for (auto& thread : threads) {
      thread.join();
    }
  }

  void submit(WriteTask task) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      task_queue.push(std::move(task));
    }
    cv.notify_one();
  }

  size_t getQueueSize() {
    std::lock_guard<std::mutex> lock(queue_mutex);
    return task_queue.size();
  }

  size_t getBytesWritten() const { return bytes_written; }
  size_t getFilesWritten() const { return files_written; }
};

class V4L2Device {
 private:
  int fd = -1;
  std::vector<void*> buffer_starts;
  std::vector<size_t> buffer_lengths;
  size_t num_buffers = 0;
  bool streaming = false;
  uint32_t width = 0;
  uint32_t height = 0;
  size_t frame_size = 0;

 public:
  V4L2Device() = default;
  ~V4L2Device() {
    stopStreaming();
    for (size_t i = 0; i < buffer_starts.size(); i++) {
      if (buffer_starts[i] != MAP_FAILED && buffer_starts[i] != nullptr) {
        munmap(buffer_starts[i], buffer_lengths[i]);
      }
    }
    if (fd >= 0) close(fd);
  }

  bool open(const std::string& device_path) {
    fd = ::open(device_path.c_str(), O_RDWR);
    return fd >= 0;
  }

  bool setFormat(uint32_t w, uint32_t h, uint32_t pixelformat) {
    width = w;
    height = h;

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == 0) {
      // Calculate frame size (SRGGB10P is packed 10-bit, so 5 bytes per 4
      // pixels)
      frame_size = (width * height * 5) / 4;
      return true;
    }
    return false;
  }

  bool setupBuffers(size_t buffer_count = 4) {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
      return false;
    }

    num_buffers = req.count;
    buffer_starts.resize(num_buffers);
    buffer_lengths.resize(num_buffers);

    for (size_t i = 0; i < num_buffers; i++) {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
        std::cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << std::endl;
        return false;
      }

      buffer_lengths[i] = buf.length;
      std::cout << "buf.length = " << buf.length << std::endl;
      buffer_starts[i] =
          mmap(nullptr, buffer_lengths[i], PROT_READ | PROT_WRITE, MAP_SHARED,
               fd, buf.m.offset);

      if (buffer_starts[i] == MAP_FAILED) {
        std::cerr << "mmap failed: " << strerror(errno) << std::endl;
        return false;
      }

      // Queue the buffer
      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << std::endl;
        return false;
      }
    }

    std::cout << "Setup " << num_buffers
              << " buffers, frame size: " << frame_size << " bytes"
              << std::endl;
    return true;
  }

  bool startStreaming() {
    if (streaming) return true;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
      std::cerr << "Failed to start streaming: " << strerror(errno)
                << std::endl;
      return false;
    }
    streaming = true;
    return true;
  }

  bool stopStreaming() {
    if (!streaming) return true;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
      std::cerr << "Failed to stop streaming: " << strerror(errno) << std::endl;
      return false;
    }
    streaming = false;
    return true;
  }

  bool captureFrame(FrameData& frame_data) {
    // Dequeue buffer
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
      if (errno != EAGAIN) {
        std::cerr << "Failed to dequeue buffer: " << strerror(errno)
                  << std::endl;
      }
      return false;
    }

    // Copy frame data
    frame_data.data.resize(buf.bytesused);
    std::cout << "buf.bytesused" << buf.bytesused << std::endl;
    memcpy(frame_data.data.data(), buffer_starts[buf.index], buf.bytesused);
    frame_data.timestamp = std::chrono::steady_clock::now();

    // Re-queue buffer
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
      std::cerr << "Failed to re-queue buffer: " << strerror(errno)
                << std::endl;
      return false;
    }

    return true;
  }

  size_t getFrameSize() const { return frame_size; }
  size_t getNumBuffers() const { return num_buffers; }
};

struct CameraConfig {
  std::string sensor_entity;
  std::string csi2_entity = "csi2";
  std::string video_entity = "rp1-cfe-csi2_ch0";
  uint32_t width = 2328;
  uint32_t height = 1748;
  uint32_t crop_width = 4656;
  uint32_t crop_height = 3496;
  uint32_t crop_left = 8;
  uint32_t crop_top = 48;
};

struct PreparedCamera {
  MediaDevice* media_device;
  std::unique_ptr<V4L2Device> video_device;
  CameraConfig config;
  std::string output_prefix;
  size_t camera_index;
  std::vector<FrameData> captured_frames;
  std::atomic<size_t> frames_captured{0};
  std::atomic<size_t> frames_dropped{0};

  // Delete copy constructor and assignment
  PreparedCamera(const PreparedCamera&) = delete;
  PreparedCamera& operator=(const PreparedCamera&) = delete;

  // Custom move constructor
  PreparedCamera(PreparedCamera&& other) noexcept
      : media_device(other.media_device),
        video_device(std::move(other.video_device)),
        config(std::move(other.config)),
        output_prefix(std::move(other.output_prefix)),
        camera_index(other.camera_index),
        captured_frames(std::move(other.captured_frames)),
        frames_captured(other.frames_captured.load()),
        frames_dropped(other.frames_dropped.load()) {
    other.media_device = nullptr;
  }

  // Custom move assignment
  PreparedCamera& operator=(PreparedCamera&& other) noexcept {
    if (this != &other) {
      media_device = other.media_device;
      video_device = std::move(other.video_device);
      config = std::move(other.config);
      output_prefix = std::move(other.output_prefix);
      camera_index = other.camera_index;
      captured_frames = std::move(other.captured_frames);
      frames_captured = other.frames_captured.load();
      frames_dropped = other.frames_dropped.load();
      other.media_device = nullptr;
    }
    return *this;
  }

  // Default constructor
  PreparedCamera() = default;
};

bool configureCamera(MediaDevice& media, const CameraConfig& config,
                     std::unique_ptr<V4L2Device>& video_device,
                     size_t buffer_count) {
  // Set sensor crop
  if (!media.setCrop(config.sensor_entity, 0, config.crop_left, config.crop_top,
                     config.crop_width, config.crop_height)) {
    std::cerr << "Failed to set sensor crop" << std::endl;
    return false;
  }
  std::cout << "Set sensor crop to (" << config.crop_left << ","
            << config.crop_top << ")/" << config.crop_width << "x"
            << config.crop_height << std::endl;

  // Configure formats (use output width/height - sensor handles binning)
  if (!media.setFormat(config.sensor_entity, 0, config.width,
                       config.height, MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set sensor format to " << config.width << "x"
              << config.height << std::endl;
    return false;
  }
  std::cout << "Set sensor format to " << config.width << "x"
            << config.height << std::endl;

  if (!media.setFormat(config.csi2_entity, 0, config.width,
                       config.height, MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set CSI2 pad0 format" << std::endl;
    return false;
  }
  std::cout << "Set CSI2 pad0 format" << std::endl;

  if (!media.setFormat(config.csi2_entity, 4, config.width,
                       config.height, MEDIA_BUS_FMT_SRGGB10_1X10)) {
    std::cerr << "Failed to set CSI2 pad4 format" << std::endl;
    return false;
  }
  std::cout << "Set CSI2 pad4 format" << std::endl;

  // Enable links
  if (!media.setLink(config.sensor_entity, 0, config.csi2_entity, 0, true)) {
    std::cerr << "Failed to enable sensor to CSI2 link" << std::endl;
    return false;
  }
  std::cout << "Enabled sensor to CSI2 link" << std::endl;

  if (!media.setLink(config.csi2_entity, 4, config.video_entity, 0, true)) {
    std::cerr << "Failed to enable CSI2 to video link" << std::endl;
    return false;
  }
  std::cout << "Enabled CSI2 to video link" << std::endl;

  // Get video device path
  std::string video_path = media.getVideoDevicePath(config.video_entity);
  if (video_path.empty()) {
    std::cerr << "Failed to get video device path" << std::endl;
    return false;
  }
  std::cout << "Video device path: " << video_path << std::endl;

  // Open video device
  video_device = std::make_unique<V4L2Device>();
  if (!video_device->open(video_path)) {
    std::cerr << "Failed to open video device" << std::endl;
    return false;
  }

  if (!video_device->setFormat(config.width, config.height,
                               V4L2_PIX_FMT_SRGGB10P)) {
    std::cerr << "Failed to set video format" << std::endl;
    return false;
  }
  std::cout << "Set video format" << std::endl;

  if (!video_device->setupBuffers(buffer_count)) {
    std::cerr << "Failed to setup buffers" << std::endl;
    return false;
  }

  return true;
}

std::vector<MediaDevice> findAllCameras() {
  std::vector<MediaDevice> cameras;
  DIR* dir = opendir("/dev");
  if (!dir) return cameras;

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "media", 5) != 0) continue;

    std::string path = "/dev/" + std::string(entry->d_name);
    MediaDevice media;
    if (media.open(path)) {
      // Check if this media device has an IMX519 sensor
      std::string sensor_entity = media.findIMX519SensorEntity();
      if (!sensor_entity.empty()) {
        std::cout << "Found IMX519 camera at " << path
                  << " with sensor entity: " << sensor_entity << std::endl;
        cameras.push_back(std::move(media));
      }
    }
  }
  closedir(dir);

  return cameras;
}

// Capture thread function for continuous capture
void captureThread(PreparedCamera& cam, std::atomic<bool>& stop_capture,
                   CaptureMode mode, WriterThreadPool* writer_pool,
                   size_t batch_size, size_t max_frames) {
  std::vector<FrameData> batch;
  batch.reserve(batch_size);

  while (!stop_capture && cam.frames_captured < max_frames) {
    FrameData frame;
    frame.camera_index = cam.camera_index;
    frame.frame_number = cam.frames_captured;

    if (cam.video_device->captureFrame(frame)) {
      cam.frames_captured++;

      if (mode == CaptureMode::BUFFER_ALL) {
        // Store in memory
        cam.captured_frames.push_back(std::move(frame));
      } else {
        // Batch write mode
        batch.push_back(std::move(frame));

        if (batch.size() >= batch_size) {
          // Submit batch to writer pool
          for (auto& f : batch) {
            std::stringstream filename;
            filename << cam.output_prefix << "cam" << f.camera_index << "-"
                     << f.frame_number << ".raw";

            WriteTask task;
            task.filename = filename.str();
            task.data = std::move(f.data);
            writer_pool->submit(std::move(task));
          }
          batch.clear();

          // Check if writer queue is getting too large
          if (writer_pool->getQueueSize() > batch_size * 4) {
            // Drop frames if we can't keep up with writing
            cam.frames_dropped++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
      }
    }
  }

  // Write any remaining frames in batch mode
  if (mode == CaptureMode::BATCH_WRITE && !batch.empty()) {
    for (auto& f : batch) {
      std::stringstream filename;
      filename << cam.output_prefix << "cam" << f.camera_index << "-"
               << f.frame_number << ".raw";

      WriteTask task;
      task.filename = filename.str();
      task.data = std::move(f.data);
      writer_pool->submit(std::move(task));
    }
  }
}

void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [options]" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -m, --mode <buffer|batch>  Capture mode (default: batch)"
            << std::endl;
  std::cout << "  -f, --frames <N>           Number of frames to capture "
               "(default: 100)"
            << std::endl;
  std::cout
      << "  -b, --batch-size <N>       Batch size for writing (default: 10)"
      << std::endl;
  std::cout
      << "  -t, --threads <N>          Number of writer threads (default: 4)"
      << std::endl;
  std::cout
      << "  -p, --prefix <prefix>      Output file prefix (default: camera)"
      << std::endl;
  std::cout
      << "  -v, --v4l2-buffers <N>     Number of V4L2 buffers (default: 4)"
      << std::endl;
  std::cout << "  -h, --help                 Show this help message"
            << std::endl;
}

int main(int argc, char* argv[]) {
  // Parse command line arguments
  CaptureMode mode = CaptureMode::BATCH_WRITE;
  size_t num_frames = 100;
  size_t batch_size = 10;
  size_t writer_threads = 4;
  size_t v4l2_buffers = 4;
  std::string output_prefix = "camera";

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];

    if (arg == "-m" || arg == "--mode") {
      if (++i < argc) {
        std::string mode_str = argv[i];
        if (mode_str == "buffer") {
          mode = CaptureMode::BUFFER_ALL;
        } else if (mode_str == "batch") {
          mode = CaptureMode::BATCH_WRITE;
        } else {
          std::cerr << "Invalid mode: " << mode_str << std::endl;
          return 1;
        }
      }
    } else if (arg == "-f" || arg == "--frames") {
      if (++i < argc) num_frames = std::stoul(argv[i]);
    } else if (arg == "-b" || arg == "--batch-size") {
      if (++i < argc) batch_size = std::stoul(argv[i]);
    } else if (arg == "-t" || arg == "--threads") {
      if (++i < argc) writer_threads = std::stoul(argv[i]);
    } else if (arg == "-v" || arg == "--v4l2-buffers") {
      if (++i < argc) v4l2_buffers = std::stoul(argv[i]);
    } else if (arg == "-p" || arg == "--prefix") {
      if (++i < argc) output_prefix = argv[i];
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    }
  }

  // Find all cameras with IMX519 sensors
  std::vector<MediaDevice> cameras = findAllCameras();

  if (cameras.empty()) {
    std::cerr << "No IMX519 cameras found!" << std::endl;
    return 1;
  }

  std::cout << "\nFound " << cameras.size() << " IMX519 camera(s)" << std::endl;
  std::cout << "Capture mode: "
            << (mode == CaptureMode::BUFFER_ALL ? "Buffer All" : "Batch Write")
            << std::endl;
  std::cout << "Frames to capture: " << num_frames << " per camera"
            << std::endl;
  std::cout << "V4L2 buffers: " << v4l2_buffers << std::endl;

  if (mode == CaptureMode::BATCH_WRITE) {
    std::cout << "Batch size: " << batch_size << std::endl;
    std::cout << "Writer threads: " << writer_threads << std::endl;
  }

  // Calculate memory requirements
  size_t frame_size = (2328 * 1748 * 5) / 4;  // SRGGB10P format
  size_t total_memory_needed = frame_size * num_frames * cameras.size();
  std::cout << "Estimated memory usage: "
            << (total_memory_needed / (1024 * 1024)) << " MB" << std::endl;

  if (mode == CaptureMode::BUFFER_ALL &&
      total_memory_needed > 3ULL * 1024 * 1024 * 1024) {
    std::cerr << "Warning: Buffer-all mode may exceed available RAM!"
              << std::endl;
  }

  std::cout << "========================================" << std::endl;

  // Prepare all cameras
  std::vector<PreparedCamera> prepared_cameras;

  // Phase 1: Configure all cameras and prepare buffers
  std::cout << "\nPhase 1: Configuring all cameras..." << std::endl;
  std::cout << "========================================" << std::endl;

  for (size_t i = 0; i < cameras.size(); ++i) {
    std::cout << "\n=== Configuring camera " << i << " ===" << std::endl;

    MediaDevice& media = cameras[i];

    // Reset all links
    if (!media.reset()) {
      std::cerr << "Failed to reset media device " << i << std::endl;
      continue;
    }
    std::cout << "Reset media device " << i << std::endl;

    // List available entities for debugging
    std::cout << "\nCamera " << i << " entities:" << std::endl;
    media.listEntities();

    // Configure the camera
    CameraConfig config;
    config.sensor_entity = media.findIMX519SensorEntity();

    // Configure camera and prepare video device
    std::unique_ptr<V4L2Device> video_device;
    if (!configureCamera(media, config, video_device, v4l2_buffers)) {
      std::cerr << "Failed to configure camera " << i << std::endl;
      continue;
    }

    // Store prepared camera info
    PreparedCamera prep_cam;
    prep_cam.media_device = &media;
    prep_cam.video_device = std::move(video_device);
    prep_cam.config = config;
    prep_cam.output_prefix = output_prefix;
    prep_cam.camera_index = i;

    if (mode == CaptureMode::BUFFER_ALL) {
      prep_cam.captured_frames.reserve(num_frames);
    }

    prepared_cameras.push_back(std::move(prep_cam));

    std::cout << "Successfully configured camera " << i << " ("
              << media.getDevicePath() << ")" << std::endl;
  }

  if (prepared_cameras.empty()) {
    std::cerr << "No cameras were successfully configured!" << std::endl;
    return 1;
  }

  std::cout << "\nSuccessfully configured " << prepared_cameras.size()
            << " camera(s)" << std::endl;

  // Phase 2: Start streaming on all cameras
  std::cout << "\nPhase 2: Starting streaming on all cameras..." << std::endl;
  std::cout << "========================================" << std::endl;

  for (auto& prep_cam : prepared_cameras) {
    std::cout << "Starting streaming on camera " << prep_cam.camera_index
              << "..." << std::endl;
    if (!prep_cam.video_device->startStreaming()) {
      std::cerr << "Failed to start streaming on camera "
                << prep_cam.camera_index << std::endl;
      // Continue with other cameras
    }
  }

  std::cout << "\nAll cameras are now streaming and ready for capture"
            << std::endl;

  // Create writer thread pool for batch mode
  std::unique_ptr<WriterThreadPool> writer_pool;
  if (mode == CaptureMode::BATCH_WRITE) {
    writer_pool = std::make_unique<WriterThreadPool>(writer_threads);
  }

  // Phase 3: Capture frames from all cameras
  std::cout << "\nPhase 3: Starting multi-camera capture..." << std::endl;
  std::cout << "========================================" << std::endl;

  auto start_time = std::chrono::steady_clock::now();
  std::atomic<bool> stop_capture{false};
  std::vector<std::thread> capture_threads;

  // Start capture threads for each camera
  for (auto& prep_cam : prepared_cameras) {
    capture_threads.emplace_back(captureThread, std::ref(prep_cam),
                                 std::ref(stop_capture), mode,
                                 writer_pool.get(), batch_size, num_frames);
  }

  // Monitor progress
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    size_t total_captured = 0;
    size_t total_dropped = 0;
    bool all_done = true;

    for (const auto& cam : prepared_cameras) {
      total_captured += cam.frames_captured;
      total_dropped += cam.frames_dropped;
      if (cam.frames_captured < num_frames) {
        all_done = false;
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    float seconds =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count()) *
        1e-3;

    std::cout << "\rFrames captured: " << total_captured << "/"
              << (num_frames * prepared_cameras.size())
              << " | Dropped: " << total_dropped
              << " | Total FPS: " << std::fixed << std::setprecision(3)
              << (seconds > 0 ? total_captured / seconds : 0)
              << " | Time: " << std::fixed << std::setprecision(3) << seconds
              << "s" << std::flush;

    if (all_done) break;
  }

  std::cout << std::endl;

  // Signal threads to stop and wait for them
  stop_capture = true;
  for (auto& thread : capture_threads) {
    thread.join();
  }

  auto capture_end_time = std::chrono::steady_clock::now();

  // Phase 4: Save captured frames (for buffer-all mode)
  if (mode == CaptureMode::BUFFER_ALL) {
    std::cout << "\nPhase 4: Saving all buffered frames..." << std::endl;
    std::cout << "========================================" << std::endl;

    size_t total_written = 0;
    auto write_start_time = std::chrono::steady_clock::now();

    for (auto& cam : prepared_cameras) {
      std::cout << "Writing " << cam.captured_frames.size()
                << " frames from camera " << cam.camera_index << "..."
                << std::endl;

      for (const auto& frame : cam.captured_frames) {
        std::stringstream filename;
        filename << cam.output_prefix << "-" << frame.frame_number << ".raw";

        std::ofstream file(filename.str(), std::ios::binary);
        if (file) {
          file.write(reinterpret_cast<const char*>(frame.data.data()),
                     frame.data.size());
          total_written++;
        }
      }
    }

    auto write_end_time = std::chrono::steady_clock::now();
    auto write_duration = std::chrono::duration_cast<std::chrono::seconds>(
                              write_end_time - write_start_time)
                              .count();

    std::cout << "Written " << total_written << " frames in " << write_duration
              << " seconds" << std::endl;
    std::cout << "Write speed: "
              << (write_duration > 0 ? (total_written * frame_size) /
                                           (write_duration * 1024 * 1024)
                                     : 0)
              << " MB/s" << std::endl;
  }

  // Wait for writer pool to finish (batch mode)
  if (writer_pool) {
    std::cout << "\nWaiting for writer threads to complete..." << std::endl;
    while (writer_pool->getQueueSize() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Writer threads completed. Files written: "
              << writer_pool->getFilesWritten() << ", Bytes written: "
              << (writer_pool->getBytesWritten() / (1024 * 1024)) << " MB"
              << std::endl;
  }

  // Phase 5: Stop streaming on all cameras
  std::cout << "\nPhase 5: Stopping streaming on all cameras..." << std::endl;
  std::cout << "========================================" << std::endl;

  for (auto& prep_cam : prepared_cameras) {
    std::cout << "Stopping streaming on camera " << prep_cam.camera_index
              << "..." << std::endl;
    prep_cam.video_device->stopStreaming();
  }

  // Summary
  float total_duration =
      static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                             capture_end_time - start_time)
                             .count()) *
      1e-3;

  std::cout << "\n========================================" << std::endl;
  std::cout << "Capture Summary:" << std::endl;
  std::cout << "- Total cameras: " << prepared_cameras.size() << std::endl;
  std::cout << "- Capture duration: " << total_duration << " seconds"
            << std::endl;

  size_t total_frames = 0;
  size_t total_dropped = 0;

  for (const auto& cam : prepared_cameras) {
    total_frames += cam.frames_captured;
    total_dropped += cam.frames_dropped;
    std::cout << "- Camera " << cam.camera_index << ": " << cam.frames_captured
              << " frames captured, " << cam.frames_dropped << " dropped"
              << std::endl;
  }

  std::cout << "- Total frames captured: " << total_frames << std::endl;
  std::cout << "- Total frames dropped: " << total_dropped << std::endl;
  std::cout << "- Average FPS: " << std::fixed << std::setprecision(3)
            << (total_duration > 0 ? total_frames / total_duration : 0)
            << std::endl;
  std::cout << "- Data rate: " << std::fixed << std::setprecision(3)
            << (total_duration > 0 ? (total_frames * frame_size) /
                                         (total_duration * 1024 * 1024)
                                   : 0)
            << " MB/s" << std::endl;

  return 0;
}
