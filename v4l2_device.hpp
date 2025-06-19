#pragma once

#include <string>
#include <vector>

#include "types.hpp"

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
  size_t bytes_per_line = 0;

 public:
  V4L2Device() = default;
  ~V4L2Device();

  bool open(const std::string& device_path);
  bool setFormat(uint32_t w, uint32_t h, uint32_t pixelformat);
  bool setupBuffers(size_t buffer_count = 4);
  bool startStreaming();
  bool stopStreaming();
  bool captureFrame(FrameData& frame_data);

  size_t getFrameSize() const { return frame_size; }
  size_t getBytesPerLine() const { return bytes_per_line; }
  size_t getNumBuffers() const { return num_buffers; }
  uint32_t getWidth() const { return width; }
  uint32_t getHeight() const { return height; }
};
