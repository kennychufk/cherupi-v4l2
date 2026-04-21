#pragma once

#include <linux/videodev2.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "types.hpp"

// Thin V4L2 wrapper. Supports VIDEO_CAPTURE (existing grey-world path) plus
// META_CAPTURE (PiSP Frontend stats, `/dev/video6`) and META_OUTPUT
// (PiSP Frontend config, `/dev/video7`) for the Stage 3 AWB pipeline. All
// buffer types use MMAP.
class V4L2Device {
 public:
  V4L2Device() = default;
  ~V4L2Device();

  bool open(const std::string& device_path);

  // Video capture (SRGGB10P / SRGGB16 / ...).
  bool setFormat(uint32_t w, uint32_t h, uint32_t pixelformat);
  bool captureFrame(FrameData& frame_data);

  // Metadata plane (META_CAPTURE or META_OUTPUT). `size` is the payload
  // size in bytes (e.g. sizeof(pisp_statistics), sizeof(pisp_fe_config)).
  bool setMetaFormat(uint32_t dataformat, uint32_t size, bool output);

  // Common buffer / streaming control.
  bool setupBuffers(size_t buffer_count = 4);
  bool startStreaming();
  bool stopStreaming();

  // META_OUTPUT: copy `size` bytes from `cfg` into an available buffer and
  // QBUF it. Dequeues any previously-consumed buffer first so the pool
  // keeps rotating.
  bool queueMetaOutputBuffer(const void* cfg, size_t size);

  // META_CAPTURE: dequeue a stats buffer, copy into `out` (up to `out_size`),
  // return actual bytes used via `used`, and re-queue.
  bool dequeueMetaCaptureBuffer(void* out, size_t out_size, size_t& used);

  int getFd() const { return fd; }
  size_t getFrameSize() const { return frame_size; }
  size_t getBytesPerLine() const { return bytes_per_line; }
  size_t getNumBuffers() const { return num_buffers; }
  uint32_t getWidth() const { return width; }
  uint32_t getHeight() const { return height; }
  v4l2_buf_type getBufType() const { return buf_type; }

 private:
  int fd = -1;
  std::vector<void*> buffer_starts;
  std::vector<size_t> buffer_lengths;
  size_t num_buffers = 0;
  bool streaming = false;

  // Video-capture geometry (ignored for META types).
  uint32_t width = 0;
  uint32_t height = 0;
  size_t frame_size = 0;
  size_t bytes_per_line = 0;

  // Buffer type in use — selected by setFormat / setMetaFormat.
  v4l2_buf_type buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // META payload size (bytes) when buf_type is a meta type.
  size_t meta_size = 0;
};
