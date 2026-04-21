#include "v4l2_device.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

V4L2Device::~V4L2Device() {
  stopStreaming();
  for (size_t i = 0; i < buffer_starts.size(); i++) {
    if (buffer_starts[i] != MAP_FAILED && buffer_starts[i] != nullptr) {
      munmap(buffer_starts[i], buffer_lengths[i]);
    }
  }
  if (fd >= 0) close(fd);
}

bool V4L2Device::open(const std::string& device_path) {
  fd = ::open(device_path.c_str(), O_RDWR);
  return fd >= 0;
}

bool V4L2Device::setFormat(uint32_t w, uint32_t h, uint32_t pixelformat) {
  width = w;
  height = h;
  buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixelformat;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == 0) {
    // Calculate frame size (SRGGB10P is packed 10-bit, so 5 bytes per 4 pixels)
    frame_size = (width * height * 5) / 4;
    // For SRGGB10P, bytes per line is width * 5 / 4
    bytes_per_line = (width * 5) / 4;

    // Get the actual format to confirm bytes per line
    struct v4l2_format get_fmt;
    memset(&get_fmt, 0, sizeof(get_fmt));
    get_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &get_fmt) == 0) {
      bytes_per_line = get_fmt.fmt.pix.bytesperline;
    }

    return true;
  }
  return false;
}

bool V4L2Device::setMetaFormat(uint32_t dataformat, uint32_t size, bool output) {
  buf_type =
      output ? V4L2_BUF_TYPE_META_OUTPUT : V4L2_BUF_TYPE_META_CAPTURE;

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = buf_type;
  fmt.fmt.meta.dataformat = dataformat;
  fmt.fmt.meta.buffersize = size;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    std::cerr << "VIDIOC_S_FMT (meta) failed: " << strerror(errno) << std::endl;
    return false;
  }

  // Read back the negotiated size.
  struct v4l2_format get_fmt;
  memset(&get_fmt, 0, sizeof(get_fmt));
  get_fmt.type = buf_type;
  if (ioctl(fd, VIDIOC_G_FMT, &get_fmt) == 0) {
    meta_size = get_fmt.fmt.meta.buffersize;
  } else {
    meta_size = size;
  }
  return true;
}

bool V4L2Device::setupBuffers(size_t buffer_count) {
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = buffer_count;
  req.type = buf_type;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
    return false;
  }

  num_buffers = req.count;
  buffer_starts.resize(num_buffers);
  buffer_lengths.resize(num_buffers);

  const bool is_output = (buf_type == V4L2_BUF_TYPE_META_OUTPUT);

  for (size_t i = 0; i < num_buffers; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = buf_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      std::cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << std::endl;
      return false;
    }

    buffer_lengths[i] = buf.length;
    buffer_starts[i] = mmap(nullptr, buffer_lengths[i], PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, buf.m.offset);

    if (buffer_starts[i] == MAP_FAILED) {
      std::cerr << "mmap failed: " << strerror(errno) << std::endl;
      return false;
    }

    // For OUTPUT queues we want userland to fill buffers before queueing, so
    // leave them unqueued here; the first queue happens via
    // queueMetaOutputBuffer(). For CAPTURE queues (video + meta-capture) we
    // prime the kernel-side pool up front.
    if (!is_output) {
      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << std::endl;
        return false;
      }
    }
  }

  return true;
}

bool V4L2Device::startStreaming() {
  if (streaming) return true;

  enum v4l2_buf_type type = buf_type;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "Failed to start streaming: " << strerror(errno) << std::endl;
    return false;
  }
  streaming = true;
  return true;
}

bool V4L2Device::stopStreaming() {
  if (!streaming) return true;

  enum v4l2_buf_type type = buf_type;
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    std::cerr << "Failed to stop streaming: " << strerror(errno) << std::endl;
    return false;
  }
  streaming = false;
  return true;
}

bool V4L2Device::captureFrame(FrameData& frame_data) {
  // Dequeue buffer
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = buf_type;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
    if (errno != EAGAIN) {
      std::cerr << "Failed to dequeue buffer: " << strerror(errno) << std::endl;
    }
    return false;
  }

  // Copy frame data
  frame_data.data.resize(buf.bytesused);
  memcpy(frame_data.data.data(), buffer_starts[buf.index], buf.bytesused);
  frame_data.width = width;
  frame_data.height = height;
  frame_data.bytes_per_line = bytes_per_line;

  // Re-queue buffer
  if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    std::cerr << "Failed to re-queue buffer: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool V4L2Device::queueMetaOutputBuffer(const void* cfg, size_t size) {
  if (buf_type != V4L2_BUF_TYPE_META_OUTPUT) return false;
  if (size > meta_size) return false;

  // Try to reap a completed buffer first; on the first call all buffers are
  // free so DQBUF returns EAGAIN and we fall through to picking index 0.
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = buf_type;
  buf.memory = V4L2_MEMORY_MMAP;

  static thread_local size_t rotate_index = 0;
  int idx;
  if (streaming && ioctl(fd, VIDIOC_DQBUF, &buf) == 0) {
    idx = buf.index;
  } else {
    idx = rotate_index;
    rotate_index = (rotate_index + 1) % num_buffers;
    memset(&buf, 0, sizeof(buf));
    buf.type = buf_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = idx;
  }

  memcpy(buffer_starts[idx], cfg, size);
  buf.bytesused = size;
  buf.m.planes = nullptr;

  if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    std::cerr << "Meta QBUF failed: " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool V4L2Device::dequeueMetaCaptureBuffer(void* out, size_t out_size,
                                          size_t& used) {
  if (buf_type != V4L2_BUF_TYPE_META_CAPTURE) return false;

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = buf_type;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
    if (errno != EAGAIN) {
      std::cerr << "Meta DQBUF failed: " << strerror(errno) << std::endl;
    }
    return false;
  }

  used = buf.bytesused;
  size_t copy = std::min(used, out_size);
  memcpy(out, buffer_starts[buf.index], copy);

  if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    std::cerr << "Meta QBUF (re-queue) failed: " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}
