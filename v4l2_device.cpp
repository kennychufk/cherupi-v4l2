#include "v4l2_device.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

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

bool V4L2Device::setupBuffers(size_t buffer_count) {
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
    buffer_starts[i] = mmap(nullptr, buffer_lengths[i], PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, buf.m.offset);

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

  return true;
}

bool V4L2Device::startStreaming() {
  if (streaming) return true;

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "Failed to start streaming: " << strerror(errno) << std::endl;
    return false;
  }
  streaming = true;
  return true;
}

bool V4L2Device::stopStreaming() {
  if (!streaming) return true;

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
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
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
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
