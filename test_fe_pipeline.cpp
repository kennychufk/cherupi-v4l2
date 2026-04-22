// Smoke test: drives Camera directly via libcamera, no websockets.
// Captures N frames and prints AWB gains from each.

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <libcamera/libcamera.h>

#include "camera.hpp"
#include "camera_manager.hpp"

LogLevel Logger::current_level = LogLevel::DEBUG;
std::mutex Logger::log_mutex;

int main(int argc, char** argv) {
  int num_frames = 30;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-n") && i + 1 < argc) num_frames = atoi(argv[++i]);
  }

  LOG_INFO("Test", "frames = " + std::to_string(num_frames));

  CameraManager mgr;
  size_t count = mgr.discoverCameras();
  if (count == 0) {
    LOG_ERROR("Test", "No IMX519 cameras found");
    return 1;
  }
  LOG_INFO("Test", "Found " + std::to_string(count) + " camera(s)");

  CameraConfig cfg;
  if (!mgr.configureAll(cfg)) {
    LOG_ERROR("Test", "configureAll failed");
    return 1;
  }

  if (!mgr.startAll()) {
    LOG_ERROR("Test", "startAll failed");
    return 1;
  }

  Camera* cam = mgr.getCamera(0);
  int received = 0;
  while (received < num_frames) {
    if (!cam->waitForNewFrame(std::chrono::milliseconds(1000))) {
      LOG_WARN("Test", "Timeout waiting for frame");
      continue;
    }
    FrameData frame;
    if (cam->getFrameForStreaming(frame)) {
      LOG_INFO("Test", "frame " + std::to_string(frame.frame_id) + " size=" +
                           std::to_string(frame.data.size()) +
                           " fmt=0x" +
                           [&]{ std::ostringstream s; s << std::hex << frame.pixel_format; return s.str(); }());
      cam->releaseStreamingFrame();
      ++received;
    }
  }

  mgr.stopAll();
  LOG_INFO("Test", "Done");
  return 0;
}
