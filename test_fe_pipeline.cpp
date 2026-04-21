// Standalone smoke test for the Stage 3 Bayes FE pipeline. Drives Camera
// directly without websockets/streaming. Pass `--grey` to force the old
// grey-world single-device path for a sanity check.

#include <dirent.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "camera.hpp"
#include "media_device.hpp"

LogLevel Logger::current_level = LogLevel::DEBUG;
std::mutex Logger::log_mutex;

int main(int argc, char** argv) {
  bool use_bayes = true;
  int num_frames = 30;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--grey")) use_bayes = false;
    else if (!strcmp(argv[i], "-n") && i + 1 < argc) num_frames = atoi(argv[++i]);
  }

  LOG_INFO("Test", std::string("Mode = ") + (use_bayes ? "BAYES" : "GREY_WORLD") +
                       ", frames = " + std::to_string(num_frames));

  // Find an IMX519 via /dev/media*.
  MediaDevice media;
  std::string sensor;
  DIR* d = opendir("/dev");
  struct dirent* ent;
  while (d && (ent = readdir(d))) {
    if (strncmp(ent->d_name, "media", 5)) continue;
    MediaDevice m;
    if (!m.open(std::string("/dev/") + ent->d_name)) continue;
    std::string s = m.findIMX519SensorEntity();
    if (!s.empty()) {
      media = std::move(m);
      sensor = s;
      LOG_INFO("Test", "Using " + std::string("/dev/") + ent->d_name +
                           " sensor=" + sensor);
      break;
    }
  }
  if (d) closedir(d);
  if (sensor.empty()) {
    LOG_ERROR("Test", "No IMX519 camera found");
    return 1;
  }

  CameraConfig cfg;
  cfg.sensor_entity = sensor;
  cfg.awb.mode = use_bayes ? AwbMode::BAYES : AwbMode::GREY_WORLD;

  Camera cam(0, &media, cfg);
  std::atomic<int> captured{0};
  cam.onFrameCaptured = [&](const FrameData& f) {
    int n = ++captured;
    if (n <= 3 || n == num_frames) {
      LOG_INFO("Test", "frame " + std::to_string(n) +
                           " size=" + std::to_string(f.data.size()) +
                           " gain_r=" + std::to_string(f.awb_gain_r) +
                           " gain_b=" + std::to_string(f.awb_gain_b));
    }
  };

  if (!cam.configure()) {
    LOG_ERROR("Test", "configure() failed");
    return 2;
  }
  if (!cam.start()) {
    LOG_ERROR("Test", "start() failed");
    return 3;
  }

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (captured < num_frames &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  cam.stop();
  int got = captured.load();
  LOG_INFO("Test", "Done. Captured " + std::to_string(got) + "/" +
                       std::to_string(num_frames));
  return got >= num_frames ? 0 : 10;
}
