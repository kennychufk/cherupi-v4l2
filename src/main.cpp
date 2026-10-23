#include <malloc.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "websocket_server.hpp"

// Initialize static members
LogLevel Logger::current_level = LogLevel::INFO;
std::mutex Logger::log_mutex;

std::unique_ptr<WebSocketServer> server;

void signalHandler(int signum) {
  LOG_INFO("Main",
           "Received signal " + std::to_string(signum) + ", shutting down...");
  if (server) {
    server->stop();
  }
  exit(0);
}

// Point libcamera's Pi 5 pipeline handler at our own configuration, unless the
// caller already chose one with LIBCAMERA_RPI_CONFIG_FILE. Must run before the
// libcamera CameraManager starts, which is when the file is read.
//
// camera_timeout_value_ms: libcamera's frontend dequeue timeout (default
// max(1 s, 5 frame durations)) is checked on its own event-loop thread, after
// that thread's handlers have run — including our requestCompleted callback.
// One callback that stalls past the timeout (a page fault blocking in reclaim
// on a 1 GB Pi under memory pressure; measured up to 2.2 s) has the timer
// expire before the buffer already waiting is seen: "Camera frontend has
// timed out!", and capture never resumes. 10 s rides out such stalls; our own
// capture watchdog still reports a silent camera after 3 s. The cost is that a
// frame longer than 10 s (exposure) would now trip it.
//
// disable_hdr: multi-frame HDR (unused here) allocates two extra full-size
// Bayer buffers — 2 x 24.7 MB at 4056x3040 with the 16-bit raw stream.
// (disable_tdn is not set: with the raw stream configured, the camera then
// delivers no frame at all.)
void configureLibcameraPipeline() {
  if (const char* existing = getenv("LIBCAMERA_RPI_CONFIG_FILE")) {
    LOG_INFO("Main", std::string("libcamera pipeline config from "
                                 "LIBCAMERA_RPI_CONFIG_FILE: ") + existing);
    return;
  }
  const std::string path =
      "/tmp/cherupi-v4l2-pisp-" + std::to_string(getuid()) + ".yaml";
  std::ofstream out(path, std::ios::trunc);
  out << "{\n"
         "  \"version\": 1.0,\n"
         "  \"target\": \"pisp\",\n"
         "  \"pipeline_handler\": {\n"
         "    \"camera_timeout_value_ms\": 10000,\n"
         "    \"disable_hdr\": true\n"
         "  }\n"
         "}\n";
  out.close();
  if (!out) {
    LOG_WARN("Main", "Could not write " + path +
                         "; libcamera keeps its default pipeline config");
    return;
  }
  setenv("LIBCAMERA_RPI_CONFIG_FILE", path.c_str(), 1);
  LOG_INFO("Main", "libcamera pipeline config: " + path +
                       " (camera timeout 10 s, HDR off)");
}

void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [OPTIONS]\n"
            << "Options:\n"
            << "  --debug    Enable debug logging\n"
            << "  --help     Show this help message\n";
}

int main(int argc, char* argv[]) {
  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--debug") == 0) {
      Logger::setLevel(LogLevel::DEBUG);
      std::cout << "Debug logging enabled" << std::endl;
    } else if (strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  // Serve every allocation of 1 MiB or more with its own mmap, and give it
  // back to the kernel when freed. glibc's default threshold adapts upward
  // after the first large free, after which multi-MB frame buffers (YUV
  // frames, stacked images) come from per-thread heaps that rarely shrink:
  // measured on a 1 GB Pi 5, the server sat at 516 MB resident while idle,
  // with half a gigabyte in swap — the swap thrash that ends in libcamera's
  // frontend watchdog (see CLAUDE.md, Resource Guards).
  mallopt(M_MMAP_THRESHOLD, 1 << 20);

  // Register signal handlers
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  LOG_INFO("Main", "IMX519 Camera WebSocket Server");
  LOG_INFO("Main", "==============================");

  configureLibcameraPipeline();

  try {
    server = std::make_unique<WebSocketServer>();
    server->run();
  } catch (const std::exception& e) {
    LOG_ERROR("Main", "Fatal error: " + std::string(e.what()));
    return 1;
  }

  return 0;
}
