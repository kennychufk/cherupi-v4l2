#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

// Logging levels
enum class LogLevel { ERROR = 0, WARN = 1, INFO = 2, DEBUG = 3 };

// Global logger class
class Logger {
 private:
  static LogLevel current_level;
  static std::mutex log_mutex;

  static std::string getLevelString(LogLevel level) {
    switch (level) {
      case LogLevel::ERROR:
        return "ERROR";
      case LogLevel::WARN:
        return "WARN ";
      case LogLevel::INFO:
        return "INFO ";
      case LogLevel::DEBUG:
        return "DEBUG";
      default:
        return "UNKNOWN";
    }
  }

  static std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
  }

 public:
  static void setLevel(LogLevel level) { current_level = level; }
  static LogLevel getLevel() { return current_level; }

  static void log(LogLevel level, const std::string& component,
                  const std::string& message) {
    if (level > current_level) return;

    std::lock_guard<std::mutex> lock(log_mutex);
    auto& stream = (level == LogLevel::ERROR) ? std::cerr : std::cout;
    stream << "[" << getTimestamp() << "] [" << getLevelString(level) << "] ["
           << component << "] " << message << std::endl;
  }
};

// Convenience macros for logging
#define LOG_ERROR(component, msg) Logger::log(LogLevel::ERROR, component, msg)
#define LOG_WARN(component, msg) Logger::log(LogLevel::WARN, component, msg)
#define LOG_INFO(component, msg) Logger::log(LogLevel::INFO, component, msg)
#define LOG_DEBUG(component, msg) Logger::log(LogLevel::DEBUG, component, msg)

// Frame metadata structure (packed for network transmission)
struct FrameHeader {
  uint32_t frame_id;
  uint32_t camera_id;
  uint32_t bytes_per_line;
  uint32_t width;
  uint32_t height;
} __attribute__((packed));

// Chunked transfer structures
struct ChunkStartMarker {
  uint32_t magic = 0x4348554E;  // 'CHUN' in hex
  uint32_t version = 1;
} __attribute__((packed));

struct ChunkHeader {
  uint32_t frame_uuid;
  uint32_t frame_id;
  uint32_t camera_id;
  uint32_t total_chunks;
  uint32_t total_size;
  uint32_t bytes_per_line;
  uint32_t width;
  uint32_t height;
  uint32_t frames_saved;
  float awb_gain_r;
  float awb_gain_b;
  float awb_cct;
} __attribute__((packed));

struct ChunkData {
  uint32_t magic = 0x43484E4B;  // 'CHNK' in hex
  uint32_t frame_uuid;
  uint32_t chunk_index;
  uint32_t chunk_size;
  // Actual data follows
} __attribute__((packed));

// Frame data structure for internal use
struct FrameData {
  std::vector<uint8_t> data;
  uint32_t frame_id;
  uint32_t camera_id;
  uint32_t width;
  uint32_t height;
  uint32_t bytes_per_line;
  float awb_gain_r = 1.0f;
  float awb_gain_b = 1.0f;
  float awb_cct = 0.0f;

  // Deep copy constructor
  FrameData() = default;
  FrameData(const FrameData& other)
      : data(other.data),
        frame_id(other.frame_id),
        camera_id(other.camera_id),
        width(other.width),
        height(other.height),
        bytes_per_line(other.bytes_per_line),
        awb_gain_r(other.awb_gain_r),
        awb_gain_b(other.awb_gain_b),
        awb_cct(other.awb_cct) {}

  FrameData& operator=(const FrameData& other) {
    if (this != &other) {
      data = other.data;
      frame_id = other.frame_id;
      camera_id = other.camera_id;
      width = other.width;
      height = other.height;
      bytes_per_line = other.bytes_per_line;
      awb_gain_r = other.awb_gain_r;
      awb_gain_b = other.awb_gain_b;
      awb_cct = other.awb_cct;
    }
    return *this;
  }
};

// AWB configuration. The mode selects between the existing CPU grey-world
// estimator and the Bayesian estimator driven by PiSP Frontend stats.
// Stage 1 scaffolding: Bayes mode is wired through the type system but the
// estimator itself is still a stub — GREY_WORLD remains the default.
enum class AwbMode {
  GREY_WORLD,  // Subsampled Minkowski-norm from SRGGB10P (CPU, current prod).
  BAYES,       // Libcamera-style Bayesian from pisp_awb_statistics.
};

struct AwbConfig {
  bool enabled = true;
  AwbMode mode = AwbMode::GREY_WORLD;
  int interval = 10;       // Grey-world: recompute gains every N frames.
  int period = 3;          // Bayes: run estimator every N FE stats frames.
  float speed = 0.05f;     // IIR smoothing factor (post-warmup).
  int warmup_frames = 10;  // Fast convergence for first N frames.
};

// Camera configuration
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
  AwbConfig awb;
};

// Frame saving modes
enum class SaveMode { NONE, BUFFER, BATCH, CHECKERBOARD };

// Camera state
enum class CameraState { IDLE, CONFIGURED, RUNNING, ERROR };

// Save configuration
struct SaveConfig {
  SaveMode mode = SaveMode::NONE;
  std::string output_dir = ".";
  bool prepend_timestamp_to_dir = false;
  size_t batch_size = 10;
  size_t writer_threads = 4;

  // Checkerboard detection parameters
  int checkerboard_rows = 8;
  int checkerboard_cols = 11;
  bool checkerboard_full_res_detection = false;
  int checkerboard_num_threads = 4;
};

// Message types for WebSocket protocol
namespace Protocol {
// Commands from client
constexpr const char* CMD_DISCOVER = "discover";
constexpr const char* CMD_CONFIGURE = "configure";
constexpr const char* CMD_SET_SAVE_MODE = "set_save_mode";
constexpr const char* CMD_START_CAMERAS = "start_cameras";
constexpr const char* CMD_START_STREAM = "start_stream";
constexpr const char* CMD_STOP_STREAM = "stop_stream";
constexpr const char* CMD_STOP_CAMERAS = "stop_cameras";
constexpr const char* CMD_RESET_FRAME_COUNTS = "reset_frame_counts";
constexpr const char* CMD_SET_HEADER_ONLY = "set_header_only";

// Response types from server
constexpr const char* TYPE_DISCOVERY = "discovery";
constexpr const char* TYPE_STATUS = "status";
constexpr const char* TYPE_FRAME = "frame";
constexpr const char* TYPE_ERROR = "error";
}  // namespace Protocol
