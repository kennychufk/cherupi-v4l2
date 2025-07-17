#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
} __attribute__((packed));

struct ChunkData {
  uint32_t magic = 0x43484E4B;  // 'CHNK' in hex - magic for chunk data
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
};

// Camera configuration
struct CameraConfig {
  std::string sensor_entity;
  std::string csi2_entity = "csi2";
  std::string video_entity = "rp1-cfe-csi2_ch0";
  uint32_t width = 1456;
  uint32_t height = 1088;
  uint32_t crop_width = 1456;
  uint32_t crop_height = 1088;
  uint32_t crop_left = 0;
  uint32_t crop_top = 0;
};

// Frame saving modes
enum class SaveMode { NONE, BUFFER, BATCH };

// Camera state
enum class CameraState { IDLE, CONFIGURED, RUNNING, ERROR };

// Save configuration
struct SaveConfig {
  SaveMode mode = SaveMode::NONE;
  std::string prefix = "camera";
  size_t batch_size = 10;
  size_t writer_threads = 4;
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

// Response types from server
constexpr const char* TYPE_DISCOVERY = "discovery";
constexpr const char* TYPE_STATUS = "status";
constexpr const char* TYPE_FRAME = "frame";
constexpr const char* TYPE_ERROR = "error";
}  // namespace Protocol
