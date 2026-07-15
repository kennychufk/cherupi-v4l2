#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
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

// Chunked transfer structures
struct ChunkStartMarker {
  uint32_t magic = 0x4348554E;  // 'CHUN' in hex
  uint32_t version = 6;
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
  uint32_t pixel_format;  // FourCC (e.g. YUV420 = V4L2_PIX_FMT_YUV420)
  uint32_t frames_saved;
  // v3 additions: hardware-level frame timing from libcamera metadata.
  // timestamp_us: monotonic hardware capture timestamp in µs (from
  //   FrameMetadata::timestamp / 1000). Clients can diff consecutive
  //   timestamps to compute actual inter-frame intervals (real FPS).
  // frame_duration_us: actual frame duration reported by the ISP/IPA for this
  //   frame (libcamera FrameDuration metadata, µs). Reflects real per-frame
  //   cadence including any ISP scheduling overhead (e.g. multi-camera sharing).
  //   0 if the metadata was not available.
  uint64_t timestamp_us;
  uint32_t frame_duration_us;
  // v4 additions: variable-size detection block. When the save mode is a
  // detector mode (`checkerboard`/`checkerboard2x2` from v4, `aruco`/`aruco2x2`
  // from v6) and detection found at least one board/marker, the same WS message
  // that carries this ChunkHeader also carries the block immediately after.
  // `num_corner_sets` counts the records in the block (one per detected board,
  // or one per detected ArUco marker); `corner_block_size` is its total byte
  // length. The per-record layout depends on `detection_kind` (see below):
  // `CornerSetHeader` records for checkerboard, `MarkerSetHeader` records for
  // aruco. Coordinates are in full-frame Y-plane pixels regardless of the
  // saver's `*_full_res_detection` setting or 2x2 quadrant split. Both fields
  // are 0 when no block follows.
  uint32_t corner_block_size;
  uint16_t num_corner_sets;
  uint16_t reserved;
  // v5 additions: per-frame focus metadata reported by the libcamera IPA for
  // this exact frame (EXIF-style), valid in manual, auto and continuous AF.
  // lens_position: focus distance the IPA actually applied, in dioptres
  //   (reciprocal metres; 0 = infinity). NaN when libcamera reported no
  //   LensPosition for the frame (e.g. sensor module without a focuser).
  // af_state: libcamera AfState enum (0=Idle, 1=Scanning, 2=Focused,
  //   3=Failed). 0xFF when no AfState was reported for the frame.
  float lens_position;
  uint8_t af_state;
  // v6 addition: classifies the detection block that follows this header (and
  // therefore how to parse each of its `num_corner_sets` records). 0 = none
  // (no detector ran; no block), 1 = checkerboard (records are CornerSetHeader),
  // 2 = aruco (records are MarkerSetHeader). See DetectionKind. Carried in a
  // byte that was `reserved2[0]` (always 0) before v6, so ChunkHeader stays
  // 68 bytes.
  uint8_t detection_kind;
  uint8_t reserved2[2];
} __attribute__((packed));

// Classifies the detection block appended to a ChunkHeader (its `detection_kind`
// field) and, equivalently, which per-record header layout the block uses.
enum class DetectionKind : uint8_t {
  NONE = 0,          // no detector ran; no block follows
  CHECKERBOARD = 1,  // records are CornerSetHeader
  ARUCO = 2,         // records are MarkerSetHeader
};

// Per corner-set header inside a checkerboard CornerBlock (v4+;
// detection_kind == CHECKERBOARD). For `checkerboard` mode the single emitted
// set has set_id=0. For `checkerboard2x2`, one set is emitted per detecting
// quadrant with set_id = row*2 + col (0..3, where row and col are 0 for
// top/left and 1 for bottom/right).
struct CornerSetHeader {
  uint8_t set_id;
  // bit 0: coordinates are in full-frame Y-plane pixel space (always 1 in v4)
  uint8_t flags;
  uint16_t num_corners;  // = checkerboard_rows × checkerboard_cols
  // followed by num_corners × { float x; float y; }  (8 bytes each)
} __attribute__((packed));

// Per-marker header inside an aruco MarkerBlock (v6+; detection_kind == ARUCO).
// One record is emitted per detected ArUco/AprilTag marker. For `aruco` mode
// every record has quadrant=0 (whole frame); for `aruco2x2`, quadrant =
// row*2 + col (0..3) identifies the sub-frame the marker was detected in (the
// same marker may appear in more than one quadrant record). The 4 corners
// follow in the dictionary's canonical order (clockwise from the marker's
// top-left).
struct MarkerSetHeader {
  int32_t marker_id;  // ArUco/AprilTag dictionary id (DICT_APRILTAG_16h5)
  uint8_t quadrant;   // 0 (aruco) or row*2+col (aruco2x2, 0..3)
  // bit 0: coordinates are in full-frame Y-plane pixel space (always 1 in v6)
  uint8_t flags;
  uint16_t num_corners;  // = 4
  // followed by num_corners × { float x; float y; }  (8 bytes each)
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
  uint32_t frame_id = 0;
  uint32_t camera_id = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t bytes_per_line = 0;
  uint32_t pixel_format = 0;  // FourCC
  uint64_t timestamp_us = 0;      // hardware capture timestamp (µs)
  uint32_t frame_duration_us = 0; // actual frame duration from ISP metadata (µs)
  // Per-frame focus metadata from the libcamera IPA (see ChunkHeader v5).
  // NaN / 0xFF mean "not reported for this frame".
  float lens_position = std::numeric_limits<float>::quiet_NaN();  // dioptres
  uint8_t af_state = 0xFF;  // libcamera AfState enum, 0xFF = unavailable

  FrameData() = default;
  FrameData(const FrameData& other)
      : data(other.data),
        frame_id(other.frame_id),
        camera_id(other.camera_id),
        width(other.width),
        height(other.height),
        bytes_per_line(other.bytes_per_line),
        pixel_format(other.pixel_format),
        timestamp_us(other.timestamp_us),
        frame_duration_us(other.frame_duration_us),
        lens_position(other.lens_position),
        af_state(other.af_state) {}

  FrameData& operator=(const FrameData& other) {
    if (this != &other) {
      data = other.data;
      frame_id = other.frame_id;
      camera_id = other.camera_id;
      width = other.width;
      height = other.height;
      bytes_per_line = other.bytes_per_line;
      pixel_format = other.pixel_format;
      timestamp_us = other.timestamp_us;
      frame_duration_us = other.frame_duration_us;
      lens_position = other.lens_position;
      af_state = other.af_state;
    }
    return *this;
  }
};

// Camera configuration
struct CameraConfig {
  uint32_t width = 2328;
  uint32_t height = 1748;
};

// Per-frame processing modes. Each mode names the pipeline applied to a
// captured frame; whether processed frames are also written to disk is a
// separate axis controlled by ProcessConfig::save_frames.
enum class ProcessMode {
  NONE,
  BUFFER,
  BATCH,
  CHECKERBOARD,
  CHECKERBOARD2X2,
  ARUCO,
  ARUCO2X2
};

// Map a process mode to the DetectionKind its detector emits (NONE for the
// non-detector modes). Used to stamp ChunkHeader.detection_kind and to select
// the block serialization format.
inline DetectionKind detectionKindForMode(ProcessMode mode) {
  switch (mode) {
    case ProcessMode::CHECKERBOARD:
    case ProcessMode::CHECKERBOARD2X2:
      return DetectionKind::CHECKERBOARD;
    case ProcessMode::ARUCO:
    case ProcessMode::ARUCO2X2:
      return DetectionKind::ARUCO;
    default:
      return DetectionKind::NONE;
  }
}

// True for the four modes whose frames flow through the async best-effort
// detection worker (checkerboard/aruco, whole-frame or 2x2). These modes stream
// only detector-processed frames (carrying the detected corners/markers as a
// side-product) and, when `save_frames` is set, also write to disk only the
// frames that produced a detection.
inline bool isDetectorMode(ProcessMode mode) {
  return mode == ProcessMode::CHECKERBOARD || mode == ProcessMode::CHECKERBOARD2X2 ||
         mode == ProcessMode::ARUCO || mode == ProcessMode::ARUCO2X2;
}

// Camera state
enum class CameraState { IDLE, CONFIGURED, RUNNING, ERROR };

// Per-frame processing configuration.
struct ProcessConfig {
  ProcessMode mode = ProcessMode::NONE;

  // Whether processed frames are written to disk. Default true for backward
  // compatibility. Setting it false decouples detection from persistence: in a
  // detector mode (checkerboard/aruco, whole-frame or 2x2) the detector still
  // runs and still streams its side-products (corners/markers) to the client,
  // but no frames are written — the writer pool is not even started, and the
  // per-detection full-frame copy into the write queue is skipped. For the
  // pure-save modes (buffer/batch) false simply produces no output (there is no
  // side-product), so it is not a useful combination.
  bool save_frames = true;

  std::string output_dir = ".";
  bool prepend_timestamp_to_dir = false;
  size_t batch_size = 10;
  size_t writer_threads = 4;

  // Checkerboard detection parameters
  int checkerboard_rows = 8;
  int checkerboard_cols = 11;
  bool checkerboard_full_res_detection = false;
  int checkerboard_num_threads = 4;

  // ArUco detection parameters (aruco / aruco2x2 modes). The dictionary is
  // hard-coded to DICT_APRILTAG_16h5. `aruco_full_res_detection` mirrors the
  // checkerboard flag (false ⇒ detect on the 2x-subsampled Y plane for speed).
  // `aruco_num_threads` bounds the aruco2x2 quadrant parallelism ([1, 4]).
  // `aruco_corner_refine` false ⇒ CORNER_REFINE_NONE (fastest, real-time),
  // true ⇒ CORNER_REFINE_SUBPIX (more precise corners, slower).
  bool aruco_full_res_detection = false;
  int aruco_num_threads = 4;
  bool aruco_corner_refine = false;
};

// Message types for WebSocket protocol
namespace Protocol {
// Commands from client
constexpr const char* CMD_DISCOVER = "discover";
constexpr const char* CMD_GET_STATE = "get_state";
constexpr const char* CMD_CONFIGURE = "configure";
constexpr const char* CMD_UNCONFIGURE = "unconfigure";
constexpr const char* CMD_SET_PROCESS_MODE = "set_process_mode";
constexpr const char* CMD_START_CAMERAS = "start_cameras";
constexpr const char* CMD_START_STREAM = "start_stream";
constexpr const char* CMD_STOP_STREAM = "stop_stream";
constexpr const char* CMD_STOP_CAMERAS = "stop_cameras";
constexpr const char* CMD_RESET_FRAME_COUNTS = "reset_frame_counts";
constexpr const char* CMD_SET_HEADER_ONLY = "set_header_only";
constexpr const char* CMD_SET_LENS_POSITION = "set_lens_position";
constexpr const char* CMD_SET_EXPOSURE_TIME = "set_exposure_time";
constexpr const char* CMD_SET_FRAME_DURATION = "set_frame_duration";
constexpr const char* CMD_GET_FRAME_DURATION_LIMITS = "get_frame_duration_limits";
constexpr const char* CMD_GET_LENS_POSITION_LIMITS = "get_lens_position_limits";

// Response types from server
constexpr const char* TYPE_DISCOVERY = "discovery";
constexpr const char* TYPE_STATE = "state";
constexpr const char* TYPE_STATUS = "status";
constexpr const char* TYPE_FRAME = "frame";
constexpr const char* TYPE_ERROR = "error";
constexpr const char* TYPE_FRAME_DURATION_LIMITS = "frame_duration_limits";
constexpr const char* TYPE_LENS_POSITION_LIMITS = "lens_position_limits";
}  // namespace Protocol

// Hardware LensPosition range from libcamera's ControlInfoMap, in dioptres
// (min ~0 = infinity, max = closest macro). `def` is the IPA's default lens
// position. Any field is NaN when the control is unavailable (no focuser, or
// cameras not acquired) — distinguishes "0 dioptres" (a valid value) from
// "unknown".
struct LensPositionLimits {
  float min;
  float max;
  float def;
};
