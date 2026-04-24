#include "frame_saver_helpers.hpp"

#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace frame_saver_helpers {

std::string normalizeBaseDir(const std::string& base) {
  std::string trimmed = base;
  trimmed.erase(0, trimmed.find_first_not_of(" \t\n\r"));
  if (!trimmed.empty()) {
    trimmed.erase(trimmed.find_last_not_of(" \t\n\r") + 1);
  }
  if (trimmed.empty()) return ".";
  return trimmed;
}

std::string makeTimestampedDir(const std::string& base,
                               std::chrono::system_clock::time_point now) {
  std::filesystem::path dir_path(base);
  std::filesystem::path parent_path = dir_path.parent_path();
  std::string dir_name = dir_path.filename().string();

  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_copy;
  std::tm* tm_ptr = std::localtime(&time_t_now);
  if (tm_ptr != nullptr) {
    tm_copy = *tm_ptr;
  } else {
    std::memset(&tm_copy, 0, sizeof(tm_copy));
  }

  std::stringstream timestamp_ss;
  timestamp_ss << std::put_time(&tm_copy, "%Y%m%d-%H%M-");
  std::string prefix = timestamp_ss.str();

  std::string timestamped = prefix + dir_name;
  if (parent_path.empty() || parent_path == ".") {
    return timestamped;
  }
  return (parent_path / timestamped).string();
}

std::string makeFilename(const std::string& dir, uint32_t camera_id,
                         uint32_t frame_id) {
  std::stringstream ss;
  ss << dir << "/cam" << camera_id << "-" << frame_id << ".yuv";
  return ss.str();
}

std::vector<uint8_t> extractYFromYUV420(const FrameData& frame, bool full_res) {
  int out_width =
      full_res ? static_cast<int>(frame.width)
               : static_cast<int>(frame.width / 2);
  int out_height =
      full_res ? static_cast<int>(frame.height)
               : static_cast<int>(frame.height / 2);

  std::vector<uint8_t> grayscale_data(
      static_cast<size_t>(out_width) * static_cast<size_t>(out_height));
  if (frame.data.empty() || frame.bytes_per_line == 0) return grayscale_data;

  if (full_res) {
    for (int row = 0; row < out_height; ++row) {
      std::memcpy(grayscale_data.data() +
                      static_cast<size_t>(row) * frame.width,
                  frame.data.data() +
                      static_cast<size_t>(row) * frame.bytes_per_line,
                  frame.width);
    }
  } else {
    for (int row = 0; row < out_height; ++row) {
      const uint8_t* src =
          frame.data.data() +
          static_cast<size_t>(row * 2) * frame.bytes_per_line;
      uint8_t* dst =
          grayscale_data.data() + static_cast<size_t>(row) * out_width;
      for (int col = 0; col < out_width; ++col) {
        dst[col] = src[col * 2];
      }
    }
  }

  return grayscale_data;
}

}  // namespace frame_saver_helpers
