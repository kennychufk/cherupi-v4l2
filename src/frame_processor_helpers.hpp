#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "types.hpp"

namespace frame_processor_helpers {

// Trim leading/trailing whitespace. If the trimmed string is empty, returns
// ".".
std::string normalizeBaseDir(const std::string& base);

// Given an already-normalized base path (no leading/trailing whitespace) and a
// time point, produce `<parent>/YYYYMMDD-HHMM-<leaf>`. If `base` is just a leaf
// ("runs") or "./leaf", the parent portion is preserved unchanged.
std::string makeTimestampedDir(const std::string& base,
                               std::chrono::system_clock::time_point now);

// Produce the on-disk filename a FrameProcessor uses, given its actual output
// directory. Pure string concatenation; no filesystem calls.
std::string makeFilename(const std::string& dir, uint32_t camera_id,
                         uint32_t frame_id);

// Extract the Y (luma) plane from a YUV420 frame into a packed buffer.
// If `full_res` is false, a 2x2 subsample is taken (every other row/column).
// Assumes frame.data holds at least height * bytes_per_line bytes of Y plane
// followed by the UV planes (which are ignored).
std::vector<uint8_t> extractYFromYUV420(const FrameData& frame, bool full_res);

}  // namespace frame_processor_helpers
