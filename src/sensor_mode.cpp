#include "sensor_mode.hpp"

#include <cctype>

namespace sensor_mode {

unsigned bitDepthOf(std::string_view format_name) {
  size_t i = 0;
  while (i < format_name.size() &&
         !std::isdigit(static_cast<unsigned char>(format_name[i]))) {
    ++i;
  }
  unsigned depth = 0;
  for (; i < format_name.size() &&
         std::isdigit(static_cast<unsigned char>(format_name[i]));
       ++i) {
    depth = depth * 10 + static_cast<unsigned>(format_name[i] - '0');
  }
  return depth;
}

std::optional<Selection> selectRawMode(const std::vector<RawFormat>& formats,
                                       uint32_t width, uint32_t height) {
  std::optional<Selection> best;
  for (const auto& fmt : formats) {
    const unsigned depth = bitDepthOf(fmt.name);
    if (depth == 0) continue;
    for (const auto& s : fmt.sizes) {
      if (s.width != width || s.height != height) continue;
      if (!best || depth > best->bit_depth) best = Selection{fmt.name, depth};
      break;
    }
  }
  return best;
}

std::string describe(const std::vector<RawFormat>& formats) {
  std::string out;
  for (const auto& fmt : formats) {
    if (!out.empty()) out += "; ";
    out += fmt.name + ":";
    for (const auto& s : fmt.sizes) {
      out += " " + std::to_string(s.width) + "x" + std::to_string(s.height);
    }
  }
  return out;
}

}  // namespace sensor_mode
