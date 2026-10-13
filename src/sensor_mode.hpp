#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Choice of the sensor's native raw mode for a requested output size. Pure:
// takes the raw formats libcamera advertises on the Raw stream (one entry per
// pixel format, each listing the sensor mode sizes at that format) so the
// selection is unit-testable without hardware.
namespace sensor_mode {

struct Size {
  uint32_t width = 0;
  uint32_t height = 0;
};

// One raw pixel format and the sensor mode sizes it is available at, e.g.
// {"SRGGB12_CSI2P", {2028x1080, 2028x1520, 4056x3040}}.
struct RawFormat {
  std::string name;
  std::vector<Size> sizes;
};

struct Selection {
  std::string format;      // libcamera pixel format name, e.g. "SRGGB12_CSI2P"
  unsigned bit_depth = 0;  // e.g. 12
};

// Bit depth encoded in a libcamera raw format name: the first run of digits
// ("SRGGB12_CSI2P" -> 12, "R10_CSI2P" -> 10, "SBGGR8" -> 8). 0 if none.
unsigned bitDepthOf(std::string_view format_name);

// The raw format whose mode list contains exactly `width`x`height`. Sensors
// tie bit depth to mode (the IMX477 offers 1332x990 only at 10 bits and
// 2028x1080 only at 12), and libcamera's RPi pipeline rejects a
// sensorConfig whose bitDepth/outputSize pair is not a mode. If several
// formats offer the size, the deepest wins. nullopt if none does.
std::optional<Selection> selectRawMode(const std::vector<RawFormat>& formats,
                                       uint32_t width, uint32_t height);

// Human-readable mode list for error messages:
// "SRGGB10_CSI2P: 1332x990; SRGGB12_CSI2P: 2028x1080 2028x1520 4056x3040".
std::string describe(const std::vector<RawFormat>& formats);

}  // namespace sensor_mode
