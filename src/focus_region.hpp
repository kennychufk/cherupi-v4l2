#pragma once

#include <cstdint>

// The image region libcamera's Raspberry Pi IPA averages into the per-frame
// `FocusFoM` metadata. Pure: geometry only, so it is unit-testable without
// hardware.
//
// How the IPA gets the number (raspberrypi/libcamera, libpisp):
//   - The PiSP frontend measures contrast (CDAF statistics) on the raw sensor
//     image over an 8 x 8 grid (`PISP_CDAF_STATS_SIZE`). The IPA sets no grid
//     offset or cell size, so libpisp's `finalise_cdaf` lays the grid from the
//     top-left corner with cells `max(2, (dim / 8) & ~1)` pixels wide/high —
//     truncated to even, so a few pixels on the right/bottom edge are never
//     measured.
//   - `IpaBase::reportMetadata` averages the cells in rows
//     [rows / 3, rows - rows / 3) and columns [cols / 4, cols - cols / 4).
//     On the 8 x 8 grid integer division makes that rows 2-5 and columns 2-5:
//     the middle half of the image on BOTH axes.
// No control moves this block (AfWindows / AfMetering steer only the AF
// algorithm), so it is fixed for a given sensor mode.
//
// Assumes the frontend does not decimate the statistics, which it only does
// for raw lines wider than 6144 px — wider than any sensor this server drives.
namespace focus_region {

// libpisp `PISP_CDAF_STATS_SIZE`: the CDAF grid is this many cells per side.
constexpr uint32_t kPispCdafGridSize = 8;

struct Rect {
  uint32_t x = 0;
  uint32_t y = 0;
  uint32_t width = 0;
  uint32_t height = 0;
};

struct Region {
  // The averaged block in raw sensor-mode pixels, where the statistics are
  // actually gathered.
  Rect sensor;
  // The same block in output-frame pixels (what clients receive). Identical
  // to `sensor` when the output size equals the sensor mode size, which is
  // the case for every size `configure` accepts; otherwise scaled by
  // output / mode on each axis.
  Rect frame;
  // Output frame size `frame` is expressed in.
  uint32_t frame_width = 0;
  uint32_t frame_height = 0;
  // Grid cells the IPA averages: columns [first_col, first_col + num_cols),
  // rows [first_row, first_row + num_rows) of a grid_size x grid_size grid.
  uint32_t grid_size = 0;
  uint32_t first_col = 0;
  uint32_t first_row = 0;
  uint32_t num_cols = 0;
  uint32_t num_rows = 0;
};

// The FocusFoM region for a sensor mode of `mode_width` x `mode_height` whose
// ISP output is `frame_width` x `frame_height`.
Region fomRegion(uint32_t mode_width, uint32_t mode_height,
                 uint32_t frame_width, uint32_t frame_height,
                 uint32_t grid_size = kPispCdafGridSize);

}  // namespace focus_region
