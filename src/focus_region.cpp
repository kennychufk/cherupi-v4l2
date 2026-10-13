#include "focus_region.hpp"

#include <algorithm>

namespace focus_region {

namespace {

// libpisp finalise_cdaf: cell size along one axis (offset 0).
uint32_t cellSize(uint32_t dim, uint32_t grid_size) {
  return std::max<uint32_t>(2, (dim / grid_size) & ~1u);
}

// Map [begin, end) from a `from`-pixel axis onto a `to`-pixel axis, rounding
// each edge to the nearest pixel.
void scaleSpan(uint32_t begin, uint32_t end, uint32_t from, uint32_t to,
               uint32_t& out_begin, uint32_t& out_size) {
  if (from == to || from == 0) {
    out_begin = begin;
    out_size = end - begin;
    return;
  }
  const auto scale = [&](uint32_t v) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(v) * to + from / 2) / from);
  };
  out_begin = scale(begin);
  out_size = scale(end) - out_begin;
}

}  // namespace

Region fomRegion(uint32_t mode_width, uint32_t mode_height,
                 uint32_t frame_width, uint32_t frame_height,
                 uint32_t grid_size) {
  Region r;
  r.grid_size = grid_size;
  r.frame_width = frame_width;
  r.frame_height = frame_height;
  // ipa_base.cpp reportMetadata: rows [rows/3, rows - rows/3),
  // cols [cols/4, cols - cols/4).
  r.first_row = grid_size / 3;
  r.num_rows = grid_size - 2 * r.first_row;
  r.first_col = grid_size / 4;
  r.num_cols = grid_size - 2 * r.first_col;

  const uint32_t cell_w = cellSize(mode_width, grid_size);
  const uint32_t cell_h = cellSize(mode_height, grid_size);
  r.sensor.x = r.first_col * cell_w;
  r.sensor.y = r.first_row * cell_h;
  r.sensor.width = r.num_cols * cell_w;
  r.sensor.height = r.num_rows * cell_h;

  scaleSpan(r.sensor.x, r.sensor.x + r.sensor.width, mode_width, frame_width,
            r.frame.x, r.frame.width);
  scaleSpan(r.sensor.y, r.sensor.y + r.sensor.height, mode_height,
            frame_height, r.frame.y, r.frame.height);
  return r;
}

}  // namespace focus_region
