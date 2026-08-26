#include "frame_subsample.hpp"

#include <cstddef>

namespace frame_subsample {

namespace {

// Decimate one 8-bit plane. `dst` is dst_w x dst_h, tightly packed; the
// caller guarantees dst_w * factor <= src_w and dst_h * factor <= src_h.
void decimatePlane(const uint8_t* src, size_t src_stride, uint32_t factor,
                   uint8_t* dst, uint32_t dst_w, uint32_t dst_h) {
  const uint32_t norm = factor * factor;
  const uint32_t half = norm / 2;
  for (uint32_t oy = 0; oy < dst_h; ++oy) {
    const uint8_t* block_row = src + static_cast<size_t>(oy) * factor * src_stride;
    uint8_t* out_row = dst + static_cast<size_t>(oy) * dst_w;
    for (uint32_t ox = 0; ox < dst_w; ++ox) {
      const uint8_t* block = block_row + static_cast<size_t>(ox) * factor;
      uint32_t sum = 0;
      for (uint32_t by = 0; by < factor; ++by) {
        const uint8_t* row = block + static_cast<size_t>(by) * src_stride;
        for (uint32_t bx = 0; bx < factor; ++bx) sum += row[bx];
      }
      out_row[ox] = static_cast<uint8_t>((sum + half) / norm);
    }
  }
}

}  // namespace

Geometry subsampledGeometry(uint32_t width, uint32_t height, uint32_t factor) {
  if (factor == 0) factor = 1;
  Geometry g;
  g.width = (width / factor) & ~1u;
  g.height = (height / factor) & ~1u;
  return g;
}

FrameData subsampleYuv420(const FrameData& src, uint32_t factor) {
  if (factor <= 1) return src;

  const Geometry g = subsampledGeometry(src.width, src.height, factor);

  FrameData out;
  out.frame_id = src.frame_id;
  out.camera_id = src.camera_id;
  out.width = g.width;
  out.height = g.height;
  out.bytes_per_line = g.width;
  out.pixel_format = src.pixel_format;
  out.timestamp_us = src.timestamp_us;
  out.frame_duration_us = src.frame_duration_us;
  out.lens_position = src.lens_position;
  out.af_state = src.af_state;

  // Source layout (see docs/websocket-protocol.md §5.5): Y is bytes_per_line
  // x height, then U and V each (bytes_per_line / 2) x (height / 2).
  const size_t src_y_stride = src.bytes_per_line;
  const size_t src_uv_stride = src.bytes_per_line / 2;
  const size_t src_uv_h = src.height / 2;
  const size_t needed = src_y_stride * src.height + 2 * src_uv_stride * src_uv_h;
  if (g.width == 0 || g.height == 0 || src.data.size() < needed) {
    return out;  // geometry only; nothing sensible to decimate
  }

  const size_t dst_y = static_cast<size_t>(g.width) * g.height;
  const uint32_t uv_w = g.width / 2;
  const uint32_t uv_h = g.height / 2;
  out.data.resize(dst_y + 2 * static_cast<size_t>(uv_w) * uv_h);

  const uint8_t* sy = src.data.data();
  const uint8_t* su = sy + src_y_stride * src.height;
  const uint8_t* sv = su + src_uv_stride * src_uv_h;
  uint8_t* dy = out.data.data();
  uint8_t* du = dy + dst_y;
  uint8_t* dv = du + static_cast<size_t>(uv_w) * uv_h;

  decimatePlane(sy, src_y_stride, factor, dy, g.width, g.height);
  decimatePlane(su, src_uv_stride, factor, du, uv_w, uv_h);
  decimatePlane(sv, src_uv_stride, factor, dv, uv_w, uv_h);
  return out;
}

void scaleCornerSets(std::vector<CornerSet>& sets, uint32_t factor) {
  if (factor <= 1) return;
  const float inv = 1.0f / static_cast<float>(factor);
  for (auto& set : sets) {
    for (auto& p : set.corners) {
      p.x *= inv;
      p.y *= inv;
    }
  }
}

}  // namespace frame_subsample
