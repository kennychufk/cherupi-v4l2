#pragma once

#include <cstdint>
#include <vector>

#include "frame_processor.hpp"  // CornerSet
#include "types.hpp"

// Decimation of a planar YUV420 frame for a bandwidth-limited stream (the
// observer's `subsample` option). Pure functions: no I/O, no shared state.
namespace frame_subsample {

constexpr uint32_t kMaxFactor = 8;

// Only powers of two up to kMaxFactor, so chroma stays aligned.
inline bool isValidFactor(uint32_t factor) {
  return factor == 1 || factor == 2 || factor == 4 || factor == 8;
}

struct Geometry {
  uint32_t width = 0;
  uint32_t height = 0;
};

// Output size for `factor`: each dimension divided by the factor and rounded
// down to an even number, so the half-resolution U/V planes stay integral.
Geometry subsampledGeometry(uint32_t width, uint32_t height, uint32_t factor);

// Box-average decimation of `src` by `factor` (1 returns a copy). Every output
// sample is the rounded mean of a factor x factor block of the corresponding
// source plane. Metadata is carried over; `width`/`height` become the
// subsampled geometry and `bytes_per_line` equals the new width (tightly
// packed). If `src.data` is shorter than its geometry implies, the returned
// frame carries the new geometry but an empty payload.
FrameData subsampleYuv420(const FrameData& src, uint32_t factor);

// Map detection coordinates (full-frame Y-plane pixels) into the subsampled
// frame's pixel space, in place. A sample at source (x, y) lands at
// (x / factor, y / factor).
void scaleCornerSets(std::vector<CornerSet>& sets, uint32_t factor);

}  // namespace frame_subsample
