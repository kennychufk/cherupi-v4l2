#pragma once

#include <cstdint>

#include "types.hpp"

// Grey-world AWB processor: computes per-channel gains from a packed
// SRGGB10P raw Bayer frame and stamps smoothed gains onto FrameData.
class AwbProcessor {
 public:
  AwbProcessor();

  void setConfig(const AwbConfig& cfg);
  void reset();

  // Update internal state based on the frame (recomputes every N frames),
  // then stamps current smoothed gains / CCT onto `frame`.
  void update(FrameData& frame);

 private:
  AwbConfig cfg_;
  float gain_r_ = 1.0f;
  float gain_b_ = 1.0f;
  float cct_ = 0.0f;
  int frames_since_compute_ = 0;
  int warmup_left_ = 0;

  // Compute raw grey-world gains from packed SRGGB10P by summing 8-bit MSB
  // bytes on a subsampled grid. Returns false if stats were too sparse.
  bool computeRawGains(const FrameData& frame, float& raw_r, float& raw_b,
                       float& raw_cct) const;
};
