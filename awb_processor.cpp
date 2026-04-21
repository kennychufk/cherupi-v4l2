#include "awb_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

constexpr float kGainMin = 0.5f;
constexpr float kGainMax = 4.0f;

// Row / group subsampling stride — ~14K groups for 2328x1748.
constexpr int kRowStep = 8;
constexpr int kGroupStep = 8;

// Exclude near-black (below black-level pedestal + margin) and near-saturated
// pixels — they bias the mean and break "Shades of grey" assumptions.
// Values are on the MSB-byte scale (10-bit raw >> 2, so 0..255).
constexpr uint8_t kPixelMin = 24;
constexpr uint8_t kPixelMax = 240;

// Minkowski p-norm exponent for "Shades of grey". p=1 is grey-world;
// p→∞ is max-RGB. p=6 is the established sweet spot (Finlayson & Trezzi,
// "Shades of Gray and Colour Constancy", 2004).
constexpr int kMinkowskiP = 6;

// x^6 via three multiplies. uint8 max → 255^6 ≈ 2.75e14, fits in uint64
// for any plausible sample count.
inline uint64_t pow6(uint8_t v) {
  uint64_t x = v;
  uint64_t x2 = x * x;
  return x2 * x2 * x2;
}

// McCamy's approximation: CCT from CIE xy.
float mccamyCct(float x, float y) {
  float denom = 0.1858f - y;
  if (std::fabs(denom) < 1e-6f) return 0.0f;
  float n = (x - 0.3320f) / denom;
  return 449.0f * n * n * n + 3525.0f * n * n + 6823.3f * n + 5520.33f;
}

// Very rough sensor-RGB -> XYZ (identity-ish). Good enough as a relative
// indicator; not colourimetrically accurate without a sensor calibration.
void rgbToXy(float r, float g, float b, float& x, float& y) {
  float X = 0.4124f * r + 0.3576f * g + 0.1805f * b;
  float Y = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  float Z = 0.0193f * r + 0.1192f * g + 0.9505f * b;
  float sum = X + Y + Z;
  if (sum < 1e-6f) {
    x = y = 0.0f;
    return;
  }
  x = X / sum;
  y = Y / sum;
}

}  // namespace

AwbProcessor::AwbProcessor() = default;

void AwbProcessor::setConfig(const AwbConfig& cfg) {
  cfg_ = cfg;
  reset();
}

void AwbProcessor::reset() {
  gain_r_ = 1.0f;
  gain_b_ = 1.0f;
  cct_ = 0.0f;
  frames_since_compute_ = cfg_.interval;  // Force compute on first frame
  warmup_left_ = cfg_.warmup_frames;
}

bool AwbProcessor::computeRawGains(const FrameData& frame, float& raw_r,
                                   float& raw_b, float& raw_cct) const {
  const uint8_t* data = frame.data.data();
  const uint32_t stride = frame.bytes_per_line;
  const uint32_t height = frame.height;
  const uint32_t width = frame.width;

  if (!data || stride == 0 || height < 2 || width < 4) return false;

  const uint32_t groups_per_row = width / 4;

  uint64_t sum_r = 0, sum_gr = 0, sum_gb = 0, sum_b = 0;
  uint32_t n_r = 0, n_gr = 0, n_gb = 0, n_b = 0;

  // Accumulate p-th powers of pixels that pass the [kPixelMin, kPixelMax]
  // window. Rejecting saturated green pixels is critical on this sensor —
  // G clips before R/B in bright areas and would otherwise depress gains.
  auto accum = [](uint8_t v, uint64_t& sum, uint32_t& n) {
    if (v >= kPixelMin && v <= kPixelMax) {
      sum += pow6(v);
      ++n;
    }
  };

  // Iterate row-pairs (even: R/Gr, odd: Gb/B).
  for (uint32_t y = 0; y + 1 < height; y += kRowStep * 2) {
    const uint8_t* row_even = data + y * stride;
    const uint8_t* row_odd = data + (y + 1) * stride;

    for (uint32_t g = 0; g < groups_per_row; g += kGroupStep) {
      const uint8_t* ge = row_even + g * 5;
      const uint8_t* go = row_odd + g * 5;

      // Even row: R, Gr, R, Gr (byte 4 is packed LSBs — skip).
      accum(ge[0], sum_r, n_r);
      accum(ge[2], sum_r, n_r);
      accum(ge[1], sum_gr, n_gr);
      accum(ge[3], sum_gr, n_gr);

      // Odd row: Gb, B, Gb, B.
      accum(go[0], sum_gb, n_gb);
      accum(go[2], sum_gb, n_gb);
      accum(go[1], sum_b, n_b);
      accum(go[3], sum_b, n_b);
    }
  }

  // Require a minimum sample count per channel — if a channel is mostly
  // saturated or mostly black, the estimate is unreliable. Fall through to
  // the previous smoothed gains by returning false.
  constexpr uint32_t kMinSamples = 256;
  if (n_r < kMinSamples || n_gr < kMinSamples ||
      n_gb < kMinSamples || n_b < kMinSamples) {
    return false;
  }

  // Minkowski p-norm: ( sum(x^p) / N )^(1/p). Biases the estimate toward
  // bright pixels, which reflect the illuminant more than dark scene content.
  const float inv_p = 1.0f / static_cast<float>(kMinkowskiP);
  float mean_r = std::pow(static_cast<float>(sum_r) / n_r, inv_p);
  float mean_gr = std::pow(static_cast<float>(sum_gr) / n_gr, inv_p);
  float mean_gb = std::pow(static_cast<float>(sum_gb) / n_gb, inv_p);
  float mean_b = std::pow(static_cast<float>(sum_b) / n_b, inv_p);
  float mean_g = 0.5f * (mean_gr + mean_gb);

  raw_r = mean_g / std::max(mean_r, 1.0f);
  raw_b = mean_g / std::max(mean_b, 1.0f);

  raw_r = std::clamp(raw_r, kGainMin, kGainMax);
  raw_b = std::clamp(raw_b, kGainMin, kGainMax);

  // CCT estimate from white-balanced means (apply gains, then xy -> CCT).
  float r_wb = mean_r * raw_r;
  float g_wb = mean_g;
  float b_wb = mean_b * raw_b;
  float x, y;
  rgbToXy(r_wb, g_wb, b_wb, x, y);
  raw_cct = mccamyCct(x, y);
  if (!std::isfinite(raw_cct) || raw_cct < 1000.0f || raw_cct > 20000.0f) {
    raw_cct = 0.0f;
  }

  return true;
}

void AwbProcessor::update(FrameData& frame) {
  if (!cfg_.enabled) {
    frame.awb_gain_r = 1.0f;
    frame.awb_gain_b = 1.0f;
    frame.awb_cct = 0.0f;
    return;
  }

  frames_since_compute_++;
  if (frames_since_compute_ >= cfg_.interval) {
    float raw_r = gain_r_, raw_b = gain_b_, raw_cct = cct_;
    if (computeRawGains(frame, raw_r, raw_b, raw_cct)) {
      float speed = (warmup_left_ > 0) ? 1.0f : cfg_.speed;
      gain_r_ = speed * raw_r + (1.0f - speed) * gain_r_;
      gain_b_ = speed * raw_b + (1.0f - speed) * gain_b_;
      cct_ = (cct_ == 0.0f) ? raw_cct
                            : speed * raw_cct + (1.0f - speed) * cct_;
      if (warmup_left_ > 0) warmup_left_--;
    }
    frames_since_compute_ = 0;
  }

  frame.awb_gain_r = gain_r_;
  frame.awb_gain_b = gain_b_;
  frame.awb_cct = cct_;
}
