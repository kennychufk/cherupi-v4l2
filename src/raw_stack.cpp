#include "raw_stack.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <future>

#include <opencv2/imgproc.hpp>

#include "aruco_detector.h"
#include "checkerboard_detector.h"

namespace raw_stack {

void Accumulator::reset(uint32_t raw_width, uint32_t raw_height) {
  width_ = raw_width / 2;
  height_ = raw_height / 2;
  frames_ = 0;
  peak_ = 0;
  sum_.assign(static_cast<size_t>(width_) * height_, 0u);
}

size_t Accumulator::add(const RawFrame& frame, uint16_t clip_level) {
  if (!frame.data || frame.width / 2 != width_ || frame.height / 2 != height_)
    return 0;
  size_t clipped = 0;
  uint16_t peak = peak_;
  const size_t row_bytes = static_cast<size_t>(2 * width_) * sizeof(uint16_t);
  rows_.resize(static_cast<size_t>(4 * width_));
  for (uint32_t y = 0; y < height_; ++y) {
    const uint8_t* src = frame.data + static_cast<size_t>(2 * y) * frame.stride;
    std::memcpy(rows_.data(), src, row_bytes);
    std::memcpy(rows_.data() + 2 * width_, src + frame.stride, row_bytes);
    const uint16_t* r0 = rows_.data();
    const uint16_t* r1 = rows_.data() + 2 * width_;
    uint32_t* out = sum_.data() + static_cast<size_t>(y) * width_;
    // Kept branch-free so the compiler vectorises it: the whole 12 MP frame
    // is summed on the libcamera completion thread.
    uint16_t row_peak = 0;
    uint32_t row_clipped = 0;
    for (uint32_t x = 0; x < width_; ++x) {
      const uint16_t a = r0[2 * x], b = r0[2 * x + 1];
      const uint16_t c = r1[2 * x], d = r1[2 * x + 1];
      out[x] += static_cast<uint32_t>(a) + b + c + d;
      row_peak = std::max(row_peak, std::max(std::max(a, b), std::max(c, d)));
      row_clipped += static_cast<uint32_t>(a >= clip_level) + (b >= clip_level) +
                     (c >= clip_level) + (d >= clip_level);
    }
    peak = std::max(peak, row_peak);
    clipped += row_clipped;
  }
  peak_ = peak;
  ++frames_;
  return clipped;
}

uint16_t fullScaleCode(unsigned bit_depth) {
  if (bit_depth == 0 || bit_depth > 16) bit_depth = 16;
  return static_cast<uint16_t>(((1u << bit_depth) - 1u) << (16u - bit_depth));
}

cv::Mat meanImage(const Accumulator& acc, float black_level,
                  std::vector<uint8_t>& out_buf) {
  out_buf.resize(static_cast<size_t>(acc.width()) * acc.height() * sizeof(float));
  cv::Mat out(static_cast<int>(acc.height()), static_cast<int>(acc.width()),
              CV_32FC1, out_buf.data());
  if (acc.frames() == 0 || out.empty()) {
    out.setTo(0);
    return out;
  }
  const double scale = 1.0 / (4.0 * acc.frames());
  const float black = std::isfinite(black_level) ? black_level : 0.f;
  const uint32_t* src = acc.sum().data();
  for (int y = 0; y < out.rows; ++y) {
    float* row = out.ptr<float>(y);
    const uint32_t* s = src + static_cast<size_t>(y) * acc.width();
    for (int x = 0; x < out.cols; ++x)
      row[x] = static_cast<float>(s[x] * scale) - black;
  }
  return out;
}

cv::Mat meanImage(const Accumulator& acc, float black_level) {
  std::vector<uint8_t> buf;
  cv::Mat view = meanImage(acc, black_level, buf);
  return view.clone();
}

float percentile(const cv::Mat& img, double q) {
  if (img.empty()) return 0.f;
  const size_t n = img.total();
  const size_t step = std::max<size_t>(1, n / 65536);
  std::vector<float> v;
  v.reserve(n / step + 1);
  for (int y = 0; y < img.rows; ++y) {
    const float* row = img.ptr<float>(y);
    for (int x = static_cast<int>((static_cast<size_t>(y) * img.cols) % step);
         x < img.cols; x += static_cast<int>(step))
      v.push_back(row[x]);
  }
  if (v.empty()) return 0.f;
  const size_t k = std::min(
      v.size() - 1, static_cast<size_t>(std::clamp(q, 0.0, 1.0) * (v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

bool exposureMatches(std::optional<int32_t> target_us, int32_t actual_us) {
  if (!target_us) return true;
  if (actual_us < 0) return false;
  const double tol = std::max(0.02 * *target_us, 100.0);
  return std::fabs(static_cast<double>(actual_us) - *target_us) <= tol;
}

bool gainMatches(std::optional<float> target, float actual) {
  if (!target) return true;
  if (!std::isfinite(actual)) return false;
  return std::fabs(actual - *target) <= 0.02f * *target + 0.01f;
}

cv::Point2f binnedToFull(cv::Point2f p) {
  return {2.f * p.x + 0.5f, 2.f * p.y + 0.5f};
}

namespace {

// One detection job: an 8-bit view to detect on, the float view of the same
// region to refine on, and where the region sits in the whole image.
struct Region {
  cv::Mat enc;  // CV_8UC1
  cv::Mat lin;  // CV_32FC1, linear (0..255 scale for the marker modes)
  cv::Point2f offset;
  uint8_t set_id = 0;
};

// Checkerboard on one region: SB on the 8-bit rendering, then cornerSubPix
// on the float one. SB is retried at half size when it fails at full size:
// it misses large, soft squares (~95 px at the binned IMX477 resolution is at
// the edge of what it reliably finds), and the refinement does not care how
// coarse the starting corners were.
bool detectBoard(const Region& r, const DetectConfig& cfg, CornerSet* out) {
  CheckerboardDetector det(cfg.cols, cfg.rows);
  std::vector<cv::Point2f> corners;
  bool ok = det.detect(r.enc.data, r.enc.cols, r.enc.rows, r.enc.step, &corners);
  if (!ok) {
    cv::Mat half;
    cv::resize(r.enc, half, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    ok = det.detect(half.data, half.cols, half.rows, half.step, &corners);
    for (auto& p : corners) p = {2.f * p.x + 0.5f, 2.f * p.y + 0.5f};
  }
  if (!ok) return false;
  det.refine(r.lin, corners, cfg.subpix_half_window);
  for (auto& p : corners) p += r.offset;
  out->set_id = r.set_id;
  out->marker_id = -1;
  out->corners = std::move(corners);
  return true;
}

void detectMarkers(const Region& r, const DetectConfig& cfg,
                   std::vector<CornerSet>* out) {
  ArucoDetector det(cfg.aruco_corner_refine, /*edge_refine=*/false);
  std::vector<ArucoDetector::Marker> markers;
  if (!det.detect(r.enc.data, r.enc.cols, r.enc.rows, r.enc.step, &markers))
    return;
  for (auto& m : markers) {
    if (cfg.aruco_edge_refine)
      ArucoDetector::refineCornersFromEdgesLinear(r.lin, m.corners);
    for (auto& p : m.corners) p += r.offset;
    CornerSet set;
    set.set_id = r.set_id;
    set.marker_id = m.id;
    set.corners = std::move(m.corners);
    out->push_back(std::move(set));
  }
}

}  // namespace

std::vector<CornerSet> detect(const cv::Mat& linear, const DetectConfig& cfg) {
  std::vector<CornerSet> sets;
  if (cfg.kind == ProcessMode::NONE || linear.empty() ||
      linear.type() != CV_32FC1)
    return sets;

  // Render the 8-bit gamma-2.2 image the detectors look at, normalised to
  // the image's white, through a lookup table: no float copies of the image
  // and no per-pixel pow on the 1 GB Pi this runs on.
  float white = percentile(linear, 0.995);
  if (!(white > 0.f)) white = 1.f;
  constexpr int kLut = 4096;
  std::array<uint8_t, kLut + 1> lut;
  for (int i = 0; i <= kLut; ++i)
    lut[static_cast<size_t>(i)] = static_cast<uint8_t>(
        std::lround(255.0 * std::pow(static_cast<double>(i) / kLut, 1.0 / 2.2)));
  cv::Mat enc(linear.size(), CV_8UC1);
  const float to_index = kLut / white;
  for (int y = 0; y < linear.rows; ++y) {
    const float* src = linear.ptr<float>(y);
    uint8_t* dst = enc.ptr<uint8_t>(y);
    for (int x = 0; x < linear.cols; ++x) {
      const float v = std::clamp(src[x] * to_index, 0.f, static_cast<float>(kLut));
      dst[x] = lut[static_cast<size_t>(v)];
    }
  }
  // cornerSubPix is indifferent to scale, so the board refines on `linear`
  // itself. The edge fit's contrast floor assumes the 0..255 scale, so the
  // marker modes refine on a copy scaled to white = 255.
  const bool board = cfg.kind == ProcessMode::CHECKERBOARD ||
                     cfg.kind == ProcessMode::CHECKERBOARD2X2;
  cv::Mat lin = linear;
  if (!board) linear.convertTo(lin, CV_32F, 255.0 / white);

  std::vector<Region> regions;
  const bool split = cfg.kind == ProcessMode::CHECKERBOARD2X2 ||
                     cfg.kind == ProcessMode::ARUCO2X2;
  if (split) {
    const int hw = lin.cols / 2, hh = lin.rows / 2;
    for (int row = 0; row < 2; ++row)
      for (int col = 0; col < 2; ++col) {
        const cv::Rect roi(col * hw, row * hh, hw, hh);
        regions.push_back({enc(roi), lin(roi),
                           cv::Point2f(static_cast<float>(roi.x),
                                       static_cast<float>(roi.y)),
                           static_cast<uint8_t>(row * 2 + col)});
      }
  } else {
    regions.push_back({enc, lin, cv::Point2f(0.f, 0.f), 0});
  }

  auto run = [&](const Region& r) {
    std::vector<CornerSet> found;
    if (board) {
      CornerSet set;
      if (detectBoard(r, cfg, &set)) found.push_back(std::move(set));
    } else {
      detectMarkers(r, cfg, &found);
    }
    return found;
  };

  // The quadrants are independent; run them `num_threads` at a time.
  const size_t batch =
      static_cast<size_t>(std::clamp(cfg.num_threads, 1, 4));
  for (size_t i = 0; i < regions.size(); i += batch) {
    std::vector<std::future<std::vector<CornerSet>>> jobs;
    for (size_t j = i; j < std::min(regions.size(), i + batch); ++j)
      jobs.push_back(std::async(std::launch::async, run, std::cref(regions[j])));
    for (auto& job : jobs)
      for (auto& set : job.get()) sets.push_back(std::move(set));
  }
  return sets;
}

}  // namespace raw_stack
