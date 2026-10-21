#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "frame_processor.hpp"  // CornerSet
#include "types.hpp"

// Pure pieces of raw frame stacking (stack_capture): summing raw Bayer frames,
// turning the sum into a linear image, deciding whether a frame was taken
// with the requested exposure, and detecting on the result. No libcamera, no
// threads, so all of it is unit-tested.
//
// Why raw and why stack: on a still rig a corner's frame-to-frame scatter is
// photon noise, so averaging N frames divides it by sqrt(N) — but only when
// the frames' noise is independent. The ISP's temporal denoise correlates
// consecutive YUV frames (lag-1 correlation ~0.7 on the Pi 5), so stacking
// its output gains ~1.5x over 32 frames; the same 32 raw frames gain the
// full 5.7x. Measured on an IMX477 at 4056x3040, 11x8 board, gain 1, 50 ms:
// 0.0154 px for one frame, 0.0029 px for 32 (cornerSubPix on the stack).
namespace raw_stack {

// Running sum of raw frames, binned 2x2: each output sample is the sum of one
// Bayer quad (R + G + G + B for a BGGR/RGGB sensor) over every added frame.
// The quad sum is linear in scene radiance for a neutral (grey/black/white)
// target, which is what a calibration board is, and needs no demosaic.
// uint32 holds 4 x 65535 x 16384 frames; kMaxFrames keeps far inside that.
class Accumulator {
 public:
  static constexpr uint32_t kMaxFrames = 1024;

  // Size the sum for a `raw_width` x `raw_height` sensor image (odd trailing
  // rows/columns are dropped) and zero it.
  void reset(uint32_t raw_width, uint32_t raw_height);

  // Add one frame. `clip_level` is the 16-bit code at or above which a sample
  // counts as clipped. Returns the number of clipped samples in this frame.
  //
  // Each row pair is first memcpy'd into a scratch buffer and summed from
  // there. The frame is a DMA buffer the CPU maps uncached: measured on a Pi
  // 5 at 4056x3040, summing straight out of it took 320 ms per frame, the
  // copy-then-sum ~30 ms, because memcpy's wide sequential loads stream where
  // the loop's small strided ones each stall on memory.
  size_t add(const RawFrame& frame, uint16_t clip_level);

  uint32_t width() const { return width_; }    // binned
  uint32_t height() const { return height_; }  // binned
  uint32_t frames() const { return frames_; }
  uint16_t peak() const { return peak_; }      // brightest sample seen
  const std::vector<uint32_t>& sum() const { return sum_; }
  // Drop the sum's memory (12 MB at 4056x3040) once the image is formed.
  void release() {
    std::vector<uint32_t>().swap(sum_);
    std::vector<uint16_t>().swap(rows_);
  }

 private:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t frames_ = 0;
  uint16_t peak_ = 0;
  std::vector<uint32_t> sum_;
  std::vector<uint16_t> rows_;  // one row pair, copied out of the frame
};

// The largest code a left-justified `bit_depth` sample can take
// (e.g. 12-bit: 4095 << 4 = 65520).
uint16_t fullScaleCode(unsigned bit_depth);

// The stacked image: the mean of each quad's four samples over all frames,
// minus `black_level`, as CV_32FC1 (black = 0, sensor full scale =
// fullScaleCode - black_level).
cv::Mat meanImage(const Accumulator& acc, float black_level);
// The same, written into `out` (resized to fit) so the image can live in a
// frame's payload without a second copy; the returned Mat views `out`.
cv::Mat meanImage(const Accumulator& acc, float black_level,
                  std::vector<uint8_t>& out);

// Value at fraction `q` (0..1) of `img`'s samples, estimated from a regular
// subsample of at most ~64k of them.
float percentile(const cv::Mat& img, double q);

// Whether a frame's reported exposure / gain is the one asked for. Exposure
// is quantised to sensor lines and gain to the sensor's gain steps, so
// "equal" means within max(2 %, 100 us) and 2 % + 0.01 respectively. An
// unset target always matches; an unreported value never matches a target.
bool exposureMatches(std::optional<int32_t> target_us, int32_t actual_us);
bool gainMatches(std::optional<float> target, float actual);

// Map a point in a 2x2-binned image to the full-resolution sensor frame:
// binned pixel b covers sensor pixels 2b and 2b + 1, whose centre is 2b + 0.5.
cv::Point2f binnedToFull(cv::Point2f p);

// What to detect on a stacked image, and with which parameters (taken from
// the process configuration; see ProcessConfig).
struct DetectConfig {
  ProcessMode kind = ProcessMode::NONE;  // NONE or one of the 4 detector modes
  int rows = 8;
  int cols = 11;
  // cornerSubPix half-window in the STACKED (binned) image's pixels.
  int subpix_half_window = 20;
  int num_threads = 4;  // 2x2 quadrant parallelism, clamped [1, 4]
  bool aruco_corner_refine = false;
  bool aruco_edge_refine = true;
};

// Detect on a stacked linear image (CV_32FC1, any scale). Detection itself
// runs on an 8-bit gamma-2.2 rendering normalised to the image's 99.5th
// percentile — findChessboardCornersSB and detectMarkers want 8-bit — while
// the refinement (cornerSubPix for a board, the edge fit for a marker) runs
// on the float image, so the stack's sub-DN precision survives. Corner sets
// are in `linear`'s pixel coordinates, laid out as the live detector modes
// lay them out (set_id = quadrant for the 2x2 modes, marker_id for ArUco).
std::vector<CornerSet> detect(const cv::Mat& linear, const DetectConfig& cfg);

// Write a CV_32FC1 image as a NumPy .npy file (format 1.0, '<f4', C order).
// Returns false on any I/O failure.
bool writeNpy(const std::string& path, const cv::Mat& img);

}  // namespace raw_stack
