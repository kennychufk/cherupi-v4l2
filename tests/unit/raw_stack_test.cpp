#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "aruco_test_utils.hpp"
#include "raw_stack.hpp"

namespace {

// A raw frame over a caller-owned buffer of left-justified uint16 samples.
RawFrame viewOf(const std::vector<uint16_t>& buf, uint32_t w, uint32_t h,
                size_t stride_samples, unsigned bit_depth = 12) {
  RawFrame f;
  f.data = reinterpret_cast<const uint8_t*>(buf.data());
  f.stride = stride_samples * sizeof(uint16_t);
  f.width = w;
  f.height = h;
  f.bit_depth = bit_depth;
  return f;
}

TEST(RawStackTest, AccumulatorSumsEachBayerQuadAndHonoursStride) {
  // 4x2 sensor image in a 6-sample-wide buffer: two quads, padding ignored.
  const uint32_t w = 4, h = 2;
  const size_t stride = 6;
  std::vector<uint16_t> buf(stride * h, 9999);
  const uint16_t px[2][4] = {{100, 200, 300, 400}, {500, 600, 700, 65520}};
  for (uint32_t y = 0; y < h; ++y)
    for (uint32_t x = 0; x < w; ++x) buf[y * stride + x] = px[y][x];

  raw_stack::Accumulator acc;
  acc.reset(w, h);
  ASSERT_EQ(acc.width(), 2u);
  ASSERT_EQ(acc.height(), 1u);
  EXPECT_EQ(acc.add(viewOf(buf, w, h, stride), 65000), 1u);  // one clipped
  EXPECT_EQ(acc.add(viewOf(buf, w, h, stride), 65000), 1u);
  EXPECT_EQ(acc.frames(), 2u);
  EXPECT_EQ(acc.sum()[0], 2u * (100 + 200 + 500 + 600));
  EXPECT_EQ(acc.sum()[1], 2u * (300 + 400 + 700 + 65520));
  EXPECT_EQ(acc.peak(), 65520);

  // Mean of the quad's samples over both frames, less black.
  cv::Mat mean = raw_stack::meanImage(acc, 100.f);
  ASSERT_EQ(mean.type(), CV_32FC1);
  EXPECT_FLOAT_EQ(mean.at<float>(0, 0), (100 + 200 + 500 + 600) / 4.f - 100.f);
  EXPECT_FLOAT_EQ(mean.at<float>(0, 1), (300 + 400 + 700 + 65520) / 4.f - 100.f);
}

TEST(RawStackTest, AccumulatorRejectsAFrameOfAnotherSize) {
  std::vector<uint16_t> buf(16, 1000);
  raw_stack::Accumulator acc;
  acc.reset(4, 4);
  EXPECT_EQ(acc.add(viewOf(buf, 2, 2, 2), 65000), 0u);
  EXPECT_EQ(acc.frames(), 0u);
}

TEST(RawStackTest, FullScaleCodeIsLeftJustified) {
  EXPECT_EQ(raw_stack::fullScaleCode(12), 65520);
  EXPECT_EQ(raw_stack::fullScaleCode(10), 65472);
  EXPECT_EQ(raw_stack::fullScaleCode(16), 65535);
}

TEST(RawStackTest, SettingsMatchWithinQuantisation) {
  EXPECT_TRUE(raw_stack::exposureMatches(std::nullopt, -1));
  EXPECT_TRUE(raw_stack::exposureMatches(30000, 29980));
  EXPECT_TRUE(raw_stack::exposureMatches(30000, 30550));   // 2 %
  EXPECT_FALSE(raw_stack::exposureMatches(30000, 31000));
  EXPECT_TRUE(raw_stack::exposureMatches(1000, 1090));     // 100 us floor
  EXPECT_FALSE(raw_stack::exposureMatches(30000, -1));     // not reported
  EXPECT_TRUE(raw_stack::gainMatches(std::nullopt, NAN));
  EXPECT_TRUE(raw_stack::gainMatches(1.0f, 1.02f));
  EXPECT_FALSE(raw_stack::gainMatches(1.0f, 1.2f));
  EXPECT_FALSE(raw_stack::gainMatches(1.0f, NAN));
}

TEST(RawStackTest, BinnedPixelMapsToTheCentreOfItsQuad) {
  const cv::Point2f p = raw_stack::binnedToFull({10.f, 3.25f});
  EXPECT_FLOAT_EQ(p.x, 20.5f);
  EXPECT_FLOAT_EQ(p.y, 7.0f);
}

TEST(RawStackTest, WritesALoadableNpy) {
  cv::Mat img(3, 5, CV_32FC1);
  for (int i = 0; i < 15; ++i) img.at<float>(i / 5, i % 5) = 0.5f * i;
  const std::string path = std::string(CHERUPI_TEST_SCRATCH_DIR) + "/stack.npy";
  std::system(("mkdir -p " + std::string(CHERUPI_TEST_SCRATCH_DIR)).c_str());
  ASSERT_TRUE(raw_stack::writeNpy(path, img));

  std::ifstream f(path, std::ios::binary);
  std::vector<char> bytes((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
  ASSERT_GE(bytes.size(), 10u);
  EXPECT_EQ(std::string(bytes.data() + 1, 5), "NUMPY");
  const size_t header_len = static_cast<uint8_t>(bytes[8]) |
                            (static_cast<uint8_t>(bytes[9]) << 8);
  EXPECT_EQ((10 + header_len) % 64, 0u);
  const std::string header(bytes.data() + 10, header_len);
  EXPECT_NE(header.find("'descr': '<f4'"), std::string::npos);
  EXPECT_NE(header.find("'shape': (3, 5)"), std::string::npos);
  EXPECT_EQ(header.back(), '\n');
  ASSERT_EQ(bytes.size(), 10 + header_len + 15 * sizeof(float));
  float last;
  std::memcpy(&last, bytes.data() + bytes.size() - sizeof(float), sizeof(float));
  EXPECT_FLOAT_EQ(last, 7.0f);
}

// --- stacking a synthetic board ---------------------------------------------

// A blurred 11x8-inner-corner board at sensor resolution, as radiance in
// [0, 1], with its corners at known non-integer pixel-centre positions.
struct Board {
  cv::Mat radiance;  // CV_32FC1
  std::vector<cv::Point2f> truth;
};

Board renderBoard(double square_px, cv::Size size, cv::Point2d origin) {
  const double angle = 0.04;
  const cv::Point2d u(std::cos(angle), std::sin(angle));
  const cv::Point2d v(-std::sin(angle), std::cos(angle));
  cv::Mat img(size, CV_32FC1);
  constexpr int kSuper = 4;
  for (int y = 0; y < size.height; ++y) {
    for (int x = 0; x < size.width; ++x) {
      int dark = 0;
      for (int sy = 0; sy < kSuper; ++sy)
        for (int sx = 0; sx < kSuper; ++sx) {
          const cv::Point2d p(x + (sx + 0.5) / kSuper - 0.5 - origin.x,
                              y + (sy + 0.5) / kSuper - 0.5 - origin.y);
          const double bu = p.dot(u) / square_px, bv = p.dot(v) / square_px;
          if (bu < -1 || bu >= 11 || bv < -1 || bv >= 8) continue;
          if (((static_cast<int>(std::floor(bu)) +
                static_cast<int>(std::floor(bv))) & 1) == 0)
            ++dark;
        }
      img.at<float>(y, x) = 0.8f - 0.7f * dark / (kSuper * kSuper);
    }
  }
  cv::GaussianBlur(img, img, cv::Size(0, 0), 3.0);
  Board b{img, {}};
  for (int j = 0; j < 8; ++j)
    for (int i = 0; i < 11; ++i)
      b.truth.emplace_back(origin + u * (i * square_px) + v * (j * square_px));
  return b;
}

// One noisy raw frame of `board`: black 4096, white ~40000 of the 16-bit
// scale, Gaussian noise of `sigma` DN per sample.
std::vector<uint16_t> rawFrame(const Board& board, double sigma, cv::RNG& rng) {
  cv::Mat noise(board.radiance.size(), CV_32FC1);
  rng.fill(noise, cv::RNG::NORMAL, 0.0, sigma);
  cv::Mat dn = board.radiance * 45000.0 + 4096.0 + noise;
  std::vector<uint16_t> out(board.radiance.total());
  for (int y = 0; y < dn.rows; ++y)
    for (int x = 0; x < dn.cols; ++x)
      out[static_cast<size_t>(y) * dn.cols + x] = static_cast<uint16_t>(
          std::clamp(dn.at<float>(y, x), 0.f, 65520.f));
  return out;
}

// Detect on a stack of `n` frames; corners in sensor pixels, canonical order.
std::vector<cv::Point2f> stackAndDetect(const Board& board, int n, double sigma,
                                        cv::RNG& rng,
                                        const raw_stack::DetectConfig& cfg) {
  const uint32_t w = static_cast<uint32_t>(board.radiance.cols);
  const uint32_t h = static_cast<uint32_t>(board.radiance.rows);
  raw_stack::Accumulator acc;
  acc.reset(w, h);
  for (int i = 0; i < n; ++i) {
    const auto buf = rawFrame(board, sigma, rng);
    acc.add(viewOf(buf, w, h, w), 65000);
  }
  auto sets = raw_stack::detect(raw_stack::meanImage(acc, 4096.f), cfg);
  if (sets.size() != 1) return {};
  std::vector<cv::Point2f> c = sets[0].corners;
  for (auto& p : c) p = raw_stack::binnedToFull(p);
  if (cv::norm(c.front() - board.truth.front()) >
      cv::norm(c.back() - board.truth.front()))
    std::reverse(c.begin(), c.end());
  return c;
}

// RMS scatter of corner positions about their per-corner mean over trials.
double scatter(const std::vector<std::vector<cv::Point2f>>& runs) {
  const size_t n = runs.front().size();
  double sum = 0;
  for (size_t k = 0; k < n; ++k) {
    cv::Point2d mean(0, 0);
    for (const auto& r : runs) mean += cv::Point2d(r[k]);
    mean *= 1.0 / runs.size();
    for (const auto& r : runs) {
      const cv::Point2d d = cv::Point2d(r[k]) - mean;
      sum += d.dot(d);
    }
  }
  return std::sqrt(sum / (n * (runs.size() - 1)));
}

TEST(RawStackTest, StackingSixteenFramesCutsCornerScatterAboutFourfold) {
  const Board board = renderBoard(48.0, cv::Size(940, 720), {150.37, 120.71});
  raw_stack::DetectConfig cfg;
  cfg.kind = ProcessMode::CHECKERBOARD;
  cfg.subpix_half_window = 10;
  cv::RNG rng(1234);
  const double sigma = 1500.0;  // a noisy (high-gain) frame
  constexpr int kTrials = 8;
  std::vector<std::vector<cv::Point2f>> single, stacked;
  for (int t = 0; t < kTrials; ++t) {
    single.push_back(stackAndDetect(board, 1, sigma, rng, cfg));
    stacked.push_back(stackAndDetect(board, 16, sigma, rng, cfg));
    ASSERT_EQ(single.back().size(), 88u);
    ASSERT_EQ(stacked.back().size(), 88u);
  }
  const double s1 = scatter(single), s16 = scatter(stacked);
  double err = 0;
  for (size_t k = 0; k < 88; ++k) {
    const cv::Point2f d = stacked[0][k] - board.truth[k];
    err += d.dot(d);
  }
  err = std::sqrt(err / 88);
  std::cout << "synthetic raw stack scatter: 1 frame " << s1 << " px, 16 frames "
            << s16 << " px (ratio " << s1 / s16 << "); error vs truth " << err
            << " px\n";
  EXPECT_GT(s1 / s16, 2.5);  // ideal 4
  EXPECT_LT(err, 0.05);      // the binned-to-full mapping lands on truth
}

TEST(RawStackTest, QuadrantDetectionReportsQuadrantAndFullImageCoordinates) {
  // The board sits in the bottom-right quadrant of a 2x-larger image.
  const Board quad = renderBoard(24.0, cv::Size(470, 360), {75.3, 60.4});
  cv::Mat whole(720, 940, CV_32FC1, cv::Scalar(0.8f));
  quad.radiance.copyTo(whole(cv::Rect(470, 360, 470, 360)));
  whole *= 40000.0;

  raw_stack::DetectConfig cfg;
  cfg.kind = ProcessMode::CHECKERBOARD2X2;
  cfg.subpix_half_window = 9;
  auto sets = raw_stack::detect(whole, cfg);
  ASSERT_EQ(sets.size(), 1u);
  EXPECT_EQ(sets[0].set_id, 3);
  ASSERT_EQ(sets[0].corners.size(), 88u);
  std::vector<cv::Point2f> c = sets[0].corners;
  const cv::Point2f t0 = quad.truth.front() + cv::Point2f(470.f, 360.f);
  if (cv::norm(c.front() - t0) > cv::norm(c.back() - t0))
    std::reverse(c.begin(), c.end());
  for (size_t k = 0; k < 88; ++k)
    EXPECT_LT(cv::norm(c[k] - (quad.truth[k] + cv::Point2f(470.f, 360.f))), 0.1);
}

TEST(RawStackTest, FindsAMarkerOnALinearStack) {
  cv::Mat canvas = aruco_test_utils::makeMarkerCanvas(/*id=*/7, 120, 240);
  cv::Mat blurred;
  cv::GaussianBlur(canvas, blurred, cv::Size(0, 0), 1.5);
  cv::Mat lin;
  blurred.convertTo(lin, CV_32F, 30000.0 / 255.0);  // linear DN, any scale

  raw_stack::DetectConfig cfg;
  cfg.kind = ProcessMode::ARUCO;
  auto sets = raw_stack::detect(lin, cfg);
  ASSERT_EQ(sets.size(), 1u);
  EXPECT_EQ(sets[0].marker_id, 7);
  ASSERT_EQ(sets[0].corners.size(), 4u);
  // Canvas 240, marker 120: its outer corners at 60 and 180 (pixel edges, so
  // 59.5 and 179.5 in pixel-centre coordinates).
  for (const auto& p : sets[0].corners) {
    EXPECT_NEAR(std::min(std::fabs(p.x - 59.5f), std::fabs(p.x - 179.5f)), 0.f, 0.3f);
    EXPECT_NEAR(std::min(std::fabs(p.y - 59.5f), std::fabs(p.y - 179.5f)), 0.f, 0.3f);
  }
}

TEST(RawStackTest, NoneDetectsNothing) {
  cv::Mat img(100, 100, CV_32FC1, cv::Scalar(100.f));
  raw_stack::DetectConfig cfg;
  EXPECT_TRUE(raw_stack::detect(img, cfg).empty());
}

}  // namespace
