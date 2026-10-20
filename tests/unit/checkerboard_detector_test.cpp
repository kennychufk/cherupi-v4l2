#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "checkerboard_detector.h"

namespace {

// Minimal PGM (P5) loader. Ignores binary comments for simplicity — the
// fixture generator never emits them.
struct Pgm {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;
};

Pgm loadPgm(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return {};
  std::string magic;
  f >> magic;
  if (magic != "P5") return {};
  int w, h, maxval;
  f >> w >> h >> maxval;
  f.get();  // eat single whitespace after maxval
  Pgm img;
  img.width = w;
  img.height = h;
  img.data.resize(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char*>(img.data.data()), img.data.size());
  return img;
}

#ifndef CHERUPI_TEST_FIXTURES_DIR
#error "CHERUPI_TEST_FIXTURES_DIR must be defined"
#endif

TEST(CheckerboardDetectorTest, DetectsRealCheckerboard) {
  auto img = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/checkerboard_8x11.pgm");
  ASSERT_GT(img.width, 0);
  CheckerboardDetector det(11, 8);
  EXPECT_TRUE(det.detect(img.data.data(), img.width, img.height));
}

TEST(CheckerboardDetectorTest, RejectsNonCheckerboard) {
  auto img = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/no_checkerboard.pgm");
  ASSERT_GT(img.width, 0);
  CheckerboardDetector det(11, 8);
  EXPECT_FALSE(det.detect(img.data.data(), img.width, img.height));
}

TEST(CheckerboardDetectorTest, DetectsSubRectangleViaStride) {
  // Pack the fixture into one quadrant of a 2x-larger gray buffer and ask the
  // detector to view only that quadrant via stride. Mirrors what the
  // CHECKERBOARD2X2 save mode does when it points each detect() at one
  // quadrant of a packed Y plane without copying.
  auto img = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/checkerboard_8x11.pgm");
  ASSERT_GT(img.width, 0);

  const int full_w = img.width * 2;
  const int full_h = img.height * 2;
  std::vector<uint8_t> big(static_cast<size_t>(full_w) * full_h, 128);
  for (int y = 0; y < img.height; ++y) {
    std::memcpy(big.data() + static_cast<size_t>(y) * full_w,
                img.data.data() + static_cast<size_t>(y) * img.width,
                static_cast<size_t>(img.width));
  }

  CheckerboardDetector det(11, 8);
  // Top-left quadrant holds the board.
  EXPECT_TRUE(det.detect(big.data(), img.width, img.height,
                         static_cast<size_t>(full_w)));
  // Top-right quadrant is uniform gray — no board.
  EXPECT_FALSE(det.detect(big.data() + img.width, img.width, img.height,
                          static_cast<size_t>(full_w)));
}

TEST(CheckerboardDetectorTest, ReturnsCornersWhenOutParamProvided) {
  auto img = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/checkerboard_8x11.pgm");
  ASSERT_GT(img.width, 0);

  CheckerboardDetector det(11, 8);
  std::vector<cv::Point2f> corners;
  ASSERT_TRUE(det.detect(img.data.data(), img.width, img.height,
                         /*stride=*/0, &corners));
  EXPECT_EQ(corners.size(), 11u * 8u);

  // Negative case clears the vector even if it was previously populated.
  auto neg = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/no_checkerboard.pgm");
  ASSERT_GT(neg.width, 0);
  EXPECT_FALSE(det.detect(neg.data.data(), neg.width, neg.height,
                          /*stride=*/0, &corners));
  EXPECT_TRUE(corners.empty());
}

TEST(CheckerboardDetectorTest, StatelessAcrossCalls) {
  auto pos = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/checkerboard_8x11.pgm");
  auto neg = loadPgm(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                     "/no_checkerboard.pgm");
  ASSERT_GT(pos.width, 0);
  ASSERT_GT(neg.width, 0);

  CheckerboardDetector det(11, 8);
  EXPECT_TRUE(det.detect(pos.data.data(), pos.width, pos.height));
  EXPECT_FALSE(det.detect(neg.data.data(), neg.width, neg.height));
  EXPECT_TRUE(det.detect(pos.data.data(), pos.width, pos.height));
}

// Render a blurred, slightly rotated 12x9-square board (11x8 inner corners)
// with every corner at a known non-integer position, the way it would look
// to the server: decimate 2x for detection, refine at full resolution.
struct SyntheticBoard {
  cv::Mat image;
  std::vector<cv::Point2f> truth;  // row-major, 11 per row
};

SyntheticBoard renderBoard(double square_px) {
  const double angle = 0.05;  // rad
  const cv::Point2d origin(150.37, 120.71);  // first inner corner
  const cv::Point2d u(std::cos(angle), std::sin(angle));
  const cv::Point2d v(-std::sin(angle), std::cos(angle));
  const int w = static_cast<int>(13 * square_px + 300);
  const int h = static_cast<int>(10 * square_px + 240);
  cv::Mat img(h, w, CV_8UC1);
  constexpr int kSuper = 4;  // supersampling per axis for exact area coverage
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int dark = 0;
      for (int sy = 0; sy < kSuper; ++sy) {
        for (int sx = 0; sx < kSuper; ++sx) {
          const cv::Point2d p(x + (sx + 0.5) / kSuper - 0.5 - origin.x,
                              y + (sy + 0.5) / kSuper - 0.5 - origin.y);
          // Board coordinates in squares, inner corners at integers 0..10/0..7.
          const double bu = p.dot(u) / square_px;
          const double bv = p.dot(v) / square_px;
          if (bu < -1 || bu >= 11 || bv < -1 || bv >= 8) continue;  // white margin
          if (((static_cast<int>(std::floor(bu)) +
                static_cast<int>(std::floor(bv))) & 1) == 0) {
            ++dark;
          }
        }
      }
      img.at<uint8_t>(y, x) = static_cast<uint8_t>(
          200 - (200 - 50) * dark / (kSuper * kSuper));
    }
  }
  cv::GaussianBlur(img, img, cv::Size(0, 0), 2.0);
  SyntheticBoard board{img, {}};
  for (int j = 0; j < 8; ++j)
    for (int i = 0; i < 11; ++i)
      board.truth.emplace_back(origin + u * (i * square_px) + v * (j * square_px));
  return board;
}

// RMS distance from each corner to its nearest ground-truth corner (the
// detector may report the grid in either of its two 180-degree orders).
double rmsError(const std::vector<cv::Point2f>& corners,
                const std::vector<cv::Point2f>& truth) {
  double sum = 0;
  for (const auto& c : corners) {
    double best = 1e9;
    for (const auto& t : truth) best = std::min(best, cv::norm(c - t));
    sum += best * best;
  }
  return std::sqrt(sum / corners.size());
}

TEST(CheckerboardDetectorTest, RefineRecoversSubpixelCorners) {
  const SyntheticBoard board = renderBoard(/*square_px=*/48.0);
  cv::Mat half;
  cv::resize(board.image, half, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

  CheckerboardDetector det(11, 8);
  std::vector<cv::Point2f> corners;
  ASSERT_TRUE(det.detect(half.data, half.cols, half.rows,
                         static_cast<size_t>(half.step), &corners));
  for (auto& p : corners) p *= 2.0f;
  const double before = rmsError(corners, board.truth);

  const int window = det.refine(board.image.data, board.image.cols,
                                board.image.rows,
                                static_cast<size_t>(board.image.step), corners,
                                /*max_half_window=*/40);
  // Capped at 40 % of the smallest neighbour spacing, which on the unrefined
  // corners is at most the true 48 px.
  EXPECT_GE(window, 2);
  EXPECT_LE(window, 19);
  const double after = rmsError(corners, board.truth);
  EXPECT_LT(after, 0.05) << "before refinement: " << before;
  std::cout << "synthetic board RMS error: SB " << before << " px, refined "
            << after << " px (window " << window << ")\n";
  EXPECT_LT(after, before);
}

TEST(CheckerboardDetectorTest, RefineDisabledOrTooSmallLeavesCornersAlone) {
  const SyntheticBoard board = renderBoard(/*square_px=*/48.0);
  CheckerboardDetector det(11, 8);
  std::vector<cv::Point2f> corners = board.truth;
  for (auto& p : corners) p += cv::Point2f(0.7f, -0.4f);
  const std::vector<cv::Point2f> original = corners;

  EXPECT_EQ(det.refine(board.image.data, board.image.cols, board.image.rows,
                       static_cast<size_t>(board.image.step), corners, 0),
            0);
  EXPECT_EQ(corners, original);

  // Corners 4 px apart: 40 % of that is a 1 px window, too small to refine.
  std::vector<cv::Point2f> tiny;
  for (int j = 0; j < 8; ++j)
    for (int i = 0; i < 11; ++i) tiny.emplace_back(200.f + 4 * i, 200.f + 4 * j);
  const std::vector<cv::Point2f> tiny_original = tiny;
  EXPECT_EQ(det.refine(board.image.data, board.image.cols, board.image.rows,
                       static_cast<size_t>(board.image.step), tiny, 40),
            0);
  EXPECT_EQ(tiny, tiny_original);
}

}  // namespace
