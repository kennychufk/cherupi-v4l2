#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
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

}  // namespace
