#include <gtest/gtest.h>

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
