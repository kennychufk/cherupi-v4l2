#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "frame_subsample.hpp"
#include "types.hpp"

namespace {

using frame_subsample::isValidFactor;
using frame_subsample::scaleCornerSets;
using frame_subsample::subsampledGeometry;
using frame_subsample::subsampleYuv420;

// A small YUV420 frame with a row stride wider than the image (padding
// columns hold 255 to prove they are never read). Y[y][x] = x + 10*y,
// U[y][x] = 100 + x, V = 200 everywhere.
FrameData makeGradientFrame(uint32_t width, uint32_t height, uint32_t stride) {
  FrameData f;
  f.width = width;
  f.height = height;
  f.bytes_per_line = stride;
  f.camera_id = 3;
  f.frame_id = 42;
  f.pixel_format = 0x32315559;  // 'YU12'
  f.timestamp_us = 123456;
  f.frame_duration_us = 33333;
  f.lens_position = 2.5f;
  f.af_state = 2;

  const size_t y_size = static_cast<size_t>(stride) * height;
  const size_t uv_size = static_cast<size_t>(stride / 2) * (height / 2);
  f.data.assign(y_size + 2 * uv_size, 255);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      f.data[static_cast<size_t>(y) * stride + x] =
          static_cast<uint8_t>(x + 10 * y);
    }
  }
  uint8_t* u = f.data.data() + y_size;
  uint8_t* v = u + uv_size;
  for (uint32_t y = 0; y < height / 2; ++y) {
    for (uint32_t x = 0; x < stride / 2; ++x) {
      u[static_cast<size_t>(y) * (stride / 2) + x] = static_cast<uint8_t>(100 + x);
      v[static_cast<size_t>(y) * (stride / 2) + x] = 200;
    }
  }
  return f;
}

TEST(FrameSubsampleTest, ValidFactorsArePowersOfTwoUpToEight) {
  EXPECT_TRUE(isValidFactor(1));
  EXPECT_TRUE(isValidFactor(2));
  EXPECT_TRUE(isValidFactor(4));
  EXPECT_TRUE(isValidFactor(8));
  EXPECT_FALSE(isValidFactor(0));
  EXPECT_FALSE(isValidFactor(3));
  EXPECT_FALSE(isValidFactor(6));
  EXPECT_FALSE(isValidFactor(16));
}

TEST(FrameSubsampleTest, GeometryDividesAndRoundsDownToEven) {
  auto g = subsampledGeometry(2328, 1748, 2);
  EXPECT_EQ(g.width, 1164u);
  EXPECT_EQ(g.height, 874u);
  g = subsampledGeometry(2328, 1748, 4);
  EXPECT_EQ(g.width, 582u);
  EXPECT_EQ(g.height, 436u);  // 437 rounded down so U/V stay integral
  g = subsampledGeometry(4608, 2592, 8);
  EXPECT_EQ(g.width, 576u);
  EXPECT_EQ(g.height, 324u);
  g = subsampledGeometry(10, 10, 8);
  EXPECT_EQ(g.width, 0u);
  EXPECT_EQ(g.height, 0u);
}

TEST(FrameSubsampleTest, FactorTwoBoxAveragesEachPlane) {
  FrameData src = makeGradientFrame(8, 4, 16);
  FrameData out = subsampleYuv420(src, 2);

  EXPECT_EQ(out.width, 4u);
  EXPECT_EQ(out.height, 2u);
  EXPECT_EQ(out.bytes_per_line, 4u);
  ASSERT_EQ(out.data.size(), 4u * 2u + 2u * (2u * 1u));

  // Y block (0,0) = {0,1,10,11}: mean 5.5 rounds to 6.
  EXPECT_EQ(out.data[0], 6);
  // Y block (1,0) = {2,3,12,13}: 30/4 = 7.5 -> 8.
  EXPECT_EQ(out.data[1], 8);
  // Y block (0,1) = {20,21,30,31}: 102/4 = 25.5 -> 26.
  EXPECT_EQ(out.data[4], 26);

  // U (2x1): block 0 = {100,101,100,101} -> 101; block 1 = {102,103,...} -> 103.
  const size_t y_out = 8;
  EXPECT_EQ(out.data[y_out], 101);
  EXPECT_EQ(out.data[y_out + 1], 103);
  // V constant.
  EXPECT_EQ(out.data[y_out + 2], 200);
  EXPECT_EQ(out.data[y_out + 3], 200);

  // Metadata is carried over untouched.
  EXPECT_EQ(out.frame_id, 42u);
  EXPECT_EQ(out.camera_id, 3u);
  EXPECT_EQ(out.pixel_format, 0x32315559u);
  EXPECT_EQ(out.timestamp_us, 123456u);
  EXPECT_EQ(out.frame_duration_us, 33333u);
  EXPECT_FLOAT_EQ(out.lens_position, 2.5f);
  EXPECT_EQ(out.af_state, 2);
}

TEST(FrameSubsampleTest, FactorFourAveragesUniformFrameToItself) {
  FrameData src;
  src.width = 16;
  src.height = 8;
  src.bytes_per_line = 16;
  src.data.assign(16 * 8 * 3 / 2, 77);
  FrameData out = subsampleYuv420(src, 4);
  EXPECT_EQ(out.width, 4u);
  EXPECT_EQ(out.height, 2u);
  ASSERT_EQ(out.data.size(), 4u * 2u + 2u * (2u * 1u));
  for (uint8_t b : out.data) EXPECT_EQ(b, 77);
}

TEST(FrameSubsampleTest, FactorOneReturnsACopy) {
  FrameData src = makeGradientFrame(8, 4, 16);
  FrameData out = subsampleYuv420(src, 1);
  EXPECT_EQ(out.data, src.data);
  EXPECT_EQ(out.bytes_per_line, src.bytes_per_line);
  EXPECT_EQ(out.width, src.width);
}

TEST(FrameSubsampleTest, ShortPayloadYieldsGeometryOnly) {
  FrameData src = makeGradientFrame(8, 4, 16);
  src.data.resize(10);
  FrameData out = subsampleYuv420(src, 2);
  EXPECT_EQ(out.width, 4u);
  EXPECT_EQ(out.height, 2u);
  EXPECT_TRUE(out.data.empty());
}

TEST(FrameSubsampleTest, ScaleCornerSetsDividesCoordinates) {
  CornerSet set;
  set.marker_id = 5;
  set.corners = {{100.f, 50.f}, {3.f, 1.f}};
  std::vector<CornerSet> sets{set};
  scaleCornerSets(sets, 4);
  ASSERT_EQ(sets.size(), 1u);
  EXPECT_EQ(sets[0].marker_id, 5);
  EXPECT_FLOAT_EQ(sets[0].corners[0].x, 25.f);
  EXPECT_FLOAT_EQ(sets[0].corners[0].y, 12.5f);
  EXPECT_FLOAT_EQ(sets[0].corners[1].x, 0.75f);
  EXPECT_FLOAT_EQ(sets[0].corners[1].y, 0.25f);
  scaleCornerSets(sets, 1);
  EXPECT_FLOAT_EQ(sets[0].corners[0].x, 25.f);
}

}  // namespace
