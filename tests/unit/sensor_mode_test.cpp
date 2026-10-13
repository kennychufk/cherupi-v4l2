#include <gtest/gtest.h>

#include <vector>

#include "sensor_mode.hpp"

namespace {

using sensor_mode::bitDepthOf;
using sensor_mode::RawFormat;
using sensor_mode::selectRawMode;

// The IMX477 (HQ camera) mode table as libcamera advertises it on the Raw
// stream: bit depth differs per mode.
std::vector<RawFormat> imx477Formats() {
  return {{"SRGGB10_CSI2P", {{1332, 990}}},
          {"SRGGB12_CSI2P", {{2028, 1080}, {2028, 1520}, {4056, 3040}}}};
}

TEST(SensorModeTest, BitDepthOfParsesFirstDigitRun) {
  EXPECT_EQ(bitDepthOf("SRGGB10_CSI2P"), 10u);
  EXPECT_EQ(bitDepthOf("SRGGB12_CSI2P"), 12u);
  EXPECT_EQ(bitDepthOf("SBGGR8"), 8u);
  EXPECT_EQ(bitDepthOf("R10_CSI2P"), 10u);
  EXPECT_EQ(bitDepthOf("SRGGB16"), 16u);
  EXPECT_EQ(bitDepthOf("BGGR_PISP_COMP"), 0u);
  EXPECT_EQ(bitDepthOf(""), 0u);
}

TEST(SensorModeTest, Imx477PicksTheModesOwnBitDepth) {
  const auto formats = imx477Formats();

  auto m = selectRawMode(formats, 1332, 990);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->format, "SRGGB10_CSI2P");
  EXPECT_EQ(m->bit_depth, 10u);

  for (auto [w, h] : {std::pair{2028u, 1080u}, {2028u, 1520u}, {4056u, 3040u}}) {
    m = selectRawMode(formats, w, h);
    ASSERT_TRUE(m.has_value()) << w << "x" << h;
    EXPECT_EQ(m->format, "SRGGB12_CSI2P");
    EXPECT_EQ(m->bit_depth, 12u);
  }
}

TEST(SensorModeTest, AllTenBitSensorStaysTenBit) {
  const std::vector<RawFormat> imx708 = {
      {"SRGGB10_CSI2P", {{1536, 864}, {2304, 1296}, {4608, 2592}}}};
  auto m = selectRawMode(imx708, 4608, 2592);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->bit_depth, 10u);
}

TEST(SensorModeTest, NonNativeSizeHasNoMode) {
  EXPECT_FALSE(selectRawMode(imx477Formats(), 1920, 1080).has_value());
  // Width alone matching a mode is not enough.
  EXPECT_FALSE(selectRawMode(imx477Formats(), 2028, 990).has_value());
  EXPECT_FALSE(selectRawMode({}, 1332, 990).has_value());
}

TEST(SensorModeTest, DeepestFormatWinsWhenSeveralOfferTheSize) {
  const std::vector<RawFormat> formats = {
      {"SRGGB10_CSI2P", {{1920, 1080}}},
      {"SRGGB12_CSI2P", {{1920, 1080}}},
      {"SRGGB8", {{1920, 1080}}}};
  auto m = selectRawMode(formats, 1920, 1080);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->format, "SRGGB12_CSI2P");
}

TEST(SensorModeTest, FormatWithoutBitDepthIsSkipped) {
  const std::vector<RawFormat> formats = {{"BGGR_PISP_COMP", {{1920, 1080}}}};
  EXPECT_FALSE(selectRawMode(formats, 1920, 1080).has_value());
}

TEST(SensorModeTest, DescribeListsEveryMode) {
  EXPECT_EQ(sensor_mode::describe(imx477Formats()),
            "SRGGB10_CSI2P: 1332x990; "
            "SRGGB12_CSI2P: 2028x1080 2028x1520 4056x3040");
}

}  // namespace
