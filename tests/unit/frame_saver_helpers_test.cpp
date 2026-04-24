#include <gtest/gtest.h>

#include <chrono>
#include <ctime>

#include "frame_saver_helpers.hpp"

namespace {

TEST(FrameSaverHelpersTest, MakeFilenameFormat) {
  EXPECT_EQ(frame_saver_helpers::makeFilename("/tmp/out", 0, 42),
            "/tmp/out/cam0-42.yuv");
  EXPECT_EQ(frame_saver_helpers::makeFilename(".", 3, 0), "./cam3-0.yuv");
}

TEST(FrameSaverHelpersTest, NormalizeTrimsWhitespace) {
  EXPECT_EQ(frame_saver_helpers::normalizeBaseDir("  /tmp/foo \n"),
            "/tmp/foo");
}

TEST(FrameSaverHelpersTest, NormalizeEmptyFallsBackToCurrentDir) {
  EXPECT_EQ(frame_saver_helpers::normalizeBaseDir(""), ".");
  EXPECT_EQ(frame_saver_helpers::normalizeBaseDir("   \t"), ".");
}

TEST(FrameSaverHelpersTest, TimestampedDirLeafOnly) {
  // Construct a fixed time: 2026-04-24 13:37:00 local. Use std::mktime so the
  // test is stable across DST boundaries (the function formats in local
  // time).
  std::tm fixed{};
  fixed.tm_year = 2026 - 1900;
  fixed.tm_mon = 3;  // April
  fixed.tm_mday = 24;
  fixed.tm_hour = 13;
  fixed.tm_min = 37;
  fixed.tm_sec = 0;
  fixed.tm_isdst = -1;
  auto tp = std::chrono::system_clock::from_time_t(std::mktime(&fixed));

  std::string result = frame_saver_helpers::makeTimestampedDir("runs", tp);
  EXPECT_EQ(result, "20260424-1337-runs");
}

TEST(FrameSaverHelpersTest, TimestampedDirWithParent) {
  std::tm fixed{};
  fixed.tm_year = 2026 - 1900;
  fixed.tm_mon = 3;
  fixed.tm_mday = 24;
  fixed.tm_hour = 13;
  fixed.tm_min = 37;
  fixed.tm_isdst = -1;
  auto tp = std::chrono::system_clock::from_time_t(std::mktime(&fixed));

  EXPECT_EQ(frame_saver_helpers::makeTimestampedDir("/tmp/foo", tp),
            "/tmp/20260424-1337-foo");
}

TEST(FrameSaverHelpersTest, TimestampedDirDotSlashLeaf) {
  // "./runs" has parent_path "." — expected to collapse to just the leaf
  // with the prefix (matches the old FrameSaver behaviour).
  std::tm fixed{};
  fixed.tm_year = 2026 - 1900;
  fixed.tm_mon = 3;
  fixed.tm_mday = 24;
  fixed.tm_hour = 13;
  fixed.tm_min = 37;
  fixed.tm_isdst = -1;
  auto tp = std::chrono::system_clock::from_time_t(std::mktime(&fixed));

  std::string result = frame_saver_helpers::makeTimestampedDir("./runs", tp);
  EXPECT_EQ(result, "20260424-1337-runs");
}

TEST(FrameSaverHelpersTest, ExtractYFullResCopiesLumaPlane) {
  // Synthesize a 4x2 YUV420 frame. Y plane is 4 bytes/row x 2 rows; stride 4.
  // (Real YUV420 has UV following the Y plane; we fill a few bytes after so
  // the extractor's bounds are exercised.)
  FrameData frame;
  frame.width = 4;
  frame.height = 2;
  frame.bytes_per_line = 4;
  frame.data = {
      0x10, 0x11, 0x12, 0x13,  // Y row 0
      0x20, 0x21, 0x22, 0x23,  // Y row 1
      0x80, 0x80, 0x80, 0x80,  // UV (ignored)
  };

  auto y = frame_saver_helpers::extractYFromYUV420(frame, /*full_res=*/true);
  ASSERT_EQ(y.size(), 8u);
  EXPECT_EQ(y[0], 0x10);
  EXPECT_EQ(y[3], 0x13);
  EXPECT_EQ(y[4], 0x20);
  EXPECT_EQ(y[7], 0x23);
}

TEST(FrameSaverHelpersTest, ExtractYHalfResSubsamples) {
  // 4x4 Y plane, subsample every other row/col -> 2x2.
  FrameData frame;
  frame.width = 4;
  frame.height = 4;
  frame.bytes_per_line = 4;
  frame.data = {
      0x01, 0x02, 0x03, 0x04,  //
      0x11, 0x12, 0x13, 0x14,  //
      0x21, 0x22, 0x23, 0x24,  //
      0x31, 0x32, 0x33, 0x34,  //
  };

  auto y = frame_saver_helpers::extractYFromYUV420(frame, /*full_res=*/false);
  ASSERT_EQ(y.size(), 4u);
  // Rows 0 and 2, cols 0 and 2.
  EXPECT_EQ(y[0], 0x01);
  EXPECT_EQ(y[1], 0x03);
  EXPECT_EQ(y[2], 0x21);
  EXPECT_EQ(y[3], 0x23);
}

TEST(FrameSaverHelpersTest, ExtractYRespectsNonContiguousStride) {
  // bytes_per_line > width: simulate real libcamera strides.
  FrameData frame;
  frame.width = 2;
  frame.height = 2;
  frame.bytes_per_line = 4;  // 2 bytes valid + 2 bytes pad per row
  frame.data = {
      0xAA, 0xBB, 0x00, 0x00,  // Y row 0 (only first 2 valid)
      0xCC, 0xDD, 0x00, 0x00,  // Y row 1
  };

  auto y = frame_saver_helpers::extractYFromYUV420(frame, /*full_res=*/true);
  ASSERT_EQ(y.size(), 4u);
  EXPECT_EQ(y[0], 0xAA);
  EXPECT_EQ(y[1], 0xBB);
  EXPECT_EQ(y[2], 0xCC);
  EXPECT_EQ(y[3], 0xDD);
}

}  // namespace
