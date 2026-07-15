#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "aruco_detector.h"
#include "aruco_test_utils.hpp"

namespace {

using aruco_test_utils::makeMarkerCanvas;

TEST(ArucoDetectorTest, DetectsMarkerWithIdAndFourCorners) {
  const int kId = 17;
  cv::Mat img = makeMarkerCanvas(kId);
  ASSERT_TRUE(img.isContinuous());

  ArucoDetector det;
  std::vector<ArucoDetector::Marker> markers;
  ASSERT_TRUE(det.detect(img.data, img.cols, img.rows, /*stride=*/img.step,
                         &markers));
  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, kId);
  ASSERT_EQ(markers[0].corners.size(), 4u);

  // Corners fall inside the marker region [off, off+side] = [40, 120].
  for (const auto& p : markers[0].corners) {
    EXPECT_GE(p.x, 30.f);
    EXPECT_LE(p.x, 130.f);
    EXPECT_GE(p.y, 30.f);
    EXPECT_LE(p.y, 130.f);
  }
}

TEST(ArucoDetectorTest, RejectsUniformImage) {
  cv::Mat img(160, 160, CV_8UC1, cv::Scalar(128));
  ArucoDetector det;
  std::vector<ArucoDetector::Marker> markers;
  EXPECT_FALSE(det.detect(img.data, img.cols, img.rows, /*stride=*/img.step,
                          &markers));
  EXPECT_TRUE(markers.empty());
}

TEST(ArucoDetectorTest, DetectsSubRectangleViaStride) {
  // Paste a marker canvas into the top-left quadrant of a 2x-larger gray buffer
  // and detect only that quadrant via stride — mirrors what ARUCO2X2 does when
  // it points each detect() at one quadrant of a packed Y plane without copying.
  const int kId = 5;
  cv::Mat canvas = makeMarkerCanvas(kId);  // 160x160
  const int full_w = canvas.cols * 2;
  const int full_h = canvas.rows * 2;
  std::vector<uint8_t> big(static_cast<size_t>(full_w) * full_h, 128);
  for (int y = 0; y < canvas.rows; ++y) {
    std::memcpy(big.data() + static_cast<size_t>(y) * full_w,
                canvas.ptr<uint8_t>(y), static_cast<size_t>(canvas.cols));
  }

  ArucoDetector det;
  std::vector<ArucoDetector::Marker> markers;
  // Top-left quadrant holds the marker.
  EXPECT_TRUE(det.detect(big.data(), canvas.cols, canvas.rows,
                         static_cast<size_t>(full_w), &markers));
  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, kId);

  // Top-right quadrant is uniform gray — no marker.
  EXPECT_FALSE(det.detect(big.data() + canvas.cols, canvas.cols, canvas.rows,
                          static_cast<size_t>(full_w), &markers));
  EXPECT_TRUE(markers.empty());
}

TEST(ArucoDetectorTest, SubpixelRefineStillDetects) {
  // The corner-refine flag must not change detection outcome, only precision.
  const int kId = 11;
  cv::Mat img = makeMarkerCanvas(kId);
  ArucoDetector det(/*corner_refine=*/true);
  std::vector<ArucoDetector::Marker> markers;
  ASSERT_TRUE(det.detect(img.data, img.cols, img.rows, /*stride=*/img.step,
                         &markers));
  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, kId);
}

}  // namespace
