#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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


// --- the edge refinement ----------------------------------------------------

double meanSide(const std::vector<cv::Point2f>& c) {
  double s = 0.0;
  for (int k = 0; k < 4; ++k) {
    const cv::Point2f d = c[(k + 1) % 4] - c[k];
    s += std::hypot(d.x, d.y);
  }
  return s / 4.0;
}

// What a real camera makes of the sharp render: optical blur (sigma 2 px is
// the 5 px 10-90 % edge width measured on a 4608 px Camera Module 3 Wide
// stream), then the ISP's gamma encoding of the luma.
cv::Mat blurAndGammaEncode(const cv::Mat& sharp, double sigma, double gamma) {
  cv::Mat blurred;
  cv::GaussianBlur(sharp, blurred, cv::Size(0, 0), sigma);
  cv::Mat enc(blurred.size(), CV_8UC1);
  for (int y = 0; y < enc.rows; ++y)
    for (int x = 0; x < enc.cols; ++x)
      enc.at<uint8_t>(y, x) = cv::saturate_cast<uint8_t>(std::lround(
          255.0 * std::pow(blurred.at<uint8_t>(y, x) / 255.0, 1.0 / gamma)));
  return enc;
}

// The black square of makeMarkerCanvas(id, side, canvas) spans pixels
// [off, off+side) in both axes, so in pixel-centre coordinates its corners
// are at off-0.5 and off+side-0.5.
std::vector<cv::Point2f> trueCorners(int side, int canvas) {
  const float o = static_cast<float>((canvas - side) / 2) - 0.5f;
  const float e = o + static_cast<float>(side);
  return {{o, o}, {e, o}, {e, e}, {o, e}};
}

std::vector<cv::Point2f> detectOne(ArucoDetector& det, const cv::Mat& img,
                                   int id) {
  std::vector<ArucoDetector::Marker> markers;
  EXPECT_TRUE(det.detect(img.data, img.cols, img.rows, img.step, &markers));
  EXPECT_EQ(markers.size(), 1u);
  if (markers.size() != 1) return {};
  EXPECT_EQ(markers[0].id, id);
  EXPECT_EQ(markers[0].corners.size(), 4u);
  return markers[0].corners;
}

TEST(ArucoDetectorTest, EdgeRefineUndoesTheBlurAndGammaCornerBias) {
  const int kId = 3, kSide = 96, kCanvas = 200;
  const cv::Mat sharp = makeMarkerCanvas(kId, kSide, kCanvas);
  const cv::Mat camera = blurAndGammaEncode(sharp, 2.0, 2.2);
  const auto truth = trueCorners(kSide, kCanvas);

  // OpenCV's own corners, SUBPIX included, sit well inside the true square
  // on such an image — the bias the refinement exists for.
  ArucoDetector plain(/*corner_refine=*/true, /*edge_refine=*/false);
  const auto plain_c = detectOne(plain, camera, kId);
  ASSERT_EQ(plain_c.size(), 4u);
  EXPECT_LT(meanSide(plain_c), kSide - 2.0);
  for (int k = 0; k < 4; ++k) {
    const cv::Point2f d = plain_c[k] - truth[k];
    EXPECT_GT(std::hypot(d.x, d.y), 1.0) << "corner " << k;
  }

  // Edge-refined in linear light: back on the true corners.
  ArucoDetector refined(/*corner_refine=*/true, /*edge_refine=*/true, 2.2f);
  const auto ref_c = detectOne(refined, camera, kId);
  ASSERT_EQ(ref_c.size(), 4u);
  EXPECT_NEAR(meanSide(ref_c), kSide, 0.25);
  for (int k = 0; k < 4; ++k) {
    const cv::Point2f d = ref_c[k] - truth[k];
    EXPECT_LT(std::hypot(d.x, d.y), 0.25) << "corner " << k;
  }

  // Without the decode the half-level crossing of the ENCODED luma is on
  // the dark side of the edge, so the square still reads small: the gamma
  // is not a cosmetic knob.
  ArucoDetector encoded(/*corner_refine=*/true, /*edge_refine=*/true, 1.0f);
  const auto enc_c = detectOne(encoded, camera, kId);
  ASSERT_EQ(enc_c.size(), 4u);
  EXPECT_LT(meanSide(enc_c), kSide - 2.0);
}

TEST(ArucoDetectorTest, EdgeRefineLeavesSharpCornersWhereTheyAre) {
  const int kId = 11, kSide = 96, kCanvas = 200;
  const cv::Mat sharp = makeMarkerCanvas(kId, kSide, kCanvas);
  const auto truth = trueCorners(kSide, kCanvas);
  ArucoDetector refined(/*corner_refine=*/false, /*edge_refine=*/true, 2.2f);
  const auto c = detectOne(refined, sharp, kId);
  ASSERT_EQ(c.size(), 4u);
  for (int k = 0; k < 4; ++k) {
    const cv::Point2f d = c[k] - truth[k];
    EXPECT_LT(std::hypot(d.x, d.y), 0.05) << "corner " << k;
  }
}

TEST(ArucoDetectorTest, EdgeRefineKeepsCornersItCannotFit) {
  // No edges under the quad: the fit must report failure and not touch the
  // corners — the fallback that keeps OpenCV's corners when the marker is
  // too small, too faint or cut by the sub-rectangle.
  cv::Mat flat(120, 120, CV_8UC1, cv::Scalar(128));
  const std::vector<cv::Point2f> quad = {{20, 20}, {100, 20}, {100, 100}, {20, 100}};
  std::vector<cv::Point2f> corners = quad;
  const auto lut = ArucoDetector::makeDecodeLut(2.2f);
  EXPECT_FALSE(ArucoDetector::refineCornersFromEdges(flat, lut, corners));
  EXPECT_EQ(corners, quad);
  // A quad hanging off the image is refused too, not sampled out of bounds.
  std::vector<cv::Point2f> off = {{-30, 20}, {50, 20}, {50, 100}, {-30, 100}};
  const std::vector<cv::Point2f> off_copy = off;
  EXPECT_FALSE(ArucoDetector::refineCornersFromEdges(flat, lut, off));
  EXPECT_EQ(off, off_copy);
}

TEST(ArucoDetectorTest, DecodeLutIsIdentityForGammaOneAndMonotonicOtherwise) {
  const auto one = ArucoDetector::makeDecodeLut(1.0f);
  const auto none = ArucoDetector::makeDecodeLut(0.0f);
  const auto g22 = ArucoDetector::makeDecodeLut(2.2f);
  for (int i = 0; i < 256; ++i) {
    EXPECT_FLOAT_EQ(one[i], static_cast<float>(i));
    EXPECT_FLOAT_EQ(none[i], static_cast<float>(i));
    if (i > 0) EXPECT_GT(g22[i], g22[i - 1]);
  }
  EXPECT_FLOAT_EQ(g22[0], 0.f);
  EXPECT_FLOAT_EQ(g22[255], 255.f);
  EXPECT_LT(g22[128], 64.f);  // mid-grey encoded is dark in linear light
}

}  // namespace
