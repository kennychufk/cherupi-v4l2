#include "checkerboard_detector.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

CheckerboardDetector::CheckerboardDetector(int board_width, int board_height)
    : pattern_size(board_width, board_height) {}

bool CheckerboardDetector::detect(const uint8_t* image_data, int width,
                                  int height, size_t stride,
                                  std::vector<cv::Point2f>* corners) {
  // Create OpenCV Mat from raw data (no copy). Pass stride through to cv::Mat
  // so a sub-rectangle of a larger buffer can be detected without copying.
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data),
              stride > 0 ? stride : static_cast<size_t>(cv::Mat::AUTO_STEP));

  // Use the caller's vector when provided so we hand back the corners; fall
  // back to a local one (findChessboardCornersSB requires the parameter even
  // when the caller doesn't care).
  std::vector<cv::Point2f> local_corners;
  std::vector<cv::Point2f>& out = corners ? *corners : local_corners;
  out.clear();

  // findChessboardCornersSB uses a different algorithm that's often faster
  // and doesn't automatically refine corners
  // CALIB_CB_EXHAUSTIVE is disabled for performance (especially when the
  // checkerboard is absent)
  const bool detected = cv::findChessboardCornersSB(
      img, pattern_size, out, cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!detected && corners) corners->clear();
  return detected;
}

int CheckerboardDetector::refine(const uint8_t* image_data, int width,
                                 int height, size_t stride,
                                 std::vector<cv::Point2f>& corners,
                                 int max_half_window) const {
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data),
              stride > 0 ? stride : static_cast<size_t>(cv::Mat::AUTO_STEP));
  return refine(img, corners, max_half_window);
}

int CheckerboardDetector::refine(const cv::Mat& img,
                                 std::vector<cv::Point2f>& corners,
                                 int max_half_window) const {
  const int cols = pattern_size.width;
  if (max_half_window <= 0 || corners.empty() || cols <= 0 ||
      corners.size() % static_cast<size_t>(cols) != 0) {
    return 0;
  }

  // Smallest distance between grid neighbours (along rows and columns).
  float min_spacing = std::numeric_limits<float>::max();
  const size_t n = corners.size();
  for (size_t i = 0; i < n; ++i) {
    if ((i + 1) % static_cast<size_t>(cols) != 0) {
      min_spacing = std::min(
          min_spacing, static_cast<float>(cv::norm(corners[i + 1] - corners[i])));
    }
    if (i + cols < n) {
      min_spacing = std::min(
          min_spacing, static_cast<float>(cv::norm(corners[i + cols] - corners[i])));
    }
  }
  const int half = std::min(max_half_window,
                            static_cast<int>(std::floor(0.4f * min_spacing)));
  if (half < 2) return 0;

  cv::cornerSubPix(
      img, corners, cv::Size(half, half), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                       1e-3));
  return half;
}
