#include "checkerboard_detector.h"

#include <iostream>

CheckerboardDetector::CheckerboardDetector(int board_width, int board_height)
    : pattern_size(board_width, board_height) {}

bool CheckerboardDetector::detect(const uint8_t* image_data, int width,
                                  int height, size_t stride) {
  // Create OpenCV Mat from raw data (no copy). Pass stride through to cv::Mat
  // so a sub-rectangle of a larger buffer can be detected without copying.
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data),
              stride > 0 ? stride : static_cast<size_t>(cv::Mat::AUTO_STEP));

  // Storage for corner points (we don't need them, but API requires it)
  std::vector<cv::Point2f> corners;

  // findChessboardCornersSB uses a different algorithm that's often faster
  // and doesn't automatically refine corners
  // CALIB_CB_EXHAUSTIVE is disabled for performance (especially when the
  // checkerboard is absent)
  return cv::findChessboardCornersSB(img, pattern_size, corners,
                                     cv::CALIB_CB_NORMALIZE_IMAGE);
}
