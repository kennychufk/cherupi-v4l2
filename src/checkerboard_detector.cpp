#include "checkerboard_detector.h"

#include <iostream>

CheckerboardDetector::CheckerboardDetector(int board_width, int board_height)
    : pattern_size(board_width, board_height) {}

bool CheckerboardDetector::detect(const uint8_t* image_data, int width,
                                  int height) {
  // Create OpenCV Mat from raw data (no copy)
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data));

  // Storage for corner points (we don't need them, but API requires it)
  std::vector<cv::Point2f> corners;

  // findChessboardCornersSB uses a different algorithm that's often faster
  // and doesn't automatically refine corners
  // CALIB_CB_EXHAUSTIVE is disabled for performance (especially when the
  // checkerboard is absent)
  return cv::findChessboardCornersSB(img, pattern_size, corners,
                                     cv::CALIB_CB_NORMALIZE_IMAGE);
}
