#include "checkerboard_detector.h"

#include <iostream>

CheckerboardDetector::CheckerboardDetector(int board_width, int board_height)
    : pattern_size(board_width, board_height),
      adaptive_thresh_win_size(11),
      adaptive_thresh_c(2),
      normalize_image(true),
      fast_check(true) {
  // Set optimized flags for RPi5
  flags = cv::CALIB_CB_ADAPTIVE_THRESH;

  if (normalize_image) {
    flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
  }

  if (fast_check) {
    flags |= cv::CALIB_CB_FAST_CHECK;
  }
}

bool CheckerboardDetector::detect(const uint8_t* image_data, int width,
                                  int height) {
  // Create OpenCV Mat from raw data (no copy)
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data));

  // Pre-process for better detection on RPi5
  cv::Mat processed;

  // Apply histogram equalization for better contrast
  cv::equalizeHist(img, processed);

  // Optional: Apply Gaussian blur to reduce noise
  // This can help with noisy images but adds processing time
  // cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.0);

  // Storage for corner points (we don't need them, but API requires it)
  std::vector<cv::Point2f> corners;

  // Set corner refinement criteria for speed
  cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                            0.01);

  // Perform detection
  bool found =
      cv::findChessboardCorners(processed, pattern_size, corners, flags);

  // If found and we want more accuracy, we could refine corners
  // But since we only need true/false, we skip this step for speed
  /*
  if (found) {
      cv::cornerSubPix(processed, corners, cv::Size(11, 11), cv::Size(-1, -1),
  criteria);
  }
  */

  return found;
}
