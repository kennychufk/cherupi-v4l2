#ifndef CHECKERBOARD_DETECTOR_H
#define CHECKERBOARD_DETECTOR_H

#include <cstdint>
#include <opencv2/opencv.hpp>

class CheckerboardDetector {
 public:
  CheckerboardDetector(int board_width = 11, int board_height = 8);

  // Detect checkerboard in grayscale image data
  bool detect(const uint8_t* image_data, int width, int height);

  // Get detection parameters for tuning
  void setAdaptiveThreshWinSize(int size) { adaptive_thresh_win_size = size; }
  void setAdaptiveThreshC(double c) { adaptive_thresh_c = c; }
  void setNormalizeImage(bool normalize) { normalize_image = normalize; }
  void setFastCheck(bool fast) { fast_check = fast; }

 private:
  cv::Size pattern_size;
  int flags;

  // Detection parameters optimized for RPi5
  int adaptive_thresh_win_size;
  double adaptive_thresh_c;
  bool normalize_image;
  bool fast_check;
};

#endif  // CHECKERBOARD_DETECTOR_H
