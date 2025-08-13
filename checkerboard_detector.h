#ifndef CHECKERBOARD_DETECTOR_H
#define CHECKERBOARD_DETECTOR_H

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vector>

class CheckerboardDetector {
 public:
  CheckerboardDetector(int board_width = 11, int board_height = 8);

  // Detect checkerboard in grayscale image data
  bool detect(const uint8_t* image_data, int width, int height);

 private:
  cv::Size pattern_size;
};

#endif  // CHECKERBOARD_DETECTOR_H
