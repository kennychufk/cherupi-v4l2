#ifndef CHECKERBOARD_DETECTOR_H
#define CHECKERBOARD_DETECTOR_H

#include <cstddef>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vector>

class CheckerboardDetector {
 public:
  CheckerboardDetector(int board_width = 11, int board_height = 8);

  // Detect checkerboard in grayscale image data. `stride` is bytes-per-row;
  // 0 means tightly packed (stride == width). A non-zero stride lets callers
  // view a sub-rectangle of a larger buffer without copying.
  bool detect(const uint8_t* image_data, int width, int height,
              size_t stride = 0);

 private:
  cv::Size pattern_size;
};

#endif  // CHECKERBOARD_DETECTOR_H
