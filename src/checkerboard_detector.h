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
  //
  // If `corners` is non-null, its contents are replaced with the detected
  // corner coordinates (in the input image's pixel space) on success, or
  // cleared on failure. Pass nullptr to skip the allocation when the caller
  // only cares about the bool.
  bool detect(const uint8_t* image_data, int width, int height,
              size_t stride = 0,
              std::vector<cv::Point2f>* corners = nullptr);

 private:
  cv::Size pattern_size;
};

#endif  // CHECKERBOARD_DETECTOR_H
