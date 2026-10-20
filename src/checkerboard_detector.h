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

  // Refine `corners` in place with cv::cornerSubPix on `image_data`, which
  // must be the full-resolution image the corners are expressed in (not the
  // subsampled one detection may have run on). `corners` must be in the
  // detector's row-major pattern order, as returned by detect().
  //
  // findChessboardCornersSB alone is both noisy and biased: on a stationary
  // board its corners scatter ~0.15 px RMS frame to frame at 2x-subsampled
  // detection, and on defocused corners they sit up to several px off the
  // saddle point. cornerSubPix's gradient fit over a window of edge pixels
  // brings the scatter to ~0.02 px (measured on an IMX477 at 4056x3040).
  //
  // The window half-size is `max_half_window`, shrunk to 40 % of the
  // smallest spacing between neighbouring corners so the window never
  // reaches the next corner on a small or distant board. Returns the
  // half-size used; 0 means nothing was refined (refinement off, or the board
  // too small for a window of at least 2 px).
  int refine(const uint8_t* image_data, int width, int height, size_t stride,
             std::vector<cv::Point2f>& corners, int max_half_window) const;

 private:
  cv::Size pattern_size;
};

#endif  // CHECKERBOARD_DETECTOR_H
