#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>

// The ArUco marker-rendering call differs across OpenCV versions (see
// src/aruco_detector.h): `drawMarker` (≤4.6) was renamed to
// `generateImageMarker` (≥4.7) and the dictionary changed from
// `cv::Ptr<Dictionary>` to `Dictionary` by value. Keep this guard in sync.
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

namespace aruco_test_utils {

// Render one DICT_APRILTAG_16h5 marker (`id`) at `side` pixels, centered in a
// `canvas`×`canvas` white image so the marker has a quiet zone all around
// (detection needs the white border). Returns a tightly-packed grayscale
// (CV_8UC1) image.
inline cv::Mat makeMarkerCanvas(int id, int side = 80, int canvas = 160) {
  cv::Mat marker;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::Dictionary dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  cv::aruco::generateImageMarker(dict, id, side, marker, /*borderBits=*/1);
#else
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  cv::aruco::drawMarker(dict, id, side, marker, /*borderBits=*/1);
#endif

  cv::Mat img(canvas, canvas, CV_8UC1, cv::Scalar(255));
  const int off = (canvas - side) / 2;
  marker.copyTo(img(cv::Rect(off, off, side, side)));
  return img;
}

}  // namespace aruco_test_utils
