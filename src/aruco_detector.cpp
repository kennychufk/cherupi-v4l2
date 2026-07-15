#include "aruco_detector.h"

#if CHERUPI_ARUCO_NEW_API
namespace {
// Build a configured ArucoDetector (OpenCV 4.7+ API). Used from the ctor
// init-list so we don't rely on a default-constructed detector being reassign-
// able. Dictionary hard-coded to DICT_APRILTAG_16h5 (small 4x4-bit family,
// cheap to identify).
cv::aruco::ArucoDetector buildDetector(bool corner_refine) {
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  cv::aruco::DetectorParameters params;
  // Corner refinement dominates per-frame cost. Default to none for real-time;
  // subpixel is opt-in via the save-mode param for calibration-grade corners.
  params.cornerRefinementMethod = corner_refine
                                      ? cv::aruco::CORNER_REFINE_SUBPIX
                                      : cv::aruco::CORNER_REFINE_NONE;
  return cv::aruco::ArucoDetector(dictionary, params);
}
}  // namespace

ArucoDetector::ArucoDetector(bool corner_refine)
    : detector_(buildDetector(corner_refine)) {}
#else
ArucoDetector::ArucoDetector(bool corner_refine) {
  // Hard-coded dictionary for now (see CLAUDE.md / protocol doc). AprilTag
  // 16h5 is a small 4x4-bit family (30 ids) that is cheap to identify.
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);

  params_ = cv::aruco::DetectorParameters::create();
  // Corner refinement dominates per-frame cost. Default to none for real-time;
  // subpixel is opt-in via the save-mode param for calibration-grade corners.
  params_->cornerRefinementMethod = corner_refine
                                        ? cv::aruco::CORNER_REFINE_SUBPIX
                                        : cv::aruco::CORNER_REFINE_NONE;
}
#endif

bool ArucoDetector::detect(const uint8_t* image_data, int width, int height,
                           size_t stride, std::vector<Marker>* markers) {
  // Wrap the raw grayscale data (no copy). detectMarkers accepts a
  // single-channel image directly, so the Y plane needs no color conversion.
  // A non-zero stride views a sub-rectangle of a larger buffer.
  cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(image_data),
              stride > 0 ? stride : static_cast<size_t>(cv::Mat::AUTO_STEP));

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
#if CHERUPI_ARUCO_NEW_API
  detector_.detectMarkers(img, corners, ids);
#else
  cv::aruco::detectMarkers(img, dictionary_, corners, ids, params_);
#endif

  if (markers) {
    markers->clear();
    markers->reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      Marker m;
      m.id = ids[i];
      m.corners = std::move(corners[i]);
      markers->push_back(std::move(m));
    }
  }
  return !ids.empty();
}
