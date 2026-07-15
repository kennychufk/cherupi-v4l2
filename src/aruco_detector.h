#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>

// The ArUco API changed incompatibly in OpenCV 4.7: the module moved into
// `objdetect`, `getPredefinedDictionary` returns a `Dictionary` by value (was
// `cv::Ptr<Dictionary>`), `DetectorParameters` became default-constructible
// (dropping the static `create()`), and detection moved onto the
// `cv::aruco::ArucoDetector` class. We support both so the server builds on the
// 4.6 dev host and the deployed Pi (4.7+). Keep the guard in sync with the one
// in aruco_detector.cpp and the aruco test helper.
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define CHERUPI_ARUCO_NEW_API 1
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#define CHERUPI_ARUCO_NEW_API 0
#include <opencv2/aruco.hpp>
#endif

// Detects ArUco/AprilTag markers in a grayscale image. The dictionary is
// hard-coded to DICT_APRILTAG_16h5. The dictionary and detector parameters are
// built once at construction and reused across frames; detection reads them
// read-only, so a single instance is safe to call concurrently from multiple
// threads (e.g. the four quadrant jobs in aruco2x2).
class ArucoDetector {
 public:
  // One detected marker: its dictionary id plus its 4 corners in the input
  // image's pixel space (dictionary canonical order — clockwise from top-left).
  struct Marker {
    int id = -1;
    std::vector<cv::Point2f> corners;
  };

  // `corner_refine` selects the corner-refinement method: false ⇒
  // CORNER_REFINE_NONE (fastest, raw quad corners; best for real-time on the
  // Pi 5), true ⇒ CORNER_REFINE_SUBPIX (sub-pixel accuracy, slower).
  explicit ArucoDetector(bool corner_refine = false);

  // Detect markers in single-channel 8-bit image data. `stride` is
  // bytes-per-row; 0 means tightly packed (stride == width). A non-zero stride
  // lets callers view a sub-rectangle of a larger buffer without copying (used
  // by the 2x2 quadrant split).
  //
  // If `markers` is non-null, its contents are replaced with the detected
  // markers (empty on no detection). Returns true iff at least one marker was
  // found.
  bool detect(const uint8_t* image_data, int width, int height,
              size_t stride = 0, std::vector<Marker>* markers = nullptr);

 private:
#if CHERUPI_ARUCO_NEW_API
  cv::aruco::ArucoDetector detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
#endif
};

#endif  // ARUCO_DETECTOR_H
