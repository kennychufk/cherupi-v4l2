#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <array>
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

  // `corner_refine` selects OpenCV's corner-refinement method: false ⇒
  // CORNER_REFINE_NONE (fastest, raw quad corners; best for real-time on the
  // Pi 5), true ⇒ CORNER_REFINE_SUBPIX (sub-pixel accuracy, slower).
  //
  // `edge_refine` re-derives each marker's corners from its EDGES after
  // OpenCV has found and identified it. Every corner estimate OpenCV offers —
  // the thresholded contour's quad, cornerSubPix — is taken AT the corner,
  // where a blurred L-corner is rounded, and it is taken on the luma the ISP
  // delivers, which is gamma-encoded, so the half-level crossing of a soft
  // edge lands on the dark side of the true edge. Both pull every corner
  // inward: measured on a Camera Module 3 Wide at 4608 px (10-90 % edge width
  // 4-6 px), ~1 px per corner, every ~95 px tag reading ~2 % small and a
  // five-tag PnP left with a 2-3 px systematic residual. The edge fit samples
  // the luma across the middle 60 % of each edge — away from the corners,
  // where a symmetric blur leaves the half-level crossing unbiased — in
  // LINEAR light (the 8-bit luma decoded through `edge_gamma`), fits a line
  // per edge and intersects adjacent lines. On that camera the tags came back
  // to their true size within 0.04 % and the PnP residual fell to 0.85 px.
  // A marker whose fit does not hold (too little contrast, too few edge
  // points, a corner moving more than max(3 px, 10 % of the side)) keeps its
  // OpenCV corners unchanged, so the refinement can only ever act on a
  // marker it can see clearly. Costs microseconds per marker.
  //
  // `edge_gamma` is the decode exponent, luma_linear = 255 * (Y/255)^gamma;
  // 2.2 is what the Pi ISP's output measured as (tags restored to size), 1.0
  // fits the encoded luma as-is. Ignored unless `edge_refine`.
  explicit ArucoDetector(bool corner_refine = false, bool edge_refine = false,
                         float edge_gamma = 2.2f);

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

  // The decode table `edge_gamma` builds: 8-bit encoded luma -> linear light
  // on the same 0..255 scale. gamma <= 0 or == 1 gives the identity.
  static std::array<float, 256> makeDecodeLut(float gamma);

  // The edge refinement on one marker quad (4 corners, clockwise from
  // top-left, in `gray`'s pixel space; `gray` is CV_8UC1). On success the
  // corners are replaced by the edge intersections and true is returned; on
  // any failure they are left untouched and false is returned. Exposed so
  // the tests can exercise it against a known geometry.
  static bool refineCornersFromEdges(const cv::Mat& gray,
                                     const std::array<float, 256>& lut,
                                     std::vector<cv::Point2f>& corners);

 private:
  bool edge_refine_;
  std::array<float, 256> lut_;
#if CHERUPI_ARUCO_NEW_API
  cv::aruco::ArucoDetector detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
#endif
};

#endif  // ARUCO_DETECTOR_H
