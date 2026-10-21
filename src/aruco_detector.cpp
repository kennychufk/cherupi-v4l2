#include "aruco_detector.h"

#include <algorithm>
#include <cmath>

namespace {

// --- the edge fit -----------------------------------------------------------

constexpr int kLinesPerEdge = 31;      // sample lines across each edge...
constexpr float kAlongLo = 0.2f;       // ...over its middle 60 %, clear of the
constexpr float kAlongHi = 0.8f;       //    rounded corners
constexpr float kStepPx = 0.25f;       // spacing of the samples across an edge
constexpr int kMinEdgePoints = 6;      // crossings a line fit needs

// Bilinear samplers for the edge fit, interpolating in linear light. False
// outside the image (the 2x2 quadrant views end at their sub-rectangle).
//
// LutSampler reads 8-bit encoded luma, decoding each neighbour through `lut`
// before interpolating.
struct LutSampler {
  const cv::Mat& img;  // CV_8UC1
  const std::array<float, 256>& lut;
  bool operator()(float x, float y, float* out) const {
    if (!(x >= 0.f && y >= 0.f && x < static_cast<float>(img.cols - 1) &&
          y < static_cast<float>(img.rows - 1)))
      return false;
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);
    const uint8_t* r0 = img.ptr<uint8_t>(y0);
    const uint8_t* r1 = img.ptr<uint8_t>(y0 + 1);
    const float top = lut[r0[x0]] * (1.f - fx) + lut[r0[x0 + 1]] * fx;
    const float bot = lut[r1[x0]] * (1.f - fx) + lut[r1[x0 + 1]] * fx;
    *out = top * (1.f - fy) + bot * fy;
    return true;
  }
};

// FloatSampler reads an image that is already linear (CV_32FC1).
struct FloatSampler {
  const cv::Mat& img;  // CV_32FC1
  bool operator()(float x, float y, float* out) const {
    if (!(x >= 0.f && y >= 0.f && x < static_cast<float>(img.cols - 1) &&
          y < static_cast<float>(img.rows - 1)))
      return false;
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);
    const float* r0 = img.ptr<float>(y0);
    const float* r1 = img.ptr<float>(y0 + 1);
    const float top = r0[x0] * (1.f - fx) + r0[x0 + 1] * fx;
    const float bot = r1[x0] * (1.f - fx) + r1[x0 + 1] * fx;
    *out = top * (1.f - fy) + bot * fy;
    return true;
  }
};

struct Line {
  cv::Point2f p;  // a point on the line
  cv::Point2f d;  // unit direction
};

// Total-least-squares line: the principal axis of the points' covariance.
bool fitLine(const std::vector<cv::Point2f>& pts, Line* out) {
  if (pts.size() < 2) return false;
  double cx = 0.0, cy = 0.0;
  for (const auto& p : pts) {
    cx += p.x;
    cy += p.y;
  }
  cx /= static_cast<double>(pts.size());
  cy /= static_cast<double>(pts.size());
  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (const auto& p : pts) {
    const double dx = p.x - cx, dy = p.y - cy;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  const double tr = sxx + syy;
  const double det = sxx * syy - sxy * sxy;
  const double l1 = 0.5 * tr + std::sqrt(std::max(0.0, 0.25 * tr * tr - det));
  cv::Point2d d;
  if (std::fabs(sxy) > 1e-12)
    d = cv::Point2d(l1 - syy, sxy);
  else
    d = sxx >= syy ? cv::Point2d(1.0, 0.0) : cv::Point2d(0.0, 1.0);
  const double n = std::hypot(d.x, d.y);
  if (n < 1e-12) return false;
  out->p = cv::Point2f(static_cast<float>(cx), static_cast<float>(cy));
  out->d = cv::Point2f(static_cast<float>(d.x / n), static_cast<float>(d.y / n));
  return true;
}

inline float cross2(const cv::Point2f& a, const cv::Point2f& b) {
  return a.x * b.y - a.y * b.x;
}

// One edge of the black square, from corner `a` to corner `b`, with the
// marker's centre `ctr` telling which side is inside. At each of the sample
// lines the luma profile is read across the edge, from `reach` px inside to
// `reach` px outside; the black and white plateaus are the profile's two
// ends and the edge point is the first crossing of their mid level. A line is
// fit through the crossings, trimmed of the odd stray (a bolt, a shadow,
// the plate's rim) and refit.
template <class Sampler>
bool fitEdge(const Sampler& sample, const cv::Point2f& a, const cv::Point2f& b,
             const cv::Point2f& ctr, float reach, Line* out) {
  cv::Point2f e = b - a;
  const float len = std::hypot(e.x, e.y);
  if (len < 4.f) return false;
  e *= 1.f / len;
  cv::Point2f n(-e.y, e.x);
  if (n.dot(ctr - a) > 0.f) n = -n;  // outward
  const int half = std::max(2, static_cast<int>(std::lround(reach / kStepPx)));
  const int n_prof = 2 * half + 1;
  const int plateau = std::max(2, n_prof / 4);
  std::vector<float> prof(static_cast<size_t>(n_prof));
  std::vector<cv::Point2f> pts;
  pts.reserve(kLinesPerEdge);
  for (int s = 0; s < kLinesPerEdge; ++s) {
    const float frac =
        kAlongLo + (kAlongHi - kAlongLo) * static_cast<float>(s) /
                       static_cast<float>(kLinesPerEdge - 1);
    const cv::Point2f p = a + e * (frac * len);
    bool ok = true;
    for (int i = 0; i < n_prof && ok; ++i) {
      const float t = static_cast<float>(i - half) * kStepPx;
      ok = sample(p.x + n.x * t, p.y + n.y * t, &prof[i]);
    }
    if (!ok) continue;
    float blk = 0.f, wht = 0.f;
    for (int i = 0; i < plateau; ++i) {
      blk += prof[i];
      wht += prof[n_prof - 1 - i];
    }
    blk /= static_cast<float>(plateau);
    wht /= static_cast<float>(plateau);
    if (wht - blk < std::max(4.f, 0.15f * wht)) continue;  // no edge here
    const float mid = 0.5f * (blk + wht);
    for (int i = 0; i + 1 < n_prof; ++i) {
      if (prof[i] < mid && prof[i + 1] >= mid) {
        const float f = (mid - prof[i]) / (prof[i + 1] - prof[i]);
        const float t = (static_cast<float>(i - half) + f) * kStepPx;
        pts.push_back(p + n * t);
        break;
      }
    }
  }
  if (pts.size() < static_cast<size_t>(kMinEdgePoints)) return false;
  Line first;
  if (!fitLine(pts, &first)) return false;
  std::vector<float> res(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    res[i] = std::fabs(cross2(pts[i] - first.p, first.d));
  std::vector<float> sorted = res;
  std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2,
                   sorted.end());
  const float thr = std::max(0.3f, 3.f * sorted[sorted.size() / 2]);
  std::vector<cv::Point2f> kept;
  kept.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    if (res[i] <= thr) kept.push_back(pts[i]);
  if (kept.size() < static_cast<size_t>(kMinEdgePoints)) return false;
  return fitLine(kept, out);
}

bool intersect(const Line& l1, const Line& l2, cv::Point2f* out) {
  // l1.p + s * l1.d == l2.p + t * l2.d
  const float det = cross2(l1.d, l2.d);
  if (std::fabs(det) < 1e-6f) return false;  // parallel: not a corner
  const cv::Point2f r = l2.p - l1.p;
  const float s = cross2(r, l2.d) / det;
  *out = l1.p + l1.d * s;
  return true;
}

}  // namespace

std::array<float, 256> ArucoDetector::makeDecodeLut(float gamma) {
  std::array<float, 256> lut{};
  const bool identity = !(gamma > 0.f) || std::fabs(gamma - 1.f) < 1e-6f;
  for (int i = 0; i < 256; ++i) {
    const float v = static_cast<float>(i);
    lut[static_cast<size_t>(i)] =
        identity ? v : 255.f * std::pow(v / 255.f, gamma);
  }
  return lut;
}

namespace {

// The edge refinement on one quad, reading the image through `sample`.
template <class Sampler>
bool refineQuad(const Sampler& sample, std::vector<cv::Point2f>& corners) {
  if (corners.size() != 4) return false;
  cv::Point2f ctr(0.f, 0.f);
  float side = 0.f;
  for (int k = 0; k < 4; ++k) {
    ctr += corners[static_cast<size_t>(k)];
    const cv::Point2f d =
        corners[static_cast<size_t>((k + 1) % 4)] - corners[static_cast<size_t>(k)];
    side += std::hypot(d.x, d.y);
  }
  ctr *= 0.25f;
  side *= 0.25f;
  if (side < 12.f) return false;  // a border ring too thin to sample inside
  // Reach half-way into the marker's border ring (one cell of six for
  // 16h5), and as far into the quiet zone.
  const float reach = std::clamp(side / 12.f, 2.f, 10.f);
  Line lines[4];
  for (int k = 0; k < 4; ++k) {
    if (!fitEdge(sample, corners[static_cast<size_t>(k)],
                 corners[static_cast<size_t>((k + 1) % 4)], ctr, reach,
                 &lines[k]))
      return false;
  }
  std::vector<cv::Point2f> out(4);
  // How far a corner may move. OpenCV's corner is not merely biased: next
  // to a dark feature (a table hole beside one tag's corner, measured) the
  // thresholded contour cuts the corner and the quad's vertex sits 6-7 px
  // inside on a 96 px tag, so a tight limit refuses the repair exactly where
  // it is needed. The edges themselves are what the fit trusts; the limit
  // only rejects a fit that landed on some other square.
  const float max_shift = std::max(3.f, 0.10f * side);
  for (int k = 0; k < 4; ++k) {
    // corner k is where edge k-1 (into it) meets edge k (out of it)
    if (!intersect(lines[(k + 3) % 4], lines[k], &out[static_cast<size_t>(k)]))
      return false;
    const cv::Point2f d = out[static_cast<size_t>(k)] - corners[static_cast<size_t>(k)];
    if (std::hypot(d.x, d.y) > max_shift) return false;
  }
  corners = std::move(out);
  return true;
}

}  // namespace

bool ArucoDetector::refineCornersFromEdges(const cv::Mat& gray,
                                           const std::array<float, 256>& lut,
                                           std::vector<cv::Point2f>& corners) {
  if (gray.type() != CV_8UC1 || gray.empty()) return false;
  return refineQuad(LutSampler{gray, lut}, corners);
}

bool ArucoDetector::refineCornersFromEdgesLinear(
    const cv::Mat& linear, std::vector<cv::Point2f>& corners) {
  if (linear.type() != CV_32FC1 || linear.empty()) return false;
  return refineQuad(FloatSampler{linear}, corners);
}

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

ArucoDetector::ArucoDetector(bool corner_refine, bool edge_refine,
                             float edge_gamma)
    : edge_refine_(edge_refine),
      lut_(makeDecodeLut(edge_gamma)),
      detector_(buildDetector(corner_refine)) {}
#else
ArucoDetector::ArucoDetector(bool corner_refine, bool edge_refine,
                             float edge_gamma)
    : edge_refine_(edge_refine), lut_(makeDecodeLut(edge_gamma)) {
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

  if (edge_refine_) {
    // OpenCV's corners located and identified the marker; its edges now say
    // where the corners are. A quad the fit cannot hold keeps OpenCV's.
    for (auto& quad : corners) refineCornersFromEdges(img, lut_, quad);
  }

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
