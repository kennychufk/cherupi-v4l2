#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "frontend/pisp_statistics.h"
#include "pwl.hpp"
#include "types.hpp"

// Bayesian AWB estimator driven by PiSP Frontend hardware stats.
//
// Port of libcamera's RPiController::AwbBayes (see
// src/ipa/rpi/controller/rpi/{awb.cpp,awb_bayes.cpp}), stripped of the
// libcamera Controller/Metadata plumbing. Runs synchronously on the
// capture thread — A76 measures ~50 µs per call at 32x32 zones.
class AwbBayes {
 public:
  AwbBayes();

  void setConfig(const AwbConfig& cfg);
  void reset();

  // No stats this frame (FE warmup or one-frame lag): stamp previous
  // smoothed gains onto `frame`.
  void update(FrameData& frame);

  // Full Bayesian update given fresh PiSP statistics and scene lux.
  // Runs the estimator every `cfg_.period` frames; other frames just re-stamp.
  // `lux` must be > 0; if ≤ 0 a safe default (400 lux) is used.
  void update(FrameData& frame, const pisp_statistics& stats, double lux);

 private:
  struct Zone {
    double R, G, B;
  };

  // Per-CT lens-shading calibration table (1024 entries = 32x32 zones).
  struct AlscCalib {
    double ct;
    std::vector<float> table;
  };

  AwbConfig cfg_;

  // Tuning (IMX519 defaults — Stage 5 will generate these from imx519.json).
  Pwl ct_r_;        // CT -> R/G
  Pwl ct_b_;        // CT -> B/G
  double ct_lo_ = 2500.0;
  double ct_hi_ = 7900.0;
  double min_pixels_ = 16.0;
  uint16_t min_g_ = 32;
  uint32_t min_regions_ = 10;
  double coarse_step_ = 0.2;
  double whitepoint_r_ = 0.0;
  double whitepoint_b_ = 0.0;
  double transverse_pos_ = 0.02027;
  double transverse_neg_ = 0.01935;
  double sensitivity_r_ = 1.0;
  double sensitivity_b_ = 1.0;
  double delta_limit_ = 0.2;
  std::vector<std::pair<double, Pwl>> priors_;  // sorted by lux

  // ALSC calibration tables loaded from imx519.json (two CT points each).
  std::vector<AlscCalib> alsc_cr_;
  std::vector<AlscCalib> alsc_cb_;
  bool alsc_enabled_ = false;

  // State
  int frame_count_ = 0;
  int frame_phase_ = 0;
  double prev_gain_r_ = 1.0;
  double prev_gain_b_ = 1.0;
  double prev_cct_ = 4500.0;
  double raw_gain_r_ = 1.0;
  double raw_gain_b_ = 1.0;
  double raw_cct_ = 4500.0;
  std::vector<Zone> zones_;
  std::vector<Pwl::Point> coarse_points_;

  void loadTuning();
  void loadAlscFromJson(const std::string& path);
  // Interpolate ALSC tables at `ct`; outputs point to PISP_AWB_STATS_NUM_ZONES
  // floats each. nullptr inputs mean ALSC is disabled.
  void interpolateAlsc(double ct, std::vector<float>& cr_out,
                       std::vector<float>& cb_out) const;
  bool generateZones(const pisp_awb_statistics& stats, const float* alsc_cr,
                     const float* alsc_cb);
  double computeDelta2Sum(double gain_r, double gain_b) const;
  Pwl interpolatePrior(double lux) const;
  double coarseSearch(const Pwl& prior);
  void fineSearch(double& t, double& r, double& b, const Pwl& prior) const;
  static double interpolateQuadratic(const Pwl::Point& a, const Pwl::Point& b,
                                     const Pwl::Point& c);
  void stamp(FrameData& frame);
};
