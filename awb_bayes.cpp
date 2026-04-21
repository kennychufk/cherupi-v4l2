#include "awb_bayes.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

#include "frontend/pisp_statistics.h"
#include <nlohmann/json.hpp>

namespace {

// IMX519 AWB tuning (from /usr/share/libcamera/ipa/rpi/pisp/imx519.json).
// Stage 5 replaces these with a generated header. Values below are copied
// verbatim from the `rpi.awb` block.

constexpr double kDefaultCT = 4500.0;

struct CtEntry {
  double ct, r_over_g, b_over_g;
};
constexpr CtEntry kCtCurve[] = {
    {2890.0, 0.7328, 0.3734}, {3550.0, 0.6228, 0.4763},
    {4500.0, 0.5208, 0.5825}, {5700.0, 0.4467, 0.6671},
    {7900.0, 0.3858, 0.7411},
};

struct PriorEntry {
  double lux;
  std::vector<Pwl::Point> points;  // (CT, log-likelihood)
};

// Fallback lux used only when the caller passes lux ≤ 0.
constexpr double kDefaultLux = 400.0;

}  // namespace

AwbBayes::AwbBayes() { loadTuning(); }

void AwbBayes::loadTuning() {
  ct_r_.clear();
  ct_b_.clear();
  for (const auto& e : kCtCurve) {
    ct_r_.append(e.ct, e.r_over_g);
    ct_b_.append(e.ct, e.b_over_g);
  }
  priors_.clear();
  // lux=0: prior skewed warm.
  {
    Pwl p;
    p.append(2000.0, 1.0);
    p.append(3000.0, 0.0);
    p.append(13000.0, 0.0);
    priors_.emplace_back(0.0, std::move(p));
  }
  // lux=800: balanced.
  {
    Pwl p;
    p.append(2000.0, 0.0);
    p.append(6000.0, 2.0);
    p.append(13000.0, 2.0);
    priors_.emplace_back(800.0, std::move(p));
  }
  // lux=1500: daylight-biased.
  {
    Pwl p;
    p.append(2000.0, 0.0);
    p.append(4000.0, 1.0);
    p.append(6000.0, 6.0);
    p.append(6500.0, 7.0);
    p.append(7000.0, 1.0);
    p.append(13000.0, 1.0);
    priors_.emplace_back(1500.0, std::move(p));
  }

  // Seed previous results to something sane around D50.
  prev_cct_ = ct_r_.domain().clamp(4000.0);
  prev_gain_r_ = 1.0 / ct_r_.eval(prev_cct_);
  prev_gain_b_ = 1.0 / ct_b_.eval(prev_cct_);

  loadAlscFromJson("/usr/share/libcamera/ipa/rpi/pisp/imx519.json");
}

void AwbBayes::loadAlscFromJson(const std::string& path) {
  alsc_cr_.clear();
  alsc_cb_.clear();
  alsc_enabled_ = false;

  std::ifstream f(path);
  if (!f.is_open()) return;

  nlohmann::json root;
  try {
    f >> root;
  } catch (...) {
    return;
  }

  auto load_table = [](const nlohmann::json& entry,
                       std::vector<AlscCalib>& out) {
    AlscCalib c;
    c.ct = entry["ct"].get<double>();
    const auto& arr = entry["table"];
    c.table.reserve(arr.size());
    for (const auto& v : arr) c.table.push_back(v.get<float>());
    out.push_back(std::move(c));
  };

  for (const auto& algo : root["algorithms"]) {
    if (!algo.contains("rpi.alsc")) continue;
    const auto& alsc = algo["rpi.alsc"];
    for (const auto& e : alsc["calibrations_Cr"]) load_table(e, alsc_cr_);
    for (const auto& e : alsc["calibrations_Cb"]) load_table(e, alsc_cb_);
    break;
  }

  if (!alsc_cr_.empty() &&
      alsc_cr_[0].table.size() == PISP_AWB_STATS_NUM_ZONES &&
      !alsc_cb_.empty() &&
      alsc_cb_[0].table.size() == PISP_AWB_STATS_NUM_ZONES) {
    alsc_enabled_ = true;
  }
}

void AwbBayes::interpolateAlsc(double ct, std::vector<float>& cr_out,
                               std::vector<float>& cb_out) const {
  // Mirrors libcamera's compensateLambdasForCal: interpolate the calibration
  // table then normalize so the minimum zone correction = 1.0.
  auto interp_and_normalize = [&](const std::vector<AlscCalib>& tables,
                                  std::vector<float>& out) {
    out.resize(PISP_AWB_STATS_NUM_ZONES);
    if (tables.size() == 1 || ct <= tables.front().ct) {
      out = tables.front().table;
    } else if (ct >= tables.back().ct) {
      out = tables.back().table;
    } else {
      size_t i = 0;
      while (i + 1 < tables.size() && tables[i + 1].ct < ct) ++i;
      double t0 = tables[i].ct, t1 = tables[i + 1].ct;
      float alpha = static_cast<float>((ct - t0) / (t1 - t0));
      const auto& a = tables[i].table;
      const auto& b = tables[i + 1].table;
      for (size_t j = 0; j < PISP_AWB_STATS_NUM_ZONES; ++j)
        out[j] = a[j] + alpha * (b[j] - a[j]);
    }
    // Normalize: minimum zone gets gain 1.0, brighter zones get > 1.0.
    float min_val = *std::min_element(out.begin(), out.end());
    if (min_val > 0.0f)
      for (auto& v : out) v /= min_val;
  };

  interp_and_normalize(alsc_cr_, cr_out);
  interp_and_normalize(alsc_cb_, cb_out);
}

void AwbBayes::setConfig(const AwbConfig& cfg) { cfg_ = cfg; }

void AwbBayes::reset() {
  frame_count_ = 0;
  frame_phase_ = 0;
  loadTuning();
  raw_gain_r_ = prev_gain_r_;
  raw_gain_b_ = prev_gain_b_;
  raw_cct_ = prev_cct_;
}

void AwbBayes::stamp(FrameData& frame) {
  frame.awb_gain_r = static_cast<float>(prev_gain_r_);
  frame.awb_gain_b = static_cast<float>(prev_gain_b_);
  frame.awb_cct = static_cast<float>(prev_cct_);
}

void AwbBayes::update(FrameData& frame) {
  // No stats this frame — apply IIR toward last raw estimate and stamp.
  double speed = (frame_count_ < cfg_.warmup_frames) ? 1.0 : cfg_.speed;
  prev_gain_r_ = speed * raw_gain_r_ + (1.0 - speed) * prev_gain_r_;
  prev_gain_b_ = speed * raw_gain_b_ + (1.0 - speed) * prev_gain_b_;
  prev_cct_ = speed * raw_cct_ + (1.0 - speed) * prev_cct_;
  stamp(frame);
}

void AwbBayes::update(FrameData& frame, const pisp_statistics& stats,
                      double lux) {
  if (!cfg_.enabled) {
    stamp(frame);
    return;
  }

  if (lux <= 0.0) lux = kDefaultLux;

  if (frame_count_ < cfg_.warmup_frames) frame_count_++;
  frame_phase_++;

  int period = std::max(1, cfg_.period);
  bool run_estimator = (frame_phase_ >= period) ||
                       (frame_count_ < cfg_.warmup_frames);

  if (run_estimator) {
    frame_phase_ = 0;
    std::vector<float> alsc_cr, alsc_cb;
    if (alsc_enabled_) interpolateAlsc(prev_cct_, alsc_cr, alsc_cb);
    const float* pcr = alsc_enabled_ ? alsc_cr.data() : nullptr;
    const float* pcb = alsc_enabled_ ? alsc_cb.data() : nullptr;
    if (generateZones(stats.awb, pcr, pcb) && zones_.size() > min_regions_) {
      // Divide out G once up front so computeDelta2Sum doesn't repeat it.
      for (auto& z : zones_) {
        z.R = z.R / (z.G + 1.0);
        z.B = z.B / (z.G + 1.0);
      }

      Pwl prior = interpolatePrior(lux);
      double prior_scale =
          static_cast<double>(zones_.size()) /
          static_cast<double>(PISP_AWB_STATS_NUM_ZONES);
      prior *= prior_scale;

      double t = coarseSearch(prior);
      double r = ct_r_.eval(t);
      double b = ct_b_.eval(t);
      fineSearch(t, r, b, prior);

      raw_cct_ = t;
      raw_gain_r_ = (1.0 / r) * sensitivity_r_;
      raw_gain_b_ = (1.0 / b) * sensitivity_b_;
    }
    // If zones insufficient, keep raw_* unchanged (hold last estimate).
  }

  // IIR smoothing. Fast warmup (speed=1) for first N frames, then cfg_.speed.
  double speed = (frame_count_ < cfg_.warmup_frames) ? 1.0 : cfg_.speed;
  prev_gain_r_ = speed * raw_gain_r_ + (1.0 - speed) * prev_gain_r_;
  prev_gain_b_ = speed * raw_gain_b_ + (1.0 - speed) * prev_gain_b_;
  prev_cct_ = speed * raw_cct_ + (1.0 - speed) * prev_cct_;
  stamp(frame);
}

bool AwbBayes::generateZones(const pisp_awb_statistics& stats,
                             const float* alsc_cr, const float* alsc_cb) {
  zones_.clear();
  zones_.reserve(PISP_AWB_STATS_NUM_ZONES);
  for (unsigned int i = 0; i < PISP_AWB_STATS_NUM_ZONES; ++i) {
    const auto& z = stats.zones[i];
    if (z.counted < min_pixels_) continue;
    double G = static_cast<double>(z.G_sum) / z.counted;
    if (G < min_g_) continue;
    double R = static_cast<double>(z.R_sum) / z.counted;
    double B = static_cast<double>(z.B_sum) / z.counted;
    if (alsc_cr) R *= alsc_cr[i];
    if (alsc_cb) B *= alsc_cb[i];
    zones_.push_back({R * sensitivity_r_, G, B * sensitivity_b_});
  }
  return !zones_.empty();
}

double AwbBayes::computeDelta2Sum(double gain_r, double gain_b) const {
  double sum = 0;
  for (const auto& z : zones_) {
    double dR = gain_r * z.R - 1.0 - whitepoint_r_;
    double dB = gain_b * z.B - 1.0 - whitepoint_b_;
    double d2 = dR * dR + dB * dB;
    if (d2 > delta_limit_) d2 = delta_limit_;
    sum += d2;
  }
  return sum;
}

Pwl AwbBayes::interpolatePrior(double lux) const {
  if (lux <= priors_.front().first) return priors_.front().second;
  if (lux >= priors_.back().first) return priors_.back().second;
  size_t idx = 0;
  while (priors_[idx + 1].first < lux) idx++;
  double lux0 = priors_[idx].first;
  double lux1 = priors_[idx + 1].first;
  return Pwl::combine(priors_[idx].second, priors_[idx + 1].second,
                      [&](double /*x*/, double y0, double y1) {
                        return y0 + (y1 - y0) * (lux - lux0) / (lux1 - lux0);
                      });
}

double AwbBayes::coarseSearch(const Pwl& prior) {
  coarse_points_.clear();
  size_t best = 0;
  double t = ct_lo_;
  int spanR = 0, spanB = 0;
  while (true) {
    double r = ct_r_.eval(t, &spanR);
    double b = ct_b_.eval(t, &spanB);
    double d2 = computeDelta2Sum(1.0 / r, 1.0 / b);
    double prior_ll = prior.eval(prior.domain().clamp(t));
    double ll = d2 - prior_ll;
    coarse_points_.emplace_back(t, ll);
    if (coarse_points_.back().second < coarse_points_[best].second)
      best = coarse_points_.size() - 1;
    if (t == ct_hi_) break;
    t = std::min(t + t / 10.0 * coarse_step_, ct_hi_);
  }
  t = coarse_points_[best].first;
  if (coarse_points_.size() > 2) {
    size_t bp = std::min(best, coarse_points_.size() - 2);
    bp = std::max(size_t{1}, bp);
    t = interpolateQuadratic(coarse_points_[bp - 1], coarse_points_[bp],
                             coarse_points_[bp + 1]);
  }
  return t;
}

void AwbBayes::fineSearch(double& t, double& r, double& b,
                          const Pwl& prior) const {
  int spanR = -1, spanB = -1;
  ct_r_.eval(t, &spanR);
  ct_b_.eval(t, &spanB);
  double step = t / 10.0 * coarse_step_ * 0.1;
  int nsteps = 5;
  double rDiff = ct_r_.eval(t + nsteps * step, &spanR) -
                 ct_r_.eval(t - nsteps * step, &spanR);
  double bDiff = ct_b_.eval(t + nsteps * step, &spanB) -
                 ct_b_.eval(t - nsteps * step, &spanB);
  // Transverse unit vector: (bDiff, -rDiff), orthogonal to tangent in (r,b).
  double tx = bDiff, ty = -rDiff;
  double tlen2 = tx * tx + ty * ty;
  if (tlen2 < 1e-6) return;
  double tlen = std::sqrt(tlen2);
  tx /= tlen;
  ty /= tlen;

  double bestLL = 0, bestT = 0, bestR = 0, bestB = 0;
  double tr_range = transverse_neg_ + transverse_pos_;
  constexpr int kMaxNumDeltas = 12;
  int numDeltas = static_cast<int>(std::floor(tr_range * 100.0 + 0.5)) + 1;
  numDeltas = std::clamp(numDeltas, 3, kMaxNumDeltas);
  nsteps += numDeltas;

  for (int i = -nsteps; i <= nsteps; ++i) {
    double tTest = t + i * step;
    double prior_ll = prior.eval(prior.domain().clamp(tTest));
    double rCurve = ct_r_.eval(tTest, &spanR);
    double bCurve = ct_b_.eval(tTest, &spanB);

    Pwl::Point points[kMaxNumDeltas];
    int bp = 0;
    for (int j = 0; j < numDeltas; ++j) {
      double off = -transverse_neg_ + tr_range * j / (numDeltas - 1);
      double rTest = rCurve + tx * off;
      double bTest = bCurve + ty * off;
      double d2 = computeDelta2Sum(1.0 / rTest, 1.0 / bTest);
      points[j] = {off, d2 - prior_ll};
      if (points[j].second < points[bp].second) bp = j;
    }
    bp = std::max(1, std::min(bp, numDeltas - 2));
    double interpOff =
        interpolateQuadratic(points[bp - 1], points[bp], points[bp + 1]);
    double rTest = rCurve + tx * interpOff;
    double bTest = bCurve + ty * interpOff;
    double d2 = computeDelta2Sum(1.0 / rTest, 1.0 / bTest);
    double finalLL = d2 - prior_ll;
    if (bestT == 0 || finalLL < bestLL) {
      bestLL = finalLL;
      bestT = tTest;
      bestR = rTest;
      bestB = bTest;
    }
  }
  t = bestT;
  r = bestR;
  b = bestB;
}

double AwbBayes::interpolateQuadratic(const Pwl::Point& a, const Pwl::Point& b,
                                      const Pwl::Point& c) {
  // Given 3 sample points, return the x of the quadratic extremum,
  // clamped to [a.x, c.x]. Degenerates to picking the lowest-y endpoint.
  constexpr double eps = 1e-3;
  double cax = c.first - a.first, cay = c.second - a.second;
  double bax = b.first - a.first, bay = b.second - a.second;
  double denom = 2.0 * (bay * cax - cay * bax);
  if (std::abs(denom) > eps) {
    double num = bay * cax * cax - cay * bax * bax;
    double result = num / denom + a.first;
    return std::max(a.first, std::min(c.first, result));
  }
  if (a.second < c.second - eps) return a.first;
  if (c.second < a.second - eps) return c.first;
  return b.first;
}
