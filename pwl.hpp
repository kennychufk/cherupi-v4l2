#pragma once

// Piecewise-linear function over (x, y) control points. Ported from
// libcamera src/ipa/libipa/pwl.{h,cpp} (BSD-2-Clause, (C) Raspberry Pi Ltd),
// stripped of the libcamera Vector/YAML dependencies.

#include <functional>
#include <string>
#include <utility>
#include <vector>

class Pwl {
 public:
  using Point = std::pair<double, double>;

  struct Interval {
    Interval(double s, double e) : start(s), end(e) {}
    bool contains(double v) const { return v >= start && v <= end; }
    double clamp(double v) const {
      return v < start ? start : (v > end ? end : v);
    }
    double length() const { return end - start; }
    double start, end;
  };

  Pwl() = default;
  explicit Pwl(std::vector<Point> points) : points_(std::move(points)) {}

  void append(double x, double y, double eps = 1e-6);

  bool empty() const { return points_.empty(); }
  void clear() { points_.clear(); }
  size_t size() const { return points_.size(); }

  Interval domain() const;
  Interval range() const;

  double eval(double x, int* span = nullptr, bool updateSpan = true) const;

  std::pair<Pwl, bool> inverse(double eps = 1e-6) const;
  Pwl compose(const Pwl& other, double eps = 1e-6) const;

  void map(std::function<void(double x, double y)> f) const;

  static Pwl combine(
      const Pwl& a, const Pwl& b,
      std::function<double(double x, double y0, double y1)> f,
      double eps = 1e-6);

  Pwl& operator*=(double d);

  std::string toString() const;

 private:
  static void map2(const Pwl& a, const Pwl& b,
                   std::function<void(double x, double y0, double y1)> f);
  void prepend(double x, double y, double eps = 1e-6);
  int findSpan(double x, int span) const;

  std::vector<Point> points_;
};
