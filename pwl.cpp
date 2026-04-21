#include "pwl.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

void Pwl::append(double x, double y, double eps) {
  if (points_.empty() || points_.back().first + eps < x)
    points_.emplace_back(x, y);
}

void Pwl::prepend(double x, double y, double eps) {
  if (points_.empty() || points_.front().first - eps > x)
    points_.insert(points_.begin(), Point{x, y});
}

Pwl::Interval Pwl::domain() const {
  return Interval(points_.front().first, points_.back().first);
}

Pwl::Interval Pwl::range() const {
  double lo = points_.front().second;
  double hi = lo;
  for (auto& p : points_) {
    lo = std::min(lo, p.second);
    hi = std::max(hi, p.second);
  }
  return Interval(lo, hi);
}

int Pwl::findSpan(double x, int span) const {
  int lastSpan = static_cast<int>(points_.size()) - 2;
  span = std::max(0, std::min(lastSpan, span));
  while (span < lastSpan && x >= points_[span + 1].first) span++;
  while (span && x < points_[span].first) span--;
  return span;
}

double Pwl::eval(double x, int* span, bool updateSpan) const {
  int index = findSpan(
      x, span && *span != -1 ? *span
                             : static_cast<int>(points_.size()) / 2 - 1);
  if (span && updateSpan) *span = index;

  if (points_.size() == 1) return points_[0].second;

  const Point& p0 = points_[index];
  const Point& p1 = points_[index + 1];
  return p0.second +
         (x - p0.first) * (p1.second - p0.second) / (p1.first - p0.first);
}

std::pair<Pwl, bool> Pwl::inverse(double eps) const {
  bool appended = false, prepended = false, neither = false;
  Pwl inv;

  for (const Point& p : points_) {
    if (inv.empty()) {
      inv.append(p.second, p.first, eps);
    } else if (std::abs(inv.points_.back().first - p.second) <= eps ||
               std::abs(inv.points_.front().first - p.second) <= eps) {
      /* skip duplicate */;
    } else if (p.second > inv.points_.back().first) {
      inv.append(p.second, p.first, eps);
      appended = true;
    } else if (p.second < inv.points_.front().first) {
      inv.prepend(p.second, p.first, eps);
      prepended = true;
    } else {
      neither = true;
    }
  }

  bool trueInverse = !(neither || (appended && prepended));
  return {inv, trueInverse};
}

Pwl Pwl::compose(const Pwl& other, double eps) const {
  double thisX = points_[0].first, thisY = points_[0].second;
  int thisSpan = 0, otherSpan = other.findSpan(thisY, 0);
  Pwl result({Point{thisX, other.eval(thisY, &otherSpan, false)}});

  while (thisSpan != static_cast<int>(points_.size()) - 1) {
    double dx = points_[thisSpan + 1].first - points_[thisSpan].first;
    double dy = points_[thisSpan + 1].second - points_[thisSpan].second;
    if (std::abs(dy) > eps &&
        otherSpan + 1 < static_cast<int>(other.points_.size()) &&
        points_[thisSpan + 1].second >=
            other.points_[otherSpan + 1].first + eps) {
      thisX = points_[thisSpan].first +
              (other.points_[otherSpan + 1].first - points_[thisSpan].second) *
                  dx / dy;
      thisY = other.points_[++otherSpan].first;
    } else if (std::abs(dy) > eps && otherSpan > 0 &&
               points_[thisSpan + 1].second <=
                   other.points_[otherSpan - 1].first - eps) {
      thisX = points_[thisSpan].first +
              (other.points_[otherSpan + 1].first - points_[thisSpan].second) *
                  dx / dy;
      thisY = other.points_[--otherSpan].first;
    } else {
      thisSpan++;
      thisX = points_[thisSpan].first;
      thisY = points_[thisSpan].second;
    }
    result.append(thisX, other.eval(thisY, &otherSpan, false), eps);
  }
  return result;
}

void Pwl::map(std::function<void(double x, double y)> f) const {
  for (auto& p : points_) f(p.first, p.second);
}

void Pwl::map2(const Pwl& a, const Pwl& b,
               std::function<void(double, double, double)> f) {
  int spanA = 0, spanB = 0;
  double x = std::min(a.points_[0].first, b.points_[0].first);
  f(x, a.eval(x, &spanA, false), b.eval(x, &spanB, false));

  while (spanA < static_cast<int>(a.points_.size()) - 1 ||
         spanB < static_cast<int>(b.points_.size()) - 1) {
    if (spanA == static_cast<int>(a.points_.size()) - 1)
      x = b.points_[++spanB].first;
    else if (spanB == static_cast<int>(b.points_.size()) - 1)
      x = a.points_[++spanA].first;
    else if (a.points_[spanA + 1].first > b.points_[spanB + 1].first)
      x = b.points_[++spanB].first;
    else
      x = a.points_[++spanA].first;
    f(x, a.eval(x, &spanA, false), b.eval(x, &spanB, false));
  }
}

Pwl Pwl::combine(const Pwl& a, const Pwl& b,
                 std::function<double(double, double, double)> f, double eps) {
  Pwl result;
  map2(a, b,
       [&](double x, double y0, double y1) { result.append(x, f(x, y0, y1), eps); });
  return result;
}

Pwl& Pwl::operator*=(double d) {
  for (auto& p : points_) p.second *= d;
  return *this;
}

std::string Pwl::toString() const {
  std::stringstream ss;
  ss << "Pwl { ";
  for (auto& p : points_) ss << "(" << p.first << ", " << p.second << ") ";
  ss << "}";
  return ss.str();
}
