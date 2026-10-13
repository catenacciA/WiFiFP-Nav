#include "wififp/trajectory.hpp"

#include <algorithm>
#include <stdexcept>

namespace wififp {

Trajectory::Trajectory(std::vector<StampedPosition> samples) : samples_(std::move(samples)) {
  if (samples_.empty()) throw std::invalid_argument("Trajectory: no samples");
  std::stable_sort(samples_.begin(), samples_.end(),
                   [](const StampedPosition& a, const StampedPosition& b) { return a.t < b.t; });
}

std::optional<Eigen::Vector3d> Trajectory::positionAt(double t) const {
  if (t < startTime() || t > endTime()) return std::nullopt;
  auto hi = std::lower_bound(samples_.begin(), samples_.end(), t,
                             [](const StampedPosition& s, double v) { return s.t < v; });
  if (hi == samples_.begin()) return hi->p;
  auto lo = std::prev(hi);
  const double span = hi->t - lo->t;
  if (span <= 0.0) return hi->p;
  const double a = (t - lo->t) / span;
  return Eigen::Vector3d((1.0 - a) * lo->p + a * hi->p);
}

double Trajectory::length() const {
  double len = 0.0;
  for (size_t i = 1; i < samples_.size(); ++i) len += (samples_[i].p - samples_[i - 1].p).norm();
  return len;
}

}  // namespace wififp
