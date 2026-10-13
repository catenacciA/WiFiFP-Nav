#pragma once

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace wififp {

struct StampedPosition {
  double t = 0.0;  ///< seconds
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

/// Time-indexed positions (e.g. a SLAM trajectory) with linear interpolation.
class Trajectory {
 public:
  explicit Trajectory(std::vector<StampedPosition> samples);

  /// Position at time `t`, or nullopt if `t` lies outside the trajectory.
  std::optional<Eigen::Vector3d> positionAt(double t) const;

  const std::vector<StampedPosition>& samples() const { return samples_; }
  double startTime() const { return samples_.front().t; }
  double endTime() const { return samples_.back().t; }
  double length() const;

 private:
  std::vector<StampedPosition> samples_;
};

}  // namespace wififp
