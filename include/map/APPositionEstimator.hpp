#ifndef AP_POSITION_ESTIMATOR_HPP
#define AP_POSITION_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <map>
#include <string>
#include <vector>

#include "Fingerprinting.hpp"

namespace gtsam {
class CustomRangeFactor : public NoiseModelFactor1<Point3> {
private:
  Point3 knownPoint_;
  double measuredDistance_;

public:
  CustomRangeFactor(Key key, const Point3 &knownPoint, double measuredDistance,
                    const SharedNoiseModel &model);

  Vector
  evaluateError(const Point3 &estimate,
                boost::optional<Matrix &> H = boost::none) const override;
};
} // namespace gtsam

class APPositionEstimator {
public:
  struct APPosition {
    std::string ssid;
    gtsam::Point3 position;
    Eigen::Matrix3d covariance;

    // Constructor to initialize all member variables
    APPosition(const std::string &ssid, const gtsam::Point3 &position,
               const Eigen::Matrix3d &covariance);
  };

  APPositionEstimator(const std::vector<FingerprintingRecord> &records);

  void estimatePositions();
  const std::map<std::string, APPosition> &getEstimatedPositions() const;
  void saveResultsToFile(const std::string &filename) const;

private:
  std::vector<FingerprintingRecord> records_;
  std::map<std::string, APPosition> estimatedPositions_;

  double rssiToDistance(double frequency, double signal_dbm) const;
  void optimizeAPPosition(const std::string &macAddress,
                          std::vector<FingerprintingRecord> apRecords);
  Eigen::Matrix3d
  calculateCovarianceMatrix(const std::vector<gtsam::Point3> &knownPoints,
                            const gtsam::Point3 &estimatedPosition);

  gtsam::Point3
  computeAveragePosition(const std::vector<gtsam::Point3> &knownPoints) const;
};

#endif // AP_POSITION_ESTIMATOR_HPP
