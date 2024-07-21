#include "../include/WiFiMeasurementFactor.hpp"

WiFiMeasurementFactor::WiFiMeasurementFactor(
    gtsam::Key key, double distance, const gtsam::Point3 &apPosition,
    const gtsam::SharedNoiseModel &model)
    : gtsam::NoiseModelFactor1<gtsam::Point3>(model, key), distance_(distance),
      apPosition_(apPosition) {}

gtsam::Vector
WiFiMeasurementFactor::evaluateError(const gtsam::Point3 &x,
                                     boost::optional<gtsam::Matrix &> H) const {
  double predictedDistance = (x - apPosition_).norm();

  if (H) {
    gtsam::Vector3 diff = x - apPosition_;
    *H = diff.transpose() / predictedDistance;
  }

  return (gtsam::Vector(1) << predictedDistance - distance_).finished();
}

gtsam::Point3 WiFiMeasurementFactor::getApPosition() const {
  return apPosition_;
}
