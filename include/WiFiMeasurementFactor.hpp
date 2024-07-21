#ifndef WIFI_MEASUREMENT_FACTOR_HPP
#define WIFI_MEASUREMENT_FACTOR_HPP

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

class WiFiMeasurementFactor : public gtsam::NoiseModelFactor1<gtsam::Point3> {
  double distance_;
  gtsam::Point3 apPosition_;

public:
  WiFiMeasurementFactor(gtsam::Key key, double distance,
                        const gtsam::Point3 &apPosition,
                        const gtsam::SharedNoiseModel &model);

  gtsam::Vector evaluateError(
      const gtsam::Point3 &x,
      boost::optional<gtsam::Matrix &> H = boost::none) const override;

  gtsam::Point3 getApPosition() const;
};

#endif // WIFI_MEASUREMENT_FACTOR_HPP
