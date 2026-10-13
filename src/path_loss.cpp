#include "wififp/path_loss.hpp"

#include <gtsam/linear/NoiseModel.h>

#include <cmath>

namespace wififp {

double predictRssi(const gtsam::Point3& rx, const gtsam::Point3& ap, const gtsam::Vector2& params,
                   gtsam::OptionalJacobian<1, 3> H_rx, gtsam::OptionalJacobian<1, 3> H_ap,
                   gtsam::OptionalJacobian<1, 2> H_params) {
  constexpr double d0_sq = kReferenceDistance * kReferenceDistance;
  const gtsam::Vector3 delta = rx - ap;
  const double r_sq = delta.squaredNorm() + d0_sq;
  const double log_d = 0.5 * std::log10(r_sq / d0_sq);  // log10(r / d0)
  const double p0 = params(0), n = params(1);

  // d/d(rx) of -10 n log10(r) = -10 n / (r ln 10) * delta / r
  const gtsam::Matrix13 d_rx = (-10.0 * n / (r_sq * std::log(10.0))) * delta.transpose();
  if (H_rx) *H_rx = d_rx;
  if (H_ap) *H_ap = -d_rx;
  if (H_params) *H_params << 1.0, -10.0 * log_d;
  return p0 - 10.0 * n * log_d;
}

gtsam::ExpressionFactor<double> rssiFactor(const gtsam::Point3_& rx, const gtsam::Point3_& ap,
                                           const Vector2_& params, double rssi_dbm,
                                           const gtsam::SharedNoiseModel& noise) {
  const gtsam::Double_ predicted(&predictRssi, rx, ap, params);
  return gtsam::ExpressionFactor<double>(noise, rssi_dbm, predicted);
}

gtsam::SharedNoiseModel robustRssiNoise(double sigma_db, double huber_k) {
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
                                           gtsam::noiseModel::Isotropic::Sigma(1, sigma_db));
}

}  // namespace wififp
