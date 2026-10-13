#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>

namespace wififp {

/// Reference distance d0 of the log-distance model, in metres. It also softens the
/// model at the access point: distance is taken as sqrt(|rx - ap|^2 + d0^2).
inline constexpr double kReferenceDistance = 1.0;

/// Log-distance path-loss model, the measurement function of every RSSI factor:
///
///   rssi = P0 - 10 n log10( sqrt(|rx - ap|^2 + d0^2) / d0 )
///
/// `params` = (P0 [dBm at d0], n [path-loss exponent]). Jacobians are analytic.
double predictRssi(const gtsam::Point3& rx, const gtsam::Point3& ap, const gtsam::Vector2& params,
                   gtsam::OptionalJacobian<1, 3> H_rx = {}, gtsam::OptionalJacobian<1, 3> H_ap = {},
                   gtsam::OptionalJacobian<1, 2> H_params = {});

using Vector2_ = gtsam::Expression<gtsam::Vector2>;

/// Factor on (receiver, AP, path-loss parameters) for one RSSI reading. Any of the
/// three can be a variable (`Point3_(key)`) or a constant (`Point3_(value)`): the
/// mapper solves for the AP with known receivers, the localizer does the opposite.
gtsam::ExpressionFactor<double> rssiFactor(const gtsam::Point3_& rx, const gtsam::Point3_& ap,
                                           const Vector2_& params, double rssi_dbm,
                                           const gtsam::SharedNoiseModel& noise);

/// Huber-robust 1-D noise model in dB: WiFi RSSI has heavy multipath outliers.
gtsam::SharedNoiseModel robustRssiNoise(double sigma_db, double huber_k);

}  // namespace wififp
