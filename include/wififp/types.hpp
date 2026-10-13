#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace wififp {

/// One access point (BSSID) heard in a scan.
struct Observation {
  std::string bssid;
  std::string ssid;
  int freq_mhz = 0;
  double rssi_dbm = 0.0;
};

/// Everything heard in a single WiFi scan.
struct Scan {
  std::int64_t timestamp_ns = 0;
  std::vector<Observation> observations;

  double seconds() const { return static_cast<double>(timestamp_ns) * 1e-9; }
};

/// A scan taken at a known position (from SLAM): one sample of the radio map.
struct Fingerprint {
  Scan scan;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

/// Log-distance path-loss model of one access point:
///   rssi(x) = p0_dbm - 10 * exponent * log10(|x - position|)  (+ N(0, sigma_db^2))
struct AccessPoint {
  std::string bssid;
  std::string ssid;
  int freq_mhz = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d position_cov = Eigen::Matrix3d::Identity();
  double p0_dbm = -35.0;   ///< RSSI at the 1 m reference distance
  double exponent = 2.5;   ///< path-loss exponent
  double sigma_db = 4.0;   ///< residual standard deviation of the fit
  int n_obs = 0;           ///< number of fingerprints that heard this AP
};

/// Output of a localizer.
struct Estimate {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// Marginal covariance of the position; NaN-filled when the method has none.
  Eigen::Matrix3d cov = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  int n_aps = 0;  ///< number of access points that contributed
};

}  // namespace wififp
