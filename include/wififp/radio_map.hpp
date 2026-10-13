#pragma once

#include "wififp/trajectory.hpp"
#include "wififp/types.hpp"

#include <filesystem>
#include <vector>

namespace wififp {

/// Tunables of the access-point fit. Defaults are what the benchmark uses.
struct MapperOptions {
  int min_observations = 3;       ///< APs heard in fewer fingerprints are dropped
  double rssi_sigma_db = 4.0;     ///< measurement noise assumed during the fit
  double huber_k = 1.345;         ///< robust-kernel threshold, in sigmas
  double min_sigma_db = 3.0;      ///< floor for the per-AP residual sigma

  // Priors. A walk at constant height cannot tell whether an AP is above or below the
  // receiver plane, so the AP height gets a prior instead of being left free.
  double ap_height_above_receiver = 1.5;  ///< m, typical ceiling/desk mounting
  double ap_height_sigma = 0.5;
  double p0_prior_dbm = -35.0;
  double p0_sigma_db = 15.0;
  double exponent_prior = 2.5;  ///< indoor offices are typically 2..4
  double exponent_sigma = 0.7;
  double horizontal_sigma = 100.0;  ///< near-flat prior on x,y, only keeps the problem well-posed
};

/// Fits the path-loss model of a single access point from the fingerprints that heard it.
/// Unknowns: AP position (Point3) and (P0, n). Solved as a small factor graph with
/// Levenberg-Marquardt; the position covariance is the GTSAM marginal at the optimum.
AccessPoint fitAccessPoint(const std::vector<Eigen::Vector3d>& receivers, const std::vector<double>& rssi_dbm,
                           double receiver_height, const MapperOptions& options = {});

struct BuildStats {
  size_t scans_total = 0;
  size_t scans_outside_trajectory = 0;
  size_t aps_seen = 0;
  size_t aps_dropped = 0;  ///< heard by fewer than `min_observations` fingerprints
};

/// Attaches a position to every scan by interpolating the trajectory at the scan
/// timestamp. Scans outside the trajectory's time span are skipped.
std::vector<Fingerprint> makeFingerprints(const Trajectory& trajectory, const std::vector<Scan>& scans,
                                          BuildStats* stats = nullptr);

/// The radio map: located fingerprints plus a path-loss model per access point.
struct RadioMap {
  std::vector<Fingerprint> fingerprints;
  std::vector<AccessPoint> access_points;

  /// Mean height of the fingerprints, i.e. of the receiver while mapping.
  double receiverHeight() const;

  /// Fits one AccessPoint per BSSID heard in `fingerprints`.
  static RadioMap fromFingerprints(std::vector<Fingerprint> fingerprints, const MapperOptions& options = {},
                                   BuildStats* stats = nullptr);

  /// Writes `fingerprints.csv` and `access_points.csv` into `dir`.
  void save(const std::filesystem::path& dir) const;
  static RadioMap load(const std::filesystem::path& dir);
};

}  // namespace wififp
