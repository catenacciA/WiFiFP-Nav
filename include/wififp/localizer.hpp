#pragma once

#include "wififp/radio_map.hpp"
#include "wififp/types.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace wififp {

/// Estimates where a scan was taken.
class Localizer {
 public:
  virtual ~Localizer() = default;
  /// nullopt when the scan does not contain enough known access points.
  virtual std::optional<Estimate> locate(const Scan& scan) const = 0;
};

/// Weighted k-nearest-neighbours fingerprinting: compares the scan with every
/// fingerprint in RSSI space and averages the positions of the k closest ones.
class WknnLocalizer : public Localizer {
 public:
  struct Options {
    int k = 3;
    double missing_dbm = -100.0;  ///< RSSI assumed for an AP that was not heard
  };

  WknnLocalizer(const std::vector<Fingerprint>& fingerprints, Options options);
  explicit WknnLocalizer(const std::vector<Fingerprint>& fingerprints)
      : WknnLocalizer(fingerprints, Options{}) {}
  std::optional<Estimate> locate(const Scan& scan) const override;

 private:
  Options options_;
  std::unordered_map<std::string, int> column_;  // bssid -> column of the RSSI matrix
  Eigen::MatrixXd rssi_;                          // fingerprints x APs
  std::vector<Eigen::Vector3d> positions_;
};

/// Maximum-likelihood localization with the per-AP path-loss models, as a factor graph:
///
///   x  -- receiver position (the unknown)
///   a_j -- position of each heard AP, with a Gaussian prior from the radio map
///   f_j(x, a_j) -- robust RSSI factor, rssi_j ~ N(P0_j - 10 n_j log10|x - a_j|, sigma_j^2)
///
/// Keeping a_j as variables propagates the map's own uncertainty into the estimate,
/// so a poorly mapped AP pulls less than a well mapped one.
class PathLossLocalizer : public Localizer {
 public:
  struct Options {
    double receiver_height = 0.0;   ///< receiver z while mapping (from RadioMap::receiverHeight)
    double height_sigma = 0.1;      ///< receivers move on (roughly) one floor
    double huber_k = 1.5;
    bool model_ap_uncertainty = true;
    int min_aps = 3;
  };

  /// `initializer` (optional, not owned) provides the starting point, e.g. WKNN;
  /// without it the optimisation starts from the power-weighted centroid of the heard APs.
  PathLossLocalizer(std::vector<AccessPoint> access_points, Options options,
                    const Localizer* initializer = nullptr);
  std::optional<Estimate> locate(const Scan& scan) const override;

 private:
  Options options_;
  std::unordered_map<std::string, AccessPoint> aps_;
  const Localizer* initializer_;
};

/// Baseline that ignores the scan and always answers the centre of the mapped area.
class CentroidLocalizer : public Localizer {
 public:
  explicit CentroidLocalizer(const std::vector<Fingerprint>& fingerprints);
  std::optional<Estimate> locate(const Scan& scan) const override;

 private:
  Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
};

/// Methods available from the command line and in the benchmark.
enum class Method { Centroid, Wknn, PathLoss, Hybrid };

std::string toString(Method m);
std::optional<Method> parseMethod(const std::string& name);
std::vector<Method> allMethods();

/// Owns a localizer (and its initializer, if any) built from a radio map.
class LocalizerBundle : public Localizer {
 public:
  /// `model_ap_uncertainty` is forwarded to PathLossLocalizer (ablation switch).
  LocalizerBundle(const RadioMap& map, Method method, bool model_ap_uncertainty = true);
  std::optional<Estimate> locate(const Scan& scan) const override { return main_->locate(scan); }

 private:
  std::unique_ptr<Localizer> init_;
  std::unique_ptr<Localizer> main_;
};

}  // namespace wififp
