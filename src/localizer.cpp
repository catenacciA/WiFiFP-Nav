#include "wififp/localizer.hpp"

#include "wififp/path_loss.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace wififp {

// ---------------------------------------------------------------- WKNN

WknnLocalizer::WknnLocalizer(const std::vector<Fingerprint>& fingerprints, Options options)
    : options_(options) {
  for (const auto& fp : fingerprints)
    for (const auto& o : fp.scan.observations)
      column_.emplace(o.bssid, static_cast<int>(column_.size()));

  rssi_ = Eigen::MatrixXd::Constant(static_cast<Eigen::Index>(fingerprints.size()),
                                    static_cast<Eigen::Index>(column_.size()), options_.missing_dbm);
  for (size_t i = 0; i < fingerprints.size(); ++i) {
    for (const auto& o : fingerprints[i].scan.observations)
      rssi_(static_cast<Eigen::Index>(i), column_.at(o.bssid)) = o.rssi_dbm;
    positions_.push_back(fingerprints[i].position);
  }
}

std::optional<Estimate> WknnLocalizer::locate(const Scan& scan) const {
  Eigen::RowVectorXd query = Eigen::RowVectorXd::Constant(rssi_.cols(), options_.missing_dbm);
  int known = 0;
  for (const auto& o : scan.observations)
    if (auto it = column_.find(o.bssid); it != column_.end()) {
      query(it->second) = o.rssi_dbm;
      ++known;
    }
  if (known == 0 || positions_.empty()) return std::nullopt;

  const Eigen::VectorXd dist =
      ((rssi_.rowwise() - query).array().square().rowwise().mean()).sqrt().matrix();
  std::vector<int> order(static_cast<size_t>(dist.size()));
  std::iota(order.begin(), order.end(), 0);
  const int k = std::min<int>(options_.k, static_cast<int>(order.size()));
  std::partial_sort(order.begin(), order.begin() + k, order.end(),
                    [&](int a, int b) { return dist(a) < dist(b); });

  Estimate est;
  double total = 0.0;
  for (int i = 0; i < k; ++i) {
    const double w = 1.0 / (dist(order[i]) + 1e-6);
    est.position += w * positions_[static_cast<size_t>(order[i])];
    total += w;
  }
  est.position /= total;
  est.n_aps = known;
  return est;
}

// ---------------------------------------------------------------- path loss

PathLossLocalizer::PathLossLocalizer(std::vector<AccessPoint> access_points, Options options,
                                     const Localizer* initializer)
    : options_(options), initializer_(initializer) {
  for (auto& ap : access_points) aps_.emplace(ap.bssid, std::move(ap));
}

std::optional<Estimate> PathLossLocalizer::locate(const Scan& scan) const {
  std::vector<std::pair<const AccessPoint*, double>> heard;
  for (const auto& o : scan.observations)
    if (auto it = aps_.find(o.bssid); it != aps_.end()) heard.emplace_back(&it->second, o.rssi_dbm);
  if (static_cast<int>(heard.size()) < options_.min_aps) return std::nullopt;

  // Starting point.
  Eigen::Vector3d x0 = Eigen::Vector3d::Zero();
  std::optional<Estimate> init = initializer_ ? initializer_->locate(scan) : std::nullopt;
  if (init) {
    x0 = init->position;
  } else {
    double total = 0.0;
    for (const auto& [ap, rssi] : heard) {
      const double w = std::pow(10.0, rssi / 10.0);
      x0 += w * ap->position;
      total += w;
    }
    x0 /= total;
  }
  x0.z() = options_.receiver_height;

  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Only the height is constrained a priori; x, y come from the RSSI factors.
  graph.addPrior(X(0), gtsam::Point3(x0),
                 gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e3, 1e3, options_.height_sigma)));
  initial.insert(X(0), gtsam::Point3(x0));

  const gtsam::Point3_ rx(X(0));
  for (size_t j = 0; j < heard.size(); ++j) {
    const auto& [ap, rssi] = heard[j];
    const Vector2_ params(gtsam::Vector2(ap->p0_dbm, ap->exponent));
    const auto noise = robustRssiNoise(ap->sigma_db, options_.huber_k);
    if (options_.model_ap_uncertainty) {
      const Eigen::Matrix3d cov = ap->position_cov + 1e-6 * Eigen::Matrix3d::Identity();
      graph.addPrior(A(j), gtsam::Point3(ap->position), gtsam::noiseModel::Gaussian::Covariance(cov));
      initial.insert(A(j), gtsam::Point3(ap->position));
      graph.add(rssiFactor(rx, gtsam::Point3_(A(j)), params, rssi, noise));
    } else {
      graph.add(rssiFactor(rx, gtsam::Point3_(gtsam::Point3(ap->position)), params, rssi, noise));
    }
  }

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

  Estimate est;
  est.position = result.at<gtsam::Point3>(X(0));
  est.n_aps = static_cast<int>(heard.size());
  try {
    est.cov = gtsam::Marginals(graph, result).marginalCovariance(X(0));
  } catch (const std::exception&) {
    // leave NaN: the position is still the ML estimate, only its covariance is unknown
  }
  return est;
}

// ---------------------------------------------------------------- baseline

CentroidLocalizer::CentroidLocalizer(const std::vector<Fingerprint>& fingerprints) {
  for (const auto& fp : fingerprints) centroid_ += fp.position;
  if (!fingerprints.empty()) centroid_ /= static_cast<double>(fingerprints.size());
}

std::optional<Estimate> CentroidLocalizer::locate(const Scan&) const {
  Estimate est;
  est.position = centroid_;
  return est;
}

// ---------------------------------------------------------------- factory

std::string toString(Method m) {
  switch (m) {
    case Method::Centroid: return "centroid";
    case Method::Wknn: return "wknn";
    case Method::PathLoss: return "pathloss";
    case Method::Hybrid: return "hybrid";
  }
  return "?";
}

std::optional<Method> parseMethod(const std::string& name) {
  for (Method m : allMethods())
    if (toString(m) == name) return m;
  return std::nullopt;
}

std::vector<Method> allMethods() { return {Method::Centroid, Method::Wknn, Method::PathLoss, Method::Hybrid}; }

LocalizerBundle::LocalizerBundle(const RadioMap& map, Method method, bool model_ap_uncertainty) {
  PathLossLocalizer::Options pl;
  pl.receiver_height = map.receiverHeight();
  pl.model_ap_uncertainty = model_ap_uncertainty;
  switch (method) {
    case Method::Centroid:
      main_ = std::make_unique<CentroidLocalizer>(map.fingerprints);
      break;
    case Method::Wknn:
      main_ = std::make_unique<WknnLocalizer>(map.fingerprints);
      break;
    case Method::PathLoss:
      main_ = std::make_unique<PathLossLocalizer>(map.access_points, pl);
      break;
    case Method::Hybrid:
      init_ = std::make_unique<WknnLocalizer>(map.fingerprints);
      main_ = std::make_unique<PathLossLocalizer>(map.access_points, pl, init_.get());
      break;
  }
}

}  // namespace wififp
