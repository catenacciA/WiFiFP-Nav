#include "wififp/radio_map.hpp"

#include "wififp/io.hpp"
#include "wififp/path_loss.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>

namespace wififp {

namespace {
const gtsam::Symbol kApKey('a', 0);
const gtsam::Symbol kParamsKey('p', 0);
}  // namespace

AccessPoint fitAccessPoint(const std::vector<Eigen::Vector3d>& receivers, const std::vector<double>& rssi_dbm,
                           double receiver_height, const MapperOptions& opt) {
  if (receivers.size() != rssi_dbm.size() || receivers.empty())
    throw std::invalid_argument("fitAccessPoint: need one RSSI per receiver position");

  // Initial guess: power-weighted centroid of the receivers (strong readings dominate).
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double total = 0.0;
  for (size_t i = 0; i < receivers.size(); ++i) {
    const double w = std::pow(10.0, rssi_dbm[i] / 10.0);
    centroid += w * receivers[i];
    total += w;
  }
  centroid /= total;
  const gtsam::Point3 ap0(centroid.x(), centroid.y(), receiver_height + opt.ap_height_above_receiver);
  const gtsam::Vector2 params0(opt.p0_prior_dbm, opt.exponent_prior);

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(kApKey, ap0,
                 gtsam::noiseModel::Diagonal::Sigmas(
                     gtsam::Vector3(opt.horizontal_sigma, opt.horizontal_sigma, opt.ap_height_sigma)));
  graph.addPrior(kParamsKey, params0,
                 gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(opt.p0_sigma_db, opt.exponent_sigma)));

  const auto noise = robustRssiNoise(opt.rssi_sigma_db, opt.huber_k);
  const gtsam::Point3_ ap_(kApKey);
  const Vector2_ params_(kParamsKey);
  for (size_t i = 0; i < receivers.size(); ++i)
    graph.add(rssiFactor(gtsam::Point3_(gtsam::Point3(receivers[i])), ap_, params_, rssi_dbm[i], noise));

  gtsam::Values initial;
  initial.insert(kApKey, ap0);
  initial.insert(kParamsKey, params0);
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

  AccessPoint ap;
  ap.position = result.at<gtsam::Point3>(kApKey);
  const auto params = result.at<gtsam::Vector2>(kParamsKey);
  ap.p0_dbm = params(0);
  ap.exponent = params(1);
  ap.n_obs = static_cast<int>(receivers.size());

  // Residual spread, corrected for the fitted degrees of freedom, floored so that
  // APs with a handful of readings are not trusted more than the data supports.
  double sq = 0.0;
  for (size_t i = 0; i < receivers.size(); ++i) {
    const double e = rssi_dbm[i] - predictRssi(receivers[i], ap.position, params);
    sq += e * e;
  }
  const double dof = std::max<double>(static_cast<double>(receivers.size()) - 2.0, 1.0);
  ap.sigma_db = std::max(std::sqrt(sq / dof), opt.min_sigma_db);

  try {
    ap.position_cov = gtsam::Marginals(graph, result).marginalCovariance(kApKey);
  } catch (const std::exception&) {  // degenerate geometry: fall back to the prior
    ap.position_cov = Eigen::Vector3d(opt.horizontal_sigma, opt.horizontal_sigma, opt.ap_height_sigma)
                          .array()
                          .square()
                          .matrix()
                          .asDiagonal();
  }
  return ap;
}

std::vector<Fingerprint> makeFingerprints(const Trajectory& trajectory, const std::vector<Scan>& scans,
                                          BuildStats* stats) {
  std::vector<Fingerprint> fingerprints;
  size_t outside = 0;
  for (const auto& scan : scans) {
    if (auto p = trajectory.positionAt(scan.seconds())) {
      fingerprints.push_back({scan, *p});
    } else {
      ++outside;
    }
  }
  if (stats) {
    stats->scans_total = scans.size();
    stats->scans_outside_trajectory = outside;
  }
  return fingerprints;
}

double RadioMap::receiverHeight() const {
  if (fingerprints.empty()) return 0.0;
  double z = 0.0;
  for (const auto& fp : fingerprints) z += fp.position.z();
  return z / static_cast<double>(fingerprints.size());
}

RadioMap RadioMap::fromFingerprints(std::vector<Fingerprint> fingerprints, const MapperOptions& options,
                                    BuildStats* stats) {
  RadioMap map;
  map.fingerprints = std::move(fingerprints);
  const double height = map.receiverHeight();

  struct Readings {
    const Observation* first = nullptr;
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> rssi;
  };
  std::map<std::string, Readings> by_bssid;  // ordered: deterministic output
  for (const auto& fp : map.fingerprints)
    for (const auto& o : fp.scan.observations) {
      auto& r = by_bssid[o.bssid];
      if (!r.first) r.first = &o;
      r.positions.push_back(fp.position);
      r.rssi.push_back(o.rssi_dbm);
    }

  size_t dropped = 0;
  for (const auto& [bssid, r] : by_bssid) {
    if (static_cast<int>(r.rssi.size()) < options.min_observations) {
      ++dropped;
      continue;
    }
    AccessPoint ap = fitAccessPoint(r.positions, r.rssi, height, options);
    ap.bssid = bssid;
    ap.ssid = r.first->ssid;
    ap.freq_mhz = r.first->freq_mhz;
    map.access_points.push_back(std::move(ap));
  }
  if (stats) {
    stats->aps_seen = by_bssid.size();
    stats->aps_dropped = dropped;
  }
  return map;
}

void RadioMap::save(const std::filesystem::path& dir) const {
  std::filesystem::create_directories(dir);
  writeFingerprints(dir / "fingerprints.csv", fingerprints);
  writeAccessPoints(dir / "access_points.csv", access_points);
}

RadioMap RadioMap::load(const std::filesystem::path& dir) {
  RadioMap map;
  map.fingerprints = readFingerprints(dir / "fingerprints.csv");
  map.access_points = readAccessPoints(dir / "access_points.csv");
  return map;
}

}  // namespace wififp
