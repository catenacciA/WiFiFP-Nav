// End-to-end checks on synthetic data: APs and receivers are placed at known
// positions, RSSI is generated from the model, and the estimators must recover them.

#include "wififp/evaluation.hpp"
#include "wififp/localizer.hpp"
#include "wififp/path_loss.hpp"
#include "wififp/radio_map.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace wififp;

namespace {

constexpr double kHeight = 0.8;

struct SyntheticAp {
  std::string bssid;
  Eigen::Vector3d position;
  double p0, n;
};

std::vector<SyntheticAp> syntheticAps() {
  return {{"02:00:00:00:00:01", {0, 0, 2.3}, -35, 2.2},  {"02:00:00:00:00:02", {12, 1, 2.3}, -38, 2.6},
          {"02:00:00:00:00:03", {6, 10, 2.3}, -33, 2.4}, {"02:00:00:00:00:04", {-3, 8, 2.3}, -40, 2.8},
          {"02:00:00:00:00:05", {10, -6, 2.3}, -36, 2.5}};
}

Scan scanAt(const Eigen::Vector3d& rx, double noise_db, std::mt19937& rng, std::int64_t t_ns) {
  std::normal_distribution<double> noise(0.0, noise_db);
  Scan scan;
  scan.timestamp_ns = t_ns;
  for (const auto& ap : syntheticAps()) {
    const double rssi = predictRssi(rx, ap.position, gtsam::Vector2(ap.p0, ap.n)) + noise(rng);
    scan.observations.push_back({ap.bssid, "net", 2412, rssi});
  }
  return scan;
}

/// A lawn-mower walk over a 16 x 16 m area, one scan every metre.
std::vector<Fingerprint> survey(double noise_db, unsigned seed = 7) {
  std::mt19937 rng(seed);
  std::vector<Fingerprint> fps;
  std::int64_t t = 1'000'000'000;
  for (int row = 0; row <= 16; row += 2)
    for (int col = 0; col <= 16; ++col) {
      const double x = (row / 2) % 2 ? 16 - col : col;
      const Eigen::Vector3d p(x - 4, row - 4, kHeight);
      fps.push_back({scanAt(p, noise_db, rng, t), p});
      t += 1'000'000'000;
    }
  return fps;
}

}  // namespace

TEST(Mapper, RecoversAccessPointFromNoiseFreeData) {
  const auto fps = survey(0.0);
  const RadioMap map = RadioMap::fromFingerprints(fps);
  ASSERT_EQ(map.access_points.size(), syntheticAps().size());
  for (const auto& truth : syntheticAps()) {
    const auto it = std::find_if(map.access_points.begin(), map.access_points.end(),
                                 [&](const AccessPoint& a) { return a.bssid == truth.bssid; });
    ASSERT_NE(it, map.access_points.end());
    EXPECT_LT((it->position - truth.position).head<2>().norm(), 0.1) << truth.bssid;
    EXPECT_NEAR(it->p0_dbm, truth.p0, 0.5);
    EXPECT_NEAR(it->exponent, truth.n, 0.05);
  }
}

TEST(Mapper, CovarianceIsPositiveDefiniteAndShrinksWithData) {
  const auto fps = survey(3.0);
  std::vector<Eigen::Vector3d> pos;
  std::vector<double> rssi;
  for (const auto& fp : fps) {
    pos.push_back(fp.position);
    rssi.push_back(fp.scan.observations[0].rssi_dbm);
  }
  const AccessPoint all = fitAccessPoint(pos, rssi, kHeight);
  const AccessPoint few = fitAccessPoint({pos.begin(), pos.begin() + 10}, {rssi.begin(), rssi.begin() + 10}, kHeight);
  EXPECT_GT(all.position_cov.llt().matrixL().determinant(), 0.0);
  EXPECT_LT(all.position_cov.trace(), few.position_cov.trace());
  EXPECT_NEAR(all.sigma_db, 3.0, 0.6);
}

TEST(Localizer, PathLossRecoversReceiverWithPerfectMap) {
  std::vector<AccessPoint> aps;
  for (const auto& s : syntheticAps()) {
    AccessPoint ap;
    ap.bssid = s.bssid;
    ap.position = s.position;
    ap.position_cov = 1e-4 * Eigen::Matrix3d::Identity();
    ap.p0_dbm = s.p0;
    ap.exponent = s.n;
    ap.sigma_db = 3.0;
    aps.push_back(ap);
  }
  PathLossLocalizer::Options opt;
  opt.receiver_height = kHeight;
  const PathLossLocalizer localizer(aps, opt);

  std::mt19937 rng(1);
  const Eigen::Vector3d truth(3.0, 4.0, kHeight);
  const auto est = localizer.locate(scanAt(truth, 0.0, rng, 0));
  ASSERT_TRUE(est);
  EXPECT_LT((est->position - truth).norm(), 1e-3);
  EXPECT_TRUE(est->cov.allFinite());
}

TEST(Localizer, RejectsScansWithTooFewKnownAps) {
  const RadioMap map = RadioMap::fromFingerprints(survey(0.0));
  Scan scan;
  scan.observations = {{"02:00:00:00:00:01", "", 2412, -50}, {"ff:ff:ff:ff:ff:ff", "", 2412, -50}};
  EXPECT_FALSE(LocalizerBundle(map, Method::PathLoss).locate(scan));
  EXPECT_TRUE(LocalizerBundle(map, Method::Wknn).locate(scan));  // one known AP is enough for WKNN
}

TEST(Evaluation, LeaveOneOutBeatsBaselineOnSyntheticSurvey) {
  std::vector<Fingerprint> fps;  // every third scan: still covers the area, keeps the test fast
  const auto all = survey(4.0);
  for (size_t i = 0; i < all.size(); i += 3) fps.push_back(all[i]);
  const auto methods = allMethods();
  const auto summary = summarize(leaveOneOut(fps, methods), methods);
  ASSERT_EQ(summary.size(), methods.size());
  const auto& centroid = summary[0];
  for (const auto& s : summary) {
    EXPECT_EQ(s.n, fps.size());
    EXPECT_EQ(s.failed, 0u);
    if (s.method != Method::Centroid) EXPECT_LT(s.median, 0.5 * centroid.median) << toString(s.method);
  }
}
