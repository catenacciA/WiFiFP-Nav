#include "wififp/io.hpp"
#include "wififp/radio_map.hpp"

#include <gtest/gtest.h>

#include <fstream>

using namespace wififp;
namespace fs = std::filesystem;

namespace {

class TempDir : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("wififp_test_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }
  fs::path dir_;
};

void writeText(const fs::path& p, const std::string& text) { std::ofstream(p) << text; }

}  // namespace

TEST(Csv, SplitHandlesQuotes) {
  EXPECT_EQ(csv::split("a,b,,c"), (std::vector<std::string>{"a", "b", "", "c"}));
  EXPECT_EQ(csv::split(R"(1,"Cafe, upstairs","say ""hi""")"),
            (std::vector<std::string>{"1", "Cafe, upstairs", R"(say "hi")"}));
  EXPECT_EQ(csv::split(csv::quote("x,\"y\"")), (std::vector<std::string>{"x,\"y\""}));
}

TEST(Trajectory, InterpolatesLinearlyAndRejectsOutside) {
  Trajectory traj({{2.0, {2, 0, 0}}, {0.0, {0, 0, 0}}, {1.0, {1, 2, 0}}});  // unsorted on purpose
  EXPECT_TRUE(traj.positionAt(0.5)->isApprox(Eigen::Vector3d(0.5, 1, 0)));
  EXPECT_TRUE(traj.positionAt(2.0)->isApprox(Eigen::Vector3d(2, 0, 0)));
  EXPECT_FALSE(traj.positionAt(-0.1));
  EXPECT_FALSE(traj.positionAt(2.1));
  EXPECT_NEAR(traj.length(), 2 * std::sqrt(5.0), 1e-12);
}

TEST_F(TempDir, ReadsTumAndScansAndBuildsFingerprints) {
  writeText(dir_ / "traj.tum", "# t x y z qx qy qz qw\n10.0 0 0 0 0 0 0 1\n20.0 10 0 0 0 0 0 1\n");
  writeText(dir_ / "scans.csv",
            "timestamp_ns,bssid,ssid,freq_mhz,rssi_dbm\n"
            "15000000000,aa:bb:cc:dd:ee:01,lab,2412,-48\n"
            "15000000000,aa:bb:cc:dd:ee:02,\"guest, 5G\",5180,-71\n"
            "25000000000,aa:bb:cc:dd:ee:01,lab,2412,-60\n");

  const auto traj = readTumTrajectory(dir_ / "traj.tum");
  const auto scans = readScans(dir_ / "scans.csv");
  ASSERT_EQ(scans.size(), 2u);
  EXPECT_EQ(scans[0].observations.size(), 2u);
  EXPECT_EQ(scans[0].observations[1].ssid, "guest, 5G");
  EXPECT_DOUBLE_EQ(scans[0].observations[1].rssi_dbm, -71.0);

  BuildStats stats;
  const auto fps = makeFingerprints(traj, scans, &stats);
  ASSERT_EQ(fps.size(), 1u);  // the scan at t=25 s is after the trajectory ends
  EXPECT_EQ(stats.scans_outside_trajectory, 1u);
  EXPECT_TRUE(fps[0].position.isApprox(Eigen::Vector3d(5, 0, 0)));
}

TEST_F(TempDir, RadioMapRoundTrip) {
  RadioMap map;
  map.fingerprints.push_back({{42, {{"aa:bb:cc:dd:ee:01", "a,b", 2412, -55.5}}}, {1, 2, 3}});
  AccessPoint ap;
  ap.bssid = "aa:bb:cc:dd:ee:01";
  ap.ssid = "a,b";
  ap.freq_mhz = 2412;
  ap.position = {1.5, -2.25, 2.0};
  ap.position_cov << 1, 0.1, 0.2, 0.1, 2, 0.3, 0.2, 0.3, 3;
  ap.p0_dbm = -37.25;
  ap.exponent = 2.75;
  ap.sigma_db = 4.5;
  ap.n_obs = 9;
  map.access_points.push_back(ap);

  map.save(dir_);
  const RadioMap back = RadioMap::load(dir_);
  ASSERT_EQ(back.fingerprints.size(), 1u);
  EXPECT_EQ(back.fingerprints[0].scan.timestamp_ns, 42);
  EXPECT_EQ(back.fingerprints[0].scan.observations[0].ssid, "a,b");
  ASSERT_EQ(back.access_points.size(), 1u);
  const auto& a = back.access_points[0];
  EXPECT_EQ(a.ssid, "a,b");
  EXPECT_TRUE(a.position.isApprox(ap.position));
  EXPECT_TRUE(a.position_cov.isApprox(ap.position_cov));
  EXPECT_DOUBLE_EQ(a.p0_dbm, ap.p0_dbm);
  EXPECT_DOUBLE_EQ(a.exponent, ap.exponent);
  EXPECT_EQ(a.n_obs, 9);
}

TEST_F(TempDir, MissingColumnIsAClearError) {
  writeText(dir_ / "bad.csv", "timestamp_ns,bssid,rssi_dbm\n1,aa,-50\n");
  try {
    readScans(dir_ / "bad.csv");
    FAIL() << "expected an exception";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("missing column 'ssid'"), std::string::npos);
  }
}
