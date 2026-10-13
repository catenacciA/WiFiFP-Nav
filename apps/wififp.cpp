// wififp: build a WiFi radio map from a SLAM run, localize scans, and benchmark methods.

#include "wififp/evaluation.hpp"
#include "wififp/io.hpp"
#include "wififp/localizer.hpp"
#include "wififp/radio_map.hpp"

#include <CLI/CLI.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;
using namespace wififp;

namespace {

int buildMap(const fs::path& trajectory_file, const fs::path& scans_file, const fs::path& out_dir) {
  const Trajectory trajectory = readTumTrajectory(trajectory_file);
  const std::vector<Scan> scans = readScans(scans_file);

  BuildStats stats;
  auto fingerprints = makeFingerprints(trajectory, scans, &stats);
  const RadioMap map = RadioMap::fromFingerprints(std::move(fingerprints), {}, &stats);
  map.save(out_dir);

  std::printf("trajectory   %zu poses, %.1f s, %.1f m walked\n", trajectory.samples().size(),
              trajectory.endTime() - trajectory.startTime(), trajectory.length());
  std::printf("scans        %zu (%zu outside the trajectory time span, skipped)\n", stats.scans_total,
              stats.scans_outside_trajectory);
  std::printf("access pts   %zu modelled, %zu dropped (heard < %d times)\n", map.access_points.size(),
              stats.aps_dropped, MapperOptions{}.min_observations);
  std::printf("written      %s/{fingerprints,access_points}.csv\n", out_dir.string().c_str());
  return 0;
}

int localize(const fs::path& map_dir, const fs::path& scans_file, const std::string& method_name,
             const fs::path& out_file) {
  const auto method = parseMethod(method_name);
  if (!method) throw CLI::ValidationError("--method", "unknown method " + method_name);
  const RadioMap map = RadioMap::load(map_dir);
  const LocalizerBundle localizer(map, *method);

  std::ofstream out;
  if (!out_file.empty()) {
    out.open(out_file);
    out << "timestamp_ns,x,y,z,std_x,std_y,n_aps\n";
  }
  std::printf("%-20s %9s %9s %9s %5s\n", "timestamp_ns", "x [m]", "y [m]", "std [m]", "APs");
  for (const Scan& scan : readScans(scans_file)) {
    const auto est = localizer.locate(scan);
    if (!est) {
      std::printf("%-20lld  (not enough known access points)\n", static_cast<long long>(scan.timestamp_ns));
      continue;
    }
    const double sx = std::sqrt(est->cov(0, 0)), sy = std::sqrt(est->cov(1, 1));
    const double sigma = std::sqrt(0.5 * (est->cov(0, 0) + est->cov(1, 1)));  // RMS horizontal std
    char sigma_str[16] = "        -";  // methods without a covariance
    if (std::isfinite(sigma)) std::snprintf(sigma_str, sizeof sigma_str, "%9.2f", sigma);
    std::printf("%-20lld %9.2f %9.2f %s %5d\n", static_cast<long long>(scan.timestamp_ns), est->position.x(),
                est->position.y(), sigma_str, est->n_aps);
    if (out.is_open())
      out << scan.timestamp_ns << ',' << est->position.x() << ',' << est->position.y() << ','
          << est->position.z() << ',' << sx << ',' << sy << ',' << est->n_aps << '\n';
  }
  return 0;
}

int benchmark(const fs::path& trajectory_file, const fs::path& scans_file, const fs::path& out_file,
              const EvalOptions& options) {
  const Trajectory trajectory = readTumTrajectory(trajectory_file);
  const auto fingerprints = makeFingerprints(trajectory, readScans(scans_file));
  const auto methods = allMethods();

  std::printf("Leave-one-scan-out over %zu scans (map rebuilt %zu times)...\n", fingerprints.size(),
              fingerprints.size());
  const auto t0 = std::chrono::steady_clock::now();
  const auto records = leaveOneOut(fingerprints, methods, options);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  std::printf("\n| Method   | Mean [m] | Median [m] | P90 [m] | Max [m] | Failed |\n");
  std::printf("|----------|---------:|-----------:|--------:|--------:|-------:|\n");
  for (const auto& s : summarize(records, methods))
    std::printf("| %-8s | %8.2f | %10.2f | %7.2f | %7.2f | %6zu |\n", toString(s.method).c_str(), s.mean,
                s.median, s.p90, s.max, s.failed);
  std::printf("\nhorizontal error w.r.t. LiDAR-SLAM ground truth; %.1f s\n", secs);

  if (!out_file.empty()) {
    writeEvalRecords(out_file, records);
    std::printf("per-scan results written to %s\n", out_file.string().c_str());
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"WiFiFP-Nav: WiFi radio mapping and localization with factor graphs"};
  app.require_subcommand(1);

  fs::path trajectory, scans, map_dir = "map", out;
  std::string method = "hybrid";

  auto* build = app.add_subcommand("build-map", "Fit a radio map from a SLAM trajectory and WiFi scans");
  build->add_option("-t,--trajectory", trajectory, "Trajectory in TUM format")->required()->check(CLI::ExistingFile);
  build->add_option("-s,--scans", scans, "Scans CSV")->required()->check(CLI::ExistingFile);
  build->add_option("-o,--out", map_dir, "Output directory")->capture_default_str();

  auto* loc = app.add_subcommand("localize", "Localize every scan in a CSV against a radio map");
  loc->add_option("-m,--map", map_dir, "Radio map directory (from build-map)")->check(CLI::ExistingDirectory)
      ->capture_default_str();
  loc->add_option("-s,--scans", scans, "Scans CSV to localize")->required()->check(CLI::ExistingFile);
  loc->add_option("--method", method, "centroid | wknn | pathloss | hybrid")->capture_default_str();
  loc->add_option("-o,--out", out, "Optional CSV with the estimates");

  auto* bench = app.add_subcommand("benchmark", "Leave-one-scan-out evaluation of all methods");
  bench->add_option("-t,--trajectory", trajectory, "Trajectory in TUM format")->required()->check(CLI::ExistingFile);
  bench->add_option("-s,--scans", scans, "Scans CSV")->required()->check(CLI::ExistingFile);
  bench->add_option("-o,--out", out, "Optional CSV with per-scan errors");
  EvalOptions eval;
  bench->add_flag("--no-ap-uncertainty", [&](std::int64_t) { eval.model_ap_uncertainty = false; },
                  "Treat mapped AP positions as exact (ablation)");

  CLI11_PARSE(app, argc, argv);

  try {
    if (*build) return buildMap(trajectory, scans, map_dir);
    if (*loc) return localize(map_dir, scans, method, out);
    if (*bench) return benchmark(trajectory, scans, out, eval);
  } catch (const CLI::Error& e) {
    return app.exit(e);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
