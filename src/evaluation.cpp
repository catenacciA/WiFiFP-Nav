#include "wififp/evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace wififp {

double EvalRecord::error() const {
  if (!estimate) return std::numeric_limits<double>::quiet_NaN();
  return (estimate->position - truth).head<2>().norm();
}

std::vector<EvalRecord> leaveOneOut(const std::vector<Fingerprint>& fingerprints,
                                    const std::vector<Method>& methods, const EvalOptions& options) {
  std::vector<EvalRecord> records;
  for (size_t k = 0; k < fingerprints.size(); ++k) {
    std::vector<Fingerprint> train;
    train.reserve(fingerprints.size() - 1);
    for (size_t i = 0; i < fingerprints.size(); ++i)
      if (i != k) train.push_back(fingerprints[i]);
    const RadioMap map = RadioMap::fromFingerprints(std::move(train), options.mapper);

    const Fingerprint& test = fingerprints[k];
    for (Method m : methods) {
      const LocalizerBundle localizer(map, m, options.model_ap_uncertainty);
      records.push_back({m, test.scan.timestamp_ns, test.position, localizer.locate(test.scan)});
    }
  }
  return records;
}

namespace {
double percentile(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  const double pos = p * static_cast<double>(v.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (pos - static_cast<double>(lo)) * (v[hi] - v[lo]);
}
}  // namespace

std::vector<ErrorSummary> summarize(const std::vector<EvalRecord>& records, const std::vector<Method>& methods) {
  std::vector<ErrorSummary> out;
  for (Method m : methods) {
    ErrorSummary s;
    s.method = m;
    std::vector<double> errors;
    for (const auto& r : records) {
      if (r.method != m) continue;
      ++s.n;
      if (r.estimate) {
        errors.push_back(r.error());
      } else {
        ++s.failed;
      }
    }
    if (!errors.empty()) {
      s.mean = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
      s.median = percentile(errors, 0.5);
      s.p90 = percentile(errors, 0.9);
      s.max = *std::max_element(errors.begin(), errors.end());
    }
    out.push_back(s);
  }
  return out;
}

void writeEvalRecords(const std::filesystem::path& file, const std::vector<EvalRecord>& records) {
  if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path());
  std::ofstream out(file);
  if (!out) throw std::runtime_error("cannot write " + file.string());
  out << std::setprecision(10);
  out << "method,timestamp_ns,true_x,true_y,true_z,est_x,est_y,est_z,error_m,std_x,std_y\n";
  for (const auto& r : records) {
    out << toString(r.method) << ',' << r.timestamp_ns << ',' << r.truth.x() << ',' << r.truth.y() << ','
        << r.truth.z();
    if (r.estimate) {
      const auto& e = *r.estimate;
      out << ',' << e.position.x() << ',' << e.position.y() << ',' << e.position.z() << ',' << r.error() << ','
          << std::sqrt(e.cov(0, 0)) << ',' << std::sqrt(e.cov(1, 1)) << '\n';
    } else {
      out << ",,,,,,\n";
    }
  }
}

}  // namespace wififp
