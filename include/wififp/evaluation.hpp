#pragma once

#include "wififp/localizer.hpp"
#include "wififp/radio_map.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace wififp {

struct EvalRecord {
  Method method;
  std::int64_t timestamp_ns = 0;
  Eigen::Vector3d truth = Eigen::Vector3d::Zero();
  std::optional<Estimate> estimate;  ///< nullopt: the method could not answer

  /// Horizontal error in metres (height is fixed by construction, see README).
  double error() const;
};

struct EvalOptions {
  MapperOptions mapper;
  bool model_ap_uncertainty = true;
};

/// Leave-one-scan-out cross-validation: for every fingerprint, rebuild the radio map
/// from all the *other* fingerprints and localize the held-out scan against it.
/// The SLAM position of the held-out scan is the ground truth.
std::vector<EvalRecord> leaveOneOut(const std::vector<Fingerprint>& fingerprints,
                                    const std::vector<Method>& methods, const EvalOptions& options = {});

struct ErrorSummary {
  Method method;
  size_t n = 0;
  size_t failed = 0;
  double mean = 0, median = 0, p90 = 0, max = 0;
};

std::vector<ErrorSummary> summarize(const std::vector<EvalRecord>& records, const std::vector<Method>& methods);

/// `method,timestamp_ns,true_x,true_y,true_z,est_x,est_y,est_z,error_m,std_x,std_y`
void writeEvalRecords(const std::filesystem::path& file, const std::vector<EvalRecord>& records);

}  // namespace wififp
