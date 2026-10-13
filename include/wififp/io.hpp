#pragma once

#include "wififp/trajectory.hpp"
#include "wififp/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace wififp {

/// Reads a trajectory in TUM format (`t x y z qx qy qz qw`, one pose per line).
Trajectory readTumTrajectory(const std::filesystem::path& file);

/// Reads scans in long CSV format: `timestamp_ns,bssid,ssid,freq_mhz,rssi_dbm`.
/// Rows sharing a timestamp form one scan; scans are returned sorted by time.
std::vector<Scan> readScans(const std::filesystem::path& file);
void writeScans(const std::filesystem::path& file, const std::vector<Scan>& scans, bool append = false);

/// Fingerprints: `timestamp_ns,x,y,z,bssid,ssid,freq_mhz,rssi_dbm`.
std::vector<Fingerprint> readFingerprints(const std::filesystem::path& file);
void writeFingerprints(const std::filesystem::path& file, const std::vector<Fingerprint>& fingerprints);

/// Access-point models, one per row (position, path-loss parameters, covariance).
std::vector<AccessPoint> readAccessPoints(const std::filesystem::path& file);
void writeAccessPoints(const std::filesystem::path& file, const std::vector<AccessPoint>& aps);

namespace csv {
/// Splits one CSV line, honouring double quotes ("a,b" stays one field, "" is a quote).
std::vector<std::string> split(const std::string& line);
/// Quotes a field if it contains a comma, quote or newline.
std::string quote(const std::string& field);
}  // namespace csv

}  // namespace wififp
