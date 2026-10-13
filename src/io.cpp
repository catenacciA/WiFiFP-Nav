#include "wififp/io.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace wififp {

namespace fs = std::filesystem;

namespace csv {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        field += '"';
        ++i;
      } else if (c == '"') {
        quoted = false;
      } else {
        field += c;
      }
    } else if (c == '"') {
      quoted = true;
    } else if (c == ',') {
      fields.push_back(std::move(field));
      field.clear();
    } else if (c != '\r') {
      field += c;
    }
  }
  fields.push_back(std::move(field));
  return fields;
}

std::string quote(const std::string& field) {
  if (field.find_first_of(",\"\n") == std::string::npos) return field;
  std::string out = "\"";
  for (char c : field) out += (c == '"') ? std::string("\"\"") : std::string(1, c);
  return out + '"';
}

}  // namespace csv

namespace {

std::ifstream openForReading(const fs::path& file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("cannot open " + file.string());
  return in;
}

std::ofstream openForWriting(const fs::path& file, bool append = false) {
  if (file.has_parent_path()) fs::create_directories(file.parent_path());
  std::ofstream out(file, append ? std::ios::app : std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write " + file.string());
  out << std::setprecision(10);
  return out;
}

/// CSV reader that addresses columns by header name.
class Table {
 public:
  Table(const fs::path& file, std::initializer_list<const char*> required)
      : file_(file), in_(openForReading(file)) {
    std::string header;
    if (!std::getline(in_, header)) throw std::runtime_error(file.string() + ": empty file");
    const auto names = csv::split(header);
    for (size_t i = 0; i < names.size(); ++i) columns_[names[i]] = i;
    for (const char* name : required)
      if (!columns_.count(name))
        throw std::runtime_error(file.string() + ": missing column '" + name + "'");
  }

  bool next() {
    std::string line;
    while (std::getline(in_, line)) {
      ++line_no_;
      if (line.empty() || line == "\r") continue;
      row_ = csv::split(line);
      if (row_.size() < columns_.size())
        throw std::runtime_error(where() + ": expected " + std::to_string(columns_.size()) + " fields");
      return true;
    }
    return false;
  }

  const std::string& str(const std::string& col) const { return row_[columns_.at(col)]; }
  double num(const std::string& col) const { return parse<double>(col, [](auto& s) { return std::stod(s); }); }
  long long integer(const std::string& col) const {
    return parse<long long>(col, [](auto& s) { return std::stoll(s); });
  }

 private:
  template <typename T, typename F>
  T parse(const std::string& col, F f) const {
    try {
      return f(str(col));
    } catch (const std::exception&) {
      throw std::runtime_error(where() + ": bad value '" + str(col) + "' in column " + col);
    }
  }
  std::string where() const { return file_.string() + ":" + std::to_string(line_no_ + 1); }

  fs::path file_;
  std::ifstream in_;
  std::unordered_map<std::string, size_t> columns_;
  std::vector<std::string> row_;
  size_t line_no_ = 0;
};

Observation readObservation(const Table& t) {
  return {t.str("bssid"), t.str("ssid"), static_cast<int>(t.integer("freq_mhz")), t.num("rssi_dbm")};
}

void writeObservation(std::ostream& out, const Observation& o) {
  out << o.bssid << ',' << csv::quote(o.ssid) << ',' << o.freq_mhz << ',' << o.rssi_dbm << '\n';
}

}  // namespace

Trajectory readTumTrajectory(const fs::path& file) {
  auto in = openForReading(file);
  std::vector<StampedPosition> samples;
  std::string line;
  size_t line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    StampedPosition s;
    if (!(ss >> s.t >> s.p.x() >> s.p.y() >> s.p.z()))
      throw std::runtime_error(file.string() + ":" + std::to_string(line_no) + ": not a TUM pose");
    samples.push_back(s);
  }
  return Trajectory(std::move(samples));
}

std::vector<Scan> readScans(const fs::path& file) {
  Table t(file, {"timestamp_ns", "bssid", "ssid", "freq_mhz", "rssi_dbm"});
  std::map<std::int64_t, Scan> by_time;
  while (t.next()) {
    const std::int64_t ts = t.integer("timestamp_ns");
    Scan& scan = by_time[ts];
    scan.timestamp_ns = ts;
    scan.observations.push_back(readObservation(t));
  }
  std::vector<Scan> scans;
  scans.reserve(by_time.size());
  for (auto& [ts, scan] : by_time) scans.push_back(std::move(scan));
  return scans;
}

void writeScans(const fs::path& file, const std::vector<Scan>& scans, bool append) {
  const bool header = !append || !fs::exists(file) || fs::file_size(file) == 0;
  auto out = openForWriting(file, append);
  if (header) out << "timestamp_ns,bssid,ssid,freq_mhz,rssi_dbm\n";
  for (const auto& scan : scans)
    for (const auto& o : scan.observations) {
      out << scan.timestamp_ns << ',';
      writeObservation(out, o);
    }
}

std::vector<Fingerprint> readFingerprints(const fs::path& file) {
  Table t(file, {"timestamp_ns", "x", "y", "z", "bssid", "ssid", "freq_mhz", "rssi_dbm"});
  std::map<std::int64_t, Fingerprint> by_time;
  while (t.next()) {
    const std::int64_t ts = t.integer("timestamp_ns");
    Fingerprint& fp = by_time[ts];
    fp.scan.timestamp_ns = ts;
    fp.position = {t.num("x"), t.num("y"), t.num("z")};
    fp.scan.observations.push_back(readObservation(t));
  }
  std::vector<Fingerprint> out;
  for (auto& [ts, fp] : by_time) out.push_back(std::move(fp));
  return out;
}

void writeFingerprints(const fs::path& file, const std::vector<Fingerprint>& fingerprints) {
  auto out = openForWriting(file);
  out << "timestamp_ns,x,y,z,bssid,ssid,freq_mhz,rssi_dbm\n";
  for (const auto& fp : fingerprints)
    for (const auto& o : fp.scan.observations) {
      out << fp.scan.timestamp_ns << ',' << fp.position.x() << ',' << fp.position.y() << ','
          << fp.position.z() << ',';
      writeObservation(out, o);
    }
}

std::vector<AccessPoint> readAccessPoints(const fs::path& file) {
  Table t(file, {"bssid", "ssid", "freq_mhz", "x", "y", "z", "p0_dbm", "exponent", "sigma_db", "n_obs",
                 "cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"});
  std::vector<AccessPoint> aps;
  while (t.next()) {
    AccessPoint ap;
    ap.bssid = t.str("bssid");
    ap.ssid = t.str("ssid");
    ap.freq_mhz = static_cast<int>(t.integer("freq_mhz"));
    ap.position = {t.num("x"), t.num("y"), t.num("z")};
    ap.p0_dbm = t.num("p0_dbm");
    ap.exponent = t.num("exponent");
    ap.sigma_db = t.num("sigma_db");
    ap.n_obs = static_cast<int>(t.integer("n_obs"));
    const double xx = t.num("cov_xx"), xy = t.num("cov_xy"), xz = t.num("cov_xz");
    const double yy = t.num("cov_yy"), yz = t.num("cov_yz"), zz = t.num("cov_zz");
    ap.position_cov << xx, xy, xz, xy, yy, yz, xz, yz, zz;
    aps.push_back(std::move(ap));
  }
  return aps;
}

void writeAccessPoints(const fs::path& file, const std::vector<AccessPoint>& aps) {
  auto out = openForWriting(file);
  out << "bssid,ssid,freq_mhz,x,y,z,p0_dbm,exponent,sigma_db,n_obs,"
         "cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz\n";
  for (const auto& ap : aps) {
    const auto& c = ap.position_cov;
    out << ap.bssid << ',' << csv::quote(ap.ssid) << ',' << ap.freq_mhz << ',' << ap.position.x() << ','
        << ap.position.y() << ',' << ap.position.z() << ',' << ap.p0_dbm << ',' << ap.exponent << ','
        << ap.sigma_db << ',' << ap.n_obs << ',' << c(0, 0) << ',' << c(0, 1) << ',' << c(0, 2) << ','
        << c(1, 1) << ',' << c(1, 2) << ',' << c(2, 2) << '\n';
  }
}

}  // namespace wififp
