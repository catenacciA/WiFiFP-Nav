#include "../../include/map/WiFiData.hpp"
#include "../../include/map/Utils.hpp"
#include <filesystem>
#include <fstream>
#include <gtsam/base/serialization.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace std::filesystem;
using namespace gtsam;

// Function to compare files by their timestamp derived from their filename
bool compareFilesByTimestamp(const path &a, const path &b) {
  string filename_a = a.stem().string();
  string filename_b = b.stem().string();
  try {
    long long timestamp_a = stoll(filename_a);
    long long timestamp_b = stoll(filename_b);
    return timestamp_a < timestamp_b;
  } catch (const invalid_argument &e) {
    cerr << "Error: Invalid filename format for timestamp in files " << a
         << " or " << b << endl;
    throw;
  }
}

pair<vector<vector<WiFiScan>>, Vector> loadWiFiData(const string &directory) {
  vector<vector<WiFiScan>> wifi_scans;
  vector<long long> wifi_timestamps;

  vector<path> files;
  for (const auto &entry : directory_iterator(directory)) {
    files.push_back(entry.path());
  }

  // Sort files by their timestamp
  sort(files.begin(), files.end(), compareFilesByTimestamp);

  for (const auto &entry : files) {
    ifstream file(entry);
    if (!file.is_open()) {
      cerr << "Error: Could not open the file " << entry << endl;
      continue;
    }
    string line;
    vector<WiFiScan> data;

    // Skip the header line
    getline(file, line);

    while (getline(file, line)) {
      vector<string> tokens = split(line, ',');
      if (tokens.size() != 5) {
        cerr << "Error: Invalid data format in file " << entry << endl;
        continue;
      }
      try {
        istringstream stream1(tokens[1]);
        istringstream stream3(tokens[3]);
        istringstream stream4(tokens[4]);
        stream1 >> std::setprecision(9);
        stream3 >> std::setprecision(9);
        stream4 >> std::setprecision(9);

        WiFiScan scan = {tokens[0], stod(tokens[1]), tokens[2], stod(tokens[3]),
                         stod(tokens[4])};
        data.push_back(scan);
      } catch (const invalid_argument &e) {
        cerr << "Error: Invalid argument in file " << entry << ": " << line
             << endl;
        continue;
      }
    }

    if (data.empty()) {
      cerr << "Error: No valid data found in file " << entry << endl;
      continue;
    }

    wifi_scans.push_back(data);
    string filename = entry.stem().string();
    try {
      long long timestamp = stoll(filename);
      wifi_timestamps.push_back(timestamp);
    } catch (const invalid_argument &e) {
      cerr << "Error: Invalid filename format for timestamp in file " << entry
           << endl;
      continue;
    }
  }

  Vector wifi_timestamps_vector(wifi_timestamps.size());
  for (size_t i = 0; i < wifi_timestamps.size(); ++i) {
    wifi_timestamps_vector[i] = static_cast<double>(wifi_timestamps[i]);
  }

  return make_pair(wifi_scans, wifi_timestamps_vector);
}

Vector convertWiFiTimestampsToSeconds(const Vector &wifi_timestamps) {
  Vector wifi_timestamps_in_seconds(wifi_timestamps.size());
  for (size_t i = 0; i < wifi_timestamps.size(); ++i) {
    wifi_timestamps_in_seconds[i] = wifi_timestamps[i] / 1e9;
    cout << "Converted timestamp: " << wifi_timestamps[i]
         << " to seconds: " << wifi_timestamps_in_seconds[i] << endl;
  }
  return wifi_timestamps_in_seconds;
}
