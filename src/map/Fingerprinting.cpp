// Fingerprinting.cpp
#include "../../include/map/Fingerprinting.hpp"
#include "../../include/map/Trajectory.hpp"
#include "../../include/map/WiFiData.hpp"
#include <fstream>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace std;
using namespace gtsam;

// Converts a timestamp to a string with high precision
string convertTimestampToSecondsString(long long timestamp) {
  ostringstream out;
  out << setprecision(18) << static_cast<long double>(timestamp) / 1e9;
  return out.str();
}

// Converts a long double value to a string with high precision
string toStringWithPrecision(long double value) {
  ostringstream out;
  out << setprecision(20) << value;
  return out.str();
}

// Creates a fingerprinting dataset from trajectory and WiFi scan data
vector<FingerprintingRecord>
createFingerprintingDataset(const Matrix &trajectory_data,
                            const vector<vector<WiFiScan>> &wifi_scans,
                            const Vector &wifi_timestamps) {

  vector<FingerprintingRecord> records;
  size_t trajectory_size = trajectory_data.rows();
  size_t wifi_size = wifi_timestamps.size();
  int step = trajectory_size / wifi_size;

  for (size_t i = 0; i < wifi_scans.size(); ++i) {
    const vector<WiFiScan> &wifi_scan = wifi_scans[i];
    long double wifi_timestamp = static_cast<long double>(wifi_timestamps[i]);
    int trajectory_index = i * step;

    // Ensure the trajectory index is within bounds
    if (trajectory_index >= trajectory_size) {
      cerr << "Error: Trajectory index " << trajectory_index
           << " out of bounds for trajectory size " << trajectory_size << endl;
      break;
    }

    // Skip the timestamp in the first column
    Vector closest_trajectory =
        trajectory_data.row(trajectory_index).segment(1, 3);

    for (const auto &scan : wifi_scan) {
      FingerprintingRecord record;
      record.timestamp = wifi_timestamp;
      record.mac = scan.mac;
      record.signalDBM = scan.signalDBM;
      record.ssid = scan.ssid;
      record.frequency = scan.frequency;
      record.x = closest_trajectory(0);
      record.y = closest_trajectory(1);
      record.z = closest_trajectory(2);
      records.push_back(record);
    }
  }

  return records;
}

// Saves the fingerprinting dataset to a CSV file
void saveFingerprintingDataset(
    const vector<FingerprintingRecord> &fingerprinting_data,
    const string &filepath) {
  ofstream file(filepath);
  if (!file.is_open()) {
    cerr << "Error: Could not open the file " << filepath << endl;
    exit(EXIT_FAILURE);
  }

  // Add frequency to the header
  file << "timestamp,mac,signalDBM,frequency,ssid,x,y,z\n";

  for (const auto &record : fingerprinting_data) {
    file << convertTimestampToSecondsString(
                static_cast<long long>(record.timestamp))
         << "," << record.mac << "," << toStringWithPrecision(record.signalDBM)
         << "," << toStringWithPrecision(record.frequency)
         << "," // Write frequency
         << record.ssid << "," << toStringWithPrecision(record.x) << ","
         << toStringWithPrecision(record.y) << ","
         << toStringWithPrecision(record.z) << "\n";
  }

  file.close();
}

// Prints the first 'rows' of the dataset to the console
void printDataset(const vector<FingerprintingRecord> &dataset,
                  const string &name, int rows) {
  cout << "First " << rows << " rows of " << name << ":\n";
  for (int i = 0; i < min(rows, static_cast<int>(dataset.size())); ++i) {
    cout << convertTimestampToSecondsString(
                static_cast<long long>(dataset[i].timestamp))
         << ", " << dataset[i].mac << ", "
         << toStringWithPrecision(dataset[i].signalDBM) << ", "
         << toStringWithPrecision(dataset[i].frequency)
         << ", " // Print frequency
         << dataset[i].ssid << ", " << toStringWithPrecision(dataset[i].x)
         << ", " << toStringWithPrecision(dataset[i].y) << ", "
         << toStringWithPrecision(dataset[i].z) << "\n";
  }
  cout << "\n";
}

// Prints the first 'rows' of a matrix to the console
void printDataset(const Matrix &matrix, const string &name, int rows) {
  cout << "First " << rows << " rows of " << name << ":\n";
  cout << setprecision(18); // Set precision for output
  for (int i = 0; i < min(rows, static_cast<int>(matrix.rows())); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      cout << matrix(i, j);
      if (j < matrix.cols() - 1) {
        cout << ", ";
      }
    }
    cout << "\n";
  }
  cout << "\n";
}
