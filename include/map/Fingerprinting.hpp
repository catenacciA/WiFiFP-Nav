// Fingerprinting.hpp
#ifndef FINGERPRINTING_HPP
#define FINGERPRINTING_HPP

#include "../../include/map/WiFiData.hpp"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <string>
#include <vector>

struct FingerprintingRecord {
  long double timestamp;
  std::string mac;
  long double signalDBM;
  std::string ssid;
  double frequency;
  long double x;
  long double y;
  long double z;
};

// Creates a fingerprinting dataset from trajectory and WiFi scan data
std::vector<FingerprintingRecord> createFingerprintingDataset(
    const gtsam::Matrix &trajectory_data,
    const std::vector<std::vector<WiFiScan>> &wifi_scans,
    const gtsam::Vector &wifi_timestamps);

// Saves the fingerprinting dataset to a CSV file
void saveFingerprintingDataset(
    const std::vector<FingerprintingRecord> &fingerprinting_data,
    const std::string &filepath);

// Prints the first 'rows' of the dataset to the console
void printDataset(const std::vector<FingerprintingRecord> &dataset,
                  const std::string &name, int rows);

// Prints the first 'rows' of a matrix to the console
void printDataset(const gtsam::Matrix &matrix, const std::string &name,
                  int rows);

// Converts a timestamp to a string with precision
std::string convertTimestampToSecondsString(long long timestamp);

// Converts a long double to a string with precision
std::string toStringWithPrecision(long double value);

#endif // FINGERPRINTING_HPP
