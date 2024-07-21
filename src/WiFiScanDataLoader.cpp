#include "../include/WiFiScanDataLoader.hpp"

bool WiFiScanDataLoader::loadData(const std::string &filename) {
  std::ifstream file(filename);

  std::string line;
  std::getline(file, line); // Skip header line

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string mac;
    double rssi, mean, std, frequency;
    std::getline(ss, mac, ',');
    ss >> rssi;
    ss.ignore();
    ss >> mean;
    ss.ignore();
    ss >> std;
    ss.ignore();
    ss >> frequency;
    _scanData.emplace_back(mac, rssi, mean, std, frequency);
  }

  if (_scanData.empty()) {
    std::cout << "Failed to load data from file: " << filename << std::endl;
    return false;
  }

  return true;
}

const std::vector<WiFiScanData> &WiFiScanDataLoader::getWiFiScanData() const {
  return _scanData;
}

void WiFiScanDataLoader::printWiFiScanData() const {
  for (const auto &scanData : _scanData) {
    std::cout << "MAC: " << scanData.mac << ", RSSI: " << scanData.rssi
              << ", Mean: " << scanData.mean << ", Std: " << scanData.std
              << ", Frequency: " << scanData.frequency << std::endl;
  }
}