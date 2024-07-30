#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../include/Scan.hpp"
#include "../include/wifi_scan.h"

Scan::Scan(const std::string &interface) : interface(interface) {
  if (interface.empty()) {
    throw std::invalid_argument("Interface name cannot be empty");
  }
}

bool Scan::scan(std::vector<std::vector<WiFiScanData>> &wifiScanDataVec,
                bool filter) {
  const int maxAPs = 100;
  const int numScans = 10;
  std::unordered_map<std::string, std::vector<WiFiScanData>> wifiDataMap;
  std::unordered_map<std::string, double> frequencyMap;

  struct bss_info bssInfos[maxAPs];

  for (int scanIdx = 0; scanIdx < numScans; ++scanIdx) {
    std::cout << "Starting scan " << scanIdx + 1 << " of " << numScans
              << std::endl;

    wifi_scan *wifi = wifi_scan_init(interface.c_str());
    if (!wifi) {
      std::cerr << "Failed to initialize WiFi scan on interface "
                << interface << std::endl;
      return false;
    }

    int numAPs = wifi_scan_all(wifi, bssInfos, maxAPs);

    if (numAPs < 0) {
      std::cerr << "Failed to perform WiFi scan on interface "
                << interface << std::endl;
      return false;
    }

    std::cout << "Found " << numAPs << " access points during scan "
              << scanIdx + 1 << std::endl;

    for (int i = 0; i < numAPs && i < maxAPs; ++i) {
      char bssidStr[BSSID_STRING_LENGTH];
      snprintf(bssidStr, BSSID_STRING_LENGTH, "%02x:%02x:%02x:%02x:%02x:%02x",
               bssInfos[i].bssid[0], bssInfos[i].bssid[1], bssInfos[i].bssid[2],
               bssInfos[i].bssid[3], bssInfos[i].bssid[4],
               bssInfos[i].bssid[5]);

      std::string mac(bssidStr);

      if (!filter || targetBSSIDs.find(mac) != targetBSSIDs.end()) {
        double rssi = 30 - (bssInfos[i].signal_mbm / 100);
        frequencyMap[mac] = bssInfos[i].frequency;

        WiFiScanData data(mac, rssi, 0.0, 0.0, bssInfos[i].frequency);
        wifiDataMap[mac].push_back(data);
      }
    }

    std::cout << "Scan " << scanIdx + 1 << " completed." << std::endl;
  }

  wifiScanDataVec.clear();

  if (wifiDataMap.empty()) {
    std::cerr << "No APs found after scanning." << std::endl;
    return false;
  }

  for (auto &entry : wifiDataMap) {
    const std::string &mac = entry.first;
    std::vector<WiFiScanData> &wifiData = entry.second;

    double mean = std::accumulate(wifiData.begin(), wifiData.end(), 0.0,
                                  [](double sum, const WiFiScanData &data) {
                                    return sum + data.rssi;
                                  }) /
                  wifiData.size();

    double sq_sum =
        std::accumulate(wifiData.begin(), wifiData.end(), 0.0,
                        [mean](double sum, const WiFiScanData &data) {
                          return sum + (data.rssi - mean) * (data.rssi - mean);
                        });
    double stddev = std::sqrt(sq_sum / wifiData.size());

    for (auto &data : wifiData) {
      data.mean = mean;
      data.std = stddev;
    }

    wifiScanDataVec.push_back(wifiData);
  }

  std::ofstream outputFile("scan_data.csv");
  outputFile << "MAC Address,RSSI Value,Mean,STD,Frequency\n";

  for (const auto &entry : wifiDataMap) {
    const std::string &mac = entry.first;
    const std::vector<WiFiScanData> &wifiData = entry.second;

    if (!wifiData.empty()) {
      const auto &data = wifiData.front();
      outputFile << data.mac << "," << data.rssi << "," << data.mean << ","
                 << data.std << "," << data.frequency << "\n";
    }
  }

  outputFile.close();

  std::cout << "Scan data has been written to scan_data.csv" << std::endl;

  return true;
}
