#ifndef WIFI_SCAN_DATA_HPP
#define WIFI_SCAN_DATA_HPP

#include <string>

class WiFiScanData {
public:
  std::string mac;
  double rssi;
  double mean;
  double std;
  double frequency;

  WiFiScanData(const std::string &mac, double rssi, double mean, double std,
               double frequency);
};

#endif // WIFI_SCAN_DATA_HPP