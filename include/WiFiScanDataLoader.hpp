#ifndef WIFI_SCAN_DATA_LOADER_HPP
#define WIFI_SCAN_DATA_LOADER_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "CSVLoader.hpp"
#include "WiFiScanData.hpp"

class WiFiScanDataLoader : public CSVLoader {
public:
  WiFiScanDataLoader() = default;
  bool loadData(const std::string &filename) override;
  const std::vector<WiFiScanData> &getWiFiScanData() const;
  void printWiFiScanData() const;

private:
  std::vector<WiFiScanData> _scanData;
};

#endif // WIFI_SCAN_DATA_LOADER_HPP
