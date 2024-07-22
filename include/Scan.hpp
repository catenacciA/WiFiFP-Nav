#include <unordered_set>
#include <vector>

#include "../include/wifi_scan.h"
#include "WiFiScanData.hpp"

class Scan {
public:
  Scan(const std::string &interface);
  bool scan(std::vector<std::vector<WiFiScanData>> &wifiScanDataVec,
            bool filter);

private:
  std::string interface;
  std::unordered_set<std::string> targetBSSIDs;
};