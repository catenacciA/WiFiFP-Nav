#include "WiFiScanData.hpp"
#include "../include/wifi_scan.h"
#include <vector>
#include <unordered_set>

class Scan {
public:
    Scan(const std::string& interface);
    bool scan(std::vector<std::vector<WiFiScanData>>& wifiScanDataVec, bool filter);

private:
    std::string interface;
    std::unordered_set<std::string> targetBSSIDs;
};