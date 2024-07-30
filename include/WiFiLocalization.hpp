#ifndef WIFILOCALIZATION_HPP
#define WIFILOCALIZATION_HPP

#include <Eigen/Dense>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "APData.hpp"
#include "APDataLoader.hpp"
#include "WiFiScanData.hpp"
#include "WiFiScanDataLoader.hpp"

class WiFiLocalization {
private:
  std::vector<APData> _apDataList;
  std::vector<WiFiScanData> _scanDataList;

  APDataLoader &_apDataLoader;
  WiFiScanDataLoader &_scanDataLoader;
  std::function<double(double, double)> _rssiToDistanceFunc;

public:
  WiFiLocalization(APDataLoader &apLoader, WiFiScanDataLoader &scanLoader,
                   std::function<double(double, double)> rssiToDistanceFunc);

  void loadAPData(const std::string &filename);
  void loadScanData(const std::string &filename);
  Eigen::Vector3d estimatePosition(int N, double maxDistance);
};

#endif // WIFILOCALIZATION_HPP
