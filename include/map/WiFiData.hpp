#ifndef WIFIDATA_HPP
#define WIFIDATA_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <string>
#include <utility>
#include <vector>

using namespace std;

// Struct to hold WiFi scan data
struct WiFiScan {
  string mac;
  double frequency;
  string ssid;
  double signalDBM;
  double signalAvgDBM;
};

// Function declarations
pair<vector<vector<WiFiScan>>, gtsam::Vector>
loadWiFiData(const string &directory);
gtsam::Vector
convertWiFiTimestampsToSeconds(const gtsam::Vector &wifi_timestamps);

#endif // WIFIDATA_HPP
