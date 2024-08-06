#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "../include/APDataLoader.hpp"
#include "../include/Scan.hpp"
#include "../include/WiFiLocalization.hpp"
#include "../include/WiFiScanData.hpp"
#include "../include/WiFiScanDataLoader.hpp"

double rssiToDistance(double frequency, double signal_dbm) {
  double distance =
      (27.55 - (20 * std::log10(frequency)) + std::abs(signal_dbm)) / 20.0;
  return distance;
}

// Function to display usage instructions
void printUsage() {
  std::cout
      << "Usage: ./program [interface | --use-file]\n"
      << "  [interface] : (Optional) Network interface to use for scanning. "
         "Default is 'wlp1s0f0'.\n"
      << "  --use-file  : Use pre-made scan file instead of live scanning.\n"
      << "Examples:\n"
      << "  ./program\n"
      << "  ./program wlan0\n"
      << "  ./program --use-file\n";
}

int main(int argc, char *argv[]) {
  // Default interface value
  std::string interface = "wlp1s0f0";
  bool scanFilter = false;
  std::vector<std::vector<WiFiScanData>> wifiScanDataVec;

  // Default to performing a live scan
  bool performLiveScan = true;

  // Argument parsing
  if (argc > 2) {
    printUsage();
    return 1;
  }

  if (argc == 2) {
    if (std::strcmp(argv[1], "--use-file") == 0) {
      performLiveScan = false;
    } else {
      interface = argv[1];
    }
  }

  if (performLiveScan) {
    Scan wifi_scanner(interface);
    wifi_scanner.scan(wifiScanDataVec, scanFilter);
  }

  APDataLoader apLoader;
  WiFiScanDataLoader scanLoader;

  WiFiLocalization localization(apLoader, scanLoader, rssiToDistance);

  const std::string apDataFile = "ap_positions.csv";
  const std::string scanDataFile =
      performLiveScan ? "scan_data.csv" : "../scan.csv";

  localization.loadAPData(apDataFile);
  localization.loadScanData(scanDataFile);

  auto estimatedPosition = localization.estimatePosition(3, 10);

  std::cout << "Estimated Position: [" << estimatedPosition.x() << ", "
            << estimatedPosition.y() << ", " << estimatedPosition.z() << "]"
            << std::endl;

  return 0;
}
