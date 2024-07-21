#include <cmath>
#include <functional>
#include <iostream>

#include "../include/APDataLoader.hpp"
#include "../include/WiFiLocalization.hpp"
#include "../include/WiFiScanDataLoader.hpp"

double rssiToDistance(double frequency, double signal_dbm) {
  double distance =
      (27.55 - (20 * std::log10(frequency)) + std::abs(signal_dbm)) / 20.0;
  return distance;
}

int main() {
  APDataLoader apLoader;
  WiFiScanDataLoader scanLoader;

  WiFiLocalization localization(apLoader, scanLoader, rssiToDistance);

  const std::string apDataFile = "../scripts/ap_positions.csv";
  const std::string scanDataFile = "../scan.csv";

  localization.loadAPData(apDataFile);
  localization.loadScanData(scanDataFile);

  auto estimatedPosition = localization.estimatePosition();

  std::cout << "Estimated Position: [" << estimatedPosition.x() << ", "
            << estimatedPosition.y() << ", " << estimatedPosition.z() << "]"
            << std::endl;

  return 0;
}
