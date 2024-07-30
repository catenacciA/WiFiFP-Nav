#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>

#include "../include/APDataLoader.hpp"
#include "../include/WiFiLocalization.hpp"
#include "../include/WiFiScanDataLoader.hpp"
#include "../include/WiFiScanData.hpp"
#include "../include/Scan.hpp"

double rssiToDistance(double frequency, double signal_dbm) {
    double distance =
        (27.55 - (20 * std::log10(frequency)) + std::abs(signal_dbm)) / 20.0;
    return distance;
}

int main(int argc, char* argv[]) {
    std::string interface = "wlp1s0f0";
    bool scanFilter = false;
    std::vector<std::vector<WiFiScanData>> wifiScanDataVec;

    // Check if the user wants to perform a live scan or use a pre-made file
    bool performLiveScan = true;
    if (argc > 1 && std::strcmp(argv[1], "--use-file") == 0) {
        performLiveScan = false;
    }

/*     if (performLiveScan) {
        Scan wifi_scanner(interface);
        wifi_scanner.scan(wifiScanDataVec, scanFilter);
    } */

    APDataLoader apLoader;
    WiFiScanDataLoader scanLoader;

    WiFiLocalization localization(apLoader, scanLoader, rssiToDistance);

    const std::string apDataFile = "../scripts/ap_positions.csv";
    const std::string scanDataFile = performLiveScan ? "scan_data.csv" : "../scan.csv";

    localization.loadAPData(apDataFile);
    localization.loadScanData(scanDataFile);

    auto estimatedPosition = localization.estimatePosition(3, 10);

    std::cout << "Estimated Position: [" << estimatedPosition.x() << ", "
              << estimatedPosition.y() << ", " << estimatedPosition.z() << "]"
              << std::endl;

    return 0;
}
