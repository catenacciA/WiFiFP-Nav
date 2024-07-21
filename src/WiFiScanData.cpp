#include "../include/WiFiScanData.hpp"

WiFiScanData::WiFiScanData(const std::string &mac, double rssi, double mean,
                           double std, double frequency)
    : mac(mac), rssi(rssi), mean(mean), std(std), frequency(frequency) {}
