#include "../include/APData.hpp"

APData::APData(const std::string &ssid, const std::string &mac,
               Eigen::Vector3d position, const Eigen::Matrix3d &covariance)
    : ssid(ssid), mac(mac), position(position), covariance(covariance) {}
