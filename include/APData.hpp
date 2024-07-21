#ifndef APDATA_HPP
#define APDATA_HPP

#include <Eigen/Dense>
#include <string>

class APData {
public:
  std::string ssid;
  std::string mac;
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;

  APData(const std::string &ssid, const std::string &mac,
         Eigen::Vector3d position, const Eigen::Matrix3d &covariance);
};

#endif // APDATA_HPP
