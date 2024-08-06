#include "../../include/map/APPositionEstimator.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <iomanip>
#include <iostream>

using namespace gtsam;

// Implementation of CustomRangeFactor constructor
CustomRangeFactor::CustomRangeFactor(Key key, const Point3 &knownPoint,
                                     double measuredDistance,
                                     const SharedNoiseModel &model)
    : NoiseModelFactor1<Point3>(model, key), knownPoint_(knownPoint),
      measuredDistance_(measuredDistance) {}

// Implementation of evaluateError method
Vector CustomRangeFactor::evaluateError(const Point3 &estimate,
                                        boost::optional<Matrix &> H) const {
  double estimatedDistance = (estimate - knownPoint_).norm();
  if (H) {
    Matrix13 d_distance_d_estimate =
        (estimate - knownPoint_).normalized().transpose();
    *H = d_distance_d_estimate;
  }
  return (Vector(1) << estimatedDistance - measuredDistance_).finished();
}

// Implementation of APPosition constructor
APPositionEstimator::APPosition::APPosition(const std::string &ssid,
                                            const gtsam::Point3 &position,
                                            const Eigen::Matrix3d &covariance)
    : ssid(ssid), position(position), covariance(covariance) {}

// Implementation of APPositionEstimator constructor
APPositionEstimator::APPositionEstimator(
    const std::vector<FingerprintingRecord> &records)
    : records_(records) {}

// Convert RSSI to distance
double APPositionEstimator::rssiToDistance(double frequency,
                                           double signal_dbm) const {
  return ((27.55 - (20 * std::log10(frequency)) + std::abs(signal_dbm)) / 20.0);
}

// Estimate positions for all APs
void APPositionEstimator::estimatePositions() {
  std::map<std::string, std::vector<FingerprintingRecord>> groupedRecords;
  for (const auto &record : records_) {
    groupedRecords[record.mac].push_back(record);
  }

  for (const auto &[mac, apRecords] : groupedRecords) {
    optimizeAPPosition(mac, apRecords);
  }
}

// Optimize the position for a single AP
void APPositionEstimator::optimizeAPPosition(
    const std::string &macAddress,
    std::vector<FingerprintingRecord> apRecords) {
  NonlinearFactorGraph graph;
  Values initialEstimate;

  std::vector<Point3> knownPoints;
  std::vector<double> distances;
  std::string ssid;

  std::cout << "Optimizing position for AP with MAC: " << macAddress
            << std::endl;

  std::sort(apRecords.begin(), apRecords.end(),
            [](const FingerprintingRecord &a, const FingerprintingRecord &b) {
              return a.signalDBM > b.signalDBM;
            });

  size_t recordsToUse = std::min(apRecords.size(), static_cast<size_t>(5));

  for (size_t i = 0; i < recordsToUse; ++i) {
    const auto &record = apRecords[i];
    Point3 point(record.x, record.y, record.z);
    knownPoints.push_back(point);
    double distance = rssiToDistance(record.frequency, record.signalDBM);
    distances.push_back(distance);
    ssid = record.ssid;
    std::cout << "Known Point " << i << ": (" << record.x << ", " << record.y
              << ", " << record.z << "), Distance: " << distance << std::endl;
  }

  Point3 initialGuess = computeAveragePosition(knownPoints);
  std::cout << "Initial Guess: " << initialGuess << std::endl;

  auto priorNoise = noiseModel::Isotropic::Sigma(3, 2.0);
  graph.emplace_shared<PriorFactor<Point3>>(Symbol('a', 0), initialGuess,
                                            priorNoise);
  initialEstimate.insert(Symbol('a', 0), initialGuess);

  for (size_t i = 0; i < knownPoints.size(); ++i) {
    auto noise = noiseModel::Isotropic::Sigma(1, 1.0);
    graph.emplace_shared<CustomRangeFactor>(Symbol('a', 0), knownPoints[i],
                                            distances[i], noise);
  }

  std::cout << "Factor graph constructed. Starting optimization..."
            << std::endl;

  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setVerbosityLM("TRYLAMBDA");

  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();

  Point3 estimatedPosition = result.at<Point3>(Symbol('a', 0));
  std::cout << "Estimated Position: " << estimatedPosition << std::endl;

  Eigen::Matrix3d covarianceMatrix =
      calculateCovarianceMatrix(knownPoints, estimatedPosition);
  std::cout << "Covariance Matrix: \n" << covarianceMatrix << std::endl;

  estimatedPositions_.emplace(
      macAddress, APPosition(ssid, estimatedPosition, covarianceMatrix));
}

// Compute the average of known points
gtsam::Point3 APPositionEstimator::computeAveragePosition(
    const std::vector<Point3> &knownPoints) const {
  double xSum = 0.0, ySum = 0.0, zSum = 0.0;
  for (const auto &point : knownPoints) {
    xSum += point.x();
    ySum += point.y();
    zSum += point.z();
  }
  double count = static_cast<double>(knownPoints.size());
  return Point3(xSum / count, ySum / count, zSum / count);
}

// Calculate the covariance matrix for the estimated position
Eigen::Matrix3d APPositionEstimator::calculateCovarianceMatrix(
    const std::vector<Point3> &knownPoints, const Point3 &estimatedPosition) {
  size_t numPoints = knownPoints.size();
  Eigen::MatrixXd residuals(numPoints, 3);

  for (size_t i = 0; i < numPoints; ++i) {
    residuals.row(i) = knownPoints[i] - estimatedPosition;
  }

  Eigen::Matrix3d covMatrix =
      residuals.transpose() * residuals / (numPoints - 1);
  return covMatrix;
}

// Get the estimated positions
const std::map<std::string, APPositionEstimator::APPosition> &
APPositionEstimator::getEstimatedPositions() const {
  return estimatedPositions_;
}

// Save results to a file
void APPositionEstimator::saveResultsToFile(const std::string &filename) const {
  std::ofstream outFile(filename);

  if (!outFile.is_open()) {
    std::cerr << "Error: Could not open file " << filename << " for writing.\n";
    return;
  }

  outFile << "SSID,MAC,X,Y,Z,Covariance\n";

  // Sort estimated positions by SSID
  std::vector<std::pair<std::string, APPosition>> sortedPositions(
      estimatedPositions_.begin(), estimatedPositions_.end());
  std::sort(sortedPositions.begin(), sortedPositions.end(),
            [](const auto &a, const auto &b) {
              return a.second.ssid < b.second.ssid;
            });

  for (const auto &[mac, apPosition] : sortedPositions) {
    const auto &position = apPosition.position;
    const auto &covariance = apPosition.covariance;

    std::ostringstream covStream;
    covStream << std::fixed << std::setprecision(6);
    covStream << covariance(0, 0) << "," << covariance(0, 1) << ","
              << covariance(0, 2) << "," << covariance(1, 0) << ","
              << covariance(1, 1) << "," << covariance(1, 2) << ","
              << covariance(2, 0) << "," << covariance(2, 1) << ","
              << covariance(2, 2);

    outFile << apPosition.ssid << "," << mac << "," << position.x() << ","
            << position.y() << "," << position.z() << ","
            << "\"" << covStream.str() << "\"\n";
  }

  outFile.close();

  if (!outFile) {
    std::cerr << "Error: Failed to write to file " << filename << ".\n";
  } else {
    std::cout << "Results successfully written to " << filename << ".\n";
  }
}
