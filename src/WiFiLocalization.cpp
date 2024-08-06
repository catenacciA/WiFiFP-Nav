#include <algorithm>
#include <cmath>
#include <functional>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "../include/WiFiLocalization.hpp"
#include "../include/WiFiMeasurementFactor.hpp"

using namespace gtsam::symbol_shorthand;

WiFiLocalization::WiFiLocalization(
    APDataLoader &apLoader, WiFiScanDataLoader &scanLoader,
    std::function<double(double, double)> rssiToDistanceFunc)
    : _apDataLoader(apLoader), _scanDataLoader(scanLoader),
      _rssiToDistanceFunc(rssiToDistanceFunc) {}

void WiFiLocalization::loadAPData(const std::string &filename) {
  if (_apDataLoader.loadData(filename)) {
    _apDataList = _apDataLoader.getAPData();
  } else {
    std::cerr << "Failed to load AP data from " << filename << std::endl;
  }
}

void WiFiLocalization::loadScanData(const std::string &filename) {
  if (_scanDataLoader.loadData(filename)) {
    _scanDataList = _scanDataLoader.getWiFiScanData();
  } else {
    std::cerr << "Failed to load WiFi scan data from " << filename << std::endl;
  }
}

Eigen::Vector3d WiFiLocalization::estimatePosition(int N, double maxDistance) {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;

  // Filter APs based on distance
  std::vector<std::pair<WiFiScanData, double>> filteredScansWithDistance;
  for (const auto &scan : _scanDataList) {
    for (const auto &ap : _apDataList) {
      if (scan.mac == ap.mac) {
        double distance = _rssiToDistanceFunc(scan.frequency, scan.rssi);
        if (distance <= maxDistance) {
          filteredScansWithDistance.emplace_back(scan, distance);
        }
      }
    }
  }

  // Sort filtered APs by distance
  std::sort(filteredScansWithDistance.begin(), filteredScansWithDistance.end(),
            [](const std::pair<WiFiScanData, double> &a,
               const std::pair<WiFiScanData, double> &b) {
              return a.second < b.second;
            });

  // Select the nearest N APs
  if (filteredScansWithDistance.size() > N) {
    filteredScansWithDistance.erase(filteredScansWithDistance.begin() + N,
                                    filteredScansWithDistance.end());
  }

  // Initial guess: weighted average based on signal strength
  gtsam::Point3 weightedAPPosition(0.0, 0.0, 0.0);
  double totalWeight = 0.0;

  for (const auto &scanPair : filteredScansWithDistance) {
    const auto &scan = scanPair.first;
    for (const auto &ap : _apDataList) {
      if (scan.mac == ap.mac) {
        double weight = std::pow(10.0, scan.rssi / 10.0);
        weightedAPPosition +=
            weight *
            gtsam::Point3(ap.position.x(), ap.position.y(), ap.position.z());
        totalWeight += weight;
      }
    }
  }

  gtsam::Point3 initialGuess(0.0, 0.0,
                             0.0); // Default if no valid scans are found
  if (totalWeight > 0) {
    initialGuess = weightedAPPosition / totalWeight;
  }

  // Adding prior factor with initial guess
  constexpr double PRIOR_NOISE_SIGMA = 10.0;
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
          PRIOR_NOISE_SIGMA, PRIOR_NOISE_SIGMA, PRIOR_NOISE_SIGMA));
  graph.add(gtsam::PriorFactor<gtsam::Point3>(1, initialGuess, priorNoise));
  initialEstimate.insert(1, initialGuess);

  // Adding WiFi measurement factors with a fixed distance error model and AP
  // position covariance
  constexpr double DISTANCE_NOISE_SIGMA = 1.0;
  gtsam::SharedNoiseModel distanceNoiseModel =
      gtsam::noiseModel::Isotropic::Sigma(1, DISTANCE_NOISE_SIGMA);

  std::unordered_set<gtsam::Key> addedKeys; // Track keys that have been added

  for (const auto &scanPair : filteredScansWithDistance) {
    const auto &scan = scanPair.first;
    for (const auto &ap : _apDataList) {
      if (scan.mac == ap.mac) {
        double distance = _rssiToDistanceFunc(scan.frequency, scan.rssi);

        // Create a 3x3 noise model for the AP position using its covariance
        // matrix
        gtsam::SharedNoiseModel apPositionNoiseModel =
            gtsam::noiseModel::Gaussian::Covariance(ap.covariance);

        // Add the WiFi measurement factor with distance noise model
        graph.add(WiFiMeasurementFactor(
            1, distance,
            gtsam::Point3(ap.position.x(), ap.position.y(), ap.position.z()),
            distanceNoiseModel));

        // Add prior factors for the AP positions to model their uncertainties
        std::size_t apKeyHash = std::hash<std::string>{}(ap.mac) &
                                0xFFFFFFFFFFFFFF; // Mask to fit within 56 bits
        gtsam::Key apKey =
            gtsam::Symbol('A', apKeyHash); // Generate a unique key for the AP

        if (addedKeys.find(apKey) == addedKeys.end()) {
          graph.add(gtsam::PriorFactor<gtsam::Point3>(
              apKey,
              gtsam::Point3(ap.position.x(), ap.position.y(), ap.position.z()),
              apPositionNoiseModel));
          initialEstimate.insert(
              apKey,
              gtsam::Point3(
                  ap.position.x(), ap.position.y(),
                  ap.position.z())); // Insert AP position into initial estimate
          addedKeys.insert(apKey);   // Mark this key as added
        }
      }
    }
  }

  // Perform optimization
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(100);
  params.setlambdaInitial(1.0); // Increase initial lambda for more regularization
  params.setlambdaFactor(2.0);  // Adjust lambda scaling factor for stability

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  std::cout << "Initial error: " << optimizer.error() << std::endl;

  gtsam::Values result = optimizer.optimize();
  std::cout << "Final error: " << optimizer.error() << std::endl;

  // Extract the optimized position
  gtsam::Point3 optimizedPosition = result.at<gtsam::Point3>(1);
  return Eigen::Vector3d(optimizedPosition.x(), optimizedPosition.y(),
                         optimizedPosition.z());
}
