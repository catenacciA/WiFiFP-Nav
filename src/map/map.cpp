#include "../../include/map/APPositionEstimator.hpp" // Include optimizer header
#include "../../include/map/Fingerprinting.hpp"
#include "../../include/map/Trajectory.hpp"
#include "../../include/map/WiFiData.hpp"
#include <functional>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;

int main() {
  try {
    // Load trajectory data
    Matrix trajectory_data = loadTrajectoryData("../datasets/lio_slam.txt");

    // Load WiFi data
    auto [wifi_scans, wifi_timestamps] = loadWiFiData("../datasets/wifi");

    // Create fingerprinting dataset using the structured data
    vector<FingerprintingRecord> fingerprinting_dataset =
        createFingerprintingDataset(trajectory_data, wifi_scans,
                                    wifi_timestamps);

    saveFingerprintingDataset(fingerprinting_dataset,
                              "fingerprinting_dataset.csv");

    // Initialize the APPositionEstimator class with the records
    APPositionEstimator estimator(fingerprinting_dataset);

    // Estimate the positions of the APs
    estimator.estimatePositions();

    // Get the estimated positions
    estimator.saveResultsToFile("ap_positions.csv");

  } catch (const exception &e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
