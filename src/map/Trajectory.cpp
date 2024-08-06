#include <fstream>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace std;
using namespace gtsam;

Matrix loadTrajectoryData(const string &filepath) {
  ifstream file(filepath);
  if (!file.is_open()) {
    cerr << "Error: Could not open the file " << filepath << endl;
    throw runtime_error("Error: Could not open the file " + filepath);
  }

  vector<vector<long double>> data;
  string line;
  int line_count = 0;
  while (getline(file, line)) {
    ++line_count;
    istringstream stream(line);
    vector<long double> row;
    long double value;
    while (stream >> value) {
      row.push_back(value);
    }
    if (!row.empty()) {
      data.push_back(row);
    } else {
      cout << "Skipped empty line " << line_count << endl;
    }
  }

  if (data.empty() || data[0].empty()) {
    cerr << "Error: No data found in file " << filepath << endl;
    throw runtime_error("Error: No data found in file " + filepath);
  }

  Matrix trajectory_data(data.size(), data[0].size());
  for (size_t i = 0; i < data.size(); ++i) {
    for (size_t j = 0; j < data[i].size(); ++j) {
      trajectory_data(i, j) = static_cast<double>(data[i][j]);
    }
  }

  return trajectory_data;
}

Vector getTrajectoryPositionAtIndex(const Matrix &trajectory_data, int index) {
  if (index < 0 || index >= trajectory_data.rows()) {
    cerr << "Error: Index " << index << " out of range in trajectory data"
         << endl;
    throw out_of_range("Index out of range in trajectory data");
  }
  Vector position = trajectory_data.row(index);
  return position;
}
