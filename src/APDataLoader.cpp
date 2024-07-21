#include <algorithm>
#include <exception>
#include <iomanip>
#include <iterator>
#include <sstream>

#include "../include/APDataLoader.hpp"

std::vector<std::string> split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  bool inQuotes = false;
  std::stringstream tokenStream;

  for (char ch : s) {
    if (ch == '"') {
      inQuotes = !inQuotes;
    } else if (ch == delimiter && !inQuotes) {
      tokens.push_back(tokenStream.str());
      tokenStream.str("");
      tokenStream.clear();
    } else {
      tokenStream << ch;
    }
  }
  tokens.push_back(tokenStream.str());

  return tokens;
}

Eigen::Matrix3d parseCovarianceMatrix(const std::string &covString) {
  std::string trimmed =
      covString.substr(1, covString.size() - 2); // Remove brackets
  auto covTokens = split(trimmed, ',');          // Use comma as delimiter here

  if (covTokens.size() != 9) {
    throw std::runtime_error("Invalid covariance matrix size");
  }

  Eigen::Matrix3d covariance;
  covariance << std::stod(covTokens[0]), std::stod(covTokens[1]),
      std::stod(covTokens[2]), std::stod(covTokens[3]), std::stod(covTokens[4]),
      std::stod(covTokens[5]), std::stod(covTokens[6]), std::stod(covTokens[7]),
      std::stod(covTokens[8]);
  return covariance;
}

bool APDataLoader::loadData(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return false;
  }

  std::string line;
  std::getline(file, line); // Skip header line

  while (std::getline(file, line)) {
    try {
      auto tokens = split(line, ',');

      if (tokens.size() != 6) {
        std::cerr << "Invalid line format: " << line << std::endl;
        continue; // Skip invalid line
      }

      std::string ssid = tokens[0];
      std::string mac = tokens[1];
      double x = std::stod(tokens[2]);
      double y = std::stod(tokens[3]);
      double z = std::stod(tokens[4]);
      std::string covarianceStr = tokens[5];

      Eigen::Vector3d position(x, y, z);
      Eigen::Matrix3d covariance = parseCovarianceMatrix(covarianceStr);

      _apData.emplace_back(ssid, mac, position, covariance);
    } catch (const std::invalid_argument &e) {
      std::cerr << "Invalid conversion in line: " << line << std::endl;
      continue; // Skip this line and continue processing the next lines
    } catch (const std::exception &e) {
      std::cerr << "Error processing line: " << line << ". Error: " << e.what()
                << std::endl;
      continue; // Skip this line and continue processing the next lines
    }
  }

  file.close();
  return !_apData.empty();
}

const std::vector<APData> &APDataLoader::getAPData() const { return _apData; }

void APDataLoader::printAPData() const {
  std::cout << std::fixed << std::setprecision(15); // Set precision for output
  for (const auto &ap : _apData) {
    std::cout << "SSID: " << ap.ssid << ", MAC: " << ap.mac << ", Position: ["
              << ap.position.x() << ", " << ap.position.y() << ", "
              << ap.position.z() << "]"
              << ", Covariance: [" << ap.covariance << "]" << std::endl;
  }
}