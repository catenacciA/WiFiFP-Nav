#include "../../include/map/Utils.hpp"
#include <limits>
#include <sstream>

vector<string> split(const string &str, char delimiter) {
  vector<string> tokens;
  string token;
  istringstream tokenStream(str);
  while (getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Define the maximum value for the index mask
const size_t maxIndexMask = 72057594037927935;

// Helper function to reduce hash to fit within the mask
size_t reduceHashValue(size_t hashValue) { return hashValue % maxIndexMask; }
