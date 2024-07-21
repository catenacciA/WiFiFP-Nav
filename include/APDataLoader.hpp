#ifndef APDATALOADER_HPP
#define APDATALOADER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "APData.hpp"
#include "CSVLoader.hpp"

class APDataLoader : public CSVLoader {
public:
  APDataLoader() = default;
  bool loadData(const std::string &filename) override;
  const std::vector<APData> &getAPData() const;
  void printAPData() const;

private:
  std::vector<APData> _apData;
};

#endif // APDATALOADER_HPP
