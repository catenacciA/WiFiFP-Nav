#ifndef CSVLOADER_HPP
#define CSVLOADER_HPP

#include <string>

class CSVLoader {
public:
  virtual bool loadData(const std::string &filename) = 0;
  virtual ~CSVLoader() = default;
};

#endif // CSVLOADER_HPP
