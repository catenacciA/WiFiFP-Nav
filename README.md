This project estimates the location of a device based on WiFi signal strength. It includes a scanning component for collecting WiFi signal strength data and a position estimation component using an optimization algorithm to estimate the device's location.

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Clone the Repository](#1-clone-the-repository)
  - [2. Install vcpkg](#2-install-vcpkg)
  - [3. Install Dependencies via vcpkg](#3-install-dependencies-via-vcpkg)
  - [3.1 Install dependencies for wifi\_scan](#31-install-dependencies-for-wifi_scan)
  - [4. Set Up the Build System](#4-set-up-the-build-system)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)
- [Contributing](#contributing)
- [License](#license)

## Prerequisites
Before you begin, ensure you have the following installed:
- **C++ Compiler**: e.g., g++, clang
- **CMake**
- **Python 3.x**
- **vcpkg**: For managing C++ dependencies
- **eigen3**: For linear algebra operations
- **gtsam**: For factor graph optimization
- **jsoncpp**: For JSON parsing

## Installation

### 1. Clone the Repository
Clone the project repository to your local machine:
```sh
git clone https://github.com/catenacciA/WiFiFP-Nav.git
cd WiFiFP-Nav
```

### 2. Install vcpkg
Install vcpkg for managing the project dependencies:
```sh
git clone https://github.com/Microsoft/vcpkg.git /usr/local/vcpkg
cd /usr/local/vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
```

### 3. Install Dependencies via vcpkg
Use vcpkg to install the required libraries:
```sh
/usr/local/vcpkg/vcpkg install jsoncpp
/usr/local/vcpkg/vcpkg install gtsam
/usr/local/vcpkg/vcpkg install eigen3
```

### 3.1 Install dependencies for wifi_scan
```sh
sudo apt-get install libmnl0 libmnl-dev
```

### 4. Set Up the Build System
Create a build directory and configure the project using CMake:
```sh
cd ~/WiFiFP-Nav
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/usr/local/vcpkg/scripts/buildsystems/vcpkg.cmake ..
make
```

## Usage

To run the FingerprintPositionEstimator:
```sh
cd build
sudo ./localizator
```

Optionally, there's the possibility to run the optimizator without making a scan with the following command:
```sh
sudo ./localizator --use-file
``` 

## Acknowledgements
This project includes the following libraries:
- [wifi-scan](https://github.com/bmegli/wifi-scan): C/C++ library for scanning WiFi signals
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp): JSON parser for C++
- [gtsam](https://github.com/borglab/gtsam): Library for factor graph optimization

## Contributing
Contributions are welcome! To contribute, please follow these steps:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## License
This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.
