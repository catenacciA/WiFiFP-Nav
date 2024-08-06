# WiFiFP-Nav

This project estimates the location of a device based on WiFi signal strength. It includes a scanning component for collecting WiFi signal strength data and a position estimation component using an optimization algorithm to estimate the device's location.

## Table of Contents
- [WiFiFP-Nav](#wififp-nav)
  - [Table of Contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
    - [1. Clone the Repository](#1-clone-the-repository)
    - [2. Install vcpkg](#2-install-vcpkg)
    - [3. Install Dependencies via vcpkg](#3-install-dependencies-via-vcpkg)
      - [3.1 Install WiFi Scan Library](#31-install-wifi-scan-library)
    - [4. Set Up the Build System](#4-set-up-the-build-system)
  - [Usage](#usage)
    - [Creating the WiFi Map](#creating-the-wifi-map)
    - [Visualizing the WiFi Map](#visualizing-the-wifi-map)
    - [Running the Localizator](#running-the-localizator)
      - [Using a Specific Network Interface](#using-a-specific-network-interface)
      - [Using a Pre-recorded Scan File](#using-a-pre-recorded-scan-file)
    - [Summary of Commands](#summary-of-commands)
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

#### 3.1 Install WiFi Scan Library
To install the WiFi scan library, you need to install the `libmnl` library, just use your machine's package manager:
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
make -j
```

## Usage

### Creating the WiFi Map
1. **Prepare the `lio_slam.txt` File**:
   - Obtain the `lio_slam.txt` file from your LIO-SLAM process. This file contains the normal LIO SLAM data.
   - Place the `lio_slam.txt` file in the `datasets` directory.

2. **Create a WiFi Folder**:
   - In the `datasets` directory, create a folder named `wifi`.
   - Collect WiFi scans in the environment where the localization will occur.
   - Ensure the filenames of these WiFi files are epoch timestamps corresponding to the time they were taken.
   - Synchronize these WiFi scans with the LIO-SLAM trajectory by ensuring they are taken simultaneously along the LIO-SLAM trajectory.

3. **Run the Map Executable**:
   - After placing the required files (`lio_slam.txt` and the WiFi folder), build the map executable:
     ```sh
     cd build
     ./map_executable
     ```
   - This command will generate two CSV files:
     - `fingerprinting_dataset.csv`: The dataset used by `map_executable` to estimate the AP positions.
     - `ap_positions.csv`: Contains the estimated positions of the Access Points (APs).

### Visualizing the WiFi Map
If you want to visualize the map:
1. **Set Up the Python Environment**:
   - Navigate to the scripts directory:
     ```sh
     cd scripts
     ```
   - Create a virtual environment and install dependencies:
     ```sh
     python3 -m venv venv
     source venv/bin/activate
     pip install -r requirements.txt
     ```

2. **Run the Visualization Script**:
   - Execute the visualization script to display the map:
     ```sh
     python plot.py
     ```

### Running the Localizator

To run the localizator, execute the following command:

```sh
cd build
sudo ./localizator
```

This command will start the WiFi scan process using the default network interface (`wlp1s0f0`) and estimate the device's location based on the WiFi signal strength. The estimated location will be displayed in the terminal.

#### Using a Specific Network Interface

If you want to specify a different network interface for scanning, use the following command, replacing `INTERFACE_NAME` with the desired interface (e.g., `wlan0`):

```sh
sudo ./localizator INTERFACE_NAME
```

#### Using a Pre-recorded Scan File

If you prefer to use a pre-recorded scan file, ensure you have a CSV file named `scan.csv` in the root directory with the following header:

```
MAC Address,RSSI Value,Mean,STD,Frequency
```

To run the localizator using this pre-recorded scan file, use the following command:

```sh
sudo ./localizator --use-file
```

### Summary of Commands

- **Default usage with live scanning on the default interface:**
  ```sh
  sudo ./localizator
  ```

- **Specify a different interface for live scanning:**
  ```sh
  sudo ./localizator wlan0
  ```

- **Use a pre-recorded scan file without live scanning:**
  ```sh
  sudo ./localizator --use-file
  ```

## Acknowledgements
This project includes the following libraries:
- [wifi-scan](https://github.com/bmegli/wifi-scan): C/C++ library for scanning WiFi signals
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp): JSON parser for C++
- [gtsam](https://github.com/borglab/gtsam): Library for factor graph optimization
- [eigen3](https://gitlab.com/libeigen/eigen): C++ template library for linear algebra
- libmnl: Library for managing Netlink messages

## Contributing
Contributions are welcome! To contribute, please follow these steps:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## License
This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.
