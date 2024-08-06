cmake_minimum_required(VERSION 3.10)

project(WiFiFP-Nav VERSION 1.0 LANGUAGES CXX C)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Verbose makefile option
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Source and include directories
set(SOURCE_DIR src)
include_directories(include)

# Source files for the first executable
set(SOURCE_FILES
    ${SOURCE_DIR}/APData.cpp
    ${SOURCE_DIR}/APDataLoader.cpp
    ${SOURCE_DIR}/Scan.cpp
    ${SOURCE_DIR}/WiFiLocalization.cpp
    ${SOURCE_DIR}/WiFiMeasurementFactor.cpp
    ${SOURCE_DIR}/WiFiScanData.cpp
    ${SOURCE_DIR}/WiFiScanDataLoader.cpp
    lib/wifi_scan.c
    ${SOURCE_DIR}/main.cpp
)

# Source file for the second executable
set(MAP_SOURCE_FILE 
    ${SOURCE_DIR}/map/Utils.cpp
    ${SOURCE_DIR}/map/map.cpp
    ${SOURCE_DIR}/map/Fingerprinting.cpp
    ${SOURCE_DIR}/map/Trajectory.cpp
    ${SOURCE_DIR}/map/WiFiData.cpp
    ${SOURCE_DIR}/map/APPositionEstimator.cpp
)

# Find required packages
find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM CONFIG REQUIRED)

# Set the target names
set(TARGET_NAME_LOCALIZATOR localizator)
set(TARGET_NAME_MAP map_executable)

# Add executable target for the first executable
add_executable(${TARGET_NAME_LOCALIZATOR} ${SOURCE_FILES})

# Include directories and link libraries for the first executable
target_include_directories(${TARGET_NAME_LOCALIZATOR} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${TARGET_NAME_LOCALIZATOR} PRIVATE 
    JsonCpp::JsonCpp
    Eigen3::Eigen 
    gtsam
    mnl
)

# Add executable target for the second executable
add_executable(${TARGET_NAME_MAP} ${MAP_SOURCE_FILE})

# Include directories and link libraries for the second executable
target_include_directories(${TARGET_NAME_MAP} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${TARGET_NAME_MAP} PRIVATE 
    JsonCpp::JsonCpp
    Eigen3::Eigen 
    gtsam
)
