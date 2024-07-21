cmake_minimum_required(VERSION 3.10)

project(WiFiFP-Nav VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_VERBOSE_MAKEFILE OFF)

set(SOURCE_DIR src)

include_directories(include)

# Source files
set(SOURCE_FILES
    ${SOURCE_DIR}/APData.cpp
    ${SOURCE_DIR}/APDataLoader.cpp
    ${SOURCE_DIR}/WiFiLocalization.cpp
    ${SOURCE_DIR}/WiFiMeasurementFactor.cpp
    ${SOURCE_DIR}/WiFiScanData.cpp
    ${SOURCE_DIR}/WiFiScanDataLoader.cpp
    ${SOURCE_DIR}/main.cpp
)

find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM CONFIG REQUIRED)

if(NOT jsoncpp_FOUND)
    message(FATAL_ERROR "jsoncpp not found")
endif()

if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found")
endif()

if(NOT GTSAM_FOUND)
    message(FATAL_ERROR "GTSAM not found")
endif()

set(TARGET_NAME localizator)

add_executable(${TARGET_NAME} ${SOURCE_FILES})

target_include_directories(${TARGET_NAME} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${TARGET_NAME} PRIVATE 
    JsonCpp::JsonCpp 
    Eigen3::Eigen 
    gtsam
)
