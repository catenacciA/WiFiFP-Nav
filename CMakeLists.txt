cmake_minimum_required(VERSION 3.16)
project(WiFiFP-Nav VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(WIFIFP_BUILD_TESTS "Build unit tests" ON)

# The scanner talks to the Linux kernel through nl80211 and needs libmnl.
find_library(MNL_LIBRARY mnl)
find_path(MNL_INCLUDE_DIR libmnl/libmnl.h)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND MNL_LIBRARY AND MNL_INCLUDE_DIR)
  set(_scanner_default ON)
else()
  set(_scanner_default OFF)
endif()
option(WIFIFP_BUILD_SCANNER "Build wififp-scan (Linux + libmnl)" ${_scanner_default})

find_package(GTSAM REQUIRED)
message(STATUS "GTSAM ${GTSAM_VERSION}")

include(FetchContent)
FetchContent_Declare(cli11
  URL https://github.com/CLIUtils/CLI11/archive/refs/tags/v2.4.2.tar.gz
  URL_HASH SHA256=f2d893a65c3b1324c50d4e682c0cdc021dd0477ae2c048544f39eed6654b699a
  DOWNLOAD_EXTRACT_TIMESTAMP ON)
set(CLI11_PRECOMPILED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cli11)

set(WIFIFP_WARNINGS $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>)

# ---------------------------------------------------------------- library
add_library(wififp
  src/evaluation.cpp
  src/io.cpp
  src/localizer.cpp
  src/path_loss.cpp
  src/radio_map.cpp
  src/trajectory.cpp)
add_library(wififp::wififp ALIAS wififp)
target_include_directories(wififp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(wififp PUBLIC gtsam)
target_compile_options(wififp PRIVATE ${WIFIFP_WARNINGS})

# ---------------------------------------------------------------- apps
add_executable(wififp_cli apps/wififp.cpp)
set_target_properties(wififp_cli PROPERTIES OUTPUT_NAME wififp)
target_link_libraries(wififp_cli PRIVATE wififp CLI11::CLI11)
target_compile_options(wififp_cli PRIVATE ${WIFIFP_WARNINGS})

if(WIFIFP_BUILD_SCANNER)
  add_library(wifi_scan STATIC third_party/wifi-scan/wifi_scan.c)
  target_include_directories(wifi_scan PUBLIC third_party/wifi-scan ${MNL_INCLUDE_DIR})
  target_link_libraries(wifi_scan PUBLIC ${MNL_LIBRARY})

  add_executable(wififp_scan apps/wififp_scan.cpp)
  set_target_properties(wififp_scan PROPERTIES OUTPUT_NAME wififp-scan)
  target_link_libraries(wififp_scan PRIVATE wififp wifi_scan CLI11::CLI11)
  target_compile_options(wififp_scan PRIVATE ${WIFIFP_WARNINGS})
endif()

# ---------------------------------------------------------------- tests
if(WIFIFP_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    DOWNLOAD_EXTRACT_TIMESTAMP ON)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(wififp_tests tests/test_estimation.cpp tests/test_io.cpp tests/test_path_loss.cpp)
  target_link_libraries(wififp_tests PRIVATE wififp GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(wififp_tests)
endif()

install(TARGETS wififp_cli RUNTIME DESTINATION bin)
if(WIFIFP_BUILD_SCANNER)
  install(TARGETS wififp_scan RUNTIME DESTINATION bin)
endif()
