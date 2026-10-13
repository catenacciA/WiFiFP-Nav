// wififp-scan: record WiFi scans to CSV on Linux (nl80211), for mapping or localization.
//
// Timestamps use the system (wall) clock in nanoseconds, the same clock ROS / SLAM
// pipelines stamp poses with, so the output can be paired with a trajectory directly.
// Triggering scans needs CAP_NET_ADMIN (e.g. run with sudo).

#include "wififp/io.hpp"

extern "C" {
#include "wifi_scan.h"
}

#include <CLI/CLI.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

std::string formatBssid(const uint8_t* b) {
  char buf[BSSID_STRING_LENGTH];
  std::snprintf(buf, sizeof buf, "%02x:%02x:%02x:%02x:%02x:%02x", b[0], b[1], b[2], b[3], b[4], b[5]);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Record WiFi scans to CSV (Linux, nl80211)"};
  std::string interface = "wlan0";
  std::filesystem::path out = "scans.csv";
  double period_s = 10.0;
  int count = 0;
  int max_age_ms = 3000;
  app.add_option("-i,--interface", interface, "Wireless interface")->capture_default_str();
  app.add_option("-o,--out", out, "CSV to append to")->capture_default_str();
  app.add_option("-p,--period", period_s, "Seconds between scan starts")->capture_default_str();
  app.add_option("-n,--count", count, "Number of scans (0 = until Ctrl-C)")->capture_default_str();
  app.add_option("--max-age", max_age_ms, "Drop BSS entries older than this (ms), stale cache")
      ->capture_default_str();
  CLI11_PARSE(app, argc, argv);

  std::signal(SIGINT, [](int) { g_stop = true; });

  wifi_scan* wifi = wifi_scan_init(interface.c_str());  // exits with a message on failure
  constexpr int kMaxBss = 256;
  bss_info bss[kMaxBss];

  for (int i = 0; !g_stop && (count == 0 || i < count); ++i) {
    const auto start = std::chrono::steady_clock::now();
    int n = wifi_scan_all(wifi, bss, kMaxBss);
    for (int retry = 0; n < 0 && errno == EBUSY && retry < 5 && !g_stop; ++retry) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      n = wifi_scan_all(wifi, bss, kMaxBss);
    }
    if (n < 0) {
      std::fprintf(stderr, "scan failed: %s\n", std::strerror(errno));
      break;
    }

    wififp::Scan scan;
    scan.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    for (int j = 0; j < std::min(n, kMaxBss); ++j) {
      if (bss[j].seen_ms_ago > max_age_ms) continue;
      scan.observations.push_back({formatBssid(bss[j].bssid), bss[j].ssid, static_cast<int>(bss[j].frequency),
                                   bss[j].signal_mbm / 100.0});
    }
    wififp::writeScans(out, {scan}, /*append=*/true);
    std::printf("scan %d: %zu access points -> %s\n", i + 1, scan.observations.size(), out.string().c_str());

    std::this_thread::sleep_until(start + std::chrono::duration<double>(period_s));
  }

  wifi_scan_close(wifi);
  return 0;
}
