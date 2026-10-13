# wifi-scan

Vendored from [bmegli/wifi-scan](https://github.com/bmegli/wifi-scan) by Bartosz Meglicki.
Licensed under the [Mozilla Public License 2.0](https://mozilla.org/MPL/2.0/) (see file headers).

A minimal C library that triggers and reads WiFi scans through the Linux nl80211
netlink interface (via `libmnl`). Used only by the optional `wififp-scan` recorder.
Only change: the include path of `wifi_scan.h`.
