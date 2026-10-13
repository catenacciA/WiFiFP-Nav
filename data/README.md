# Datasets

## `office/`: one survey walk through an office floor

| | |
|---|---|
| Recorded | May 2024 |
| Area | one office floor (rooms + corridor); the walk spans 13 × 19 m |
| Walk | 340 s, 127 m, 3407 poses at 10 Hz |
| WiFi | 34 scans, one every 10 s (33 inside the trajectory's time span) |
| Access points | 115 BSSIDs on 2.4 and 5 GHz: 24 distinct SSIDs plus 7 hidden networks |
| Sensors | LiDAR-inertial SLAM for the trajectory, USB WiFi adapter for the scans (see the main README) |

| File | Content |
|---|---|
| `trajectory.tum` | SLAM trajectory, [TUM format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats): `t x y z qx qy qz qw`, seconds and metres |
| `scans.csv` | `timestamp_ns,bssid,ssid,freq_mhz,rssi_dbm`: one row per access point per scan |
| `floorplan.png`, `floorplan.yaml` | 5 cm occupancy image of the LiDAR map, sliced at 0.3–1.8 m height, in the [ROS map_server](https://wiki.ros.org/map_server#YAML_format) convention. It shares the trajectory's frame and is only used for figures. |

### Anonymization

The raw recording was converted with
[`scripts/prepare_dataset.py --anonymize`](../scripts/prepare_dataset.py):

- every BSSID was replaced by a random, locally administered MAC (`x2:xx:xx:xx:xx:xx`),
  consistently across scans;
- every SSID was replaced by `net-NN`, numbered in random order. Hidden networks stay empty;
- the 3D point cloud was reduced to the 2D wall outline in `floorplan.png`.

