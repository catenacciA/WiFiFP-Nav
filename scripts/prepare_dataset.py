#!/usr/bin/env python3
"""Convert a raw recording into the WiFiFP-Nav dataset format.

The raw recording (as produced by the original logger) consists of
  * a LIO-SLAM trajectory in TUM format,
  * a directory with one CSV per WiFi scan, named by its timestamp in ns,
    with columns ``mac,frequency,ssid,signalDBM,signalAvgDBM``,
  * optionally a point-cloud map (binary PCD) in the same frame as the trajectory.

Output (see data/README.md):
  * ``trajectory.tum``  - copied unchanged,
  * ``scans.csv``       - one row per observation, RSSI as signed dBm,
  * ``floorplan.png`` + ``floorplan.yaml`` - 2D occupancy slice of the point cloud
    (ROS map_server convention).

The original logger wrote the signed 8-bit RSSI as an unsigned byte
(-48 dBm was stored as 208); this script undoes that. With ``--anonymize``,
BSSIDs are replaced by random locally-administered MACs and SSIDs by
``net-NN``; the mapping is random and not stored.
"""

from __future__ import annotations

import argparse
import csv
import random
import secrets
import shutil
from pathlib import Path

import numpy as np
from PIL import Image


def signed_dbm(raw: int) -> int:
    return raw - 256 if raw > 127 else raw


def random_bssid(rng: random.Random, taken: set[str]) -> str:
    while True:
        octets = [rng.randrange(256) for _ in range(6)]
        octets[0] = (octets[0] & 0xFC) | 0x02  # unicast, locally administered
        mac = ":".join(f"{o:02x}" for o in octets)
        if mac not in taken:
            taken.add(mac)
            return mac


def normalize_mac(mac: str) -> str:
    # The logger printed octets without zero padding ("34:60:f9:2a:4:af").
    return ":".join(f"{int(o, 16):02x}" for o in mac.split(":"))


def convert_scans(wifi_dir: Path, out_csv: Path, anonymize: bool) -> tuple[int, int]:
    files = sorted((p for p in wifi_dir.iterdir() if p.name.isdigit()), key=lambda p: int(p.name))
    rng = random.Random(secrets.randbits(64))
    bssid_map: dict[str, str] = {}
    ssid_map: dict[str, str] = {}
    taken: set[str] = set()
    rows = []
    for path in files:
        with path.open(newline="") as f:
            for rec in csv.DictReader(f):
                bssid = normalize_mac(rec["mac"])
                ssid = rec["ssid"]
                if anonymize:
                    if bssid not in bssid_map:
                        bssid_map[bssid] = random_bssid(rng, taken)
                    bssid = bssid_map[bssid]
                    if ssid:  # hidden networks stay hidden
                        ssid_map.setdefault(ssid, "")
                rows.append([path.name, bssid, ssid, int(rec["frequency"]), signed_dbm(int(rec["signalDBM"]))])

    if anonymize:  # number SSIDs in random order so names carry no ordering information
        names = list(ssid_map)
        rng.shuffle(names)
        for i, name in enumerate(names, 1):
            ssid_map[name] = f"net-{i:02d}"
        for row in rows:
            if row[2]:
                row[2] = ssid_map[row[2]]

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ns", "bssid", "ssid", "freq_mhz", "rssi_dbm"])
        w.writerows(rows)
    return len(files), len({r[1] for r in rows})


def read_pcd_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    header_end = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = raw[:header_end].decode()
    fields = next(l for l in header.splitlines() if l.startswith("FIELDS")).split()[1:]
    pts = np.frombuffer(raw[header_end:], dtype=np.float32).reshape(-1, len(fields))
    return pts[:, :3].astype(np.float64)


def write_floorplan(pcd: Path, trajectory: np.ndarray, out_dir: Path,
                    resolution: float, margin: float, z_min: float, z_max: float) -> None:
    pts = read_pcd_xyz(pcd)
    lo = trajectory[:, :2].min(0) - margin
    hi = trajectory[:, :2].max(0) + margin
    keep = (pts[:, 2] > z_min) & (pts[:, 2] < z_max) & np.all((pts[:, :2] >= lo) & (pts[:, :2] < hi), axis=1)
    ij = ((pts[keep, :2] - lo) / resolution).astype(int)
    w, h = np.ceil((hi - lo) / resolution).astype(int)
    hits = np.zeros((h, w), dtype=np.int32)
    np.add.at(hits, (ij[:, 1], ij[:, 0]), 1)
    occupied = hits >= 2
    img = np.where(occupied, 0, 254).astype(np.uint8)[::-1]  # row 0 is the top (max y)
    Image.fromarray(img).save(out_dir / "floorplan.png", optimize=True)
    (out_dir / "floorplan.yaml").write_text(
        "image: floorplan.png\n"
        f"resolution: {resolution}\n"
        f"origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        f"# horizontal slice of the LiDAR map between z={z_min} and z={z_max} m\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trajectory", type=Path, required=True)
    ap.add_argument("--wifi-dir", type=Path, required=True)
    ap.add_argument("--pcd", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--anonymize", action="store_true")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=6.0)
    ap.add_argument("--slice", type=float, nargs=2, default=(0.3, 1.8), metavar=("ZMIN", "ZMAX"))
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.trajectory, args.out / "trajectory.tum")
    n_scans, n_bssids = convert_scans(args.wifi_dir, args.out / "scans.csv", args.anonymize)
    print(f"scans.csv: {n_scans} scans, {n_bssids} BSSIDs")
    if args.pcd:
        traj = np.loadtxt(args.trajectory)[:, 1:4]
        write_floorplan(args.pcd, traj, args.out, args.resolution, args.margin, *args.slice)
        print("floorplan.png written")


if __name__ == "__main__":
    main()
