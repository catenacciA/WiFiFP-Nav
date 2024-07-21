import pandas as pd
from decimal import Decimal
from pathlib import Path
from typing import List, Tuple
import math
import numpy as np
import matplotlib.pyplot as plt

def load_trajectory_data(filepath: str) -> pd.DataFrame:
    """Load trajectory data from a given file in TUM format."""
    trajectory_columns = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    data = pd.read_csv(filepath, sep=r"\s+", names=trajectory_columns, dtype=str)
    data = data.astype(str).map(Decimal)
    return data

def load_wifi_data(directory: str) -> Tuple[List[pd.DataFrame], List[int]]:
    """Load Wi-Fi scan data from files in a given directory."""
    wifi_files = sorted(Path(directory).glob("*"), key=lambda x: x.stem)
    wifi_scans = [pd.read_csv(file) for file in wifi_files]
    wifi_timestamps = [int(file.stem.split("_")[-1]) for file in wifi_files]
    return wifi_scans, wifi_timestamps

def convert_wifi_timestamps_to_seconds(wifi_timestamps: List[int]) -> List[Decimal]:
    """Convert Wi-Fi timestamps from nanoseconds to seconds using Decimal for precision."""
    return [Decimal(ts) / Decimal(1e9) for ts in wifi_timestamps]

def find_closest_trajectory_timestamp(trajectory_data: pd.DataFrame, wifi_timestamp: Decimal) -> pd.Series:
    """Find the trajectory timestamp closest to a given Wi-Fi timestamp."""
    closest_idx = (trajectory_data["timestamp"] - wifi_timestamp).abs().idxmin()
    return trajectory_data.iloc[closest_idx]

def calculate_distance(frequency: float, signal_dbm: float) -> float:
    """Calculate the distance based on the signal strength and frequency."""
    return (27.55 - (20 * math.log10(frequency)) + abs(signal_dbm)) / 20.0

def create_fingerprinting_dataset(trajectory_data: pd.DataFrame, wifi_scans: List[pd.DataFrame], wifi_timestamps_in_seconds: List[Decimal]) -> pd.DataFrame:
    """Create a fingerprinting dataset with the specified columns."""
    records = []

    for wifi_scan, wifi_timestamp in zip(wifi_scans, wifi_timestamps_in_seconds):
        closest_trajectory = find_closest_trajectory_timestamp(trajectory_data, wifi_timestamp)
        for _, row in wifi_scan.iterrows():
            record = {
                "timestamp": str(wifi_timestamp),
                "mac": row.get("mac"),
                "signalDBM": row.get("signalDBM"),
                "ssid": row.get("ssid"),
                "x": str(closest_trajectory["x"]),
                "y": str(closest_trajectory["y"]),
                "z": str(closest_trajectory["z"])
            }
            records.append(record)
    
    fingerprinting_df = pd.DataFrame(records, columns=["timestamp", "mac", "signalDBM", "ssid", "x", "y", "z"])
    return fingerprinting_df

def estimate_ap_locations(fingerprinting_df: pd.DataFrame) -> pd.DataFrame:
    """Estimate the position of each unique AP."""
    ap_positions = []

    for mac, group in fingerprinting_df.groupby("mac"):
        ssid = group["ssid"].iloc[0]
        x = group["x"].astype(float)
        y = group["y"].astype(float)
        z = group["z"].astype(float)
        
        mean_x = x.mean()
        mean_y = y.mean()
        mean_z = z.mean()

        covariance_matrix = np.cov(np.stack([x, y, z]), bias=True)
        
        covariance_str = "[" + ", ".join(f"{covariance_matrix[i, j]}" for i in range(3) for j in range(3)) + "]"

        ap_positions.append({
            "SSID": ssid,
            "MAC": mac,
            "X": mean_x,
            "Y": mean_y,
            "Z": mean_z,
            "Covariance": covariance_str
        })

    ap_positions_df = pd.DataFrame(ap_positions, columns=["SSID", "MAC", "X", "Y", "Z", "Covariance"])
    ap_positions_df.sort_values(by="SSID", inplace=True)
    return ap_positions_df

def visualize_aps_and_trajectory(ap_details, trajectory_data):
    """Visualize the APs and Trajectory in 3D without grouping."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    x = trajectory_data["x"].astype(float)
    y = trajectory_data["y"].astype(float)
    z = trajectory_data["z"].astype(float)
    ax.plot(x, y, z, label="Trajectory", alpha=0.5)

    ax.set_xlim([-12.5, 7.5])
    ax.set_ylim([-20.0, 10.0])
    ax.set_zlim([-10.0, 10.0])

    for _, row in ap_details.iterrows():
        ax.scatter(
            row["X"],
            row["Y"],
            row["Z"],
            label=f'{row["SSID"]} ({row["MAC"]})',
            marker="x",
        )

    ax.set_xlabel("X coordinate")
    ax.set_ylabel("Y coordinate")
    ax.set_zlabel("Z coordinate")
    ax.set_title("Estimated Positions of APs and Trajectory")

    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), fontsize="small")
    plt.show()

def main():
    trajectory_data = load_trajectory_data("../lio_slam.txt")
    wifi_scans, wifi_timestamps = load_wifi_data("../wifi")
    wifi_timestamps_in_seconds = convert_wifi_timestamps_to_seconds(wifi_timestamps)
    
    fingerprinting_dataset = create_fingerprinting_dataset(trajectory_data, wifi_scans, wifi_timestamps_in_seconds)
    fingerprinting_dataset.to_csv("fingerprinting_dataset.csv", index=False)

    ap_positions_df = estimate_ap_locations(fingerprinting_dataset)
    ap_positions_df.to_csv("ap_positions.csv", index=False)

    visualize_aps_and_trajectory(ap_positions_df, trajectory_data)

if __name__ == "__main__":
    main()
