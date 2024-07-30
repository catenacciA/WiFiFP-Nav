import pandas as pd
from decimal import Decimal
from pathlib import Path
from typing import List, Tuple
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def load_trajectory_data(filepath: str) -> pd.DataFrame:
    trajectory_columns = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    data = pd.read_csv(filepath, sep=r"\s+", names=trajectory_columns, dtype=str)
    data = data.astype(str).map(Decimal)
    return data

def load_wifi_data(directory: str) -> Tuple[List[pd.DataFrame], List[int]]:
    wifi_files = sorted(Path(directory).glob("*"), key=lambda x: x.stem)
    wifi_scans = [pd.read_csv(file) for file in wifi_files]
    wifi_timestamps = [int(file.stem.split("_")[-1]) for file in wifi_files]
    return wifi_scans, wifi_timestamps

def convert_wifi_timestamps_to_seconds(wifi_timestamps: List[int]) -> List[Decimal]:
    return [Decimal(ts) / Decimal(1e9) for ts in wifi_timestamps]

def find_closest_trajectory_timestamp(trajectory_data: pd.DataFrame, wifi_timestamp: Decimal) -> pd.Series:
    closest_idx = (trajectory_data["timestamp"] - wifi_timestamp).abs().idxmin()
    return trajectory_data.iloc[closest_idx]

def calculate_distance(frequency: float, signal_dbm: float) -> float:
    return (27.55 - (20 * math.log10(frequency)) + abs(signal_dbm)) / 20.0

def create_fingerprinting_dataset(trajectory_data: pd.DataFrame, wifi_scans: List[pd.DataFrame], wifi_timestamps_in_seconds: List[Decimal]) -> pd.DataFrame:
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

def weighted_least_squares_trilateration(positions: np.ndarray, distances: np.ndarray, weights: np.ndarray) -> np.ndarray:
    def residuals(x, positions, distances, weights):
        return weights * (np.linalg.norm(positions - x, axis=1) - distances)

    initial_guess = np.mean(positions, axis=0)
    result = minimize(lambda x: np.sum(residuals(x, positions, distances, weights)**2), initial_guess, method='L-BFGS-B')
    
    return result.x

def estimate_ap_locations(fingerprinting_df: pd.DataFrame, nearest_n: int = None, max_distance: float = None) -> Tuple[pd.DataFrame, pd.DataFrame]:
    ap_positions = []
    ap_variances = []

    for mac, group in fingerprinting_df.groupby("mac"):
        ssid = group["ssid"].iloc[0]
        positions = group[["x", "y", "z"]].astype(float).values
        distances = np.array([calculate_distance(2400, signal_dbm) for signal_dbm in group["signalDBM"].astype(float)])
        
        if max_distance is not None:
            mask = distances <= max_distance
            positions = positions[mask]
            distances = distances[mask]
        
        if nearest_n is not None and len(positions) > nearest_n:
            sorted_indices = np.argsort(distances)
            positions = positions[sorted_indices[:nearest_n]]
            distances = distances[sorted_indices[:nearest_n]]

        weights = 1 / distances**2
        estimated_position = weighted_least_squares_trilateration(positions, distances, weights)
        deviations = positions - estimated_position
        covariance_matrix = np.cov(deviations.T)
        
        variance_x = covariance_matrix[0, 0]
        variance_y = covariance_matrix[1, 1]
        variance_z = covariance_matrix[2, 2]
        
        covariance_str = "[" + ", ".join(f"{covariance_matrix[i, j]:.6f}" for i in range(3) for j in range(3)) + "]"

        ap_positions.append({
            "SSID": ssid,
            "MAC": mac,
            "X": estimated_position[0],
            "Y": estimated_position[1],
            "Z": estimated_position[2],
            "Covariance": covariance_str
        })

        ap_variances.append({
            "SSID": ssid,
            "MAC": mac,
            "Variance_X": variance_x,
            "Variance_Y": variance_y,
            "Variance_Z": variance_z
        })

    ap_positions_df = pd.DataFrame(ap_positions, columns=["SSID", "MAC", "X", "Y", "Z", "Covariance"])
    ap_variances_df = pd.DataFrame(ap_variances, columns=["SSID", "MAC", "Variance_X", "Variance_Y", "Variance_Z"])
    
    ap_positions_df.sort_values(by="SSID", inplace=True)
    ap_variances_df.sort_values(by="SSID", inplace=True)
    
    return ap_positions_df, ap_variances_df

def visualize_aps_and_trajectory(ap_details, trajectory_data):
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
    trajectory_data = load_trajectory_data("lio_slam.txt")
    wifi_scans, wifi_timestamps = load_wifi_data("wifi")
    wifi_timestamps_in_seconds = convert_wifi_timestamps_to_seconds(wifi_timestamps)
    
    fingerprinting_dataset = create_fingerprinting_dataset(trajectory_data, wifi_scans, wifi_timestamps_in_seconds)
    fingerprinting_dataset.to_csv("fingerprinting_dataset.csv", index=False)

    nearest_n = 3
    max_distance = 10.0
    
    ap_positions_df, ap_variances_df = estimate_ap_locations(fingerprinting_dataset, nearest_n, max_distance)
    ap_positions_df.to_csv("ap_positions.csv", index=False)
    ap_variances_df.to_csv("ap_variances.csv", index=False)

    visualize_aps_and_trajectory(ap_positions_df, trajectory_data)

if __name__ == "__main__":
    main()
