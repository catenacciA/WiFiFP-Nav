import pandas as pd
from decimal import Decimal
from pathlib import Path
from typing import List, Tuple
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

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
    # Using the original path loss model for distance estimation
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

def weighted_least_squares_trilateration(positions: np.ndarray, distances: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Estimate the position of the AP using a weighted least squares optimization approach."""
    def residuals(x, positions, distances, weights):
        return weights * (np.linalg.norm(positions - x, axis=1) - distances)

    initial_guess = np.mean(positions, axis=0)
    result = minimize(lambda x: np.sum(residuals(x, positions, distances, weights)**2), initial_guess, method='L-BFGS-B')
    
    return result.x

def estimate_ap_locations(fingerprinting_df: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Estimate the position of each unique AP and store variances separately."""
    ap_positions = []
    ap_variances = []

    for mac, group in fingerprinting_df.groupby("mac"):
        ssid = group["ssid"].iloc[0]
        positions = group[["x", "y", "z"]].astype(float).values
        distances = np.array([calculate_distance(2400, signal_dbm) for signal_dbm in group["signalDBM"].astype(float)])
        
        # Filter out outliers
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        mask = (distances >= mean_distance - 2 * std_distance) & (distances <= mean_distance + 2 * std_distance)
        positions = positions[mask]
        distances = distances[mask]
        
        # Calculate weights based on signal strength
        weights = 1 / distances**2

        # Perform weighted least squares trilateration to estimate AP position
        estimated_position = weighted_least_squares_trilateration(positions, distances, weights)
        
        # Calculate deviations from estimated position
        deviations = positions - estimated_position
        
        # Calculate covariance matrix of the deviations
        covariance_matrix = np.cov(deviations.T)
        
        # Extract variance directly from the covariance matrix
        variance_x = covariance_matrix[0, 0]
        variance_y = covariance_matrix[1, 1]
        variance_z = covariance_matrix[2, 2]
        
        # Convert covariance matrix to string for storage
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

""" def calculate_euclidean_distance(point1: List[float], point2: List[float]) -> float:
    Calculate the Euclidean distance between two 3D points.
    return np.linalg.norm(np.array(point1) - np.array(point2))

def evaluate_position(hypothetical_position: List[float], ap_data: pd.DataFrame) -> pd.DataFrame:
    Evaluate a hypothetical position using mean RSSI values from APs.
    evaluations = []
    for _, row in ap_data.iterrows():
        estimated_distance = calculate_distance(row["Frequency"], row["Mean"])
        ap_position = np.array([row["X"], row["Y"], row["Z"]])
        hyp_position = np.array(hypothetical_position)
        distance = np.linalg.norm(ap_position - hyp_position)
        evaluations.append({
            "MAC": row["MAC"],
            "Estimated Distance": estimated_distance,
            "Hypothetical Distance": distance,
            "Difference": abs(estimated_distance - distance)
        })
    evaluation_df = pd.DataFrame(evaluations, columns=["MAC", "Estimated Distance", "Hypothetical Distance", "Difference"])
    return evaluation_df


def find_closest_position(ap_mac_addresses: List[str], mean_rssi_values: List[float], fingerprinting_df: pd.DataFrame) -> pd.Series:
    
    Find the closest trajectory position based on mean RSSI values from a list of APs.
    
    :param ap_mac_addresses: List of AP MAC addresses.
    :param mean_rssi_values: List of mean RSSI values corresponding to the AP MAC addresses.
    :param fingerprinting_df: DataFrame containing the fingerprinting dataset.
    :return: Series representing the closest trajectory position.
    
    assert len(ap_mac_addresses) == len(mean_rssi_values), "The number of MAC addresses must match the number of RSSI values."

    # Filter the dataset for the given AP MAC addresses
    filtered_df = fingerprinting_df[fingerprinting_df['mac'].isin(ap_mac_addresses)]

    # Calculate the difference between the given mean RSSI values and the recorded RSSI values
    distance_df = pd.DataFrame()

    for mac, mean_rssi in zip(ap_mac_addresses, mean_rssi_values):
        mac_filtered_df = filtered_df[filtered_df['mac'] == mac].copy()
        mac_filtered_df['rssi_diff'] = (mac_filtered_df['signalDBM'].astype(float) - mean_rssi).abs()
        distance_df = pd.concat([distance_df, mac_filtered_df])

    # Group by timestamp and sum the rssi_diff to get the total difference for all APs at each timestamp
    grouped_df = distance_df.groupby('timestamp').agg({'rssi_diff': 'sum', 'x': 'first', 'y': 'first', 'z': 'first'}).reset_index()

    # Find the row with the minimum total difference
    closest_position = grouped_df.loc[grouped_df['rssi_diff'].idxmin()]

    return closest_position """

def main():
    trajectory_data = load_trajectory_data("../lio_slam.txt")
    wifi_scans, wifi_timestamps = load_wifi_data("../wifi")
    wifi_timestamps_in_seconds = convert_wifi_timestamps_to_seconds(wifi_timestamps)
    
    fingerprinting_dataset = create_fingerprinting_dataset(trajectory_data, wifi_scans, wifi_timestamps_in_seconds)
    fingerprinting_dataset.to_csv("fingerprinting_dataset.csv", index=False)

    ap_positions_df, ap_variances_df = estimate_ap_locations(fingerprinting_dataset)
    ap_positions_df.to_csv("ap_positions.csv", index=False)
    ap_variances_df.to_csv("ap_variances.csv", index=False)

    visualize_aps_and_trajectory(ap_positions_df, trajectory_data)

"""     hypothetical_position = [2.89319855549, -13.0765297555, -5.27105065983]

    # Retrieve actual AP coordinates from the ap_positions_df using the provided MAC addresses
    mac_addresses = ["54:af:97:df:76:eb", "4c:9e:ff:8e:2b:c1", "4e:3b:ff:83:95:3c"]
    mean_rssi_values = [205.65, 191.56, 192.56]
    ap_data = ap_positions_df[ap_positions_df["MAC"].isin(mac_addresses)]

    # Add mean RSSI values and frequencies to the ap_data DataFrame
    rssi_values = pd.DataFrame({
        "MAC": ["54:af:97:df:76:eb", "4c:9e:ff:8e:2b:c1", "4e:3b:ff:83:95:3c"],
        "Mean": [205.65, 191.56, 192.56],
        "STD": [10.32, 11.86, 11.9],
        "Frequency": [5220, 5240, 5240]
    })

    ap_data = ap_data.merge(rssi_values, on="MAC")

    evaluation_df = evaluate_position(hypothetical_position, ap_data)
    print(evaluation_df)

    closest_position = find_closest_position(mac_addresses, mean_rssi_values, fingerprinting_dataset)
    print("Closest position based on given RSSI values:")
    print(closest_position)

    closest_position_coords = [float(closest_position['x']), float(closest_position['y']), float(closest_position['z'])]
    distance_to_hypothetical = calculate_euclidean_distance(hypothetical_position, closest_position_coords)
    print(f"Distance to hypothetical position: {distance_to_hypothetical}") """

if __name__ == "__main__":
    main()
