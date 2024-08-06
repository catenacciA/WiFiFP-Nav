import pandas as pd
import matplotlib.pyplot as plt
from decimal import Decimal
import mplcursors

def load_trajectory_data(filepath: str) -> pd.DataFrame:
    trajectory_columns = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    data = pd.read_csv(filepath, sep=r"\s+", names=trajectory_columns, dtype=str)
    data = data.astype(str).map(Decimal)
    return data

def load_ap_data(filepath: str) -> pd.DataFrame:
    # Load the AP positions from the provided CSV file
    ap_data = pd.read_csv(filepath)
    return ap_data

def visualize_aps_and_trajectory(ap_data: pd.DataFrame, trajectory_data: pd.DataFrame):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the trajectory
    x = trajectory_data["x"].astype(float)
    y = trajectory_data["y"].astype(float)
    z = trajectory_data["z"].astype(float)
    trajectory_plot, = ax.plot(x, y, z, label="Trajectory", alpha=0.5)

    # Set axis limits
    ax.set_xlim([-12.5, 7.5])
    ax.set_ylim([-20.0, 10.0])
    ax.set_zlim([-10.0, 10.0])

    # Plot the APs
    sc = ax.scatter(
        ap_data["X"],
        ap_data["Y"],
        ap_data["Z"],
        marker="x",
        label="Access Points",
        c='r'
    )

    # Add hover annotations for APs
    cursor = mplcursors.cursor(sc, hover=True)
    @cursor.connect("add")
    def on_add(sel):
        idx = sel.index
        sel.annotation.set(text=f'{ap_data["SSID"].iloc[idx]} ({ap_data["MAC"].iloc[idx]})')

    # Add hover annotations for trajectory
    cursor_trajectory = mplcursors.cursor(trajectory_plot, hover=True)
    @cursor_trajectory.connect("add")
    def on_add(sel):
        sel.annotation.set(text=f'Time: {trajectory_data["timestamp"].iloc[sel.index]}')

    ax.set_xlabel("X coordinate")
    ax.set_ylabel("Y coordinate")
    ax.set_zlabel("Z coordinate")
    ax.set_title("Estimated Positions of APs and Trajectory")

    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), fontsize="small")
    plt.show()

def main():
    # Load trajectory data from a fixed file path
    trajectory_data = load_trajectory_data("../../../../datasets/lio_slam.txt")
    
    # Load AP data from a CSV file provided by the user
    ap_data = load_ap_data("../../../../build/ap_positions.csv")

    # Visualize the trajectory and AP positions
    visualize_aps_and_trajectory(ap_data, trajectory_data)

if __name__ == "__main__":
    main()
