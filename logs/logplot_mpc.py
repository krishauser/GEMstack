###Jugal's Code###
###Following file gives the plots for MPC logs. Go inside logs folder where the logs are saved and run- python3 logplot_mpc.py "filename" to get the plots.
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def find_mpc_csv(directory):
    for filename in os.listdir(directory):
        if filename.startswith("MPCTrajectoryTracker_debug") and filename.endswith(".csv"):
            return os.path.join(directory, filename)
    raise FileNotFoundError("No MPCTrajectoryTracker_debug.csv found in the directory.")

def plot_mpc_debug(csv_path):
    df = pd.read_csv(csv_path)

    time = df['mpc/accel time'] - df['mpc/accel time'].iloc[0]
    accel = df['mpc/accel']
    delta = df['mpc/delta']
    state_x = df['mpc/state_x']
    state_y = df['mpc/state_y']
    target_x = df['mpc/target_x']
    target_y = df['mpc/target_y']
    yaw = df['mpc/state_yaw']
    target_theta = df['mpc/target_theta']

    jerk_time = time[1:]
    jerk = np.diff(accel) / np.diff(time)
    heading_acc = np.diff(yaw) / np.diff(time)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.subplots_adjust(hspace=0.4, wspace=0.3)

    axs[0,0].plot(jerk_time, jerk, color='blue')
    axs[0,0].set_title("Vehicle Jerk Over Time")
    axs[0,0].set_xlabel("Time (s)")
    axs[0,0].set_ylabel("Jerk (m/s³)")
    axs[0,0].grid(True)

    axs[0,1].plot(jerk_time, heading_acc, color='orange')
    axs[0,1].set_title("Heading Acceleration Over Time")
    axs[0,1].set_xlabel("Time (s)")
    axs[0,1].set_ylabel("Heading Acceleration (rad/s²)")
    axs[0,1].grid(True)

    axs[1,0].plot(time, state_x - target_x, label="CTE X", color='green')
    axs[1,0].plot(time, state_y - target_y, label="CTE Y", color='purple')
    axs[1,0].set_title("Cross Track Error Over Time")
    axs[1,0].set_xlabel("Time (s)")
    axs[1,0].set_ylabel("CTE (m)")
    axs[1,0].legend()
    axs[1,0].grid(True)

    axs[1,1].plot(state_x, state_y, label='Actual Trajectory', color='blue')
    axs[1,1].plot(target_x, target_y, label='Target Trajectory', color='orange')
    axs[1,1].set_title("Trajectory Comparison")
    axs[1,1].set_xlabel("X (m)")
    axs[1,1].set_ylabel("Y (m)")
    axs[1,1].legend()
    axs[1,1].grid(True)

    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 logplot_mpc.py <log_directory>")
        sys.exit(1)

    log_dir = sys.argv[1]
    if not os.path.isdir(log_dir):
        print(f"Error: '{log_dir}' is not a valid directory.")
        sys.exit(1)

    try:
        csv_file = find_mpc_csv(log_dir)
        print(f"Found MPC CSV: {csv_file}")
        plot_mpc_debug(csv_file)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

