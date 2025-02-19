import json
import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm 
import matplotlib.colors as mcolors
import numpy as np

ACC_SAFE = 0.8 # |acceleration| <= 2.0 m/s² is safe 
ACC_UNSAFE = 2.0 # |acceleration| >= 3.0 m/s² is unsafe
HR_SAFE = 0.05 # |heading_rate| <= 0.1 rad/s is safe 
HR_UNSAFE = 0.2 # |heading_rate| >= 0.3 rad/s is unsafe

def parse_log(filename):
    """Reads a JSON log file and extracts time, acceleration, and heading_rate"""
    times = []
    accelerations = []
    heading_rates = []

    with open(filename, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"Skipping invalid JSON line: {line.strip()}")
                continue
            if "vehicle" in entry:
                t = entry.get("time")
                vehicle_data = entry["vehicle"].get("data", {})
                acceleration = vehicle_data.get("acceleration")
                heading_rate = vehicle_data.get("heading_rate")
                # Only add if all required fields are available
                if t is not None and acceleration is not None and heading_rate is not None:
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
    
    return np.array(times), np.array(accelerations), np.array(heading_rates)

def compute_safety_factor(value, safe_thresh, unsafe_thresh):
    """Computes a safety factor between 0 and 1 to be used by plots"""
    abs_val = abs(value)
    if abs_val <= safe_thresh:
        return 1.0
    elif abs_val >= unsafe_thresh:
        return 0.0
    else:
        return 1.0 - (abs_val - safe_thresh) / (unsafe_thresh - safe_thresh)

def plot_metrics(times, accelerations, heading_rates):
    """Creates two subplots:
        - Instantaneous acceleration vs. time
        - Heading rate vs. time
    """
    # Compute safety factors for acceleration and heading rate
    acc_safety = np.vectorize(compute_safety_factor)(accelerations, ACC_SAFE, ACC_UNSAFE)
    hr_safety = np.vectorize(compute_safety_factor)(heading_rates, HR_SAFE, HR_UNSAFE)

    fig, axs = plt.subplots(2, 1, figsize=(12, 8))
    cmap = cm.get_cmap('RdYlGn')
    # Plot acceleration with scatter coloring
    sc1 = axs[0].scatter(times, accelerations, c=acc_safety, cmap=cmap, vmin=0, vmax=1, marker='o')
    axs[0].plot(times, accelerations, color='gray', alpha=0.5, linestyle='--')
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Acceleration (m/s²)")
    axs[0].set_title("Vehicle Acceleration Over Time")
    axs[0].grid(True)
    cbar1 = fig.colorbar(sc1, ax=axs[0])
    cbar1.set_label("Safety Factor (1=safe, 0=unsafe)")

    # Plot heading rate with scatter coloring
    sc2 = axs[1].scatter(times, heading_rates, c=hr_safety, cmap=cmap, vmin=0, vmax=1, marker='x')
    axs[1].plot(times, heading_rates, color='gray', alpha=0.5, linestyle='--')
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Heading Rate (rad/s)")
    axs[1].set_title("Vehicle Heading Rate Over Time")
    axs[1].grid(True)
    cbar2 = fig.colorbar(sc2, ax=axs[1])
    cbar2.set_label("Safety Factor (1=safe, 0=unsafe)")

    plt.tight_layout()
    plt.show()

if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_file>.json")
        sys.exit(1)

    filename = sys.argv[1]
    times, accelerations, heading_rates = parse_log(filename)
    plot_metrics(times, accelerations, heading_rates)

    print("Max (abs) acceleration:", np.max(np.abs(accelerations)))
    print("Max (abs) heading rate:", np.max(np.abs(heading_rates)))