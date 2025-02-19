import json
import sys
import matplotlib.pyplot as plt

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
    return times, accelerations, heading_rates

def plot_metrics(times, accelerations, heading_rates):
    """Creates two subplots:
        - Instantaneous acceleration vs. time
        - Heading rate vs. time
    """
    plt.figure(figsize=(12, 8))
    # Plot acceleration
    plt.subplot(2, 1, 1)
    plt.plot(times, accelerations, 'b-', marker='o', label="Acceleration (m/s²)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.title("Vehicle Acceleration Over Time")
    plt.legend()
    plt.grid(True)

    # Plot heading rate
    plt.subplot(2, 1, 2)
    plt.plot(times, heading_rates, 'r-', marker='x', label="Heading Rate (rad/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Heading Rate (rad/s)")
    plt.title("Vehicle Heading Rate Over Time")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_file>.json")
        sys.exit(1)

    filename = sys.argv[1]
    times, accelerations, heading_rates = parse_log(filename)
    plot_metrics(times, accelerations, heading_rates)