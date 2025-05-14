import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.getcwd())

import json
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# from GEMstack.mathutils.transforms import lat_lon_to_xy

from pyproj import Proj, Transformer
from scipy.spatial import ConvexHull

def latlon_to_xy(latitudes, longitudes, origin_lat, origin_lon):
    """
    Converts lat/lon to local X/Y coordinates in meters using a local ENU projection.

    latitudes, longitudes: arrays of GPS coordinates
    origin_lat, origin_lon: reference point (usually the first point)

    Returns: xs, ys (in meters, relative to origin)
    """
    # WGS84 to ENU local projection (east-north-up)
    transformer = Transformer.from_crs(
        "epsg:4326",  # WGS84 lat/lon
        f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
        always_xy=True
    )

    xs, ys = transformer.transform(longitudes, latitudes)
    return xs, ys


def parse_behavior_log(filename):
    """
    Parses the behavior log file and extracts the following data:
    - vehicle time
    - vehicle acceleration
    - vehicle heading rate
    - speed
    """
    times = []
    accelerations = []
    heading_rates = []
    speeds = []
    xs = []
    ys = []
    ref_lat = 0.0
    ref_long = 0.0
    pedals = []
    pedal_times = []
    
    with open(filename, 'r') as f:
        for idx, line in enumerate(f):
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                print(f"Skipping invalid JSON line: {line.strip()}")
                continue
            # Process vehicle state data
            if "vehicle" in entry:
                t = entry.get("time")
                vehicle_data = entry["vehicle"].get("data", {})
                acceleration = vehicle_data.get("acceleration")
                heading_rate = vehicle_data.get("heading_rate")
                speed = vehicle_data.get("v")
                # print(speed)
                # pose_data = vehicle_data["pose"].get("pose")
                frame = vehicle_data["pose"].get("frame")
                x = vehicle_data["pose"].get("x")
                y = vehicle_data["pose"].get("y")
                # if idx == 0:
                #     ref_lat = y
                #     ref_long = x
                # x, y = latlon_to_xy(y,x,ref_lat,ref_long)
                # Only add if all fields are available
                if None not in (t, acceleration, heading_rate, speed)  and frame != 3:
                    times.append(t)
                    accelerations.append(acceleration)
                    heading_rates.append(heading_rate)
                    speeds.append(speed)
                    
                    xs.append(x)
    
                    ys.append(y)
            if "vehicle_interface_command" in entry:
                commmand_data = entry["vehicle_interface_command"].get("data", {})
                pedal = commmand_data.get("brake_pedal_position")
                t = entry.get("time")
                if pedal is not None:
                    pedals.append(pedal)
                    pedal_times.append(t)
                   
        # print(len(speeds))
        # print(len(times))
    return np.array(times), np.array(xs), np.array(ys), np.array(speeds), np.array(accelerations), np.array(heading_rates), np.array(pedals), np.array(pedal_times)

def parse_tracker_csv(filename):
    """
    Parses the pure pursuit tracker log file and extracts the following data:
      - vehicle time (from column index 19)
      - X position actual (from column index 2)
      - Y position actual (from column index 5)
      - X position desired (from column index 11)
      - Y position desired (from column index 14)
    """

    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    vehicle_time = data[:, 19]
    cte = data[:, 20]
    x_actual, y_actual = data[:, 2], data[:, 5]
    # x_desired, y_desired = data[:, 11], data[:, 14]  ### PP
    x_desired, y_desired = data[:, 8], data[:, 11] #Stanley
    speed_actual = data[:, -1]
    des_speed = data[:, -13]
    return vehicle_time, cte, x_actual, y_actual, x_desired, y_desired, speed_actual, des_speed

def compute_derivative(times, values):
    """
    Computes the derivative of array with respect to time.
    Returns the time array and derivative values.
    """
    dt = np.diff(times)
    dv = np.diff(values)
    derivative = dv / dt
    return times[1:], derivative

def moving_average(data, window_size=5):
    """
    Simple moving average smoother for 1D array.
    Pads edges by repeating border values.
    """
    pad = window_size // 2
    padded = np.pad(data, (pad, pad), mode='edge')
    smoothed = np.convolve(padded, np.ones(window_size) / window_size, mode='valid')
    return smoothed

def remove_duplicate_positions(times, xs, ys):
    """
    Removes consecutive duplicate (x, y) points.
    Keeps only unique steps in position.
    """
    xs = np.array(xs)
    ys = np.array(ys)
    times = np.array(times)

    # Compute deltas
    dx = np.diff(xs)
    dy = np.diff(ys)

    # Find where movement occurs
    moved = (dx != 0) | (dy != 0)

    # Always keep the first point
    keep_idx = np.insert(moved, 0, True)

    return times[keep_idx], xs[keep_idx], ys[keep_idx]

def compute_gs(times, xs, ys, speeds, accelerations, heading_rates):
    # vxs = np.gradient(xs, times)
    # vys  = np.gradient(ys, times)
    times = times - times[0]
    dx = np.diff(xs)
    dy = np.diff(ys)
    dt = np.diff(times)

    vxs = dx/dt
    vys = dy/dt
    vs = np.sqrt(vxs**2 + vys**2)

    vs = moving_average(vs, window_size=7)

    valid_mask = vs > 1e-6  # small threshold to avoid float noise
    vs = vs[valid_mask]
    times = times[:-1]
    times = times[valid_mask]

    long_accels = np.gradient(vs, times)

    accel_valid_mask = np.abs(long_accels) <  5 

    psis = np.arctan2(vys, vxs)
    psis = psis[valid_mask]
    psi_mask = psis > 1e-6
    psis = psis[psi_mask]
    times = times[psi_mask]
    vs = vs[psi_mask]
    long_accels = long_accels[psi_mask]

    yaw_rates = np.gradient(psis, times)
    
    lat_accels = yaw_rates * vs
    g = 9.81
    longitudinal_gs = long_accels / g
    lateral_gs = lat_accels / g

    print("max long accel: " + str(np.max(long_accels)))
    print("min long accel: " + str(np.min(long_accels)))
    print("max lat accel: " + str(np.max(lat_accels)))


    return longitudinal_gs, lateral_gs, vs, lat_accels, long_accels, times, valid_mask

def plot_pedals(axis, pedal, acceleration):
    """Plots vehicle actual and desired positions vs. time"""

    axis.plot(pedal, acceleration)
    axis.set_xlabel("Pedal position")
    axis.set_ylabel("Acceleration")
    axis.grid(True)

def brake_model(p, d, b_max, n):
    return -d - b_max * p**n

def brake_pedal_from_decel(a, d, b_max, n):
    """
    Compute brake pedal position p from desired deceleration a,
    using fitted model parameters.
    
    Returns 0 if a is not strong enough to require braking (a > -d).
    """
    a = np.asarray(a)
    p = np.zeros_like(a)

    valid = a < -d  # Only apply braking if decel > dry decel

    p[valid] = ((- (a[valid] + d)) / b_max)**(1 / n)

    # Clamp between 0 and 1
    return np.clip(p, 0.0, 1.0)


if __name__=='__main__':
    if len(sys.argv) != 2:
        print("Usage: python test_comfort_metrics.py <log_directory>")
        sys.exit(1)
    
    log_dir = sys.argv[1]
    behavior_file = os.path.join(log_dir, "behavior.json")
    tracker_file = os.path.join(log_dir, "StanleyTrajectoryTracker_debug.csv")
    # tracker_file = os.path.join(log_dir, "PurePursuitTrajectoryTracker_debug.csv")
    
    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(behavior_file):
        print("Error: behavior.json file is missing in log folder.")
        sys.exit(1)
    
    # Parse behavior log file and compute metrics
    times, xs, ys, speeds, accelerations, heading_rates, pedals, pedal_times = parse_behavior_log(behavior_file)
    
    mask = np.abs(accelerations) > 0.1
    filtered_times = times[mask]
    filtered_accels = accelerations[mask]
    mask2 = filtered_accels < 0
    filtered_times = filtered_times[mask2]
    filtered_accels = filtered_accels[mask2]

    matching_pedals = np.empty_like(filtered_accels)
    for i, t in enumerate(filtered_times):
        idx = np.argmin(np.abs(pedal_times - t))
        matching_pedals[i] = pedals[idx]

    pedal_mask = np.abs(matching_pedals) > 0.05
    matching_pedals = matching_pedals[pedal_mask]
    filtered_accels = filtered_accels[pedal_mask]


    # Use your matched data
    # brake_positions: x data
    # decelerations: y data (should be negative)

    p_data = matching_pedals  # brake pedal positions
    a_data = filtered_accels  # measured decelerations (should be negative)

    # Initial guesses: dry decel=0.1, max brake=4.0, exponent=2.0
    popt, _ = curve_fit(brake_model, p_data, a_data, p0=(0.1, 4.0, 2.0))

    d_est, b_max_est, n_est = popt
    print(f"Estimated parameters:\n  dry decel = {d_est:.3f}\n  max brake = {b_max_est:.3f}\n  exponent = {n_est:.3f}")

    p_fit = np.linspace(0, 1, 100)
    a_fit = brake_model(p_fit, *popt)

    plt.figure(figsize=(8, 5))
    plt.scatter(p_data, a_data, label='Measured Data', color='blue')
    plt.plot(p_data, a_data, label='Measured Data line', color='blue')
    plt.plot(p_fit, a_fit, label='Fitted Model', color='red')
    plt.xlabel('Brake Pedal Position')
    plt.ylabel('Deceleration (m/s²)')
    plt.title('Brake Pedal vs Deceleration Fit')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


    # fig, axs = plt.subplots(1, 2, figsize=(12, 8))
    # plot_pedals(axs[1], matching_pedals, filtered_accels) 
 
    # plt.tight_layout()
    # name = "pedal_to_accel.png"
    # file_path = os.path.join(log_dir, name)

    # plt.savefig(file_path, dpi=300, format='png')
    # print(f"Plots saved as {name}")

    # plt.show()

