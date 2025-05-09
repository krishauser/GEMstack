import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy.interpolate import splprep, splev
from ...knowledge.vehicle.geometry import steer2front

from ...knowledge.vehicle import dynamics
from ...knowledge.vehicle import geometry
from ...utils import settings
import pandas as pd

def parse_route_csv(filename):
    """
    Parses the route file and  extracts:
      - X position desired (from column index 0)
      - Y position desired (from column index 1)
    """

    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    x_desired = data[:, 0]
    y_desired = data[:, 1]
    return x_desired, y_desired

def compute_spline_curvature(x, y, s=0.0, num=1000):
    """
    Computes curvature from 2D path (x, y) using parametric splines.
    
    Parameters:
        x, y (np.array): path coordinates
        s (float): Smoothing factor for spline (0 = interpolating spline)
        num (int): Number of points to evaluate spline and curvature at
            
    Returns:
        kappa (float): curvature profile along the trajectory
    """
    # Fit parametric spline
    num = len(x)
    tck, _ = splprep([x, y], s=s)
    u_fine = np.linspace(0, 1, num)
    
    # Evaluate spline and its first and second derivatives
    dx, dy = splev(u_fine, tck, der=1)
    ddx, ddy = splev(u_fine, tck, der=2)

    # Compute curvature: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    denom = (dx**2 + dy**2)**1.5 + 1e-8  # Avoid divide by zero
    kappa = (dx * ddy - dy * ddx) / denom

    return kappa

def lateral_speed_limit(kappa, ay_max, v_max):
    """
    Limits the velocity based on curvature and lateral acceleration limits.
    
    Parameters:
        kappa (np.ndarray): curvature profile along the trajectory
        ay_max (float): maximum lateral acceleration
        v_max (float): maximum velocity

    Returns:
        np.minimum(v_lat, v_max): velocity profile limited by lateral acceleration
    """
    v_lat = np.sqrt(np.maximum(ay_max / (np.abs(kappa) + 1e-8), 0))
    return np.minimum(v_lat, v_max)

def limit_ax_for_friction_ellipse(v, kappa, ax_limit, ay_max):
    """
    Longitudinal acceleration limit based on current lateral acceleration.
    
    Parameters:
        v (float): velocity
        kappa (float): curvature
        ax_limit (float): maximum longitudinal acceleration
        ay_limit (float): maximum lateral acceleration

    Returns:
        ax_limit * ax_ratio: maximum allowed longitudinal acceleration
    """
    # Compute a_y at current v
    ay = v**2 * kappa
    ay_ratio = ay / ay_max
    ay_ratio = np.clip(ay_ratio, -1.0, 1.0)
    # Remaining fraction for ax
    ax_ratio = np.sqrt(np.maximum(1.0 - ay_ratio**2, 0.0))
    return ax_limit * ax_ratio

def apply_trail_braking(ds, v_lat, kappa, ax_max, ax_min, ay_max):
    """
    Limits the velocity based on lateral and longitudinal acceleration limits.
    
    Parameters:
        ds (np.ndarray): distance diff along the trajectory.
        v_lat (np.ndarray): velocity profile along the trajectory.
        kappa (float): curvature profile along the trajectory
        ax_max (float): maximum longitudinal acceleration
        ax_min (float): minimum longitudinal acceleration (braking)
        ay_max (float): maximum lateral acceleration

    Returns:
        v_profile (np.ndarray): acceleration limited velocity profile.
        forward_vs (np.ndarray): positive acceleration limited velocity profile
        forward_vs, (np.ndarray):  negative acceleration limited velocity  profile

    """
    N = len(ds)
    v_profile = v_lat.copy()
    v_profile[0] = 0.0
    forward_vs = v_lat.copy()
    backward_vs = v_lat.copy()

    # Forward (accelerating out of curves)
    for i in range(1, N):
        ax_limit = limit_ax_for_friction_ellipse(v_profile[i-1], kappa[i-1], ax_max, ay_max)
        ax_limit = np.clip(ax_limit, ax_min, ax_max)
        v_allowed = np.sqrt(v_profile[i-1]**2 + 2 * ax_limit * ds[i-1])
        v_profile[i] = min(v_profile[i], v_allowed)
        forward_vs[i] = v_allowed

    # Backward (trail braking into corners)
    for i in range(N - 1, 0, -1):
        ax_limit = limit_ax_for_friction_ellipse(v_profile[i], kappa[i], abs(ax_min), ay_max)
        arg = v_profile[i]**2 + 2 * ax_limit * ds[i]
        v_allowed = np.sqrt(max(arg,0.0))
        v_profile[i-1] = min(v_profile[i-1], v_allowed)
        
        backward_vs[i-1] = v_allowed
        
    return v_profile, forward_vs, backward_vs

def limit_velocity_by_steering_rate(ds, velocity, wheelbase, max_steering_rate, kappa):
    """
    Limits the velocity based on steering angle rate of change.
    
    Parameters:
        ds (np.ndarray): distance diff along the trajectory.
        velocity (np.ndarray): velocity profile along the trajectory.
        wheelbase (float): vehicle's wheelbase (L).
        max_steering_rate (float): maximum allowable steering angle rate (rad/s).
        kappa (float): curvature profile along the trajectory

    Returns:
        limited_velocity (np.ndarray): steering rate limited velocity profile.
    """
    # Compute steering angle delta: delta = arctan(L * kappa)
    steering_angle = np.arctan(wheelbase * kappa)

    # Compute steering rate: d(delta)/dt
    ddelta = np.gradient(steering_angle)
    dt = ds / (velocity + 1e-6)  # Add small value to prevent division by zero
    ddelta_dt = ddelta / (dt + 1e-6)

    # max steering wheel rate to steering rate
    max_steering_rate = steer2front(max_steering_rate)

    # Limit velocity where steering rate exceeds max
    limited_velocity = np.copy(velocity)
    for i in range(len(velocity)):
        if abs(ddelta_dt[i]) > max_steering_rate:
            ddelta_dt[i] = max_steering_rate
            limited_velocity[i] = max(0, min(
                velocity[i],
                abs((ds[i] / (ddelta[i] / ddelta_dt[i])+ 1e-6) - 1e-6)
            ))
    return limited_velocity

def compute_time_profile(ds, v_profile):
    """
    Compute updated trajectory times based on a path and velocity profile
    
    Parameters:
        ds (np.ndarray): distance diff along the trajectory.
        v_profile (np.ndarray): velocity profile along the trajectory.
        
    Returns:
        t (np.ndarray): time profile for trajectory.
    """

    v_profile = np.array(v_profile)

    # Use average speed between points to estimate time between them
    v_mid = (v_profile[:-1] + v_profile[1:]) / 2
    v_mid = np.clip(v_mid, 0.1, None)  # Avoid divide-by-zero

    dt = ds[:-1] / v_mid
    t = np.insert(np.cumsum(dt), 0, 0.0)  # cumulative time starting from 0

    return t

def plot_speed_profile_gradient(fig, axis, xs, ys, velocity, cmap='jet'):
    """
    Plotting color gradient velocity profile on the path
    
    Parameters:
        fig, axis: figure and axis on which to plot
        xs, ys (np.ndarray): path coordinates
        velocity (np.ndarray): velocity profile
        cmap: color map
    """
    xs = np.array(xs)
    ys = np.array(ys)
    velocity = np.array(velocity)

    # Build segments between points
    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Create line collection
    lc = LineCollection(segments, cmap=cmap, linewidth=2)
    lc.set_array(velocity[:-1])  # color values
    lc.set_linewidth(2)

    # fig, ax = plt.subplots(figsize=(8, 6))
    line = axis.add_collection(lc)
    fig.colorbar(line, ax=axis, label='Speed (m/s)')
    axis.set_xlim(xs.min(), xs.max())
    axis.set_ylim(ys.min(), ys.max())
    axis.set_aspect('equal', adjustable='box')
    axis.set_title('Speed Profile Along Path')
    axis.set_xlabel('X [m]')
    axis.set_ylabel('Y [m]')
    plt.grid(True)

def plot_x_y(axis, x, y, x_label, y_label):
    """
    Plotting x vs y variables
    
    Parameters:
        axis: axis on which to plot
        x, y (np.ndarray): variables to plot
        x_label, y_label (str): axis labels
    """
    axis.plot(x, y)
    axis.set_xlabel(x_label)
    axis.set_ylabel(y_label)
    axis.grid(True)

def compute_velocity_profile(points, plot=True):
    """
    Compute a vehicle dynamics limited velocity profile for a set path
    
    Parameters:
        points: path x,y coordinates
        plot (bool): visualization plots
        
    Returns:
        t (np.ndarray): time profile for trajectory.
        v_profile (np.ndarray): velocity profile for trajectory.
    """
    # Vehicle dynamics limits
    v_max  = settings.get('vehicle.limits.max_speed')
    ax_max = settings.get('vehicle.limits.max_longitudinal_acceleration')
    ax_min = settings.get('vehicle.limits.min_longitudinal_acceleration')
    ay_max = settings.get('vehicle.limits.max_lateral_acceleration')
    max_steering_rate = settings.get('vehicle.limits.max_steering_rate')
    wheelbase = settings.get('vehicle.geometry.wheelbase')

    points = np.array(points)
    xs, ys = points[:,0], points [:,1]
    dx = np.diff(xs)
    dy = np.diff(ys)
    ds = np.hypot(dx, dy)
    ds = np.append(ds, ds[-1])

    # curvature profile of the path
    kappa = compute_spline_curvature(xs, ys)

    # max speeds from lateral acceleration limits
    v_lat = lateral_speed_limit(kappa, ay_max, v_max)

    # max speeds from steering limits
    v_steer = limit_velocity_by_steering_rate(ds, v_lat, wheelbase, max_steering_rate, kappa)   

    #max speeds from longitudinal acceleration limits
    v_profile, forward_vs, backward_vs = apply_trail_braking(ds, v_steer, kappa, ax_max, ax_min, ay_max)

    t = compute_time_profile(ds, v_profile)

    if plot is False:
        return t, v_profile
    else:
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))

        dv = np.diff(v_profile)
        dt = np.diff(t)

        a = dv/dt

        plot_x_y(axs[0, 0], t, v_profile, "t", "v")
        plot_x_y(axs[1, 0], t[:-1], a, "t", "a")
        # plot_x_y(axs[1, 1], t, forward_vs, "t", "forward & backward vs")
        # plot_x_y(axs[1, 1], t, backward_vs, "t", "forward & backward vs")
        plot_x_y(axs[0, 1], t, kappa, "t", "kappa")

        plot_speed_profile_gradient(fig, axs[1, 1], xs, ys, v_profile)

        # # Save to CSV
        # df = pd.DataFrame({'x': xs, 'y': ys, 'v': v_profile, 't': t})
        # df.to_csv('v_profile.csv', index=False)


        plt.tight_layout()
        plt.show()

        return t, v_profile


if __name__=='__main__':
    ### offline velocity profile plotting from a route
    if len(sys.argv) != 2:
        print("Usage: python velocity_profile.py <route file name>")
        sys.exit(1)
    
    path_file = sys.argv[1]
    
    # if behavior.json doesn't exist, print error and exit
    if not os.path.exists(path_file):
        print("Error: route file not found.")
        sys.exit(1)
    
    xs, ys = parse_route_csv(path_file)

    points = list(zip(xs,ys))

    t, v_profile = compute_velocity_profile(points, plot=True)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))

    dv = np.diff(v_profile)
    dt = np.diff(t)

    a = dv/dt

    plot_x_y(axs[0, 0], t, v_profile, "t", "v")
    # plot_x_y(axs[0, 1], t[:-1], a, "t", "a")
    # plot_x_y(axs[1, 0], xs, ys, "x", "y")
    # plot_x_y(axs[1, 1], t, forward_vs, "t", "forward & backward vs")
    # plot_x_y(axs[1, 1], t, backward_vs, "t", "forward & backward vs")
    # plot_x_y(axs[1, 1], t, xs, "t", "x & y")
    # plot_x_y(axs[1, 1], t, ys, "t", "x & y")

    plot_speed_profile_gradient(fig, axs[1, 0], xs, ys, v_profile)
    plt.tight_layout()
    plt.show()