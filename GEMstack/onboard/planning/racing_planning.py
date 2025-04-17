import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def trajectory_generation(init_state, final_state, N=30, T=0.1, Lr=1.5,
                                              w_c=10.0, w_eps=0.0, w_vvar=5.0,
                                              w_terminal=10.0,
                                              v_min=3.0, v_max=11.0,
                                              waypoint=None, waypoint_tolerance=0.5):
    def cost(p):
        x_, y_, psi_, c_, v_, eps_ = np.split(p, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
        c_seq = np.concatenate(([init_state['c']], c_))
        v_seq = np.concatenate(([init_state['v']], v_))
        x_final, y_final, psi_final, c_final = x_[-1], y_[-1], psi_[-1], c_[-1]

        cost_c = w_c * np.sum(c_seq ** 2)
        cost_eps = w_eps * np.sum(eps_ ** 2)
        v_mean = np.mean(v_seq)
        cost_vvar = w_vvar * np.mean((v_seq - v_mean) ** 2)

        cost_terminal = w_terminal * (
            (x_final - final_state['x']) ** 2 +
            (y_final - final_state['y']) ** 2 +
            (psi_final - final_state['psi']) ** 2 * 100 +
            (c_final - final_state['c']) ** 2 * 100
        )

        cost_waypoint = 0.0
        if waypoint is not None:
            waypoint_penalty_weight = 100.0
            # use midpoint index to check passing near the waypoint
            mid = N // 2
            cost_waypoint = waypoint_penalty_weight * (
                (x_[mid-1] - waypoint[0])**2 + (y_[mid-1] - waypoint[1])**2
            )

        return cost_c + cost_eps + cost_vvar + cost_terminal + cost_waypoint

    def dynamics_constraints(p):
        x_, y_, psi_, c_, v_, eps_ = np.split(p, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
        constraints = []
        x_prev, y_prev, psi_prev, c_prev, v_prev = init_state['x'], init_state['y'], init_state['psi'], init_state['c'], init_state['v']
        for k in range(N - 1):
            dx = v_prev * np.cos(psi_prev + c_prev * Lr) * T
            dy = v_prev * np.sin(psi_prev + c_prev * Lr) * T
            dpsi = v_prev * c_prev * T
            dc = eps_[k] * T
            constraints.extend([
                x_[k] - (x_prev + dx),
                y_[k] - (y_prev + dy),
                psi_[k] - (psi_prev + dpsi),
                c_[k] - (c_prev + dc)
            ])
            x_prev, y_prev, psi_prev, c_prev, v_prev = x_[k], y_[k], psi_[k], c_[k], v_[k]
        return constraints

    # Initial guesses
    x_vals = np.linspace(init_state['x'], final_state['x'], N)
    y_vals = np.linspace(init_state['y'], final_state['y'], N)
    psi_vals = np.linspace(init_state['psi'], final_state['psi'], N)
    c_vals = np.linspace(init_state['c'], final_state['c'], N)
    v_vals = np.ones(N) * init_state['v']
    eps_vals = np.zeros(N - 1)

    p0 = np.concatenate([x_vals[1:], y_vals[1:], psi_vals[1:], c_vals[1:], v_vals[1:], eps_vals])
    bounds = [(None, None)] * (4 * (N - 1)) + [(v_min, v_max)] * (N - 1) + [(None, None)] * (N - 1)

    result = minimize(cost, p0, bounds=bounds,
                      constraints={'type': 'eq', 'fun': dynamics_constraints},
                      options={'maxiter': 1000})
    if not result.success:
        raise RuntimeError("Optimization failed")

    x_, y_, psi_, c_, v_, eps_ = np.split(result.x, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
    x_full = np.concatenate(([init_state['x']], x_))
    y_full = np.concatenate(([init_state['y']], y_))
    psi_full = np.concatenate(([init_state['psi']], psi_))
    c_full = np.concatenate(([init_state['c']], c_))
    v_full = np.concatenate(([init_state['v']], v_))

    final_error = {
        'x_error': abs(x_full[-1] - final_state['x']),
        'y_error': abs(y_full[-1] - final_state['y']),
        'psi_error': abs(psi_full[-1] - final_state['psi']),
        'c_error': abs(c_full[-1] - final_state['c']),
    }

    return x_full, y_full, psi_full, c_full, v_full, eps_, final_error

def plot_trajectory(x, y, v, c, eps, waypoint=None):
    plt.figure(figsize=(12, 10))

    # Trajectory plot
    plt.subplot(4, 1, 1)
    plt.plot(x, y, label="Trajectory")
    plt.scatter([x[0], x[-1]], [y[0], y[-1]], color='red', label="Start/End")
    
    if waypoint is not None:
        plt.scatter(*waypoint, color='purple', s=60, marker='X', label="Waypoint")
        plt.annotate("Waypoint", (waypoint[0], waypoint[1]), textcoords="offset points", xytext=(5,5), color='purple')

    plt.axis('equal')
    plt.ylabel("y (m)")
    plt.title("Trajectory")
    plt.grid(True)
    plt.legend()

    # Speed plot
    plt.subplot(4, 1, 2)
    plt.plot(v, label="Speed (v)", color="blue")
    plt.ylabel("Speed (m/s)")
    plt.grid(True)
    plt.legend()

    # Curvature plot
    plt.subplot(4, 1, 3)
    plt.plot(c, label="Curvature (c)", color="orange")
    plt.ylabel("Curvature (1/m)")
    plt.grid(True)
    plt.legend()

    # Curvature rate plot
    plt.subplot(4, 1, 4)
    plt.plot(eps, label="Curvature Rate (ε)", color="green")
    plt.xlabel("Step")
    plt.ylabel("ε (1/m²)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

###### Test case 1: pass a cone in slalom
def trajectory_generation_test1():
    # Init and final
    init_state = {'x': 0.0, 'y': 0.0, 'psi': 0.0, 'c': 0.0, 'v': 5.0}
    final_state = {'x': 15.0, 'y': 0.0, 'psi': np.pi / 20000000, 'c': np.pi / 20000000}
    waypoint = (8.0, 6.0)

    # Solve
    x, y, psi, c, v, eps, final_error = trajectory_generation(
        init_state, final_state, waypoint=waypoint
    )
    plot_trajectory(x, y, v, c, eps, waypoint)

    # Error
    print("\nFinal State Errors:")
    for k, e in final_error.items():
        print(f"{k}: {e:.6f}")

###### Test case 2: 90 degree turn
def trajectory_generation_test2():
    # Init and final
    init_state = {'x': 0.0, 'y': 0.0, 'psi': 0.0, 'c': 0.0, 'v': 5.0}
    final_state = {'x': 15.0, 'y': 15.0, 'psi': np.pi / 2, 'c': np.pi / 2}
    waypoint = (13.0, 3.0)

    # Solve
    x, y, psi, c, v, eps, final_error = trajectory_generation(
        init_state, final_state, waypoint=waypoint
    )
    plot_trajectory(x, y, v, c, eps, waypoint)

    # Error
    print("\nFinal State Errors:")
    for k, e in final_error.items():
        print(f"{k}: {e:.6f}")



def feasibility_check(trajectory, cone_map, car_width=2.0, safety_margin=0.3, v=10.0, Lr=1.5, T=0.1):
    """
    Check if the car trajectory collides with any cones.

    Parameters:
    - trajectory: list of (y, psi, c) states
    - cone_map: list of (x, y) cone positions
    - car_width: width of the vehicle in meters
    - safety_margin: buffer around the vehicle
    - v: vehicle constant speed (used for x position estimation)
    - Lr: distance to rear axle
    - T: time step

    Returns:
    - feasible: True if no collisions
    - collisions: list of indices of cones that were collided with (for plotting purpose)
    - x_vals, y_vals: trajectory positions for plotting (for plotting purpose)
    """
    y_vals, psi_vals, c_vals = zip(*trajectory)
    x_vals = [0.0]
    for i in range(1, len(trajectory)):
        dx = v * np.cos(psi_vals[i-1] + c_vals[i-1] * Lr) * T
        x_vals.append(x_vals[-1] + dx)

    collision_radius = (car_width / 2.0) + safety_margin
    collisions = []

    for j, (cone_x, cone_y) in enumerate(cone_map):
        for x, y in zip(x_vals, y_vals):
            if np.hypot(x - cone_x, y - cone_y) < collision_radius:
                collisions.append(j)
                break

    feasible = len(collisions) == 0
    return feasible, collisions, x_vals, y_vals

# --- Test Case ---
def test_feasibility_check():
    N = 50
    y_traj = np.linspace(0, 10, N)
    psi_traj = np.linspace(0, 0.1, N)
    c_traj = np.linspace(0, 0.2, N)
    trajectory = list(zip(y_traj, psi_traj, c_traj))

    # Cone map near the path
    cone_map = [(5.0, 1.0), (10.0, 1.5), (15.0, 2.0), (25.0, 4.0), (25.0, 10.0), (16.0, 9.0), (40.0, 5.0)]

    # Run check
    feasible, collisions, x_vals, y_vals = feasibility_check(trajectory, cone_map)

    # Plot
    plt.figure(figsize=(10, 6))
    plt.plot(x_vals, y_vals, label="Trajectory", linewidth=2)
    for i, (cx, cy) in enumerate(cone_map):
        color = 'red' if i in collisions else 'green'
        plt.scatter(cx, cy, color=color, s=100, label=f'Cone {i}' if i == 0 else "")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory and Cone Map")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    print("Feasible:", feasible)
    print("Collisions with cones:", collisions)
