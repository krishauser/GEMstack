import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def trajectory_generation(init_state, final_state, w_c=1.0, w_eps=1.0):
    N = 50
    T = 0.1
    v = 10.0
    Lr = 1.5
    small_constant = 1e-3

    def optimize_with_weights(dynamic_weights_c=None, dynamic_weights_eps=None):
        y = np.linspace(init_state['y'], final_state['y'], N)
        psi = np.linspace(init_state['psi'], final_state['psi'], N)
        c = np.linspace(init_state['c'], final_state['c'], N)
        eps = np.zeros(N - 1)

        def cost(x):
            y_, psi_, c_, eps_ = np.split(x, [N-1, 2*(N-1), 3*(N-1)])
            c_seq = np.concatenate(([init_state['c']], c_))
            cost_c = np.sum((dynamic_weights_c if dynamic_weights_c is not None else w_c) * c_seq**2)
            cost_eps = np.sum((dynamic_weights_eps if dynamic_weights_eps is not None else w_eps) * eps_**2)
            return cost_c + cost_eps

        def dynamics_constraints(x):
            y_, psi_, c_, eps_ = np.split(x, [N-1, 2*(N-1), 3*(N-1)])
            constraints = []
            y_prev, psi_prev, c_prev = init_state['y'], init_state['psi'], init_state['c']
            for k in range(N-1):
                dy = v * np.sin(psi_prev + c_prev * Lr) * T
                dpsi = v * c_prev * T
                dc = eps_[k] * T
                constraints.append(y_[k] - (y_prev + dy))
                constraints.append(psi_[k] - (psi_prev + dpsi))
                constraints.append(c_[k] - (c_prev + dc))
                y_prev, psi_prev, c_prev = y_[k], psi_[k], c_[k]
            constraints.append(y_[-1] - final_state['y'])
            constraints.append(psi_[-1] - final_state['psi'])
            constraints.append(c_[-1] - final_state['c'])
            return constraints

        x0 = np.concatenate([y[1:], psi[1:], c[1:], eps])
        result = minimize(cost, x0, constraints={'type': 'eq', 'fun': dynamics_constraints}, options={'maxiter': 1000})

        if result.success:
            y_, psi_, c_, eps_ = np.split(result.x, [N-1, 2*(N-1), 3*(N-1)])
            y_full = np.concatenate(([init_state['y']], y_))
            psi_full = np.concatenate(([init_state['psi']], psi_))
            c_full = np.concatenate(([init_state['c']], c_))
            return y_full, psi_full, c_full, eps_
        else:
            raise RuntimeError("Optimization failed")

    # First pass
    y1, psi1, c1, eps1 = optimize_with_weights()

    # Shape weights based on result
    dynamic_weights_c = np.abs(c1) + small_constant
    dynamic_weights_eps = np.abs(eps1) + small_constant

    # Second pass with weighted cost
    y2, psi2, c2, eps2 = optimize_with_weights(dynamic_weights_c, dynamic_weights_eps)
    return y2, psi2, c2, eps2

def plot_trajectory(y_traj, psi_traj, c_traj, label):
    N = len(y_traj)
    T = 0.1
    v = 5.0
    x_traj = np.zeros(N)
    for i in range(1, N):
        dx = v * np.cos(psi_traj[i-1] + c_traj[i-1] * 1.5) * T
        x_traj[i] = x_traj[i-1] + dx

    plt.figure(figsize=(8, 5))
    plt.plot(x_traj, y_traj, label=label)
    plt.scatter([x_traj[0], x_traj[-1]], [y_traj[0], y_traj[-1]], color='red', zorder=5)
    plt.annotate("init", (x_traj[0], y_traj[0]), textcoords="offset points", xytext=(-10,10), ha='center')
    plt.annotate("final", (x_traj[-1], y_traj[-1]), textcoords="offset points", xytext=(10,10), ha='center')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Trajectory from Initial to Final State')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def plot_dynamics(psi_traj, c_traj, eps_traj):
    time_steps = np.arange(len(psi_traj))
    eps_time = np.arange(len(eps_traj))

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time_steps, psi_traj, label="Heading (ψ)")
    plt.ylabel("ψ (rad)")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_steps, c_traj, label="Curvature (c)", color='orange')
    plt.ylabel("Curvature (1/m)")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(eps_time, eps_traj, label="Curvature Rate (ε)", color='green')
    plt.xlabel("Time Step")
    plt.ylabel("ε (1/m²)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

# --- Test Case ---
def test_trajectory_generation():
    init_state = {'y': 0.0, 'psi': 0.0, 'c': 0.0}
    final_state = {'y': 10.0, 'psi': 0.0, 'c': 0.0}

    y_traj, psi_traj, c_traj, eps_traj = trajectory_generation(init_state, final_state)
    plot_trajectory(y_traj, psi_traj, c_traj, label="Generated trajectory")
    plot_dynamics(psi_traj, c_traj, eps_traj)




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
