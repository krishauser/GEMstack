# from typing import List, Tuple, Union
# from ..component import Component
# from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState
# from ...utils import serialization, settings
# from ...mathutils.transforms import vector_madd
# from ...mathutils.quadratic_equation import quad_root

from ...state.trajectory import Trajectory, compute_headings
from ...utils import settings

# ===== Additional Imports =====
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
# ==============================

#TODO: Combine with current planner and GemStack
# def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float,
#                       method : str) -> Trajectory:
    
#     return Trajectory(frame=path.frame, points=dense_points, times=times)

# --------------------------------------------------------------------------- Hua-Ta's Code START
def scenario_check(vehicle_state, cone_state):
    """
    Check the cone state and determine the scenario.
    Args:
        vehicle_state: The state of the vehicle.
        cone_state: The state of the cones.
    Returns:
        str: The scenario type.
        - 'left_pass': cone pointed left, go left
        - 'right_pass': cone pointed right, go right
        - 'u_turn': cone standing up, make a U-turn
        tuple: (cone_x, cone__y)
    """
    # Get the closest cone info
    cones_ahead = sorted(cone_state, key=lambda c: np.linalg.norm(
        np.array([c['x'], c['y']]) - np.array(vehicle_state['position'])))
    cone_direction = cones_ahead[0]['orientation']
    cone_position = (cones_ahead[0]['x'], cones_ahead[0]['y'])
    
    # Check the cone orientation
    if cone_direction != 'left' and cone_direction != 'right' and cone_direction != 'standing':
        raise ValueError("Unknown cone orientation")
    
    return cone_direction, cone_position
    
def waypoint_generate(vehicle_state, cone_state):
    """
    Generate waypoints based on the scenario.
    Args:
        vehicle_state: The state of the vehicle.
        cone_state: The state of the cones.
    Returns:
        Tuple[str, Tuple[float, float], Tuple[float, float]]:
            scenario: detected scenario type
            flex_wp: flexible waypoint (used to maneuver)
            fixed_wp: fixed waypoint (goal position)
    """
    scenario, cone_position = scenario_check(vehicle_state, cone_state)
    car_position = np.array(vehicle_state['position'])
    car_heading = vehicle_state['heading']  # in radians

    # ===== Parameters =====
    u_turn_radius = 10.0  # Radius for U-turn
    offset = 5.0  # Offset for left/right pass
    lookahead_distance = 10.0  # Distance ahead for fixed point
    # ======================

    # Direction vector based on heading
    heading_vector = np.array([np.cos(car_heading), np.sin(car_heading)])
    
    # Vector perpendicular to heading (to determine left/right)
    perpendicular_vector = np.array([-np.sin(car_heading), np.cos(car_heading)])

    if scenario == 'standing':
        # U-turn: Circle around the cone
        cone = np.array(cone_position)

        # Flexible waypoint: halfway between car and a point offset from cone center
        flex_wp1 = cone + u_turn_radius * perpendicular_vector
        flex_wp2 = cone + heading_vector * lookahead_distance
        #flex_wps = [flex_wp1, flex_wp2]
        flex_wps = [flex_wp2]

        # Fixed waypoint: behind the cone after U-turn
        # fixed_wp = cone - heading_vector * lookahead_distance
        fixed_wp = cone - u_turn_radius * perpendicular_vector

    elif scenario == 'left':
        cone = np.array(cone_position)

        # Flexible waypoint: go to the left of the cone
        flex_wp1 = cone + offset * perpendicular_vector
        flex_wps = [flex_wp1]

        # Fixed waypoint: forward after passing the cone
        # fixed_wp = flex_wp + heading_vector * lookahead_distance - offset * perpendicular_vector * 2
        fixed_wp = flex_wp1 + heading_vector * lookahead_distance - offset * perpendicular_vector

    elif scenario == 'right':
        cone = np.array(cone_position)

        # Flexible waypoint: go to the right of the cone
        flex_wp1 = cone - offset * perpendicular_vector
        flex_wps = [flex_wp1]

        # Fixed waypoint: forward after passing the cone
        # fixed_wp = flex_wp + heading_vector * lookahead_distance + offset * perpendicular_vector * 2
        fixed_wp = flex_wp1 + heading_vector * lookahead_distance + offset * perpendicular_vector

    else:
        flex_wps = None
        fixed_wp = None

    return scenario, flex_wps, fixed_wp
    
def velocity_profiling(path, acceleration, deceleration, max_speed, current_speed, lateral_acc_limit):
    """
    Returns a trajectory with velocity profile that respects:
    1. max longitudinal acceleration
    2. max deceleration
    3. max lateral acceleration from curvature
    4. max speed
    """
    # Curvature-based speed limit
    curvature_safe_speeds = []
    for p in path.curvature:
        if abs(p) < 1e-3:
            curvature_safe_speeds.append(max_speed)
        else:
            max_v = np.sqrt(lateral_acc_limit / abs(p))
            curvature_safe_speeds.append(min(max_v, max_speed))

    # Create a custom profile-aware speed function
    def dynamic_max_speed_at(index):
        return curvature_safe_speeds[min(index, len(curvature_safe_speeds) - 1)]

    # You can inject dynamic speed limit into your plan
    return
# --------------------------------------------------------------------------- Hua-Ta's Code END

# --------------------------------------------------------------------------- Shilan's Code START
def trajectory_generation(init_state, final_state, N=30, T=0.1, Lr=1.5,
                                              w_c=10.0, w_eps=0.0, w_vvar=5.0,
                                              w_terminal=10.0,
                                              v_min=3.0, v_max=11.0,
                                              waypoint=None, waypoint_penalty_weight=100.0):
    """
    Generate a dynamically feasible trajectory between init_state and final_state (optionally pass a (x,y) waypoint)
    using curvature-based vehicle dynamics and nonlinear optimization.

    Note: We minimize variance of velocity to roughly keep constant velocity so it does not affect the turning radius.

    Thoughts: The trajectory generated from large velocity and large curvature rate can be similar to the trajectory 
    generated using small velovity and small curvature rate, might be scaled versions if the proportions are proper.
    For racing, we want to use the maximum velocity. So maybe we should scale the trajectory using the allowed velocity??? (TODO)
    And how to obtain that??? (TODO)

    TODO: remove v in init_state

    Parameters:
    - init_state (dict): Initial vehicle state with keys 'x', 'y', 'psi', 'c', 'v'.
    - final_state (dict): Target vehicle state with keys 'x', 'y', 'psi', 'c'.
    - N (int): Number of discrete time steps in the trajectory.
    - T (float): Duration of each time step (in seconds).
    - Lr (float): Distance from the vehicle's center to the rear axle. # TODO: find this value for GEMe4
    - w_c (float): Weight for penalizing curvature (smoothness of turns).
    - w_eps (float): Weight for penalizing curvature rate (reduces sharp steering changes).
    - w_vvar (float): Weight for penalizing speed variance (encourages speed smoothness).
    - w_terminal (float): Weight for penalizing final state deviation (soft constraint).
    - v_min (float): Minimum allowed speed (in m/s).
    - v_max (float): Maximum allowed speed (in m/s).
    - waypoint (tuple or None): Optional (x, y) coordinate that the trajectory should pass near.
    - waypoint_penalty_weight (float): Penalty weight for distance from waypoint (soft constraint).

    Returns:
    - x, y, psi, c, v, eps (np.ndarray): Arrays of optimized state and control values.
    - final_error (dict): Final state errors in x, y, psi, and c.
    """
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
            (c_final - final_state['c']) ** 2 * 100 # TODO: remove magic constant
        )

        cost_waypoint = 0.0
        if waypoint is not None:
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
######################## 
if __name__ == "__main__":
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

    # --- Test Case 1: pass a cone in slalom ---
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

    # --- Test case 2: 90 degree turn ---
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

    #############
    #########################
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
# --------------------------------------------------------------------------- Shilan's Code END

# ------------ Test Code --------------
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    
    # --------------------------
    # Plotting Function
    # --------------------------
    def plot_results(vehicle_state, cones, wpt_flexes=None, wpt_fixed=None, scenario_label=""):
        plt.figure(figsize=(10, 5))
        ax = plt.gca()
        ax.set_aspect('equal')

        all_x = [cone['x'] for cone in cones] + [vehicle_state['position'][0]]
        all_y = [cone['y'] for cone in cones] + [vehicle_state['position'][1]]

        if wpt_flexes is not None:
            all_x += [pt[0] for pt in wpt_flexes]
            all_y += [pt[1] for pt in wpt_flexes]

        if wpt_fixed is not None:
            all_x.append(wpt_fixed[0])
            all_y.append(wpt_fixed[1])

        # Compute axis limits with padding
        padding = 2
        x_min, x_max = min(all_x) - padding, max(all_x) + padding
        y_min, y_max = min(all_y) - padding, max(all_y) + padding
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        # Plot cones
        for cone in cones:
            plt.scatter(cone['x'], cone['y'], c='orange', label='Cone' if cone == cones[0] else "")
            plt.text(cone['x'], cone['y'] + 0.5, cone['orientation'][0].upper(), fontsize=10, ha='center')

        # Plot vehicle
        vx, vy = vehicle_state['position']
        plt.plot(vx, vy, 'bo', label='Vehicle Start')
        plt.arrow(vx, vy, np.cos(vehicle_state['heading']) * 2, np.sin(vehicle_state['heading']) * 2,
                head_width=0.25, color='blue')

        # Plot flexible waypoint
        if wpt_flexes is not None:
            for wpt_flex in wpt_flexes:
                plt.plot(wpt_flex[0], wpt_flex[1], 'go', label='Flexible Waypoint')
                plt.text(wpt_flex[0], wpt_flex[1] + 0.5, 'Flex', fontsize=9, color='green')

        # Plot fixed waypoint
        if wpt_fixed is not None:
            plt.plot(wpt_fixed[0], wpt_fixed[1], 'ro', label='Fixed Waypoint')
            plt.text(wpt_fixed[0], wpt_fixed[1] + 0.5, 'Fixed', fontsize=9, color='red')

        plt.title(scenario_label)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.legend()
        plt.show()
        return
    
    # --------------------------
    # Vehicle State Test Update 
    # --------------------------
    def drive(vehicle_state):
        # Update vehicle state
        vehicle_state['position'][0] += vehicle_state['velocity'] * np.cos(vehicle_state['heading'])
        vehicle_state['position'][1] += vehicle_state['velocity'] * np.sin(vehicle_state['heading'])
        return vehicle_state
    # --------------------------
    # Step 1: Generate Fake Cones
    # --------------------------
    def generate_test_cones(case='slalom'):
        if case == 'slalom':
            cones = []
            for i in range(3):
                x = (i+1) * 10
                y = 0 if i % 2 == 0 else 1
                orientation = 'left' if i % 2 == 0 else 'right'
                cones.append({'x': x, 'y': y, 'orientation': orientation})
            return cones

        elif case == 'u_turn':
            return [{'x': 10, 'y': 0, 'orientation': 'standing'}]

    # --------------------------
    # Step 2: Define Fake Vehicle
    # --------------------------    
    def get_test_vehicle_state(vehicle_state=None):
        if vehicle_state is not None:
            return vehicle_state
        return {
            'position': [0, 0],
            'heading': 0.0 * 180/np.pi,  # Facing right
            'velocity': 10.0
        }

    # --------------------------
    # Step 3: Run Scenario + Waypoints
    # --------------------------
    def test_waypoint_generation(case='slalom', test_loop=1):
        vehicle_state = get_test_vehicle_state()
        cones = generate_test_cones(case)
        
        for i in range(test_loop):
            scenario = scenario_check(vehicle_state, cones)
            scenario_label = f"Scenario: {scenario}"
            scenario, wpt_flexes, wpt_fixed  = waypoint_generate(vehicle_state, cones)
            plot_results(vehicle_state, cones, wpt_flexes, wpt_fixed, scenario_label)
            vehicle_state = drive(vehicle_state)
            if case == 'slalom':
                cones.pop(0)

    # --------------------------
    # Main for waypoint
    # --------------------------
    # test_waypoint_generation(case='slalom', test_loop=2)
    # test_waypoint_generation(case='u_turn')


    # --------------------------
    ### Combine Test
    # --------------------------
    def test_planning(case='slalom', test_loop=2):
        vehicle_state = get_test_vehicle_state()
        cones = generate_test_cones(case)
        
        for i in range(test_loop):
            # Way Points
            scenario = scenario_check(vehicle_state, cones)
            scenario_label = f"Scenario: {scenario}"
            scenario, wpt_flexes, wpt_fixed  = waypoint_generate(vehicle_state, cones)
            plot_results(vehicle_state, cones, wpt_flexes, wpt_fixed, scenario_label)
            # Trajectory
            init_state = {'y': wpt_flexes[0][1], 'psi': 0.0, 'c': 0.0}
            final_state = {'y': wpt_fixed[1], 'psi': 0.0, 'c': 0.0}

            y_traj, psi_traj, c_traj, eps_traj = trajectory_generation(init_state, final_state)
            # plot_trajectory(y_traj, psi_traj, c_traj, label="Generated trajectory")
            # plot_dynamics(psi_traj, c_traj, eps_traj)

            # Iterate
            vehicle_state = drive(vehicle_state)
            if case == 'slalom':
                cones.pop(0)
    #test_planning(case='slalom', test_loop=2)


def waypoint_search_optimization(vehicle_state, cones, search_attempts=3):
    valid_flex_wps = []
    valid_fixed_wps = []
    current_state = vehicle_state.copy()
    cones_copy = cones.copy()

    for i in range(min(search_attempts, len(cones_copy))):
        scenario, flex_wps, fixed_wp = waypoint_generate(current_state, cones_copy)

        if flex_wps is None or fixed_wp is None:
            break

        for flex_wp in flex_wps:
            init_state = {
                'x': current_state['position'][0],
                'y': current_state['position'][1],
                'psi': current_state['heading'],
                'c': 0.0,
                'v': current_state['velocity']
            }
            final_state = {
                'x': fixed_wp[0],
                'y': fixed_wp[1],
                'psi': current_state['heading'],
                'c': 0.0
            }

            try:
                x, y, psi, c, v, eps, final_error = trajectory_generation(init_state, final_state, waypoint=flex_wp)
                trajectory = list(zip(y, psi, c))
                feasible, collisions, x_vals, y_vals = feasibility_check(trajectory, [(cone['x'], cone['y']) for cone in cones])
                print(f"Checking waypoint: {flex_wp}, Fixed: {fixed_wp}, Feasible: {feasible}, Collisions: {collisions}")
                if feasible:
                    valid_flex_wps.append(flex_wp)
                    valid_fixed_wps.append(fixed_wp)
                    current_state['position'] = list(fixed_wp)
                    break
            except:
                continue

        if len(cones_copy) > 0:
            cones_copy.pop(0)

    return valid_flex_wps, valid_fixed_wps


def test_4_cone_slalom():
    vehicle_state = {
        'position': [0.0, 0.0],
        'heading': 0.0,
        'velocity': 5.0
    }

    cones = [
        {'x': 10, 'y': 0.0, 'orientation': 'left'},
        {'x': 30, 'y': 1.0, 'orientation': 'right'},
        {'x': 50, 'y': 0.0, 'orientation': 'left'},
        {'x': 70, 'y': 1.0, 'orientation': 'standing'}
    ]

    flex_wps, fixed_wps = waypoint_search_optimization(vehicle_state, cones, search_attempts=4)

    current_pos = np.array(vehicle_state['position'])
    
    for idx, (flex_wp, fixed_wp) in enumerate(zip(flex_wps, fixed_wps)):
        init_state = {
            'x': current_pos[0], 'y': current_pos[1], 'psi': vehicle_state['heading'],
            'c': 0.0, 'v': vehicle_state['velocity']
        }
        final_state = {
            'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': vehicle_state['heading'], 'c': 0.0
        }
        x, y, psi, c, v, eps, final_error = trajectory_generation(init_state, final_state, waypoint=flex_wp)

        print(f"\nSegment {idx + 1}:")
        print(f"  Waypoint: {flex_wp}")
        print(f"  Final Errors:")
        for k, e in final_error.items():
            print(f"    {k}: {e:.4f}")

        plot_trajectory(x, y, v, c, eps, waypoint=flex_wp)

        current_pos = np.array(fixed_wp)

#test_4_cone_slalom()

from ...state.trajectory import Trajectory, compute_headings

def to_gemstack_trajectory(x_all, y_all, v_all, T=0.1):
    t_vals = np.arange(len(x_all)) * T
    headings = compute_headings(x_all, y_all)
    points = [[x, y, h] for x, y, h in zip(x_all, y_all, headings)]
    return Trajectory(points=points, times=t_vals)

def plan_full_slalom_trajectory(vehicle_state, cones):
    x_all, y_all, v_all = [], [], []
    current_pos = np.array(vehicle_state['position'])
    current_heading = vehicle_state['heading']

    for cone in cones:
        scenario, flex_wps, fixed_wp = waypoint_generate(vehicle_state, [cone])
        if not flex_wps or fixed_wp is None:
            continue
        flex_wp = flex_wps[0]

        init_state = {
            'x': current_pos[0], 'y': current_pos[1], 'psi': current_heading,
            'c': 0.0, 'v': vehicle_state['velocity']
        }
        final_state = {
            'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': current_heading, 'c': 0.0
        }

        x, y, psi, c, v, eps, _ = trajectory_generation(init_state, final_state, waypoint=flex_wp)
        x_all.extend(x)
        y_all.extend(y)
        v_all.extend(v)

        current_pos = np.array([x[-1], y[-1]])

    return to_gemstack_trajectory(x_all, y_all, v_all)

from ...state.trajectory import Trajectory
from ...state.vehicle import VehicleState
from ..component import Component

class SlalomTrajectoryPlanner(Component):
    def __init__(self, **kwargs):
        # You can accept args here if needed
        self.trajectory = None
        self.planned = False

    def state_inputs(self):
        return ['vehicle']  # Will receive a VehicleState input

    def state_outputs(self):
        return ['trajectory']

    def update(self, vehicle: VehicleState):
        if not self.planned:
            cones = [
                {'x': 10, 'y': 0.0, 'orientation': 'left'},
                {'x': 30, 'y': 1.0, 'orientation': 'right'},
                {'x': 50, 'y': 0.0, 'orientation': 'left'},
                {'x': 70, 'y': 1.0, 'orientation': 'standing'}
            ]
            vehicle_dict = {
                'position': [vehicle.pose.x, vehicle.pose.y],
                'heading': vehicle.pose.yaw,
                'velocity': vehicle.v
            }
            self.trajectory = plan_full_slalom_trajectory(vehicle_dict, cones)
            self.planned = True
        return self.trajectory
