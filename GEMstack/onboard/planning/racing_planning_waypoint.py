# from typing import List, Tuple, Union
# from ..component import Component
# from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum, AgentState
# from ...utils import serialization, settings
# from ...mathutils.transforms import vector_madd
# from ...mathutils.quadratic_equation import quad_root

# ===== Additional Imports =====
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
# ==============================

#TODO: Combine with current planner and GemStack
# def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float,
#                       method : str) -> Trajectory:
    
#     return Trajectory(frame=path.frame, points=dense_points, times=times)

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
    u_turn_radius = 5.0  # Radius for U-turn
    offset = 2.0  # Offset for left/right pass
    lookahead_distance = 5.0  # Distance ahead for fixed point
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
        flex_wps = [flex_wp1, flex_wp2]

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
    test_waypoint_generation(case='slalom', test_loop=2)
    test_waypoint_generation(case='u_turn')
