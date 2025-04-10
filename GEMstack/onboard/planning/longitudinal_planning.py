# File: GEMstack/onboard/planning/longitudinal_planning.py
from typing import List, Tuple
import math
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, \
    ObjectFrameEnum
from ...utils import serialization
from ...mathutils import transforms
import numpy as np

DEBUG = False  # Set to False to disable debug output


def generate_dense_points(points: List[Tuple[float, float]], density: int = 10) -> List[Tuple[float, float]]:
    if not points:
        return []
    if len(points) == 1:
        return points.copy()

    dense_points = [points[0]]
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg_length = math.hypot(dx, dy)

        n_interp = int(round(seg_length * density))

        for j in range(1, n_interp + 1):
            fraction = j / (n_interp + 1)
            x_interp = p0[0] + fraction * dx
            y_interp = p0[1] + fraction * dy
            dense_points.append((x_interp, y_interp))

        dense_points.append(p1)

    return dense_points


def compute_cumulative_distances(points: List[List[float]]) -> List[float]:
    s_vals = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        ds = math.hypot(dx, dy)
        s_vals.append(s_vals[-1] + ds)

    if DEBUG:
        print("[DEBUG] compute_cumulative_distances: s_vals =", s_vals)

    return s_vals


def longitudinal_plan(path, acceleration, deceleration, max_speed, current_speed):
    path_normalized = path.arc_length_parameterize()
    points = list(path_normalized.points)
    dense_points = generate_dense_points(points)
    s_vals = compute_cumulative_distances(dense_points)
    L = s_vals[-1]  # Total path length
    stopping_distance = (current_speed ** 2) / (2 * deceleration)

    if DEBUG:
        print("[DEBUG] compute_cumulative_distances: s_vals =", s_vals)
        print("[DEBUG] longitudinal_plan: Total path length L =", L)
        print("[DEBUG] longitudinal_plan: Braking distance needed =", stopping_distance)

    if stopping_distance > L:  # Case where there is not enough stopping distance to stop before path ends (calls emergency brake)
        return longitudinal_brake(path, deceleration, current_speed)

    if current_speed > max_speed:  # Case where car is exceeding the max speed so we need to slow down (do initial slowdown)
        if DEBUG:
            print(f"[DEBUG] Handling case where current_speed ({current_speed:.2f}) > max_speed ({max_speed:.2f})")

        # Initial deceleration phase to reach max_speed
        initial_decel_distance = (current_speed ** 2 - max_speed ** 2) / (2 * deceleration)
        initial_decel_time = (current_speed - max_speed) / deceleration
        remaining_distance = L - initial_decel_distance

        if DEBUG:
            print(
                f"[DEBUG] Phase 1 - Initial Decel: distance = {initial_decel_distance:.2f}, time = {initial_decel_time:.2f}")
            print(f"[DEBUG] Remaining distance after reaching max_speed: {remaining_distance:.2f}")

        # Calculate final deceleration distance needed to stop from max_speed
        final_decel_distance = (max_speed ** 2) / (2 * deceleration)
        cruise_distance = remaining_distance - final_decel_distance

        if DEBUG:
            print(f"[DEBUG] Phase 2 - Cruise: distance = {cruise_distance:.2f}")
            print(f"[DEBUG] Phase 3 - Final Decel: distance = {final_decel_distance:.2f}")

        times = []
        for s in s_vals:
            if s <= initial_decel_distance:  # Phase 1: Initial deceleration to max_speed
                v = math.sqrt(current_speed ** 2 - 2 * deceleration * s)
                t = (current_speed - v) / deceleration
                if DEBUG:  # Print every 10m
                    print(f"[DEBUG] Initial Decel: s = {s:.2f}, v = {v:.2f}, t = {t:.2f}")

            elif s <= initial_decel_distance + cruise_distance:  # Phase 2: Cruise at max_speed
                s_in_cruise = s - initial_decel_distance    
                t = initial_decel_time + s_in_cruise / max_speed
                if DEBUG:  # Print every 10m
                    print(f"[DEBUG] Cruise: s = {s:.2f}, v = {max_speed:.2f}, t = {t:.2f}")

            else:  # Phase 3: Final deceleration to stop
                s_in_final_decel = s - (initial_decel_distance + cruise_distance)
                v = math.sqrt(max(max_speed ** 2 - 2 * deceleration * s_in_final_decel, 0.0))
                t = initial_decel_time + cruise_distance / max_speed + (max_speed - v) / deceleration
                if DEBUG:  # Print every 10m
                    print(f"[DEBUG] Final Decel: s = {s:.2f}, v = {v:.2f}, t = {t:.2f}")

            times.append(t)

        if DEBUG:
            print("[DEBUG] Trajectory complete: Three phases executed")
            print(f"[DEBUG] Total time: {times[-1]:.2f}")

        return Trajectory(frame=path.frame, points=dense_points, times=times)

    if acceleration <= 0:
        if DEBUG:
            print(f"[DEBUG] No acceleration allowed. Current speed: {current_speed:.2f}")

        # Pure deceleration phase
        s_decel = (current_speed ** 2) / (2 * deceleration)
        T_decel = current_speed / deceleration

        if DEBUG:
            print(f"[DEBUG] Will maintain speed until s_decel: {s_decel:.2f}")
            print(f"[DEBUG] Total deceleration time will be: {T_decel:.2f}")

        times = []
        for s in s_vals:
            if s <= L - s_decel:  # Maintain current speed until deceleration point
                t_point = s / current_speed
                if DEBUG:
                    print(f"[DEBUG] Constant Speed Phase: s = {s:.2f}, v = {current_speed:.2f}, t = {t_point:.2f}")
            else:  # Deceleration phase
                s_in_decel = s - (L - s_decel)
                v = math.sqrt(max(current_speed ** 2 - 2 * deceleration * s_in_decel, 0.0))
                t_point = (L - s_decel) / current_speed + (current_speed - v) / deceleration
                if DEBUG:
                    print(f"[DEBUG] Deceleration Phase: s = {s:.2f}, v = {v:.2f}, t = {t_point:.2f}")

            times.append(t_point)

        return Trajectory(frame=path.frame, points=dense_points, times=times)

    # Determine max possible peak speed given distance
    v_peak_possible = math.sqrt(
        (2 * acceleration * deceleration * L + deceleration * current_speed ** 2) / (acceleration + deceleration))
    v_target = min(max_speed, v_peak_possible)

    if DEBUG:
        print("[DEBUG] longitudinal_plan: v_peak_possible =", v_peak_possible, "v_target =", v_target)

    # Compute acceleration phase
    s_accel = max(0.0, (v_target ** 2 - current_speed ** 2) / (2 * acceleration))
    t_accel = max(0.0, (v_target - current_speed) / acceleration)

    # Compute deceleration phase
    s_decel = max(0.0, (v_target ** 2) / (2 * deceleration))
    t_decel = max(0.0, v_target / deceleration)

    # Compute cruise phase
    s_cruise = max(0.0, L - s_accel - s_decel)
    t_cruise = s_cruise / v_target if v_target > 0 else 0.0

    if DEBUG:
        print("[DEBUG] longitudinal_plan: s_accel =", s_accel, "t_accel =", t_accel)
        print("[DEBUG] longitudinal_plan: s_decel =", s_decel, "t_decel =", t_decel)
        print("[DEBUG] longitudinal_plan: s_cruise =", s_cruise, "t_cruise =", t_cruise)

    times = []
    for s in s_vals:
        if s <= s_accel:  # Acceleration phase
            v = math.sqrt(current_speed ** 2 + 2 * acceleration * s)
            t_point = (v - current_speed) / acceleration

            if DEBUG:
                print(f"[DEBUG] Acceleration Phase: s = {s:.2f}, v = {v:.2f}, t = {t_point:.2f}")

        elif s <= s_accel + s_cruise:  # Cruise phase
            t_point = t_accel + (s - s_accel) / v_target

            if DEBUG:
                print(f"[DEBUG] Cruise Phase: s = {s:.2f}, t = {t_point:.2f}")

        else:  # Deceleration phase
            s_decel_phase = s - s_accel - s_cruise
            v_decel = math.sqrt(max(v_target ** 2 - 2 * deceleration * s_decel_phase, 0.0))
            t_point = t_accel + t_cruise + (v_target - v_decel) / deceleration

            if t_point < times[-1]:  # Ensure time always increases
                t_point = times[-1] + 0.01  # Small time correction step

            if DEBUG:
                print(f"[DEBUG] Deceleration Phase: s = {s:.2f}, v = {v_decel:.2f}, t = {t_point:.2f}")

        times.append(t_point)

    if DEBUG:
        print("[DEBUG] longitudinal_plan: Final times =", times)

    return Trajectory(frame=path.frame, points=dense_points, times=times)


def longitudinal_brake(path: Path, deceleration: float, current_speed: float, emergency_decel: float = 8.0) -> Trajectory:
    # Vehicle already stopped - maintain position
    if current_speed <= 0:
        print("[DEBUG] longitudinal_brake: Zero velocity case! ", [path.points[0]] * len(path.points))
        print("[DEBUG] longitudinal_brake: 2")
        return Trajectory(
            frame=path.frame,
            points=[path.points[0]] * len(path.points),
            times=[float(i) for i in range(len(path.points))]
        )

    # Get total path length
    path_length = sum(
        np.linalg.norm(np.array(path.points[i+1]) - np.array(path.points[i]))
        for i in range(len(path.points)-1)
    )

    # Calculate stopping distance with normal deceleration
    T_stop_normal = current_speed / deceleration
    s_stop_normal = current_speed * T_stop_normal - 0.5 * deceleration * (T_stop_normal ** 2)
    
    # Check if emergency braking is needed
    if s_stop_normal > path_length:
        if DEBUG:
            print("[DEBUG] longitudinal_brake: Emergency braking needed!")
            print(f"[DEBUG] longitudinal_brake: Normal stopping distance: {s_stop_normal:.2f}m")
            print(f"[DEBUG] longitudinal_brake: Available distance: {path_length:.2f}m")
        
        # Calculate emergency braking parameters
        T_stop = current_speed / emergency_decel
        s_stop = current_speed * T_stop - 0.5 * emergency_decel * (T_stop ** 2)
        
        if DEBUG:
            print(f"[DEBUG] longitudinal_brake: Emergency stopping distance: {s_stop:.2f}m")
            print(f"[DEBUG] longitudinal_brake: Emergency stopping time: {T_stop:.2f}s")
        
        decel_to_use = emergency_decel
        
    else:
        if DEBUG:
            print("[DEBUG] longitudinal_brake: Normal braking sufficient")
        T_stop = T_stop_normal
        decel_to_use = deceleration

    # Generate time points (use more points for smoother trajectory)
    num_points = max(len(path.points), 50)
    times = np.linspace(0, T_stop, num_points)
    
    # Calculate distances at each time point using physics equation
    distances = current_speed * times - 0.5 * decel_to_use * (times ** 2)
    
    # Generate points along the path
    points = []
    for d in distances:
        if d <= path_length:
            points.append(path.eval(d))
        else:
            points.append(path.eval(d))

    if DEBUG:
        print(f"[DEBUG] longitudinal_brake: Using deceleration of {decel_to_use:.2f} m/s²")
        print(f"[DEBUG] longitudinal_brake: Final stopping time: {T_stop:.2f}s")

    return Trajectory(frame=path.frame, points=points, times=times.tolist())

def inverse_speed_function(distance_to_object):
    
    max_creep_speed = 1.0  # Maximum speed when creeping (m/s)
    min_creep_speed = 0.2  # Minimum speed to maintain (m/s)
    safe_distance = 5.0    # Distance at which to start slowing down (m)
    
    # Almost stop when very close (≤0.25m)
    if distance_to_object <= 0.25:
        return 0.0  # Practically zero speed
    
    # Slow creep when close (≤1.5m)
    if distance_to_object <= 3.0:
        return min_creep_speed
    speed = min_creep_speed + (max_creep_speed - min_creep_speed) * (distance_to_object / safe_distance)
    return min(speed, max_creep_speed)

def pid_speed_control(distance_to_object, target_distance, current_speed, prev_error, integral, dt, 
                     kp=0.5, ki=0.1, kd=0.05, min_speed=0.2, max_speed=1.0):
    """
    PID controller for creep speed control based on distance to object.
    
    Args:
        distance_to_object: Current distance to the object (m)
        target_distance: Desired distance to maintain (m)
        current_speed: Current vehicle speed (m/s)
        prev_error: Previous error value (m)
        integral: Accumulated error over time
        dt: Time step (s)
        kp, ki, kd: PID gains
        min_speed, max_speed: Speed limits (m/s)
        
    Returns:
        target_speed: Calculated target speed (m/s)
        error: Current error for next iteration
        integral: Updated integral term for next iteration
    """
    # Emergency stop when very close
    if distance_to_object <= 0.25:
        return 0.0, 0.0, 0.0
    
    # Calculate error (positive error means we're further than desired)
    error = distance_to_object - target_distance
    
    # Update integral term with anti-windup
    integral += error * dt
    integral = max(-5.0, min(integral, 5.0))  # Prevent integral windup
    
    # Calculate derivative term
    derivative = (error - prev_error) / dt if dt > 0 else 0
    
    # Calculate PID output
    p_term = kp * error
    i_term = ki * integral
    d_term = kd * derivative
    
    # Base speed adjustment
    speed_adjustment = p_term + i_term + d_term
    
    # Calculate target speed (base + adjustment)
    base_speed = 0.5  # Base creep speed
    target_speed = base_speed + speed_adjustment
    
    # Clamp speed between min and max values
    target_speed = max(min_speed, min(target_speed, max_speed))
    
    return target_speed, error, integral



class YieldTrajectoryPlanner(Component):
    """Follows the given route. Brakes if the ego–vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed. Also handles
    maintaining safe distance from pedestrians by accelerating or decelerating.
    """

    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 5
        self.desired_speed = 2.0
        self.deceleration = 2.0
        self.emergency_brake = 8.0
        self.follow_acceleration = 3.0  # Slightly gentler acceleration for following
        self.follow_deceleration = 1.5  # Slightly gentler deceleration for following



        self.pid_prev_error = 0.0
        self.pid_integral = 0.0
        self.pid_target_distance = 3.0


    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
        t = state.t

        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: t =", t)

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: dt =", dt)

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v
        if DEBUG:
            print(f"[DEBUG] YieldTrajectoryPlanner.update: Vehicle position = ({curr_x}, {curr_y}), speed = {curr_v}, ")

        # Determine progress along the route.
        if self.route_progress is None:
            self.route_progress = 0.0
        _, closest_parameter = route.closest_point_local(
            [curr_x, curr_y],
            (self.route_progress - 5.0, self.route_progress + 5.0)
        )
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: Closest parameter on route =", closest_parameter)
        self.route_progress = closest_parameter

        # Extract a 10 m segment of the route for planning lookahead.
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: Route Lookahead =", route_with_lookahead)

        print("[DEBUG] state", state.relations)
        # Check whether any yield relations (e.g. due to pedestrians) require braking.
        stay_braking = False
        pointSet = set()
        for i in range(len(route_with_lookahead.points)):
            if tuple(route_with_lookahead.points[i]) in pointSet:
                stay_braking = True
                break
            pointSet.add(tuple(route_with_lookahead.points[i]))

        # Check for different relation types
        should_brake = any(
            r.type == EntityRelationEnum.STOPPING_AT and r.obj1 == ''
            for r in state.relations
        )
        
        # Only check for yielding if not already braking
        should_decelerate = any(
            (r.type == EntityRelationEnum.YIELDING or r.type == EntityRelationEnum.CREEPING) and r.obj1 == ''
            for r in state.relations
        ) if not should_brake else False
        
        # Check if we need to accelerate to maintain safe distance
        should_accelerate_for_distance = any(
            r.type == EntityRelationEnum.ACCELERATING and r.obj1 == ''
            for r in state.relations
        ) if not (should_brake or should_decelerate) else False
        
        # Normal acceleration if below desired speed and no other constraints
        should_accelerate = (not should_brake and not should_decelerate and 
                            not should_accelerate_for_distance and 
                            curr_v < self.desired_speed)

        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: stay_braking =", stay_braking)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_brake =", should_brake)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_accelerate =", should_accelerate)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_decelerate =", should_decelerate)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_accelerate_for_distance =", should_accelerate_for_distance)

        if stay_braking:
            traj = longitudinal_brake(route_with_lookahead, 0.0, 0.0, 0.0)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake (stay braking).")
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.emergency_brake, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake (emergency).")
        elif should_decelerate:
            # Check if specifically in CREEPING state
            is_creeping = any(
                r.type == EntityRelationEnum.CREEPING and r.obj1 == ''
                for r in state.relations
            )
            
            if is_creeping:
                # Find the closest object in front
                closest_distance = float('inf')
                for r in state.relations:
                    if r.type == EntityRelationEnum.CREEPING and r.obj1 == '':
                        # Extract distance from relation data
                        if hasattr(r, 'distance') and r.distance < closest_distance:
                            closest_distance = r.distance
                
                # Use inverse function to determine appropriate speed
                #target_speed = inverse_speed_function(closest_distance)
                dt = t - self.t_last
                # Use PID function to determine appropriate speed
                target_speed, self.pid_prev_error, self.pid_integral = pid_speed_control(
                    closest_distance, 
                    self.pid_target_distance,
                    curr_v,
                    self.pid_prev_error,
                    self.pid_integral,
                    dt
                )

                traj = longitudinal_plan(route_with_lookahead, 0.0, 
                            self.follow_deceleration, target_speed, curr_v)
                
                # Use longitudinal_plan with zero acceleration and the target speed
                #traj = longitudinal_plan(route_with_lookahead, 0.0,   self.follow_deceleration, target_speed, curr_v)
                if DEBUG:
                    print(f"[DEBUG] YieldTrajectoryPlanner.update: CREEPING with distance={closest_distance:.2f}m, target_speed={target_speed:.2f}m/s")
            else:
                # Regular yielding behavior
                decel_value = self.deceleration
                traj = longitudinal_brake(route_with_lookahead, decel_value, curr_v)
                if DEBUG:
                    print(f"[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake with decel={decel_value}.")
        elif should_accelerate_for_distance:
            # Accelerate to maintain proper following distance
            traj = longitudinal_plan(route_with_lookahead, self.follow_acceleration,
                                    self.deceleration, self.desired_speed, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_plan (accelerate for distance).")
        elif should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration,
                                    self.deceleration, self.desired_speed, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_plan (normal accelerate).")
        else:
            # Maintain current speed if not accelerating or braking.
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)
            if DEBUG:
                print(
                    "[DEBUG] YieldTrajectoryPlanner.update: Maintaining current speed with longitudinal_plan (0 accel).")

        self.t_last = t
        if DEBUG:
            print('[DEBUG] Current Velocity of the Car: LOOK!', curr_v, self.desired_speed)
            print("[DEBUG] YieldTrajectoryPlanner.update: Returning trajectory with", len(traj.points), "points.")
        return traj
