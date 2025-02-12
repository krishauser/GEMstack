# File: GEMstack/onboard/planning/longitudinal_planning.py
from typing import List
import math
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils import transforms
import numpy as np
DEBUG = True  # Set to False to disable debug output

def compute_cumulative_distances(points: List[List[float]]) -> List[float]:
    s_vals = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        ds = math.hypot(dx, dy)
        s_vals.append(s_vals[-1] + ds)
    if DEBUG:
        print("[DEBUG] compute_cumulative_distances: s_vals =", s_vals)
    return s_vals

def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    # Reparameterize the path by arc length (to get a timed trajectory)
    path_normalized: Trajectory = path.arc_length_parameterize()
    points = list(path_normalized.points)
    s_vals = compute_cumulative_distances(points)
    L = s_vals[-1]
    if DEBUG:
        print("[DEBUG] longitudinal_plan: Total path length L =", L)
        print("[DEBUG] longitudinal_plan: Braking distance needed =", (current_speed**2) / (2 * deceleration))
    
    if acceleration <= 0:
        # Compute deceleration phase: the distance needed to decelerate to 0.
        s_decel = (current_speed**2) / (2 * deceleration)
        T_decel = current_speed / deceleration
        times = []
        for s in s_vals:
            if s <= s_decel:
                # Compute the instantaneous speed at distance s:
                v = math.sqrt(max(current_speed**2 - 2 * deceleration * s, 0.0))
                t_point = (current_speed - v) / deceleration
            else:
                t_point = T_decel
            times.append(t_point)
        if DEBUG:
            print("[DEBUG] longitudinal_plan (accel==0): Final times =", times)
        return Trajectory(frame=path.frame, points=points, times=times)
    
    # Otherwise, follow the full trapezoidal profile.
    # Determine the highest achievable speed given the available distance.
    v_peak_possible = math.sqrt((2 * acceleration * deceleration * L + deceleration * current_speed**2) / (acceleration + deceleration))
    v_target = min(max_speed, v_peak_possible)
    if DEBUG:
        print("[DEBUG] longitudinal_plan: v_peak_possible =", v_peak_possible, "v_target =", v_target)
    
    # Calculate acceleration phase (if any)
    if v_target > current_speed:
        s_accel = (v_target**2 - current_speed**2) / (2 * acceleration)
        t_accel = (v_target - current_speed) / acceleration
    else:
        s_accel = 0.0
        t_accel = 0.0

    # Calculate deceleration phase (from v_target to 0)
    s_decel = (v_target**2) / (2 * deceleration)
    t_decel = v_target / deceleration

    # Compute the cruise (constant-speed) segment, if any.
    s_cruise = max(0.0, L - s_accel - s_decel)
    t_cruise = s_cruise / v_target if v_target > 0 else 0.0

    if DEBUG:
        print("[DEBUG] longitudinal_plan: s_accel =", s_accel, "t_accel =", t_accel)
        print("[DEBUG] longitudinal_plan: s_decel =", s_decel, "t_decel =", t_decel)
        print("[DEBUG] longitudinal_plan: s_cruise =", s_cruise, "t_cruise =", t_cruise)
    
    # For each point along the path (by its cumulative distance s), compute its time stamp.
    times = []
    for s in s_vals:
        if s <= s_accel:
            # Acceleration phase: v^2 = current_speed^2 + 2*a*s.
            v = math.sqrt(current_speed**2 + 2 * acceleration * s)
            t_point = (v - current_speed) / acceleration
            if DEBUG:
                print(f"[DEBUG] longitudinal_plan (accel): s = {s:.2f}, v = {v:.2f}, t = {t_point:.2f}")
        elif s <= s_accel + s_cruise:
            # Cruise phase.
            t_point = t_accel + (s - s_accel) / v_target
            if DEBUG:
                print(f"[DEBUG] longitudinal_plan (cruise): s = {s:.2f}, t = {t_point:.2f}")
        else:
            # Deceleration phase.
            s_decel_phase = s - s_accel - s_cruise
            under_sqrt = max(v_target**2 - 2 * deceleration * s_decel_phase, 0.0)
            t_dec = (v_target - math.sqrt(under_sqrt)) / deceleration
            t_point = t_accel + t_cruise + t_dec
            if DEBUG:
                print(f"[DEBUG] longitudinal_plan (decel): s = {s:.2f}, t_dec = {t_dec:.2f}, t = {t_point:.2f}")
        times.append(t_point)
    
    if DEBUG:
        print("[DEBUG] longitudinal_plan: Final times =", times)
    
    return Trajectory(frame=path.frame, points=points, times=times)


def longitudinal_brake(path: Path, deceleration: float, current_speed: float) -> Trajectory:
    # Vehicle already stopped - maintain position
    if current_speed <= 0:
        return Trajectory(
            frame=path.frame,
            points=[path.points[0]] * len(path.points),
            times=[float(i) for i in range(len(path.points))]
        )

    # Calculate stopping time and distance
    T_stop = current_speed / deceleration
    s_stop = current_speed * T_stop - 0.5 * deceleration * (T_stop ** 2)
    
    # Get total path length
    path_length = sum(
        np.linalg.norm(np.array(path.points[i+1]) - np.array(path.points[i]))
        for i in range(len(path.points)-1)
    )

    # Generate time points (use more points for smoother trajectory)
    num_points = max(len(path.points), 50)
    times = np.linspace(0, T_stop, num_points)
    
    # Calculate distances at each time point using physics equation
    distances = current_speed * times - 0.5 * deceleration * (times ** 2)
    
    # Generate points along the path
    points = []
    for d in distances:
        if d <= path_length:
            # If within path length, evaluate path at distance
            points.append(path.eval(d))
        else:
            points.append(path.points[-1])


    return Trajectory(frame=path.frame, points=points, times=times.tolist())

class YieldTrajectoryPlanner(Component):
    """Follows the given route. Brakes if the ego–vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5
        self.desired_speed = 1.0
        self.deceleration = 2.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route      # type: Route
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
            print(f"[DEBUG] YieldTrajectoryPlanner.update: Vehicle position = ({curr_x}, {curr_y}), speed = {curr_v}")

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

        print("[DEBUG] state", state.relations)
        # Check whether any yield relations (e.g. due to pedestrians) require braking.
        should_brake = any(
            r.type == EntityRelationEnum.YIELDING and r.obj1 == ''
            for r in state.relations
        )
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: should_brake =", should_brake)
        should_accelerate = (not should_brake and curr_v < self.desired_speed)
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: should_accelerate =", should_accelerate)

        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration,
                                     self.deceleration, self.desired_speed, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_plan (accelerate).")
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake.")
        else:
            # Maintain current speed if not accelerating or braking.
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Maintaining current speed with longitudinal_plan (0 accel).")

        self.t_last = t
        if DEBUG:
            print('[DEBUG] Current Velocity of the Car: LOOK!', curr_v, self.desired_speed)
            print("[DEBUG] YieldTrajectoryPlanner.update: Returning trajectory with", len(traj.points), "points.")
        return traj
