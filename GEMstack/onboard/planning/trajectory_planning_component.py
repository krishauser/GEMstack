from typing import List, Tuple
import math
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils import transforms
import numpy as np

DEBUG = False  # Set to False to disable debug output

def scurve(x):
    x = np.clip(x, 0.0, 1.0)
    return 6 * x**5 - 15 * x**4 + 10 * x**3

def jerk_limited_brake(v0: float, a_max: float, j_max: float, dt: float = 0.01):
    tj = a_max / j_max
    T_flat = (v0 - a_max * tj) / a_max if v0 > a_max * tj else 0.0
    T = 2 * tj + T_flat
    
    times = [0.0]
    velocities = [v0]
    positions = [0.0]
    t, v, s = 0.0, v0, 0.0
    while v > 0:
        if t < tj:
            a = -j_max * t
        elif t < tj + T_flat:
            a = -a_max
        elif t < T:
            t3 = t - (T - tj)
            a = -a_max + j_max * t3
        else:
            a = 0.0
        v_next = max(0.0, v + a * dt)
        s_next = s + 0.5 * (v + v_next) * dt
        t += dt
        times.append(t)
        velocities.append(v_next)
        positions.append(s_next)
        v, s = v_next, s_next
    return times, velocities, positions

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

def longitudinal_plan(path, acceleration, deceleration, max_speed, current_speed, planner_type="standard"):
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
        return longitudinal_brake(path, deceleration, current_speed, planner_type=planner_type)
    
    if current_speed > max_speed:  # Case where car is exceeding the max speed so we need to slow down (do initial slowdown)
        if planner_type == "scurve":
            print("IN CURRENT SPEED > MAX SPEED CASE")
        
        if DEBUG:
            print(f"[DEBUG] Handling case where current_speed ({current_speed:.2f}) > max_speed ({max_speed:.2f})")
        
        # PHASE 1: Initial deceleration from current_speed to max_speed
        s_decel1 = (current_speed ** 2 - max_speed ** 2) / (2 * deceleration)
        t_decel1 = (current_speed - max_speed) / deceleration
        
        # PHASE 3: Final deceleration from max_speed to 0
        s_decel3 = (max_speed ** 2) / (2 * deceleration)
        t_decel3 = max_speed / deceleration
        
        # PHASE 2: Cruise at max_speed for remaining path
        s_cruise = max(0.0, L - s_decel1 - s_decel3)
        t_cruise = s_cruise / max_speed if max_speed > 0 else 0.0
        
        if planner_type == "standard":
            # Initial deceleration phase to reach max_speed
            initial_decel_distance = (current_speed ** 2 - max_speed ** 2) / (2 * deceleration)
            initial_decel_time = (current_speed - max_speed) / deceleration
            remaining_distance = L - initial_decel_distance
            
            if DEBUG:
                print(f"[DEBUG] Phase 1 - Initial Decel: distance = {initial_decel_distance:.2f}, time = {initial_decel_time:.2f}")
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
        
        else:  # scurve planner
            times = []
            t = 0.0
            prev_s = 0.0
            for s in s_vals:
                ds = s - prev_s
                if s <= s_decel1:
                    # Phase 1: Decel from current_speed to max_speed
                    ratio = s / s_decel1 if s_decel1 > 0 else 1.0
                    v = current_speed + (max_speed - current_speed) * scurve(ratio)
                elif s <= s_decel1 + s_cruise:
                    # Phase 2: Cruise at max speed
                    v = max_speed
                elif s <= s_decel1 + s_cruise + s_decel3:
                    s_in_final = s - (s_decel1 + s_cruise)
                    ratio = s_in_final / s_decel3 if s_decel3 > 0 else 1.0
                    v = max_speed * (1.0 - scurve(ratio))
                else:
                    if DEBUG:
                        print("[DEBUG] Trajectory complete: Three phases executed")
                        print(f"[DEBUG] Total time: {times[-1]:.2f}")
                    return Trajectory(frame=path.frame, points=dense_points[:len(times)], times=times)
                
                v = max(1e-3, v)
                dt = ds / v
                t += dt
                times.append(t)
                prev_s = s
            
            return Trajectory(frame=path.frame, points=dense_points[:len(times)], times=times)
    
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
    
    if planner_type == "standard":
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
    
    else:  # scurve planner
        times = []
        t = 0.0
        prev_s = 0.0
        beyond_stop = False
        
        for s in s_vals:
            ds = s - prev_s
            if s <= s_accel:  # Acceleration phase
                ratio = s / s_accel if s_accel > 0 else 1.0
                v = current_speed + (v_target - current_speed) * scurve(ratio)
                if DEBUG:
                    print(f"[DEBUG] Acceleration Phase: s = {s:.2f}, v = {v:.2f}")
            elif s <= s_accel + s_cruise:  # Cruise phase
                v = v_target
                if DEBUG:
                    print(f"[DEBUG] Cruise Phase: s = {s:.2f}")
            elif s <= s_accel + s_cruise + s_decel:  # Deceleration phase
                s_decel_phase = s - s_accel - s_cruise
                ratio = s_decel_phase / s_decel if s_decel > 0 else 1.0
                v = v_target * (1.0 - scurve(ratio))
                if DEBUG:
                    print(f"[DEBUG] Deceleration Phase: s = {s:.2f}, v = {v:.2f}")
            else:
                beyond_stop = True
            
            v = max(1e-3, v)
            if beyond_stop:
                times.append(times[-1] + 0.01)
            else:
                dt = ds / v
                t += dt
                times.append(t)
            prev_s = s
        
        if DEBUG:
            print("[DEBUG] longitudinal_plan: Final times =", times)
        
        return Trajectory(frame=path.frame, points=dense_points, times=times)

def longitudinal_brake(path: Path, deceleration: float, current_speed: float, emergency_decel: float = 8.0, planner_type="standard"):
    # Vehicle already stopped - maintain position
    if current_speed <= 0:
        print("[DEBUG] longitudinal_brake: Zero velocity case! ", [path.points[0]] * len(path.points))
        return Trajectory(
            frame=path.frame,
            points=[path.points[0]] * len(path.points),
            times=[float(i) for i in range(len(path.points))]
        )
    
    # Get total path length
    path_length = sum(
        np.linalg.norm(np.array(path.points[i + 1]) - np.array(path.points[i]))
        for i in range(len(path.points) - 1)
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
        
        decel_to_use = emergency_decel
    else:
        if DEBUG:
            print("[DEBUG] longitudinal_brake: Normal braking sufficient")
        decel_to_use = deceleration
    
    T_stop = current_speed / decel_to_use
    s_stop = current_speed * T_stop - 0.5 * decel_to_use * (T_stop ** 2)
    
    if DEBUG:
        if planner_type == "standard":
            print(f"[DEBUG] longitudinal_brake: Emergency stopping distance: {s_stop:.2f}m")
            print(f"[DEBUG] longitudinal_brake: Emergency stopping time: {T_stop:.2f}s")
        else:
            print(f"[DEBUG] Using deceleration = {decel_to_use:.2f}, time to stop = {T_stop:.2f}s, distance = {s_stop:.2f}m")
    
    # Generate time points (use more points for smoother trajectory)
    num_points = max(len(path.points), 50)
    times = np.linspace(0, T_stop, num_points)
    
    if planner_type == "scurve":
        velocities = [current_speed * (1.0 - scurve(t/T_stop)) for t in times]
        positions = []
        s = 0.0
        for i in range(1, num_points):
            dt = times[i] - times[i-1]
            v_avg = 0.5 * (velocities[i] + velocities[i - 1])
            s += v_avg * dt
            positions.append(s)
        
        points = []
        for d in positions:
            d_clamped = min(d, path_length)
            points.append(path.eval(d_clamped))
        
        times = times[1:]
        return Trajectory(frame=path.frame, points=points, times=times)
    else:
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

class QuinticHermiteSplinePlanner:
    """
    Core quintic-Hermite engine: given coarse 2D or 3D waypoints
    (x,y[,heading]) builds a C2-continuous spline and samples it at fixed Δt.
    """
    def __init__(self, v_des: float = 1.0, dt: float = 0.02):
        self.v_des = v_des
        self.dt = dt
    
    def _compute_headings(self, pts: np.ndarray) -> np.ndarray:
        """
        If pts.shape[1] == 3, assume pts[:,2] already contains heading ψ.
        Otherwise fall back to finite-difference approximation.
        """
        n, d = pts.shape
        if d == 3:
            # user-provided headings
            return pts[:, 2].copy()
        # approximate by central differences
        headings = np.zeros(n)
        for i in range(n):
            if i == 0:
                delta = pts[1] - pts[0]
            elif i == n - 1:
                delta = pts[-1] - pts[-2]
            else:
                delta = pts[i+1] - pts[i-1]
            headings[i] = np.arctan2(delta[1], delta[0])
        return headings
    
    def build(self,
              waypoints: List[List[float]]
             ) -> Tuple[np.ndarray, np.ndarray]:
        """
        waypoints: list of [x,y] or [x,y,ψ] entries
        returns (pts_out, t_out), each as an np.ndarray
        """
        W = np.array(waypoints, float)         # shape = (n,2) or (n,3)
        headings = self._compute_headings(W)
        tangents = np.stack([np.cos(headings),
                             np.sin(headings)], axis=1) * self.v_des
        pts_out = []
        t_out   = []
        t_accum = 0.0
        M = np.array([[1,  1,   1],
                      [3,  4,   5],
                      [6, 12,  20]], float)
        # build one quintic segment between each adjacent pair
        for i in range(len(W) - 1):
            p0, p1 = W[i,0:2],   W[i+1,0:2]
            m0, m1 = tangents[i], tangents[i+1]
            L = np.linalg.norm(p1 - p0)
            T = (L / self.v_des) if self.v_des > 0 else 0.0
            # Hermite coefficients a0..a5
            a0 = p0
            a1 = m0 * T
            a2 = np.zeros(2)
            RHS = np.vstack([
                p1      - (a0 + a1 + a2),
                m1 * T  - (        a1 + 2*a2),
                np.zeros(2) - (      2*a2)
            ])  # shape = (3,2)
            # solve for a3,a4,a5
            a3, a4, a5 = np.linalg.solve(M, RHS)
            # sample
            if T > 0:
                t_samples = np.arange(0.0, T, self.dt)
            else:
                t_samples = np.array([0.0])
            for tt in t_samples:
                s = tt / T if T > 0 else 0.0
                p = (a0
                     + a1 * s
                     + a2 * s**2
                     + a3 * s**3
                     + a4 * s**4
                     + a5 * s**5)
                pts_out.append(p.tolist())
                t_out.append(t_accum + tt)
            t_accum += T
        # append very last waypoint
        pts_out.append(W[-1,0:2].tolist())
        t_out.append(t_accum)
        return np.array(pts_out), np.array(t_out)


class LogitudinalPlanner(Component):
    """Follows the given route. Brakes if the ego-vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = settings.get("run.drive.planning.motion_planning.largs.acceleration", 5)
        self.desired_speed = settings.get("run.drive.planning.motion_planning.largs.desired_speed", 2.0)
        self.deceleration = settings.get("run.drive.planning.motion_planning.largs.deceleration", 2.0)
        self.emergency_brake = settings.get("run.drive.planning.motion_planning.largs.emergency_brake", 8.0)
    
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
        should_brake = any(
            r.type == EntityRelationEnum.STOPPING_AT and r.obj1 == ''
            for r in state.relations
        )
        should_decelerate = any(
            r.type == EntityRelationEnum.YIELDING and r.obj1 == ''
            for r in state.relations
        ) if should_brake == False else False
        should_accelerate = (not should_brake and not should_decelerate and curr_v < self.desired_speed)
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: stay_braking =", stay_braking)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_brake =", should_brake)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_accelerate =", should_accelerate)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_decelerate =", should_decelerate)
        if stay_braking:
            traj = longitudinal_brake(route_with_lookahead, 0.0, 0.0, 0.0, planner_type="standard")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake (stay braking).")
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.emergency_brake, curr_v, planner_type="standard")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake.")
        elif should_decelerate:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v, planner_type="standard")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake.")
        elif should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration,
                                     self.deceleration, self.desired_speed, curr_v, planner_type="standard")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_plan (accelerate).")
        else:
            # Maintain current speed if not accelerating or braking.
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v, planner_type="standard")
            if DEBUG:
                print(
                    "[DEBUG] YieldTrajectoryPlanner.update: Maintaining current speed with longitudinal_plan (0 accel).")
        self.t_last = t
        if DEBUG:
            print('[DEBUG] Current Velocity of the Car: LOOK!', curr_v, self.desired_speed)
            print("[DEBUG] YieldTrajectoryPlanner.update: Returning trajectory with", len(traj.points), "points.")
        return traj

class QuinticSplineScurveTrajectoryPlanner(Component):
    """Follows the given route. Brakes if the ego-vehicle must yield
    (e.g. to a pedestrian) or if the end of the route is near; otherwise,
    it accelerates (or cruises) toward a desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = settings.get("run.drive.planning.motion_planning.largs.acceleration", 5)
        self.desired_speed = settings.get("run.drive.planning.motion_planning.largs.desired_speed", 2.0)
        self.deceleration = settings.get("run.drive.planning.motion_planning.largs.deceleration", 2.0)
        self.emergency_brake = settings.get("run.drive.planning.motion_planning.largs.emergency_brake", 8.0)
        self.lookahead_dist = 10.0
        self.spline_dt = 0.02
        self._spline = QuinticHermiteSplinePlanner(v_des=self.desired_speed, dt=self.spline_dt)
    
    def state_inputs(self):
        return ['all']
    
    def state_outputs(self) -> List[str]:
        return ['trajectory']
    
    # Technically doesn't need to be specified since base class has same implementation
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
        # route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)
        route_with_lookahead = route.trim(self.route_progress, self.route_progress + self.lookahead_dist)
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: Route Lookahead =", route_with_lookahead)
        raw_points = [list(p) for p in route_with_lookahead.points]
        #Generate spline-smoothed path
        spline_pts, _ = self._spline.build(raw_points)
        spline_path = Path(frame=route_with_lookahead.frame, points=spline_pts.tolist())
        print("[DEBUG] state", state.relations)
        # Check whether any yield relations (e.g. due to pedestrians) require braking.
        stay_braking = False
        pointSet = set()
        for i in range(len(route_with_lookahead.points)):
            if tuple(route_with_lookahead.points[i]) in pointSet:
                stay_braking = True
                break
            pointSet.add(tuple(route_with_lookahead.points[i]))
        should_brake = any(
            r.type == EntityRelationEnum.STOPPING_AT and r.obj1 == ''
            for r in state.relations
        )
        should_decelerate = any(
            r.type == EntityRelationEnum.YIELDING and r.obj1 == ''
            for r in state.relations
        ) if should_brake == False else False
        should_accelerate = (not should_brake and not should_decelerate and curr_v < self.desired_speed)
        if DEBUG:
            print("[DEBUG] YieldTrajectoryPlanner.update: stay_braking =", stay_braking)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_brake =", should_brake)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_accelerate =", should_accelerate)
            print("[DEBUG] YieldTrajectoryPlanner.update: should_decelerate =", should_decelerate)
        if stay_braking:
            traj = longitudinal_brake(spline_path, 0.0, 0.0, 0.0, planner_type="scurve")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake (stay braking).")
        elif should_brake:
            traj = longitudinal_brake(spline_path, self.emergency_brake, curr_v, planner_type="scurve")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake.")
        elif should_decelerate:
            traj = longitudinal_brake(spline_path, self.deceleration, curr_v, planner_type="scurve")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_brake.")
        elif should_accelerate:
            traj = longitudinal_plan(spline_path, self.acceleration,
                                     self.deceleration, self.desired_speed, curr_v, planner_type="scurve")
            if DEBUG:
                print("[DEBUG] YieldTrajectoryPlanner.update: Using longitudinal_plan (accelerate).")
        else:
            # Maintain current speed if not accelerating or braking.
            traj = longitudinal_plan(spline_path, 0.0, self.deceleration, self.desired_speed, curr_v, planner_type="scurve")
            if DEBUG:
                print(
                    "[DEBUG] YieldTrajectoryPlanner.update: Maintaining current speed with longitudinal_plan (0 accel).")
        self.t_last = t
        if DEBUG:
            print('[DEBUG] Current Velocity of the Car: LOOK!', curr_v, self.desired_speed)
            print("[DEBUG] YieldTrajectoryPlanner.update: Returning trajectory with", len(traj.points), "points.")
        return traj
