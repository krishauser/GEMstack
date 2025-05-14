from ...utils import settings
from ...state.vehicle import VehicleState,VehicleGearEnum
from ...state.trajectory import Trajectory, Path
from ...knowledge.vehicle.geometry import front2steer
from ..component import Component
import numpy as np
import casadi
import math

###########################
#    Bo-Hao Wu's code     #
###########################

class MPCController(object):
    """Model Predictive Controller for trajectory tracking."""
    def __init__(self, T=None, dt=None, desired_speed=None):
        self.T = T if T is not None else settings.get('control.mpc.horizon', 30)
        self.dt = dt if dt is not None else settings.get('control.mpc.dt', 0.2)
        self.L = settings.get('vehicle.geometry.wheelbase')
        self.v_bounds = [-settings.get('vehicle.limits.max_reverse_speed'), settings.get('vehicle.limits.max_speed')]
        self.delta_bounds = [settings.get('vehicle.geometry.min_wheel_angle'),settings.get('vehicle.geometry.max_wheel_angle')]
        self.delta_rate_bounds = [-0.4, 0.4] # Predefined front wheel rate limit to simplify computation
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.a_bounds = [-settings.get('vehicle.limits.max_deceleration'), settings.get('vehicle.limits.max_acceleration')]
        self.switch_gear = settings.get('control.mpc.switch_gear', False)

        if desired_speed is not None:
            self.desired_speed_source = desired_speed
        else:
            self.desired_speed_source = settings.get('control.pure_pursuit.desired_speed',"path") 
        self.desired_speed = self.desired_speed_source if isinstance(self.desired_speed_source,(int,float)) else None
        
        self.prev_x = None  # Previous state trajectory
        self.prev_u = None  # Previous control inputs
        self.path = None
        self.path_arg = None
        self.iter = 0

    def set_path(self, path : Path):
        if path == self.path_arg:
            return
        self.path_arg = path
        self.iter = 0
        if len(path.points[0]) > 2:
            path = path.get_dims([0,1])
        self.current_path_parameter = 0.0
        # self.prev_u = None
        # self.prev_x = None
        if not isinstance(path,Trajectory):
            if self.desired_speed_source in ['path', 'trajectory']:
                print("rase")
                raise ValueError("MPC: Provided path has no timing. Either provide a Trajectory or set a constant desired_speed.")
            self.path = path.arc_length_parameterize(self.desired_speed)
            self.current_traj_parameter = 0.0
        else:
            self.path = path
            self.current_traj_parameter = self.path.domain()[0]
    
    def clip_reverse_path_with_times(self, points, times):
        def angle_between(p1, p2):
            delta = p2 - p1
            return np.arctan2(delta[1], delta[0])

        points = np.array(points)
        times = np.array(times)

        if len(points) < 3:
            return points, times

        ref_angle = angle_between(points[0], points[1])
        clipped_points = [points[0]]
        clipped_times = [times[0]]

        for i in range(1, len(points)):
            angle = angle_between(points[i-1], points[i])
            angle_diff = np.abs((angle - ref_angle + np.pi) % (2 * np.pi) - np.pi)
            if angle_diff > np.pi / 2:
                break
            clipped_points.append(points[i])
            clipped_times.append(times[i])

        return np.array(clipped_points), np.array(clipped_times)

    def compute(self, state: VehicleState, component: Component = None):
        """Compute the control commands using MPC."""
        if self.iter < 10 and self.prev_x is not None:
            self.iter += 1
            # return float(self.prev_u[self.iter, 0]), float(self.prev_x[self.iter + 1, 4])
        self.iter = 0

        if self.path is not None:
            if self.path.frame != state.pose.frame:
                print("Transforming trajectory from",self.path.frame.name,"to",state.pose.frame.name)
                self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
        
        x0 = np.array([state.pose.x, state.pose.y, state.pose.yaw % (2 * np.pi), state.v, state.front_wheel_angle])
        # print(x0)

        if self.switch_gear:
            closest_dist,closest_time = self.path.closest_point_local((x0[0], x0[1]),[self.current_traj_parameter-0.0,self.current_traj_parameter+1.0], True)
        else:
            closest_dist,closest_time = self.path.closest_point_local((x0[0], x0[1]),[self.current_traj_parameter-0.0,self.current_traj_parameter+5.0], True)
        self.current_traj_parameter = closest_time

        times = self.path.times
        points = self.path.points
        j = math.floor(self.current_path_parameter)
        while j < len(times) - 1 and times[j+1] < closest_time:
            j += 1
        self.current_path_parameter = j

        if self.switch_gear:
            # Slice path from j
            sliced_points = points[j:]
            sliced_times = times[j:]

            # Clip reversed part
            new_points, new_times = self.clip_reverse_path_with_times(sliced_points, sliced_times)
        else:
            new_points = points
            new_times = times

        # Interpolate trajectory points to match MPC time horizon
        traj_points = []
        j = 0
        for i in range(self.T + 1):
            t_query = closest_time + i * self.dt
            if t_query <= new_times[0]:
                traj_points.append(new_points[0])
            elif t_query >= new_times[-1]:
                traj_points.append(new_points[-1])
            else:
                while j < len(new_times) - 2 and new_times[j+1] < t_query:
                    j += 1
                alpha = (t_query - new_times[j]) / (new_times[j+1] - new_times[j])
                pt = (1 - alpha) * np.array(new_points[j]) + alpha * np.array(new_points[j+1])
                traj_points.append(pt)
        
        # print("trajectory points: ", traj_points)
        # traj_str = ", ".join(
        #     [f"np.array([{round(p[0], 8)}, {round(p[1], 8)}])" for p in traj_points]
        # )
        # print(f"traj_points = [{traj_str}]")

        # Apply gradually decreasing offset correction
        cur_offset = np.array([state.pose.x, state.pose.y]) - np.array(traj_points[0][0:2])
        overcorrection_guard = 5
        for i in range(len(traj_points)):
            s = i / (len(traj_points) + overcorrection_guard)  # Normalized [0, 1] and add some offset to prevent overcorrection
            decay_ratio = (1 - s) ** 1      # linear decay on each iteration to perform quadratic offset correction over time
            
            # Get direction of trajectory at this point
            if i < len(traj_points) - 1:
                tangent = np.array(traj_points[i+1]) - np.array(traj_points[i])
            else:
                tangent = np.array(traj_points[i]) - np.array(traj_points[i-1])
            
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm == 0:
                continue  # Skip degenerate points

            tangent /= tangent_norm
            normal = np.array([-tangent[1], tangent[0]])  # Rotate tangent by 90° to get normal

            # Project initial offset onto this normal
            offset_proj = np.dot(cur_offset, normal) * normal

            # Apply decaying lateral offset
            traj_points[i] = np.array(traj_points[i]) + decay_ratio * offset_proj

        # print("trajectory points after correction: ", traj_points)
        # traj_corrected_str = ", ".join(
        #     [f"np.array([{round(p[0], 8)}, {round(p[1], 8)}])" for p in traj_points]
        # )
        # print(f"traj_corrected = [{traj_corrected_str}]")

        # Compute target angles
        target_angles = []
        prev_angle = x0[2]

        for i in range(1, len(traj_points)):
            dx = traj_points[i][0] - traj_points[i-1][0]
            dy = traj_points[i][1] - traj_points[i-1][1]
            raw_angle = np.arctan2(dy, dx) % (2 * np.pi)

            # Convert raw_angle to be close to prev_angle (modulo 2π), within ±0.5π
            # Compute smallest angular difference
            delta = ((raw_angle - prev_angle + np.pi) % (2 * np.pi)) - np.pi

            # Clip delta to ±0.5π
            if delta > 0.5 * np.pi:
                delta -= np.pi
            elif delta < -0.5 * np.pi:
                delta += np.pi

            corrected_angle = prev_angle + delta
            assert abs(corrected_angle - prev_angle) < 0.5 * np.pi
            target_angles.append(corrected_angle)
            prev_angle = corrected_angle

        # print(target_angles)

        # delta_desired = []
        # for i in range(1, len(target_angles)):
        #     ds = np.linalg.norm(np.array(traj_points[i]) - np.array(traj_points[i-1]))
        #     dtheta = target_angles[i] - target_angles[i-1]
        #     dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
            
        #     curvature = dtheta / ds if ds > 1e-6 else 0.0
        #     delta_ref = np.arctan(self.L * curvature)
        #     delta_desired.append(delta_ref)

        # Optimization setup
        opti = casadi.Opti()
        x = opti.variable(self.T+1, 5)  # [x, y, theta, v, delta]
        u = opti.variable(self.T, 2)    # [a, delta_dot]

        def model(x, u):
            """Dynamic model of the vehicle using kinematic bicycle model"""
            px, py, theta, v, delta = x[0], x[1], x[2], x[3], x[4]
            a, delta_dot = u[0], u[1]
            dx = v * casadi.cos(theta)
            dy = v * casadi.sin(theta)
            dtheta = v * casadi.tan(delta) / self.L
            dv = a
            ddelta = delta_dot
            return casadi.vertcat(dx, dy, dtheta, dv, ddelta)

        obj = 0
        for t in range(self.T):
            # Vehicle dynamics
            x_next = x[t,:] + self.dt * model(x[t,:], u[t,:]).T
            opti.subject_to(x[t+1,:] == x_next)
            target = traj_points[t+1]
            # Cost function
            weight = 10 if t < 3 else 1  # Larger weight for the first three points
            obj += weight * casadi.sumsqr(x[t + 1, 0:2] - casadi.reshape(target[0:2], 1, 2))
            
            # Control effort penalty
            obj += 0.1 * casadi.sumsqr(u[t, :])

            # Heading angle error
            theta_error = x[t + 1, 2] - target_angles[t]
            obj += 1 * weight * casadi.sumsqr(theta_error)

            # Front wheel angle error
            # if t != self.T - 1:
            #     delta_error = x[t + 1, 4] - delta_desired[t]
            #     obj += 0.1 * weight * casadi.sumsqr(delta_error)

        # Initial condition
        opti.subject_to(x[0, :] == casadi.reshape(x0, 1, 5))

        # Constraints
        for t in range(self.T):
            opti.subject_to(opti.bounded(self.a_bounds[0], u[t,0], self.a_bounds[1]))            # a
            opti.subject_to(opti.bounded(self.delta_rate_bounds[0], u[t,1], self.delta_rate_bounds[1]))                # delta_dot
            opti.subject_to(opti.bounded(self.delta_bounds[0], x[t+1,4], self.delta_bounds[1]))  # delta as state
            opti.subject_to(opti.bounded(self.v_bounds[0], x[t+1,3], self.v_bounds[1]))          # v

        # Initial guess
        if self.prev_x is not None and self.prev_u is not None:
            if len(self.prev_x) == self.T+1 and len(self.prev_u) == self.T:
                opti.set_initial(x, np.vstack((self.prev_x[1:], self.prev_x[-1])))
                opti.set_initial(u, np.vstack((self.prev_u[1:], self.prev_u[-1])))

        # Solver settings
        opti.minimize(obj)
        p_opts = {"expand": True, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        s_opts = {"max_iter": 150}

        opti.solver("ipopt", p_opts, s_opts)

        try:
            # Solve the optimization problem
            sol = opti.solve()
            acc = float(sol.value(u[0,0]))
            # delta = float(sol.value(u[0,1]))
            delta = float(sol.value(x[1,4]))
            self.prev_x = sol.value(x)
            self.prev_u = sol.value(u)
            if component is not None:
                component.debug("mpc/accel", acc)
                component.debug("mpc/delta", delta)
                component.debug("mpc/closest_time", closest_time)
                component.debug("mpc/state_x", state.pose.x)
                component.debug("mpc/state_y", state.pose.y)
                component.debug("mpc/state_yaw", state.pose.yaw)
                component.debug("mpc/target_x", self.path.points[self.current_path_parameter][0])
                component.debug("mpc/target_y", self.path.points[self.current_path_parameter][1])
                component.debug("mpc/target_theta", target_angles[0])

            # xy_array = [f"np.array([{round(self.prev_x[t,0],8)}, {round(self.prev_x[t,1],8)}])" for t in range(self.prev_x.shape[0])]
            # print("mpc = [", ", ".join(xy_array), "]")

            print(self.current_path_parameter)
            print(acc, delta)
            # print(self.prev_u[0])
            # print(self.prev_x[0])
            # print(self.prev_x[1])
            # print(self.prev_u)
            # print(x0[4])
            # print(delta)
            return acc, delta
        except RuntimeError:
            # Handle optimization failure
            print("MPC optimization failed.")
            return 0.0, 0.0


class MPCTrajectoryTracker(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.mpc = MPCController(**args)
        self.vehicle_interface = vehicle_interface
        self.counter = 0

    def rate(self):
        return 5.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.mpc.set_path(trajectory)
        accel, delta = self.mpc.compute(vehicle, self)
        
        # Clip acceleration and steering angle to vehicle limits
        accel = np.clip(accel, self.mpc.a_bounds[0], self.mpc.a_bounds[1])
        delta = np.clip(delta, self.mpc.delta_bounds[0], self.mpc.delta_bounds[1])

        # Convert delta to steering angle
        steering_angle = np.clip(front2steer(delta), 
            self.mpc.steering_angle_range[0], self.mpc.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel, steering_angle, vehicle))

    def healthy(self):
        return self.mpc.path is not None
