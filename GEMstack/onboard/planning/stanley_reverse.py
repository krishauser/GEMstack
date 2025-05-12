import numpy as np
from math import sin, cos, atan2

from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.geometry import front2steer
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory
from ..interface.gem import GEMVehicleCommand
from ..component import Component

#####################################
# 1. Angle normalization
#####################################
def normalise_angle(angle):
    """Wraps an angle to the [-pi, pi] range."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

#####################################
# 2. Stanley-based controller with longitudinal PID
#####################################
class Stanley(object):

    def __init__(
        self,
        control_gain=None,
        softening_gain=None,
        desired_speed=None
    ):
        """
        :param control_gain:       Stanley lateral control gain k (lowered from the default to reduce overshoot).
        :param softening_gain:     Softening gain k_soft.
        :param desired_speed:      Desired speed (float) or 'path'/'trajectory'.
        """
        self.k = control_gain if control_gain is not None else settings.get('control.stanley.control_gain')
        self.k_soft = softening_gain if softening_gain is not None else settings.get('control.stanley.softening_gain')

        self.max_steer = settings.get('vehicle.geometry.max_wheel_angle')
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')

        self.speed_limit = settings.get('vehicle.limits.max_speed')
        self.reverse_speed_limit = settings.get('vehicle.limits.max_reverse_speed')
        self.max_accel = settings.get('vehicle.limits.max_acceleration')
        self.max_decel = settings.get('vehicle.limits.max_deceleration')

        p = settings.get('control.longitudinal_control.pid_p')
        d = settings.get('control.longitudinal_control.pid_d')
        i = settings.get('control.longitudinal_control.pid_i')
        self.pid_speed = PID(p, d, i, windup_limit=20)
        
        if desired_speed is not None:
            self.desired_speed_source = desired_speed
        else:
            self.desired_speed_source = settings.get('control.stanley.desired_speed', 'path')

        if isinstance(self.desired_speed_source, (int, float)):
            self.desired_speed = self.desired_speed_source
        else:
            self.desired_speed = None

        self.path_arg = None
        self.path = None
        self.trajectory = None
        self.current_path_parameter = 0.0
        self.current_traj_parameter = 0.0
        self.t_last = None
        self.reverse = None
        self.sharp_turn = False

    def set_path(self, path: Path):
        if path == self.path_arg:
            return
        self.path_arg = path

        if len(path.points[0]) > 2:
            path = path.get_dims([0,1])

        if not isinstance(path, Trajectory):
            self.path = path.arc_length_parameterize()
            self.trajectory = None
            self.current_traj_parameter = 0.0
            if self.desired_speed_source in ['path', 'trajectory']:
                raise ValueError("Stanley: Provided path has no timing. Either provide a Trajectory or set a constant desired_speed.")
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]

        self.current_path_parameter = self.path.domain()[0]

    def _find_front_axle_position(self, x, y, yaw):
        """Compute front-axle world position from the center/rear and yaw."""
        fx = x + self.wheelbase * cos(yaw)
        fy = y + self.wheelbase * sin(yaw)
        return fx, fy

    def _find_rear_axle_position(self, x, y, yaw):
        """Compute rear-axle world position from the vehicle's reference point and yaw."""
        rx = x - self.wheelbase * cos(yaw)
        ry = y - self.wheelbase * sin(yaw)
        return rx, ry

    def initialize_state_and_direction(self, state: VehicleState):
        if self.path is None:
            raise ValueError("Stanley: Path must be set before initializing state and direction.")
        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        fx, fy = self._find_front_axle_position(curr_x, curr_y, curr_yaw)
        path_domain_start, path_domain_end = self.path.domain()
        search_end = min(path_domain_end, path_domain_start + 5.0)
        search_domain_start = [path_domain_start, search_end]
        _, closest_param_at_start = self.path.closest_point_local((fx, fy), search_domain_start)
        tangent_at_start = self.path.eval_tangent(closest_param_at_start)
        initial_reverse = False
        if np.linalg.norm(tangent_at_start) > 1e-6:
            path_yaw_at_start = atan2(tangent_at_start[1], tangent_at_start[0])
            heading_diff = normalise_angle(path_yaw_at_start - curr_yaw)
            initial_reverse = abs(heading_diff) > (np.pi / 2.0)
        self.pid_speed.reset()
        self.current_path_parameter = closest_param_at_start
        if self.trajectory:
            self.current_traj_parameter = self.trajectory.domain()[0]
        return initial_reverse

    def is_target_behind_vehicle(self, vehicle_pose, target_point_coords):
        curr_x = vehicle_pose.x
        curr_y = vehicle_pose.y
        curr_yaw = vehicle_pose.yaw
        if curr_yaw is None:
            return False
        target_x = target_point_coords[0]
        target_y = target_point_coords[1]
        vec_x = target_x - curr_x
        vec_y = target_y - curr_y
        if abs(vec_x) < 1e-6 and abs(vec_y) < 1e-6:
            return False
        heading_x = cos(curr_yaw)
        heading_y = sin(curr_yaw)
        dot_product = vec_x * heading_x + vec_y * heading_y
        return (dot_product < 0)

    def _check_sharp_turn_ahead(self, lookahead_s=3.0, threshold_angle=np.pi/2.0, num_steps=4):
        if not self.path:
            return False

        path = self.path
        current_s = self.current_path_parameter
        domain_start, domain_end = path.domain()

        if current_s >= domain_end - 1e-3:
            return False

        step_s = lookahead_s / num_steps
        s_prev = current_s
        
        try:
            tangent_prev = path.eval_tangent(s_prev)
            if np.linalg.norm(tangent_prev) < 1e-6:
                s_prev_adjusted = min(s_prev + step_s / 2, domain_end)
                if s_prev_adjusted <= s_prev:
                    return False
                tangent_prev = path.eval_tangent(s_prev_adjusted)
                if np.linalg.norm(tangent_prev) < 1e-6:
                    return False
                s_prev = s_prev_adjusted
                
            angle_prev = atan2(tangent_prev[1], tangent_prev[0])

        except Exception as e:
            return False

        for i in range(num_steps):
            s_next = s_prev + step_s
            s_next = min(s_next, domain_end)

            if s_next <= s_prev + 1e-6:
                break

            try:
                tangent_next = path.eval_tangent(s_next)
                if np.linalg.norm(tangent_next) < 1e-6:
                    s_prev = s_next
                    continue 

                angle_next = atan2(tangent_next[1], tangent_next[0])
            except Exception as e:
                break

            angle_change = abs(normalise_angle(angle_next - angle_prev))

            if angle_change > threshold_angle:
                return True

            angle_prev = angle_next
            s_prev = s_next
        return False

    def compute(self, state: VehicleState, component: Component = None):
        t = state.pose.t
        if self.t_last is None:
            self.t_last = t
            dt = 0
        else:
            dt = t - self.t_last

        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        speed = state.v

        if self.path is None:
            if component:
                component.debug_event("No path provided to Stanley controller. Doing nothing.")
            self.t_last = t
            self.pid_speed.reset()
            return (0.0, 0.0)

        if self.path.frame != state.pose.frame:
            if component:
                component.debug_event(f"Transforming path from {self.path.frame.name} to {state.pose.frame.name}")
            self.path = transforms.transform_path(self.path, self.path.frame, state.pose.frame, current_pose=state.pose)

        if self.trajectory and self.trajectory.frame != state.pose.frame:
            if component:
                component.debug_event(f"Transforming trajectory from {self.trajectory.frame.name} to {state.pose.frame.name}")
            self.trajectory = transforms.transform_path(self.trajectory, self.trajectory.frame, state.pose.frame, current_pose=state.pose)


        if self.reverse is None:
            self.reverse = self.initialize_state_and_direction(state)

        if self.reverse:
            fx, fy = self._find_rear_axle_position(curr_x, curr_y, curr_yaw)
        else:
            fx, fy = self._find_front_axle_position(curr_x, curr_y, curr_yaw)
        search_start = self.current_path_parameter
        search_end   = self.current_path_parameter + 5.0
        closest_dist, closest_parameter = self.path.closest_point_local((fx, fy), [search_start, search_end])
        self.current_path_parameter = closest_parameter

        target_x, target_y = self.path.eval(self.current_path_parameter)
        tangent = self.path.eval_tangent(self.current_path_parameter)
        path_yaw = atan2(tangent[1], tangent[0])

        dx = fx - target_x
        dy = fy - target_y

        is_sharp_turn_ahead = self._check_sharp_turn_ahead(
            lookahead_s=2.0,     
            threshold_angle=np.pi/2.0
        )
        if is_sharp_turn_ahead and not self.sharp_turn:
            self.sharp_turn = True
        use_reverse = self.is_target_behind_vehicle(state.pose, (target_x, target_y))
        if use_reverse and self.sharp_turn and not self.reverse:
            self.reverse = True
            self.sharp_turn = False
        
        if self.reverse:
            cross_track_error = dx * (-tangent[1]) + dy * tangent[0]
            self.k += self.k
        else:
            left_vec = np.array([sin(curr_yaw), -cos(curr_yaw)])
            cross_track_error = np.sign(np.dot(np.array([dx, dy]), left_vec)) * closest_dist

        yaw_error = normalise_angle(path_yaw - curr_yaw)

        desired_speed = abs(self.desired_speed)
        feedforward_accel = 0.0

        if self.reverse:
            heading_term = -yaw_error
            cross_term_input = self.k * (-cross_track_error) / (self.k_soft + abs(speed))
            cross_term = atan2(cross_term_input, 1.0)
            desired_steering_angle = heading_term + cross_term

            if self.trajectory and self.desired_speed_source in ['path', 'trajectory']:
                current_trajectory_param_for_eval = self.current_path_parameter
                if hasattr(self.trajectory, 'parameter_to_time'):
                    current_trajectory_param_for_eval = self.trajectory.parameter_to_time(self.current_path_parameter)

                if self.trajectory is not None:
                    deriv = self.trajectory.eval_derivative(current_trajectory_param_for_eval)
                    path_speed_magnitude = np.linalg.norm(deriv)
                else:
                    path_speed_magnitude = 0.0

                desired_speed = min(path_speed_magnitude, self.reverse_speed_limit)

            desired_speed = desired_speed * -1.0

            is_at_start_backward = self.current_path_parameter <= self.path.domain()[0] + 1e-3

            if is_at_start_backward:
                desired_speed = 0.0
                feedforward_accel = -2.0 * -1.0

            else:
                difference_dt = 0.1
                current_speed_abs = abs(speed)
                if current_speed_abs < 0.1: estimated_path_step = 0.1 * difference_dt * -1.0
                else: estimated_path_step = current_speed_abs * difference_dt * -1.0

                future_parameter = self.current_path_parameter + estimated_path_step

                future_parameter = np.clip(future_parameter, self.path.domain()[0], self.path.domain()[-1])

                future_trajectory_param_for_eval = future_parameter
                if hasattr(self.trajectory, 'parameter_to_time'):
                    future_trajectory_param_for_eval = self.trajectory.parameter_to_time(future_parameter)

                if self.trajectory is not None:
                    future_deriv = self.trajectory.eval_derivative(future_trajectory_param_for_eval)
                    next_path_speed_magnitude = min(np.linalg.norm(future_deriv), self.speed_limit)
                else:
                    next_path_speed_magnitude = 0.0

                next_desired_speed = next_path_speed_magnitude * -1.0

                feedforward_accel = (next_desired_speed - desired_speed) / difference_dt
                feedforward_accel = np.clip(feedforward_accel, -self.max_decel, self.max_accel)

            desired_speed_magnitude_slowed = abs(desired_speed) * np.exp(-abs(cross_track_error) * 0.6)
            desired_speed = desired_speed_magnitude_slowed * -1.0 if desired_speed_magnitude_slowed != 0 else 0.0

        else:
            cross_term = atan2(self.k * cross_track_error, self.k_soft + speed)
            desired_steering_angle = yaw_error + cross_term

            if self.trajectory and self.desired_speed_source in ['path', 'trajectory']:
                if len(self.trajectory.points) < 2 or self.current_path_parameter >= self.path.domain()[1]:
                    if component:
                        component.debug_event("Stanley: Past the end of trajectory, stopping.")
                    desired_speed = 0.0
                    feedforward_accel = -2.0
                else:
                    if self.desired_speed_source == 'path':
                        current_trajectory_time = self.trajectory.parameter_to_time(self.current_path_parameter)
                    else:
                        self.current_traj_parameter += dt
                        current_trajectory_time = self.current_traj_parameter

                    deriv = self.trajectory.eval_derivative(current_trajectory_time)
                    desired_speed = min(np.linalg.norm(deriv), self.speed_limit)

                    difference_dt = 0.1
                    future_t = current_trajectory_time + difference_dt
                    if future_t > self.trajectory.domain()[1]:
                        future_t = self.trajectory.domain()[1]
                    future_deriv = self.trajectory.eval_derivative(future_t)
                    next_desired_speed = min(np.linalg.norm(future_deriv), self.speed_limit)
                    feedforward_accel = (next_desired_speed - desired_speed) / difference_dt
                    feedforward_accel = np.clip(feedforward_accel, -self.max_decel, self.max_accel)
            else:
                if desired_speed is None:
                    desired_speed = 4.0

                desired_speed *= np.exp(-abs(cross_track_error) * 0.6)

        if abs(desired_speed) > self.speed_limit:
            if self.reverse:
                desired_speed = self.speed_limit * -1.0 if desired_speed != 0 else 0.0
            else:
                desired_speed = self.speed_limit

        speed_error = desired_speed - speed
        output_accel = self.pid_speed.advance(e=speed_error, t=t, feedforward_term=feedforward_accel)

        if output_accel > self.max_accel:
            output_accel = self.max_accel
        elif output_accel < -self.max_decel:
            output_accel = -self.max_decel

        if desired_speed == 0 and abs(speed) < 0.01 and output_accel < 0:
            output_accel = 0.0

        if component is not None:
            component.debug('curr pt',(curr_x,curr_y))
            component.debug("desired_x",target_x)
            component.debug("desired_y",target_y)
            component.debug("Stanley: path param", self.current_path_parameter)
            component.debug("Stanley: crosstrack dist", closest_dist)
            component.debug("crosstrack error", cross_track_error)
            component.debug("Stanley: yaw_error", yaw_error)
            component.debug('steering_angle', desired_steering_angle)
            component.debug("Stanley: desired_speed (m/s)", desired_speed)
            component.debug("Stanley: feedforward_accel (m/s^2)", feedforward_accel)
            component.debug("Stanley: output_accel (m/s^2)", output_accel)

        self.t_last = t
        return (output_accel, desired_steering_angle)

#####################################
# 3. Tracker component
#####################################
class StanleyTrajectoryTracker(Component):

    def __init__(self, vehicle_interface=None, **kwargs):
        self.stanley = Stanley(**kwargs)
        self.vehicle_interface = vehicle_interface
        
    def rate(self):
        return 50.0

    def state_inputs(self):
        return ["vehicle", "trajectory"]

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.stanley.set_path(trajectory)

        accel, f_delta = self.stanley.compute(vehicle, self)

        steering_angle = front2steer(f_delta)
        steering_angle = np.clip(	
            steering_angle,
            settings.get('vehicle.geometry.min_steering_angle', -0.5),
            settings.get('vehicle.geometry.max_steering_angle',  0.5)
        )

        cmd = self.vehicle_interface.simple_command(accel, steering_angle, vehicle)
        self.vehicle_interface.send_command(cmd)

    def healthy(self):
        """Optional: check if the controller has a valid path."""
        return self.stanley.path is not None
