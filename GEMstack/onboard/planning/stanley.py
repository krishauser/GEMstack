import numpy as np
from math import sin, cos, atan2

# These imports match your existing project structure
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
    """
    A Stanley controller that handles lateral control (steering)
    plus a basic longitudinal control (PID + optional feedforward).
    It has been modified to reduce oscillations by:
      1) Lower gains
      2) Steering damping
      3) Low-pass filter on steering
      4) Gentler speed logic when cornering
    """

    def __init__(
        self,
        control_gain=None,
        softening_gain=None,
        yaw_rate_gain=None,
        steering_damp_gain=None,
        desired_speed=None
    ):
        """
        :param control_gain:       Stanley lateral control gain k (lowered from the default to reduce overshoot).
        :param softening_gain:     Softening gain k_soft.
        :param yaw_rate_gain:      Yaw-rate gain k_yaw_rate (helps reduce oscillations).
        :param steering_damp_gain: Steering damping gain k_damp_steer.
        :param desired_speed:      Desired speed (float) or 'path'/'trajectory'.
        """

        # 1) Lower Gains
        #    Reduced from 2.5 to 1.0 by default, to mitigate oscillations
        self.k = control_gain if control_gain is not None else settings.get('control.stanley.control_gain', 1.0)
        self.k_soft = softening_gain if softening_gain is not None else settings.get('control.stanley.softening_gain', 1.0)
        self.k_yaw_rate = yaw_rate_gain if yaw_rate_gain is not None else settings.get('control.stanley.yaw_rate_gain', 0.0)
        self.k_damp_steer = steering_damp_gain if steering_damp_gain is not None else settings.get('control.stanley.steering_damp_gain', 0.0)

        # Typically, this is the max front-wheel steering angle in radians
        self.max_steer = settings.get('vehicle.geometry.max_wheel_angle', np.deg2rad(24))
        self.wheelbase = settings.get('vehicle.geometry.wheelbase', 2.0)

        # Basic longitudinal constraints
        self.speed_limit = settings.get('vehicle.limits.max_speed', 10.0)
        self.max_accel   = settings.get('vehicle.limits.max_acceleration', 2.0)
        self.max_decel   = settings.get('vehicle.limits.max_deceleration', 2.0)

        # PID for longitudinal speed tracking
        p = settings.get('control.longitudinal_control.pid_p', 0.5)
        d = settings.get('control.longitudinal_control.pid_d', 0.0)
        i = settings.get('control.longitudinal_control.pid_i', 0.1)
        self.pid_speed = PID(p, d, i, windup_limit=20)

        # Speed source: numeric or derived from path/trajectory
        if desired_speed is not None:
            self.desired_speed_source = desired_speed
        else:
            self.desired_speed_source = settings.get('control.stanley.desired_speed', 'path')

        if isinstance(self.desired_speed_source, (int, float)):
            self.desired_speed = self.desired_speed_source
        else:
            self.desired_speed = None

        # For path/trajectory
        self.path_arg = None
        self.path = None
        self.trajectory = None
        self.current_path_parameter = 0.0
        self.current_traj_parameter = 0.0
        self.t_last = None

        # 2) Steering damping memory
        self.prev_steering_angle = 0.0

        # 3) Low-pass filter memory: final front-wheel angle
        self.prev_front_wheel_angle = 0.0

    def set_path(self, path: Path):
        """Sets the path or trajectory to track."""
        if path == self.path_arg:
            return
        self.path_arg = path

        # If the path has more than 2D, reduce to (x,y)
        if len(path.points[0]) > 2:
            path = path.get_dims([0,1])

        # If no timing info, we can't rely on 'path'/'trajectory' speed
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

        self.current_path_parameter = 0.0

    def _find_front_axle_position(self, x, y, yaw):
        """Compute front-axle world position from the center/rear and yaw."""
        fx = x + self.wheelbase * cos(yaw)
        fy = y + self.wheelbase * sin(yaw)
        return fx, fy

    def compute(self, state: VehicleState, component: Component = None):
        """Compute the control outputs: (longitudinal acceleration, front wheel angle)."""
        t = state.pose.t
        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        # Current vehicle states
        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        speed = state.v

        if self.path is None:
            if component:
                component.debug_event("No path provided to Stanley controller. Doing nothing.")
            return (0.0, 0.0)

        # Ensure same frame
        if self.path.frame != state.pose.frame:
            if component:
                component.debug_event(f"Transforming path from {self.path.frame.name} to {state.pose.frame.name}")
            self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)

        if self.trajectory and self.trajectory.frame != state.pose.frame:
            if component:
                component.debug_event(f"Transforming trajectory from {self.trajectory.frame.name} to {state.pose.frame.name}")
            self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

        # 1) Closest point
        fx, fy = self._find_front_axle_position(curr_x, curr_y, curr_yaw)
        search_start = self.current_path_parameter - 5.0
        search_end   = self.current_path_parameter + 5.0
        closest_dist, closest_parameter = self.path.closest_point_local((fx, fy), [search_start, search_end])
        self.current_path_parameter = closest_parameter

        # 2) Path heading
        target_x, target_y = self.path.eval(closest_parameter)
        tangent = self.path.eval_tangent(closest_parameter)
        path_yaw = atan2(tangent[1], tangent[0])

        # 3) Lateral error
        dx = fx - target_x
        dy = fy - target_y
        left_vec = np.array([sin(curr_yaw), -cos(curr_yaw)])
        cross_track_error = np.sign(np.dot(np.array([dx, dy]), left_vec)) * closest_dist

        # 4) Heading error
        yaw_error = normalise_angle(path_yaw - curr_yaw)

        # 5) Standard Stanley terms
        cross_term = atan2(self.k * cross_track_error, self.k_soft + speed)
        desired_steering_angle = yaw_error + cross_term

        # 5.1) Yaw-rate damping (optional if k_yaw_rate != 0)
        #      This term penalizes high yaw rates, helps reduce overshoot
        yaw_rate_damping = self.k_yaw_rate * (-speed * sin(desired_steering_angle)) / self.wheelbase
        desired_steering_angle += yaw_rate_damping

        # 5.2) Steering damping (penalize rapid changes)
        #      Resist big steering jumps from one cycle to the next
        steering_damp_term = self.k_damp_steer * (desired_steering_angle - self.prev_steering_angle)
        desired_steering_angle += steering_damp_term

        # 6) Clip the raw desired steering
        unfiltered_steering = np.clip(desired_steering_angle, -self.max_steer, self.max_steer)

        # 7) Low-pass filter for final steering
        #    alpha in [0..1], smaller -> more smoothing
        alpha = 0.2
        final_steering = alpha * unfiltered_steering + (1.0 - alpha) * self.prev_front_wheel_angle
        final_steering = np.clip(final_steering, -self.max_steer, self.max_steer)

        # Update memories
        self.prev_steering_angle = unfiltered_steering
        self.prev_front_wheel_angle = final_steering

        ################################################################
        # Speed Logic - Slightly gentler on cornering deceleration
        ################################################################

        # 8) Determine desired_speed from path/trajectory or fallback
        desired_speed = self.desired_speed
        feedforward_accel = 0.0

        if self.trajectory and self.desired_speed_source in ['path', 'trajectory']:
            if len(self.trajectory.points) < 2 or self.current_path_parameter >= self.path.domain()[1]:
                # End of trajectory -> stop
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

            # Cross-track-based slowdown (less aggressive than before).
            desired_speed *= np.exp(-abs(cross_track_error) * 0.6)

        # Steering-based slowdown: reduce speed if the steering angle is large
        angle_ratio = abs(final_steering) / self.max_steer
        # If angle_ratio is 1.0, we'll cut speed to ~ 50% of original
        speed_scale = 1.0 - 0.5 * angle_ratio
        # Keep speed_scale at least 0.4
        speed_scale = max(speed_scale, 0.4)
        desired_speed *= speed_scale

        # Clip to speed limit
        if desired_speed > self.speed_limit:
            desired_speed = self.speed_limit

        # 9) PID for longitudinal control
        speed_error = desired_speed - speed
        output_accel = self.pid_speed.advance(e=speed_error, t=t, feedforward_term=feedforward_accel)

        # Clip acceleration
        if output_accel > self.max_accel:
            output_accel = self.max_accel
        elif output_accel < -self.max_decel:
            output_accel = -self.max_decel

        # Avoid negative accel when fully stopped
        if desired_speed == 0 and abs(speed) < 0.01 and output_accel < 0:
            output_accel = 0.0

        # Debug
        if component:
            component.debug("Stanley: fx, fy", (fx, fy))
            component.debug("Stanley: path param", self.current_path_parameter)
            component.debug("Stanley: crosstrack dist", closest_dist)
            component.debug("Stanley: cross_track_error", cross_track_error)
            component.debug("Stanley: yaw_error", yaw_error)
            component.debug("Stanley: unfiltered_steering (rad)", unfiltered_steering)
            component.debug("Stanley: final_steering (rad)", final_steering)
            component.debug("Stanley: angle_ratio", angle_ratio)
            component.debug("Stanley: speed_scale", speed_scale)
            component.debug("Stanley: desired_speed (m/s)", desired_speed)
            component.debug("Stanley: feedforward_accel (m/s^2)", feedforward_accel)
            component.debug("Stanley: output_accel (m/s^2)", output_accel)
            component.debug("Stanley: current speed (m/s)", speed)

        self.t_last = t
        return (output_accel, final_steering)

#####################################
# 3. Tracker component
#####################################
class StanleyTrajectoryTracker(Component):
    """
    A trajectory-tracking Component that uses the above Stanley controller
    for lateral control plus PID-based longitudinal control.
    It now includes measures to mitigate oscillations.
    """

    def __init__(self, vehicle_interface=None, **kwargs):
        """
        :param vehicle_interface: The low-level interface to send commands to the vehicle.
        :param kwargs: Optional parameters to pass into the Stanley(...) constructor.
        """
        self.stanley = Stanley(**kwargs)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        """Control frequency in Hz."""
        return 50.0

    def state_inputs(self):
        """
        Required state inputs:
        - Vehicle state
        - Trajectory
        """
        return ["vehicle", "trajectory"]

    def state_outputs(self):
        """No direct output state here."""
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        """
        Per control cycle:
          1) Set the path/trajectory
          2) Compute (acceleration, front wheel angle)
          3) Convert front wheel angle to steering wheel angle (if necessary)
          4) Send command to the vehicle
        """
        self.stanley.set_path(trajectory)
        accel, f_delta = self.stanley.compute(vehicle, self)

        # If your low-level interface expects steering wheel angle:
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
