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


def normalise_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

class Stanley(object):
    """
    Stanley controller that supports both forward and reverse path tracking.
    Chooses front-axle reference for forward motion and rear-axle for reverse.
    """

    def __init__(
        self,
        control_gain=None,
        softening_gain=None,
        desired_speed=None  # float or 'path'/'trajectory'
    ):
        # Lateral gains
        self.k = control_gain or settings.get('control.stanley.control_gain')
        self.k_soft = softening_gain or settings.get('control.stanley.softening_gain')

        # Vehicle geometry
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')

        # Speed and acceleration limits
        self.speed_limit = settings.get('vehicle.limits.max_speed')
        self.reverse_speed_limit = settings.get('vehicle.limits.max_reverse_speed')
        self.max_accel = settings.get('vehicle.limits.max_acceleration')
        self.max_decel = settings.get('vehicle.limits.max_deceleration')

        # Longitudinal PID
        p = settings.get('control.longitudinal_control.pid_p')
        d = settings.get('control.longitudinal_control.pid_d')
        i = settings.get('control.longitudinal_control.pid_i')
        self.pid_speed = PID(p, d, i, windup_limit=20)

        # Desired speed source
        if desired_speed is not None:
            self.desired_speed_source = desired_speed
        else:
            self.desired_speed_source = settings.get('control.stanley.desired_speed', 'path')
        if isinstance(self.desired_speed_source, (int, float)):
            self.desired_speed = self.desired_speed_source
        else:
            self.desired_speed = None

        # Trajectory storage
        self.path_arg = None
        self.path = None
        self.trajectory = None
        self.current_path_parameter = 0.0
        self.current_traj_parameter = 0.0
        self.t_last = None

    def set_path(self, path: Path):
        if path == self.path_arg:
            return
        self.path_arg = path
        # Reduce to 2D
        if len(path.points[0]) > 2:
            path = path.get_dims([0,1])
        # Parameterization
        if not isinstance(path, Trajectory):
            self.path = path.arc_length_parameterize()
            self.trajectory = None
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]
        self.current_path_parameter = self.path.domain()[0]

    def _find_axle_position(self, x, y, yaw, forward: bool):
        """
        Compute axle position: front if forward, rear if reverse.
        """
        offset = self.wheelbase * (1 if forward else -1)
        ax = x + offset * cos(yaw)
        ay = y + offset * sin(yaw)
        return ax, ay

    def compute(self, state: VehicleState, component: Component = None):
        t = state.pose.t
        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        x, y, yaw = state.pose.x, state.pose.y, state.pose.yaw or 0.0
        v = state.v

        # Ensure path exists
        if self.path is None:
            if component:
                component.debug_event("No path: doing nothing.")
            return 0.0, 0.0

        # Transform path/trajectory into vehicle frame if needed
        if self.path.frame != state.pose.frame:
            self.path = transforms.transform_path(self.path, self.path.frame, state.pose.frame, current_pose=state.pose)
        if self.trajectory and self.trajectory.frame != state.pose.frame:
            self.trajectory = transforms.transform_path(self.trajectory, self.trajectory.frame, state.pose.frame, current_pose=state.pose)

        # Determine desired speed (positive forward, negative reverse)
        if isinstance(self.desired_speed_source, (int, float)):
            desired_speed = self.desired_speed_source
        else:
            # derive from trajectory speeds
            s = self.current_path_parameter
            if self.trajectory:
                t_param = self.trajectory.parameter_to_time(s) if hasattr(self.trajectory, 'parameter_to_time') else self.current_traj_parameter
                deriv = self.trajectory.eval_derivative(t_param)
                desired_speed = np.linalg.norm(deriv)
            else:
                desired_speed = self.desired_speed_source if isinstance(self.desired_speed_source, (int, float)) else 0.0
        # Clip by appropriate limit
        if desired_speed >= 0:
            desired_speed = min(desired_speed, self.speed_limit)
        else:
            desired_speed = -min(abs(desired_speed), self.reverse_speed_limit)

        # Choose forward or reverse
        forward = desired_speed >= 0

        # Reference axle position
        ax, ay = self._find_axle_position(x, y, yaw, forward)

        # Closest point on path
        ds = 5.0
        start = self.current_path_parameter - ds
        end = self.current_path_parameter + ds
        dist, s_closest = self.path.closest_point_local((ax, ay), [start, end])
        self.current_path_parameter = s_closest

        # Path heading and cross-track
        tx, ty = self.path.eval(s_closest)
        tangent = self.path.eval_tangent(s_closest)
        path_yaw = atan2(tangent[1], tangent[0])
        dx, dy = ax - tx, ay - ty
        # sign based on side
        side = -1 if forward else 1
        cross_error = side * (dx * (-tangent[1]) + dy * tangent[0]) if not forward else np.sign(np.dot([dx,dy], [sin(yaw), -cos(yaw)])) * dist

        # Heading error
        yaw_error = normalise_angle(path_yaw - yaw)
        yaw_term = yaw_error
        if not forward:
            yaw_term = -yaw_error

        # Stanley lateral term
        vel = v if forward else abs(v)
        cross_term_input = self.k * cross_error / (self.k_soft + vel)
        cross_term = atan2(cross_term_input, 1.0)
        if not forward:
            cross_term = atan2(self.k * (-cross_error), (self.k_soft + vel))

        steering_angle = yaw_term + cross_term

        # Longitudinal control via PID
        speed_error = desired_speed - v
        # simple feedforward inertia suppression can be added
        accel_cmd = self.pid_speed.advance(e=speed_error, t=t)
        accel_cmd = np.clip(accel_cmd, -self.max_decel, self.max_accel)

        # Prevent creep at stop
        if desired_speed == 0 and abs(v) < 0.05 and accel_cmd * v > 0:
            accel_cmd = 0.0

        if component:
            component.debug('mode', 'forward' if forward else 'reverse')
            component.debug('axle', (ax, ay))
            component.debug('s', self.current_path_parameter)
            component.debug('cte', cross_error)
            component.debug('yaw_err', yaw_error)
            component.debug('steer', steering_angle)
            component.debug('spd_des', desired_speed, 'spd', v)
            component.debug('acc', accel_cmd)

        self.t_last = t
        return accel_cmd, steering_angle

class StanleyTrajectoryTracker(Component):
    def __init__(self, vehicle_interface=None, **kwargs):
        self.stanley = Stanley(**kwargs)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.stanley.set_path(trajectory)
        accel, delta = self.stanley.compute(vehicle, self)
        steer = front2steer(delta)
        steer = np.clip(steer,
                        settings.get('vehicle.geometry.min_steering_angle', -0.5),
                        settings.get('vehicle.geometry.max_steering_angle', 0.5))
        cmd = self.vehicle_interface.simple_command(accel, steer, vehicle)
        self.vehicle_interface.send_command(cmd)

    def healthy(self):
        return self.stanley.path is not None
       
