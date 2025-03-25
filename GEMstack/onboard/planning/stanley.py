from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np

class PurePursuit(object):
    """Implements a pure pursuit controller on a second-order Dubins vehicle."""
    # ... [Keep existing PurePursuit implementation unchanged] ...

class StanleyController(object):
    """Implements a Stanley controller for path tracking."""
    def __init__(self, k_gain=None, soft_gain=None, desired_speed=None):
        self.k_gain = k_gain if k_gain is not None else settings.get('control.stanley.k_gain', 2.5)
        self.soft_gain = soft_gain if soft_gain is not None else settings.get('control.stanley.soft_gain', 0.1)
        self.front_wheel_angle_scale = 3.0
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')
        self.wheel_angle_range = [settings.get('vehicle.geometry.min_wheel_angle'), 
                                settings.get('vehicle.geometry.max_wheel_angle')]
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),
                                    settings.get('vehicle.geometry.max_steering_angle')]
        
        if desired_speed is not None:
            self.desired_speed_source = desired_speed
        else:
            self.desired_speed_source = settings.get('control.stanley.desired_speed', "path")
        self.desired_speed = self.desired_speed_source if isinstance(self.desired_speed_source, (int, float)) else None
        self.speed_limit = settings.get('vehicle.limits.max_speed')
        self.max_accel = settings.get('vehicle.limits.max_acceleration')  # m/s^2
        self.max_decel = settings.get('vehicle.limits.max_deceleration')  # m/s^2
        self.pid_speed = PID(settings.get('control.longitudinal_control.pid_p', 0.5),
                            settings.get('control.longitudinal_control.pid_d', 0.0),
                            settings.get('control.longitudinal_control.pid_i', 0.1),
                            windup_limit=20)

        self.path_arg = None
        self.path = None         # type: Trajectory
        self.trajectory = None   # type: Trajectory
        self.current_path_parameter = 0.0
        self.current_traj_parameter = 0.0
        self.t_last = None

    def set_path(self, path: Path):
        if path == self.path_arg:
            return
        self.path_arg = path
        if len(path.points[0]) > 2:
            path = path.get_dims([0, 1])
        if not isinstance(path, Trajectory):
            self.path = path.arc_length_parameterize()
            self.trajectory = None
            self.current_traj_parameter = 0.0
            if self.desired_speed_source in ['path', 'trajectory']:
                raise ValueError("Can't provide an untimed path to StanleyController and expect it to use the path velocity.")
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]
        self.current_path_parameter = 0.0

    def compute(self, state: VehicleState, component: Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
        t = state.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        
        # speed = state.v
        # Protect against divide-by-zero issues
        # I think there is arctan(k * e / v) in Stanley
        speed = max(state.v, 0.01)

        # if self.path is None:
        #     # just stop
        #     accel = self.pid_speed.advance(0.0, t)
        #     raise RuntimeError("Behavior without path not implemented")
        if self.path is None:
            print("[StanleyController] Warning: No path set. Sending zero commands.")
            return (0.0, 0.0)

        if self.path.frame != state.pose.frame:
            print("Transforming path from", self.path.frame.name, "to", state.pose.frame.name)
            self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
        if self.trajectory is not None:
            if self.trajectory.frame != state.pose.frame:
                print("Transforming trajectory from", self.trajectory.frame.name, "to", state.pose.frame.name)
                self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

        # Find closest point on path
        closest_dist, closest_parameter = self.path.closest_point_local(
            (curr_x, curr_y), [self.current_path_parameter - 5.0, self.current_path_parameter + 5.0])
        self.current_path_parameter = closest_parameter
        self.current_traj_parameter += dt

        # Get path point and tangent at closest point
        path_point = self.path.eval(closest_parameter)
        path_tangent = self.path.eval_tangent(closest_parameter)
        path_yaw = np.arctan2(path_tangent[1], path_tangent[0])

        # Calculate crosstrack error (e) - signed distance from front axle to path
        front_x = curr_x + self.wheelbase * np.cos(curr_yaw)
        front_y = curr_y + self.wheelbase * np.sin(curr_yaw)
        
        # Vector from path point to front axle
        dx = front_x - path_point[0]
        dy = front_y - path_point[1]
        
        # Calculate perpendicular distance (crosstrack error)
        # Cross product between path tangent and vector to front axle gives signed distance
        e = (dx * path_tangent[1] - dy * path_tangent[0]) / np.linalg.norm(path_tangent)

        # Calculate heading error (θ)
        theta_e = transforms.normalize_angle(path_yaw - curr_yaw)

        # Stanley control law
        # Avoid division by zero with soft_gain term
        crosstrack_term = np.arctan2(self.k_gain * e, speed + self.soft_gain)
        f_delta = theta_e + crosstrack_term
        f_delta = np.clip(f_delta, self.wheel_angle_range[0], self.wheel_angle_range[1])

        # # Debug outputs
        # print("Crosstrack Error: " + str(round(e, 3)))
        # print("Heading Error: " + str(round(np.degrees(theta_e), 2)) + " degrees")
        # print("Front wheel angle: " + str(round(np.degrees(f_delta), 2)) + " degrees")
        # steering_angle = np.clip(front2steer(f_delta), self.steering_angle_range[0], self.steering_angle_range[1])
        # print("Steering wheel angle: " + str(round(np.degrees(steering_angle), 2)) + " degrees")

        # Debug outputs
        steering_angle = np.clip(front2steer(f_delta), self.steering_angle_range[0], self.steering_angle_range[1])
        debug_info = {
            "Crosstrack Error (m)": round(e, 3),
            "Heading Error (deg)": round(np.degrees(theta_e), 2),
            "Front Wheel Angle (deg)": round(np.degrees(f_delta), 2),
            "Steering Angle (deg)": round(np.degrees(steering_angle), 2),
            "Desired Speed (m/s)": round(desired_speed, 2),
            "Output Acceleration (m/s^2)": round(output_accel, 2)
        }
        for k, v in debug_info.items():
            print(f"{k}: {v}")

        # Speed control (same as PurePursuit)
        desired_speed = self.desired_speed
        feedforward_accel = 0.0
        if self.desired_speed_source in ['path', 'trajectory']:
            if len(self.trajectory.points) < 2 or self.current_path_parameter >= self.path.domain()[1]:
                if component is not None:
                    component.debug_event('Past the end of trajectory')
                # desired_speed = 0.0
                # feedforward_accel = -2.0

                # Try smooth deceleration
                desired_speed *= 0.5
                if desired_speed < 0.1:
                    desired_speed = 0.0
                    feedforward_accel = -2.0  # Full stop
            else:
                if self.desired_speed_source == 'path':
                    current_trajectory_time = self.trajectory.parameter_to_time(self.current_path_parameter)
                else:
                    current_trajectory_time = self.current_traj_parameter
                deriv = self.trajectory.eval_derivative(current_trajectory_time)
                desired_speed = min(np.linalg.norm(deriv), self.speed_limit)
                difference_dt = 0.1
                if current_trajectory_time >= self.trajectory.domain()[1]:
                    prev_deriv = self.trajectory.eval_derivative(current_trajectory_time - difference_dt)
                    prev_desired_speed = min(np.linalg.norm(prev_deriv), self.speed_limit)
                    feedforward_accel = (desired_speed - prev_desired_speed)/difference_dt
                else:
                    next_deriv = self.trajectory.eval_derivative(current_trajectory_time + difference_dt)
                    next_desired_speed = min(np.linalg.norm(next_deriv), self.speed_limit)
                    feedforward_accel = (next_desired_speed - desired_speed)/difference_dt
                feedforward_accel = np.clip(feedforward_accel, -self.max_decel, self.max_accel)
        else:
            # decay speed when crosstrack error is high
            desired_speed *= np.exp(-abs(e) * 0.4)
        
        if desired_speed > self.speed_limit:
            desired_speed = self.speed_limit

        output_accel = self.pid_speed.advance(e=desired_speed - speed, t=t, feedforward_term=feedforward_accel)
        output_accel = np.clip(output_accel, -self.max_decel, self.max_accel)

        if component is not None:
            component.debug('curr pt', (curr_x, curr_y))
            component.debug('closest path pt', path_point)
            component.debug('path tangent (rad)', path_yaw)
            component.debug('crosstrack error', e)
            component.debug('heading error (rad)', theta_e)
            component.debug('front wheel angle (rad)', f_delta)
            component.debug('desired speed (m/s)', desired_speed)
            component.debug('output accel (m/s^2)', output_accel)

        self.t_last = t
        return (output_accel, f_delta)


class StanleyTrajectoryTracker(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.stanley = StanleyController(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.stanley.set_path(trajectory)
        accel, wheel_angle = self.stanley.compute(vehicle, self)
        steering_angle = np.clip(
            front2steer(wheel_angle),
            self.stanley.steering_angle_range[0],
            self.stanley.steering_angle_range[1]
        )
        self.vehicle_interface.send_command(
            self.vehicle_interface.simple_command(accel, steering_angle, vehicle)
        )
    
    def healthy(self):
        return self.stanley.path is not None


def is_goal_reached(self, state: VehicleState, threshold=0.5):
    """
    Checks if the vehicle is near the end of the path.
    Used to trigger stop, mode switch, or task completion.
    """
    if self.path is None:
        return False
    end_point = self.path.eval(self.path.domain()[1])
    dx = state.pose.x - end_point[0]
    dy = state.pose.y - end_point[1]
    return np.hypot(dx, dy) < threshold