from ...mathutils.control import PID
from ...mathutils.signal import OnlineLowPassFilter
from ...utils import settings
from ...mathutils import transforms
from ...state.vehicle import VehicleState
from ...state.trajectory import Path,Trajectory,compute_headings
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np

class PurePursuit(object):   
    """Implements a pure pursuit controller on a second-order Dubins vehicle."""
    def __init__(self, lookahead = None, lookahead_scale = None, wheelbase = None, crosstrack_gain = None):
        self.look_ahead = lookahead if lookahead is not None else settings.get('control.pure_pursuit.lookahead',4.0)
        self.look_ahead_scale = lookahead_scale if lookahead_scale is not None else settings.get('control.pure_pursuit.lookahead_scale',3.0)
        self.crosstrack_gain = crosstrack_gain if crosstrack_gain is not None else settings.get('control.pure_pursuit.crosstrack_gain',0.41)
        self.wheelbase  = wheelbase if wheelbase is not None else settings.get('vehicle.geometry.wheelbase')
        self.wheel_angle_range = [settings.get('vehicle.geometry.min_wheel_angle'),settings.get('vehicle.geometry.max_wheel_angle')]
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]

        self.desired_speed = 1.5  # m/s, reference speed
        self.max_accel     = settings.get('vehicle.limits.max_accelerator_pedal') # % of acceleration
        self.pid_speed     = PID(0.5, 0.0, 0.1, windup_limit=20)
        self.speed_filter  = OnlineLowPassFilter(1.2, 30, 4)

        self.path = None
        self.path_with_headings = None
        self.path_progress = 0.0
        self.t_start = None

    def set_path(self, path : Path):
        self.path = path
        self.path_progress = 0.0
        if len(self.path.points[0]) > 2:
            self.path = self.path.get_dims([0,1])
        if not isinstance(self.path,Trajectory):
            self.path = self.path.arc_length_parameterize()
        self.path_with_headings = compute_headings(self.path)

    def compute(self, state : VehicleState):
        if self.path is None:
            #just stop
            accel = self.pid_speed(0.0, state.t)

        if self.t_start is None:
            self.t_start = state.t
        dt = state.t - self.t_start

        if self.path.frame != state.frame:
            self.path = self.path.to_frame(state.frame)
            self.path_with_headings = self.path_with_headings.to_frame(state.frame)
    
        curr_x = state.x
        curr_y = state.y
        curr_yaw = state.heading
        speed = state.v

        
        desired_x,desired_y,desired_yaw = self.path_with_headings.eval(self.path_progress + self.look_ahead)

        # finding the distance between the goal point and the vehicle
        # true look-ahead distance between a waypoint and current position
        L = transforms.vector_dist((curr_x, curr_y), (desired_x, desired_y))

        # find the curvature and the angle 
        alpha = desired_yaw - curr_yaw

        # ----------------- tuning this part as needed -----------------
        k       = self.crosstrack_gain
        angle_i = np.arctan((k * 2 * self.wheelbase * np.sin(alpha)) / L) 
        angle   = angle_i*2
        # ----------------- tuning this part as needed -----------------

        f_delta = np.clip(angle, self.wheel_angle_range[0], self.wheel_angle_range[1])

        steering_angle = np.clip(front2steer(f_delta), self.steering_angle_range[0], self.steering_angle_range[1])

        print("Closest point distance: " + str(L))
        print("Forward velocity: " + str(speed))
        ct_error = np.sin(alpha) * L
        print("Crosstrack Error: " + str(round(ct_error,3)))
        print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
        print("Steering wheel angle: " + str(np.degrees(steering_angle)) + " degrees" )
        print("\n")

        filt_vel     = self.speed_filter(speed)
        output_accel = self.pid_speed.advance(state.t, self.desired_speed - filt_vel)

        if output_accel > self.max_accel:
            output_accel = self.max_accel

        if output_accel < 0.3:
            output_accel = 0.3
        
        self.path_progress += speed * dt


class PurePursuitTrajectoryTracker(Component):
    def __init__(self,vehicle_interface):
        self.pure_pursuit = PurePursuit()
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ['vehicle','trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle : VehicleState, trajectory: Trajectory):
        self.pure_pursuit.set_path(trajectory)
        res = self.pure_pursuit.compute(vehicle)
        res = GEMVehicleCommand(accelerator_pedal_position=res.accel, brake_pedal_position=res.brake, steering_wheel_angle=res.steer)
        self.vehicle_interface.send_command(res)
    
    def healthy(self):
        return self.pure_pursuit.path is not None