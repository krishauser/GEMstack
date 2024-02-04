from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.vehicle import VehicleState,ObjectFrameEnum
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
        
        self.desired_speed = settings.get('control.pure_pursuit.desired_speed',2.5)  #approximately 5 mph
        self.path_is_timed = False
        self.desired_speed_from_path = True    #turn this to True to use the input trajectory to determine the desired speed
        self.max_accel     = settings.get('vehicle.limits.max_acceleration') # m/s^2
        self.max_decel     = settings.get('vehicle.limits.max_deceleration') # m/s^2
        self.pid_speed     = PID(settings.get('control.longitudinal_control.pid_p',0.5), settings.get('control.longitudinal_control.pid_d',0.0), settings.get('control.longitudinal_control.pid_i',0.1), windup_limit=20)

        self.path = None
        self.current_path_parameter = 0.0
        self.t_last = None

    def set_path(self, path : Path):
        if path == self.path:
            return
        self.path = path
        if len(self.path.points[0]) > 2:
            self.path = self.path.get_dims([0,1])
        if not isinstance(self.path,Trajectory):
            self.path = self.path.arc_length_parameterize()
            self.path_is_timed = False
            self.desired_speed_from_path = False
            if self.desired_speed_from_path:
                raise ValueError("Can't provide an untimed path to PurePursuit and expect it to use the path velocity")
        else:
            self.path_is_timed = True
        self.current_path_parameter = 0.0

    def compute(self, state : VehicleState):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
        t = state.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
  
        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        speed = state.v

        if self.path is None:
            #just stop
            accel = self.pid_speed.advance(0.0, t)
            # TODO

        if self.path.frame != state.pose.frame:
            print("Transforming path from",self.path.frame.name,"to",state.pose.frame.name)
            self.path = self.path.to_frame(state.pose.frame)

        closest_dist,closest_parameter = self.path.closest_point_local((curr_x,curr_y),[self.current_path_parameter-5.0,self.current_path_parameter+5.0])
        #TODO: calculate parameter that is look_ahead distance away from the closest point
        #(rather than just advancing the parameter)
        des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * speed
        self.current_path_parameter = closest_parameter
        print("Closest parameter: " + str(closest_parameter),"distance to path",closest_dist)
        print("Closest point",self.path.eval(closest_parameter),"vs",(curr_x,curr_y))
        desired_x,desired_y = self.path.eval(des_parameter)
        desired_yaw = np.arctan2(desired_y-curr_y,desired_x-curr_x)
        print("Current yaw",curr_yaw,"desired yaw",desired_yaw)

        # distance between the desired point and the vehicle
        L = transforms.vector2_dist((desired_x,desired_y),(curr_x,curr_y))

        # find the curvature and the angle 
        alpha = transforms.normalize_angle(desired_yaw - curr_yaw)
        if alpha > np.pi:
            alpha -= np.pi*2

        # ----------------- tuning this part as needed -----------------
        k       = self.crosstrack_gain
        angle_i = np.arctan((k * 2 * self.wheelbase * np.sin(alpha)) / L) 
        angle   = angle_i*2.0
        # ----------------- tuning this part as needed -----------------

        f_delta = np.clip(angle, self.wheel_angle_range[0], self.wheel_angle_range[1])
        
        print("Closest point distance: " + str(L))
        print("Forward velocity: " + str(speed))
        ct_error = np.sin(alpha) * L
        print("Crosstrack Error: " + str(round(ct_error,3)))
        print("Front steering angle: " + str(round(np.degrees(f_delta),2)) + " degrees")
        steering_angle = np.clip(front2steer(f_delta), self.steering_angle_range[0], self.steering_angle_range[1])
        print("Steering wheel angle: " + str(round(np.degrees(steering_angle),2)) + " degrees" )
        
        desired_speed = self.desired_speed
        feedforward_accel = 0.0
        if self.desired_speed_from_path:
            #determine desired speed from trajectory
            deriv = self.path.eval_derivative(self.current_path_parameter)
            desired_speed = np.linalg.norm(deriv)
            feedforward_accel = (desired_speed - speed)/dt 
            feedforward_accel= np.clip(feedforward_accel, -self.max_decel, self.max_accel)
            print("Desired speed",desired_speed,"m/s")
            print("Feedforward accel: " + str(feedforward_accel) + " m/s^2")
        else:
            #decay speed when crosstrack error is high
            desired_speed *= np.exp(-ct_error*0.1)
        output_accel = self.pid_speed.advance(e = desired_speed - speed, t = t, feedforward_term=feedforward_accel)
        print("Output acceleration",output_accel)

        if output_accel > self.max_accel:
            output_accel = self.max_accel

        if output_accel < -self.max_decel:
            output_accel = -self.max_decel

        self.t_last = t
        return (output_accel, f_delta)


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
        accel,wheel_angle = self.pure_pursuit.compute(vehicle)
        #print("Desired wheel angle",wheel_angle)
        steering_angle = np.clip(front2steer(wheel_angle), self.pure_pursuit.steering_angle_range[0], self.pure_pursuit.steering_angle_range[1])
        #print("Desired steering angle",steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
    
    def healthy(self):
        return self.pure_pursuit.path is not None