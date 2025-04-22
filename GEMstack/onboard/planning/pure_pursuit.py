
# from typing import Dict
# from ...mathutils.control import PID
# from ...utils import settings
# from ...mathutils import transforms
# from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
# from ...state.vehicle import VehicleState,ObjectFrameEnum
# from ...state.trajectory import Path,Trajectory,compute_headings
# from ...knowledge.vehicle.geometry import front2steer
# from ...state import AgentState, AgentEnum, EntityRelation, EntityRelationEnum, ObjectFrameEnum, VehicleState
# from ..interface.gem import GEMVehicleCommand
# from ..component import Component
# import numpy as np

# class PurePursuit(Component):   
#     """Implements a pure pursuit controller on a second-order Dubins vehicle."""
#     def __init__(self, lookahead = None, lookahead_scale = None, crosstrack_gain = None, desired_speed = None):
#         self.look_ahead = lookahead if lookahead is not None else settings.get('control.pure_pursuit.lookahead',4.0)
#         self.look_ahead_scale = lookahead_scale if lookahead_scale is not None else settings.get('control.pure_pursuit.lookahead_scale',3.0)
#         self.crosstrack_gain = crosstrack_gain if crosstrack_gain is not None else settings.get('control.pure_pursuit.crosstrack_gain',0.41)
#         self.front_wheel_angle_scale = 3.0
#         self.wheelbase  = settings.get('vehicle.geometry.wheelbase')
#         self.wheel_angle_range = [settings.get('vehicle.geometry.min_wheel_angle'),settings.get('vehicle.geometry.max_wheel_angle')]
#         self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        
#         if desired_speed is not None:
#             self.desired_speed_source = desired_speed
#         else:
#             self.desired_speed_source = settings.get('control.pure_pursuit.desired_speed',"path") 
#         self.desired_speed = self.desired_speed_source if isinstance(self.desired_speed_source,(int,float)) else None
#         self.speed_limit = settings.get('vehicle.limits.max_speed')
#         self.max_accel     = settings.get('vehicle.limits.max_acceleration') # m/s^2
#         self.max_decel     = settings.get('vehicle.limits.max_deceleration') # m/s^2
#         self.pid_speed     = PID(settings.get('control.longitudinal_control.pid_p',0.5), settings.get('control.longitudinal_control.pid_d',0.0), settings.get('control.longitudinal_control.pid_i',0.1), windup_limit=20)

#         self.path_arg = None
#         self.path = None         # type: Trajectory  
#         #if trajectory = None, then only an untimed path was provided and we can't use the path velocity as the reference
#         self.trajectory = None   # type: Trajectory

#         self.current_path_parameter = 0.0
#         self.current_traj_parameter = 0.0
#         self.t_last = None

#     def set_path(self, path : Path):
#         if path == self.path_arg:
#             return
#         self.path_arg = path
#         if len(path.points[0]) > 2:
#             path = path.get_dims([0,1])
#         if not isinstance(path,Trajectory):
#             self.path = path.arc_length_parameterize()
#             self.trajectory = None
#             self.current_traj_parameter = 0.0
#             if self.desired_speed_source in ['path','trajectory']:
#                 raise ValueError("Can't provide an untimed path to PurePursuit and expect it to use the path velocity. Set control.pure_pursuit.desired_speed to a constant.")
#         else:
#             self.path = path.arc_length_parameterize()
#             self.trajectory = path
#             self.current_traj_parameter = self.trajectory.domain()[0]
#         self.current_path_parameter = 0.0

#     def compute(self, state: VehicleState, agents: Dict[str, AgentState], component: Component = None):
#         assert state.pose.frame != ObjectFrameEnum.CURRENT
#         t = state.pose.t

#         if self.t_last is None:
#             self.t_last = t
#         dt = t - self.t_last
  
#         curr_x = state.pose.x
#         curr_y = state.pose.y
#         curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
#         speed = state.v

#         safety_factor = 1.0
#         #for agent_id, agent in agents.items():
#             # #distance = dist_to_agent(agent, state)
#             # if distance < 6.0:  # If agent is within 6 meters
#             #     safety_factor = 0.1  # Reduce to 1/10 of normal parameters
#             #     if component is not None:
#             #         component.debug_event(f'Agent {agent_id} detected at {distance:.2f}m - reducing speed')
#                # break

#         if self.path is None:
#             #just stop
#             accel = self.pid_speed.advance(0.0, t)
#             # TODO
#             raise RuntimeError("Behavior without path not implemented")

#         if self.path.frame != state.pose.frame:
#             print("Transforming path from",self.path.frame.name,"to",state.pose.frame.name)
#             self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
#         if self.trajectory is not None:
#             if self.trajectory.frame != state.pose.frame:
#                 print("Transforming trajectory from",self.trajectory.frame.name,"to",state.pose.frame.name)
#                 self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

#         closest_dist,closest_parameter = self.path.closest_point_local((curr_x,curr_y),[self.current_path_parameter-5.0,self.current_path_parameter+5.0])
#         self.current_path_parameter = closest_parameter
#         self.current_traj_parameter += dt
#         #TODO: calculate parameter that is look_ahead distance away from the closest point?
#         #(rather than just advancing the parameter)
#         des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * speed
#         print("Closest parameter: " + str(closest_parameter),"distance to path",closest_dist)
#         if closest_dist > 0.1:
#             print("Closest point",self.path.eval(closest_parameter),"vs",(curr_x,curr_y))
#         if des_parameter >= self.path.domain()[1]:
#             #we're at the end of the path, calculate desired point by extrapolating from the end of the path
#             end_pt = self.path.points[-1]
#             if len(self.path.points) > 1:
#                 end_dir = self.path.eval_tangent(self.path.domain()[1])
#             else:
#                 #path is just a single point, just look at current direction
#                 end_dir = (np.cos(curr_yaw),np.sin(curr_yaw))
#             desired_x,desired_y = transforms.vector_madd(end_pt,end_dir,(des_parameter-self.path.domain()[1]))
#         else:
#             desired_x,desired_y = self.path.eval(des_parameter)
#         desired_yaw = np.arctan2(desired_y-curr_y,desired_x-curr_x)
        
#         print("Desired point",(desired_x,desired_y)," with lookahead distance",self.look_ahead + self.look_ahead_scale * speed)
#         print("Current x",curr_x,"Current y",curr_y)
#         print("Current yaw",curr_yaw,"Desired yaw",desired_yaw)
#         print("Parameter : ", self.current_path_parameter)
#         print("Last index : ",  self.path.domain()[1])
        
#         # distance between the desired point and the vehicle
#         L = transforms.vector2_dist((desired_x,desired_y),(curr_x,curr_y))

#         # find the curvature and the angle 
#         alpha = transforms.normalize_angle(desired_yaw - curr_yaw)
#         if alpha > np.pi:
#             alpha -= np.pi*2

#         # ----------------- tuning this part as needed -----------------
#         k       = self.crosstrack_gain
#         angle_i = np.arctan((k * 2 * self.wheelbase * np.sin(alpha)) / L) 
#         angle   = angle_i*self.front_wheel_angle_scale
#         # ----------------- tuning this part as needed -----------------

#         f_delta = np.clip(angle, self.wheel_angle_range[0], self.wheel_angle_range[1])
        
#         #print("Closest point distance: " + str(L))
#         print("Forward velocity: " + str(speed))
#         ct_error = np.sin(alpha) * L
#         print("Crosstrack Error: " + str(round(ct_error,3)))
#         print("Front wheel angle: " + str(round(np.degrees(f_delta),2)) + " degrees")
#         steering_angle = np.clip(front2steer(f_delta), self.steering_angle_range[0], self.steering_angle_range[1])
#         print("Steering wheel angle: " + str(round(np.degrees(steering_angle),2)) + " degrees" )
        
#         desired_speed = self.desired_speed
#         feedforward_accel = 0.0
#         if self.desired_speed_source in ['path','trajectory']:
#             #determine desired speed from trajectory
#             if len(self.trajectory.points) < 2 or self.current_path_parameter >= self.path.domain()[1]:
#                 if component is not None:
#                     component.debug_event('Past the end of trajectory')
#                 #past the end, just stop
#                 desired_speed = 0.0
#                 feedforward_accel = -2.0
#                 f_delta = 0
#             else:
#                 if self.desired_speed_source=='path':
#                     current_trajectory_time = self.trajectory.parameter_to_time(self.current_path_parameter)
#                 else:
#                     current_trajectory_time = self.current_traj_parameter
#                 deriv = self.trajectory.eval_derivative(current_trajectory_time)
#                 desired_speed = min(np.linalg.norm(deriv),self.speed_limit)
#                 difference_dt = 0.1
#                 if current_trajectory_time >= self.trajectory.domain()[1]:
#                     prev_deriv = self.trajectory.eval_derivative(current_trajectory_time - difference_dt)
#                     prev_desired_speed = min(np.linalg.norm(prev_deriv),self.speed_limit)
#                     feedforward_accel = (desired_speed - prev_desired_speed)/difference_dt
#                     print("Desired speed",desired_speed,"m/s",", from prior",prev_desired_speed,"m/s")
#                 else:
#                     next_deriv = self.trajectory.eval_derivative(current_trajectory_time + difference_dt)
#                     next_desired_speed = min(np.linalg.norm(next_deriv),self.speed_limit)
#                     feedforward_accel = (next_desired_speed - desired_speed)/difference_dt
#                     print("Desired speed",desired_speed,"m/s",", trying to reach desired",next_desired_speed,"m/s")
#                 feedforward_accel= np.clip(feedforward_accel, -self.max_decel, self.max_accel)
#                 print("Feedforward accel: " + str(feedforward_accel) + " m/s^2")
#         else:
#             #decay speed when crosstrack error is high
#             desired_speed *= np.exp(-abs(ct_error)*0.4)
            
#             if len(self.trajectory.points) < 2 or self.current_path_parameter >= self.path.domain()[1]:
#                 if component is not None:
#                     component.debug_event('Past the end of trajectory')
#                 #past the end, just stop
#                 desired_speed = 0.0
#                 feedforward_accel = -2.0
#                 f_delta = 0
            
            
            
#         if desired_speed > self.speed_limit:
#             desired_speed = self.speed_limit 
#         output_accel = self.pid_speed.advance(e = desired_speed - speed, t = t, feedforward_term=feedforward_accel)
#         if component is not None:
#             component.debug('curr pt',(curr_x,curr_y))
#             component.debug('curr param',self.current_path_parameter)
#             component.debug('desired pt',(desired_x,desired_y))
#             component.debug('desired yaw (rad)',desired_yaw)
#             component.debug('crosstrack error',ct_error)
#             component.debug('front wheel angle (rad)',f_delta)
#             component.debug('desired speed (m/s)',desired_speed)
#             component.debug('feedforward accel (m/s^2)',feedforward_accel)
#             component.debug('output accel (m/s^2)',output_accel)
#         print("Output accel: " + str(output_accel) + " m/s^2")

#         if output_accel > self.max_accel:
#             output_accel = self.max_accel

#         if output_accel < -self.max_decel:
#             output_accel = -self.max_decel

#         self.t_last = t
#         return (output_accel, f_delta)

# class PurePursuitTrajectoryTracker(Component):
#     def __init__(self,vehicle_interface=None, **args):
#         self.pure_pursuit = PurePursuit(**args)
#         self.vehicle_interface = vehicle_interface

#     def rate(self):
#         return 50.0

#     def state_inputs(self):
#         return ['vehicle','trajectory']

#     def state_outputs(self):
#         return []

#     def update(self, vehicle : VehicleState, trajectory: Trajectory):
#         self.pure_pursuit.set_path(trajectory)
#         accel,wheel_angle = self.pure_pursuit.compute(vehicle, self)
#         #print("Desired wheel angle",wheel_angle)
#         steering_angle = np.clip(front2steer(wheel_angle), self.pure_pursuit.steering_angle_range[0], self.pure_pursuit.steering_angle_range[1])
#         #print("Desired steering angle",steering_angle)
#         self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
    
#     def healthy(self):
#         return self.pure_pursuit.path is not None

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
        self.k = control_gain if control_gain is not None else settings.get('control.stanley.control_gain')
        self.k_soft = softening_gain if softening_gain is not None else settings.get('control.stanley.softening_gain')

        # Typically, this is the max front-wheel steering angle in radians
        self.max_steer = settings.get('vehicle.geometry.max_wheel_angle')
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')

        # Basic longitudinal constraints
        self.speed_limit = settings.get('vehicle.limits.max_speed')
        self.max_accel   = settings.get('vehicle.limits.max_acceleration')
        self.max_decel   = settings.get('vehicle.limits.max_deceleration')

        # PID for longitudinal speed tracking
        p = settings.get('control.longitudinal_control.pid_p')
        d = settings.get('control.longitudinal_control.pid_d')
        i = settings.get('control.longitudinal_control.pid_i')
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
        desired_x = target_x
        desired_y = target_y
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

        # Clip to speed limit
        if desired_speed > self.speed_limit:
            desired_speed = self.speed_limit

        # PID for longitudinal control
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
        if component is not None:
            # component.debug("Stanley: fx, fy", (fx, fy))
            component.debug('curr pt',(curr_x,curr_y))
            component.debug("desired_x",desired_x)
            component.debug("desired_y",desired_y)
            component.debug("Stanley: path param", self.current_path_parameter)
            component.debug("Stanley: crosstrack dist", closest_dist)
            component.debug("crosstrack error", cross_track_error)
            component.debug("Stanley: yaw_error", yaw_error)
            component.debug('steering_angle', desired_steering_angle)
            component.debug("Stanley: desired_speed (m/s)", desired_speed)
            component.debug("Stanley: feedforward_accel (m/s^2)", feedforward_accel)
            component.debug("Stanley: output_accel (m/s^2)", output_accel)

        if output_accel > self.max_accel:
            output_accel = self.max_accel

        if output_accel < -self.max_decel:
            output_accel = -self.max_decel
            
        self.t_last = t
        return (output_accel, desired_steering_angle)

#####################################
# 3. Tracker component
#####################################
#class StanleyTrajectoryTracker(Component):
class PurePursuitTrajectoryTracker(Component):

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