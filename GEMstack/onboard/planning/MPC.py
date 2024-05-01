import time
import yaml
import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component
from ...knowledge.vehicle.geometry import front2steer
from scipy.interpolate import interp1d
from ...utils import settings
from ...mathutils import transforms        

class MPC(object):
    def __init__(self, dt, horizon, Q, R):
        # Defining the tunable parameters
        self.dt = dt
        self.horizon = horizon # horizon represented in seconds
        self.timesteps = int(self.horizon / self.dt) # horizon represented in the number of time steps
        self.Q = Q
        self.R = R
        self.look_ahead = 2.0
        self.look_ahead_scale = 1.0

        self.path_arg = None
        self.path = None 
        self.trajectory = None

        # Defining geometry and dynamic constraints on the vehicle
        # self.front_wheel_angle_scale = 3.0
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')
        self.max_deceleration = settings.get('vehicle.limits.max_deceleration')
        self.max_acceleration = settings.get('vehicle.limits.max_acceleration')
        self.min_wheel_angle = settings.get('vehicle.geometry.min_wheel_angle')
        self.max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle')
        self.min_steering_angle = settings.get('vehicle.geometry.min_steering_angle')
        self.max_steering_angle = settings.get('vehicle.geometry.max_steering_angle')
        self.max_speed = settings.get('vehicle.limits.max_speed')

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
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]
        self.current_path_parameter = 0.0

    def get_model_equations(self, states, controls):
        # Implement the kinematic bicycle model equations using CasADi
        x_next = states[0] + states[3] * ca.cos(states[2]) * self.dt
        y_next = states[1] + states[3] * ca.sin(states[2]) * self.dt
        theta_next = states[2] + (states[3] / self.wheelbase) * ca.tan(controls[1]) * self.dt
        v_next = states[3] + controls[0] * self.dt
        return ca.vertcat(x_next, y_next, theta_next, v_next)

    def get_cost_function(self, states, controls, ref_trajectory, start_time):
        # Implement the cost function using CasADi
        cost = 0
        t = start_time
        i = 0
        index_from_time = ref_trajectory.time_to_index(t)[0]
        while i < self.timesteps:
            index_from_time = ref_trajectory.time_to_index(t)[0]

            v_x = 0
            v_y = 0
            if index_from_time + 1 >= len(ref_trajectory.points):
                break
            else:
                v_x = ref_trajectory.eval_derivative(t)[0]
                v_y = ref_trajectory.eval_derivative(t)[1]
            ref_v = np.sqrt(v_x ** 2 + v_y ** 2)

            # penalizes x and y deviations
            cost += self.Q[0] * (ref_trajectory.eval(t)[0] - states[0, i])**2 + \
                self.Q[1] * (ref_trajectory.eval(t)[1] - states[1, i])**2
            
            # penalizes heading angle and velocity deviations
            cost += self.Q[2] * (ref_trajectory.eval(t)[2] - states[2, i])**2 + \
                self.Q[3] * (ref_v - states[3, i])**2
            
            # penalizes large control inputs
            cost += self.R[0] * controls[0, i] ** 2 + self.R[1] * controls[1, i] ** 2

            # penalizes large fluctuations in consecutive controls
            if i >= 1: 
                cost += self.R[2] * (controls[0, i] - controls[0, i - 1]) ** 2 + \
                    self.R[3] * (controls[1, i] - controls[1, i - 1]) ** 2
            t += self.dt
            i += 1
            # index_from_time += 1
        return cost

    def compute(self, state: VehicleState, component: Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
        t = state.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        current_state = [state.pose.x, state.pose.y, state.pose.yaw if state.pose.yaw is not None else 0.0, state.v]

        if self.path is None:
            raise RuntimeError("Behavior without path not implemented")

        if self.path.frame != state.pose.frame:
            print("Transforming path from", self.path.frame.name, "to", state.pose.frame.name)
            self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
        if self.trajectory is not None:
            if self.trajectory.frame != state.pose.frame:
                print("Transforming trajectory from", self.trajectory.frame.name, "to", state.pose.frame.name)
                self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

        closest_dist,closest_parameter = self.path.closest_point_local((state.pose.x,state.pose.y),[self.current_path_parameter-5.0,self.current_path_parameter+5.0])
        self.current_path_parameter = closest_parameter
        self.current_traj_parameter += dt

        des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * state.v
        print("Desired parameter: " + str(des_parameter),"distance to path",closest_dist)
        # print(self.path.parameter_to_time(des_parameter))

        # Slice a range of trajectory given the horizon value
        
        ref_trajectory = self.path
        ref_trajectory = compute_headings(ref_trajectory)
        # print("REF TRAJECTORY: ", ref_trajectory)
        # print("ref trajectory points: ", len(ref_trajectory.points))
        # print("ref trajectory times: ", len(ref_trajectory.times))

        # Set up the optimization problem
        opti = ca.Opti()
        x_vars = opti.variable(4, self.timesteps + 1)  # State variables
        u_vars = opti.variable(2, self.timesteps) # Control variables

        # Set initial conditions
        opti.subject_to(x_vars[:, 0] == current_state)

        for t in range(self.timesteps):
            # Model equations constraints
            x_next = self.get_model_equations(x_vars[:, t], u_vars[:, t])
            opti.subject_to(x_vars[:, t + 1] == x_next)

            # State constraints
            opti.subject_to(x_vars[3, t] < self.max_speed)
            # opti.subject_to(x_vars[:, 2] < self.max_wheel_angle)
            # opti.subject_to(x_vars[:, 2] > self.min_wheel_angle)

            # Control input constraints
            opti.subject_to(opti.bounded(-self.max_deceleration, u_vars[0, t], self.max_acceleration))
            opti.subject_to(opti.bounded(self.min_wheel_angle, u_vars[1, t], self.max_wheel_angle))
            
        # Set the objective function
        obj = self.get_cost_function(x_vars, u_vars, ref_trajectory, des_parameter)
        opti.minimize(obj)

        options = {'ipopt': {
            'tol': 1e-8,
            'max_iter': 5000,
        }}


        # Set up the solver
        # opti.solver('ipopt')
        opti.solver('ipopt', options)

        # Solve the optimization problem
        sol = opti.solve()

        # Extract the optimal control inputs
        optimal_control = sol.value(u_vars)

        # Extract the optimal steering angle and acceleration
        optimal_acceleration = optimal_control[0, 0]
        optimal_steering = optimal_control[1, 0]

        # Convert the steering angle to the corresponding steering wheel angle
        steering_wheel_angle = optimal_steering

        return optimal_acceleration, steering_wheel_angle

class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.MPC = MPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 5.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        start_time = time.perf_counter()
        self.MPC.set_path(trajectory)
        acceleration, wheel_angle = self.MPC.compute(vehicle)
        print("acceleration: ", acceleration)
        print("wheel_angle", wheel_angle)
        steering_wheel_angle = np.clip(front2steer(wheel_angle), self.MPC.min_steering_angle, self.MPC.max_steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(acceleration, steering_wheel_angle, vehicle))

        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.4f} seconds")

    def healthy(self):
        return self.MPC.path is not None
        