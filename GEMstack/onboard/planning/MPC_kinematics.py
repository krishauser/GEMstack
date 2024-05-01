# from ...mathutils.control import PID
# from ...utils import settings
# from ...mathutils import transforms
# from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
# from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
# from ...state.vehicle import VehicleState,ObjectFrameEnum
# from ...state.trajectory import Path,Trajectory,compute_headings
# from ...knowledge.vehicle.geometry import front2steer
# from ..interface.gem import GEMVehicleCommand
# from ..component import Component
# import numpy as np
# from casadi import *
# import time
# #Vehicle Param
# L = 2.56  # Wheelbase, GEMe4
# safety_margin = 0.1  


# # Setup the MPC problem 
# def setup_mpc(N, dt, L, path_points, x, gear=0):
#     # Constraints
#     delta_max = 0.6108
#     omega_max = 0.2 # TODO: What is a good value for this?
#     a_max = 1.0 
#     a_min = -1.0
#     v_max = 2.0
#     v_min = -2.0


#     # Initialization
#     x0, y0, theta0, v0, delta0 = x
#     v0 = v0.clip(v_min, v_max) # TODO: clip to avoid constrain issues, should be handled more carefully
#     # MPC setup
#     opti = Opti()  

#     # Control variables
#     A = opti.variable(N)
#     Omega = opti.variable(N)
#     # State variables
#     X = opti.variable(N+1)
#     Y = opti.variable(N+1)
#     theta = opti.variable(N+1)
#     V = opti.variable(N+1)
#     Delta = opti.variable(N+1)

#     # Provide an initial guess
#     for i in range(N):
#         opti.set_initial(X[i+1], path_points[i,0])
#         opti.set_initial(Y[i+1], path_points[i,1])
#     opti.set_initial(theta, theta0)
#     opti.set_initial(V, v0)
#     opti.set_initial(Delta, delta0)
#     # Initial control
#     opti.set_initial(A, 0)
#     opti.set_initial(Omega, 0)

#     # Constraints
#     opti.subject_to([X[0] == x0, Y[0] == y0, theta[0] == theta0, V[0] == v0, Delta[0] == delta0])
#     for j in range(N):
#         # Dynamics
#         opti.subject_to(X[j+1] == X[j] + dt * V[j] * cos(theta[j]))
#         opti.subject_to(Y[j+1] == Y[j] + dt * V[j] * sin(theta[j]))
#         opti.subject_to(theta[j+1] == theta[j] + dt * V[j] / L * tan(Delta[j]))
#         opti.subject_to(V[j+1] == V[j] + dt * A[j])
#         opti.subject_to(Delta[j+1] == Delta[j] + dt * Omega[j])
#         # Control bounds
#         opti.subject_to(a_min <= A[j])
#         opti.subject_to(A[j] <= a_max)
#         opti.subject_to(-omega_max <= Omega[j])
#         opti.subject_to(Omega[j] <= omega_max)
#         # State bounds
#         opti.subject_to(-delta_max <= Delta[j])
#         opti.subject_to(Delta[j] <= delta_max)
#         opti.subject_to(V[j] <= v_max)
#         opti.subject_to(v_min <= V[j])
        

#     # Set up the optimization problem
#     objective = 0
#     for j in range(N):
#         # TODO: Add heading objective?
#         objective += ((X[j+1] - path_points[j,0])**2 + (Y[j+1] - path_points[j,1])**2)
#         if path_points[j,2] is not None:
#             objective += (cos(theta[j+1]) - cos(path_points[j,2]))**2
#             objective += (sin(theta[j+1]) - sin(path_points[j,2]))**2
#     opti.minimize(objective)

#     # Set solver options for debugging
#     opti.solver('ipopt', {'ipopt': {'print_level': 0, 'tol': 1e-6}})
#     sol = opti.solve()

#     # Update to next state
#     a, omega = sol.value(A[0]), sol.value(Omega[0])
#     a += sign(a)*0.6 # To balance drag
#     print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     print("path points: ", path_points)
#     print("X: ", sol.value(X))
#     print("Y: ", sol.value(Y))
#     print("theta: ", sol.value(theta))
#     print("V: ", sol.value(V))
#     print("A: ", sol.value(A))
#     print("Delta: ", sol.value(Delta))
#     print("Omega: ", sol.value(Omega))
#     print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     return a, delta0 + 2.0*omega # TODO: What is a good way to update the steering angle?

# class MPCTracker(Component):
#     def __init__(self,vehicle_interface=None, **args):
#         self.vehicle_interface = vehicle_interface
#         self.N = 4
#         self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
#         self.max_a = 1.0
#         self.min_a = -1.0
#         self.max_d = 0.6108
#         self.min_d = -0.6108
#         self.dt = 1.75
#         self.horizen_scale = 1.0


#         self.last_progress = 0

#         self.start_time = time.time()

#     def rate(self):
#         return 10.0

#     def state_inputs(self):
#         return ['vehicle','trajectory']

#     def state_outputs(self):
#         return []

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

#     def update(self, vehicle : VehicleState, trajectory: Trajectory):

        
#         x_start, y_start, theta_start, v_start, wheel_angle_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v, vehicle.front_wheel_angle
#         print("current state :", [x_start, y_start, theta_start, v_start, wheel_angle_start, vehicle.heading_rate])
#         path_points = []
#         closest_dist,closest_parameter = trajectory.closest_point([x_start, y_start])
#         # closest_parameter = max(self.last_progress, closest_parameter)
#         ind, _ = trajectory.time_to_index(closest_parameter)
#         if trajectory.gear == None:
#             gear = 1
#         else: 
#             gear = trajectory.gear[ind]
#         # if closest_parameter - self.last_progress < 0.02:
#         #     self.horizen_scale *= 0.9
#         # else:
#         #     self.horizen_scale = 1.0
#         # self.horizen_scale = max(min(self.horizen_scale, 1.0), 0.1)
#         dt = self.dt * self.horizen_scale
#         # self.last_progress = closest_parameter + 0.01

#         trajectory = compute_headings(trajectory)
#         for i in range(1,self.N+1):
#             position = trajectory.eval(closest_parameter+i*dt)
#             velocity = trajectory.eval_derivative(closest_parameter+i*dt)
#             # yaw = trajectory.eval_yaw(closest_parameter+i*dt)
#             yaw = position[2]
#             velocity = np.linalg.norm(velocity)
#             path_points.append([position[0], position[1], yaw, velocity])
        
#         path_points = np.array(path_points)
#         # print([x_start, y_start, theta_start, v_start, wheel_angle_start])
#         # print(path_points)

#         accel, wheel_angle = setup_mpc(self.N, dt, L, path_points, [x_start, y_start, theta_start, v_start, wheel_angle_start], gear)
#         # print(accel, wheel_angle)
        
#         desired_yaw = np.arctan2(path_points[1][0],path_points[1][1])
#         if desired_yaw < 0.1:
#             accel = 1.2 * accel
#         elif 0.1 <= desired_yaw < 0.25:
#             accel = 0.6 * accel
#         elif desired_yaw >= 0.25:
#             accel = 0.4 * accel
#         accel = max(min(accel, self.max_a), self.min_a)

        
#         yaw_error = desired_yaw - vehicle.pose.yaw
#         if yaw_error < 0.1:
#             wheel_angle = 0.5 * wheel_angle
#         elif 0.1 <= yaw_error < 0.25 : 
#             wheel_angle = 1.0 * wheel_angle
#         elif yaw_error >= 0.25:
#             wheel_angle = 1.5 * wheel_angle
#         wheel_angle = max(min(wheel_angle, self.max_d), self.min_d)

#         d_remain = sqrt((vehicle.pose.x - path_points[-1][0]) ** 2 + (vehicle.pose.y - path_points[-1][1]) ** 2)
#         # print("remain distance: ", d_remain)
#         # if 0.2< d_remain <= 1.0:
#         #     wheel_angle = 0
#         #     accel = -1.0
#         # elif d_remain <= 0.2:
#         #     wheel_angle = 0
#         #     accel = 0.0
#         if time.time() - self.start_time < 5:
#             wheel_angle = 0
#             accel = 0

#         steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
#         self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
#         print(f"command to vehicle is ({accel, steering_angle, vehicle.gear})")


#     def healthy(self):
#         return True



import time
import yaml
import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component
from ...knowledge.vehicle.geometry import front2steer
from ...utils import settings      

class MPC(object):
    """Implements a MPC controller on a second-order Dubins vehicle."""
    def __init__(self, dt, horizon, Q, R, fixed, evaluation):
        # Defining the tunable parameters
        self.dt = dt
        self.horizon = horizon # horizon represented in seconds
        self.timesteps = int(self.horizon / self.dt) # horizon represented in the number of time steps
        self.Q = Q
        self.R = R
        self.fixed = fixed
        self.evaluation = evaluation
        self.look_ahead = 2.0
        self.look_ahead_scale = 1.0

        self.path_arg = None
        self.path = None 
        self.path_with_angles = None
        self.trajectory = None

        # Defining geometry and dynamic constraints on the vehicle
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')
        self.max_deceleration = settings.get('vehicle.limits.max_deceleration')
        self.max_acceleration = settings.get('vehicle.limits.max_acceleration')
        self.min_wheel_angle = settings.get('vehicle.geometry.min_wheel_angle')
        self.max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle')
        self.min_steering_angle = settings.get('vehicle.geometry.min_steering_angle')
        self.max_steering_angle = settings.get('vehicle.geometry.max_steering_angle')
        self.max_speed = settings.get('vehicle.limits.max_speed')

        self.t_last = None

        # reference and actual path for the evaluation
        self.ref_path_evaluation = []
        self.actual_path_evaluation = []


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

    def set_path_with_angles(self):
        self.path_with_angles = compute_headings(self.trajectory)

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

        closest_dist,closest_parameter = self.trajectory.closest_point_local((state.pose.x,state.pose.y),[self.current_traj_parameter-5.0,self.current_traj_parameter+5.0])
        self.current_traj_parameter = closest_parameter
        # self.current_traj_parameter += dt

        des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * state.v
        print("Desired parameter: " + str(des_parameter),"distance to path",closest_dist)
        
        ref_trajectory = self.trajectory
        if self.fixed:
            ref_trajectory = self.path_with_angles
        else:
            ref_trajectory = compute_headings(ref_trajectory)

        if self.evaluation:
            ref_state = [ref_trajectory.eval(t)[0], ref_trajectory.eval(t)[1], ref_trajectory.eval(t)[2]]
            self.ref_path_evaluation.append(ref_state)
            self.actual_path_evaluation.append(current_state[:3])
        
        print("REF PATH: ", self.ref_path_evaluation)
        print("ACTUAL PATH: ", self.actual_path_evaluation)

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

            # Control input constraints
            opti.subject_to(opti.bounded(-self.max_deceleration, u_vars[0, t], self.max_acceleration))
            opti.subject_to(opti.bounded(self.min_wheel_angle, u_vars[1, t], self.max_wheel_angle))
            
        # Set the objective function
        obj = self.get_cost_function(x_vars, u_vars, ref_trajectory, des_parameter)
        opti.minimize(obj)

        options = {'ipopt': {
            'print_level': 0,
            'tol': 1e-8,
            'max_iter': 5000,
        }}

        # Set up the solver
        opti.solver('ipopt', options)

        # Solve the optimization problem
        sol = opti.solve()

        print("----------------------------------------------------------------------------------------------")
        print(f"X: {sol.value(x_vars[0, :])}\n",
        f"Y: {sol.value(x_vars[1, :])}\n",
        f"Yaw: {sol.value(x_vars[2, :])}\n",
        f"Velocity: {sol.value(x_vars[3, :])}\n",
        f"Accel: {sol.value(u_vars[0, :])}\n",
        f"Delta: {sol.value(u_vars[1, :])}\n")
        print("----------------------------------------------------------------------------------------------")

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
        self.set_once = 1
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        start_time = time.perf_counter()
        self.MPC.set_path(trajectory)

        # Making sure we only calculate the headings once when dealing with
        # a fixed route to save computation time
        if self.set_once:
            self.MPC.set_path_with_angles()
            self.set_once = 0
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