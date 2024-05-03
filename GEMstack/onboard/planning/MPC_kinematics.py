from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...state.vehicle import VehicleState,ObjectFrameEnum
from ...state.trajectory import Path,Trajectory,compute_headings
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np
from casadi import *
import time

class MPC(object):
    """Implements a MPC controller on a second-order Dubins vehicle."""
    def __init__(self, N, dt, Q, R, fixed):
        # Defining the tunable parameters
        self.N = N
        self.dt = dt
        self.Q = Q
        self.R = R
        self.fixed = fixed
        self.path_arg = None
        self.path = None 
        self.path_with_angles = None
        self.trajectory = None

        # Defining geometry and dynamic constraints on the vehicle
        self.L = settings.get('vehicle.geometry.wheelbase')
        self.a_min = -settings.get('vehicle.limits.max_deceleration')
        self.a_max = settings.get('vehicle.limits.max_acceleration')
        self.delta_max = settings.get('vehicle.geometry.max_wheel_angle')
        self.omega_max = settings.get('vehicle.limits.max_steering_rate') 
        self.v_max = settings.get('vehicle.limits.max_speed')
        self.v_min = -self.v_max

        self.t_last = None
        self.closest_parameter = 0

    def set_path(self, path: Path):
        if self.fixed:
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

    # Setup the MPC problem 
    def setup_mpc(self, path_points, x):
        # Initialization
        x0, y0, theta0, v0, delta0 = x
        v0 = v0.clip(self.v_min, self.v_max)

        # MPC setup
        opti = Opti()  

        # Control variables
        A = opti.variable(self.N)
        Omega = opti.variable(self.N)
        # State variables
        X = opti.variable(self.N+1)
        Y = opti.variable(self.N+1)
        theta = opti.variable(self.N+1)
        V = opti.variable(self.N+1)
        Delta = opti.variable(self.N+1)

        # Provide an initial guess
        for i in range(self.N):
            opti.set_initial(X[i+1], path_points[i,0])
            opti.set_initial(Y[i+1], path_points[i,1])
        opti.set_initial(theta, theta0)
        opti.set_initial(V, v0)
        opti.set_initial(Delta, delta0)
        # Initial control
        opti.set_initial(A, 0)
        opti.set_initial(Omega, 0)

        # Constraints
        opti.subject_to([X[0] == x0, Y[0] == y0, theta[0] == theta0, V[0] == v0, Delta[0] == delta0])
        for j in range(self.N):
            # Dynamics
            opti.subject_to(X[j+1] == X[j] + self.dt * V[j] * cos(theta[j]))
            opti.subject_to(Y[j+1] == Y[j] + self.dt * V[j] * sin(theta[j]))
            opti.subject_to(theta[j+1] == theta[j] + self.dt * V[j] / self.L * tan(Delta[j]))
            opti.subject_to(V[j+1] == V[j] + self.dt * A[j])
            opti.subject_to(Delta[j+1] == Delta[j] + self.dt * Omega[j])
            # Control bounds
            opti.subject_to(self.a_min <= A[j])
            opti.subject_to(A[j] <= self.a_max)
            opti.subject_to(-self.omega_max <= Omega[j])
            opti.subject_to(Omega[j] <= self.omega_max)
            # State bounds
            opti.subject_to(-self.delta_max <= Delta[j])
            opti.subject_to(Delta[j] <= self.delta_max)
            opti.subject_to(V[j] <= self.v_max)
            opti.subject_to(self.v_min <= V[j])
            
        # Set up the optimization problem
        objective = 0
        for j in range(self.N):
            objective += self.Q[0] * ((X[j+1] - path_points[j,0])**2 + (Y[j+1] - path_points[j,1])**2)
            if path_points[j,2] is not None:
                objective += self.Q[1] * (cos(theta[j+1]) - cos(path_points[j,2]))**2
                objective += self.Q[1] * (sin(theta[j+1]) - sin(path_points[j,2]))**2
            objective += self.Q[2] * (V[j+1] - path_points[j,3])**2

            # penalizes large control inputs
            objective += self.R[0] * A[j] ** 2 + self.R[1] * Omega[j] ** 2

            # penalizes large fluctuations in consecutive controls
            if j >= 1: 
                objective += self.R[2] * (A[j] - A[j-1]) ** 2 + \
                    self.R[3] * (Omega[j] - Omega[j-1]) ** 2 
        
        opti.minimize(objective)

        # Set solver options for debugging
        opti.solver('ipopt', {'ipopt': {'print_level': 0, 'tol': 1e-6}})
        sol = opti.solve()

        # Update to next state
        a, omega = sol.value(A[0]), sol.value(Omega[0])
        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # print("path_points: ", path_points)
        # print("X: ", sol.value(X))
        # print("Y: ", sol.value(Y))
        # print("theta: ", sol.value(theta))
        # print("V: ", sol.value(V))
        # print("A: ", sol.value(A))
        # print("Delta: ", sol.value(Delta))
        # print("Omega: ", sol.value(Omega))
        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        return a, delta0 + omega 
    
    def compute(self, state: VehicleState, component: Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT

        curr_state = state.pose.x, state.pose.y, state.pose.yaw, state.v, state.front_wheel_angle
       
        path_points = []

        if self.path is None:
            raise RuntimeError("Behavior without path not implemented")

        if self.path.frame != state.pose.frame:
            print("Transforming path from", self.path.frame.name, "to", state.pose.frame.name)
            self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
        if self.trajectory is not None:
            if self.trajectory.frame != state.pose.frame:
                print("Transforming trajectory from", self.trajectory.frame.name, "to", state.pose.frame.name)
                self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

        closest_dist,self.closest_parameter = self.trajectory.closest_point_local((state.pose.x, state.pose.y),[self.closest_parameter,self.closest_parameter+3.0])

        if self.fixed:
            ref_trajectory = self.path_with_angles
        else:
            if self.trajectory.yaws is None:
                ref_trajectory = compute_headings(self.trajectory)
            else:
                ref_trajectory = self.trajectory

        for i in range(1,self.N+1):
            desired_parameter = self.closest_parameter+i*self.dt
            if ref_trajectory.yaws is None:
                position = ref_trajectory.eval(desired_parameter)[:2]
                yaw = ref_trajectory.eval(desired_parameter)[2]
                
            else: 
                position = ref_trajectory.eval(desired_parameter)
                yaw = ref_trajectory.eval_yaw(desired_parameter)

            if desired_parameter >= ref_trajectory.domain()[1]:
                velocity = 0
            else:   
                velocity = ref_trajectory.eval_derivative(desired_parameter)
            # velocity = trajectory.eval_derivative(desired_parameter)
            velocity = np.linalg.norm(velocity)
            path_points.append([position[0], position[1], yaw, velocity])
     
        path_points = np.array(path_points)
        # print([x_start, y_start, theta_start, v_start, wheel_angle_start])
        # print(path_points)

        accel, wheel_angle = self.setup_mpc(path_points, curr_state)
        return accel, wheel_angle


class MPCTrajectoryTracker(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.MPC = MPC(**args)
        self.vehicle_interface = vehicle_interface
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.start_time = time.time()
        self.set_once = 1

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['vehicle','trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle : VehicleState, trajectory: Trajectory):

        start_time = time.perf_counter()
        self.MPC.set_path(trajectory)
        if self.set_once:
            self.MPC.set_path_with_angles()
            self.set_once = 0
        accel, wheel_angle = self.MPC.compute(vehicle)
        print("acceleration: ", accel)
        print("wheel_angle", wheel_angle)
        
        if time.time() - self.start_time < 5: # 5s for A* to find the path
            wheel_angle = 0
            accel = 0

        steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))

        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.4f} seconds")

    def healthy(self):
        return True

