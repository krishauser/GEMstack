# from ...mathutils.control import PID
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
#Vehicle Param

L = 2.56  # Wheelbase, GEMe4
safety_margin = 0.1  


# Setup the MPC problem 
def setup_mpc(N, dt, L, path_points, x, gear=0):
    # Constraints
    delta_max = 0.6108
    omega_max = 0.2 # TODO: What is a good value for this?
    a_max = 1.0 
    a_min = -1.0
    v_max = 2.0
    v_min = -2.0


    # Initialization
    x0, y0, theta0, v0, delta0 = x

    v0 = v0.clip(v_min, v_max) # TODO: clip to avoid constrain issues, should be handled more carefully

    # MPC setup
    opti = Opti()  

    # Control variables
    A = opti.variable(N)
    Omega = opti.variable(N)
    # State variables
    X = opti.variable(N+1)
    Y = opti.variable(N+1)
    theta = opti.variable(N+1)
    V = opti.variable(N+1)
    Delta = opti.variable(N+1)

    # Provide an initial guess
    for i in range(N):
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
    for j in range(N):
        # Dynamics
        opti.subject_to(X[j+1] == X[j] + dt * V[j] * cos(theta[j]))
        opti.subject_to(Y[j+1] == Y[j] + dt * V[j] * sin(theta[j]))
        opti.subject_to(theta[j+1] == theta[j] + dt * V[j] / L * tan(Delta[j]))
        opti.subject_to(V[j+1] == V[j] + dt * A[j])
        opti.subject_to(Delta[j+1] == Delta[j] + dt * Omega[j])
        # Control bounds
        opti.subject_to(a_min <= A[j])
        opti.subject_to(A[j] <= a_max)
        opti.subject_to(-omega_max <= Omega[j])
        opti.subject_to(Omega[j] <= omega_max)
        # State bounds
        opti.subject_to(-delta_max <= Delta[j])
        opti.subject_to(Delta[j] <= delta_max)
        opti.subject_to(V[j] <= v_max)

        opti.subject_to(v_min <= V[j])


    # Set up the optimization problem
    objective = 0
    for j in range(N):
        # TODO: Add heading objective?
        objective += ((X[j+1] - path_points[j,0])**2 + (Y[j+1] - path_points[j,1])**2)

        if path_points[j,2] is not None:
            objective += (cos(theta[j+1]) - cos(path_points[j,2]))**2
            objective += (sin(theta[j+1]) - sin(path_points[j,2]))**2

    opti.minimize(objective)

    # Set solver options for debugging
    opti.solver('ipopt', {'ipopt': {'print_level': 0, 'tol': 1e-6}})
    sol = opti.solve()

    # Update to next state
    a, omega = sol.value(A[0]), sol.value(Omega[0])
    a += sign(a)*0.2 # To balance drag
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    # print(path_points)
    # print(sol.value(X))
    # print(sol.value(Y))
    # print(sol.value(theta))
    # print("V: ", sol.value(V))
    # print(sol.value(A))
    # print(sol.value(Delta))
    # print(sol.value(Omega))
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    return a, delta0 + 2.0*omega # TODO: What is a good way to update the steering angle?


class MPCTracker(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.vehicle_interface = vehicle_interface
        self.N = 4

        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.max_a = 1.0
        self.min_a = -1.0
        self.max_d = 0.6108
        self.min_d = -0.6108
        self.dt = 1.75
        self.horizen_scale = 1.0


        self.last_progress = 0

        self.start_time = time.time()


    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['vehicle','trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle : VehicleState, trajectory: Trajectory):
        
        x_start, y_start, theta_start, v_start, wheel_angle_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v, vehicle.front_wheel_angle
        path_points = []
        closest_dist,closest_parameter = trajectory.closest_point([x_start, y_start])

        # closest_parameter = max(self.last_progress, closest_parameter)
        ind, _ = trajectory.time_to_index(closest_parameter)
        gear = trajectory.gear[ind]
        # if closest_parameter - self.last_progress < 0.02:
        #     self.horizen_scale *= 0.9
        # else:
        #     self.horizen_scale = 1.0
        # self.horizen_scale = max(min(self.horizen_scale, 1.0), 0.1)
        dt = self.dt * self.horizen_scale
        # self.last_progress = closest_parameter + 0.01
        for i in range(1,self.N+1):
            position = trajectory.eval(closest_parameter+i*dt)
            velocity = trajectory.eval_derivative(closest_parameter+i*dt)
            yaw = trajectory.eval_yaw(closest_parameter+i*dt)

            velocity = np.linalg.norm(velocity)
            path_points.append([position[0], position[1], yaw, velocity])
        
        path_points = np.array(path_points)

        # print([x_start, y_start, theta_start, v_start, wheel_angle_start])
        # print(path_points)

        accel, wheel_angle = setup_mpc(self.N, dt, L, path_points, [x_start, y_start, theta_start, v_start, wheel_angle_start], gear)

        # print(accel, wheel_angle)
        
        desired_yaw = np.arctan2(path_points[1][0],path_points[1][1])
        if desired_yaw < 0.1:
            accel = 1.2 * accel
        elif 0.1 <= desired_yaw < 0.25:
            accel = 0.6 * accel
        elif desired_yaw >= 0.25:
            accel = 0.4 * accel
        accel = max(min(accel, self.max_a), self.min_a)

        
        yaw_error = desired_yaw - vehicle.pose.yaw
        if yaw_error < 0.1:
            wheel_angle = 0.5 * wheel_angle
        elif 0.1 <= yaw_error < 0.25 : 
            wheel_angle = 1.0 * wheel_angle
        elif yaw_error >= 0.25:
            wheel_angle = 1.5 * wheel_angle
        wheel_angle = max(min(wheel_angle, self.max_d), self.min_d)

        d_remain = sqrt((vehicle.pose.x - path_points[-1][0]) ** 2 + (vehicle.pose.y - path_points[-1][1]) ** 2)
        # print("remain distance: ", d_remain)
        # if 0.2< d_remain <= 1.0:
        #     wheel_angle = 0
        #     accel = -1.0
        # elif d_remain <= 0.2:
        #     wheel_angle = 0
        #     accel = 0.0

        if time.time() - self.start_time < 5:
            wheel_angle = 0
            accel = 0


        steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))



    def healthy(self):
        return True
