from ...mathutils.control import PID
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
import casadi as ca
#Vehicle Param
L = 1.75 # Wheelbase
delta_max = np.pi/4 # max steering angle  
safety_margin = 0.1  
dt = 0.5

# Setup the MPC problem 
def setup_mpc(N, dt, L, x0, y0, theta0, v0, x_goal, y_goal, theta_goal, v_goal, obstacles):
    """
    Setup and solve the MPC problem.
    Returns the first control inputs (v, delta) from the optimized trajectory.
    """

    # Steering angle and velocity limits
    delta_min, delta_max = -np.pi/4, np.pi/4
    omega_min, omega_max = -0.2, 0.2

    a_min, a_max = -0.5, 0.5
    v_min, v_max = -1, 1

    v0 = np.clip(v0, v_min, v_max) # TODO: clip to avoid constrain issues, should be handled more carefully

    opti = ca.Opti()  # Create an optimization problem

    # State
    # TODO: Add second order dynamics for steering angle
    X = opti.variable(4, N+1)  # [x, y, theta, v]
    # X = opti.variable(5, N+1)  # [x, y, theta, v, delta]
    U = opti.variable(2, N)    # [a, delta/omega]

    # Initial constraints
    opti.subject_to(X[:,0] == [x0, y0, theta0, v0])
    # opti.subject_to(X[:,0] == [x0, y0, theta0, v0, delta0])

    # Dynamics constraints
    for k in range(N):
        x_next = X[0,k] + X[3,k]*ca.cos(X[2,k])*dt
        y_next = X[1,k] + X[3,k]*ca.sin(X[2,k])*dt
        v_next = X[3,k] + U[0,k]*dt
        theta_next = X[2,k] + X[3,k]/L*ca.tan(U[1,k])*dt
        # theta_next = X[2,k] + X[3,k]/L*ca.tan(X[4,k])*dt
        # delta_next = X[4,k] + U[1,k]*dt
        
        opti.subject_to(X[0,k+1] == x_next)
        opti.subject_to(X[1,k+1] == y_next)
        opti.subject_to(X[2,k+1] == theta_next)
        opti.subject_to(X[3,k+1] == v_next)
        # opti.subject_to(X[4,k+1] == delta_next)

        # Obstacle constraints
        # TODO: Add soft constraints
        penalty_scale = 2000
        obstacle_penalty = 0
        for obs in obstacles:
            obs_x, obs_y, obs_w, obs_l, obs_h = obs
            distance_squared = (X[0,k] - obs_x)**2 + (X[1,k] - obs_y)**2
            # opti.subject_to(distance_squared >= obs_w**2)
            obstacle_penalty += penalty_scale / (distance_squared + 1)


    # Control costraints
    opti.subject_to(opti.bounded(a_min, U[0,:], a_max))
    opti.subject_to(opti.bounded(delta_min, U[1,:], delta_max))
    opti.subject_to(opti.bounded(v_min, X[3,:], v_max))
    # opti.subject_to(opti.bounded(omega_min, U[1,:], omega_max))
    # opti.subject_to(opti.bounded(delta_min, X[4,:], delta_max))
    

    # objective
    # objective = ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal])
    objective = ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal]) + obstacle_penalty
    opti.minimize(objective)

    # Solver
    opts = {"ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", opts)

    sol = opti.solve()
    x_sol = sol.value(X[0,:])
    y_sol = sol.value(X[1,:])
    theta_sol = sol.value(X[2,:])
    v_sol = sol.value(X[3,:])[0]
    delta_sol = sol.value(U[1,:])[0]
    acc_sol = sol.value(U[0,:])[0]
    # delta_sol = sol.value(X[4,:])[0]
    # omega_sol = sol.value(U[1,:])[0]

    return delta_sol, acc_sol

class MPCTrajectoryPlanner(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.vehicle_interface = vehicle_interface
        self.N = 10
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def update(self, state : AllState):
        agents = state.agents
        vehicle = state.vehicle
        route = state.route
        x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v
        # x_start, y_start, theta_start, v_start, delta_start= vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v,vehicle.front_wheel_angle
        x_goal, y_goal, theta_goal, v_goal = *route.points[-1], 0., 0.

        agents = [a.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
        agents = [[a.pose.x, a.pose.y, *a.dimensions] for a in agents]

        wheel_angle, accel = setup_mpc(self.N, dt, L, x_start, y_start, theta_start, v_start, \
                                              x_goal, y_goal, theta_goal, v_goal, agents)
        # wheel_angle, accel = setup_mpc(self.N, dt, L, x_start, y_start, theta_start, v_start, delta_start,\
        #                                       x_goal, y_goal, theta_goal, v_goal, agents)
        print(wheel_angle, accel)
        if np.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2) <= 0.1: # threshold for reaching the goal
            wheel_angle = 0
            accel = 0
        steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))


    def healthy(self):
        return True