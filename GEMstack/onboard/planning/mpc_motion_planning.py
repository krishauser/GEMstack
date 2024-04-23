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

# Vehicle Param
L = settings.get('vehicle.geometry.wheelbase') # Wheelbase
DELTA_MIN = settings.get('MPC_planner.MPC_planner.delta_min')
DELTA_MAX = settings.get('MPC_planner.MPC_planner.delta_max')
A_MIN = settings.get('MPC_planner.MPC_planner.a_min')
A_MAX = settings.get('MPC_planner.MPC_planner.a_max')
V_MIN = settings.get('MPC_planner.MPC_planner.v_min')
V_MAX = settings.get('MPC_planner.MPC_planner.v_max')

# MPC Param
OBSTACLE_PENALTY = settings.get('MPC_planner.MPC_planner.penalty.obstacle_penalty')
LANE_BOUND_PENALTY = settings.get('MPC_planner.MPC_planner.penalty.lane_bond_penalty')
A_SQUARED = settings.get('MPC_planner.MPC_planner.penalty.ellipse_a_squared')
B_SQUARED = settings.get('MPC_planner.MPC_planner.penalty.ellipse_b_squared')

WEIGHT_G = settings.get('MPC_planner.MPC_planner.weight.w_g')
WEIGHT_O = settings.get('MPC_planner.MPC_planner.weight.w_o')
WEIGHT_L = settings.get('MPC_planner.MPC_planner.weight.w_l')


# Setup the MPC problem 
def setup_mpc(N, dt, L, x0, y0, theta0, v0, x_goal, y_goal, theta_goal, v_goal, agents):
    """
    Setup and solve the MPC problem.
    Returns the first control inputs (v, delta) from the optimized trajectory.
    """

    # Steering angle, accl and velocity limits
    delta_min, delta_max = DELTA_MIN, DELTA_MAX
    a_min, a_max = A_MIN, A_MAX
    v_min, v_max = V_MIN, V_MAX
    lane_left, lane_rigth = -1,1

    v0 = np.clip(v0, v_min, v_max) 

    opti = ca.Opti()  # Create an optimization problem

    # State
    X = opti.variable(4, N+1)  # [x, y, theta, v]
    U = opti.variable(2, N)    # [a, delta]

    # Initial constraints
    opti.subject_to(X[:,0] == [x0, y0, theta0, v0])

    # Dynamics constraints
    for k in range(N):
        x_next = X[0,k] + X[3,k]*ca.cos(X[2,k])*dt
        y_next = X[1,k] + X[3,k]*ca.sin(X[2,k])*dt
        v_next = X[3,k] + U[0,k]*dt
        theta_next = X[2,k] + X[3,k]/L*ca.tan(U[1,k])*dt
        
        opti.subject_to(X[0,k+1] == x_next)
        opti.subject_to(X[1,k+1] == y_next)
        opti.subject_to(X[2,k+1] == theta_next)
        opti.subject_to(X[3,k+1] == v_next)

        # Obstacle constraints
        obstacle_penalty = 0
        obstacle_penalty_scale = OBSTACLE_PENALTY
        
        a_squared = A_SQUARED # control the x direction in the ellipse obstacle penalty
        b_squared = B_SQUARED # control the y direction in the ellipse obstacle penalty
        
        penalty_scale_lane = LANE_BOUND_PENALTY  # Penalty for going out of lane bounds
        lane_bounds_penalty = ca.sum1(ca.fmax(0, lane_left - X[1,k])**2 + ca.fmax(0, X[1,k] - lane_rigth)**2) * penalty_scale_lane

        for agent in agents:
            obs_x, obs_y, obs_w, obs_l, obs_h = agent.pose.x, agent.pose.y, *agent.dimensions
            obs_vx, obs_vy, obs_vz = agent.velocity_local()

            # soft constraint
            # distance_squared = (X[0,k] - (obs_x + obs_vx*dt*k))**2 + (X[1,k] - (obs_y + obs_vy*dt*k))**2
            # obstacle_penalty += penalty_scale / (distance_squared + 1)
            ellipse_distance = (X[0,k] - (obs_x + obs_vx*dt*k))**2 / a_squared + (X[1,k] - (obs_y + obs_vy*dt*k))**2 / b_squared
            obstacle_penalty += obstacle_penalty_scale / (ellipse_distance + 1)
            
            # hard constraint
            # opti.subject_to(distance_squared >= obs_w**2)

    # Control costraints
    opti.subject_to(opti.bounded(a_min, U[0,:], a_max))
    opti.subject_to(opti.bounded(delta_min, U[1,:], delta_max))
    opti.subject_to(opti.bounded(v_min, X[3,:], v_max))

    # weight
    w_g = WEIGHT_G # goal follw weight
    w_o = WEIGHT_O # obstacle avoidance weight
    w_l = WEIGHT_L # lane boundary weight
   
    # objective
    objective = w_g * ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal]) + w_o * obstacle_penalty + w_l * lane_bounds_penalty
    opti.minimize(objective)

    # Solver
    opts = {"ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", opts)
    try: 
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
    except RuntimeError as e:  
        if "Infeasible_Problem_Detected" in str(e):
            return 0,0 

def find_closest_agent(agents, pose):
    min_distance = float('inf')  
    closest_agent = None 

    for agent in agents:
        # Calculate distance between the agent's position and the car's position
        distance = ((agent[0] - pose[0]) ** 2 + (agent[1] - pose[1]) ** 2) ** 0.5
        # Update the closest agent if a closer one is found
        if distance < min_distance:
            min_distance = distance
            closest_agent = agent

    return min_distance
   

class MPCTrajectoryPlanner(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.vehicle_interface = vehicle_interface
        self.N = settings.get('MPC_planner.MPC_planner.N')
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.safe_dist = settings.get('MPC_planner.MPC_planner.safe_dist')
        self.dt = settings.get('MPC_planner.MPC_planner.dt')
        self._rate = settings.get('MPC_planner.MPC_planner.rate')

    def rate(self):
        return self._rate

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def update(self, state : AllState):
        agents = state.agents
        vehicle = state.vehicle
        # route = state.route
        parking_slot = state.parking_slot
        goal = parking_slot.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose)
        x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v
        x_goal, y_goal, theta_goal, v_goal = goal.x, goal.y, 0., 0.

        agents = [a.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
        obstacle = [[a.pose.x, a.pose.y, *a.dimensions] for a in agents]

        collision_dis = find_closest_agent(obstacle, pose = [vehicle.pose.x, vehicle.pose.y])
        if collision_dis > self.safe_dist:
            wheel_angle, accel = setup_mpc(self.N, self.dt, L, x_start, y_start, theta_start, v_start, \
                                                x_goal, y_goal, theta_goal, v_goal, agents)
            # print(wheel_angle, accel)
            if np.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2 + (theta_start - theta_goal)**2) <= 0.1: # threshold for reaching the goal
                wheel_angle = 0
                accel = 0
        
        elif collision_dis <= self.safe_dist:
            accel = A_MIN
            wheel_angle = 0
            if vehicle.v == 0:
                accel = 0
                wheel_angle = 0

        print("collision distance: ", collision_dis)
        steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))


    def healthy(self):
        return True
