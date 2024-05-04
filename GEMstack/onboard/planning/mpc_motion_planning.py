from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...state.vehicle import VehicleState,ObjectFrameEnum
from ...state.trajectory import Path,Trajectory,compute_headings
from ...state.agent import AgentEnum, AgentState
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np
import casadi as ca
from typing import List

class MPC:
    def __init__(self):
        # Vehicle Param
        self.L = settings.get('vehicle.geometry.wheelbase') # Wheelbase
        self.DELTA_MIN = settings.get('MPC_planner.MPC_planner.delta_min')
        self.DELTA_MAX = settings.get('MPC_planner.MPC_planner.delta_max')
        self.A_MIN = settings.get('MPC_planner.MPC_planner.a_min')
        self.A_MAX = settings.get('MPC_planner.MPC_planner.a_max')
        self.V_MIN = settings.get('MPC_planner.MPC_planner.v_min')
        self.V_MAX = settings.get('MPC_planner.MPC_planner.v_max')

        # MPC Param
        self.OBSTACLE_PENALTY = settings.get('MPC_planner.MPC_planner.penalty.obstacle_penalty')
        self.LANE_BOUND_PENALTY = settings.get('MPC_planner.MPC_planner.penalty.lane_bond_penalty')
        self.A_SQUARED = settings.get('MPC_planner.MPC_planner.penalty.ellipse_a_squared')
        self.B_SQUARED = settings.get('MPC_planner.MPC_planner.penalty.ellipse_b_squared')

        self.WEIGHT_G = settings.get('MPC_planner.MPC_planner.weight.w_g')
        self.WEIGHT_O = settings.get('MPC_planner.MPC_planner.weight.w_o')
        self.WEIGHT_L = settings.get('MPC_planner.MPC_planner.weight.w_l')

        self.N = settings.get('MPC_planner.MPC_planner.horizon_steps')
        self.dt = settings.get('MPC_planner.MPC_planner.dt')

        self.healthy = True

    # Setup the MPC problem 
    def compute(self, x0, y0, theta0, v0, x_goal, y_goal, theta_goal, v_goal, agents, lane_bound = [-1,1]):
        """
        Setup and solve the MPC problem.
        Returns the first control inputs (v, delta) from the optimized trajectory.
        """

        # Steering angle, accl and velocity limits
        delta_min, delta_max = self.DELTA_MIN, self.DELTA_MAX
        a_min, a_max = self.A_MIN, self.A_MAX
        v_min, v_max = self.V_MIN, self.V_MAX
        lane_left, lane_rigth = lane_bound

        v0 = np.clip(v0, v_min, v_max) 

        opti = ca.Opti()  # Create an optimization problem

        # State
        X = opti.variable(4, self.N+1)  # [x, y, theta, v]
        U = opti.variable(2, self.N)    # [a, delta]

        # Initial constraints
        opti.subject_to(X[:,0] == [x0, y0, theta0, v0])

        # Dynamics constraints
        for k in range(self.N):
            x_next = X[0,k] + X[3,k]*ca.cos(X[2,k])*self.dt
            y_next = X[1,k] + X[3,k]*ca.sin(X[2,k])*self.dt
            v_next = X[3,k] + U[0,k]*self.dt
            theta_next = X[2,k] + X[3,k]/self.L*ca.tan(U[1,k])*self.dt
            
            opti.subject_to(X[0,k+1] == x_next)
            opti.subject_to(X[1,k+1] == y_next)
            opti.subject_to(X[2,k+1] == theta_next)
            opti.subject_to(X[3,k+1] == v_next)

            # Obstacle constraints
            obstacle_penalty = 0
            obstacle_penalty_scale = self.OBSTACLE_PENALTY
            
            a_squared = self.A_SQUARED # control the x direction in the ellipse obstacle penalty
            b_squared = self.B_SQUARED # control the y direction in the ellipse obstacle penalty
            
            penalty_scale_lane = self.LANE_BOUND_PENALTY  # Penalty for going out of lane bounds
            lane_bounds_penalty = ca.sum1(ca.fmax(0, lane_left - X[1,k])**2 + ca.fmax(0, X[1,k] - lane_rigth)**2) * penalty_scale_lane

            for agent in agents:
                obs_x, obs_y, obs_w, obs_l, obs_h = agent.pose.x, agent.pose.y, *agent.dimensions
                obs_vx, obs_vy, obs_vz = agent.velocity_local()

                # soft constraint
                # distance_squared = (X[0,k] - (obs_x + obs_vx*dt*k))**2 + (X[1,k] - (obs_y + obs_vy*dt*k))**2
                # obstacle_penalty += penalty_scale / (distance_squared + 1)
                ellipse_distance = (X[0,k] - (obs_x + obs_vx*self.dt*k))**2 / a_squared + (X[1,k] - (obs_y + obs_vy*self.dt*k))**2 / b_squared
                obstacle_penalty += obstacle_penalty_scale / (ellipse_distance + 1)
                
                # hard constraint
                # opti.subject_to(distance_squared >= obs_w**2)

        # Control costraints
        opti.subject_to(opti.bounded(a_min, U[0,:], a_max))
        opti.subject_to(opti.bounded(delta_min, U[1,:], delta_max))
        opti.subject_to(opti.bounded(v_min, X[3,:], v_max))

        # weight
        w_g = self.WEIGHT_G # goal follw weight
        w_o = self.WEIGHT_O # obstacle avoidance weight
        w_l = self.WEIGHT_L # lane boundary weight
    
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
            self.healthy = False
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


class MPC_ACC_LKA(object):
    def __init__(self):
        self.horizon_steps = settings.get('MPC_planner.MPC_planner.horizon_steps', 10)
        self.dt = settings.get('MPC_planner.MPC_planner.dt', 0.1) # s
        self.rate = settings.get('MPC_planner.MPC_planner.rate', 10.0) # Hz

        self.wheelbase  = settings.get('vehicle.geometry.wheelbase') # m
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')] # radians

        self.max_speed = settings.get('vehicle.limits.max_speed') # m/s
        self.max_reverse_speed = settings.get('vehicle.limits.max_reverse_speed') # m/s
        self.max_accel = settings.get('vehicle.limits.max_acceleration') # m/s^2
        self.max_decel = settings.get('vehicle.limits.max_deceleration') # m/s^2

        self.min_follow_dist = settings.get('MPC_planner.MPC_planner.min_follow_dist', 6.0) # meters
        self.min_obst_dist = settings.get('MPC_planner.MPC_planner.safe_dist', 3.0) # meters
        self.time_headway = settings.get('MPC_planner.MPC_planner.time_headway', 5.0) # seconds
        self.follow_degree_range = settings.get('MPC_planner.MPC_planner.follow_degree_range', 10.0) # degrees
        self.front_relevance_dist = settings.get('model_predictive_controller.front_relevance_dist', 50.0) # meters

        self.lane_penalty_constant = settings.get('model_predictive_controller.lane_penalty_constant', 1)
        self.lane_centerline = None # list of tuples or None

        self.healthy = True # Track whether a solution was found

    def compute(self, x0, y0, theta0, v0, obstacles):
        """
        Setup and solve the MPC problem.
        Returns the first control inputs (v, delta) from the optimized trajectory.

        Lane keeping: get the closest lane by centerline. Then, at each point in the horizon,
        get the segment closest to the current point and try to aim the yaw toward that line
        segment and minimize lateral distance.

        Adaptive cruise control: identify a vehicle in front of the car and aim to a
        minimum distance plus a constant time headway behind that vehicle.
        """

        # Clip velocity to avoid constraint issues
        # In the future, this should be handled more carefully
        v0 = np.clip(v0, -1 * self.max_reverse_speed, self.max_speed) 

        opti = ca.Opti()  # Create an optimization problem

        # State
        X = opti.variable(4, self.horizon_steps+1)  # [x, y, theta, v]
        U = opti.variable(2, self.horizon_steps)    # [a, delta/omega]

        # Initial constraints
        opti.subject_to(X[:,0] == [x0, y0, theta0, v0])

        # Dynamics constraints
        obstacle_penalty = 0
        lane_penalty = 0
        for k in range(self.horizon_steps):
            x_next = X[0,k] + X[3,k] * ca.cos(X[2,k]) * self.dt
            y_next = X[1,k] + X[3,k] * ca.sin(X[2,k]) * self.dt
            v_next = X[3,k] + U[0,k] * self.dt
            theta_next = X[2,k] + X[3,k]/self.wheelbase * ca.tan(U[1,k]) * self.dt
            theta_next = self.wrap_to_pi(theta_next)
            
            opti.subject_to(X[0,k+1] == x_next)
            opti.subject_to(X[1,k+1] == y_next)
            opti.subject_to(X[2,k+1] == theta_next)
            opti.subject_to(X[3,k+1] == v_next)

            # Lane keeping
            if self.lane_centerline:
                min_dist = ca.inf
                best_yaw = 0

                px, py = X[0, k], X[1, k]
                for i in range(len(self.lane_centerline)-1):
                    ax, ay, _ = self.lane_centerline[i]
                    bx, by, _ = self.lane_centerline[i+1]

                    dist, point = self.segment_distance(px, py, ax, ay, bx, by)
                    segment_yaw = ca.atan2(by - ay, bx - ax)

                    update_cond = dist < min_dist
                    min_dist = ca.if_else(update_cond, dist, min_dist)
                    best_yaw = ca.if_else(update_cond, segment_yaw, best_yaw)
                
                lane_penalty += ca.sumsqr(X[2,k] - best_yaw)
                lane_penalty += ca.sumsqr(px - point[0]) + ca.sumsqr(py - point[1])


            # Obstacle constraints
            for obs in obstacles:
                obs_type, obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_h = obs
                obs_x = obs_x + (obs_vx * self.dt * k)
                obs_y = obs_y + (obs_vy * self.dt * k)

                # Get the displacement between us and them
                disp_x = obs_x - X[0, k]
                disp_y = obs_y - X[1, k]
                disp_angle = ca.atan2(disp_y, disp_x) * 180.0 / ca.pi

                # Calculate distance
                distance_squared = disp_x**2 + disp_y**2

                # Check if the obstacle is a car
                is_car = (obs_type == AgentEnum.CAR or obs_type == AgentEnum.LARGE_TRUCK or obs_type == AgentEnum.MEDIUM_TRUCK)

                # Check if they are in front of us (within range)
                in_front = ca.logic_and(ca.fabs(disp_angle - self.wrap_to_pi(X[2, k])) <= self.follow_degree_range,
                                        distance_squared <= self.front_relevance_dist**2)

                # Condition to apply car penalty
                car_in_front = ca.logic_and(is_car, in_front)

                # Keep a constant time headway distance based on current speed
                desired_dist = self.min_follow_dist + self.time_headway * X[3, k]
                real_dist = ca.sqrt(distance_squared)

                # Calculate penalty for car in front within desired distance
                car_penalty = ca.sumsqr(desired_dist - real_dist)

                # Apply car penalty if condition is met, otherwise apply default penalty
                penalty_to_apply = ca.if_else(car_in_front, car_penalty, 0)

                # Add the appropriate penalty
                obstacle_penalty += penalty_to_apply

                opti.subject_to(distance_squared >= self.min_obst_dist**2)

        # Control costraints
        opti.subject_to(opti.bounded(-1 * self.max_decel, U[0,:], self.max_accel))
        opti.subject_to(opti.bounded(self.steering_angle_range[0], U[1,:], self.steering_angle_range[1]))
        opti.subject_to(opti.bounded(-1 * self.max_reverse_speed, X[3,:], self.max_speed))

        # Objective
        objective = obstacle_penalty + lane_penalty
        opti.minimize(objective)

        # Solver
        opts = {"ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", opts)

        try:
            sol = opti.solve()
        except:
            self.healthy = False
            return 0, 0

        delta_sol = sol.value(U[1,:])[0]
        acc_sol = sol.value(U[0,:])[0]

        return delta_sol, acc_sol

    def wrap_to_pi(self, angle):
        """Wrap angle in radians to [-pi, pi]"""
        return ca.fmod(angle + ca.pi, 2 * ca.pi) - ca.pi
    
    def segment_distance(self, px, py, ax, ay, bx, by):
        # Create CasADi variables for input
        p = ca.vertcat(px, py)
        a = ca.vertcat(ax, ay)
        b = ca.vertcat(bx, by)
        
        # Calculate vectors
        ab = b - a
        ap = p - a
        
        # Project point onto line segment
        ab_mag = ca.dot(ab, ab)
        proj_scalar = ca.dot(ap, ab) / ab_mag
        proj_point = a + ca.fmax(ca.fmin(proj_scalar, 1), 0) * ab
        
        # Distance to the closest point on the segment
        distance = ca.norm_2(p - proj_point)
        
        return distance, proj_point
    
class MPCTrajectoryPlanner(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.vehicle_interface = vehicle_interface
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.safe_dist = settings.get('MPC_planner.MPC_planner.safe_dist')
        self._rate = settings.get('MPC_planner.MPC_planner.rate')

        if "acc_lka" in args and args["acc_lka"]:
            self.mpc = MPC_ACC_LKA()
        else:
            self.mpc = MPC()

    def rate(self):
        return self._rate

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def update(self, state : AllState):
        # Distinguish between versions of the planner
        if type(self.mpc) == MPC:
            agents = state.agents
            vehicle = state.vehicle
            lane_goal = state.lane_goal
            goal = lane_goal.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
            x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v
            x_goal, y_goal, theta_goal, v_goal = goal.x, goal.y, 0., 0.

            lane_bound = state.lane_bound

            agents = [a.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
            obstacle = [[a.pose.x, a.pose.y, *a.dimensions] for a in agents]

            collision_dis = find_closest_agent(obstacle, pose = [vehicle.pose.x, vehicle.pose.y])
            if collision_dis > self.safe_dist:
                wheel_angle, accel = self.mpc.compute(x_start, y_start, theta_start, v_start, \
                                                    x_goal, y_goal, theta_goal, v_goal, agents, lane_bound)
                # print(wheel_angle, accel)
                if np.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2 + (theta_start - theta_goal)**2) <= 0.1: # threshold for reaching the goal
                    wheel_angle = 0
                    accel = 0
            
            elif collision_dis <= self.safe_dist:
                accel = self.mpc.A_MIN
                wheel_angle = 0
                if vehicle.v == 0:
                    accel = 0
                    wheel_angle = 0

            print("collision distance: ", collision_dis)
            steering_angle = np.clip(front2steer(wheel_angle), self.steering_angle_range[0], self.steering_angle_range[1])
            self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))

        else:
            # Put all agents in the same frame as the lanes
            vehicle = state.vehicle.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, start_pose_abs=state.start_vehicle_pose)
            x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v

            agents = [a.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, start_pose_abs=state.start_vehicle_pose) for a in state.agents.values()]
            agents = [[a.type, a.pose.x, a.pose.y, a.velocity[0], a.velocity[1], *a.dimensions] for a in agents]

            # Get the current lane segment
            curr_lane = state.roadgraph.get_current_lane(state.vehicle)
            if curr_lane:
                min_dist = float('inf')
                seg_idx = 0
                for idx, seg in enumerate(state.roadgraph.lanes[curr_lane].center.segments):
                    for i in range(len(seg)-1):
                        dist, _ = transforms.point_segment_distance((x_start, y_start, 0), seg[i], seg[i+1])
                        if dist < min_dist:
                            min_dist = dist
                            seg_idx = idx

                self.mpc.lane_centerline = state.roadgraph.lanes[curr_lane].center.segments[seg_idx]

            # Compute the controls
            wheel_angle, accel = self.mpc.compute(x_start, y_start, theta_start, v_start, agents)
            print(f"Wheel angle: {wheel_angle}, Acceleration: {accel}")

            steering_angle = np.clip(front2steer(wheel_angle), self.mpc.steering_angle_range[0], self.mpc.steering_angle_range[1])
            self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel, steering_angle, vehicle))


    def healthy(self):
        return self.mpc.healthy
