from typing import List, Tuple
from ..component import Component
from ...utils import serialization, settings
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...mathutils import collisions
from ...mathutils.transforms import normalize_vector
from .reeds_shepp_heuristic import path_length, get_optimal_path, eval_path, rad2deg, precompute
from .obstacle_heuristic import obstacle_heuristic
from ...state.intent import VehicleIntent,VehicleIntentEnum
import os
import copy
from time import time
from dataclasses import replace
from queue import PriorityQueue
import numpy as np
import math
from ...state.physical_object import ObjectFrameEnum, convert_point

class StaticRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, frame : str = 'start'):
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension",ext)

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route

class DummyRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, start : List[float], end : List[float]):
        self.route = Route(frame=ObjectFrameEnum.START,points=[start[:2],end[:2]])

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route
    
class Node:
    def __init__(self,state,cost,heuristic=0.,parent=None):
        self.state = state
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent
    
    def f(self):
        return self.cost + self.heuristic

    def __lt__(self,other):
        return self.f() < other.f()
    

class PickupDropoffRoutePlanner(Component):
    def __init__(self):
        self.pullover_route = None
        self.start_vehicle_pose = None

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self, state : AllState):
        current_intent = state.intent.intent
        route = Route(frame=ObjectFrameEnum.CURRENT,points=[[0.0,0]])

        if(current_intent is VehicleIntentEnum.DRIVING):
            route = Route(frame=ObjectFrameEnum.CURRENT,points=[[0,0],[10,0]])

        elif(current_intent is VehicleIntentEnum.PULL_OVER):

            # TODO: An option should be added to use the A* planner here. 

            if(self.pullover_route is None):
                intent_current_frame = state.intent.to_frame(ObjectFrameEnum.CURRENT, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
                pts = self.create_pullover_trajectory_from_endpoint(intent_current_frame.pullover_target[0], intent_current_frame.pullover_target[1])
                p_route = Route(frame=ObjectFrameEnum.CURRENT,points=pts)
                self.pullover_route  = p_route.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
            route = self.pullover_route

        return route

    def create_pullover_trajectory(self, distance_forward, distance_right, steps=20, drive_length=4, parking_length=4):
        res = []
        for i in range(int(steps+1)):
            u = i / steps
            res.append([u * drive_length, 0])

        for i in range(steps+1):
            curr_forward = distance_forward * (i / (steps))
            curr_right = (-1 * math.cos((math.pi*curr_forward)/distance_forward) * distance_right)/2
            curr_right = curr_right + (distance_right/2)

            res.append([curr_forward + drive_length, curr_right])

        last_res = res[-1]
        for i in range(int(steps+1)):
            u = i / steps
            res.append([last_res[0]+(u*parking_length), last_res[1]])
        
        return res
    
    def create_pullover_trajectory_from_endpoint(self, endpoint_forward, endpoint_right):
        distance_forward = 4
        distance_right = endpoint_right
        parking_length = 2
        drive_length = endpoint_forward - (distance_forward + parking_length)
        return self.create_pullover_trajectory(distance_forward=distance_forward, distance_right=distance_right,\
                                                drive_length=drive_length, parking_length=parking_length)

    
class SearchNavigationRoutePlanner(Component):
    """Returns a straight line as the desired route."""
    def __init__(self):
        # start and end are [x, y, yaw, speed, steer] in the START frame
        self._rate = settings.get('A_star_planner.search_planner.rate')
        
        self.v = settings.get('A_star_planner.search_planner.velocity')
        self.L = settings.get('vehicle.geometry.wheelbase')
        self.wheel_max = settings.get('A_star_planner.search_planner.max_wheel_angle')
        turn_radius = self.L/np.tan(self.wheel_max)
        self.turn_radius = turn_radius

        self.N_controls = settings.get('A_star_planner.search_planner.N_sample_controls')
        self.dt = settings.get('A_star_planner.search_planner.dt')

        self.target_threshold = settings.get('A_star_planner.search_planner.target_threshold')
        self.RS_threshold = settings.get('A_star_planner.search_planner.RS_threshold')
        self.RS_resolution = settings.get('A_star_planner.search_planner.RS_resolution')
        self.RS_p = settings.get('A_star_planner.search_planner.RS_prob')

        self.backward_cost_scale = settings.get('A_star_planner.search_planner.backward_cost_scale')
        self.gear_cost = settings.get('A_star_planner.search_planner.gear_cost')

        self.smooth_threshold = settings.get('A_star_planner.search_planner.smooth_threshold')

        self.route = None
        self.last_path = None
        self.last_end = None
        self.LATERAL_DISTANCE_BUFFER = .5
        self.LONGITUDINAL_DISTANCE_BUFFER = .5

    def state_inputs(self):
        # return ['vehicle', 'roadgraph']
        return ['all'] # Temporary for collision detection, should be replaced by the roadgraph

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return self._rate
    
    def update(self, state : AllState):
        # We can use agents to detect collisions, just like hw3
        # TODO: Shoule be replaced by the roadgraph to follow the GEMstack decision-making graph
        vehicle = copy.deepcopy(state.vehicle)
        agents = state.agents
        agents = [a.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]

        if True: # Currently only parking
            parking_slot = state.parking_slot
            parking_slot = parking_slot.to_frame(ObjectFrameEnum.START, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
            start = [vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, 0]
            end = [parking_slot.x, parking_slot.y, parking_slot.yaw, 0]
        elif False: # Some other task
            pass

        self.setup_search(start, end)

        def check_constraints(state):
            s = self.state2s(state)
            if s[0] < 0 or s[0] >= self.s_bound[0] or s[1] < 0 or s[1] >= self.s_bound[1]:
                return False
            vehicle_ = copy.deepcopy(vehicle)
            vehicle_.pose = replace(vehicle_.pose,x=state[0],y=state[1],yaw=state[2])
            vehicle_object = vehicle_.to_object()
            l,w,h = vehicle_object.dimensions
            new_l, new_w = l + 2*self.LONGITUDINAL_DISTANCE_BUFFER, w + 2*self.LATERAL_DISTANCE_BUFFER
            vehicle_object.dimensions = (new_l, new_w, h)
            vehicle_poly = vehicle_object.polygon_parent()
            for agent in agents:
                if collisions.polygon_intersects_polygon_2d(vehicle_poly, agent.polygon_parent()):
                    return False
            return True
        
        def check_path(path):
            for p in path[::5]:
                if not check_constraints(p):
                    return False
            return True
        
        # blending to handle small changes in the goal
        if self.last_end is None:
            self.last_path = None
        else:
            delta_end = np.array(self.end[:3])-np.array(self.last_end[:3])
            if np.linalg.norm(delta_end[:2]) + np.abs(delta_end[2]) > 3.0:
                self.last_path = None
            else:
                total_t = self.route.length()
                current_t = 0
                # linear blending
                for i in range(len(self.last_path)-1, 0, -1):
                    delta_p = delta_end*(1-current_t/total_t)
                    current_t += np.linalg.norm(np.array(self.last_path[i-1][:2])-np.array(self.last_path[i][:2]))
                    self.last_path[i][0] += delta_p[0]
                    self.last_path[i][1] += delta_p[1]
                    self.last_path[i][2] += delta_p[2]
                route = [i[:2] for i in self.last_path]
                yaws = [i[2] for i in self.last_path]
                gear = [i[3] for i in self.last_path]
                self.route = Route(frame=ObjectFrameEnum.START,points=route,yaws=yaws)
        self.last_end = copy.deepcopy(self.end)

        
        replan = self.last_path is None or not check_path(self.last_path)
        if self.route and self.route.closest_point([vehicle.pose.x, vehicle.pose.y])[0] > 5.0:
            replan = True
        # replan = True
        if replan: # replan when last route is not valid
            # Compute grid map
            for i in range(self.grid_map.shape[0]):
                for j in range(self.grid_map.shape[1]):
                    x, y = self.grid_resolution*i+self.x_bound[0], self.grid_resolution*j+self.y_bound[0]
                    if not check_constraints([x,y,0,0]):
                        self.grid_map[i,j] = -1
            i = round((self.end[0]-self.x_bound[0])/self.grid_resolution)
            j = round((self.end[1]-self.y_bound[0])/self.grid_resolution)
            obstacle_heuristic(self.grid_map, [i,j])

            start = [vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, 0]
            end = [*self.end[:3], 0]
            root = Node(start,0,heuristic=self.heuristic(start,end))

            queue = PriorityQueue()
            queue.put(root)

            visited = set()
            visited.add(tuple(start))

            while not queue.empty():
                current = queue.get()
                if self.distance(current.state,end) < self.target_threshold:
                    break
                elif np.random.uniform() < self.rs_prob(current.state):
                    xs,ys,ths = current.state[:3]
                    states = [xs/self.turn_radius,ys/self.turn_radius,ths]
                    xe,ye,the = end[:3]
                    statee = [xe/self.turn_radius,ye/self.turn_radius,the]
                    ps = eval_path(get_optimal_path(states,statee), current.state[:3], \
                                   radius=self.turn_radius, resolution=self.RS_resolution)
                    collision = False
                    for p in ps:
                        if not check_constraints(p):
                            collision = True
                            break
                    if not collision:
                        for p in ps:
                            cost = self.cost(current.state,p) + current.cost
                            child = Node(p,cost,heuristic=0.,parent=current)
                            current = child
                        break
                for state in self.expand(current.state):
                    s = self.state2s(state)
                    if tuple(s) not in visited and check_constraints(state):
                        visited.add(tuple(s))
                        cost = self.cost(current.state,state) + current.cost
                        child = Node(state,cost,heuristic=self.heuristic(state,end),parent=current)
                        queue.put(child)

            if self.distance(current.state,end) > self.target_threshold:
                print("Failed to find a path")

            # smooth the path
            c = current
            while c.parent is not None:
                p = c.parent.parent
                count = 5
                while p is not None and count > 0:
                    # if p and c are close enough, set c's parent to p
                    if self.distance(p.state,c.state) < self.smooth_threshold:
                        c.parent = p
                    count -= 1
                c = c.parent

            route = []
            yaws = []
            gear = []
            last_path = []
            while current is not None:
                route.append(current.state[:2])
                yaws.append(current.state[2])
                gear.append(current.state[3])
                last_path.append(current.state)
                current = current.parent
            route = route[::-1]
            yaws = yaws[::-1]
            gear = gear[::-1]
            last_path = last_path[::-1]
            self.route = Route(frame=ObjectFrameEnum.START,points=route,yaws=yaws)
            self.last_path = last_path
        
        route = self.route
        
        return route
    
    def setup_search(self, start, end):
        self.start = start
        self.end = end
        x_bound = [min(self.start[0], self.end[0])-2, max(self.start[0], self.end[0])+2]
        y_bound = [min(self.start[1], self.end[1])-2, max(self.start[1], self.end[1])+2]
        if self.last_path is not None:
            for p in self.last_path:
                x_bound = [min(x_bound[0], p[0]-2), max(x_bound[1], p[0]+2)]
                y_bound = [min(y_bound[0], p[1]-2), max(y_bound[1], p[1]+2)]
        theta_bound = [0, 2*np.pi]
        self.x_bound = x_bound
        self.y_bound = y_bound

        resolution = settings.get('A_star_planner.search_planner.resolution')
        angle_resolution = settings.get('A_star_planner.search_planner.angle_resolution')
        self.s_bound = (int((x_bound[1]-x_bound[0])/resolution), \
                        int((y_bound[1]-y_bound[0])/resolution), \
                        int(2*np.pi/angle_resolution))
        
        self.s2state = lambda s: [x_bound[0] + s[0]*(x_bound[1]-x_bound[0])/self.s_bound[0], \
                                  y_bound[0] + s[1]*(y_bound[1]-y_bound[0])/self.s_bound[1], \
                                  theta_bound[0] + s[2]*(theta_bound[1]-theta_bound[0])/self.s_bound[2], \
                                  s[3]]
        
        self.state2s = lambda state: [round((state[0]-x_bound[0])/(x_bound[1]-x_bound[0])*self.s_bound[0]), \
                                      round((state[1]-y_bound[0])/(y_bound[1]-y_bound[0])*self.s_bound[1]), \
                                      round((state[2]-theta_bound[0])/(theta_bound[1]-theta_bound[0])*self.s_bound[2])%self.s_bound[2], \
                                      state[3]]
        
        self.grid_resolution = 0.5      
        self.grid_map = np.full((int((x_bound[1]-x_bound[0])/self.grid_resolution)+1, \
                                 int((y_bound[1]-y_bound[0])/self.grid_resolution)+1), \
                                 np.inf)
        
        def sample_steer(N=self.N_controls):
            return np.random.uniform(-self.wheel_max,self.wheel_max,N)
        
        def sim(state,steer,f,dt=self.dt):
            x, y, theta, _ = state
            dtheta = self.v*np.tan(steer)/self.L
            x = x + f*np.cos(theta)*self.v*dt
            y = y + f*np.sin(theta)*self.v*dt
            theta = theta + dtheta*dt
            return [x,y,theta,f]
        
        def expand(state):
            next_states = []
            for steer in sample_steer():
                for f in [-1,1]: # -1 for backward, 1 for forward
                    next_state = sim(state,steer,f)
                    x, y, theta, _ = next_state
                    if x < x_bound[0] or x > x_bound[1] or y < y_bound[0] or y > y_bound[1]:
                        continue
                    next_states.append(next_state)
            return next_states
        self.expand = expand

        def heuristic(state1,state2,agents=[]):
            xs,ys,ths = state1[:3]
            start = [xs/self.turn_radius,ys/self.turn_radius,ths]
            xe,ye,the = state2[:3]
            end = [xe/self.turn_radius,ye/self.turn_radius,the]
            h = precompute(start,end)*self.turn_radius
            # h = path_length(get_optimal_path(start,end))*self.turn_radius
            i, j = round((state1[0]-self.x_bound[0])/self.grid_resolution), round((state1[1]-self.y_bound[0])/self.grid_resolution)
            h = max(self.grid_map[i,j]*self.grid_resolution,h)
            return h*2
        self.heuristic = heuristic

        def cost(state1,state2):
            scale = self.backward_cost_scale if state2[3] < 0 else 1.0
            c = np.linalg.norm(np.array(state1[:2])-np.array(state2[:2]))
            c = c*scale
            c += self.gear_cost if state1[3]*state2[3] < 0 else 0
            return c
        self.cost = cost

        def distance(state1,state2):
            return np.linalg.norm(np.array(state1[:3])-np.array(state2[:3])) + \
                min(abs(state1[2]-state2[2])%(2*np.pi), 2*np.pi-abs(state1[2]-state2[2])%(2*np.pi))
        self.distance = distance

        def rs_prob(state):
            if distance(state,self.end) < 0.2*self.RS_threshold:
                return 5*self.RS_p
            if distance(state,self.end) < 0.5*self.RS_threshold:
                return 2*self.RS_p
            if distance(state,self.end) < self.RS_threshold:
                return self.RS_p
            return 0.0
        self.rs_prob = rs_prob