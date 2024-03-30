from typing import List, Tuple
from ..component import Component
from ...utils import serialization
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...mathutils import collisions
from ...mathutils.transforms import normalize_vector
from .reeds_shepp import path_length, get_optimal_path, rad2deg
import os
import copy
from time import time
from dataclasses import replace
from queue import PriorityQueue
import numpy as np


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

    
class SearchNavigationRoutePlanner(Component):
    """Returns a straight line as the desired route."""
    def __init__(self, start : List[float], end : List[float]):
        # start and end are [x, y, yaw, speed, steer] in the START frame
        self.start = start
        self.start = [0.,0.,0.,0.,0.]
        self.end = end
        x_bound = [min(self.start[0], self.end[0])-2, max(self.start[0], self.end[0])+1]
        y_bound = [min(self.start[1], self.end[1])-2, max(self.start[1], self.end[1])+3]
        theta_bound = [0, 2*np.pi]

        resolution = 0.5
        self.s_bound = (int((x_bound[1]-x_bound[0])/resolution), int((y_bound[1]-y_bound[0])/resolution), 16)
        self.s2state = lambda s: [x_bound[0] + s[0]*(x_bound[1]-x_bound[0])/self.s_bound[0], \
                                  y_bound[0] + s[1]*(y_bound[1]-y_bound[0])/self.s_bound[1], \
                                  theta_bound[0] + s[2]*(theta_bound[1]-theta_bound[0])/self.s_bound[2], \
                                  s[3]]
        
        self.state2s = lambda state: [round((state[0]-x_bound[0])/(x_bound[1]-x_bound[0])*self.s_bound[0]), \
                                      round((state[1]-y_bound[0])/(y_bound[1]-y_bound[0])*self.s_bound[1]), \
                                      round((state[2]-theta_bound[0])/(theta_bound[1]-theta_bound[0])*self.s_bound[2])%self.s_bound[2], \
                                      state[3]]
        
        v = 1.0
        L = 1.75
        steer_max = np.pi/3
        turn_radius = L/np.tan(steer_max)
        
        def sim(state,steer,f,dt=0.5):
            x, y, theta, _ = state
            dtheta = v*np.tan(steer)/L
            x = x + f*np.cos(theta)*v*dt
            y = y + f*np.sin(theta)*v*dt
            theta = theta + dtheta*dt
            return [x,y,theta,f]
        
        def expand(state):
            next_states = []
            for steer in [-steer_max,0,steer_max]:
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
            start = [xs/turn_radius,ys/turn_radius,ths]
            xe,ye,the = state2[:3]
            end = [xe/turn_radius,ye/turn_radius,the]
            h = path_length(get_optimal_path(start,end))*turn_radius
            return h*2
        self.heuristic = heuristic

        def cost(state1,state2):
            scale = 1.2 if state2[3] < 0 else 1.0
            c = np.linalg.norm(np.array(state1[:2])-np.array(state2[:2]))
            c = c*scale
            c += 3 if state1[3]*state2[3] < 0 else 0
            return c
        self.cost = cost

        def distance(state1,state2):
            return np.linalg.norm(np.array(state1[:3])-np.array(state2[:3])) + \
                min(abs(state1[2]-state2[2])%(2*np.pi), 2*np.pi-abs(state1[2]-state2[2])%(2*np.pi))
        self.distance = distance

        self.route = None
        self.LATERAL_DISTANCE_BUFFER = .5
        self.LONGITUDINAL_DISTANCE_BUFFER = .5


        print("NavigationRoutePlanner: start",start)
        print("NavigationRoutePlanner: end",end)

    def state_inputs(self):
        # return ['vehicle', 'roadgraph']
        return ['all'] # Temporary for collision detection, should be replaced by the roadgraph

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        # return 1.0
        return 0.2

    # def update(self, vehicle : VehicleState, roadgraph : Tuple[Roadmap,Roadgraph]):
    #     # TODO: Figure out what is a Roadgraph
    #     roadmap, roadgraph = roadgraph

    #     # TODO: make this a real route
    #     route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2],self.end[:2]])
    #     return route
    
    def update(self, state : AllState):
        # We can use agents to detect collisions, just like hw3
        # TODO: Shoule be replaced by the roadgraph to follow the GEMstack decision-making graph
        vehicle = copy.deepcopy(state.vehicle)
        agents = state.agents
        agents = [a.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
        # TODO: make this a real route

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
        
        if True: # always replan
            start = [*self.start[:3], 0]
            end = [*self.end[:3], 0]
            root = Node(start,0,heuristic=self.heuristic(start,end))

            queue = PriorityQueue()
            queue.put(root)

            visited = set()
            visited.add(tuple(start))

            while not queue.empty():
                current = queue.get()
                if self.distance(current.state,end) < 1.0:
                    break
                for state in self.expand(current.state):
                    s = self.state2s(state)
                    if tuple(s) not in visited and check_constraints(state):
                        visited.add(tuple(s))
                        cost = self.cost(current.state,state) + current.cost
                        child = Node(state,cost,heuristic=self.heuristic(state,end),parent=current)
                        queue.put(child)
            
            route = []
            while current is not None:
                route.append(current.state[:2])
                current = current.parent
            route = route[::-1]
            self.route = Route(frame=ObjectFrameEnum.START,points=route)
        
        route = self.route
        
        # route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2]])
        return route