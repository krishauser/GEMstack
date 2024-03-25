from typing import List, Tuple
from ..component import Component
from ...utils import serialization
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...mathutils import collisions
from ...mathutils.transforms import normalize_vector
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
    def __init__(self,s,cost,heuristic=0.,parent=None):
        self.s = s
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
        x_bound = [min(self.start[0], self.end[0])-2, max(self.start[0], self.end[0])+8]
        y_bound = [min(self.start[1], self.end[1])-2, max(self.start[1], self.end[1])+3]
        theta_bound = [0, 2*np.pi]

        resolution = 1.
        self.s_bound = (int((x_bound[1]-x_bound[0])/resolution), int((y_bound[1]-y_bound[0])/resolution), 8)
        self.s2state = lambda s: [x_bound[0] + s[0]*(x_bound[1]-x_bound[0])/self.s_bound[0], \
                                  y_bound[0] + s[1]*(y_bound[1]-y_bound[0])/self.s_bound[1], \
                                  theta_bound[0] + s[2]*(theta_bound[1]-theta_bound[0])/self.s_bound[2]]
        
        self.state2s = lambda state: [round((state[0]-x_bound[0])/(x_bound[1]-x_bound[0])*self.s_bound[0]), \
                                      round((state[1]-y_bound[0])/(y_bound[1]-y_bound[0])*self.s_bound[1]), \
                                      round((state[2]-theta_bound[0])/(theta_bound[1]-theta_bound[0])*self.s_bound[2])%8]
        
        def expand(s):
            next_s = []
            for k in range(-1,2):
                # for f in [-1,1]: # -1 for backward, 1 for forward
                for f in [1]:
                    theta = (s[2] + k)*np.pi/4
                    i = s[0] + f*round(np.cos(theta))
                    j = s[1] + f*round(np.sin(theta))
                    if i >= 0 and i < self.s_bound[0] and j >= 0 and j < self.s_bound[1]:
                        next_s.append([i,j,(s[2]+k)%8])
            return next_s
        self.expand = expand

        def heuristic(s1,s2,agents=[]):
            state1 = self.s2state(s1)
            state2 = self.s2state(s2)
            return np.linalg.norm(np.array(state1[:2])-np.array(state2[:2]))
        self.heuristic = heuristic

        def cost(s1,s2):
            state1 = self.s2state(s1)
            state2 = self.s2state(s2)
            return np.linalg.norm(np.array(state1[:2])-np.array(state2[:2])) + \
                   abs(state1[2]-state2[2])
        self.cost = cost

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

        def check_constraints(s):
            if s[0] < 0 or s[0] >= self.s_bound[0] or s[1] < 0 or s[1] >= self.s_bound[1]:
                return False
            state = self.s2state(s)
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
            start = self.state2s(self.start)
            end = self.state2s(self.end)
            root = Node(start,0,heuristic=self.heuristic(start,end))

            queue = PriorityQueue()
            queue.put(root)

            visited = set()
            visited.add(tuple(start))

            while not queue.empty():
                current = queue.get()
                if self.cost(current.s,end) < 0.5:
                    break
                for s in self.expand(current.s):
                    if tuple(s) not in visited and check_constraints(s):
                        visited.add(tuple(s))
                        cost = self.cost(current.s,s)
                        child = Node(s,cost,heuristic=self.heuristic(s,end),parent=current)
                        queue.put(child)
            
            route = []
            while current is not None:
                route.append(self.s2state(current.s)[:2])
                current = current.parent
            route = route[::-1]
            self.route = Route(frame=ObjectFrameEnum.START,points=route)
        
        route = self.route
        
        # route = Route(frame=ObjectFrameEnum.START,points=[self.start[:2]])
        return route