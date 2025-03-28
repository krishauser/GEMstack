import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
import numpy as np
import random  
import math

class Obstacle:
    def __init__(self,x=0,y=0,r=0.2):
        self.x = x
        self.y = y
        self.radius = r

class Point:
    def __init__(self, x=0, y=0, heading = None):
        self.x = x
        self.y = y
        self.heading = heading # in radian
        self.parent = None
        self.cost = float('inf')  # Cost to reach this node

class BiRRT:
    def __init__(self, start : list, goal : list, obstacles : list, MAP_SIZE : list):
        
        self.path = []
        self.tree_from_start = []
        self.tree_from_end = []
        
        # start position (current vehicle position)
        self.start_point = Point(x = start[0],y = start[1],heading = start[2])
        
        # desire position (reverse heading for building tree) 
        # (will revert back after route found)
        self.end_point = Point(x = goal[0],y = goal[1],heading = self.angle_inverse(goal[2]))
        
        # obstacles and map boundary
        self.OBSTACLE_LIST = []
        for i in range(len(obstacles)):
            self.OBSTACLE_LIST.append(Obstacle(obstacles[i][0], obstacles[i][1], r = 0.2))
        
        # min distace of vehicle center to obstacle
        # should be roughly 1/2 of vehicle width
        self.OFFSET = 0.8 # meter
        
        # max iteration size for performing route search
        self.MAX_Iteration = 20000
        
        # step size for local planner
        self.step_size = 0.5 # meter
        
        # radius for determine neighbor node
        self.search_r = 1.3 # meter
        
        # angle limit for vehicle turning per step size
        self.heading_limit = math.pi/6 # limit the heading change in route
        
        # Map boundary
        self.MAP_X_LOW = MAP_SIZE[0] # meter
        self.MAP_X_HIGH = MAP_SIZE[1] # meter
        self.MAP_Y_LOW = MAP_SIZE[2] # meter
        self.MAP_Y_HIGH = MAP_SIZE[3] # meter
        
    def search(self):
        # initialize two tree
        self.tree_from_start.append(self.start_point)
        self.tree_from_end.append(self.end_point)
        
        # perform search within max number of iterration
        for iterration in range(self.MAX_Iteration):
            # uniformly sample a point within in the map
            sample_p = Point(random.uniform(self.MAP_X_LOW,self.MAP_X_HIGH),random.uniform(self.MAP_Y_LOW,self.MAP_Y_HIGH))
            Direction = None
            # update the tree form start or tree from end with 1/2 probability 
            rand_num = random.uniform(0.0, 1.0)
            if rand_num > 0.5:
                tree_a = self.tree_from_start
                tree_b = self.tree_from_end
                Direction = "forward"
            else:
                tree_a = self.tree_from_end
                tree_b = self.tree_from_start
                Direction = "backward"
            
            print(Direction + " AT {} Interation".format(iterration))
            # find nearest point in the tree 
            nearest_point_a = self.Nearest(tree_a, sample_p)
            # use local planner to move one step size
            new_p = self.LocalPlanner(nearest_point_a, sample_p, self.step_size)
            # check collision
            if not self.is_valid(new_p):
                continue
            # check if there exist previous point with less cost to new point
            neighbor_points = self.Neighbors(new_p,tree_a)
            min_cost = nearest_point_a.cost + self.distance(new_p,nearest_point_a)
            parent_p = nearest_point_a
            for point in neighbor_points:
                curr_cost = point.cost + self.distance(point, new_p)
                if curr_cost < min_cost:
                    min_cost = curr_cost
                    parent_p = point
            # update point's paraent                 
            new_p.cost = min_cost
            new_p.parent = parent_p
            new_p.heading = self.heading(parent_p,new_p)
            
            # check heading limit and collision
            if self.angle_diff(new_p.heading,new_p.parent.heading) > (self.heading_limit):
                continue
            if not self.is_valid(new_p):
                continue
            
            # point is valid, add to tree
            tree_a.append(new_p)
            
            # rewrite tree to smooth the route
            for point in neighbor_points:
                if point == parent_p:
                    continue
                if new_p.cost + self.distance(new_p,point) < point.cost:
                    # check heading limit
                    if self.angle_diff(new_p.heading,point.heading) > (self.heading_limit):
                        continue
                    if self.angle_diff(new_p.heading,self.heading(new_p,point)) > (self.heading_limit):
                        continue
                    if self.angle_diff(point.heading,self.heading(new_p,point)) > (self.heading_limit):
                        continue
                    point.parent = new_p
                    point.cost = new_p.cost + self.distance(new_p, point)
                    point.heading = self.heading(new_p,point)
            
            # find nearest point in another tree
            nearest_point_b = self.Nearest(tree_b, new_p)
            
            # check if two tree can be connected
            if self.distance(new_p,nearest_point_b) > self.step_size:
                continue
            # check heading limit
            if self.angle_diff(new_p.heading,self.angle_inverse(nearest_point_b.heading)) > (self.heading_limit):
                continue
            if self.angle_diff(new_p.heading,self.heading(new_p,nearest_point_b)) > (self.heading_limit):
                continue
            if self.angle_diff(nearest_point_b.heading,self.heading(nearest_point_b,new_p)) > (self.heading_limit):
                continue
            
            # check if there exist another point that can connect two tree with less cost 
            neighbor_points = self.Neighbors(new_p,tree_b)
            min_cost = new_p.cost + nearest_point_b.cost + self.distance(new_p,nearest_point_a)
            for point in neighbor_points:
                curr_cost = new_p.cost + point.cost + self.distance(point, new_p)
                if curr_cost < min_cost:
                    if self.angle_diff(new_p.heading,self.angle_inverse(point.heading)) > (self.heading_limit):
                        continue
                    if self.angle_diff(new_p.heading,self.heading(new_p,point)) > (self.heading_limit):
                        continue
                    if self.angle_diff(point.heading,self.heading(point,new_p)) > (self.heading_limit):
                        continue
                    min_cost = curr_cost
                    nearest_point_b = point
            # generate a route from start point to end point
            self.trace_path(new_p,nearest_point_b)
            return self.path
        
        print("========== route not found ==========")    
        return []
    
    # if the distance of point in the tree and sample point is less or equal to search radius
    # it is considered as a neighbor of sample point  
    def Neighbors(self,sample_p,tree):
        neighbor_points = []
        for point in tree:
            if self.distance(point, sample_p) <= self.search_r:
                neighbor_points.append(point)
        return neighbor_points
    
    # the relation of two tree is opsite, revert one of them
    def trace_path(self,point_a,point_b):
        path_start = []
        path_end = []
        
        if point_a not in self.tree_from_start:
            point_temp = point_a
            point_a = point_b
            point_b = point_temp
            
        while point_a is not None:
            path_start.append([point_a.x,point_a.y,point_a.heading])
            point_a = point_a.parent
        while point_b is not None:
            point_b.heading = self.angle_inverse(point_b.heading)
            path_end.append([point_b.x,point_b.y,point_b.heading])
            point_b = point_b.parent
            
        self.path = path_start[::-1] + path_end

    # return euclidean distance between two point/obstacle 
    def distance(self,a,b):
        return math.sqrt(math.pow(a.x-b.x,2) + math.pow(a.y-b.y,2))

    # return angel within -pi to pi
    def angle_norm(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # return absolute value of angle difference within 0 to pi 
    def angle_diff(self,a,b):
        a = self.angle_norm(a)
        b = self.angle_norm(b)
        diff = a - b
        return abs(self.angle_norm(diff))

    # return the angle in oppsite direction
    def angle_inverse(self,angle):
        angle = self.angle_norm(angle)
        if angle < 0:
            return angle + math.pi
        return angle-math.pi
    
    # check if the point/move is collision free
    def is_valid(self,point):
        for obstacle in self.OBSTACLE_LIST:
            if self.distance(point, obstacle) < obstacle.radius + self.OFFSET:
                return False
        return True
                
    # return the nearest point in the tree 
    def Nearest(self,tree,sample_p):
        min = 10000000
        nearest_p = None
        for point in tree:
            d = self.distance(point,sample_p)
            if d < min:
                min = d
                nearest_p = point
        return nearest_p

    # calculate the point heading based on parent point
    def heading(self,parent,p2):
        delta = np.array([p2.x,p2.y]) - np.array([parent.x,parent.y])
        return math.atan2(delta[1],delta[0])

    # create a point that is one step size away from nearest point toward the sample point
    def LocalPlanner(self,nearest_p,sample_p, step_size=0.5):
        dist = self.distance(nearest_p,sample_p)
        if dist < step_size:
            return sample_p
        direction = np.array([sample_p.x,sample_p.y]) - np.array([nearest_p.x,nearest_p.y])
        delta = (direction / dist) * step_size
        new_p = Point()
        new_p.x = nearest_p.x + delta[0]
        new_p.y = nearest_p.y + delta[1]
        new_p.parent = nearest_p
        new_p.cost = dist
        return new_p
