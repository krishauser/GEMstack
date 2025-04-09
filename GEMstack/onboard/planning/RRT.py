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

OFFSET = 0.8 # meter

OBSTACLE_LIST = []

# return euclidean distance between two point/obstacle 
def distance(a,b):
    return math.sqrt(math.pow(a.x-b.x,2) + math.pow(a.y-b.y,2))

# return angel within -pi to pi
def angle_norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# return absolute value of angle difference within 0 to pi 
def angle_diff(a,b):
    a = angle_norm(a)
    b = angle_norm(b)
    diff = a - b
    return abs(angle_norm(diff))

# return the angle in oppsite direction
def angle_inverse(angle):
    angle = angle_norm(angle)
    if angle < 0:
        return angle + math.pi
    return angle-math.pi

def is_valid(point):
    for obstacle in OBSTACLE_LIST:
        if distance(point, obstacle) < obstacle.radius + OFFSET:
            return False
    return True
            
# return the nearest point in the tree 
def Nearest(tree,sample_p):
    min = 10000000
    nearest_p = None
    for point in tree:
        d = distance(point,sample_p)
        if d < min:
            min = d
            nearest_p = point
    return nearest_p

# calculate the point heading based on parent point
def heading(parent,p2):
    delta = np.array([p2.x,p2.y]) - np.array([parent.x,parent.y])
    return math.atan2(delta[1],delta[0])

# create a point that is one step size away from nearest point toward the sample point
def LocalPlanner(nearest_p,sample_p, step_size=0.5):
    dist = distance(nearest_p,sample_p)
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

class BiRRT:
    def __init__(self, start : list, goal : list, obstacles : list, map : list):
        
        self.path = []
        self.tree_from_start = []
        self.tree_from_end = []
        
        self.start_point = Point(x = start[0],y = start[1],heading = start[2])
        self.end_point = Point(x = goal[0],y = goal[1],heading = goal[2])
        
        OBSTACLE_LIST = []
        for i in range(len(obstacles)):
            OBSTACLE_LIST.append(Obstacle(obstacles[i][0], obstacles[i][1], r = 0.2))
        
        self.MAX_Iteration = 20000
        self.step_size = 0.5 # meter
        self.search_r = 1.3 # meter
        self.heading_limit = math.pi/6 # limit the heading change in route
        self.goal_sample_rate = 0.1 # rate to check area near end-goal
        
        # Map boundary
        self.MAP_X_LOW = map[0] # meter
        self.MAP_X_HIGH = map[1] # meter
        self.MAP_Y_LOW = map[2] # meter
        self.MAP_Y_HIGH = map[3] # meter
        
    def search(self):
        # initialize two tree
        self.tree_from_start.append(self.start_point)
        self.tree_from_end.append(self.end_point)
        
        # self.start_time = time.time()
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
            nearest_point_a = Nearest(tree_a, sample_p)
            # use local planner to move one step size
            new_p = LocalPlanner(nearest_point_a, sample_p, self.step_size)
            # check collision
            if not is_valid(new_p):
                continue
            # check if there exist previous point with less cost to new point
            neighbor_points = self.Neighbors(new_p,tree_a)
            min_cost = nearest_point_a.cost + distance(new_p,nearest_point_a)
            parent_p = nearest_point_a
            for point in neighbor_points:
                curr_cost = point.cost + distance(point, new_p)
                if curr_cost < min_cost:
                    min_cost = curr_cost
                    parent_p = point
            # update point's paraent                 
            new_p.cost = min_cost
            new_p.parent = parent_p
            new_p.heading = heading(parent_p,new_p)
            
            # check heading limit and collision
            if angle_diff(new_p.heading,new_p.parent.heading) > (self.heading_limit):
                continue
            if not is_valid(new_p):
                continue
            
            # point is valid, add to tree
            tree_a.append(new_p)
            
            # rewrite tree to smooth the route
            for point in neighbor_points:
                if point == parent_p:
                    continue
                if new_p.cost + distance(new_p,point) < point.cost:
                    # check heading limit
                    if angle_diff(new_p.heading,point.heading) > (self.heading_limit):
                        continue
                    if angle_diff(new_p.heading,heading(new_p,point)) > (self.heading_limit):
                        continue
                    if angle_diff(point.heading,heading(new_p,point)) > (self.heading_limit):
                        continue
                    point.parent = new_p
                    point.cost = new_p.cost + distance(new_p, point)
                    point.heading = heading(new_p,point)
            
            # find nearest point in another tree
            nearest_point_b = Nearest(tree_b, new_p)
            
            # check if two tree can be connected
            if distance(new_p,nearest_point_b) > self.step_size:
                continue
            # check heading limit
            if angle_diff(new_p.heading,angle_inverse(nearest_point_b.heading)) > (self.heading_limit):
                continue
            if angle_diff(new_p.heading,heading(new_p,nearest_point_b)) > (self.heading_limit):
                continue
            if angle_diff(nearest_point_b.heading,heading(nearest_point_b,new_p)) > (self.heading_limit):
                continue
            
            # check if there exist another point that can connect two tree with less cost 
            neighbor_points = self.Neighbors(new_p,tree_b)
            min_cost = new_p.cost + nearest_point_b.cost + distance(new_p,nearest_point_a)
            for point in neighbor_points:
                curr_cost = new_p.cost + point.cost + distance(point, new_p)
                if curr_cost < min_cost:
                    if angle_diff(new_p.heading,angle_inverse(point.heading)) > (self.heading_limit):
                        continue
                    if angle_diff(new_p.heading,heading(new_p,point)) > (self.heading_limit):
                        continue
                    if angle_diff(point.heading,heading(point,new_p)) > (self.heading_limit):
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
            if distance(point, sample_p) <= self.search_r:
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
            path_start.append(point_a)
            point_a = point_a.parent
        while point_b is not None:
            point_b.heading = angle_inverse(point_b.heading)
            path_end.append(point_b)
            point_b = point_b.parent
            
        self.path = path_start[::-1] + path_end
