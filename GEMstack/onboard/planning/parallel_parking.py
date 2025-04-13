from typing import List
from ..component import Component
from ...utils import serialization
from ...mathutils import collisions
from ...state import Route,ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
import os
import numpy as np
import time
from typing import List
import yaml
from ...onboard.planning import RRT
import reeds_shepp
from shapely.geometry import Polygon
from typing import List, Tuple
import math


Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))


# TODO: Pass constant to settings or config, double-check value rho (turning radius)
def reeds_shepp_path(start_pose, final_pose, step_size=0.1, rho=3.657):# 3.657
    path = reeds_shepp.path_sample(start_pose, final_pose, rho, step_size)
    waypoints = [(x, y, yaw) for x, y, yaw in path]
    #waypoints = np.array(waypoints_for_obstacles_check)[:,:2]
    return waypoints


def rectangle_polygon(center: Pose, dims: Dims) -> Polygon:
    """
    Build a shapely Polygon for an oriented rectangle.
    
    Args:
        center: (x, y, yaw) of rectangle center in world frame.
        dims:   (width, length) of rectangle.
        
    Returns:
        shapely Polygon of the 4 corners, in CCW order.
    """
    x, y, yaw = center
    w, L      = dims
    # corners in vehicle frame (forward is +x, left is +y)
    corners = np.array([
        [ +L/2, +w/2],
        [ +L/2, -w/2],
        [ -L/2, -w/2],
        [ -L/2, +w/2],
    ])
    # rotation matrix
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw),  np.cos(yaw)]])
    # rotate & translate
    world_corners = (R @ corners.T).T + np.array([x, y])
    return Polygon(world_corners)

def is_trajectory_collision_free(
    trajectory: List[Pose],
    vehicle_dims: Dims,
    obstacles: List[Obstacle]
) -> bool:
    """
    Check whether following `trajectory` will avoid all `obstacles`.
    
    Args:
        trajectory: List of (x, y, yaw) vehicle poses along planned path.
        vehicle_dims: (width, length) of your vehicle rectangle.
        obstacles: List of static obstacles, each as (x, y, yaw, (width, length)).
        
    Returns:
        True if no pose along the trajectory intersects any obstacle; False otherwise.
    """
    # Pre-build obstacle polygons once
    obstacle_polys = [
        rectangle_polygon((ox, oy, oyaw), dims)
        for ox, oy, oyaw, dims in obstacles
    ]
    
    # For each pose along the trajectory, test intersection
    for pose in trajectory:
        veh_poly = rectangle_polygon(pose, vehicle_dims)
        for obs_poly in obstacle_polys:
            if veh_poly.intersects(obs_poly):
                # collision detected
                return False
    # no collisions anywhere
    return True


def find_parking_positions(
    obstacles: List[Tuple[float, float, float, Tuple[float, float]]],
    vehicle_dims: Tuple[float, float],
    margin: float = 0.2
    ) -> List[Tuple[float, float, float]]:
    """
    Compute all the parking‐slot center positions and orientations along a single lane
    where a vehicle of given dimensions can fit between existing obstacles.

    Parameters
    ----------
    obstacles : List of (x, y, theta, (w, l))
        Each obstacle is centered at (x,y), oriented by yaw theta (radians),
        and has width w (across lane) and length l (along lane).
    vehicle_dims : (w_v, l_v)
        Width and length of the vehicle (same convention as obstacles).
    margin : float
        Minimum clearance to leave between parked vehicles (in meters).

    Returns
    -------
    slots : List of (x_c, y_c, theta_c)
        Center‐points and yaw for each available parking slot.
    """
    if not obstacles:
        return []

    # Assume all obstacles lie along one straight lane with the same orientation
    lane_theta = obstacles[0][2]
    cos_t = math.cos(lane_theta)
    sin_t = math.sin(lane_theta)

    # Project each obstacle center onto the lane axis (s-coordinate)
    projected = []
    for x, y, theta, (w, l) in obstacles:
        # s = x*cos + y*sin
        s = x * cos_t + y * sin_t
        half_len = l / 2.0
        projected.append((s, half_len, (x, y)))

    # Sort by increasing s
    projected.sort(key=lambda e: e[0])

    slots: List[Tuple[float, float, float]] = []
    w_v, l_v = vehicle_dims

    # Examine gaps between consecutive obstacles
    for (s_i, half_i, (x_i, y_i)), (s_j, half_j, (x_j, y_j)) in zip(projected, projected[1:]):
        # compute start/end of free interval in s‐space
        free_start = s_i + half_i + margin/2
        free_end   = s_j - half_j - margin/2
        free_length = free_end - free_start

        # How many cars of length l_v can we fit (with margin between them)?
        if free_length >= l_v:
            # spacing = car length + margin
            spacing = l_v + margin
            n_fit = int(math.floor((free_length + margin) / spacing))
            for k in range(n_fit):
                # center s‐coordinate for each slot
                s_c = free_start + (spacing * k) + (l_v / 2)
                # map back to (x,y): x = s*cos, y = s*sin (plus any perpendicular offset if needed)
                x_c = s_c * cos_t
                y_c = s_c * sin_t
                slots.append((x_c, y_c, lane_theta))

    return slots



def parking_spot_finder(static_horizontal_curb, parked_cars, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
    if not parked_cars:
        print("There are no parked cars.")
        # TODO: compute x1 and x2 for general case, when curbs line connecting curbs is not horizontal to Y axis
        x1 = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1]/2
        print("x1", x1)
        y1 = static_horizontal_curb[0][1]
        x2 = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1]/2
        print("x2", x2)
        y2 = static_horizontal_curb[1][1]
        dx = x2 - x1
        dy = y2 - y1
        L = math.hypot(dx, dy)
        print("L", L)
        if L == 0:
           raise ValueError("Points are identical; direction is undefined.")
        
        ratio = int(L/compact_parking_spot_size[1])
        print("ratio", ratio)
        if ratio == 0:
           raise ValueError("No car can fit in the parking lot.")
        

        x1 = x1 + compact_parking_spot_size[1]/2
        parking_spots = []
        parking_spots.append((x1, y1, yaw_of_parked_cars))
        
        for i in range(1, ratio):
            x = x1 + i*compact_parking_spot_size[1] #dx * i / ratio
            y = y1 + dy * i / ratio
            parking_spots.append((x-shift_from_center_to_rear_axis, y, yaw_of_parked_cars))
        return parking_spots
    
    else:
        print("There are parked cars.")
        parked_cars_sorted = sorted(parked_cars, key=lambda x: x[0])
        

        

def pick_parking_spot(available_parking_spots,  x,y, yaw = 0.0):
    print("Available parking spots:", available_parking_spots)
    print("Vehicle pose:", x, y, yaw)
    

    def distance(spot):
        dx = spot[0] - x
        dy = spot[1] - y
        return math.hypot(dx, dy)
    
    return max(available_parking_spots, key=distance)





class SummoningParkingRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, vehicle_interface : GEMInterface, frame : str = 'start'):
        
        self.vehicle_interface = vehicle_interface
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[0:300,:2]  
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
        
        
        # TODO: Find coordinates of curbs in World frame
        self.static_horizontal_curb = [
            (0.0, -2.44, 0.0, (2.44, 0.5)), # horizontal curb at start of parking lot
            (15.14 , -2.44, 0.0, (2.44, 0.5)),  # horizontal curb at end of parking lot #39.79 (7 SLOTS) 10.26 (2 SLOTS) 
        ]
        self.static_vertical_curb_length = self.static_horizontal_curb[1][0] - self.static_horizontal_curb[0][0] + self.static_horizontal_curb[1][3][1]/2 + self.static_horizontal_curb[0][3][1]/2 
        self.static_vertical_curb_center = (self.static_horizontal_curb[1][0] - self.static_horizontal_curb[1][3][1]/2) - (self.static_horizontal_curb[0][0] + self.static_horizontal_curb[0][3][1]/2) + self.static_horizontal_curb[0][3][1]/2 
        self.static_obstacles_vertical_curb = [(self.static_vertical_curb_center, -4.88, 0.0, (2.44, self.static_vertical_curb_length))]

        self.parked_cars = [] #[(5.0, -2.44, 0.0, (2.44, 4.88)), (19.64, -2.44, 0.0, (2.44, 4.88))]
        self.objects_to_avoid_collisions = self.static_horizontal_curb + self.static_obstacles_vertical_curb + self.parked_cars
        # US Compact Space for parking (2.44, 4.88)
        # TODO: get from gem_e4_geometry.yaml 
        self.vehicle_dims = (1.7, 3.2)
        #vehicle_dims_plus_margin = (vehicle_dims[0] + 0.2, vehicle_dims[1] + 0.2)
        self.compact_parking_spot_size = (2.44, 4.88)
        self.available_parking_spots = parking_spot_finder(self.static_horizontal_curb, [], self.compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2)
        self.parking_spot_to_go = [pick_parking_spot(self.available_parking_spots, 0.0, 0.0)]
        self.x_axis_of_search = 0.0



        # waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (x, 0.0, 0.0))
        # waypoints2 , waypoints_for_obstacles_check2 = reeds_shepp_path((x, 0.0, 0.0), self.parking_spot_to_go[0])
        # waypoints3, waypoints_for_obstacles_check3 = reeds_shepp_path((self.parking_spot_to_go[0][0]-0.2, self.parking_spot_to_go[0][1], self.parking_spot_to_go[0][2]), self.parking_spot_to_go[0])
        while True:
            waypoints = reeds_shepp_path((0.0, 0.0, 0.0), (self.x_axis_of_search, 0.0, 0.0))
            waypoints2  = reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0),self.parking_spot_to_go[0])
            #waypoints2  = reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0), (self.parking_spot_to_go[0][0]-0.7, self.parking_spot_to_go[0][1], 0.0))
            #waypoints3 = reeds_shepp_path((self.parking_spot_to_go[0][0]-0.7, self.parking_spot_to_go[0][1], 0.0), self.parking_spot_to_go[0])
            self.waypoints_for_obstacles_check = (waypoints + waypoints2)# + waypoints3)   
            self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:,:2]
            if is_trajectory_collision_free(self.waypoints_for_obstacles_check, self.vehicle_dims, self.objects_to_avoid_collisions) is True:
               break 
            self.x_axis_of_search += 0.1

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, vehicle: VehicleState,x=0.0):
        self.current_pose = vehicle.pose

        # TODO: Cone detection -> parking position dataset -> select the nearest one
        # TODO: Add angle and gear direction in waypoints. 
        # TODO: Check if the route is feasable (i.e. no collision), if not, re-plan the route by
        #       construcating holonomic path some path planning algorithm (e.g. RRT, A*).
        #       Accross holonomic path by a method of deveision find sequence of Reeds-Shepp path 
        #       that connect the start and goal pose.
        # TODO: To execute the parking be able drive vehicle in reverse: Talk to control group
        # TODO: Update from preseption data
        # TODO: Triger when new cone detection is available
        # TODO: Add a method to update find axis of search
        parked_cars = []
            
        
        self.route = Route(frame=ObjectFrameEnum.START,points = self.waypoints_to_go.tolist())

        return self.route

