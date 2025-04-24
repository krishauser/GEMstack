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
# def reeds_shepp_path(start_pose, final_pose, step_size=0.1, rho=3.657):# 3.657
#     path = reeds_shepp.path_sample(start_pose, final_pose, rho, step_size)
#     waypoints = [(x, y, yaw) for x, y, yaw in path]
#     #waypoints = np.array(waypoints_for_obstacles_check)[:,:2]
#     return waypoints


# def rectangle_polygon(center: Pose, dims: Dims) -> Polygon:
#     """
#     Build a shapely Polygon for an oriented rectangle.
    
#     Args:
#         center: (x, y, yaw) of rectangle center in world frame.
#         dims:   (width, length) of rectangle.
        
#     Returns:
#         shapely Polygon of the 4 corners, in CCW order.
#     """
#     x, y, yaw = center
#     w, L      = dims
#     # corners in vehicle frame (forward is +x, left is +y)
#     corners = np.array([
#         [ +L/2, +w/2],
#         [ +L/2, -w/2],
#         [ -L/2, -w/2],
#         [ -L/2, +w/2],
#     ])
#     # rotation matrix
#     R = np.array([[np.cos(yaw), -np.sin(yaw)],
#                   [np.sin(yaw),  np.cos(yaw)]])
#     # rotate & translate
#     world_corners = (R @ corners.T).T + np.array([x, y])
#     return Polygon(world_corners)

# def is_trajectory_collision_free(
#     trajectory: List[Pose],
#     vehicle_dims: Dims,
#     obstacles: List[Obstacle]
# ) -> bool:
#     """
#     Check whether following `trajectory` will avoid all `obstacles`.
    
#     Args:
#         trajectory: List of (x, y, yaw) vehicle poses along planned path.
#         vehicle_dims: (width, length) of your vehicle rectangle.
#         obstacles: List of static obstacles, each as (x, y, yaw, (width, length)).
        
#     Returns:
#         True if no pose along the trajectory intersects any obstacle; False otherwise.
#     """
#     # Pre-build obstacle polygons once
#     obstacle_polys = [
#         rectangle_polygon((ox, oy, oyaw), dims)
#         for ox, oy, oyaw, dims in obstacles
#     ]
    
#     # For each pose along the trajectory, test intersection
#     for pose in trajectory:
#         veh_poly = rectangle_polygon(pose, vehicle_dims)
#         for obs_poly in obstacle_polys:
#             if veh_poly.intersects(obs_poly):
#                 # collision detected
#                 return False
#     # no collisions anywhere
#     return True


# def find_parking_positions(
#     obstacles: List[Tuple[float, float, float, Tuple[float, float]]],
#     vehicle_dims: Tuple[float, float],
#     margin: float = 0.2
#     ) -> List[Tuple[float, float, float]]:
#     """
#     Compute all the parking‐slot center positions and orientations along a single lane
#     where a vehicle of given dimensions can fit between existing obstacles.

#     Parameters
#     ----------
#     obstacles : List of (x, y, theta, (w, l))
#         Each obstacle is centered at (x,y), oriented by yaw theta (radians),
#         and has width w (across lane) and length l (along lane).
#     vehicle_dims : (w_v, l_v)
#         Width and length of the vehicle (same convention as obstacles).
#     margin : float
#         Minimum clearance to leave between parked vehicles (in meters).

#     Returns
#     -------
#     slots : List of (x_c, y_c, theta_c)
#         Center‐points and yaw for each available parking slot.
#     """
#     if not obstacles:
#         return []

#     # Assume all obstacles lie along one straight lane with the same orientation
#     lane_theta = obstacles[0][2]
#     cos_t = math.cos(lane_theta)
#     sin_t = math.sin(lane_theta)

#     # Project each obstacle center onto the lane axis (s-coordinate)
#     projected = []
#     for x, y, theta, (w, l) in obstacles:
#         # s = x*cos + y*sin
#         s = x * cos_t + y * sin_t
#         half_len = l / 2.0
#         projected.append((s, half_len, (x, y)))

#     # Sort by increasing s
#     projected.sort(key=lambda e: e[0])

#     slots: List[Tuple[float, float, float]] = []
#     w_v, l_v = vehicle_dims

#     # Examine gaps between consecutive obstacles
#     for (s_i, half_i, (x_i, y_i)), (s_j, half_j, (x_j, y_j)) in zip(projected, projected[1:]):
#         # compute start/end of free interval in s‐space
#         free_start = s_i + half_i + margin/2
#         free_end   = s_j - half_j - margin/2
#         free_length = free_end - free_start

#         # How many cars of length l_v can we fit (with margin between them)?
#         if free_length >= l_v:
#             # spacing = car length + margin
#             spacing = l_v + margin
#             n_fit = int(math.floor((free_length + margin) / spacing))
#             for k in range(n_fit):
#                 # center s‐coordinate for each slot
#                 s_c = free_start + (spacing * k) + (l_v / 2)
#                 # map back to (x,y): x = s*cos, y = s*sin (plus any perpendicular offset if needed)
#                 x_c = s_c * cos_t
#                 y_c = s_c * sin_t
#                 slots.append((x_c, y_c, lane_theta))

#     return slots

# def all_parking_spots_in_parking_lot(static_horizontal_curb, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
#     if not static_horizontal_curb:
#        raise ValueError("No static horizontal curb provided.")
    
#     curb0_x = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1]/2
#     curb0_y = static_horizontal_curb[0][1]
#     curb1_x  = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1]/2
#     curb1_y  = static_horizontal_curb[1][1]
#     dx = curb1_x - curb0_x
#     dy = curb1_y - curb0_y
#     L = math.hypot(dx, dy)
#     if L == 0:
#        raise ValueError("Points are identical; direction is undefined.")
#     ratio = int(L/compact_parking_spot_size[1])
#     if ratio == 0:
#        raise ValueError("No car can fit in the parking lot.")   

#     x1 = curb0_x + compact_parking_spot_size[1]/2
#     parking_spots = []
#     parking_spots.append((x1, curb0_y, yaw_of_parked_cars))
    
#     for i in range(1, ratio):
#         x = x1 + i*compact_parking_spot_size[1] #dx * i / ratio
#         y = curb0_y 
#         parking_spots.append((x, y, yaw_of_parked_cars))

#     return parking_spots  

# def pick_parking_spot(available_parking_spots, x, y, yaw = 0.0, closest = True):

#     def distance(spot):
#         dx = spot[0] - x
#         dy = spot[1] - y
#         return math.hypot(dx, dy)
    
#     if closest:
#        return min(available_parking_spots, key=distance)
#     else:
#        return max(available_parking_spots, key=distance)    

# def search_axis_direction(parking_spot_to_go, vehicle_pose):
#     dx = parking_spot_to_go[0][0] - vehicle_pose[0]
#     if dx > 0:
#         return True
#     else:
#         return False     

# def available_parking_spots(all_parking_spots_in_parking_lot, parked_cars, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
#     available_parking_spots = []
#     for spot in all_parking_spots_in_parking_lot:
#         # Check if the parking spot is occupied by a parked car
#         is_occupied = False
#         for parked_car in parked_cars:
#             if (abs(parked_car[0] - spot[0]) < compact_parking_spot_size[1] / 2) and (abs(parked_car[1] - spot[1]) < compact_parking_spot_size[0] / 2):
#                 is_occupied = True
#                 break

#         # If the parking spot is not occupied, add it to the available list
#         if not is_occupied:
#             available_parking_spots.append(spot)

#     return available_parking_spots           

# def parking_spot_finder(static_horizontal_curb, parked_cars, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
    
#     # TODO: compute x1 and x2 for general case, when curbs line connecting curbs is not horizontal to Y axis
#     if not static_horizontal_curb:
#         raise ValueError("No static horizontal curb provided.")


#     if not parked_cars:

#         x1 = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1]/2
#         print("x1: ", x1)
#         y1 = static_horizontal_curb[0][1]
#         x2 = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1]/2
#         y2 = static_horizontal_curb[1][1]
#         print("There are no parked cars.")
#         dx = x2 - x1
#         dy = y2 - y1
#         L = math.hypot(dx, dy)
#         if L == 0:
#            raise ValueError("Points are identical; direction is undefined.")
        
#         ratio = int(L/compact_parking_spot_size[1])
#         if ratio == 0:
#            raise ValueError("No car can fit in the parking lot.")


#         print("ratio: ", ratio)

#         x1 = x1 + compact_parking_spot_size[1]/2
#         parking_spots = []
#         parking_spots.append((x1, y1, yaw_of_parked_cars))
#         print("x1: ", x1)
#         print("compact_parking_spot_size[1] = ", compact_parking_spot_size[1])
#         for i in range(1, ratio):
#             x = x1 + i*compact_parking_spot_size[1] #dx * i / ratio
#             y = y1 + dy * i / ratio
#             x_shifted = x - shift_from_center_to_rear_axis
#             parking_spots.append((x_shifted, y, yaw_of_parked_cars))
#         return parking_spots
    
#     else:
#         print("There are parked cars.")
#         parked_cars_sorted = sorted(parked_cars, key=lambda x: x[0])
#         curb0_x = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1]/2
#         curb0_y = static_horizontal_curb[0][1]
#         curb1_x  = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1]/2
#         curb1_y  = static_horizontal_curb[1][1]

        
#         # Curb 1 and closes parked car
#         # TODO: Consider the case when curb is not horizontal to Y axis.
#         parked_0_x = parked_cars_sorted[0][0] - compact_parking_spot_size[1]/2
#         dx = parked_0_x- curb0_x
#         dy = parked_cars_sorted[0][1] - curb0_y
#         L = math.hypot(dx, dy)
#         ratio = int(L/compact_parking_spot_size[1])

#         # Adding available parking spots between curb 1 and closes parked car if at least one car can fit
#         if ratio >= 1:
#            curb0_x = curb0_x + compact_parking_spot_size[1]/2
#            parking_spots = []
#            parking_spots.append((curb0_x, curb0_y, yaw_of_parked_cars))
#         if ratio >= 2:
#             for i in range(1, ratio):
#                 x = curb0_x+ i*compact_parking_spot_size[1] #dx * i / ratio
#                 y = curb0_y
#                 parking_spots.append((x, y, yaw_of_parked_cars))

    
#         # Adding available parking spots between parked cars
#         for i in range(len(parked_cars_sorted)-1):
#             parked_0_x = parked_cars_sorted[i][0] + compact_parking_spot_size[1]/2
#             parked_0_y = parked_cars_sorted[i][1]
#             parked_n_x = parked_cars_sorted[i+1][0] - compact_parking_spot_size[1]/2
#             dx = parked_n_x - parked_0_x
#             dy = parked_0_y - parked_n_x
#             L = math.hypot(dx, dy)
#             ratio = int(L/compact_parking_spot_size[1])
#             # Adding available parking spots between parked cars if at least one car can fit
#             if ratio >= 1:
#                 parked_0_x = parked_0_x + compact_parking_spot_size[1]/2
#                 parking_spots.append((parked_0_x, parked_0_y, yaw_of_parked_cars))
#             if ratio >= 2:
#                 for i in range(1, ratio):
#                     x = parked_0_x + i*compact_parking_spot_size[1]
#                     y = parked_0_y
#                     parking_spots.append((x, y, yaw_of_parked_cars))

#         # Curb 2 and closes parked car
#         parked_n_x = parked_cars_sorted[-1][0] + compact_parking_spot_size[1]/2
#         dx = curb1_x - parked_n_x
#         dy = curb1_y - parked_cars_sorted[-1][1]
#         L = math.hypot(dx, dy)
#         ratio = int(L/compact_parking_spot_size[1])
#         # Adding available parking spots between curb 2 and closes parked car if at least one car can fit
#         if ratio >= 1:
#            curb1_x = curb1_x - compact_parking_spot_size[1]/2
#            parking_spots.append((curb1_x, curb1_y, yaw_of_parked_cars))
#         if ratio >= 2:
#             for i in range(1, ratio):
#                 x = curb1_x - i*compact_parking_spot_size[1]
#                 y = curb1_y
#                 parking_spots.append((x, y, yaw_of_parked_cars))            

#         return parking_spots
        

    







# class SummoningParkingRoutePlanner(Component):
#     """Reads a route from disk and returns it as the desired route."""
#     def __init__(self, routefn : str, vehicle_interface : GEMInterface, frame : str = 'start'):
        
#         self.vehicle_interface = vehicle_interface
#         self.routefn = routefn
#         base, ext = os.path.splitext(routefn)
#         if ext in ['.json','.yml','.yaml']:
#             with open(routefn,'r') as f:
#                 self.route = serialization.load(f)
#         elif ext == '.csv':
#             waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
#             if waypoints.shape[1] == 3:
#                 waypoints = waypoints[0:300,:2]  
#             if frame == 'start':
#                 self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
#             elif frame == 'global':
#                 self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
#             elif frame == 'cartesian':
#                 self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
#             else:
#                 raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
#         else:
#             raise ValueError("Unknown route file extension",ext)
        
#         # TODO: Move to parameters YAML
#         self.vehicle_dims = (1.7, 3.2)
#         self.compact_parking_spot_size = (2.44, 4.88) # US Compact Space for parking (2.44, 4.88)
#         self.shift_from_center_to_rear_axis = 2.56/2
#         self.x_axis_of_search = 0.0
#         self.search_step_size = 0.1
#         self.closest = False # If True, the closest parking spot will be selected, otherwise the farthest one will be selected
#         self.initial_pose_of_vehicle = (0.0, 0.0, 0.0) 
    
#         # Curbs and parked cars are in the same frame 
        
#         # TODO: Find coordinates of curbs in World frame
#         # Examples of parking lots: 10.26 (2 SLOTS), 24.9 (5 SLOTS), 39.79 (7 SLOTS) 
#         self.static_horizontal_curb = [
#             (0.0, -2.44, 0.0, (2.44, 0.5)), # horizontal curb at start of parking lot
#             (24.9 , -2.44, 0.0, (2.44, 0.5)),  # horizontal curb at end of parking lot 
#         ]
#         self.static_vertical_curb_length = self.static_horizontal_curb[1][0] - self.static_horizontal_curb[0][0] + self.static_horizontal_curb[1][3][1]/2 + self.static_horizontal_curb[0][3][1]/2 
#         self.static_vertical_curb_center = (self.static_horizontal_curb[1][0] - self.static_horizontal_curb[1][3][1]/2) - (self.static_horizontal_curb[0][0] + self.static_horizontal_curb[0][3][1]/2) + self.static_horizontal_curb[0][3][1]/2 
#         self.static_obstacles_vertical_curb = [(self.static_vertical_curb_center, -4.88, 0.0, (2.44, self.static_vertical_curb_length))]
        
#         # TODO: Replace with real parked cars
#         # Slot 1 (2.69, -2.44, 0.0, (2.44, 4.88)), Slot 2 (7.57, -2.44, 0.0, (2.44, 4.88)),  Slot 3 (12.45, -2.44, 0.0, (2.44, 4.88)),Slot 4 (17.33, -2.44, 0.0, (2.44, 4.88)), Slot 5 (22.11, -2.44, 0.0, (2.44, 4.88))
#         self.parked_cars = [(2.69, -2.44, 0.0, (1.7, 3.2)),(7.57, -2.44, 0.0, (1.7, 3.2)),(12.45, -2.44, 0.0, (1.7, 3.2))]
#         self.objects_to_avoid_collisions = self.static_horizontal_curb + self.static_obstacles_vertical_curb + self.parked_cars
#         self.all_parking_spots_in_parking_lot = all_parking_spots_in_parking_lot(self.static_horizontal_curb, self.compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2)
#         self.available_parking_spots = available_parking_spots(self.all_parking_spots_in_parking_lot, self.parked_cars, self.compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2)
#         self.parking_spot_to_go = [pick_parking_spot(self.available_parking_spots, 0.0, 0.0, yaw=0.0, closest = self.closest)]
#         x_shift = self.parking_spot_to_go[0][0] - self.shift_from_center_to_rear_axis
#         self.parking_spot_to_go[0] = (x_shift, self.parking_spot_to_go[0][1], self.parking_spot_to_go[0][2])
#         self.x_axis_of_search_direction_positive = search_axis_direction(self.parking_spot_to_go, self.initial_pose_of_vehicle)
        


#         while True:
#             waypoints = reeds_shepp_path((0.0, 0.0, 0.0) , (self.x_axis_of_search, 0.0, 0.0))
#             waypoints2  = reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0),self.parking_spot_to_go[0])
#             #waypoints2  = reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0), (self.parking_spot_to_go[0][0]-0.3, self.parking_spot_to_go[0][1], 0.0))
#             #waypoints3 = reeds_shepp_path((self.parking_spot_to_go[0][0]-0.3, self.parking_spot_to_go[0][1], 0.0), self.parking_spot_to_go[0])
#             self.waypoints_for_obstacles_check = (waypoints + waypoints2)# + waypoints3)   
#             self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:,:2]
#             if is_trajectory_collision_free(self.waypoints_for_obstacles_check, self.vehicle_dims, self.objects_to_avoid_collisions) is True:
#                break 
#             if self.x_axis_of_search_direction_positive:
#                self.x_axis_of_search += self.search_step_size
#                if self.x_axis_of_search > self.static_horizontal_curb[1][0] + self.compact_parking_spot_size[1]:
#                   raise ValueError("No parking spot available.")   
#             else:
#                self.x_axis_of_search -= self.search_step_size 
#                if self.x_axis_of_search < self.static_horizontal_curb[0][0] - self.compact_parking_spot_size[1]:
#                   raise ValueError("No parking spot available.")


#     def state_inputs(self):
#         return ['vehicle']

#     def state_outputs(self) -> List[str]:
#         return ['route']

#     def rate(self):
#         return 10.0

#     def update(self, vehicle: VehicleState,x=0.0):
#         self.current_pose = vehicle.pose

#         # TODO: Cone detection -> parking position dataset -> select the parking spot [+]
#         # TODO: Triger when new cone detection is available []
#         # TODO: To execute the parking be able drive vehicle in reverse []

#         self.route = Route(frame=ObjectFrameEnum.START,points = self.waypoints_to_go.tolist())

#         return self.route







class ParkingUtils:
    def __init__(self):
        self.vehicle_dims = (1.7, 3.2)
        self.compact_parking_spot_size = (2.44, 4.88)  # US Compact Space for parking (2.44, 4.88)
        self.shift_from_center_to_rear_axis = 2.56 / 2 # TODO: Check
        self.search_step_size = 0.1
        self.closest = False  # If True, the closest parking spot will be selected, otherwise the farthest one will be selected
        self.initial_pose_of_vehicle = (0.0, 0.0, 0.0)
        self.x_axis_of_search = self.initial_pose_of_vehicle[0]

        self.static_horizontal_curb = [
            (0.0, -2.44, 0.0, (2.44, 0.5)),  # horizontal curb at start of parking lot
            (24.9, -2.44, 0.0, (2.44, 0.5)),  # horizontal curb at end of parking lot
        ]
        self.static_vertical_curb_length = self.static_horizontal_curb[1][0] - self.static_horizontal_curb[0][0] + \
                                            self.static_horizontal_curb[1][3][1] / 2 + self.static_horizontal_curb[0][3][1] / 2
        self.static_vertical_curb_center = (self.static_horizontal_curb[1][0] - self.static_horizontal_curb[1][3][1] / 2) - \
                                            (self.static_horizontal_curb[0][0] + self.static_horizontal_curb[0][3][1] / 2) + \
                                            self.static_horizontal_curb[0][3][1] / 2
        self.static_obstacles_vertical_curb = [
            (self.static_vertical_curb_center, -4.88, 0.0, (2.44, self.static_vertical_curb_length))
        ]

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
            ParkingUtils.rectangle_polygon((ox, oy, oyaw), dims)
            for ox, oy, oyaw, dims in obstacles
        ]
        
        # For each pose along the trajectory, test intersection
        for pose in trajectory:
            veh_poly = ParkingUtils.rectangle_polygon(pose, vehicle_dims)
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

    def all_parking_spots_in_parking_lot(static_horizontal_curb, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
        if not static_horizontal_curb:
            raise ValueError("No static horizontal curb provided.")
        
        curb0_x = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1]/2
        curb0_y = static_horizontal_curb[0][1]
        curb1_x  = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1]/2
        curb1_y  = static_horizontal_curb[1][1]
        dx = curb1_x - curb0_x
        dy = curb1_y - curb0_y
        L = math.hypot(dx, dy)
        if L == 0:
            raise ValueError("Points are identical; direction is undefined.")
        ratio = int(L/compact_parking_spot_size[1])
        if ratio == 0:
            raise ValueError("No car can fit in the parking lot.")   

        x1 = curb0_x + compact_parking_spot_size[1]/2
        parking_spots = []
        parking_spots.append((x1, curb0_y, yaw_of_parked_cars))
        
        for i in range(1, ratio):
            x = x1 + i*compact_parking_spot_size[1] #dx * i / ratio
            y = curb0_y 
            parking_spots.append((x, y, yaw_of_parked_cars))

        return parking_spots  

    def pick_parking_spot(available_parking_spots, x, y, yaw = 0.0, closest = True):

        def distance(spot):
            dx = spot[0] - x
            dy = spot[1] - y
            return math.hypot(dx, dy)
        
        if closest:
            return min(available_parking_spots, key=distance)
        else:
            return max(available_parking_spots, key=distance)    

    def search_axis_direction(parking_spot_to_go, vehicle_pose):
        dx = parking_spot_to_go[0][0] - vehicle_pose[0]
        if dx > 0:
            return True
        else:
            return False     

    def available_parking_spots(all_parking_spots_in_parking_lot, parked_cars, compact_parking_spot_size, yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis = 2.56/2):
        available_parking_spots = []
        for spot in all_parking_spots_in_parking_lot:
            # Check if the parking spot is occupied by a parked car
            is_occupied = False
            for parked_car in parked_cars:
                if (abs(parked_car[0] - spot[0]) < compact_parking_spot_size[1] / 2) and (abs(parked_car[1] - spot[1]) < compact_parking_spot_size[0] / 2):
                    is_occupied = True
                    break

            # If the parking spot is not occupied, add it to the available list
            if not is_occupied:
                available_parking_spots.append(spot)

        return available_parking_spots  
        
    def find_collision_free_trajectory(self, parked_cars = None):

        self.parked_cars = parked_cars
        self.objects_to_avoid_collisions = self.static_horizontal_curb + self.static_obstacles_vertical_curb + self.parked_cars
        self.all_parking_spots_in_parking_lot = ParkingUtils.all_parking_spots_in_parking_lot(
        self.static_horizontal_curb, self.compact_parking_spot_size, yaw_of_parked_cars=0.0,
        shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )
        self.available_parking_spots = ParkingUtils.available_parking_spots(
        self.all_parking_spots_in_parking_lot, self.parked_cars, self.compact_parking_spot_size,
        yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )
        self.parking_spot_to_go = [ParkingUtils.pick_parking_spot(self.available_parking_spots, 0.0, 0.0, yaw=0.0, closest=self.closest)]
        x_shift = self.parking_spot_to_go[0][0] - self.shift_from_center_to_rear_axis
        self.parking_spot_to_go[0] = (x_shift, self.parking_spot_to_go[0][1], self.parking_spot_to_go[0][2])
        self.x_axis_of_search_direction_positive = ParkingUtils.search_axis_direction(self.parking_spot_to_go, self.initial_pose_of_vehicle)

        while True:
            waypoints = ParkingUtils.reeds_shepp_path((0.0, 0.0, 0.0), (self.x_axis_of_search, 0.0, 0.0))
            waypoints2 = ParkingUtils.reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0), self.parking_spot_to_go[0])
            self.waypoints_for_obstacles_check = waypoints + waypoints2
            self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:, :2]
            if ParkingUtils.is_trajectory_collision_free(self.waypoints_for_obstacles_check, self.vehicle_dims,
                                            self.objects_to_avoid_collisions):
                break
            if self.x_axis_of_search_direction_positive:
                self.x_axis_of_search += self.search_step_size
                if self.x_axis_of_search > self.static_horizontal_curb[1][0] + self.compact_parking_spot_size[1]:
                    raise ValueError("No parking spot available.")
            else:
                self.x_axis_of_search -= self.search_step_size
                if self.x_axis_of_search < self.static_horizontal_curb[0][0] - self.compact_parking_spot_size[1]:
                    raise ValueError("No parking spot available.")


class SummoningParkingRoutePlanner(Component):
    def __init__(self, routefn: str, vehicle_interface: GEMInterface, frame: str = 'start'):
        self.vehicle_interface = vehicle_interface
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(routefn, 'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[0:300, :2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension", ext)
        
        # SLOT 4 (17.33, -2.44, 0.0, (1.7, 3.2))
        # SLOT 5 (22.11, -2.44, 0.0, (1.7, 3.2))
        self.parked_cars = [
            (2.69, -2.44, 0.0, (1.7, 3.2)),
            (7.57, -2.44, 0.0, (1.7, 3.2)),
            (12.45, -2.44, 0.0, (1.7, 3.2)),   
        ]
        self.parking_utils = ParkingUtils()
        

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, vehicle: VehicleState, x=0.0):
        self.current_pose = vehicle.pose
        self.parking_utils.find_collision_free_trajectory(self.parked_cars)
        self.waypoints_to_go = self.parking_utils.waypoints_to_go
        self.route = Route(frame=ObjectFrameEnum.START, points=self.waypoints_to_go.tolist())
        return self.route