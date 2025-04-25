from typing import List
import numpy as np
from typing import List
import reeds_shepp
from shapely.geometry import Polygon
from typing import List, Tuple
import math


Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))



class ReedsSheppParking:
    def __init__(self,initial_pose_of_vehicle = (0.0, 0.0, 0.0), vehicle_dims = (1.7, 3.2), compact_parking_spot_size = (2.44, 4.88),
                 shift_from_center_to_rear_axis = 1.25, search_step_size = 0.1, closest = False,
                 static_horizontal_curb = [(0.0, -2.44, 0.0, (1.77, 0.5)),(24.9, -2.44, 0.0, (1.77, 0.5))],
                 static_vertical_curb = [(12.45, -4.88, 0.0, (2.44, 24.9))],
                 parked_cars = []):

        self.initial_pose_of_vehicle = initial_pose_of_vehicle
        self.x_axis_of_search = self.initial_pose_of_vehicle[0]
        self.static_horizontal_curb = static_horizontal_curb 
        self.static_vertical_curb = static_vertical_curb
        self.parked_cars = parked_cars

        self.vehicle_dims = vehicle_dims 
        self.compact_parking_spot_size = compact_parking_spot_size   # US Compact Space for parking (2.44, 4.88)
        self.shift_from_center_to_rear_axis = shift_from_center_to_rear_axis # TODO: Check
        self.search_step_size = search_step_size
        # TODO: Add thrid option: park in the middle
        self.closest = closest # If True, the closest parking spot will be selected, otherwise the farthest one will be selected

    

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
            ReedsSheppParking.rectangle_polygon((ox, oy, oyaw), dims)
            for ox, oy, oyaw, dims in obstacles
        ]
        
        # For each pose along the trajectory, test intersection
        for pose in trajectory:
            veh_poly = ReedsSheppParking.rectangle_polygon(pose, vehicle_dims)
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
        
    def find_collision_free_trajectory(self, parked_cars = []):

        self.parked_cars = parked_cars
        self.objects_to_avoid_collisions = self.static_horizontal_curb + self.parked_cars + self.static_vertical_curb
        self.all_parking_spots_in_parking_lot = ReedsSheppParking.all_parking_spots_in_parking_lot(
        self.static_horizontal_curb, self.compact_parking_spot_size, yaw_of_parked_cars=0.0,
        shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )
        self.available_parking_spots = ReedsSheppParking.available_parking_spots(
        self.all_parking_spots_in_parking_lot, self.parked_cars, self.compact_parking_spot_size,
        yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )
        self.parking_spot_to_go = [ReedsSheppParking.pick_parking_spot(self.available_parking_spots, 0.0, 0.0, yaw=0.0, closest=self.closest)]
        x_shift = self.parking_spot_to_go[0][0] - self.shift_from_center_to_rear_axis
        self.parking_spot_to_go[0] = (x_shift, self.parking_spot_to_go[0][1], self.parking_spot_to_go[0][2])
        self.x_axis_of_search_direction_positive = ReedsSheppParking.search_axis_direction(self.parking_spot_to_go, self.initial_pose_of_vehicle)

        while True:
            waypoints = ReedsSheppParking.reeds_shepp_path((0.0, 0.0, 0.0), (self.x_axis_of_search, 0.0, 0.0))
            waypoints2 = ReedsSheppParking.reeds_shepp_path((self.x_axis_of_search, 0.0, 0.0), self.parking_spot_to_go[0])
            self.waypoints_for_obstacles_check = waypoints + waypoints2
            self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:, :2]
            if ReedsSheppParking.is_trajectory_collision_free(self.waypoints_for_obstacles_check, self.vehicle_dims,
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