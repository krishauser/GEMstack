from typing import List
import numpy as np
from typing import List
import reeds_shepp  # TODO: check module
from shapely.geometry import Polygon
from typing import List, Tuple
import math
import yaml
import time


Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))


# TODO: Take List[roadgrpah.regions.outline] (List[List[Tuple[float,float]]]) as the parking_slots input and find the desired variables in this method.
def find_parking_region(vehicle_pose, parking_lots):

    return []



def reeds_shepp_path(start_pose, final_pose, step_size=0.1, rho=3.657):  # 3.657
    path = reeds_shepp.path_sample(start_pose, final_pose, rho, step_size)
    waypoints = [(x, y, yaw) for x, y, yaw in path]
    # waypoints = np.array(waypoints_for_obstacles_check)[:,:2]
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
    w, L = dims
    # corners in vehicle frame (forward is +x, left is +y)
    corners = np.array([
        [+L / 2, +w / 2],
        [+L / 2, -w / 2],
        [-L / 2, -w / 2],
        [-L / 2, +w / 2],
    ])
    # rotation matrix
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw), np.cos(yaw)]])
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
        free_start = s_i + half_i + margin / 2
        free_end = s_j - half_j - margin / 2
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


def all_parking_spots_in_parking_lot(static_horizontal_curb, compact_parking_spot_size, yaw_of_parked_cars=0.0,
                                     shift_from_center_to_rear_axis=2.56 / 2):
    if not static_horizontal_curb:
        raise ValueError("No static horizontal curb provided.")

    curb0_x = static_horizontal_curb[0][0] + static_horizontal_curb[0][3][1] / 2
    curb0_y = static_horizontal_curb[0][1]
    curb1_x = static_horizontal_curb[1][0] - static_horizontal_curb[1][3][1] / 2
    curb1_y = static_horizontal_curb[1][1]
    dx = curb1_x - curb0_x
    dy = curb1_y - curb0_y
    L = math.hypot(dx, dy)
    if L == 0:
        raise ValueError("Points are identical; direction is undefined.")
    ratio = int(L / compact_parking_spot_size[1])
    if ratio == 0:
        raise ValueError("No car can fit in the parking lot.")

    x1 = curb0_x + compact_parking_spot_size[1] / 2
    parking_spots = []
    parking_spots.append((x1, curb0_y, yaw_of_parked_cars))

    for i in range(1, ratio):
        x = x1 + i * compact_parking_spot_size[1]  # dx * i / ratio
        y = curb0_y
        parking_spots.append((x, y, yaw_of_parked_cars))

    return parking_spots


def pick_parking_spot(available_parking_spots, x, y, yaw=0.0, closest=True):
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


def available_parking_spots(all_parking_spots_in_parking_lot, parked_cars, compact_parking_spot_size,
                            yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis=2.56 / 2):
    available_parking_spots = []
    for spot in all_parking_spots_in_parking_lot:
        # Check if the parking spot is occupied by a parked car
        is_occupied = False
        for parked_car in parked_cars:
            if (abs(parked_car[0] - spot[0]) < compact_parking_spot_size[1] / 2) and (
                    abs(parked_car[1] - spot[1]) < compact_parking_spot_size[0] / 2):
                is_occupied = True
                break

        # If the parking spot is not occupied, add it to the available list
        if not is_occupied:
            available_parking_spots.append(spot)

    return available_parking_spots


def yaw_of_parked_cars(curb_0, curb_1):
    # Compute the vector v from p1 to p2
    # v_x = x2 - x1, v_y = y2 - y1
    v = (curb_1[0] - curb_0[0], curb_1[1] - curb_0[1])
    angle_rad = math.atan2(v[1], v[0])  # TODO: Double check if CCW is the positive direction
    return angle_rad


def shift_points_perpendicular_ccw(p1, p2, shift_amount):
    """
    Shift points p1 and p2 by a given amount perpendicular (to the left)
    of the vector from p1 to p2.

    Args:
        p1 (tuple of float): First point (x1, y1)
        p2 (tuple of float): Second point (x2, y2)
        shift_amount (float): Distance to shift perpendicular to the left of vector v

    Returns:
        p1_shifted (tuple of float): Shifted first point
        p2_shifted (tuple of float): Shifted second point
        dir_unit (tuple of float): Normalized direction vector v̂ = (v_x, v_y) / |v|
        shift_vec (tuple of float): Actual shift vector applied = perp_unit * shift_amount
    """
    x1, y1 = p1
    x2, y2 = p2

    # 1) Compute connecting vector v = p2 - p1
    v_x = x2 - x1
    v_y = y2 - y1

    # 2) Compute its magnitude |v|
    length = math.hypot(v_x, v_y)
    if length == 0:
        raise ValueError("p1 and p2 must be distinct points to define a direction.")

    # 3) Normalize v to get unit direction v̂ = (v_x, v_y) / |v|
    dir_x = v_x / length
    dir_y = v_y / length

    v_norm = (dir_x, dir_y)

    # 4) Compute the left-perpendicular unit vector: perp = (-dir_y, dir_x)
    perp_x = -dir_y
    perp_y = dir_x

    # 5) Scale this perpendicular by the desired shift_amount
    shift_x = perp_x * shift_amount
    shift_y = perp_y * shift_amount

    # 6) Apply shift to both points
    p1_shifted = (x1 + shift_x, y1 + shift_y)
    p2_shifted = (x2 + shift_x, y2 + shift_y)

    # Return shifted points
    return p1_shifted, p2_shifted, v_norm


def project_point_on_axis(p1, p2, p3):
    """
    Project point p3 orthogonally onto the line (axis) defined by p1 -> p2.

    Args:
        p1 (tuple of float): First point on the axis (x1, y1)
        p2 (tuple of float): Second point on the axis (x2, y2)
        p3 (tuple of float): The point to be projected (x3, y3)

    Returns:
        p_proj (tuple of float): Coordinates of the projection of p3 onto the line p1–p2
        t (float): The parameter along the line (0 at p1, 1 at p2, can be outside [0,1])
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # Compute vector along the axis v = p2 - p1
    v_x = x2 - x1
    v_y = y2 - y1

    # Compute vector from p1 to p3: u = p3 - p1
    u_x = x3 - x1
    u_y = y3 - y1

    # Compute dot products
    dot_uv = u_x * v_x + u_y * v_y  # u · v
    dot_vv = v_x * v_x + v_y * v_y  # v · v

    if dot_vv == 0:
        raise ValueError("p1 and p2 must be distinct to define an axis.")

    # Parameter t gives the position along the line: p_proj = p1 + t * v
    t = dot_uv / dot_vv

    # Compute projected point coordinates
    proj_x = x1 + t * v_x
    proj_y = y1 + t * v_y

    return (proj_x, proj_y)


def move_point_along_vector(p0, direction, step=0.1, positive_direction=True):
    """
    Move the point p0 by a fixed step along the given direction vector.

    Args:
        p0 (tuple of float): The starting point (x0, y0).
        direction (tuple of float): The direction vector (dx, dy).
        step (float): Distance to move along the direction (default 0.1).

    Returns:
        tuple of float: The new point (x_new, y_new) after moving.
    Raises:
        ValueError: if the direction vector has zero length.
    """
    x0, y0 = p0
    dx, dy = direction

    # Compute the length of the direction vector
    length = math.hypot(dx, dy)
    if length == 0:
        raise ValueError("Direction vector must be non-zero to define a movement direction.")

    # Normalize the direction vector to unit length
    ux = dx / length
    uy = dy / length

    # Move the point by 'step' along the unit direction
    if positive_direction:
        x_new = x0 + ux * step
        y_new = y0 + uy * step
    else:
        x_new = x0 - ux * step
        y_new = y0 - uy * step

    return (x_new, y_new)



class ReedsSheppParking:
    def __init__(self, vehicle_pose = (0.0, 0.0, 0.0), parking_slots=None, detected_cones = [], update_rate=1.0,
                 compact_parking_spot_size = (2.44, 4.88),
                 search_step_size = 0.1, closest = False, parking_lot_axis_shift_margin = 2.44,
                 static_horizontal_curb_size = (2.44, 0.5),
                 static_horizontal_curb_xy_coordinates = [(0.0, -2.44),(24.9, -2.44)],
                 add_static_vertical_curb_as_obstacle = True,
                 static_vertical_curb_xy_coordinates = [(12.45, -4.88)],
                 static_vertical_curb_size = (2.44, 24.9),
                 add_static_horizontal_curb_as_obstacle = True
                 ):

        
        self.detected_cones = detected_cones
        self.parked_cars = []
        self.objects_to_avoid_collisions = []

        self.static_horizontal_curb_xy_coordinates = static_horizontal_curb_xy_coordinates
        self.static_horizontal_curb_size  = static_horizontal_curb_size
        self.add_static_vertical_curb_as_obstacle = add_static_vertical_curb_as_obstacle

        self.static_vertical_curb_size = static_vertical_curb_size
        self.static_vertical_curb_xy_coordinates = static_vertical_curb_xy_coordinates
        self.add_static_horizontal_curb_as_obstacle = add_static_horizontal_curb_as_obstacle

        self.vehicle_pose = vehicle_pose
        self.x_axis_of_search = self.vehicle_pose[0]

        vehicle_geometry_yaml = "../../knowledge/vehicle/gem_e4_geometry.yaml"
        with open(vehicle_geometry_yaml, 'r') as f:
            vehicle_geometry = yaml.safe_load(f)
        self.vehicle_dims = (vehicle_geometry['width'], vehicle_geometry['length'])
        self.shift_from_center_to_rear_axis = vehicle_geometry['wheelbase'] / 2  # TODO: Check (Yinan: Checked.)

        self.compact_parking_spot_size = compact_parking_spot_size   # US Compact Space for parking (2.44, 4.88)
        self.search_step_size = search_step_size
        self.parking_lot_axis_shift_margin = parking_lot_axis_shift_margin  
        # TODO: Add thrid option: park in the middle
        self.closest = closest # If True, the closest parking spot will be selected, otherwise the farthest one will be selected

        self.time_limit = 1.0 / update_rate


    def find_collision_free_trajectory(self, update_pose=False):
        # TODO: (Yinan: detected_cones and vehicle_pose are already in the object. should delete.)
    # def find_collision_free_trajectory(self, detected_cones, vehicle_pose = (0.0, 0.0, 0.0), update_pose = False):
        # # Update detected cones
        # self.detected_cones = detected_cones
        #
        # # Update vehicle pose
        # if update_pose:
        #    self.vehicle_pose = vehicle_pose

        # Compute the angel between the vector v (connecting start horizontal curb_0 to end horizontal curb_1)
        # and the x-axis. This value is used to compute the orientation of static horizontal curbs,
        # static vertical curb and the yaw of parked cars (we set this since cones do not have orientaion).
        self.yaw_of_parked_cars = yaw_of_parked_cars(self.static_horizontal_curb_xy_coordinates[0], self.static_horizontal_curb_xy_coordinates[1])
        self.static_horizontal_curb = [
            (self.static_horizontal_curb_xy_coordinates[0][0], self.static_horizontal_curb_xy_coordinates[0][1], self.yaw_of_parked_cars, self.static_horizontal_curb_size),
            (self.static_horizontal_curb_xy_coordinates[1][0], self.static_horizontal_curb_xy_coordinates[1][1], self.yaw_of_parked_cars, self.static_horizontal_curb_size)
        ]
        self.static_vertical_curb = [
            (self.static_vertical_curb_xy_coordinates[0][0], self.static_vertical_curb_xy_coordinates[0][1], self.yaw_of_parked_cars, self.static_vertical_curb_size),
        ]

        # Adding obstacles to the list of objects to avoid collisions
        if self.detected_cones != []:
            for cone in self.detected_cones:
                self.parked_cars.append((cone[0], cone[1], self.yaw_of_parked_cars, self.vehicle_dims))
            
        else:
            self.parked_cars = self.detected_cones

        # Adding all the parked cars and static curbs to the list of objects to avoid collisions
        self.objects_to_avoid_collisions += self.parked_cars
        if self.add_static_horizontal_curb_as_obstacle:
            self.objects_to_avoid_collisions += self.static_horizontal_curb
        if self.add_static_vertical_curb_as_obstacle:
            self.objects_to_avoid_collisions += self.static_vertical_curb
        
        # Compute coordinates of parking spots in the parking lot defined by the static horizontal curb

        self.all_parking_spots_in_parking_lot = all_parking_spots_in_parking_lot(
        self.static_horizontal_curb, self.compact_parking_spot_size, yaw_of_parked_cars = self.yaw_of_parked_cars,
        shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )

        # Compute the available parking spots in the parking lot
        self.available_parking_spots = available_parking_spots(
        self.all_parking_spots_in_parking_lot, self.parked_cars, self.compact_parking_spot_size,
        yaw_of_parked_cars=0.0, shift_from_center_to_rear_axis=self.shift_from_center_to_rear_axis
        )



        self.parking_spot_to_go = [pick_parking_spot(self.available_parking_spots, 0.0, 0.0, yaw=0.0, closest=self.closest)]
        x_shift = self.parking_spot_to_go[0][0] - self.shift_from_center_to_rear_axis
        self.parking_spot_to_go[0] = (x_shift, self.parking_spot_to_go[0][1], self.parking_spot_to_go[0][2])
        self.x_axis_of_search_direction_positive = search_axis_direction(self.parking_spot_to_go, self.vehicle_pose)



        # Compute axis of search
        curb_0_xy_shifted, curb_1_xy_shifted, new_axis_direction = shift_points_perpendicular_ccw(self.static_horizontal_curb_xy_coordinates[0],
                                                        self.static_horizontal_curb_xy_coordinates[1], 
                                                        self.parking_lot_axis_shift_margin)
       
        # Compute the projected point on the axis of search
        vehicle_pose_proj = project_point_on_axis(curb_0_xy_shifted, curb_1_xy_shifted, self.vehicle_pose[0:2])

        start_time = time.time()
        while (time.time()-start_time) <= self.time_limit:

            vehicle_pose_proj = move_point_along_vector(vehicle_pose_proj, new_axis_direction,
                                                                          step = self.search_step_size, 
                                                                          positive_direction = self.x_axis_of_search_direction_positive)
            
            waypoints = reeds_shepp_path(self.vehicle_pose , (vehicle_pose_proj[0], vehicle_pose_proj[1], self.yaw_of_parked_cars))
            waypoints2 = reeds_shepp_path((vehicle_pose_proj[0], vehicle_pose_proj[1], self.yaw_of_parked_cars), self.parking_spot_to_go[0])
            
            self.waypoints_for_obstacles_check = waypoints + waypoints2
            self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:, :2]
            
            # Exit if the trajectory is collision free
            if is_trajectory_collision_free(self.waypoints_for_obstacles_check,
                                                              self.vehicle_dims,
                                                              self.objects_to_avoid_collisions):
                break
            
            
            # if self.x_axis_of_search_direction_positive:
            #     self.x_axis_of_search += self.search_step_size
            #     if self.x_axis_of_search > self.static_horizontal_curb[1][0] + self.compact_parking_spot_size[1]:
            #         raise ValueError("No parking spot available.")
            # else:
            #     self.x_axis_of_search -= self.search_step_size
            #     if self.x_axis_of_search < self.static_horizontal_curb[0][0] - self.compact_parking_spot_size[1]:
            #         raise ValueError("No parking spot available.")