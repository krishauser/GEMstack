from typing import List
from ..component import Component
from ...utils import serialization
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


def get_all_lane_points(roadgraph: Roadgraph) -> List:
    all_points = []
    for value in roadgraph.lanes.values():
        for pts in value.left.segments:
            for pt in pts:
                all_points.append(pt)
    return all_points


def find_available_pose(position, lane_points):
    target_xy = np.array(position[:2])
    lane_np = np.array([[x, y] for x, y, _ in lane_points])

    dists = np.linalg.norm(lane_np - target_xy, axis=1)
    idx = np.argmin(dists)
    x, y = lane_np[idx]

    if idx < len(lane_points) - 1:
        next_pt = lane_points[idx + 1]
    elif idx > 0:
        next_pt = lane_points[idx - 1]
    else:
        return x, y, 0.0

    dx = next_pt[0] - x
    dy = next_pt[1] - y
    yaw = np.arctan2(dy, dx)

    return [x, y, yaw]


def generate_route_free_run(current_pose, goal_position, roadgraph, roadgraph_type, map_margin=5.0, try_times=5):
    """
    Assume the vehicle can run in both directions in the lanes
    """
    if roadgraph_type == 'roadgraph':
        all_lane_points = get_all_lane_points(roadgraph)
    else:  # roadgraph_type == 'point_list'
        all_lane_points = roadgraph
    points = np.array(all_lane_points)

    # Decide the route searching boundaries
    map_boundaries = [np.min(points[:, 0]) - map_margin, np.max(points[:, 0]) + map_margin,
                      np.min(points[:, 1]) - map_margin, np.max(points[:, 1]) + map_margin]

    start_pose = [current_pose.x, current_pose.y, current_pose.yaw]
    # TODO: find yaw with free lane direction
    goal_pose = find_available_pose(goal_position, points)

    searcher = RRT.BiRRT(start_pose, goal_pose, all_lane_points, map_boundaries)
    waypoints = []
    for _ in range(try_times):
        waypoints = searcher.search()
        if waypoints:
            break
    print("waypoints:", waypoints)
    if not waypoints:
        raise RuntimeError('No waypoints found')
    return waypoints


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str, frame : str = 'start'):
        self.frame = frame
        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(roadgraphfn, 'r') as f:
                self.roadgraph = serialization.load(f)
                self.roadgraph_type = 'roadgraph'
        elif ext == '.csv':
            self.roadgraph = np.loadtxt(roadgraphfn,delimiter=',',dtype=float)
            self.roadgraph_type = 'point_list'
        else:
            raise ValueError("Unknown roadgraph file extension",ext)

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, vehicle: VehicleState):
        self.current_pose = vehicle.pose

        # TODO: get from the server
        self.goal_positions = [(23.0, 3.0, 0.0),(2.0, 6.0, 0.0)]    # x, y, z

        waypoints = generate_route_free_run(self.current_pose, self.goal_positions[0], self.roadgraph, self.roadgraph_type)
        waypoints = np.array(waypoints)
        if waypoints.shape[1] == 3:
            waypoints = waypoints[:, :2]

        if self.frame == 'start':
            self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
        elif self.frame == 'global':
            self.route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints.tolist())
        elif self.frame == 'cartesian':
            self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints.tolist())

        return self.route
    



Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))


# TODO: Pass constant to settings or config, double-check value rho (turning radius)
def reeds_shepp_path(start_pose, final_pose, step_size=0.1, rho=3.657):
    path = reeds_shepp.path_sample(start_pose, final_pose, rho, step_size)
    waypoints_for_obstacles_check  = [(x, y, yaw) for x, y, yaw in path]
    waypoints = np.array(waypoints_for_obstacles_check)[:,:2]
    return waypoints , waypoints_for_obstacles_check


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

        # TODO: add curbs coordinates, assuming that given
        static_obstacles_horizontal_curb = [
            (-2.045, -1.9, 0.0, (0.64, 1.9)), # horizontal curb at start of parking lot
            (39.995, -1.9, 0.0, (0.64, 1.9)),  # horizontal curb at end of parking lot
        ]
        static_obstacles_vertical_curb = [(23.065, -3.17, 0.0, (0.64, 42.68))] # vertical curb accross the parking lot]

        # TODO: get from the cone detection, dicuss whether it is possible to have direction
        obstacles = [
            # car size obstacles in the parking lot 
            (5.0, -1.9, 0.0, (1.7, 2.45)),
            # another car size obstacle
            (16.8, -1.9, 0.0, (1.7, 2.45)) # (12.0, -1.9, 0.0, (1.7, 2.45)) 
        ]

        # TODO: To execute the parking be able drive vehicle in reverse: Talk to control group

        # TODO: get from gem_e4_geometry.yaml 
        vehicle_dims = (1.7, 2.45)

        # TODO: define method based on obstacles, find parking position and return parking position closest to the vehilce
        parking_spots= [
            (10.9, -1.9, 0.0)
        ]




        # free_slots = find_parking_positions(obstacles, vehicle_dims, margin=0.2)
        # print("Available parking slots (x, y, theta):")
        # for slot in free_slots:
        #     print(slot) 

        waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (x, 0.0, 0.0))
        waypoints2 , waypoints_for_obstacles_check2 = reeds_shepp_path((x, 0.0, 0.0), parking_spots[0])
        waypoints = np.concatenate((waypoints, waypoints2), axis=0) 
        
        waypoints_for_obstacles_check = (
        waypoints_for_obstacles_check +
        waypoints_for_obstacles_check2
        )   
        
        while is_trajectory_collision_free(waypoints_for_obstacles_check, vehicle_dims, obstacles) is not True:
            x += 0.2
            waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (x, 0.0, 0.0))
            waypoints2 , waypoints_for_obstacles_check2 = reeds_shepp_path((x, 0.0, 0.0), parking_spots[0])
            waypoints = np.concatenate((waypoints, waypoints2), axis=0)
            
            waypoints_for_obstacles_check = (
            waypoints_for_obstacles_check +
            waypoints_for_obstacles_check2
            )
            
        

        # print("Collision‐free?" ,is_trajectory_collision_free(waypoints_for_obstacles_check, vehicle_dims, obstacles))
        # print("Collision‐free Start coordinates:", (x, 0.0, 0.0))
        # print("Collision‐free Goal coordinates:", parking_spots[0])
        self.route = Route(frame=ObjectFrameEnum.START,points = waypoints.tolist())

        return self.route

