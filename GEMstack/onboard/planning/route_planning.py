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
ok = False

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

    def update(self, vehicle: VehicleState, x=10.0):
        self.current_pose = vehicle.pose

        # TODO: Cone detection -> parking position dataset -> select the nearest one
        # TODO: Add angle and gear direction in waypoints. 
        # TODO: Check if the route is feasable (i.e. no collision), if not, re-plan the route by
        #       construcating holonomic path some path planning algorithm (e.g. RRT, A*).
        #       Accross holonomic path by a method of deveision find sequence of Reeds-Shepp path 
        #       that connect the start and goal pose.
    
        
        # waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (14.0, 0.0, 0.0))
        # waypoints2 , waypoints_for_obstacles_check2= reeds_shepp_path((14.0 , 0.0, 0.0), (19.0, -1.9, 0.0))
        # waypoints = np.concatenate((waypoints, waypoints2), axis=0)
        # waypoints3 , waypoints_for_obstacles_check3 = reeds_shepp_path((19.0, -1.9, 0.0), (20.5, -1.9, 0.0))
        # waypoints = np.concatenate((waypoints, waypoints3), axis=0)
        
        
        # waypoints_for_obstacles_check = (
        #     waypoints_for_obstacles_check +
        #     waypoints_for_obstacles_check2 +
        #     waypoints_for_obstacles_check3
        # )



        # TODO: get from the cone detection, dicuss whether it is possible to have direction
        obstacles = [
            # centered at (2.5, 0.0), aligned with world frame, size 1.7, 2.45 m
            (27.0, -1.9, 0.0, (1.7, 2.45)),
            # # another at (4.0, 1.0), rotated 45°
            (15.0, -1.9, 0.0, (1.7, 2.45))
        ]

        vehicle_dims = (1.7, 2.45)
        # ok = is_trajectory_collision_free(waypoints_for_obstacles_check, vehicle_dims, obstacles)
        
        waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (x, 0.0, 0.0))
        waypoints2 , waypoints_for_obstacles_check2 = reeds_shepp_path((x, 0.0, 0.0), (15.0 + (27.0 - 15.0)/2, -1.9, 0.0))
        waypoints = np.concatenate((waypoints, waypoints2), axis=0)
            
        waypoints_for_obstacles_check = (
        waypoints_for_obstacles_check +
        waypoints_for_obstacles_check2
        )
        
        #x = 10.0
        while is_trajectory_collision_free(waypoints_for_obstacles_check, vehicle_dims, obstacles) is not True:
            x += 0.2
            waypoints, waypoints_for_obstacles_check= reeds_shepp_path((0.0, 0.0, 0.0), (x, 0.0, 0.0))
            waypoints2 , waypoints_for_obstacles_check2 = reeds_shepp_path((x, 0.0, 0.0), (20.5, -1.9, 0.0))
            waypoints = np.concatenate((waypoints, waypoints2), axis=0)
            
            waypoints_for_obstacles_check = (
            waypoints_for_obstacles_check +
            waypoints_for_obstacles_check2
            )
        print("x;", x)    
            

        print("Collision‐free?" ,is_trajectory_collision_free(waypoints_for_obstacles_check, vehicle_dims, obstacles))
        self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())

        return self.route

