from typing import List
from ..component import Component
from ...utils import serialization
from ...state import AllState, Roadgraph, Route, MissionEnum, ObjectFrameEnum, Path, ObjectPose, RoadgraphRegionEnum
import os
import numpy as np
from .RRT import BiRRT
import yaml
import os


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


def get_lane_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    """
    Get all points of the lanes in a Roadgraph.
    Ouput: A list of [x, y]
    """
    lane_points = []
    for lane in roadgraph.lanes.values():
        if not lane.left.crossable:
            for pts in lane.left.segments:
                for pt in pts:
                    lane_points.append(pt[:2])
        if not lane.right.crossable:
            for pts in lane.right.segments:
                for pt in pts:
                    lane_points.append(pt[:2])
    return lane_points


def find_available_pose_in_lane(position: list, roadgraph: Roadgraph, goal_yaw=None, map_type='roadgraph'):
    goal = np.array(position)
    if map_type == 'roadgraph':
        goal_lane = None
        min_right_dist = np.inf
        goal_lane_right_seg_idx = None
        min_right_idx = None
        for lane in roadgraph.lanes.values():
            for seg_idx, right_pts in enumerate(lane.right.segments):
                right_pts = np.array(right_pts)
                dists = np.linalg.norm(right_pts[:, :2] - goal, axis=1)
                min_idx = np.argmin(dists)
                dist = dists[min_idx]
                if dist < min_right_dist:
                    min_right_dist = dist
                    min_right_idx = min_idx
                    goal_lane_right_seg_idx = seg_idx
                    goal_lane = lane
        goal_lane_right_boundary = goal_lane.right.segments[goal_lane_right_seg_idx]
        right_x, right_y = goal_lane_right_boundary[min_right_idx][:2]

        # Find the closest point in left boundary to the point in right boundary
        min_left_dist = np.inf
        goal_lane_left_seg_idx = None
        min_left_idx = None
        for seg_idx, left_pts in enumerate(goal_lane.left.segments):
            left_pts = np.array(left_pts)
            dists = np.linalg.norm(left_pts[:, :2] - np.array([right_x, right_y]), axis=1)
            min_idx = np.argmin(dists)
            dist = dists[min_idx]
            if dist < min_left_dist:
                min_left_dist = dist
                min_left_idx = min_idx
                goal_lane_left_seg_idx = seg_idx
        goal_lane_left_boundary = goal_lane.left.segments[goal_lane_left_seg_idx]
        left_x, left_y = goal_lane_left_boundary[min_left_idx][:2]

        goal_x = (left_x + right_x) / 2
        goal_y = (left_y + right_y) / 2

        # Find the heading along the lane
        if goal_yaw is None:
            # Find orientation
            goal_lane_right_boundary = np.array(goal_lane_right_boundary)
            if 0 < min_right_idx < len(goal_lane_right_boundary) - 1:
                tangent = goal_lane_right_boundary[min_right_idx + 1] - goal_lane_right_boundary[min_right_idx - 1]
            elif min_right_idx == 0:
                tangent = goal_lane_right_boundary[1] - goal_lane_right_boundary[0]
            else:  # idx == last point
                tangent = goal_lane_right_boundary[-1] - goal_lane_right_boundary[-2]
            tangent_unit = tangent / np.linalg.norm(tangent)
            goal_yaw = np.arctan2(tangent_unit[1], tangent_unit[0])

        return [goal_x, goal_y, goal_yaw]

    elif map_type == 'pointlist':
        pts = np.array(roadgraph)
        dists = np.linalg.norm(pts[:, :2] - goal, axis=1)
        min_idx = np.argmin(dists)
        if 0 < min_idx < len(pts) - 1:
            if np.linalg.norm(pts[min_idx] - pts[min_idx - 1]) < np.linalg.norm(pts[min_idx + 1] - pts[min_idx]):
                tangent = pts[min_idx] - pts[min_idx - 1]
            else:
                tangent = pts[min_idx + 1] - pts[min_idx]
        elif min_idx == 0:
            tangent = pts[1] - pts[0]
        else:  # idx == last point
            tangent = pts[-1] - pts[-2]
        tangent_unit = tangent / np.linalg.norm(tangent)
        goal_yaw = np.arctan2(tangent_unit[1], tangent_unit[0])

        return [goal[0], goal[1], goal_yaw]

    else:
        raise ValueError('map_type must be one of "roadgraph", "pointlist"')


def find_closest_lane(position: list, roadgraph: Roadgraph, traffic_rule='right'):
    position = np.array(position)
    closest_lane = None
    min_dist = np.inf

    for lane in roadgraph.lanes.values():
        if traffic_rule == 'right':
            segments = lane.right.segments
        else:
            segments = lane.left.segments
        for pts in segments:
            pts = np.array(pts)
            dists = np.linalg.norm(pts[:, :2] - position, axis=1)
            min_idx = np.argmin(dists)
            dist = dists[min_idx]
            if dist < min_dist:
                right_x, right_y, _ = pts[min_idx]
                min_dist = dist
                closest_lane = lane
    return closest_lane


def find_parallel_parking_lots(roadgraph: Roadgraph, goal_pose: ObjectPose, start_vehicle_pose, farthest_parking_distance=40):
    # Find the lane where the goal position is.
    goal_lane = find_closest_lane([goal_pose.x, goal_pose.y], roadgraph)
    goal_lane_points = np.array(goal_lane.right.segments[0])

    closest_lot = None
    farthest_lot = None
    closest_point = None
    farthest_point = None
    closest_idx = None
    farthest_idx = None
    min_dist = np.inf
    max_dist = -np.inf
    # Find the parking lots attached to the lane in current frame of the goal pose
    parking_lots = []
    for region in roadgraph.regions.values():
        if region.type == RoadgraphRegionEnum.PARKING_LOT:
            region_current = region.to_frame(roadgraph.frame, ObjectFrameEnum.CURRENT, current_origin=goal_pose, global_origin=start_vehicle_pose)
            available = True
            for pt in region_current.outline:
                # Remove not available regions
                if pt[1] > 0 or pt[1] < -10 or pt[0] > farthest_parking_distance or pt[0] < 3:
                    available = False
            if available:
                parking_lots.append(region.outline)
                for i, pt in enumerate(region_current.outline):
                    dist = np.sqrt(pt[0]**2 + pt[1]**2)
                    if dist > max_dist:
                        max_dist = dist
                        farthest_idx = i
                        farthest_lot = region.outline
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                        closest_lot = region.outline

    def next_point(index, outline, direction='ccw'):
        if direction == 'ccw':
            next_index = index + 1
        else:
            next_index = index - 1
        if next_index == len(outline):
            next_index = 0
        return outline[next_index]

    # Find the closest and farthest parking lots and the middle points of the start and end curves as the parking area
    parking_area_start_end = None
    if len(parking_lots) > 0:
        # Find the middle point of the start curve and the end curve of the parking area
        closest_point = closest_lot[closest_idx]
        closest_next = next_point(closest_idx, closest_lot, direction='ccw')
        parking_area_start = [(closest_point[0]+closest_next[0])/2, (closest_point[1]+closest_next[1])/2]
        farthest_point = farthest_lot[farthest_idx]
        farthest_next = next_point(farthest_idx, farthest_lot, direction='ccw')
        parking_area_end = [(farthest_point[0]+farthest_next[0])/2, (farthest_point[1]+farthest_next[1])/2]
        parking_area_start_end = [parking_area_start, parking_area_end]

    return parking_lots, parking_area_start_end


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self, roadgraphfn: str = None, map_type: str = 'roadgraph', map_frame: str = 'start'):
        # Moving this import here to avoid dependency error related to reeds-shepp python package
        from .reeds_shepp_parking import ReedsSheppParking
        self.planner = None
        self.route = None
        self.map_type = map_type
        self.lane_points = []
        self.map_boundary = None
        self.obstacle_list = []
        self.goal_pose = None

        print(map_type, map_frame)

        """ Read offline map of lanes """
        if map_frame == 'global':
            self.map_frame = ObjectFrameEnum.GLOBAL
        elif map_frame == 'cartesian':
            self.map_frame = ObjectFrameEnum.ABSOLUTE_CARTESIAN
        elif map_frame == 'start':
            self.map_frame = ObjectFrameEnum.START
        else:
            raise ValueError("Frame argument not available. Should be 'start', 'cartesian' or 'global'.")

        base, ext = os.path.splitext(roadgraphfn)
        if ext in ['.json', '.yml', '.yaml']:
            if self.map_type == 'roadgraph':
                with open(roadgraphfn, 'r') as f:
                    self.roadgraph = serialization.load(f)
            else:
                raise ValueError('map_type must be "roadgraph" for ".json", ".yml", ".yaml" extensions.')
        elif ext in ['.csv', '.txt'] and self.map_type == 'pointlist':
            if self.map_type == 'pointlist':
                roadgraph = np.loadtxt(roadgraphfn, delimiter=',', dtype=float)
                self.roadgraph = Path(frame=self.map_frame, points=roadgraph.tolist())
            else:
                raise ValueError('map_type must be "pointlist" for ".csv", ".txt" extensions.')
        else:
            raise ValueError("Unknown roadgraph file extension", ext)

        # Used as route searchers' time limit as well as the update rate of the component
        self.update_rate = 0.5

        self.parked_cars = []
        self.reedssheppparking = ReedsSheppParking()
        self.reedssheppparking.static_horizontal_curb_xy_coordinates = None
        self.reedssheppparking.add_static_horizontal_curb_as_obstacle = False
        self.reedssheppparking.add_static_vertical_curb_as_obstacle = False
        self.parking_route_existed = False
        self.parking_velocity_is_zero = False


    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return self.update_rate

    def update(self, state: AllState):
        mission = state.mission
        vehicle = state.vehicle
        obstacles = state.obstacles
        route = state.route

        print("Mission:", mission.type, mission.goal_pose)
        print("Vehicle pose:", vehicle.pose)

        """ Transform offline map to start frame """
        start_vehicle_pose = state.start_vehicle_pose
        if start_vehicle_pose.frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN and self.map_frame == ObjectFrameEnum.GLOBAL:
            # For global map simulation test
            print(os.getcwd())
            with open("scenes/summoning_map_sim_setting.yaml", "r") as f:
                data = yaml.safe_load(f)
                start = data['start_pose_global']
            # Convert roadgraph to start frame
            start_vehicle_pose = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=start_vehicle_pose.t,
                                    x=start[0], y=start[1], yaw=start[2])

        self.roadgraph = self.roadgraph.to_frame(ObjectFrameEnum.START, start_pose_abs=start_vehicle_pose)

        # Get all the points of lanes
        if self.map_type == 'roadgraph':
            self.lane_points = get_lane_points_from_roadgraph(self.roadgraph)
        elif self.map_type == 'pointlist':
            self.lane_points = self.roadgraph.points
        # Define map boundary for searching
        margin = 10
        lane_points = np.array(self.lane_points)
        # Map_boundary: x_min, x_max, y_min, y_max
        self.map_boundary = [np.min(lane_points[:, 0]) - margin, np.max(lane_points[:, 0]) + margin,
                             np.min(lane_points[:, 1]) - margin, np.max(lane_points[:, 1]) + margin]

        """ Get obstacle list """
        # TODO: Include dimension
        # Predefined offline obstacles
        with open("scenes/summoning_map_sim_setting.yaml", "r") as f:
            data = yaml.safe_load(f)
            obs_frame = data['obstacles']['frame']
            offline_obstacles = data['obstacles']['positions']
        # Convert obstacles ot start frame
        for obstacle in obstacles:
            if obs_frame == 'cartesian':
                obs_pose = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, t=start_vehicle_pose.t, x=offline_obstacles[0], y=offline_obstacles[1])
            elif obs_frame == 'start':
                obs_pose = ObjectPose(frame=ObjectFrameEnum.START, t=start_vehicle_pose.t, x=offline_obstacles[0], y=offline_obstacles[1])
            elif obs_frame == 'global':
                obs_pose = ObjectPose(frame=ObjectFrameEnum.GLOBAL, t=start_vehicle_pose.t, x=offline_obstacles[0], y=offline_obstacles[1])
            else:
                raise ValueError("Unknown obstacle frame", obs_frame)
            obs_pose = obs_pose.to_frame(ObjectFrameEnum.START, start_pose_abs=start_vehicle_pose)
            self.obstacle_list.append([obs_pose.x, obs_pose.y])

        # Online obstacles
        for obstacle in obstacles.values():
            self.obstacle_list.append([obstacle.pose.x, obstacle.pose.y])

        """ Route planing in different mission states """
        # Idle mode. No mission, no driving. (Added by Summoning)
        if mission.type == MissionEnum.IDLE:
            print("I am in IDLE mode")
            if state.vehicle.v < 0.01:
                self.route = None

        # Summoning driving mode.
        elif mission.type == MissionEnum.SUMMON_DRIVING:
            print("I am in SUMMON_DRIVING mode")
            if self.route is None:
                # Find appropri ate start and goal points that are on the lanes and fix for searching
                start_pose = find_available_pose_in_lane([vehicle.pose.x, vehicle.pose.y],
                                                         self.roadgraph, goal_yaw=vehicle.pose.yaw,
                                                         map_type=self.map_type)
                goal_pose1 = find_available_pose_in_lane([mission.goal_pose.x, mission.goal_pose.y],
                                                        self.roadgraph, map_type=self.map_type)
                # Reverse direction of goal_pose1
                goal_pose2 = [goal_pose1[0], goal_pose1[1], (goal_pose1[2] + np.pi*2) % (2 * np.pi) - np.pi]

                # Search for waypoints. search for two goal yaws and choose the shorter path.
                searcher = BiRRT(self.lane_points, self.map_boundary, update_rate=self.update_rate)
                waypoints1 = searcher.search(start_pose, goal_pose1, obstacle_list=self.obstacle_list)
                waypoints2 = searcher.search(start_pose, goal_pose2, obstacle_list=self.obstacle_list)
                if len(waypoints1) != 0 and len(waypoints2) != 0:
                    if len(waypoints1) <= len(waypoints2):
                        waypoints = waypoints1
                        goal_pose = goal_pose1
                    else:
                        waypoints = waypoints2
                        goal_pose = goal_pose2
                elif len(waypoints1) != 0:
                    waypoints = waypoints1
                    goal_pose = goal_pose1
                elif len(waypoints2) != 0:
                    waypoints = waypoints2
                    goal_pose = goal_pose2
                else:
                    waypoints = []
                    goal_pose = goal_pose1

                print('Start pose:', start_pose)
                print('Goal pose:', goal_pose)
                self.goal_pose = ObjectPose(frame=ObjectFrameEnum.START, t=start_vehicle_pose.t, x=goal_pose[0], y=goal_pose[1], yaw=goal_pose[2])
                # For now, waypoints of [x, y, heading] is not working in longitudinal_planning. Use [x, y] instead.
                if waypoints:
                    waypoints = np.array(waypoints)
                    if waypoints.shape[1] == 3:
                        waypoints = waypoints[:, :2]
                    waypoints = waypoints.tolist()

                if waypoints:
                    self.route = Route(frame=ObjectFrameEnum.START, points=waypoints)
                else:
                    self.route = route  # Fail to find a path, keep the origin route.

        # Parallel parking mode.
        elif mission.type == MissionEnum.PARALLEL_PARKING:
            print("I am in PARALLEL_PARKING mode")

            if self.parking_velocity_is_zero == False and state.vehicle.v > 0.01:
                print("Vehicle is moving, wait to stop.")
                self.route = Route(frame=ObjectFrameEnum.START, points=[[vehicle.pose.x, vehicle.pose.y],[vehicle.pose.x, vehicle.pose.y]])

            elif self.map_type == 'roadgraph':
                parking_lots, parking_area_start_end = find_parallel_parking_lots(self.roadgraph, self.goal_pose, start_vehicle_pose)
                self.reedssheppparking.static_horizontal_curb_xy_coordinates = parking_area_start_end

                self.parking_velocity_is_zero = True

                print("Parking area start and end:", self.reedssheppparking.static_horizontal_curb_xy_coordinates)
                print("Obstacle list:", self.obstacle_list)

                if not self.reedssheppparking.static_horizontal_curb_xy_coordinates:
                    print("No parking area found, stop.")
                    self.route = None

                elif not self.parking_route_existed:
                    self.current_pose = [vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw]
                    self.reedssheppparking.find_available_parking_spots_and_search_vector(self.obstacle_list,
                                                                                          self.current_pose)
                    self.reedssheppparking.find_collision_free_trajectory_to_park(self.obstacle_list, self.current_pose, True)
                    self.parking_route_existed = True

                else:
                    self.waypoints_to_go = self.reedssheppparking.waypoints_to_go
                    if len(self.waypoints_to_go) > 0:
                        self.route = Route(frame=ObjectFrameEnum.START, points=self.waypoints_to_go.tolist())
                    else:
                        self.route = None
                        print("No route found, stop.")
            else:
                print("No parking lots, stop.")
                self.route = None

        else:
            print("Unknown mode")

        if self.route is not None:
            print("Route existed.")

        return self.route