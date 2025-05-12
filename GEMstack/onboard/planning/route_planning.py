from typing import List
from ..component import Component
from ...utils import serialization
from ...state import AllState, Roadgraph, Route, MissionEnum, ObjectFrameEnum, Path, ObjectPose, RoadgraphRegionEnum
import os
import numpy as np
from .RRT import BiRRT
from .reeds_shepp_parking import ReedsSheppParking


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
        left_x, left_y = goal
        right_x, right_y = goal
        goal_lane = None
        min_right_dist = np.inf
        min_right_idx = None
        for lane in roadgraph.lanes.values():
            for pts in lane.right.segments:
                pts = np.array(pts)
                dists = np.linalg.norm(pts[:, :2] - goal, axis=1)
                min_right_idx = np.argmin(dists)
                dist = dists[min_right_idx]
                if dist < min_right_dist:
                    min_right_dist = dist
                    right_x, right_y, _ = pts[min_right_idx]
                    goal_lane = lane

        # Find the closest point in left boundary to the point in right boundary
        min_left_dist = np.inf
        for pts in goal_lane.left.segments:
            pts = np.array(pts)
            dists = np.linalg.norm(pts[:, :2] - np.array([right_x, right_y]), axis=1)
            min_left_idx = np.argmin(dists)
            dist = dists[min_left_idx]
            if dist < min_left_dist:
                left_x, left_y, _ = pts[min_left_idx]

        goal_x = (left_x + right_x) / 2
        goal_y = (left_y + right_y) / 2

        if goal_yaw is None:
            # Find orientation
            if 0 < min_right_idx < len(pts) - 1:
                tangent = pts[min_right_idx + 1] - pts[min_right_idx - 1]
            elif min_right_idx == 0:
                tangent = pts[1] - pts[0]
            else:  # idx == last point
                tangent = pts[-1] - pts[-2]
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


def find_parallel_parking_lots(roadgraph: Roadgraph, goal_pose: ObjectPose, max_lane_to_parking_lot_gap=1.0):
    # Find the lane where the goal position is.
    goal_lane = find_closest_lane([goal_pose.x, goal_pose.y], roadgraph)
    goal_lane_points = np.array(goal_lane.right.segments[0])

    # Find the parking lots that attached to the lane
    parking_lots = []
    for region in roadgraph.regions.values():
        if region.type == RoadgraphRegionEnum.PARKING_LOT:
            for pt in region.outline:
                dist = np.linalg.norm(goal_lane_points[:, :2] - np.array(pt[:2]), axis=1)
                if np.min(dist) < max_lane_to_parking_lot_gap:
                    parking_lots.append(region.outline)
                    break

    # Find the closest and farthest parking lots and the middle points of the start and end curves as the parking area
    # Assume that the closest lot is close to the start of the lane, and the farthest lot is close to the end.
    closest_lot = None
    farthest_lot = None
    closest_start_point = None
    closest_start_index = None
    closest_end_point = None
    closest_end_index = None
    min_start_dist = np.inf
    min_end_dist = np.inf
    parking_area_start_end = None

    def next_point(index, outline, direction='ccw'):
        if direction == 'ccw':
            next_index = index + 1
        else:
            next_index = index - 1
        if next_index == len(outline):
            next_index = 0
        return outline[next_index]

    if len(parking_lots) > 0:
        for outline in parking_lots:
            for idx, pt in enumerate(outline):
                start_dist = np.linalg.norm(goal_lane_points[0, :2] - np.array(pt[:2]))
                end_dist = np.linalg.norm(goal_lane_points[-1, :2] - np.array(pt[:2]))
                if start_dist < min_start_dist:
                    min_start_dist = start_dist
                    closest_start_point = pt
                    closest_start_index = idx
                    closest_lot = outline
                if end_dist < min_end_dist:
                    min_end_dist = end_dist
                    closest_end_point = pt
                    closest_end_index = idx
                    farthest_lot = outline
        # Find the middle point of the start curve and the end curve of the parking area
        parking_area_start = (np.array(closest_start_point) + np.array(
            next_point(closest_start_index, closest_lot, direction='ccw'))) / 2
        parking_area_end = (np.array(closest_end_point) + np.array(
            next_point(closest_end_index, farthest_lot, direction='cw'))) / 2
        parking_area_start_end = [parking_area_start, parking_area_end]

    return parking_lots, parking_area_start_end


class SummoningRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""

    def __init__(self, roadgraphfn: str = None, map_type: str = 'roadgraph', map_frame: str = 'start'):
        self.planner = None
        self.route = None
        self.map_type = map_type
        self.lane_points = []
        self.map_boundary = None

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
        # if self.roadgraph.frame is not ObjectFrameEnum.START:
        print("=+++++++++++++++++++++++++",state.start_vehicle_pose)
        self.roadgraph = self.roadgraph.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose)
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
        obstacle_list = []
        for obstacle in obstacles.values():
            obstacle_list.append([obstacle.pose.x, obstacle.pose.y])
        self.obstacle_list = np.array(obstacle_list)

        """ Route planing in different mission states """
        # Idle mode. No mission, no driving. (Added by Summoning)
        if mission.type == MissionEnum.IDLE:
            print("I am in IDLE mode")
            if state.vehicle.v < 0.01:
                self.route = None

        # Summoning driving mode.
        elif mission.type == MissionEnum.SUMMONING_DRIVE:
            print("I am in SUMMON_DRIVING mode")
            if self.route is None:
                # Find appropri ate start and goal points that are on the lanes and fix for searching
                start_pose = find_available_pose_in_lane([vehicle.pose.x, vehicle.pose.y],
                                                         self.roadgraph, goal_yaw=vehicle.pose.yaw,
                                                         map_type=self.map_type)
                goal_pose = find_available_pose_in_lane([mission.goal_pose.x, mission.goal_pose.y],
                                                        self.roadgraph, map_type=self.map_type)
                print('Start pose:', start_pose)
                print('Goal pose:', goal_pose)

                # Search for waypoints
                searcher = BiRRT(self.lane_points, self.map_boundary, update_rate=self.update_rate)
                waypoints = searcher.search(start_pose, goal_pose, obstacle_list=self.obstacle_list.tolist())

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
                print("Vehicle is moving, stop it first.")
                self.route = Route(frame=ObjectFrameEnum.START, points=[[vehicle.pose.x, vehicle.pose.y],[vehicle.pose.x, vehicle.pose.y]])

            if self.map_type == 'roadgraph':
                parking_lots, parking_area_start_end = find_parallel_parking_lots(self.roadgraph, vehicle.pose)
                self.reedssheppparking.static_horizontal_curb_xy_coordinates = parking_area_start_end

                self.parking_velocity_is_zero = True

                # TODO: Test only. Remove it later.
                # self.reedssheppparking.static_horizontal_curb_xy_coordinates = [(0, -3),(30, -3)]
                self.obstacle_list= [(4, -3),(24, -3)]
                print("Parking area start and end:", self.reedssheppparking.static_horizontal_curb_xy_coordinates)
                print("Obstacle list:", self.obstacle_list)

                if not self.parking_route_existed:
                    self.current_pose = [vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw]
                    self.reedssheppparking.find_available_parking_spots_and_search_vector(self.obstacle_list,
                                                                                          self.current_pose)
                    self.reedssheppparking.find_collision_free_trajectory_to_park(self.obstacle_list, self.current_pose, True)
                    self.parking_route_existed = True

                else:
                    self.waypoints_to_go = self.reedssheppparking.waypoints_to_go
                    self.route = Route(frame=ObjectFrameEnum.START, points=self.waypoints_to_go.tolist())
                    # print("Route:", self.route)

        else:
            print("Unknown mode")

        if self.route is not None:
            print("Route existed.")

        return self.route