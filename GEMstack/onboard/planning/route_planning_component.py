import os
from typing import Dict, List

import numpy as np
from numpy import ndarray

# from GEMstack.onboard.component import Component
# from GEMstack.state.agent import AgentState
# from GEMstack.state.all import AllState,
# from GEMstack.state.physical_object import ObjectFrameEnum
# from GEMstack.state.route import PlannerEnum, Route
from .rrt_star import RRTStar
from ..interface.gem import GEMInterface
from ..component import Component
from ...state import AllState, Roadgraph, Route, PlannerEnum, ObjectFrameEnum, Path, \
    VehicleState, AgentState, AgentEnum, RoadgraphRegion, RoadgraphRegionEnum
from .RRT import BiRRT
from ...utils import serialization



def get_lane_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    """
    Get all points of the lanes in a roadgraph object.
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


def find_available_pose_in_lane(position, roadgraph, pose_yaw=None, map_type='roadgraph'):
    goal = np.array(position)
    if map_type == 'roadgraph':
        left_x, left_y = goal
        right_x, right_y = goal
        goal_lane = None
        min_dist = np.inf
        goal_yaw = pose_yaw
        for lane in roadgraph.lanes.values():
            for pts in lane.right.segments:
                pts = np.array(pts)
                dists = np.linalg.norm(pts[:, :2] - goal, axis=1)
                min_idx = np.argmin(dists)
                dist = dists[min_idx]
                if dist < min_dist:
                    min_dist = dist
                    right_x, right_y, _ = pts[min_idx]
                    goal_lane = lane

                    # Find orientation
                    if 0 < min_idx < len(pts) - 1:
                        tangent = pts[min_idx + 1] - pts[min_idx - 1]
                    elif min_idx == 0:
                        tangent = pts[1] - pts[0]
                    else:  # idx == last point
                        tangent = pts[-1] - pts[-2]
                    tangent_unit = tangent / np.linalg.norm(tangent)
                    goal_yaw = np.arctan2(tangent_unit[1], tangent_unit[0])

        min_dist = np.inf
        for pts in goal_lane.left.segments:
            pts = np.array(pts)
            dists = np.linalg.norm(pts[:, :2] - np.array([right_x, right_y]), axis=1)
            min_idx = np.argmin(dists)
            dist = dists[min_idx]
            if dist < min_dist:
                left_x, left_y, _ = pts[min_idx]

        goal_x = (left_x + right_x) / 2
        goal_y = (left_y + right_y) / 2

        if pose_yaw is not None:
            return [goal_x, goal_y, pose_yaw]
        else:
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


def find_closest_lane(position, roadgraph : Roadgraph):
    position = np.array(position)
    closest_lane = None
    min_dist = np.inf
    for lane in roadgraph.lanes.values():
        for pts in lane.right.segments:
            pts = np.array(pts)
            dists = np.linalg.norm(pts[:, :2] - position, axis=1)
            min_idx = np.argmin(dists)
            dist = dists[min_idx]
            if dist < min_dist:
                right_x, right_y, _ = pts[min_idx]
                min_dist = dist
                closest_lane = lane
    return closest_lane


def find_parking_lots(roadgraph, goal_pose, max_lane_to_parking_lot_gap=1.0):
    # TODO: incomplete, finish goal_parking_area_central_axis
    parking_lots = []
    for region in roadgraph.regions.values():
        if region.type == RoadgraphRegionEnum.PARKING_LOT:
            parking_lots.append(region.outline)

    goal_lane = find_closest_lane(goal_pose, roadgraph)

    goal_parking_lots = []
    for lane_points in goal_lane.right.segments:
        for outline in parking_lots:
            for pt in outline:
                if np.linalg.norm(np.array(lane_points) - np.array(pt[:, :2]), axis=1) < max_lane_to_parking_lot_gap:
                    goal_parking_lots.append(outline)

    goal_parking_area_central_axis = None

    return goal_parking_area_central_axis


class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, roadgraphfn : str = None, map_type : str = 'roadgraph', map_frame : str = 'start'):
        self.planner = None
        self.route = None
        self.map_type = map_type
        self.lane_points = []
        self.map_boundary = None

        print(map_type, map_frame)

        """ Read offline map of lanes """
        if map_frame == 'global':
            self.map_frame = ObjectFrameEnum.GLOBAL
        elif map_frame == 'start':
            self.map_frame = ObjectFrameEnum.START
        else:
            raise ValueError("Frame argument not available. Should be 'start' or 'global'.")

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

        # If parking route existed, not search anymore. For simulation testing only. TODO: Delete after integration
        self.parking_route_existed = False

        # Used as route searchers' time limit as well as the update rate of the component
        self.update_rate = 0.5

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return self.update_rate

    def update(self, state: AllState):
        print("Route Planner's mission:", state.mission_plan.planner_type, state.mission_plan.goal_pose)
        # print("type of mission plan:", type(PlannerEnum.RRT_STAR))
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value)
        # print("Route Planner's mission:", state.mission_plan.planner_type.value == PlannerEnum.PARKING.value)
        # print("Mission plan:", state.mission_plan)
        print("Vehicle x:", state.vehicle.pose.x)
        print("Vehicle y:", state.vehicle.pose.y)
        print("Vehicle yaw:", state.vehicle.pose.yaw)

        """ Transform offline map to start frame """
        if self.roadgraph.frame is not ObjectFrameEnum.START:
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
        """"""""""""

        if state.mission_plan.planner_type.value == PlannerEnum.PARKING.value:
            print("I am in PARKING mode")
            # Return a route after doing some processing based on mission plan REMOVE ONCE OTHER PLANNERS ARE IMPLEMENTED
            base_path = os.path.dirname(__file__)
            file_path = os.path.join(base_path, "../../knowledge/routes/forward_15m_extra.csv")
        
            waypoints = np.loadtxt(file_path, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                    waypoints = waypoints[:,:2]
            print("waypoints", waypoints)
            self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())

        elif state.mission_plan.planner_type.value == PlannerEnum.RRT_STAR.value:
            print("I am in RRT mode")
            start = (state.vehicle.pose.x+1, state.vehicle.pose.y+1)
            goal = (state.mission_plan.goal_x, state.mission_plan.goal_y) #When we implement kinodynamic, we need to include target_yaw also
            x_bounds = (0,20)
            y_bounds = (0,20)
            step_size = 1.0
            max_iter = 2000
            occupancy_grid = np.zeros((20, 20), dtype=int) 
            occupancy_grid[5:10, 5:10] = 1
            self.planner = RRTStar(start, goal, x_bounds, y_bounds, max_iter=max_iter, step_size=step_size, vehicle_width=1, occupancy_grid=occupancy_grid)
            rrt_resp = self.planner.plan()
            self.route = Route(frame=ObjectFrameEnum.START, points=rrt_resp)

        # Idle mode. No mission, no driving. (Added by Summoning)
        elif state.mission_plan.planner_type == PlannerEnum.IDLE:
            print("I am in IDLE mode")
            if state.vehicle.v < 0.01:
                self.route = None

        # Summoning driving mode. TODO: to be integrated with planning team's searcher
        elif state.mission_plan.planner_type == PlannerEnum.SUMMON_DRIVING:
            print("I am in SUMMON_DRIVING mode")

            # Find appropriate start and goal points that are on the lanes and fix for searching
            start_pose = find_available_pose_in_lane([state.vehicle.pose.x, state.vehicle.pose.y],
                                                    self.roadgraph, pose_yaw=state.vehicle.pose.yaw, map_type=self.map_type)
            goal_pose = find_available_pose_in_lane([state.mission_plan.goal_pose.x, state.mission_plan.goal_pose.y],
                                                    self.roadgraph, map_type=self.map_type)
            print('Start pose:', start_pose)
            print('Goal pose:', goal_pose)

            # Search for waypoints
            searcher = BiRRT(start_pose, goal_pose, self.lane_points, self.map_boundary, update_rate=self.update_rate)
            waypoints = searcher.search()

            # For now, waypoints of [x, y, heading] is not working in longitudinal_planning. Use [x, y] instead.
            if waypoints:
                waypoints = np.array(waypoints)
                if waypoints.shape[1] == 3:
                    waypoints = waypoints[:, :2]
                waypoints = waypoints.tolist()

            if waypoints:
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints)
            else:
                self.route = state.route   # Fail to find a path, keep the origin route.

        # Parallel parking mode.
        elif state.mission_plan.planner_type == PlannerEnum.PARALLEL_PARKING:
            print("I am in PARALLEL_PARKING mode")

            if self.parking_route_existed is False:

                vehicle_pose = [state.vehicle.pose.x, state.vehicle.pose.y, state.vehicle.pose.yaw]

                detected_cones = []
                for name, agent in state.agents.items():
                    if agent.type == AgentEnum.CONE:
                        detected_cones.append([agent.pose.x, agent.pose.y])

                if self.map_type == 'roadgraph':
                    parking_slots = []
                    for name, region in self.roadgraph.regions.items():
                        if region.type == RoadgraphRegionEnum.PARKING_LOT:
                            parking_slots.append(region.outline)
                else:
                    parking_slots = None

                # print("Parking slots:", parking_slots)

                # Parking slots: [[(30.0, 12.5), (30.0, 15), (0.0, 15), (0.0, 12.5)]]
                # curbs => center points of both sides of the parking slot
                # Choose closest point and set to 1st curb
                # ex) vehicle_pose = (30,15,3.14)
                #       curbs = [(30.0, 13.75),(0.0, 13.75)]
                # ex) vehicle_pose = (0,15,0)
                #       curbs = [(0.0, 13.75),(30.0, 13.75)]

                # TODO: Just stop when there is no parking slot

                # TODO: Just stop when slot is not available after detection

                # TODO: Test only. Move to roadgraph or comment out
                # Scenario 1: Start parking from [0,0], park around [12, -2.44]
                parking_slots = [[(0.0, 0.0), (0.0, -4.88), (24.9, -4.88), (24.9, 0.0)]]
                detected_cones = [
                    (2.69, -2.44),
                    (22.11, -2.44)  
                ]

                # Find the closest parking slot point to the current vehicle position
                closest_parking_slot = None
                closest_parking_slot_point = None
                min_distance_to_slot = 0
                for ps in parking_slots:
                    for pt in ps:
                        d = math.dist((vehicle_pose[0], vehicle_pose[1]), pt)
                        if closest_parking_slot_point is None or d < min_distance_to_slot:
                            closest_parking_slot = ps
                            closest_parking_slot_point = pt
                            min_distance_to_slot = d
                print("Closest parking slot:", closest_parking_slot)
                print("Closest parking slot point:", closest_parking_slot_point)

                # curb1: closest curb. calculate from the center of 1st and 2nd points
                curb1 = ((closest_parking_slot[0][0] + closest_parking_slot[1][0]) / 2,
                        (closest_parking_slot[0][1] + closest_parking_slot[1][1]) / 2)
                # curb2: the other curb. calculate from the center of 3rd and 4th points
                curb2 = ((closest_parking_slot[2][0] + closest_parking_slot[3][0]) / 2,
                        (closest_parking_slot[2][1] + closest_parking_slot[3][1]) / 2)
                print("Curb:", [curb1, curb2])

                print("Detected cones:", detected_cones)
                
                if closest_parking_slot:
                    searcher = ReedsSheppParking()
                    searcher.add_static_vertical_curb_as_obstacle = False
                    searcher.add_static_horizontal_curb_as_obstacle = False
                    searcher.static_horizontal_curb_xy_coordinates = [curb1, curb2]
                    searcher.find_available_parking_spots_and_search_vector(detected_cones)
                    searcher.find_collision_free_trajectory_to_park(detected_cones)
                    self.route = Route(frame=ObjectFrameEnum.START, points=searcher.waypoints_to_go.tolist())
                    print("Route:", self.route)
                    self.parking_route_existed = True
                # If there is no parking slot, stop the vehicle
                else:
                    print("No parking slot available. Stop the vehicle.")
                    return None
            
            # There is a parking route already
            else:
                self.route = state.route

        else:
            print("Unknown mode")

        if self.route is not None:
            print("Route existed.")
        
        return self.route