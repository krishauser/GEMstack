from typing import List
from ..component import Component
#these could be helpful in your implementation
from .longitudinal_planning import longitudinal_brake, longitudinal_plan
from ...state import AllState, VehicleState, PhysicalObject, AgentEnum, AgentState, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization,settings
from ...mathutils.transforms import vector_madd
from ...mathutils import collisions
import copy
import math
import numpy as np
from typing import Dict

def calculate_centroid(polygon):
    """Calculate the centroid of a polygon."""
    xs, ys = zip(*polygon)
    centroid_x = sum(xs) / len(polygon)
    centroid_y = sum(ys) / len(polygon)
    return (centroid_x, centroid_y)
class PedestrianAvoidanceMotionPlanner(Component):
    """Follows the given route.  Brakes to yield to pedestrians or you are at the
    end of the route, otherwise accelerates to the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5
        self.desired_speed = 1.0
        self.deceleration = 1.8

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        vehicle = copy.deepcopy(state.vehicle) # type: VehicleState
        route = state.route   # type: Route
        agents = state.agents # type: Dict[str,AgentState]
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        #put the route and the agents in the vehicle's frame
        if state.route.frame != vehicle.pose.frame:
            state.route = state.route.to_frame(vehicle.pose.frame, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)
        for k,a in agents.items():
            a.pose = a.pose.to_frame(vehicle.pose.frame, current_pose=state.vehicle.pose, start_pose_abs=state.start_vehicle_pose)

        #figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        all_pedestrians = []
        for k,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:
                all_pedestrians.append(a)

        #extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)
        route_with_lookahead = route_with_lookahead.arc_length_parameterize()

        #TODO: use the collision detection primitives to determine whether to stop for a pedestrian
        #TODO: modify the margins around the vehicle to keep a safe distance from pedestrians
        longitudinal_d = settings.get('avoidance.longitudinal_d')
        lateral_d = settings.get('avoidance.lateral_d')
        should_brake = False
        all_pedestrians = []
        for k,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:
                all_pedestrians.append(a)
        collision_check_resolution = 0.1  #m
        progress_start, progress_end = route_with_lookahead.domain()
        print("progress_start",progress_start)
        print("progress_end",progress_end)

        progress = progress_start
        max_progress = None
        while progress < progress_end:
            xy = route_with_lookahead.eval(progress)
            vehicle.pose.x = xy[0]
            vehicle.pose.y = xy[1]
            try:
                v = route_with_lookahead.eval_derivative(t)
                vehicle.pose.yaw = math.atan2(v[1],v[0])
            except Exception:
                #path has only one point?
                vehicle.pose.yaw = 0
            #this is a CCW polygon specifying the vehicle's position at the given progress, in the START frame
            vehicle_poly_world = vehicle.to_object().polygon_parent()
            vehicle_p = vehicle_poly_world
            centroid = calculate_centroid(vehicle_p)
            vehicle_p[0][0] = vehicle_p[0][0] - (lateral_d)
            vehicle_p[0][1] = vehicle_p[0][1] - (longitudinal_d)

            vehicle_p[1][0] = vehicle_p[1][0] + (lateral_d)
            vehicle_p[1][1] = vehicle_p[1][1] - (longitudinal_d)

            vehicle_p[2][0] = vehicle_p[2][0] + (lateral_d)
            vehicle_p[2][1] = vehicle_p[2][1] + (longitudinal_d)

            vehicle_p[3][0] = vehicle_p[3][0] - (lateral_d)
            vehicle_p[3][1] = vehicle_p[3][1] + (longitudinal_d)
            if len(all_pedestrians) > 0:
                #TODO: figure out a good way to check for collisions with pedestrians with the desired margins

                # for x in range(len(vehicle_poly_world)):
                #     direction_to_centroid = (centroid[0] - vehicle_poly_world[x][0], centroid[1] - vehicle_poly_world[x][1])
                #     length = np.sqrt(direction_to_centroid[0] ** 2 + direction_to_centroid[1] ** 2)
                #     if length == 0:  # To handle degenerate case
                #         continue
                #     normalized_direction = (-direction_to_centroid[0] / length, -direction_to_centroid[1] / length)
                #     vehicle_p[x][0] = vehicle_poly_world[x][0] + (normalized_direction[0] *(lateral_d/2))
                #     vehicle_p[x][1] = vehicle_poly_world[x][1] + (normalized_direction[1] *(longitudinal_d / 2))


                if any([collisions.polygon_intersects_polygon_2d(vehicle_p,a.polygon_parent()) for a in all_pedestrians]):
                    print("Predicted collision with pedestrian at",progress)
                    print("Vehicle position",xy[0],xy[1])
                    #print("Pedestrian position",a.pose.x,a.pose.y)
                    #print("Vehicle poly",vehicle_poly_world)
                    #print("Pedestrian poly",a.polygon_parent())
                    #prevent
                    max_progress = max(progress - collision_check_resolution,0.0)
                    should_brake = True
                    break

            progress += collision_check_resolution
        if max_progress is not None:
            #allow enough room to brake from the current velocity
            braking_distance = 0.5*(curr_v)**2/self.deceleration * 0.96
            max_progress = max(max_progress,braking_distance)
            route_with_lookahead = route_with_lookahead.trim(progress_start,max_progress)
        else:
            max_progress = progress_end
        print("Progress",route_with_lookahead.domain()[1])

        should_acc = ((not should_brake) and curr_v < self.desired_speed )
        #choose whether to accelerate, brake, or keep at current velocity
        if should_acc:
            print("=======Should accelerate============================================================")
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        # else:
        #     traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)
        # if max_progress > 0:
        #     traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        # else:
        #     traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        print("Stopped distance",traj.length())
        return traj                 