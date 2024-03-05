from typing import List
from ..component import Component
#these could be helpful in your implementation
from .longitudinal_planning import longitudinal_brake, longitudinal_plan
from ...state import AllState, VehicleState, PhysicalObject, AgentEnum, AgentState, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
from ...mathutils import collisions
import copy
import math
import numpy as np
from typing import Dict

buffer_x = settings.get('planning.pedestrian_avoidance.buffer.longitudinal')
buffer_y = settings.get('planning.pedestrian_avoidance.buffer.lateral')

def get_resolution(vehicle, all_pedestrians, braking_distance): # collision check resolution
    res_min = settings.get('planning.pedestrian_avoidance.resolution.min')
    res_max = settings.get('planning.pedestrian_avoidance.resolution.max')
    resolution = res_min

    # if planner is too slow, resolution should be proportional to the vehicle-pedestrian distance
    # vehicle_poly = vehicle.to_object().polygon_parent() # in world frame
    # peds_poly = [a.polygon_parent() for a in all_pedestrians]
    # peds_dist = [collisions.polygon_polygon_distance_2d(vehicle_poly, p) for p in peds_poly]
    # d = min(peds_dist) # distance to closest pedestrian
    # if d <= braking_distance:
    #     resolution = res_min
    # elif d >= 2 * braking_distance: 
    #     resolution = res_max
    # else:
    #     resolution = res_min + (res_max - res_min) * (d / braking_distance - 1)

    return resolution

def add_buffer(dims):
    l, w, h = dims
    l += 2 * buffer_x
    w += 2 * buffer_y
    return [l, w, h]

    # to get corners of new polygon in the local frame
    # x = [-l/2 - buffer_x, l/2 + buffer_x]
    # y = [-w/2 - buffer_y, w/2 + buffer_y]
    # return [(x[i], y[j]) for i in range(2) for j in range(2)]

class PedestrianAvoidanceMotionPlanner(Component):
    """Follows the given route.  Brakes to yield to pedestrians or you are at the
    end of the route, otherwise accelerates to the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 1.0
        self.desired_speed = 2.0
        self.deceleration = 1.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return settings.get('planning.pedestrian_avoidance.rate')

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

        # extract out a segment of length = braking distance
        braking_distance = curr_v**2 / (2 * self.deceleration)
        lookahead = min(braking_distance, buffer_x)
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + lookahead)
        route_with_lookahead = route_with_lookahead.arc_length_parameterize()
        
        #TODO: use the collision detection primitives to determine whether to stop for a pedestrian

        collision_check_resolution = get_resolution(vehicle, all_pedestrians, braking_distance)
        progress_start, progress_end = route_with_lookahead.domain()
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
            
            # modify the margins aroun  d the vehicle to keep a safe distance from pedestrians
            vehicle_obj = vehicle.to_object()
            vehicle_obj.dimensions = add_buffer(vehicle_obj.dimensions)

            # this is a CCW polygon specifying the vehicle's position at the given progress, in the START frame
            vehicle_poly_world = vehicle_obj.polygon_parent() 
            
            if len(all_pedestrians) > 0:
                #TODO: figure out a good way to check for collisions with pedestrians with the desired margins
                if any([collisions.polygon_intersects_polygon_2d(vehicle_poly_world,a.polygon_parent()) for a in all_pedestrians]):
                    print("Predicted collision with pedestrian at",progress)
                    print("Vehicle position",xy[0],xy[1])
                    print("Pedestrian position",a.pose.x,a.pose.y)
                    print("Vehicle poly",vehicle_poly_world)
                    print("Pedestrian poly",a.polygon_parent())
                    #prevent
                    max_progress = max(progress - collision_check_resolution,0.0)
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

        #choose whether to accelerate, brake, or keep at current velocity
        if max_progress > 0:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        else:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        print("Stopped distance",traj.length())
        return traj 
