from typing import List
from ..component import Component
#these could be helpful in your implementation
from .longitudinal_planning import longitudinal_brake, longitudinal_plan
from ...state import AllState, VehicleState, PhysicalObject, AgentEnum, AgentState, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
from ...mathutils import collisions
import copy
import math
import numpy as np
from typing import Dict


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
        return 10.0 #Hz

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
        curr_long = 0 #longitudnal dist to pedestrian 
        curr_lat = 0 #lateral ditance to the pedestrian 

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
        all_pedestrians = []
        for k,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:
                all_pedestrians.append(a)

        collision_check_resolution = 0.1  #m
        progress_start, progress_end = route_with_lookahead.domain()
        progress = progress_start
        max_progress = None
        
        #find the closest pedestrian and the distance to that pedestrian
        nearest_agent = None
        nearest_dist = None 
        for a in all_pedestrians:
            distance = (a.pose.x-curr_x)**2 + (a.pose.y-curr_y)**2 #euclidean distance 
            if nearest_agent is None:
                nearest_agent = a
                nearest_dist = distance
            elif distance < nearest_dist:
                nearest_agent = a
            
        #vehicle margins 
        margin_lat = 1.0
        margin_long = 3.0

        #lookahead distance d=v^2/2a
        l_distance = (self.desired_speed ** 2)/(2 * self.deceleration)

        if len(all_pedestrians) > 0:
            curr_long = nearest_agent.pose.x - curr_x
            curr_lat = nearest_agent.pose.y - curr_y 
        
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
            if len(all_pedestrians) > 0:
                #TODO: figure out a good way to check for collisions with pedestrians with the desired margins
                #Calculate distance between  vehicle and pedestrian
                # closest pedestrian's position (x,y) and dimensions (w,h)
                pedestrian_x = nearest_agent.pose.x
                pedestrian_y = nearest_agent.pose.y
                pedestrian_w = nearest_agent.dimensions[0]
                pedestrian_h = nearest_agent.dimensions[1]

                #vehicle's position and dimension
                vehicle_x = curr_x
                vehicle_y = curr_y
                vehicle_w = margin_lat
                vehicle_h = margin_long

                #pedestrian boundaries 
                p_left = pedestrian_y - pedestrian_w
                p_right = pedestrian_y + pedestrian_w
                p_top = pedestrian_x + pedestrian_h
                p_bottom = pedestrian_x - pedestrian_h

                #vehicle boundaries
                v_left = vehicle_y - (vehicle_w/2)
                v_right = vehicle_y + (vehicle_w/2)
                v_top = vehicle_x + (vehicle_h/2)
                v_bottom = vehicle_x - (vehicle_h/2)

                #if boundaries overlap then collision warning 
                if p_left > v_right or p_right < v_left or p_bottom < v_top or p_top > v_bottom:
                    print("Collision Warning!")

                if any([collisions.polygon_intersects_polygon_2d(vehicle_poly_world,a.polygon_parent()) for a in all_pedestrians]):
                    print("Predicted collision with pedestrian at",progress)
                    print("Vehicle position",xy[0],xy[1])
                    print("Pedestrian position",a.pose.x,a.pose.y)
                    #print("Vehicle poly",vehicle_poly_world)
                    #print("Pedestrian poly",a.polygon_parent())
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
        # if max_progress > 0 and curr_long>l_distance and curr_lat>margin_lat:
        #     traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        # else:
        #     traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        # print("Stopped distance",traj.length())
         if (curr_long <= l_distance + margin_long and curr_lat <= margin_lat) or max_progress <= 0:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        print("Stopped distance",traj.length())

        return traj 
