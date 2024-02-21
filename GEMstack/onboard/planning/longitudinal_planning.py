from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, vector_add, vector2_angle
import math
import numpy as np
import matplotlib.pyplot as plt

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times

    points = [p for p in path_normalized.points]
    angle = math.cos(vector2_angle(points[-1], [1, 0]))
    print("angle !!!!!!!!", angle)

    points = [points[0]]
    times = [0.0]
    current_speed *= angle
    max_speed *= angle


    total_distance = path_normalized.times[-1]
    decel_dist_curr_speed = current_speed**2/(2*deceleration)
    decel_dist_max_speed = max_speed**2/(2*deceleration)
 
    dt = 0.05

    # if decel_dist_curr_speed > total_distance:
    #     # decelerate to 0
    #     decel_time = max(current_speed / deceleration, 1)
    #     t = np.arange(0, decel_time + dt, dt)
    #     s = current_speed*t - 0.5*deceleration*t**2
    #     last_point = points[-1]
    #     for i in range(len(t)):
    #         points.append(vector_add(last_point, [s[i], 0.0]))
    #     t = t.tolist()
    #     t = [x + times[-1] for x in t]
    #     times = times + t
    #     return Trajectory(path.frame, points, times)


    if acceleration == 0 and current_speed != 0:
        while total_distance-decel_dist_curr_speed > 0:
            dist_covered = current_speed*dt
            points.append(vector_madd(points[-1], [current_speed, 0], dt))
            times.append(times[-1]+dt)
            total_distance -= dist_covered
        
        decel_time = max(current_speed / deceleration, 1)
        t = np.arange(0, decel_time + dt, dt)
        s = current_speed*t - 0.5*deceleration*t**2
        last_point = points[-1]
        for i in range(len(t)):
            points.append(vector_add(last_point, [s[i], 0.0]))
        t = t.tolist()
        t = [x + times[-1] for x in t]
        times = times + t
        return Trajectory(path.frame, points, times)

    if current_speed > max_speed:
        #decelerate to max speed
        decel_time1 = (max_speed-current_speed)/deceleration
        t = np.arange(0, decel_time1 + dt, dt)
        s = current_speed*t - 0.5*deceleration*t**2
        for i in range(len(t)):
            points.append(vector_add(points[0], [s[i], 0.0]))
        t = t.tolist()
        t = [x + times[-1] for x in t]
        times = times + t

        decel_dist1 = (max_speed**2-current_speed**2)/(2*deceleration)
        while total_distance-decel_dist1-decel_dist_max_speed > 0:
            dist_covered = max_speed*dt
            points.append(vector_madd(points[-1], [max_speed, 0], dt))
            times.append(times[-1]+dt)
            total_distance -= dist_covered
        
        # decelerate to 0
        decel_time2 = max(max_speed / deceleration, 1)
        t = np.arange(0, decel_time2 + dt, dt)
        s = max_speed*t - 0.5*deceleration*t**2
        last_point = points[-1]
        for i in range(len(t)):
            points.append(vector_add(last_point, [s[i], 0.0]))
        t = t.tolist()
        t = [x + times[-1] for x in t]
        times = times + t
        return Trajectory(path.frame, points, times)
    

    decel_dist_max_speed = max_speed**2/(2*deceleration)
    accel_dist_curr_to_max = (max_speed**2 - current_speed**2) / (2 * acceleration)

    #if we'll reach max speed and decelerate
    if total_distance-decel_dist_max_speed > accel_dist_curr_to_max:
        # print("this was run")
        # accelerate to max speed
        accel_time = (max_speed-current_speed) / acceleration
        t = np.arange(0, accel_time + dt, dt)
        s = current_speed*t + 0.5*acceleration*t**2
        last_point = points[-1]
        for i in range(len(t)):
            points.append(vector_add(last_point, [s[i], 0.0]))
        t = t.tolist()
        t = [x + times[-1] for x in t]
        times = times + t

        # stay at max speed
        const_speed_dist = total_distance-decel_dist_max_speed-accel_dist_curr_to_max
        while const_speed_dist > 0:
            # print("this was run")
            dist_covered = max_speed*dt
            points.append(vector_madd(points[-1], [max_speed, 0], dt))
            times.append(times[-1]+dt)
            const_speed_dist -= dist_covered

        # decelerate to 0
        decel_time2 = max(max_speed / deceleration, 1)
        t = np.arange(0, decel_time2 + dt, dt)
        s = max_speed*t - 0.5*deceleration*t**2
        last_point = points[-1]
        for i in range(len(t)):
            points.append(vector_add(last_point, [s[i], 0.0]))
        t = t.tolist()
        t = [x + times[-1] for x in t]
        times = times + t
        print(len(points), len(times))
        # plt.plot(times,[p[0] for p in points])
        # # plt.show()
        return Trajectory(path.frame, points, times)
    
    #Accelerate then decelerate without reaching max speed
    while total_distance-decel_dist_curr_speed > 0:
        #accelerate
        current_speed = current_speed + acceleration*dt
        times.append(times[-1]+dt)
        s = current_speed*dt + 0.5*acceleration*dt**2
        points.append(vector_add(points[-1], [s, 0.0]))
        total_distance -= s
        decel_dist_curr_speed = current_speed**2/(2*deceleration)
    
    decel_time = max(current_speed / deceleration, 1)
    t = np.arange(0, decel_time + dt, dt)
    s = current_speed*t - 0.5*deceleration*t**2
    last_point = points[-1]
    for i in range(len(t)):
        points.append(vector_add(last_point, [s[i], 0.0]))
    t = t.tolist()
    t = [x + times[-1] for x in t]
    times = times + t
    return Trajectory(path.frame, points, times)




def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times

    if path_normalized.times[-1] == 0:
        return Trajectory(path.frame,path_normalized.points,path_normalized.times)
    
    # Compute the time needed to decelerate to stop
    decel_time = max(current_speed / deceleration, 1)
    
    if current_speed == 0:
        points = [(0, i) for i in np.arange(0.0, decel_time, 0.2)]
        times = np.arange(0, decel_time, 0.2).tolist()
    else:
        # change 0.2 to polling rate
        t = np.arange(0.0, decel_time + 0.2, 0.2)
        s = current_speed*t - 0.5*deceleration*t**2
        points = []
        for i in range(len(t)):
            points.append([s[i], 0.0])
        times = t.tolist()
    return Trajectory(path.frame,points,times)


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5 #settings.get('control.longitudinal_planning_yielding.acceleration')
        self.desired_speed = 2.0  #settings.get('control.longitudinal_planning_yielding.desired_speed')
        self.deceleration = 1.0 #settings.get('control.longitudinal_planning_yielding.deceleration')

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state : AllState):
        vehicle = state.vehicle # type: VehicleState
        route = state.route   # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
  
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        #figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        self.route_progress = closest_parameter

        #extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)

        #parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj 
