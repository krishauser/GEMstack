from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd
import math
import numpy as np

def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times

    points = [x for (x,_) in path_normalized.points]
    input_speed = current_speed
    current_pos = 0
    current_time = 0
    positions = []
    times = []
    velocities = []
    if current_speed < max_speed and acceleration != 0:
        time_to_max_speed = (max_speed - current_speed) / acceleration
        times_to_max = np.arange(0, time_to_max_speed + 0.2, 0.2)
        positions_until_max_speed = current_speed * times_to_max + (1/2)*acceleration*(times_to_max **2)
        velocities_until_max = current_speed + acceleration*(times_to_max)
        positions.append(positions_until_max_speed)
        times.append(times_to_max)
        velocities.append(velocities_until_max)
        current_pos = positions_until_max_speed[-1]
        current_time = times_to_max[-1]
        print("Curr Pos:", current_pos)
        print("Curr Time:", current_time)

    current_speed = max_speed
    print('Curr V:', current_speed)
    if current_pos < points[-1]:
        s = points[-1] - current_pos
        print("Dist to End:", s)
        time_to_path_end = s / current_speed
        print(time_to_path_end)
        times_to_end = np.arange(0, time_to_path_end + 0.2, 0.2)
        print("Times to End:", times_to_end)
        positions_until_end = (current_speed * times_to_end) + current_pos
        times_to_end += current_time
        velocities_to_end = np.ones(len(positions_until_end)) * current_speed
        positions.append(positions_until_end)
        times.append(times_to_end)
        velocities.append(velocities_to_end)

    positions = np.concatenate(positions)
    available_distance = points[-1] - positions

    velocities = np.concatenate(velocities)
    braking_distance = (velocities**2)/(2*deceleration)

    times = np.concatenate(times)

    can_brake = available_distance > braking_distance
    velocities = velocities[can_brake]
    positions = positions[can_brake]
    times = times[can_brake]

    if len(velocities) != 0:
        current_speed = velocities[-1]
        current_time = times[-1]
        current_pos = positions[-1]
    else:
        current_speed = input_speed
        current_time = 0
        current_pos = 0

    time_to_stop = current_speed / deceleration
    times_to_stop = np.arange(0, time_to_stop + 0.2, 0.2)
    positions_to_stop = (current_speed * times_to_stop - (1/2)*deceleration*(times_to_stop**2)) + current_pos
    times_to_stop += current_time

    positions = np.concatenate([positions, positions_to_stop])
    times = np.concatenate([times, times_to_stop])

    positions = positions.tolist()
    positions = zip(positions, np.zeros(len(positions)))

    return Trajectory(path.frame, positions, times.tolist())


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times

    points = [p for p in path_normalized.points]
    times = [t/(path_normalized.times[-1]) for t in path_normalized.times]

    if path_normalized.times[-1] == 0:
        return Trajectory(path.frame,path_normalized.points,path_normalized.times)
    
    points = [p for p in path_normalized.points]
    times = [t/(path_normalized.times[-1]) for t in path_normalized.times]
    times = []
    # Compute the time needed to decelerate to stop
    decel_time = max(current_speed / deceleration, 1)
    if current_speed == 0:
        points = [(0, i) for i in np.arange(0.0, decel_time, 0.2)]
        times = np.arange(0, decel_time, 0.2).tolist()
    else:
        # change 0.2 to polling rate
        t = np.arange(0.0, decel_time + 0.2, 0.2)
        s = current_speed*t - 0.5*deceleration*t**2
        zeros = np.zeros(len(s))
        points = zip(s, zeros)
        times = t.tolist()

    


    # times = [t * decel_time for t in times] 
    return Trajectory(path.frame,points,times)


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = settings.get('control.longitudinal_planning_yielding.acceleration')
        self.desired_speed = settings.get('control.longitudinal_planning_yielding.desired_speed')
        self.deceleration = settings.get('control.longitudinal_planning_yielding.deceleration')

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
