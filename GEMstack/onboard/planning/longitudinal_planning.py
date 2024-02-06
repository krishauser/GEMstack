from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
import math

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
    times = [t for t in path_normalized.times]

    # Function to calculate distance between two points
    def distance(p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    path_length = sum(distance(points[i], points[i+1]) for i in range(len(points)-1))
    total_cover_d = 0
    total_t = 0

    d_d_c = (current_speed ** 2)/(2 * (deceleration))

    # if can't brake before hitting the end of the path or
    if d_d_c > path_length:

        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)

            # find the time need to reach the end
            segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
            current_speed = max(current_speed - deceleration * segment_time, 0)


            total_t += segment_time
            times[i+1] = total_t
        # keep deceleration
        curr_location = points[-1]
        while current_speed != 0:

            # find the time need to reach the end
            segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
            current_speed = max(current_speed - deceleration * segment_time, 0)

            total_t += segment_time
            times.append(total_t)
            curr_location =  (curr_location[0]+segment_length, curr_location[1])
            points.append(curr_location)

        # Create the trajectory object with the calculated points and times
        trajectory = Trajectory(frame=path.frame, points=points, times=times)

    elif acceleration == 0.0:
        d_d = (max_speed ** 2)/(2 * (deceleration))
        d_cruise = path_length - d_d
        t_cruise = d_cruise / current_speed

        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)

            # cruising
            # print('(total_cover_d + segment_length) <= d_cruise:', (total_cover_d + segment_length) <= d_cruise)
            # print('d_cruise', d_cruise)
            if (total_cover_d + segment_length) <= d_cruise:
                segment_time = segment_length/current_speed
            else:
                # find the time need to reach the end
                segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
                current_speed = max(current_speed - deceleration * segment_time, 0)

            total_t += segment_time
            times[i+1] = total_t
            total_cover_d += segment_length

        # Create the trajectory object with the calculated points and times
        trajectory = Trajectory(frame=path.frame, points=points, times=times)

    else:
        #calculate the time for acceleration
        t_a = (max_speed - current_speed) / acceleration
        #calculate the distance for acceleration
        d_a = current_speed * t_a + 0.5 * acceleration * (t_a ** 2)
        #calculate the distance for deceleration
        d_d = (max_speed ** 2)/(2 * (deceleration))

        d_total = d_a + d_d #total distance
        # path_length = path_normalized.calculate_path_length()


        #determine if there is enough distance to reach max_speed and then decelerate to 0:
        if d_total > path_length:
            # acceleration and deceleration may not be same
            peak_speed = ((2 * acceleration * deceleration * path_length + deceleration * current_speed ** 2) ** 0.5 + current_speed ** 2) / (acceleration + deceleration)
            # print('peak_speed', peak_speed)
            t_a = (peak_speed - current_speed) / acceleration
            d_a = (current_speed + peak_speed) / 2 * t_a
            d_d = (peak_speed ** 2)/(2 * (deceleration))
            d_total = d_a + d_d
        else:
            peak_speed = max_speed

        #calculate the distance at the max_speed
        d_at_max = path_length - d_d
        # #calculate the time at the max_speed
        # t_at_max = d_at_max / peak_speed

        # #calculate the time to decelerate to 0
        # t_d = peak_speed / -deceleration

        #interpolate points and construct time list:
        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)
            segment_time = 0
            # print('accelerate', (total_cover_d+segment_length) <= d_a.real)
            # print('cruise', (total_cover_d +segment_length) <= d_at_max.real)
            # print('d_a.real + d_at_max.real', d_at_max.real)
            # print('(total_cover_d +segment_length)',(total_cover_d +segment_length))
            #Determine the state (accelerating, cruising, decelerating) and calculate segment time:
            if (total_cover_d+segment_length) <= d_a.real:
                #Accelerating
                segment_time = (math.sqrt(current_speed**2 + 2 * acceleration * segment_length) - current_speed) / acceleration
                current_speed += acceleration * segment_time

            elif (total_cover_d +segment_length) <= d_at_max.real:
                #Cruising
                segment_time = segment_length / peak_speed
            else:
                segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
                current_speed = max(current_speed - deceleration * segment_time, 0)

            total_t += segment_time
            times[i+1] = total_t
            total_cover_d += segment_length
            # print("current_speed",current_speed)
            # print("times", times[i])

        trajectory = Trajectory(path.frame,points,times)
    # print(current_speed)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    def distance(p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    path_length = sum(distance(path.points[i], path.points[i+1]) for i in range(len(path.points) - 1))

    if current_speed <= 0:
        return Trajectory(path, [points[0]] * len(points),times)

    #calculate time to stop
    t_s = current_speed / deceleration
    #calculate distance for deceleration
    d_s = (current_speed ** 2) / (2 * deceleration)

    #check if braking distance is longer than the path length:
    if d_s > path_length:
        # deceleration = -(current_speed ** 2) / (2 * path_length)
        # t_s = current_speed / -deceleration
        total_t = 0
        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)

            # find the time need to reach the end
            segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
            current_speed = max(current_speed - deceleration * segment_time, 0)

            total_t += segment_time
            if len(times)-1 < i+1:
                times.append(total_t)
            else:
                times[i+1] = total_t

        # keep deceleration
        curr_location = points[-1]

        while current_speed != 0:

            # find the time need to reach the end
            segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
            current_speed = max(current_speed - deceleration * segment_time, 0)

            total_t += segment_time
            times.append(total_t)
            curr_location =  (curr_location[0]+segment_length, curr_location[1])
            points.append(curr_location)

        # Create the trajectory object with the calculated points and times
        trajectory = Trajectory(frame=path.frame, points=points, times=times)
    else:

        current_position = 0
        current_time = 0

        #interpolate points and construct time list:
        for i in range(len(points) - 1):
            segment_start = points[i]
            segment_end = points[i + 1]
            segment_length = distance(segment_start, segment_end)

            if current_speed == 0:
                current_time += 1
                times[i+1] = current_time
                points[i+1] = points[i]
                continue

            # calculate time:
            # if current_position + segment_length > d_s:
            #     d_remain = d_s - current_position + segment_length
            #     t_remain = math.sqrt(abs(2 * d_remain / -deceleration))
            #     segment_time = t_remain
            #     current_speed = max(current_speed - deceleration * times[i],0)
            # else:
            #     segment_time = 2 * segment_length / (current_speed + current_speed + deceleration * current_time)
            #     current_speed = max(current_speed - deceleration * times[i],0)

            segment_time = (current_speed - math.sqrt(current_speed**2 - 2 * -deceleration * segment_length)) / -deceleration
            current_speed = max(current_speed - deceleration * segment_time, 0)

            current_position += segment_length
            current_time += segment_time
            times[i+1] = current_time




        # for i in range(1, len(times)):

        #     times[i] = max(times[i], times[i-1])
        # points[-1] = points[-2]

        trajectory = Trajectory(path.frame,points,times)
    print('current_speed',current_speed)
    return trajectory


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """
    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5
        self.desired_speed = 1.0
        self.deceleration = 2.0

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
