from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd

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
    # print("points", points)
    # print("times", times)

    dt = 0.05
    
    ### Debug: 02/06/2024
    init_point = points[0]
    point = init_point
    end_point = points[-1]  
    
    init_speed = current_speed
    time = 0
    out_times = [0]
    out_points = [init_point]

    # accelerate to max speed
    while current_speed < max_speed:
        current_speed = init_speed + acceleration * time
        point_x = init_point[0] + time * init_speed + 0.5 * acceleration * time**2
        point_y = 0 # not used for now
        point = (point_x, point_y)

        # if point[0] >= end_point[0]:
        #     # decelerate to stop
        #     dec_init_speed = current_speed
        #     dec_init_pos = point
        #     dec_time = 0
        #     t_stop = dec_init_speed / deceleration
        #     p_step = dec_init_pos + dec_init_speed**2 / (2*deceleration)
        #     while current_speed > 0:
        #         current_speed = dec_init_speed - deceleration * dec_time
        #         point_x = dec_init_pos[0] + dec_time*dec_init_speed + 1/2 * dec_time**2 * deceleration
        #         point_y = 0
        #         point = (point_x, point_y)
        #         dec_time += dt
        #         time += dt
        #         out_times.append(time)
        #         out_points.append(point)
        #     break
        # else:
        time += dt
        out_times.append(time)
        out_points.append(point)

    # determine which point to brake
    stop_point = end_point[0] - max_speed**2 / (2*deceleration)
    # travel along max speed
    point_x = point[0]
    while point[0] < stop_point:
        point_x += max_speed * dt
        point_y = 0
        point = (point_x, point_y)
        time += dt
        out_times.append(time)
        out_points.append(point)
    
    # decelerate to stop
    dec_init_speed = max_speed
    dec_init_pos = point
    dec_time = 0
    while current_speed > 0:
        current_speed = dec_init_speed - deceleration * dec_time
        point_x = dec_init_pos[0] + dec_time*dec_init_speed + 1/2 * dec_time**2 * deceleration
        point_y = 0
        point = (point_x, point_y)
        dec_time += dt
        time += dt
        out_times.append(time)
        out_points.append(point)
    
    trajectory = Trajectory(path.frame, out_points, out_times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
    dt = 0.05 # manully set
   
    init_speed = current_speed
    init_point = points[0]
    t_stop = init_speed / deceleration
    p_stop = points[0][0] + init_speed**2 / (2*deceleration)
    print("t_stop", t_stop) 
    print("p_stop", p_stop)

    # create new time list
    times = [i*dt for i in range(int(t_stop/dt))]
    points = []
    for t in times:
        current_speed = init_speed - t * deceleration
        # point = (init_point[0] + t*init_speed - 1/2 * t**2 * deceleration, init_point[1])
        point_x = init_point[0] + t*init_speed - 1/2 * t**2 * deceleration
        point_y = 0 # not used for now
        point = (point_x, point_y)
        points.append(point)
    
    # print("points", points)
    # print('times', times)
    
    trajectory = Trajectory(path.frame, points, times)
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
            if r.type == EntityRelationEnum.YIELD and r.obj1 == '':
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
