from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization, settings
from ...mathutils.transforms import vector_madd, normalize_vector, vector_sub

DELTA_T = settings.get('longitudinal_planning.dt')
    
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

    ### Simply travel along the path ### 
    direction = normalize_vector(vector_sub(points[-1], points[0])) # heading of the vehicle
    p_path = times[-1] * 1 # total path length
    point_start = points[0]
    p = 0
    t = 0
    v = current_speed

    points = []
    times = []

    dt = DELTA_T
    while p <= p_path or v > 0:
        points.append(p)
        times.append(t)

        # What should the acceleration be, accelerate or decelerate
        if 0.5 * v**2 / deceleration + p > p_path:
            a = -deceleration
        else:
            a = acceleration

        # update position, velocity, time
        v = min(v + a * dt, max_speed)
        p = p + v * dt #+ 0.5 * a * dt**2
        if v < 0:
            break
        t = t + dt

    points = [vector_madd(point_start, direction, p) for p in points] # distance here, need to change to 2D points
    trajectory = Trajectory(path.frame,points,times)
    return trajectory

def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    #TODO: actually do something to points and times
    points = [p for p in path.points]
    # times = [t for t in path_normalized.times]
    times = []
    pointsr = []

    last_pos = points[-1][0]
    init_pos = points[0][0]
    current_pos = points[0][0]
    init_vel = current_speed
    t_stop = init_vel / deceleration
    p_stop = points[0][0] + init_vel**2 / (2*deceleration)

    time = 0
    times.append(time)
    pointsr.append(points[0])
    while time < t_stop:
        time += DELTA_T
        current_pos = init_pos + time * init_vel - 0.5 * time**2 * deceleration
        pointsr.append((current_pos,0))
        times.append(time)

    if p_stop < last_pos:
        time += DELTA_T
        times.append(time)
        pointsr.append((current_pos,0))

    trajectory = Trajectory(path.frame, pointsr, times)
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
