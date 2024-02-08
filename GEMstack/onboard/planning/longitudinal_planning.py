from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd
from scipy.optimize import fsolve
import math

DELTA_T = 0.05

# get's the position that we will stop given deceleration and speed
def get_p_stop(init_pos: float, deceleration : float, current_speed : float):
    if current_speed == 0:
        return init_pos
    if deceleration > 0 and current_speed > 0:
        return math.inf 
    p_stop = init_pos + current_speed**2 / (2*deceleration)
    return p_stop

def create_path(init_pos, init_time, init_vel, acceleration, stop_time):
    points = []
    times = []
    cur_time = init_time
    while (cur_time < stop_time):
        cur_time += DELTA_T
        time_diff = cur_time - init_time
        current_pos = init_pos + time_diff * init_vel + 0.5 * time_diff**2 * acceleration
        points.append(current_pos)
        times.append(cur_time)
    return points, times



    
def longitudinal_plan(path : Path, acceleration : float, deceleration : float, max_speed : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile. 
    
    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.

    The approach here involves solving for the equations in lecture. Basically there are two cases. 
    1. -> we can't reach the max speed while stopping in time -> in this case we will have two phases. the 
    acceleration phase and the deceleration phase. We have to solve for the position x_s, time t_s, speed v_s where we switch from acceleration to deceleration.
    2. -> we can reach the max speed while stopping in time -> in this case we will have three phases. the acceleration phase, the max speed phase and the deceleration phase. 
    We have to solve for the position x_s, time t_s, time T, speed v_s where we switch from acceleration to max speed and then from max speed to deceleration.
    We used sympy to solve for the equations in the first case and in the second case we did it manually.
    afterwards we just create the paths for each phase(acceleration, deceleration) using the variables we solved for and concatenate the paths together.
    """
    path_normalized = path.arc_length_parameterize()
    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]
    path_end = points[-1][0]
    p0 = points[0][0]
    y_coordinate = points[0][1]

    # we can use the formula for min time PP curve to solve this equation
    x1 = p0
    v1 = current_speed
    x2 = path_end
    v2 = 0 # since once we reach end position, we want to be at a complete stop
    a1 = acceleration
    a2 = -deceleration
    vL = max_speed
    delta_v = vL - v1
    ts1 = delta_v / a1 if a1 != 0 else 0
    xs1 = x1 + ts1*v1 + 0.5*(ts1**2)*a1
    # first we have to figure out if we can reach the max velocity (i.e) we can stop before xs at xs1 with speed vl
    p_stop = get_p_stop(xs1, deceleration, vL)
    if ts1==0 or p_stop <= x2:
        if ts1 == 0:
            vL = v1
        z = (v2 - vL) / a2
        ts2 = ((x2 - xs1 - 0.5*z**2*a2) / vL) + ts1 - z
        T = ts2 + z
        xs2 = xs1 + (ts2-ts1)*vL
        acc_points, acc_times = create_path(init_pos=x1, init_time=0, init_vel=v1,acceleration=a1, stop_time=ts1)
        max_speed_points, max_speed_times = create_path(init_pos=xs1,init_time=ts1, init_vel=vL, acceleration=0, stop_time=ts2)
        dec_points, dec_times = create_path(init_pos=xs2, init_time=ts2, init_vel=vL, acceleration=a2, stop_time=T)
        points = acc_points + max_speed_points + dec_points
        points = [(p, y_coordinate) for p in points]
        times = acc_times + max_speed_times + dec_times 
        # use first equation to create path since we can't reach the max velocity without stopping
    else:
        from sympy import symbols, Eq, solve
        # Define the symbols
        x_s, t_s, T, v_s = symbols('x_s t_s T v_s')
        # Given equations based on the image provided
        eq1 = Eq(v_s, v1 + a1 * t_s)
        eq2 = Eq(x_s, x1 + v1 * t_s + 0.5 * a1 * t_s**2)
        eq3 = Eq(v2, v_s + (T - t_s) * a2)
        eq4 = Eq(x2, x_s + (T - t_s) * v_s + 0.5 * (T - t_s)**2 * a2)
        # We'll solve for x_s, t_s, T, and v_s assuming that v1, x1, a1, a2 are constants.
        # Since we want to take the positive root for the quadratic, we'll need to check the solutions.

        # Solve the system of equations
        solutions = solve((eq1, eq2, eq3, eq4), (x_s, t_s, T, v_s), dict=True)
        valid_solutions = [sol for sol in solutions if sol[T] >= 0 and sol[t_s] >= 0]
        # if no valid solutions just brake because we can't stop in time
        if len(valid_solutions) == 0:
            return longitudinal_brake(path, deceleration, current_speed)
        
        T, ts, vs, xs = valid_solutions[0].values()

        acc_points, acc_times = create_path(init_pos=x1, init_time=0, init_vel=v1,acceleration=a1, stop_time=ts)
        dec_points, dec_times = create_path(init_pos=xs, init_time=ts, init_vel=vs, acceleration=a2, stop_time=T)
        points = acc_points + dec_points
        points = [(p, y_coordinate) for p in points]
        times = acc_times + dec_times

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
    while (time < t_stop and current_pos <= last_pos):
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
