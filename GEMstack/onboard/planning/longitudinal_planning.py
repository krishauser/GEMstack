from typing import List
from ..component import Component
from ...state import (
    AllState,
    VehicleState,
    EntityRelation,
    EntityRelationEnum,
    Path,
    Trajectory,
    Route,
    ObjectFrameEnum,
)
from ...utils import serialization
from ...mathutils.transforms import vector_madd


import numpy as np


def longitudinal_plan(
    path: Path,
    acceleration: float,
    deceleration: float,
    max_speed: float,
    current_speed: float,
) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times

    dt = 0.05
    positions = []
    positions.append((path.points[0][0], 0))
    times = [0.0]
    target_position = path.points[-1][0]
    flag = 0
    while True:
        if flag == 0:
            brake_distance = current_speed**2 / (2 * deceleration)
            if target_position - positions[-1][0] < brake_distance + 0.5:
                # Start to brake
                flag = 1
            else:
                # Accelerate
                if current_speed < max_speed:
                    times.append(times[-1] + dt)
                    positions.append(
                        (
                            positions[-1][0]
                            + current_speed * dt
                            + 0.5 * acceleration * dt**2,
                            0
                        )
                    )
                    current_speed += acceleration * dt
                else:
                    current_speed = max_speed
                    times.append(times[-1] + dt)
                    positions.append((positions[-1][0] + current_speed * dt, 0))
        elif flag == 1:
            if current_speed > 0:
                times.append(times[-1] + dt)
                positions.append(
                    (positions[-1][0] + current_speed * dt - 0.5 * deceleration * dt**2, 0)
                )
                current_speed -= deceleration * dt
            else:
                break

    trajectory = Trajectory(path.frame, positions, list(times))

    return trajectory


def longitudinal_brake(
    path: Path, deceleration: float, current_speed: float
) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    init_pos = path.points[0][0]
    init_vel = current_speed
    t_stop = init_vel / deceleration
    dt = 0.05

    times = np.arange(0, t_stop + dt, dt)

    positions = init_pos + init_vel * times - 0.5 * deceleration * times**2
    braking_points = [(pos, 0) for pos in positions]
    trajectory = Trajectory(path.frame, braking_points, list(times))
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
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["trajectory"]

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress - 5.0, self.route_progress + 5.0]
        )
        self.route_progress = closest_parameter

        # extract out a 10m segment of the route
        route_with_lookahead = route.trim(closest_parameter, closest_parameter + 10.0)

        # parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELD and r.obj1 == "":
                # yielding to something, brake
                should_brake = True
        should_accelerate = not should_brake and curr_v < self.desired_speed

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(
                route_with_lookahead,
                self.acceleration,
                self.deceleration,
                self.desired_speed,
                curr_v,
            )
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(
                route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v
            )

        return traj
