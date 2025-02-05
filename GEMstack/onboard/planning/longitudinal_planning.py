from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd

import time

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

    #=============================================

    print("-----LONGITUDINAL PLAN-----")
    print("path length: ", path.length())
    length = path.length()

    # Starting point
    x0 = points[0][0]
    # Time to stop
    t_stop = current_speed / deceleration
    # Position to stop
    x_stop = x0 + current_speed * t_stop - 0.5 * deceleration * t_stop**2

    # GEM will decelerate to 0 before the end of the path
    if length < 10.0 and x_stop > points[-1][0]:
        new_points = []
        for t in times:
            if t <= t_stop:
                x = x0 + current_speed * t - 0.5 * deceleration * t**2
            # Keep the position after reaching 0 velocity
            else:
                x = x_stop
            new_points.append([x, 0])
        points = new_points
        print("[BRAKE] Computed points:", points)

    # GEM will accelerate to max speed before the end of the path
    else:
        new_points = []
        for t in times:
            # Accelerate to max speed
            if max_speed > current_speed:
                x = x0 + current_speed * t + 0.5 * acceleration * t**2
            # Keep the velocity after reaching max speed
            else:
                x = x0 + current_speed * t
            new_points.append([x, 0])
        points = new_points
        print("[ACCEL] Computed points:", points)

    #=============================================

    trajectory = Trajectory(path.frame,points,times)
    return trajectory


def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()

    #TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    #=============================================

    print("=====LONGITUDINAL BRAKE=====")
    print("path length: ", path.length())
    length = path.length()

    x0 = points[0][0]
    t_stop = current_speed / deceleration
    x_stop = x0 + current_speed * t_stop - 0.5 * deceleration * t_stop**2

    new_points = []
    for t in times:
        if t <= t_stop:
            x = x0 + current_speed * t - 0.5 * deceleration * t**2
        else:
            x = x_stop
        new_points.append([x, 0])
    points = new_points
    print("[BRAKE] Computed points:", points)

    #=============================================

    trajectory = Trajectory(path.frame,points,times)
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
        start_time = time.time()

        vehicle = state.vehicle # type: VehicleState
        route = state.route   # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
  
        # Position in vehicle frame (Start (0,0) to (15,0))
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

                #=========================
                """
                Collision detection:
                    - Compute the lookahead distance required to avoid collision using:
                        d = v^2/(2*a)
                    - For many steps along the route (using a resolution that adapts if the
                    planner runs too slowly), simulate the vehicle's future positions.
                    - If a pedestrian is detected within 3m longitudinal and 1m lateral buffer,
                    determine the distance-to-collision. Then compute the required deceleration:
                        a = -(v^2)/(2*d_collision)
                    - For distant crossing pedestrians, apply a gentle deceleration based on the
                    perception-estimated pedestrian velocity.
                """

                print("#### YIELDING PLANNING ####")

                # Convert vehicle pose from vehicle to world coordinates
                # xyhead_demo: vehicle start at [4, 5], ped walks [15, 2]~[15, 10]
                # Is there relative pose of pedestrian or absolute pose of vehicle?
                abs_x = vehicle.pose.x + state.start_vehicle_pose.x
                abs_y = vehicle.pose.y + state.start_vehicle_pose.y

                # TODO: Check if there are multiple pedestrians
                for n,a in state.agents.items():
                    print("ped", a.pose.x, a.pose.y)
                    print("ego", abs_x, abs_y)

                    # TODO: Make logic for smooth deceleration and re-acceleration
                    # TEMPORARY: STOP WHEN WITHIN 10M OF PEDESTRIAN
                    if a.pose.x - abs_x < 10.0 and a.pose.x - abs_x > 0.0:
                        print("#### Yielding to",n)
                        should_brake = True

                    break

                # # UNCOMMENT TO BRAKE FOR ALL PEDESTRIANS
                # should_brake = True

                #=========================

        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        #choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj 
