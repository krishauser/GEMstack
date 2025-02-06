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

    # This assumes that the time denomination cannot be changed

    # Starting point
    x0 = points[0][0]
    cur_point = points[0]
    cur_time = times[0]
    cur_index = 0

    new_points = []
    new_times = []
    while current_speed > 0 or cur_index == 0:
        # we want to iterate through all the points and add them
        # to the new points. However, we also want to add "critical points"
        # where we reach top speed, begin decelerating, and stop
        new_points.append(cur_point)
        new_times.append(cur_time)
        print("=====================================")
        print("new points: ", new_points)
        print("current index: ", cur_index)
        print("current speed: ", current_speed)

        # Calculate how much time it would take to stop
        min_delta_t_stop = current_speed/deceleration
        min_delta_x_stop = current_speed*min_delta_t_stop - 0.5*deceleration*min_delta_t_stop**2
        assert min_delta_x_stop >= 0

        # Information we will need 
        #assert cur_index < len(points)-1

        # Check if we are done

        # If we cannot stop before or stop exactly at the final position requested
        if cur_point[0] + min_delta_x_stop >= points[-1][0]:
            print("In case one")
            # put on the breaks

            # Calculate the next point in a special manner because of too-little time to stop 
            if cur_index == len(points)-1:
                # the next point in this instance would be when we stop
                next_point = (cur_point[0] + min_delta_x_stop, 0)
            else:
                next_point = points[cur_index+1]

            # do we just progress and update time
            if next_point[0] <= points[-1][0]:
                print("continuing to next point")
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_time += delta_t_to_next_x
                cur_point = next_point
                current_speed -= deceleration*delta_t_to_next_x
                cur_index += 1
            else:
                # continue to the point in which we would stop
                # update to the next point 
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_point = next_point
                cur_time += delta_t_to_next_x
                #current_speed -= delta_t_to_next_x*deceleration
                current_speed = 0
                assert current_speed == 0

        elif current_speed < max_speed:
            print("In case two")
            next_point = points[cur_index+1]
            # accelerate to max speed
            delta_t_to_max_speed = (max_speed - current_speed)/acceleration
            delta_x_to_max_speed = current_speed*delta_t_to_max_speed + 0.5*acceleration*delta_t_to_max_speed**2
            
            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                print("go to next point")
                # accelerate to max speed
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, acceleration)
                cur_time += delta_t_to_next_x
                cur_point = [next_point[0], 0]
                current_speed += delta_t_to_next_x*deceleration
                cur_index += 1
            
            else:
                print("adding new point")
                # we would need to add a new point at max speed
                cur_time += delta_t_to_max_speed
                cur_point = [cur_point[0] + delta_x_to_max_speed, 0]
                current_speed = max_speed

        elif current_speed == max_speed:
            next_point = points[cur_index+1]
            # continue on with max speed
            print("In case three")
            
            # add point to start decelerating
            if next_point[0] + min_delta_x_stop >= points[-1][0]:
                print("Adding new point to start decelerating")
                cur_time += (points[-1][0] - min_delta_x_stop - cur_point[0])/current_speed
                cur_point = [points[-1][0] - min_delta_x_stop,0]
                current_speed = max_speed
            else:
                print("Continuing on to next point")
                cur_time += (next_point[0] - cur_point[0])/current_speed
                cur_point = next_point
                cur_index += 1
            
        elif current_speed > max_speed:
            next_point = points[cur_index+1]
            print("In case four")
            # slow down to max speed 
            delta_t_to_max_speed = (current_speed - max_speed)/deceleration
            delta_x_to_max_speed = current_speed*delta_t_to_max_speed - 0.5*deceleration*delta_t_to_max_speed**2

            if cur_point[0] + delta_x_to_max_speed >= next_point[0]:
                # next position will take care of this 
                delta_t_to_next_x = compute_time_to_x(cur_point[0], next_point[0], current_speed, -deceleration)
                cur_time += delta_t_to_next_x
                cur_point = next_point[0]
                current_speed -= delta_t_to_next_x*deceleration
                cur_index += 1
            else:
                cur_time += delta_t_to_max_speed
                cur_point = [cur_point[0] + delta_x_to_max_speed, 0]
                current_speed = max_speed

        else:
            # not sure what falls here
            raise ValueError("Not sure what to do here")
        
    new_points.append(cur_point)
    new_times.append(cur_time)
        
    points = new_points
    times = new_times
    print("[PLAN] Computed points:", points)
    print("[TIME] Computed time:", times)
    #=============================================

    trajectory = Trajectory(path.frame,points,times)
    return trajectory

def compute_time_to_x(x0 : float, x1 : float, v : float, a : float) -> float:
    """Computes the time to go from x0 to x1 with initial velocity v0 and final velocity v1
    with constant acceleration a."""
    # x1 = x0 + v0*t + 0.5*a*t^2
    # x1 - x0 = v0*t + 0.5*a*t^2
    # 0.5*a*t^2 + v0*t + x0 - x1 = 0
    # t = (-v0 + sqrt(v0^2 - 4*0.5*a*(x0-x1)))/(2*0.5*a)
    # t = (-v0 + sqrt(v0^2 + 2*a*(x1-x0)))/a
    print("x0: ", x0)
    print("x1: ", x1)
    print("v: ", v)
    print("a: ", a)

    t1 = (-v + (v**2 - 2*a*(x0-x1))**0.5)/a
    t2 = (-v - (v**2 - 2*a*(x0-x1))**0.5)/a
    print("t1: ", t1)
    print("t2: ", t2)

    return min(n for n in [t1, t2]  if n>0)

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
