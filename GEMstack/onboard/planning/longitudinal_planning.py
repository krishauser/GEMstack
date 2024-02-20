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
    position = 0.0
    velocity = current_speed
    end_position = path_normalized.points[-1][0]
    
    dt = 0.05
    time = 0

    times = []
    positions = []

    while position<end_position - velocity**2/(2*deceleration):
        # if velocity<0.4:
        #     time += dt
        #     position += velocity*dt
        #     times.append(time)
        #     positions.append(path_normalized.eval(position))
        #     velocity += acceleration*dt*0.2
        #     continue
        if velocity>max_speed+acceleration*dt:
            velocity -= deceleration*dt
        elif velocity<max_speed:
            velocity += acceleration*dt
        if velocity==0:
            break
        time += dt
        position += velocity*dt
        times.append(time)
        positions.append(path_normalized.eval(position))
    while velocity >= 0:
        velocity -= deceleration*dt
        time += dt
        position += velocity*dt
    # print(path_normalized.points)
    # print(path_normalized.times)
    # return Trajectory(path.frame,path_normalized.points,path_normalized.times)
    print(f"times : {times[:10]}")
    print(f"positions :{positions[:10]}")
    return Trajectory(path.frame,positions,times)



def longitudinal_brake(path : Path, deceleration : float, current_speed : float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    position = 0.0
    t = 0.0
    dt = 0.05  # More precise time step for smoother deceleration
    times = []
    points = []

    current_speed = max(current_speed, 0)  # Ensure speed is not negative

    while current_speed > 0:
        times.append(t)
        points.append(path_normalized.eval(position))
        next_speed = max(current_speed - deceleration * dt, 0)  # Decelerate and clamp to 0 as minimum
        position += (current_speed + next_speed) / 2 * dt  # Use average speed over dt for position update
        current_speed = next_speed
        t += dt

        if current_speed == 0:
            break  # Ensure loop exits when speed is 0, preventing any further position updates

    times.append(t)  # Record the final time and position
    points.append(path_normalized.eval(position))
    print(f"points{points}")
    print(f"times{times}")
    print("--------------")
    return Trajectory(path.frame, points, times)

    # path_normalized = path.arc_length_parameterize()
    # ##TODO: actually do something to points and times
    # position = 0.0
    
    
    # t = 0.0
    # dt = 0.05
    # times = []
    # points = []

    # end_position = path_normalized.times[-1]
    # while position < end_position:
    #     times.append(t)
    #     points.append(path_normalized.eval(position))
    #     current_speed -= deceleration*dt
    #     if current_speed<0:
    #         current_speed = 0
    #     if current_speed ==0:
    #         break
    #     position += current_speed * dt
    #     t += dt
        

    # while current_speed>0 :

    #     current_speed = max(current_speed - deceleration*dt,0)
        
    #     position += current_speed*dt
    #     t += dt
    #     if current_speed ==0:
    #         break
        
    #     times.append(t)
    #     points.append(path_normalized.eval(position))


    # print(f"points{points}")
    # print(f"times{times}")
    # print("--------------")
    # return Trajectory(path.frame,points,times)

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

        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist,closest_parameter = state.route.closest_point_local((curr_x,curr_y),[self.route_progress-5.0,self.route_progress+5.0])
        route_with_lookahead = route.trim(closest_parameter,closest_parameter+10.0)
        self.route_progress = closest_parameter
        
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                #yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)
        print(f"should_accelerate: {should_accelerate}")
        print(f"should_brake: {should_brake}")
        print(f"curr_v: {curr_v}")
        if should_accelerate:
            traj = longitudinal_plan(route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj