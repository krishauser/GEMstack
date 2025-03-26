from ...state import AllState,AgentEnum,EntityRelation,EntityRelationEnum,ObjectFrameEnum
from ..component import Component
from typing import List
from ...utils import settings

VEHICLE_SIDE = 1.0

class ACCController(Component):
    """Apply adaptive cruise control algorithm to find the optimal acceleration for the vehicle.
    
    Result is stored in the relations graph.
    """
    def __init__(self, mode : str = 'real', params : dict = {}):
        # Planner mode
        self.mode = mode
        self.planner = params["planner"]
        self.acceleration = params["acceleration"]
        self.desired_speed = params["desired_speed"]
        self.safe_distance = params["safe_distance"]

        # Factors to decide how responsive the controller is to the gap and speed
        # TODO: Fine tune the gap and speed factor
        self.gap_factor = 0.2
        self.speed_factor = 0.5

        # Update current.yaml settings with the given parameters in pedestrian_detection.yaml
        settings.set("planning.longitudinal_plan.mode", self.mode)
        settings.set("planning.longitudinal_plan.planner", self.planner)
        settings.set("planning.longitudinal_plan.acceleration", self.acceleration)
        settings.set("planning.longitudinal_plan.desired_speed", self.desired_speed)

        self.max_deceleration   = settings.get("planning.longitudinal_plan.max_deceleration")
        self.deceleration       = settings.get("planning.longitudinal_plan.deceleration")

    def rate(self):
        return None
    def state_inputs(self):
        return ['all']
    def state_outputs(self):
        return ['relations']
    def update(self, state : AllState) -> List[EntityRelation]:
        res = []
        vehicle = state.vehicle
        agents = state.agents

        # Position in vehicle frame (Start (0,0) to (15,0))
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v
 
        for n,a in agents.items():
            if a.type == AgentEnum.PEDESTRIAN:
                output_decel, output_dist, output_speed = None, None, None

                # Get the pedestrian's position and velocity
                a_x, a_y = a.pose.x, a.pose.y
                a_v_x, a_v_y = a.velocity[0], a.velocity[1]  # Pedestrian speed vector

                # If the pedestrian's frame is ABSOLUTE, convert the vehicle's frame to ABSOLUTE.
                if a.pose.frame == ObjectFrameEnum.ABSOLUTE_CARTESIAN:
                    curr_x = curr_x + state.start_vehicle_pose.x
                    curr_y = curr_y + state.start_vehicle_pose.y

                # If the pedestrian's frame is CURRENT, convert the pedestrian's frame to START.
                elif a.pose.frame == ObjectFrameEnum.CURRENT:
                    a_x = a.pose.x + curr_x
                    a_y = a.pose.y + curr_y
                    a_v_x = a_v_x - curr_v

                #Filter out vehicle that is not in front of the ego vehicle
                if a_y + VEHICLE_SIDE > curr_y or a_y - VEHICLE_SIDE < curr_y:
                    continue

                gap = a_x - curr_x
                relative_speed = curr_v - a_v_x

                if gap > self.safe_distance:
                    acceleration = self.acceleration
                elif gap < self.safe_distance:
                    # Too close, decelerate
                    acceleration = max(-self.max_deceleration, (gap - self.safe_distance) * self.gap_factor + relative_speed * self.speed_factor)
                else:
                    acceleration = 0  # Maintain current speed

                # Add the relation to the list
                res.append(EntityRelation(type=EntityRelationEnum.FOLLOWING, obj1='', obj2=n,
                                          acceleration=acceleration))
                
        return res
