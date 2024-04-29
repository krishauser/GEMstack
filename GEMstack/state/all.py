from __future__ import annotations
from dataclasses import dataclass, field, fields, asdict, replace
from ..utils.serialization import register
from .physical_object import ObjectFrameEnum,ObjectPose
from .scene import SceneState
from .intent import VehicleIntent
from .agent_intent import AgentIntent,AgentIntentMixture
from .relations import EntityRelation
from .mission import MissionObjective
from .route import Route
from .trajectory import Trajectory
from .predicates import PredicateValues
from typing import Dict,List,Optional

@dataclass
@register
class AllState(SceneState):
    """Contains all items that will be generated during the computation of the
    onboard behavior.
    """
    # non-physical scene state
    start_vehicle_pose : Optional[ObjectPose] = None
    agent_intents : Dict[str,AgentIntentMixture] = field(default_factory=dict)
    relations : List[EntityRelation] = field(default_factory=list)
    predicates : PredicateValues = field(default_factory=PredicateValues)
    
    # planner-output state
    mission : MissionObjective = field(default_factory=MissionObjective)
    intent : VehicleIntent = field(default_factory=VehicleIntent)
    route : Optional[Route] = None
    trajectory : Optional[Trajectory] = None

    # added state
    # for parking
    parking_slot : Optional[ObjectPose] = None
    # for normal road driving
    lane_goal: Optional[ObjectPose] = None
    lane_bound: Optional[List[float]] = None
    
    # update times for perception items (time.time())
    vehicle_update_time : float = 0
    roadgraph_update_time : float = 0
    environment_update_time : float = 0
    agent_update_time : float = 0
    obstacle_update_time : float = 0
    start_vehicle_pose_update_time : float = 0
    agent_intents_update_time : float = 0
    relations_update_time : float = 0
    predicates_update_time : float = 0

    # update times for planner items
    mission_update_time : float = 0
    intent_update_time : float = 0
    route_update_time : float = 0
    trajectory_update_time : float = 0

    @staticmethod
    def zero():
        scene_zero = SceneState.zero()
        keys = dict((k.name,getattr(scene_zero,k.name)) for k in fields(scene_zero))
        return AllState(**keys)
    
    def to_frame(self, frame : ObjectFrameEnum) -> AllState:
        spose = self.start_vehicle_pose
        scene_to_frame = SceneState.to_frame(self,frame,current_pose=self.vehicle.pose,start_pose_abs=spose)
        new_intents = None if self.agent_intents is None else dict((k,v.to_frame(frame,current_pose=self.vehicle.pose,start_pose_abs=spose)) for k,v in self.agent_intents.items())
        new_route = None if self.route is None else self.route.to_frame(frame,current_pose=self.vehicle.pose,start_pose_abs=spose)
        new_trajectory = None if self.trajectory is None else self.trajectory.to_frame(frame,current_pose=self.vehicle.pose,start_pose_abs=spose)
        return replace(scene_to_frame, agent_intents = new_intents, route = new_route, trajectory = new_trajectory)