from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import collisions
from ..interface.gem import GEMInterface
from ..component import Component
from .object_detection import ObjectDetector

from ultralytics import YOLO
from typing import Dict
import threading
import copy


class AgentDetector(ObjectDetector):
    """Detects other agents (pedestrians + vehicles)."""

    def __init__(self, vehicle : VehicleState, camera_image, lidar_point_cloud):
        detector = YOLO(settings.get('perception.agent_detection.model'))
        super().__init__(vehicle, camera_image, lidar_point_cloud, detector)

    def object_to_agent(self, detected_object, bbox_cls):
        """Creates a 3D agent state from a PhysicalObject."""

        # set agent type based on class id
        type_dict = {
            '0': AgentEnum.PEDESTRIAN,
            '1': AgentEnum.BICYCLIST,
            '2': AgentEnum.CAR,
            '7': AgentEnum.MEDIUM_TRUCK if detected_object.dimensions[2] <= 2.0 else AgentEnum.LARGE_TRUCK
        }
        return AgentState(pose=detected_object.pose, dimensions=detected_object.dimensions, outline=None, 
                          type=type_dict[str(int(bbox_cls))], activity=AgentActivityEnum.STOPPED, 
                          velocity=(0,0,0), yaw_rate=0, attributes=None)

    def detect_agents(self):
        yolo_class_ids = [
            0,  # person
            1,  # bicycle
            2,  # car
            7   # truck
        ]
        detected_objects, bbox_classes = super().detect_objects(yolo_class_ids)

        detected_agents = []
        for i in range(len(detected_objects)):
            agent = self.object_to_agent(detected_objects[i], bbox_classes[i])
            detected_agents.append(agent)
        
        return detected_agents
    
    # def track_agents(self, detected_agents, prev_states, counter, rate):
    #     """ Given a list of detected agents, updates the state of the agents.
    #     - Keep track of which agents were detected before.
    #     - For each agent, assign appropriate ids and estimate velocities.
    #     """
    #     dt = 1 / rate # time between updates

    #     states = {}

    #     for agent in detected_agents:
    #         prev_key = super().deduplication(agent, prev_states)

    #         if prev_key is None: # new agent
    #             # velocity of a new agent is 0 by default
    #             states['agent_' + str(counter)] = agent
    #             counter += 1
    #         else:
    #             prev_agent = self.prev_states[prev_key]
    #             prev_pose = prev_agent.pose

    #             # absolute vel = vel w.r.t vehicle + vehicle velocity 
    #             v_x = (agent.pose.x - prev_pose.x) / dt + self.vehicle.v
    #             v_y = (agent.pose.y - prev_pose.y) / dt
    #             v_z = (agent.pose.z - prev_pose.z) / dt

    #             if any([v_x, v_y, v_z]):
    #                 agent.activity = AgentActivityEnum.MOVING
    #                 agent.velocity = (v_x, v_y, v_z)
            
    #             states[prev_key] = agent            
        
    #     return states, counter


class OmniscientAgentDetector(Component):
    """Obtains agent detections from a simulator"""
    def __init__(self,vehicle_interface : GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.agents = {}
        self.lock = threading.Lock()

    def rate(self):
        return 4.0
    
    def state_inputs(self):
        return []
    
    def state_outputs(self):
        return ['agents']

    def initialize(self):
        self.vehicle_interface.subscribe_sensor('agent_detector',self.agent_callback, AgentState)
    
    def agent_callback(self, name : str, agent : AgentState):
        with self.lock:
            self.agents[name] = agent

    def update(self) -> Dict[str,AgentState]:
        with self.lock:
            return copy.deepcopy(self.agents)
