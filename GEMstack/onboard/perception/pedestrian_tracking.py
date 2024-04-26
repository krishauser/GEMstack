from ...state import (
    ObjectPose,
    ObjectFrameEnum,
    AgentState,
    AgentEnum,
    AgentActivityEnum,
)
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import cv2

try:
    from sensor_msgs.msg import CameraInfo
    from image_geometry import PinholeCameraModel
    import rospy
except ImportError:
    pass

import numpy as np
from typing import Dict, List
from .kalman_tracker import KalmanTracker  # Behavior Prediction Team


class PedestrianTracker(Component):
    def __init__(
        self,
        kalman_config_file="GEMstack/onboard/perception/temp_config.py",
        detection_file_name="GEMstack/onboard/prediction/tracking_results.txt",
        test=False,
        write_all=False,
    ):
        self.kalman_tracker = KalmanTracker(config_file_path=kalman_config_file)
        self.tracking_results = {}
        self.current_frame = 0

        # For Testing
        self.test = test
        if self.test:
            self.detection_file_name = detection_file_name
            self.write_all = write_all
            f = open(self.detection_file_name, "w")
            f.close()

    """Base class for top-level components in the execution stack."""

    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 2.5

    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ["detected_agents"]

    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ["tracking_frames"]

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True

    def initialize(self):
        """Initialize the component. This is called once before the first
        update."""
        pass

    def cleanup(self):
        """Cleans up resources used by the component. This is called once after
        the last update."""
        pass

    def update(self, detected_agents: List[AgentState]):
        """Update the component."""
        self.track_agents(detected_agents=detected_agents)
        return self.output_tracking_results()

    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug(item, value)

    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, "debugger"):
            self.debugger.debug_event(label)

    def track_agents(self, detected_agents: List[AgentState]):
        """Given a list of detected agents, updates the state of the agents using a Kalman Tracker."""
        detections = []
        for agent in detected_agents:
            if agent.type == AgentEnum.PEDESTRIAN:
                x, y, z = agent.pose.x, agent.pose.y, agent.pose.z
                w, h, l = agent.dimensions
                detections.append(np.array([x, y, w, l]))

        kalman_agent_states, matches = self.kalman_tracker.update_objects_tracking(
            detections
        )
        tracking_results = {}
        for pid in kalman_agent_states:
            ag_state = kalman_agent_states[pid]
            tracking_results[pid] = AgentState(
                pose=ObjectPose(
                    t=0,
                    x=ag_state[0],
                    y=ag_state[1],
                    z=0,
                    yaw=0,
                    pitch=0,
                    roll=0,
                    frame=ObjectFrameEnum.CURRENT,
                ),
                dimensions=(ag_state[2], ag_state[3], 1.5),
                velocity=(ag_state[4], ag_state[5], 0),
                type=AgentEnum.PEDESTRIAN,
                activity=AgentActivityEnum.MOVING,
                yaw_rate=0,
                outline=None,
            )
        self.update_track_history(tracking_results)

        
        if self.test:
            return tracking_results, matches


    def update_track_history(
        self, ag_dict: Dict[int, AgentState]
    ):  # Behavior Prediction
        for pid in sorted(ag_dict):
            curr_agent = ag_dict[pid]
            curr_pose = curr_agent.pose
            # 11.0 5.0 Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.59 -1.0 0.93 -1.0
            agent_frame_data = f"{float(self.current_frame)} {float(pid)} Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 {curr_pose.x} -1.0 {curr_pose.y} -1.0\n"
            if self.current_frame in self.tracking_results:
                self.tracking_results[self.current_frame].append(agent_frame_data)
            else:
                self.tracking_results[self.current_frame] = [agent_frame_data]
        # Do not keep more than 8 frames
        if not self.write_all:
            # Remove 8th oldest frame
            self.tracking_results.pop(self.current_frame - 8, None)
        
        self.current_frame += 1

    # TODO: Write Docstring
    def output_tracking_results(self):  # Behavior Prediction
        tracking_frames = []

        recent_pids = {}
        
        # Dummy frames should be written for these pids
        valid_pids = set()
        with open(self.detection_file_name, "w") as f:
            for frame in sorted(self.tracking_results):
                for line in sorted(self.tracking_results[frame]):
                    
                    # Check whether a dummy frame should be written for this pid
                    curr_pid = line.split(" ")[1]
                    pid_seen_count = recent_pids.get(curr_pid, 0) + 1
                    if pid_seen_count >= 8:
                        valid_pids.add(curr_pid)
                        
                    recent_pids[curr_pid] = pid_seen_count
                        
                        
                    if self.test:
                        f.write(line)
                    tracking_frames.append(line)
            if not self.write_all:
                # Adding dummy frames, because we are only writing the recent frames
                for frame in range(self.current_frame, self.current_frame + 12):
                    for rpid in valid_pids:
                        dummy_frame = f"{float(frame)} {float(rpid)} Pedestrian -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 -1.0 0.0 -1.0 0.0 -1.0\n"
                        if self.test:
                            f.write(dummy_frame)
                        tracking_frames.append(dummy_frame)

        return tracking_frames
