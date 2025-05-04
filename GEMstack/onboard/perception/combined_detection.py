from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import *
from typing import Dict
import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time
import os
import yaml

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray


class CombinedDetector3D(Component):
    """
    Fuses the boxes in the lists of bounding boxes published by YoloNode and 
    PointPillarsNode with late sensor fusion.
    TODO: SUBSCRIBE TO BOUNDING BOX LISTS AND PERFORM LATE SENSOR FUSION IN THIS FILE.
    TODO: MODIFY YAML FILE FOR THE CONTROL TEAM'S BASIC PATH PLANNING CODE?

    Tracking is optional: set `enable_tracking=False` to disable persistent tracking
    and return only detections from the current frame.

    Supports multiple cameras; each camera’s intrinsics and extrinsics are
    loaded from a single YAML calibration file via plain PyYAML.
    """

    def __init__(
        self,
        vehicle_interface: GEMInterface,
        enable_tracking: bool = True,
        use_start_frame: bool = True,
        **kwargs
    ):
        # Core interfaces and state
        self.vehicle_interface   = vehicle_interface
        self.current_agents      = {}
        self.tracked_agents      = {}
        self.ped_counter         = 0
        self.latest_yolo_bbxs    = None # Stores the latest list of YOLO bounding boxes
        self.latest_pp_bbxs      = None # Stores the latest list of PointPillars bounding boxes
        self.start_pose_abs      = None
        self.start_time          = None

        # Config flags
        self.enable_tracking = enable_tracking
        self.use_start_frame = use_start_frame

    def rate(self) -> float:
        return 8

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        # Subscribe to the BoundingBox
        self.yolo_sub = Subscriber('/yolo_boxes', BoundingBoxArray)
        self.pp_sub = Subscriber('/pointpillars_boxes', BoundingBoxArray)
        self.sync = ApproximateTimeSynchronizer([
            self.yolo_sub, self.pp_sub
        ], queue_size=50, slop=0.05) # GREATLY DECREASED QUEUE SIZE, 50 might even be too much
        self.sync.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, yolo_bbxs_msg, pp_bbxs_msg):
        self.latest_yolo_bbxs = yolo_bbxs_msg
        self.latest_pp_bbxs = pp_bbxs_msg

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        # Gate guards against data not being present for both sensors:
        if self.latest_yolo_bbxs is None or self.latest_pp_bbxs is None:
            return {}
        
        # Set up current time variables
        start = time.time()
        current_time = self.vehicle_interface.time()

        if self.start_time is None:
            self.start_time = current_time
        time_elapsed = current_time - self.start_time

        agents = {}
        # TODO: Loop through bounding box lists here
        # The bounding box lists that were matched up by self.synchronized_callback SHOULD match up 
        # correctly because we manually inserted the time stamp of the lidar data into the header
        # of the bounding box list. So since ApproximateTimeSynchronizer syncs up messages which
        # have similar time stamps (assumed to be determined by the time stamp in the message header),
        # the bounding box lists being compared should be from the same point cloud data. The image
        # data paired with it may be slightly different but since the bounding boxes from both models
        # were built in 3D space using the lidar data, they should pair up well enough

        # The bounding boxes in both lists SHOULD ALREADY BE IN THE VEHICLE FRAME since we transformed
        # the data from lidar->vehicle before creating the bounding boxes and then publishing.

        # To compare the bounding boxes in the lists, we can either use a 2D intersection over union birds
        # eye view approach (since point pillars creates vertical pillars anyways) or we can do a 3D
        # intersection over union. We could then merge the boxes that match closely by averaging
        # their positions and dimensions and then we'd choose the label with the highest confidence.

        # For the leftover bounding boxes, we can still use them with their original confidence
        # (confidence was placed in the value field of each box).

        # Finally, we would need to convert each box to an AgentState object
        # We would then need to transform the AgentState object to the start frame to compare with old
        # AgentState objects to assign id's and calculate velocity
        # Then we would need to return the new list of AgentState objects

        end = time.time()
        # print('-------processing time---', end -start)
        return self.tracked_agents

# Fake 2D Combined Detector for testing purposes
# TODO FIX THIS
class FakeCombinedDetector2D(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        self.vehicle_interface = vehicle_interface
        self.times = [(5.0, 20.0), (30.0, 35.0)]
        self.t_start = None

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['agents']

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        t = self.vehicle_interface.time() - self.t_start
        res = {}
        for time_range in self.times:
            if t >= time_range[0] and t <= time_range[1]:
                res['cone0'] = box_to_fake_agent((0, 0, 0, 0))
                rospy.loginfo("Detected a Cone (simulated)")
        return res


def box_to_fake_agent(box):
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.CONE, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)


if __name__ == '__main__':
    pass