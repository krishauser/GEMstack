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
from typing import Dict, List, Optional, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

from .sensorFusion.eval_3d_bbox_performance import calculate_3d_iou


def merge_boxes(box1: BoundingBox, box2: BoundingBox) -> BoundingBox:
     # TODO:  merging 
     # Heuristics-  Average pose
     #         Average dimensions
     #         Use highest score
     #         Label specific logic
     merged_box = BoundingBox()
     merged_box.header = box1.header # Use header from one input

     ## Avg position, average dimensions, max score, box1 label
     merged_box.pose.position.x = (box1.pose.position.x + box2.pose.position.x) / 2.0
     merged_box.pose.position.y = (box1.pose.position.y + box2.pose.position.y) / 2.0
     merged_box.pose.position.z = (box1.pose.position.z + box2.pose.position.z) / 2.0
     # Avg orientation (quaternions)
     merged_box.pose.orientation = box1.pose.orientation 
     merged_box.dimensions.x = (box1.dimensions.x + box2.dimensions.x) / 2.0
     merged_box.dimensions.y = (box1.dimensions.y + box2.dimensions.y) / 2.0
     merged_box.dimensions.z = (box1.dimensions.z + box2.dimensions.z) / 2.0
     merged_box.value = max(box1.value, box2.value) # Max score
     merged_box.label = box1.label # Label from first box

     return merged_box

def get_aabb_corners(box: BoundingBox):
    """
    Calculates the 3D Intersection over Union (IoU) between two bounding boxes.
    This implementation uses axis-aligned bounding boxes so it does not consider rotation.
    """

    # Extract position and dimensions from each box
    cx, cy, cz = box.pose.position.x, box.pose.position.y, box.pose.position.z
    l, w, h = box.dimensions.x, box.dimensions.y, box.dimensions.z

    # min_x, max_x, min_y, max_y, min_z, max_z    
    return cx, cx + l, cy, cy + w, cz, cz + h

def get_volume(box):
    return box.dimensions.x * box.dimensions.y * box.dimensions.z

class CombinedDetector3D(Component):
    def __init__(
        self,
        vehicle_interface: GEMInterface,
        enable_tracking: bool = True,
        use_start_frame: bool = True,
        iou_threshold: float = 0.1, 
        **kwargs 
    ):
        self.vehicle_interface   = vehicle_interface
        self.tracked_agents: Dict[str, AgentState] = {}
        self.ped_counter         = 0
        self.latest_yolo_bbxs: Optional[BoundingBoxArray] = None
        self.latest_pp_bbxs: Optional[BoundingBoxArray] = None
        self.start_pose_abs: Optional[ObjectPose]      = None
        self.start_time: Optional[float]          = None

        self.enable_tracking = enable_tracking
        self.use_start_frame = use_start_frame
        self.iou_threshold = iou_threshold

        self.yolo_topic = "/yolo_boxes"
        self.pp_topic = "/pointpillars_boxes"
        self.debug = False

        rospy.loginfo(f"CombinedDetector3D Initialized. Subscribing to '{self.yolo_topic}' and '{self.pp_topic}'.")

    def rate(self) -> float:
        return 8.0

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        self.yolo_sub = Subscriber(self.yolo_topic, BoundingBoxArray)
        self.pp_sub = Subscriber(self.pp_topic, BoundingBoxArray)
        self.pub_fused = rospy.Publisher("/fused_boxes", BoundingBoxArray, queue_size=1)

        queue_size = 10
        slop = 0.1

        self.sync = ApproximateTimeSynchronizer(
            [self.yolo_sub, self.pp_sub],
            queue_size=queue_size,
            slop=slop
        )
        self.sync.registerCallback(self.synchronized_callback)
        rospy.loginfo("CombinedDetector3D Subscribers Initialized.")

    def synchronized_callback(self, yolo_bbxs_msg: BoundingBoxArray, pp_bbxs_msg: BoundingBoxArray):
        self.latest_yolo_bbxs = yolo_bbxs_msg
        self.latest_pp_bbxs = pp_bbxs_msg

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        current_time = self.vehicle_interface.time()

        yolo_bbx_array = self.latest_yolo_bbxs
        pp_bbx_array = self.latest_pp_bbxs

        if yolo_bbx_array is None or pp_bbx_array is None:
            return {} 

        if self.start_time is None:
            self.start_time = current_time
        if self.use_start_frame and self.start_pose_abs is None:
            self.start_pose_abs = vehicle.pose
            rospy.loginfo("CombinedDetector3D latched start pose.")

        current_frame_agents = self._fuse_bounding_boxes(yolo_bbx_array, pp_bbx_array, vehicle, current_time)

        if self.enable_tracking:
            self._update_tracking(current_frame_agents)
        else:
            self.tracked_agents = current_frame_agents # NOTE: No deepcopy

        return self.tracked_agents


    def _fuse_bounding_boxes(self,
                             yolo_bbx_array: BoundingBoxArray,
                             pp_bbx_array: BoundingBoxArray,
                             vehicle_state: VehicleState,
                             current_time: float
                            ) -> Dict[str, AgentState]:
        original_header = yolo_bbx_array.header
        current_agents_in_frame: Dict[str, AgentState] = {}
        yolo_boxes: List[BoundingBox] = yolo_bbx_array.boxes
        pp_boxes: List[BoundingBox] = pp_bbx_array.boxes

        output_frame_enum = ObjectFrameEnum.START if self.use_start_frame else ObjectFrameEnum.CURRENT

        matched_yolo_indices = set()
        matched_pp_indices = set()
        fused_boxes_list: List[BoundingBox] = [] 

        # Can optimize from NxM loop
        for i, yolo_box in enumerate(yolo_boxes):
            best_match_j = -1
            best_iou = -1.0
            for j, pp_box in enumerate(pp_boxes):
                if j in matched_pp_indices: # Skip already matched PP boxes
                    continue

                ## IoU
                iou = calculate_3d_iou(yolo_box, pp_box, get_aabb_corners, get_volume)

                if iou > self.iou_threshold and iou > best_iou:
                    best_iou = iou
                    best_match_j = j

            if best_match_j != -1:
                rospy.logdebug(f"Matched YOLO box {i} with PP box {best_match_j} (IoU: {best_iou:.3f})")
                matched_yolo_indices.add(i)
                matched_pp_indices.add(best_match_j)
                merged = merge_boxes(yolo_box, pp_boxes[best_match_j])
                fused_boxes_list.append(merged)

        ## Unmatched Bboxes
        for i, yolo_box in enumerate(yolo_boxes):
            if i not in matched_yolo_indices:
                fused_boxes_list.append(yolo_box)
                rospy.logdebug(f"Kept unmatched YOLO box {i}")

        for j, pp_box in enumerate(pp_boxes):
            if j not in matched_pp_indices:
                fused_boxes_list.append(pp_box)
                rospy.logdebug(f"Kept unmatched PP box {j}")

        if self.debug:
            # Work in progress to visualize combined results
            fused_array = BoundingBoxArray()
            fused_array.header = yolo_bbx_array.header
            fused_array.boxes = fused_boxes_list
            self.pub_fused.publish(fused_array)

        for i, box in enumerate(fused_boxes_list):
            try:
                 # Cur vehicle frame
                pos_x = box.pose.position.x; pos_y = box.pose.position.y; pos_z = box.pose.position.z
                quat_x = box.pose.orientation.x; quat_y = box.pose.orientation.y; quat_z = box.pose.orientation.z; quat_w = box.pose.orientation.w
                yaw, pitch, roll = R.from_quat([quat_x, quat_y, quat_z, quat_w]).as_euler('zyx', degrees=False)

                # Start frame
                if self.use_start_frame and self.start_pose_abs is not None:
                     vehicle_pose_in_start_frame = vehicle_state.pose.to_frame(
                         ObjectFrameEnum.START, vehicle_state.pose, self.start_pose_abs
                     )
                     T_vehicle_to_start = pose_to_matrix(vehicle_pose_in_start_frame)
                     object_pose_current_h = np.array([[pos_x],[pos_y],[pos_z],[1.0]])
                     object_pose_start_h = T_vehicle_to_start @ object_pose_current_h
                     final_x, final_y, final_z = object_pose_start_h[:3, 0]
                else:
                      final_x, final_y, final_z = pos_x, pos_y, pos_z

                final_pose = ObjectPose(
                    t=current_time, x=final_x, y=final_y, z=final_z,
                    yaw=yaw, pitch=pitch, roll=roll, frame=output_frame_enum
                )
                dims = (box.dimensions.x, box.dimensions.y, box.dimensions.z)
                ######### Mapping based on label (integer) from BoundingBox msg
                agent_type = AgentEnum.PEDESTRIAN if box.label == 0 else AgentEnum.UNKNOWN # Needs refinement
                activity = AgentActivityEnum.UNKNOWN # Placeholder

                # temp id 
                # _update_tracking assign persistent IDs
                temp_agent_id = f"FrameDet_{i}"

                current_agents_in_frame[temp_agent_id] = AgentState(
                    pose=final_pose, dimensions=dims, outline=None, type=agent_type,
                    activity=activity, velocity=(0.0,0.0,0.0), yaw_rate=0.0
                    #  score=box.value  # score
                )
            except Exception as e:
                rospy.logwarn(f"Failed to convert final BoundingBox {i} to AgentState: {e}")
                continue

        return current_agents_in_frame


    def _update_tracking(self, current_frame_agents: Dict[str, AgentState]):

        #   Todo tracking
        ## Match 'current_frame_agents' to 'self.tracked_agents'.
        ##    - Use position (already in correct START or CURRENT frame), maybe size/type.
        ##    - Need a matching algorithm (e.g., nearest neighbor within radius, Hungarian).
        ## For matched pairs:
        ##    - Update the existing agent in 'self.tracked_agents' (e.g., smooth pose, update timestamp).
        ## For unmatched 'current_frame_agents':
        ##    - These are new detections. Assign a persistent ID (e.g., f"Ped_{self.ped_counter}").
        ##    - Increment self.ped_counter.
        ##    - Add them to 'self.tracked_agents'.
        ## For unmatched 'self.tracked_agents' (agents not seen this frame):
        ##    - Increment a 'missed frames' counter or check timestamp.
        ##    - If missed for too long (e.g., > 1 second), remove from 'self.tracked_agents'.

        # return without tracking
        self.tracked_agents = current_frame_agents



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
