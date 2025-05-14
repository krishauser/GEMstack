from ...state import AllState, VehicleState, ObjectPose, ObjectFrameEnum, AgentState, AgentEnum, AgentActivityEnum
from ..interface.gem import GEMInterface
from ..component import Component
from .perception_utils import pose_to_matrix
from .perception_utils_gem import *
from typing import Dict, List, Optional, Tuple
import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Quaternion

from .sensorFusion.eval_3d_bbox_performance import calculate_3d_iou
import copy


def average_yaw(yaw1, yaw2):
    v1 = np.array([np.cos(yaw1), np.sin(yaw1)])
    v2 = np.array([np.cos(yaw2), np.sin(yaw2)])
    v_avg = (v1 + v2) / 2.0
    return np.arctan2(v_avg[1], v_avg[0])


def quaternion_to_yaw(quaternion_arr):
    return R.from_quat(quaternion_arr).as_euler('zyx', degrees=False)[0]


def avg_orientations(orientation1, orientation2):
    """Average two quaternion orientations by converting to yaw and back."""
    # This function assumes both quaternions are 2D planar rotations around the z axis
    quat1 = [orientation1.x, orientation1.y, orientation1.z, orientation1.w]
    quat2 = [orientation2.x, orientation2.y, orientation2.z, orientation2.w]
    
    yaw1 = quaternion_to_yaw(quaternion_arr=quat1)
    yaw2 = quaternion_to_yaw(quaternion_arr=quat2)

    # Compute the average yaw and then convert back into quaternions
    # (averaging quaternions directly causes problems)
    avg_yaw = average_yaw(yaw1=yaw1, yaw2=yaw2)
    avg_quat = R.from_euler('z', avg_yaw).as_quat()

    orientation = Quaternion()
    orientation.x = float(avg_quat[0])
    orientation.y = float(avg_quat[1])
    orientation.z = float(avg_quat[2])
    orientation.w = float(avg_quat[3])

    return orientation


def merge_boxes(box1: BoundingBox, box2: BoundingBox, mode: str = "Average") -> BoundingBox:
    """
    Merge two bounding boxes using the specified mode.
    
    Args:
        box1: First bounding box
        box2: Second bounding box
        mode: Merging strategy to use. Options include:
              - "Average": Simple average of position, dimensions, and orientation (default)
              - "Score": Weight average by confidence scores (box.value)
              - "Max": Use the entire box with the highest confidence score
              
    Notes:
        - The 'value' field in BoundingBox is used for the confidence score
        - The label from the box with higher confidence is used when using Score mode
        - In Average mode, the label from the first box is used
        - Max mode returns a copy of the box with the highest confidence score
        
    Returns:
        A merged bounding box
    """
     
    merged_box = BoundingBox()
    merged_box.header = box1.header # Use header from one input
    
    # Get confidence scores
    score1 = box1.value if hasattr(box1, 'value') else 0.0
    score2 = box2.value if hasattr(box2, 'value') else 0.0
    
    if mode == "Max":
        # Use the box with the higher confidence score entirely
        if score1 >= score2:
            return copy.deepcopy(box1)
        else:
            return copy.deepcopy(box2)
    
    elif mode == "Score":
        # If both scores are 0, fall back to average
        if score1 == 0.0 and score2 == 0.0:
            weight1, weight2 = 0.5, 0.5
        else:
            # Calculate normalized weights based on scores
            total_score = score1 + score2
            weight1 = score1 / total_score
            weight2 = score2 / total_score
            
        # Weighted average of position
        merged_box.pose.position.x = (weight1 * box1.pose.position.x) + (weight2 * box2.pose.position.x)
        merged_box.pose.position.y = (weight1 * box1.pose.position.y) + (weight2 * box2.pose.position.y)
        merged_box.pose.position.z = (weight1 * box1.pose.position.z) + (weight2 * box2.pose.position.z)
        
        # Weighted average of dimensions
        merged_box.dimensions.x = (weight1 * box1.dimensions.x) + (weight2 * box2.dimensions.x)
        merged_box.dimensions.y = (weight1 * box1.dimensions.y) + (weight2 * box2.dimensions.y)
        merged_box.dimensions.z = (weight1 * box1.dimensions.z) + (weight2 * box2.dimensions.z)
        
        # For orientation, we still use the average_orientations function
        # More advanced: implement weighted quaternion averaging
        merged_box.pose.orientation = avg_orientations(box1.pose.orientation, box2.pose.orientation)
        
        # For label, use the one from the higher-score box
        merged_box.label = box1.label if score1 >= score2 else box2.label

    elif mode == "BEV":
        # Merge the bounding boxes from Bird's Eye View (BEV)
        # Average the x and y centers and dimensions
        # Average the yaw orientation
        # Use YOLO bounding box (box1) for z dimension and z center
        merged_box.pose.position.x = (box1.pose.position.x + box2.pose.position.x) / 2.0
        merged_box.pose.position.y = (box1.pose.position.y + box2.pose.position.y) / 2.0
        merged_box.pose.position.z = copy.deepcopy(box1.pose.position.z)

        # Avg orientations (quaternions)
        merged_box.pose.orientation = avg_orientations(box1.pose.orientation, box2.pose.orientation) 

        merged_box.dimensions.x = (box1.dimensions.x + box2.dimensions.x) / 2.0
        merged_box.dimensions.y = (box1.dimensions.y + box2.dimensions.y) / 2.0
        merged_box.dimensions.z = copy.deepcopy(box1.dimensions.z)

        merged_box.label = box1.label # Label from first box
        
    else:  # Default to "Average" mode
        # Original averaging logic
        merged_box.pose.position.x = (box1.pose.position.x + box2.pose.position.x) / 2.0
        merged_box.pose.position.y = (box1.pose.position.y + box2.pose.position.y) / 2.0
        merged_box.pose.position.z = (box1.pose.position.z + box2.pose.position.z) / 2.0
        
        # Avg orientations (quaternions)
        merged_box.pose.orientation = avg_orientations(box1.pose.orientation, box2.pose.orientation) 

        merged_box.dimensions.x = (box1.dimensions.x + box2.dimensions.x) / 2.0
        merged_box.dimensions.y = (box1.dimensions.y + box2.dimensions.y) / 2.0
        merged_box.dimensions.z = (box1.dimensions.z + box2.dimensions.z) / 2.0
        merged_box.label = box1.label # Label from first box
    
    # Always use max score for the merged box confidence value
    merged_box.value = max(score1, score2)

    return merged_box


def get_aabb_corners(box: BoundingBox):
    """
    Get axis-aligned bounding box corners for IoU calculation.
    Use axis-aligned bounding boxes so it does not consider rotation.
    
    Returns:
        min_x, max_x, min_y, max_y, min_z, max_z coordinates
    """
    # Extract position and dimensions from each box
    cx, cy, cz = box.pose.position.x, box.pose.position.y, box.pose.position.z
    l, w, h = box.dimensions.x, box.dimensions.y, box.dimensions.z

    # min_x, max_x, min_y, max_y, min_z, max_z    
    return cx, cx + l, cy, cy + w, cz, cz + h


def get_volume(box):
    """Calculate the volume of a bounding box."""
    return box.dimensions.x * box.dimensions.y * box.dimensions.z


def get_bev_aabb_corners(box: BoundingBox):
    """
    Get axis-aligned bounding box corners for 2D Bird's Eye View IoU calculation.
    Returns:
        min_x, max_x, min_y, max_y
    """
    cx, cy = box.pose.position.x, box.pose.position.y
    l, w = box.dimensions.x, box.dimensions.y

    return cx, cx + l, cy, cy + w


def calculate_bev_iou(box1: BoundingBox, box2: BoundingBox):
    """
    Calculates the 2D Bird's Eye View IoU between two bounding boxes.
    Ignores z-axis and yaw (assumes axis aligned bounding boxes).
    """
    min_x1, max_x1, min_y1, max_y1 = get_bev_aabb_corners(box1)
    min_x2, max_x2, min_y2, max_y2 = get_bev_aabb_corners(box2)

    # Calculate intersection in BEV
    inter_min_x = max(min_x1, min_x2)
    inter_max_x = min(max_x1, max_x2)
    inter_min_y = max(min_y1, min_y2)
    inter_max_y = min(max_y1, max_y2)

    inter_w = max(0, inter_max_x - inter_min_x)
    inter_h = max(0, inter_max_y - inter_min_y)
    intersection_area = inter_w * inter_h

    # Calculate union area
    area1 = max(box1.dimensions.x * box1.dimensions.y, 1e-6)
    area2 = max(box2.dimensions.x * box2.dimensions.y, 1e-6)
    union_area = max(area1 + area2 - intersection_area, 1e-6)

    iou = intersection_area / union_area
    return max(0.0, min(iou, 1.0))  # Clamp to [0, 1]


class CombinedDetector3D(Component):
    """
    Combines detections from multiple 3D object detectors (YOLO and PointPillars).
    
    This component subscribes to bounding box outputs from YOLO and PointPillars,
    fuses overlapping detections, and outputs a unified set of 3D bounding boxes.
    """
    
    def __init__(
        self,
        vehicle_interface: GEMInterface,
        enable_tracking: bool = True,
        use_start_frame: bool = True,
        iou_threshold: float = 0.1,
        merge_mode: str = "Average",
        **kwargs 
    ):
        self.vehicle_interface = vehicle_interface
        self.tracked_agents: Dict[str, AgentState] = {}
        self.ped_counter = 0
        self.latest_yolo_bbxs: Optional[BoundingBoxArray] = None
        self.latest_pp_bbxs: Optional[BoundingBoxArray] = None
        self.start_pose_abs: Optional[ObjectPose] = None
        self.start_time: Optional[float] = None

        self.enable_tracking = enable_tracking
        self.use_start_frame = use_start_frame
        self.iou_threshold = iou_threshold
        self.merge_mode = merge_mode
        self.merge_in_bev = (merge_mode == "BEV")

        self.yolo_topic = '/yolo_boxes'
        self.pp_topic = '/pointpillars_boxes'
        self.debug = False

        rospy.loginfo(f"CombinedDetector3D Initialized. Subscribing to '{self.yolo_topic}' and '{self.pp_topic}'.")
        rospy.loginfo(f"Using merge mode: {self.merge_mode}")

    def rate(self) -> float:
        return 8.0

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def initialize(self):
        """Initialize subscribers and publishers."""
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
        """Callback for synchronized YOLO and PointPillars messages."""
        self.latest_yolo_bbxs = yolo_bbxs_msg
        self.latest_pp_bbxs = pp_bbxs_msg

    def update(self, vehicle: VehicleState) -> Dict[str, AgentState]:
        """Update function called by the GEMstack pipeline."""
        current_time = self.vehicle_interface.time()
        agents: Dict[str, AgentState] = {}

        if self.start_time is None:
            self.start_time = current_time

        yolo_bbx_array = copy.deepcopy(self.latest_yolo_bbxs)
        pp_bbx_array = copy.deepcopy(self.latest_pp_bbxs)

        if yolo_bbx_array is None or pp_bbx_array is None:
            return {} 

        original_header = yolo_bbx_array.header
        fused_boxes_list = self._fuse_bounding_boxes(yolo_bbx_array, pp_bbx_array)

        # Used to visualize the combined results in the current frame
        fused_bb_array = BoundingBoxArray()
        fused_bb_array.header = original_header

        for i, box in enumerate(fused_boxes_list):
            fused_bb_array.boxes.append(box)
            rospy.loginfo(len(fused_boxes_list))
            
            # Get position and orientation in current vehicle frame
            # pos_x - z are returned as Quaternions.
            pos_x = box.pose.position.x
            pos_y = box.pose.position.y
            pos_z = box.pose.position.z
            quat_x = box.pose.orientation.x
            quat_y = box.pose.orientation.y
            quat_z = box.pose.orientation.z
            quat_w = box.pose.orientation.w
            yaw, pitch, roll = R.from_quat([quat_x, quat_y, quat_z, quat_w]).as_euler('zyx', degrees=False)

            if self.use_start_frame:
                out_frame = ObjectFrameEnum.START
                # Get the starting vehicle pose
                if self.start_pose_abs is None:
                    self.start_pose_abs = vehicle.pose
                
                # Convert to start frame
                vehicle_start_pose = vehicle.pose.to_frame(
                    ObjectFrameEnum.START, vehicle.pose, self.start_pose_abs
                )
                # T_vehicle_to_start = pose_to_matrix(vehicle_start_pose)
                T_vehicle_to_start = vehicle_start_pose.transform()
                object_pose_current_h = np.array([[pos_x],[pos_y],[pos_z],[1.0]])
                object_pose_start_h = T_vehicle_to_start @ object_pose_current_h
                final_x, final_y, final_z = object_pose_start_h[:3, 0]
            else:
                out_frame = ObjectFrameEnum.CURRENT
                final_x = pos_x
                final_y = pos_y
                final_z = pos_z

            new_pose = ObjectPose(
                t=current_time, x=final_x, y=final_y, z=final_z,
                yaw=yaw, pitch=pitch, roll=roll, frame=out_frame
            )
            dims = (box.dimensions.x, box.dimensions.y, box.dimensions.z * 2.0) # AgentState has z center on the floor and height is full height.

            new_pose = ObjectPose(
                t=current_time, x=final_x, y=final_y, z=final_z - box.dimensions.z / 2.0,
                yaw=yaw, pitch=pitch, roll=roll, frame=out_frame
            )

            existing_id = match_existing_cone(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=2.0
            )

            if existing_id is not None:
                old_state = self.tracked_agents[existing_id]
                dt = new_pose.t - old_state.pose.t
                vx, vy, vz = compute_velocity(old_state.pose, new_pose, dt)
                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity=(vx, vy, vz),
                    yaw_rate=0
                )
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent
            else:
                agent_id = f"Pedestrian{self.ped_counter}"
                self.ped_counter += 1
                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=AgentEnum.PEDESTRIAN,
                    activity=AgentActivityEnum.MOVING,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent

        self.pub_fused.publish(fused_bb_array)

        stale_ids = [agent_id for agent_id, agent in self.tracked_agents.items()
                    if current_time - agent.pose.t > 5.0]
        for agent_id in stale_ids:
            rospy.loginfo(f"Removing stale agent: {agent_id}\n")
        for agent_id, agent in agents.items():
            p = agent.pose
            # Format pose and velocity with 3 decimals (or as needed)
            rospy.loginfo(
                f"Agent ID: {agent_id}\n"
                f"Pose: (x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, "
                f"yaw: {p.yaw:.3f}, pitch: {p.pitch:.3f}, roll: {p.roll:.3f})\n"
                f"Velocity: (vx: {agent.velocity[0]:.3f}, vy: {agent.velocity[1]:.3f}, vz: {agent.velocity[2]:.3f})\n"
        )
        
        return agents


    def _fuse_bounding_boxes(self,
                             yolo_bbx_array: BoundingBoxArray,
                             pp_bbx_array: BoundingBoxArray,
                            ):
        """
        Fuse bounding boxes from multiple detectors.
        
        Args:
            yolo_bbx_array: Bounding boxes from YOLO detector
            pp_bbx_array: Bounding boxes from PointPillars detector
        """
        yolo_boxes: List[BoundingBox] = yolo_bbx_array.boxes
        pp_boxes: List[BoundingBox] = pp_bbx_array.boxes

        matched_yolo_indices = set()
        matched_pp_indices = set()
        fused_boxes_list: List[BoundingBox] = [] 

        # Match the boxes
        # Can optimize from NxM loop
        for i, yolo_box in enumerate(yolo_boxes):
            best_match_j = -1
            best_iou = -1.0
            for j, pp_box in enumerate(pp_boxes):
                if j in matched_pp_indices: # Skip already matched PP boxes
                    continue

                ## IoU
                iou = None
                if self.merge_in_bev:
                    iou = calculate_bev_iou(yolo_box, pp_box)
                else:
                    iou = calculate_3d_iou(yolo_box, pp_box, get_aabb_corners, get_volume)

                if iou > self.iou_threshold and iou > best_iou:
                    best_iou = iou
                    best_match_j = j

            if best_match_j != -1:
                rospy.logdebug(f"Matched YOLO box {i} with PP box {best_match_j} (IoU: {best_iou:.3f})")
                matched_yolo_indices.add(i)
                matched_pp_indices.add(best_match_j)
                rospy.logdebug(f"Using merge mode: {self.merge_mode} for boxes with scores {yolo_box.value:.2f} and {pp_boxes[best_match_j].value:.2f}")
                merged = merge_boxes(yolo_box, pp_boxes[best_match_j], mode=self.merge_mode)
                fused_boxes_list.append(merged)

        ##  Add the unmatched YOLO boxes
        for i, yolo_box in enumerate(yolo_boxes):
            if i not in matched_yolo_indices:
                fused_boxes_list.append(yolo_box)
                rospy.logdebug(f"Kept unmatched YOLO box {i}")

        # Add the unmatched PointPillars boxes
        for j, pp_box in enumerate(pp_boxes):
            if j not in matched_pp_indices:
                fused_boxes_list.append(pp_box)
                rospy.logdebug(f"Kept unmatched PP box {j}")

        return fused_boxes_list


# Fake 2D Combined Detector for testing purposes
class FakeCombinedDetector2D(Component):
    """Test detector that simulates cone detections at fixed time intervals."""
    
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
    """Convert 2D bounding box to a fake agent state."""
    x, y, w, h = box
    pose = ObjectPose(t=0, x=x + w / 2, y=y + h / 2, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.CURRENT)
    dims = (w, h, 0)
    return AgentState(pose=pose, dimensions=dims, outline=None,
                      type=AgentEnum.CONE, activity=AgentActivityEnum.MOVING,
                      velocity=(0, 0, 0), yaw_rate=0)


if __name__ == '__main__':
    pass
