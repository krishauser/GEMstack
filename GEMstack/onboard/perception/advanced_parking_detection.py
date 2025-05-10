import os
import cv2
import rospy
from typing import Dict
from ultralytics import YOLO
from cv_bridge import CvBridge
from ..component import Component
from ...state import VehicleState, ObjectPose, ObjectFrameEnum, ObstacleState, ObstacleMaterialEnum, ObstacleStateEnum
from sensor_msgs.msg import Image
from .utils.detection_utils import *


class ParkingDetectorLaneBased(Component):
    def __init__(self):
        # Initial variables
        self.bridge = CvBridge()
        self.model_path = os.getcwd() + '/GEMstack/knowledge/detection/parking_spot.pt'
        self.model = YOLO(self.model_path)
        self.model.to('cuda')
        self.parking_spots_corners = []
        self.reduce_reflection = True

        # Subscribers
        self.sub_right_cam = rospy.Subscriber("/camera_fr/arena_camera_node/image_raw", Image, self.callback, queue_size=1)

        # Publishers
        self.pub_detection_fr = rospy.Publisher("/parking_spot_detection/annotated_image/front_right", Image, queue_size=1)


    # Main sensors callback
    def callback(self, right_cam_msg):
        # Process camera image message
        processed_image = self.process_image_msg(right_cam_msg)
        
        # Detection
        results = self.model(processed_image)
        r = results[0]
        masks = r.masks
        image_annotated = processed_image.copy()

        centers = []
        corners = []
        approxes = []
        if masks is not None:
            h_orig, w_orig = r.orig_shape
            for m in r.masks.data:
                # 1) Pull mask off GPU
                mask = m.detach().cpu().numpy().astype('uint8')

                # 2) Resize if shape mismatch
                if mask.shape != (h_orig, w_orig):
                    mask = cv2.resize(mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

                # 3) Find contours and approximate polygon to get corners
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    epsilon = 0.02 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)

                    if is_big_parallelogram(approx):
                        # Store corners
                        corners_four = approx.reshape(4, 2)
                        corners.append(corners_four)
                        # Store approxes
                        approxes.append(approx)
                        # Store centers
                        M = cv2.moments(mask)
                        cx = int(M['m10'] / (M['m00'] + 1e-6))
                        cy = int(M['m01'] / (M['m00'] + 1e-6))
                        centers.append((cx, cy))

        self.parking_spots_corners = corners
        self.visualize(image_annotated, centers, corners, approxes)
 

    # All local helper functions
    def process_image_msg(self, image_msg):
        # Convert image to cv2
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        if self.reduce_reflection:
            # Reduce reflection
            # Convert to LAB color space to isolate lightness channel
            lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)

            # Apply CLAHE to the Lightness channel to reduce overexposed/reflection areas
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(12, 12))
            l_clahe = clahe.apply(l)

            # Merge channels back and convert to BGR
            lab_clahe = cv2.merge((l_clahe, a, b))
            reflection_reduced_image = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)
            return reflection_reduced_image
        return cv_image
    

    def visualize(self, image, centers, corners, approxes):
        if len(corners) > 0:
            # Draw centers as red crosses
            for (cx, cy) in centers:
                cv2.drawMarker(image, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            
            # Draw corners
            for corners_four in corners:
                for (x, y) in corners_four:
                    cv2.circle(image, (x, y), 10, (255, 0, 0), -1)

            # Draw approxes
            for approx in approxes:
                cv2.polylines(image, [approx], isClosed=True, color=(0, 255, 0), thickness=5)

        # Publish the annotated
        right_cam_bev_annotated_ros_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.pub_detection_fr.publish(right_cam_bev_annotated_ros_img)


    def update(self, vehicle: VehicleState) -> Dict[str, ObstacleState]:
        pass


    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['vehicle']

    def state_outputs(self) -> list:
        return ['agents']

    def update(self, vehicle: VehicleState) -> dict:
        # return dictionary with one key: 'agents'
        return {'agents': {}}  # or real AgentState dict


if __name__ == "__main__":
    node = ParkingDetectorLaneBased()
    node.spin()