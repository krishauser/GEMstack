import os
import cv2
import rospy
import numpy as np
from typing import Dict
from ultralytics import YOLO
from cv_bridge import CvBridge
from ..component import Component
from ...state import VehicleState, ObjectPose, ObjectFrameEnum, ObstacleState, ObstacleMaterialEnum, ObstacleStateEnum
from sensor_msgs.msg import Image


def is_parallelogram(approx, length_tolerance=0.2):
    if len(approx) != 4:
        return False

    if not cv2.isContourConvex(approx):
        return False

    # Extract the 4 points
    pts = [approx[i][0] for i in range(4)]

    # Compute side lengths
    def side_length(p1, p2):
        return np.linalg.norm(p1 - p2)

    lengths = [
        side_length(pts[0], pts[1]),  # Side 1
        side_length(pts[1], pts[2]),  # Side 2
        side_length(pts[2], pts[3]),  # Side 3
        side_length(pts[3], pts[0])   # Side 4
    ]

    # Check if opposite sides are approximately equal
    def is_close(l1, l2):
        return abs(l1 - l2) / max(l1, l2) < length_tolerance

    if not (is_close(lengths[0], lengths[2]) and is_close(lengths[1], lengths[3])):
        return False

    return True


def is_big_parallelogram(approx, min_area=1000, length_tolerance=0.2):
    if not is_parallelogram(approx, length_tolerance):
        return False
    area = cv2.contourArea(approx)
    return area >= min_area


class ParkingDetectorLaneBased(Component):
    def __init__(self):
        # Initial variables
        self.bridge = CvBridge()
        self.model_path = os.getcwd() + '/GEMstack/knowledge/detection/best.pt'
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

                # 3) Compute centroid
                M = cv2.moments(mask)
                cx = int(M['m10'] / (M['m00'] + 1e-6))
                cy = int(M['m01'] / (M['m00'] + 1e-6))
                centers.append((cx, cy))

                # 4) Find contours and approximate polygon to get corners
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    epsilon = 0.02 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)

                    if is_big_parallelogram(approx):
                        approxes.append(approx)
                        corners_four = approx.reshape(4, 2)
                        corners.append(corners_four)

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