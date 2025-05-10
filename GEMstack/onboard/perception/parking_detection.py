import rospy
import yaml
import numpy as np
from sensor_msgs.msg import PointCloud2
from typing import Dict
from ..component import Component 
from ...state import ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum, ObstacleState
from ..interface.gem import GEMInterface
from .utils.constants import *
from .utils.math_utils import *
from .utils.detection_utils import *
from .utils.parking_utils import *
from .utils.visualization_utils import *


class ParkingSpotsDetector3D(Component):
    def __init__(
            self, 
            vehicle_interface: GEMInterface,
            camera_name: str,
            camera_calib_file: str,
            visualize_3d: bool = True
        ):
        # Init Variables
        self.vehicle_interface = vehicle_interface
        self.cone_pts_3D = []
        self.grouped_ordered_ground_centers_2D = []
        self.goal_parking_spot = None
        self.parking_obstacles_poses = []
        self.parking_obstacles_dims = []
        self.ground_threshold = GROUND_THRESHOLD 
        self.visualization = visualize_3d

        # Load camera lidar to vehicle transform from the supplied YAML
        with open(camera_calib_file, 'r') as f:
            calib = yaml.safe_load(f)
        cam_cfg = calib['cameras'][camera_name]
        self.T_l2v = np.array(cam_cfg['T_l2v'])

        if self.T_l2v is None:
            rospy.logerr("Camera calibration information missing. Please load the correct config.")

        # Subscribers (note: we need top lidar only for visualization)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.callback, queue_size=1)
        # self.lidar_sub = self.vehicle_interface.subscribe_sensor("top_lidar", self.callback)

        # Publishers (all topics are for visualization purposes)
        self.pub_lidar_top_vehicle_pc2 = rospy.Publisher("lidar_top_vehicle/point_cloud", PointCloud2, queue_size=10)
        self.pub_vehicle_marker = rospy.Publisher("vehicle/marker", MarkerArray, queue_size=10)
        self.pub_cones_centers_pc2 = rospy.Publisher("cones_detection/centers/point_cloud", PointCloud2, queue_size=10)
        self.pub_parking_spot_marker = rospy.Publisher("parking_spot_detection/marker", MarkerArray, queue_size=10)
        self.pub_polygon_marker = rospy.Publisher("polygon_detection/marker", MarkerArray, queue_size=10)
        self.pub_obstacles_marker = rospy.Publisher("obstacle_detection/marker", MarkerArray, queue_size=10)

    def callback(self, top_lidar_msg):
        # Top lidar points in ouster frame
        lidar_ouster_frame = pc2_to_numpy(top_lidar_msg)

        # Visualize everything in vehicle frame
        if self.visualization:
            self.visualize( 
                self.cone_pts_3D,
                self.grouped_ordered_ground_centers_2D,
                self.goal_parking_spot,
                self.parking_obstacles_poses,
                self.parking_obstacles_dims,
                lidar_ouster_frame
            )

    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['obstacles']

    def state_outputs(self) -> list:
        return ['goal', 'obstacles']


    def visualize(self, 
                cone_pts_3D,
                grouped_ordered_ground_centers_2D,
                goal_parking_spot,
                parking_obstacles_poses,
                parking_obstacles_dims,
                lidar_ouster_frame):
        # Transform top lidar pointclouds to vehicle frame for visualization
        latest_lidar_vehicle = transform_points(lidar_ouster_frame, self.T_l2v)
        latest_lidar_vehicle = filter_ground_points(latest_lidar_vehicle, self.ground_threshold)
        ros_lidar_top_vehicle_pc2 = create_point_cloud(latest_lidar_vehicle, LIDAR_PC_COLOR, VEHICLE_FRAME)
        self.pub_lidar_top_vehicle_pc2.publish(ros_lidar_top_vehicle_pc2)

        # Create vehicle marker
        ros_vehicle_marker = create_markers([VEHICLE_FRAME_ORIGIN], 
                                            [VEHICLE_MARKER_DIM], 
                                            VEHICLE_MARKER_COLOR, 
                                            "markers", VEHICLE_FRAME)
        self.pub_vehicle_marker.publish(ros_vehicle_marker)

        # Delete previous markers
        ros_delete_polygon_marker = delete_markers("polygon", MAX_POLYGON_MARKERS)
        self.pub_polygon_marker.publish(ros_delete_polygon_marker)
        ros_delete_parking_spot_markers = delete_markers("parking_spot", MAX_PARKING_SPOT_MARKERS)
        self.pub_parking_spot_marker.publish(ros_delete_parking_spot_markers)
        ros_delete_obstacles_markers = delete_markers("obstacles", MAX_OBSTACLE_MARKERS)
        self.pub_obstacles_marker.publish(ros_delete_obstacles_markers)

        # Draw polygon first
        if len(grouped_ordered_ground_centers_2D) > 0:
            ros_polygon_markers = create_polygon_markers(grouped_ordered_ground_centers_2D, ref_frame=VEHICLE_FRAME)
            self.pub_polygon_marker.publish(ros_polygon_markers)

        # Create parking spot marker
        if goal_parking_spot:
            ros_parking_spot_marker = create_parking_spot_marker(goal_parking_spot, ref_frame=VEHICLE_FRAME)
            self.pub_parking_spot_marker.publish(ros_parking_spot_marker)

        # Create parking obstacles marker
        if parking_obstacles_poses and parking_obstacles_dims:
            ros_obstacles_marker =  create_markers(parking_obstacles_poses, 
                                                      parking_obstacles_dims, 
                                                      OBSTACLE_MARKER_COLOR, 
                                                      "obstacles", VEHICLE_FRAME)
            self.pub_obstacles_marker.publish(ros_obstacles_marker)

        # Draw 3D cone centers
        cone_ground_centers = []
        if len(cone_pts_3D) > 0:
            cone_ground_centers = np.array(cone_pts_3D)
            cone_ground_centers[:, 2] = 0.0
            cone_ground_centers = [tuple(point) for point in cone_ground_centers]
        ros_cones_centers_pc2 = create_point_cloud(cone_ground_centers, color=CONE_CENTER_PC_COLOR)
        self.pub_cones_centers_pc2.publish(ros_cones_centers_pc2)


    def update(self, cone_obstacles: Dict[str, ObstacleState]):
        # Initial variables
        goal_parking_spots = []
        parking_obstacles_poses = []
        parking_obstacles_dims = []
        grouped_ordered_ground_centers_2D = []

        # Populate cone points
        cone_pts_3D = []
        for cone in cone_obstacles.values():
            cone_pt_3D = (cone.pose.x, cone.pose.y, 0.0)
            cone_pts_3D.append(cone_pt_3D)

        # Detect single/multi parking spot
        if len(cone_pts_3D) == NUM_CONES_PER_PARKING_SPOT:
            # Find goal parking pose of that spot, obstacles and centers that forms the polygon
            goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_ground_centers_2D = detect_parking_spot(cone_pts_3D)
            # add them to the list
            goal_parking_spots.append(goal_parking_spot)
            grouped_ordered_ground_centers_2D.append(ordered_ground_centers_2D)
            parking_obstacles_poses = parking_obstacles_pose
            parking_obstacles_dims = parking_obstacles_dim
        elif is_valid_multiple_of_k(len(cone_pts_3D), NUM_CONES_PER_PARKING_SPOT):
            grouped_cone_pts_3D = group_points_by_multiple_of_k(cone_pts_3D, NUM_CONES_PER_PARKING_SPOT)
            for gc in grouped_cone_pts_3D:
                # Find goal parking pose of that spot, obstacles and centers that forms the polygon
                goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_ground_centers_2D = detect_parking_spot(gc)
                # add them to the list
                goal_parking_spots.append(goal_parking_spot)
                grouped_ordered_ground_centers_2D.append(ordered_ground_centers_2D)
                parking_obstacles_poses += parking_obstacles_pose
                parking_obstacles_dims += parking_obstacles_dim

        # Update local variables for visualization
        self.cone_pts_3D = cone_pts_3D
        self.grouped_ordered_ground_centers_2D = grouped_ordered_ground_centers_2D
        self.parking_obstacles_poses = parking_obstacles_poses
        self.parking_obstacles_dims = parking_obstacles_dims
        
        # Now select the closest parking goal, return if none is found
        if len(goal_parking_spots) > 1:
            self.goal_parking_spot = closest_point_to_origin(goal_parking_spots)
        elif len(goal_parking_spots) == 1:
            self.goal_parking_spot = goal_parking_spots[0]
        else:
            self.goal_parking_spot = None
            return None
        
        # Constructing parking obstacles
        current_time = self.vehicle_interface.time()
        obstacle_id = 30
        parking_obstacles = {}
        for o_pose, o_dim in zip(self.parking_obstacles_poses, self.parking_obstacles_dims):
            x, y, z, yaw = o_pose
            obstacle_pose = ObjectPose(
                                t=current_time,
                                x=x,
                                y=y,
                                z=z,
                                yaw=yaw,
                                pitch=0.0,
                                roll=0.0,
                                frame=ObjectFrameEnum.CURRENT
                            )
            new_obstacle = Obstacle(
                                pose=obstacle_pose,
                                dimensions=o_dim,
                                outline=None,
                                material=ObstacleMaterialEnum.BARRIER,
                                collidable=True
                            )
            parking_obstacles[obstacle_id] = new_obstacle
            obstacle_id += 1
        
        # Constructing goal pose
        x, y, yaw = self.goal_parking_spot
        goal_pose = ObjectPose(
                        t=current_time,
                        x=x,
                        y=y,
                        z=0.0,
                        yaw=yaw,
                        pitch=0.0,
                        roll=0.0,
                        frame=ObjectFrameEnum.CURRENT
                    )
        
        new_state = [goal_pose, parking_obstacles]
        return new_state