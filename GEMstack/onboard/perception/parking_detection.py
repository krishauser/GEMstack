import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from typing import Dict
from ..component import Component 
from ...state import AgentState, ObjectPose, ObjectFrameEnum, Obstacle, ObstacleMaterialEnum
from ..interface.gem import GEMInterface
from .utils.detection_utils import *
from .utils.parking_utils import *
from .utils.visualization_utils import *


class ParkingSpotsDetector3D(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        # Init Variables
        self.vehicle_interface = vehicle_interface
        self.cone_pts_3D = []
        self.ordered_cone_ground_centers_2D = []
        self.goal_parking_spot = None
        self.parking_obstacles_pose = []
        self.parking_obstacles_dim = []
        self.ground_threshold = -0.15
        self.visualization = True
        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                               [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                               [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                               [0., 0., 0., 1.]])


        # Subscribers (note: we need top lidar only for visualization)
        self.lidar_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.callback, queue_size=1)

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
                self.ordered_cone_ground_centers_2D,
                self.goal_parking_spot,
                self.parking_obstacles_pose,
                self.parking_obstacles_dim,
                lidar_ouster_frame
            )

    def spin(self):
        rospy.spin()

    def rate(self) -> float:
        return 10.0  # Hz

    def state_inputs(self) -> list:
        return ['agents']

    def state_outputs(self) -> list:
        return ['goal', 'obstacles']


    def visualize(self, 
                cone_pts_3D,
                ordered_cone_ground_centers_2D,
                goal_parking_spot,
                parking_obstacles_pose,
                parking_obstacles_dim,
                lidar_ouster_frame):
        # Transform top lidar pointclouds to vehicle frame for visualization
        latest_lidar_vehicle = transform_points(lidar_ouster_frame, self.T_l2v)
        latest_lidar_vehicle = filter_ground_points(latest_lidar_vehicle, self.ground_threshold)
        ros_lidar_top_vehicle_pc2 = create_point_cloud(latest_lidar_vehicle, (255, 0, 0), "vehicle")
        self.pub_lidar_top_vehicle_pc2.publish(ros_lidar_top_vehicle_pc2)

        # Create vehicle marker
        ros_vehicle_marker = create_markers([[0.0, 0.0, 0.0, 0.0]], 
                                            [[0.8, 0.5, 0.3]], 
                                            (0.0, 0.0, 1.0, 1), 
                                            "markers", "vehicle")
        self.pub_vehicle_marker.publish(ros_vehicle_marker)

        # Delete previous markers
        ros_delete_polygon_marker = delete_markers("polygon", 1)
        self.pub_polygon_marker.publish(ros_delete_polygon_marker)
        ros_delete_parking_spot_markers = delete_markers("parking_spot", 1)
        self.pub_parking_spot_marker.publish(ros_delete_parking_spot_markers)
        ros_delete_obstacles_markers = delete_markers("obstacles", 5)
        self.pub_obstacles_marker.publish(ros_delete_obstacles_markers)

        # Draw polygon first
        if len(ordered_cone_ground_centers_2D) > 0:
            ros_polygon_marker = create_polygon_marker(ordered_cone_ground_centers_2D, ref_frame="vehicle")
            self.pub_polygon_marker.publish(ros_polygon_marker)

        # Create parking spot marker
        if goal_parking_spot:
            ros_parking_spot_marker = create_parking_spot_marker(goal_parking_spot, ref_frame="vehicle")
            self.pub_parking_spot_marker.publish(ros_parking_spot_marker)

        # Create parking obstacles marker
        if parking_obstacles_pose and parking_obstacles_dim:
            ros_obstacles_marker =  create_markers(parking_obstacles_pose, 
                                                      parking_obstacles_dim, 
                                                      (1.0, 0.0, 0.0, 0.4), 
                                                      "obstacles", "vehicle")
            self.pub_obstacles_marker.publish(ros_obstacles_marker)

        # Draw 3D cone centers
        if len(cone_pts_3D) > 0:
            cone_ground_centers = np.array(cone_pts_3D)
            cone_ground_centers[:, 2] = 0.0
            cone_ground_centers = [tuple(point) for point in cone_ground_centers]
            ros_cones_centers_pc2 = create_point_cloud(cone_ground_centers, color=(255, 0, 255))
            self.pub_cones_centers_pc2.publish(ros_cones_centers_pc2)


    def update(self, agents: Dict[str, AgentState]):
        # Initial variables
        goal_parking_spot = None
        parking_obstacles_pose = []
        parking_obstacles_dim = []
        ordered_cone_ground_centers_2D = []

        # Populate cone points
        cone_pts_3D = []
        for cone in agents.values():
            cone_pt_3D = (cone.pose.x, cone.pose.y, 0.0)
            cone_pts_3D.append(cone_pt_3D)

        # Detect parking spot
        if len(cone_pts_3D) >= 4:
            goal_parking_spot, parking_obstacles_pose, parking_obstacles_dim, ordered_cone_ground_centers_2D = detect_parking_spot(cone_pts_3D)
        # Update local variables for visualization
        self.cone_pts_3D = cone_pts_3D
        self.ordered_cone_ground_centers_2D = ordered_cone_ground_centers_2D
        self.goal_parking_spot = goal_parking_spot
        self.parking_obstacles_pose = parking_obstacles_pose
        self.parking_obstacles_dim = parking_obstacles_dim

        # Return if no goal parking spot is found
        if not goal_parking_spot:
            return None
        
        # Constructing parking obstacles
        current_time = self.vehicle_interface.time()
        obstacle_id = 0
        parking_obstacles = {}
        for o_pose, o_dim in zip(parking_obstacles_pose, parking_obstacles_dim):
            x, y, z, yaw = o_pose
            obstacle_pose = ObjectPose(
                                t=current_time,
                                x=x,
                                y=y,
                                z=z,
                                yaw=yaw,
                                pitch=0.0,
                                roll=0.0,
                                frame=ObjectFrameEnum.START
                            )
            new_obstacle = Obstacle(
                                pose=obstacle_pose,
                                dimensions=o_dim,
                                outline=None,
                                material=ObstacleMaterialEnum.BARRIER,
                                collidable=True
                            )
            parking_obstacles[f"parking_obstacle_{obstacle_id}"] = new_obstacle
            obstacle_id += 1
        
        # Constructing goal pose
        print(f"-----GOAL: {goal_parking_spot}")
        x, y, yaw = goal_parking_spot
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