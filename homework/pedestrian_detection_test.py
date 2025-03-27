#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import time

# -----------------------------
# 1) Minimal Stub Classes
# -----------------------------

class ObjectPose:
    def __init__(self, t, x, y, z, yaw, pitch, roll, frame):
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.frame = frame

class AgentState:
    def __init__(self, pose, dimensions, outline, type, activity, velocity, yaw_rate):
        self.pose = pose
        self.dimensions = dimensions
        self.outline = outline
        self.type = type
        self.activity = activity
        self.velocity = velocity
        self.yaw_rate = yaw_rate


# -----------------------------
# 2) Helper Functions
# -----------------------------



def match_existing_pedestrian(new_center, new_dims, existing_agents, distance_threshold=1.0):
    """
    Return the agent_id of the best match if within distance_threshold; otherwise None.
    """
    best_agent_id = None
    best_dist = float('inf')

    for agent_id, agent_state in existing_agents.items():
        old_center = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z])
        dist = np.linalg.norm(new_center - old_center)
        if dist < distance_threshold and dist < best_dist:
            best_dist = dist
            best_agent_id = agent_id

    return best_agent_id


def compute_velocity(old_pose, new_pose, dt):
    """
    Returns a (vx, vy, vz) velocity in the same frame as old_pose/new_pose.
    """
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)


def extract_roi_box(lidar_pc, center, half_extents):
    lower = center - half_extents
    upper = center + half_extents
    mask = np.all((lidar_pc >= lower) & (lidar_pc <= upper), axis=1)
    return lidar_pc[mask]


def backproject_pixel(u, v, K):
    """
    Backprojects pixel (u,v) into a normalized 3D ray (camera coordinates).
    """
    cx, cy = K[0, 2], K[1, 2]
    fx, fy = K[0, 0], K[1, 1]
    x = (u - cx) / fx
    y = (v - cy) / fy
    ray_dir = np.array([x, y, 1.0])
    return ray_dir / np.linalg.norm(ray_dir)


def find_human_center_on_ray(lidar_pc, ray_origin, ray_direction,
                             t_min, t_max, t_step,
                             distance_threshold, min_points, ransac_threshold):
    """
    Sweeps along ray, checks local clusters of points. Returns (refined_center, None, None) if found.
    """
    # Distance from each point to the ray
    vecs = lidar_pc - ray_origin
    proj_lengths = np.dot(vecs, ray_direction)
    proj_points = ray_origin + np.outer(proj_lengths, ray_direction)
    dists_to_ray = np.linalg.norm(lidar_pc - proj_points, axis=1)
    near_ray_mask = dists_to_ray < distance_threshold
    filtered_pc = lidar_pc[near_ray_mask]

    if filtered_pc.shape[0] < min_points:
        return None, None, None

    t_values = np.arange(t_min, t_max, t_step)
    for t in t_values:
        candidate = ray_origin + t * ray_direction
        dists = np.linalg.norm(filtered_pc - candidate, axis=1)
        nearby_points = filtered_pc[dists < distance_threshold]
        if nearby_points.shape[0] >= min_points:
            refined_candidate = np.mean(nearby_points, axis=0)
            return refined_candidate, None, None

    return None, None, None


def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    Use DBSCAN to refine cluster and return the cluster closest to 'center'.
    """
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster


def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove points within z_range of the minimum z (assume they're ground).
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    return cluster[cluster[:, 2] > (min_z + z_range)]


def display_reprojected_cluster(image, refined_cluster, T_l2c, K):

    # 确保 refined_cluster 至少有一个点
    if refined_cluster is None or refined_cluster.shape[0] == 0:
        return image

    # 将 refined_cluster 转换为齐次坐标 (N, 4)
    N = refined_cluster.shape[0]
    pts_hom = np.hstack((refined_cluster, np.ones((N, 1))))

    # 利用 T_l2c 将 LiDAR 坐标转换到相机坐标系
    pts_cam = (T_l2c @ pts_hom.T).T  # (N, 4)
    pts_cam = pts_cam[:, :3]  # 取前三列

    # 遍历每个点，投影到图像平面并绘制蓝色圆点 (BGR: (255,0,0))
    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:  # 只投影前方点
            continue
        u = int((K[0, 0] * X / Z) + K[0, 2])
        v = int((K[1, 1] * Y / Z) + K[1, 2])
        cv2.circle(image, (u, v), 2, (255, 0, 0), -1)

    # 显示带投影点的图像
    cv2.imshow("Reprojected Cluster", image)
    cv2.waitKey(0)
    return image
# -----------------------------
# 3) PedestrianDetector2D
# -----------------------------

class PedestrianDetector2D:
    """
    Detects pedestrians using YOLO (2D bounding boxes) + LiDAR (3D refinement).
    """

    def __init__(self, model_path='../../knowledge/detection/yolov8n.pt'):
        # Load YOLO model
        self.detector = YOLO(model_path)

        # Camera intrinsics and LiDAR-to-camera transforms
        self.T_l2v = np.array([
            [0.99939639, 0.02547917, 0.023615, 1.1],
            [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
            [-0.02379784, 0.00689664, 0.999693, 1.95320223],
            [0., 0., 0., 1.]
        ])
        self.K = np.array([
            [684.83331299, 0.,           573.37109375],
            [0.,           684.60968018, 363.70092773],
            [0.,           0.,           1.]
        ])
        self.T_l2c = np.array([
    [0.001090, -0.999489, -0.031941,  0.149698],
    [-0.007664,  0.031932, -0.999461, -0.397813],
    [0.999970,  0.001334, -0.007625, -0.691405],
    [0.000000,  0.000000,  0.000000,  1.000000]
])
        self.T_c2l = np.linalg.inv(self.T_l2c)
        self.R_c2l = self.T_c2l[:3, :3]
        self.camera_origin_in_lidar = self.T_c2l[:3, 3]

        # Tracking data
        self.tracked_agents = {}
        self.pedestrian_counter = 0

    def process_frame(self, image, lidar_points, current_time=0.0):
        """
        1) Runs YOLO on the image to get 2D bounding boxes for pedestrians.
        2) For each box, uses LiDAR to estimate a 3D location + bounding box.
        3) Returns a dict of AgentState objects keyed by agent_id.
        """
        agents = {}
        # print(raw_lidar_points.shape)
        # start = time.time()
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(raw_lidar_points)
        # # Apply voxel grid downsampling; adjust voxel_size for desired resolution
        # voxel_size = 0.15
        # lidar_points = np.asarray(pcd.voxel_down_sample(voxel_size=voxel_size).points)
        # # Convert back to numpy array
        # print("elapse:", time.time() - start)
        # print(lidar_points.shape)
        # 1) Run YOLO detection on the image
        results = self.detector(image, conf=0.4, classes=[0])  # classes=[0] -> persons
        # YOLO boxes in format [x_center, y_center, w, h]
        boxes = np.array(results[0].boxes.xywh.cpu()) if len(results) > 0 else []

        # 2) For each detection, do 3D fusion with LiDAR
        for box in boxes:
            cx, cy, w, h = box
            # Convert pixel center to a ray in LiDAR frame
            ray_dir_cam = backproject_pixel(cx, cy, self.K)
            ray_dir_lidar = self.R_c2l @ ray_dir_cam
            ray_dir_lidar /= np.linalg.norm(ray_dir_lidar)

            # Find approximate intersection along that ray
            intersection, _, _ = find_human_center_on_ray(
                lidar_points, self.camera_origin_in_lidar, ray_dir_lidar,
                t_min=0.4, t_max=20.0, t_step=0.1,
                distance_threshold=0.3, min_points=5, ransac_threshold=0.05
            )
            if intersection is None:
                continue

            # Estimate physical dimensions from bounding box size & distance
            d = np.linalg.norm(intersection - self.camera_origin_in_lidar)
            physical_width = (w * d) / self.K[0, 0]
            physical_height = (h * d) / self.K[1, 1]

            half_extents = np.array([0.4, 0.4, 1.25 * physical_height / 2])

            # Extract local ROI from LiDAR around that intersection
            roi_points = extract_roi_box(lidar_points, intersection, half_extents)

            # Refine cluster with DBSCAN
            if roi_points.shape[0] < 10:
                refined_cluster = roi_points
            else:
                refined_cluster = refine_cluster(roi_points, intersection, eps=0.15, min_samples=10)

            # Remove ground points
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            # If no points remain, or too few, just skip
            display_reprojected_cluster(image, refined_cluster, self.T_l2c, self.K)

            if refined_cluster is None or refined_cluster.shape[0] == 0:
                refined_center = intersection
                dims = (0, 0, 0)
                yaw = pitch = roll = 0
            elif refined_cluster.shape[0] <= 5:
                continue
            else:
                start = time.time()
                # Use oriented bounding box
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(refined_cluster)
                obb = pcd.get_oriented_bounding_box()
                refined_center = obb.center
                dims = tuple(obb.extent)
                R_lidar = obb.R.copy()

                # Transform center/orientation to vehicle frame
                refined_center_lidar_hom = np.array([refined_center[0],
                                                     refined_center[1],
                                                     refined_center[2],
                                                     1.0])
                refined_center_vehicle_hom = self.T_l2v @ refined_center_lidar_hom
                refined_center_vehicle = refined_center_vehicle_hom[:3]

                R_vehicle = self.T_l2v[:3, :3] @ R_lidar
                euler_angles_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=True)
                yaw, pitch, roll = euler_angles_vehicle
                refined_center = refined_center_vehicle


            # Create the new pose
            new_pose = ObjectPose(
                t=current_time,
                x=refined_center[0],
                y=refined_center[1],
                z=refined_center[2],
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                frame=0  # Just a placeholder
            )

            # Attempt to match with an existing pedestrian (tracking)
            existing_id = match_existing_pedestrian(
                new_center=np.array([new_pose.x, new_pose.y, new_pose.z]),
                new_dims=dims,
                existing_agents=self.tracked_agents,
                distance_threshold=1.0
            )

            if existing_id is not None:
                # Update existing agent
                old_agent_state= self.tracked_agents[existing_id]
                dt = new_pose.t - old_agent_state.pose.t
                vx, vy, vz = compute_velocity(old_agent_state.pose, new_pose, dt)

                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,       # e.g., AgentEnum.PEDESTRIAN
                    activity=0,   # e.g., AgentActivityEnum.MOVING
                    velocity=(vx, vy, vz),
                    yaw_rate=0
                )
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent

            else:
                # Create a new agent
                agent_id = f"pedestrian{self.pedestrian_counter}"
                self.pedestrian_counter += 1

                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,       # e.g., AgentEnum.PEDESTRIAN
                    activity=0,   # e.g., AgentActivityEnum.MOVING
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent
        # print("THISIS", time.time() - start)
        return agents

def load_lidar_from_npz(file_path):

    data = np.load(file_path)

    lidar_points = data['arr_0']

    return lidar_points

def main():
    # -----------------------------
    # Added Visualization Switch and Code
    # -----------------------------
    SHOW_VISUALIZATION = True  # Set to False to disable all visualization

    # Instantiate the detector
    detector = PedestrianDetector2D(model_path='yolov8n.pt')

    # Collect test images & LiDAR files
    # Adjust the patterns if your filenames differ
    image_files = sorted(glob.glob(os.path.join('../data', 'color*.png')))
    lidar_files = sorted(glob.glob(os.path.join('../data', 'lidar*.npz')))

    num_frames = min(len(image_files), len(lidar_files))
    if num_frames == 0:
        print("No matching data found in 'data/' folder.")
        return

    print(f"Found {num_frames} matching frames.")

    # Process each frame
    for i in range(36, 43):
        print(f"\n--- Frame {i+1}/{num_frames} ---")
        image = cv2.imread(f'../data/color{i}.png')
        if image is None:
            print(f"Failed to read {image_files[i]}")
            continue
        try:
            lidar_points = load_lidar_from_npz(f'../data/lidar{i}.npz')
        except Exception as e:
            continue

        # A simple “timestamp” or frame index
        current_time = float(i)

        # Run detection & fusion
        agents = detector.process_frame(image, lidar_points, current_time)

        # Print out the results
        if agents:
            print(f"Detected {len(agents)} agents:")
            for agent_id, agent_state in agents.items():
                p = agent_state.pose
                print(f"  {agent_id}: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
        else:
            print("No agents detected in this frame.")

        # -----------------------------
        # Modified Visualization for Each Frame
        # -----------------------------
        if SHOW_VISUALIZATION:
            # Visualize YOLO bounding boxes on the image
            vis_image = image.copy()
            results = detector.detector(vis_image, conf=0.4, classes=[0])
            boxes = np.array(results[0].boxes.xywh.cpu()) if len(results) > 0 else []
            for box in boxes:
                cx, cy, w, h = box
                top_left = (int(cx - w/2), int(cy - h/2))
                bottom_right = (int(cx + w/2), int(cy + h/2))
                cv2.rectangle(vis_image, top_left, bottom_right, (255, 0, 0), 2)
            cv2.imshow("YOLO Bounding Boxes", vis_image)
            cv2.waitKey(1)

            # Create Open3D geometries for visualization
            geometries = []
            # LiDAR point cloud (gray)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(lidar_points)
            pcd.paint_uniform_color([0.7, 0.7, 0.7])
            geometries.append(pcd)

            # Compute the inverse transform (vehicle->LiDAR) for visualization conversion
            T_v2l = np.linalg.inv(detector.T_l2v)

            # For each detected agent, add visualization elements in LiDAR frame
            for agent_id, agent_state in agents.items():
                # Convert pose from vehicle frame to LiDAR frame
                pose_vehicle = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z, 1])
                center_lidar = (T_v2l @ pose_vehicle)[:3]

                # Add a green sphere at the agent center in LiDAR frame
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
                sphere.translate(center_lidar)
                sphere.paint_uniform_color([0, 1, 0])
                geometries.append(sphere)

                # Add an oriented bounding box if dimensions are valid
                dims = agent_state.dimensions
                if dims != (0, 0, 0):
                    yaw = agent_state.pose.yaw
                    pitch = agent_state.pose.pitch
                    roll = agent_state.pose.roll
                    # Compute the vehicle frame rotation matrix from yaw, pitch, roll
                    R_vehicle = R.from_euler('zyx', [yaw, pitch, roll], degrees=True).as_matrix()
                    # Convert the rotation to LiDAR frame
                    R_lidar = T_v2l[:3, :3] @ R_vehicle
                    obb = o3d.geometry.OrientedBoundingBox(center_lidar, R_lidar, dims)
                    obb.color = (1, 0, 1)  # Magenta
                    geometries.append(obb)

                # Add a ray (yellow line) from the camera origin (LiDAR frame) to the agent center
                camera_origin = detector.camera_origin_in_lidar
                points_line = [camera_origin, center_lidar]
                lines = [[0, 1]]
                colors = [[0.7, 0.5, 0]]
                ray_line = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(points_line),
                    lines=o3d.utility.Vector2iVector(lines)
                )
                ray_line.colors = o3d.utility.Vector3dVector(colors)
                geometries.append(ray_line)

            o3d.visualization.draw_geometries(geometries, window_name=f"Frame {i}")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
