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
def downsample_points(lidar_points, voxel_size=0.15):
    """简单示例：用 Open3D voxel_down_sample 对点云下采样"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points)


def transform_points_l2c(lidar_points, T_l2c):
    """
    将 LiDAR 坐标系下的点 (N,3) 通过 4x4 齐次矩阵 T_l2c 转到相机坐标系。
    返回形状 (N,3) 的 numpy 数组。
    """
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
    return pts_cam[:, :3]


def project_points(pts_cam, K, original_lidar_points):
    """
    Vectorized version.
    pts_cam: (N,3) array of points in camera coordinates.
    original_lidar_points: (N,3) array of points in LiDAR coordinates.
    Returns a (M,5) array: [u, v, X_lidar, Y_lidar, Z_lidar] for all points with Z>0.
    """
    # 只保留 Z>0 的点
    mask = pts_cam[:, 2] > 0
    pts_cam_valid = pts_cam[mask]
    lidar_valid = original_lidar_points[mask]

    # 分别提取 X, Y, Z
    Xc = pts_cam_valid[:, 0]
    Yc = pts_cam_valid[:, 1]
    Zc = pts_cam_valid[:, 2]

    # 根据内参计算 u, v（注意转换为 int）
    u = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(np.int32)
    v = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(np.int32)

    # 将 u, v 与原始 LiDAR 坐标拼接，返回 (M,5) 数组
    proj = np.column_stack((u, v, lidar_valid))
    return proj


def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    对 roi_points (N,3) 进行 DBSCAN 聚类，并返回与 center 最近的簇。
    """
    if roi_points.shape[0] < min_samples:
        return roi_points
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    valid_clusters = [roi_points[labels == l] for l in set(labels) if l != -1]
    if not valid_clusters:
        return roi_points
    best_cluster = min(valid_clusters, key=lambda c: np.linalg.norm(np.mean(c, axis=0) - center))
    return best_cluster


def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    去除离最低 z 值在 z_range 内的点（假设它们是地面）。
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    return cluster[cluster[:, 2] > (min_z + z_range)]


def match_existing_pedestrian(new_center, new_dims, existing_agents, distance_threshold=1.0):
    """
    尝试在 existing_agents 中找一个最近的 agent (欧氏距离<distance_threshold)。
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
    """计算 (vx, vy, vz)"""
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)


def filter_depth_points(lidar_points, max_human_depth=0.9):
    """
    Filter points that are beyond (min_depth + max_human_depth).
    假设点云中第一列为深度（X轴）信息，此处可根据实际情况调整。
    """
    if lidar_points.shape[0] == 0:
        return lidar_points
    # 假设使用第一列作为深度（或根据实际情况选择合适的轴）
    lidar_points_dist = lidar_points[:, 0]
    min_dist = np.min(lidar_points_dist)
    max_possible_dist = min_dist + max_human_depth
    filtered_array = lidar_points[lidar_points_dist < max_possible_dist]
    return filtered_array


def display_reprojected_cluster(image, refined_cluster, T_l2c, K):
    """
    将 refined_cluster（形状为 (N, 3) 的 LiDAR 坐标点）转换到相机坐标系，
    并利用内参矩阵 K 投影到图像平面上，然后在图像上用蓝色点标记。
    """
    if refined_cluster is None or refined_cluster.shape[0] == 0:
        return image

    N = refined_cluster.shape[0]
    pts_hom = np.hstack((refined_cluster, np.ones((N, 1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    pts_cam = pts_cam[:, :3]

    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:
            continue
        u = int(K[0, 0] * X / Z + K[0, 2])
        v = int(K[1, 1] * Y / Z + K[1, 2])
        cv2.circle(image, (u, v), 2, (255, 0, 0), -1)
    return image


# -----------------------------
# 3) PedestrianDetector2D (New Approach)
# -----------------------------
class PedestrianDetector2D:
    """
    用新的思路：
      1) 下采样 LiDAR
      2) 转到相机坐标系
      3) 投影全部点
      4) 根据 YOLO 2D box 在投影点中筛选
      5) DBSCAN / 去地面
      6) OrientedBBox + Vehicle Frame
      7) (可选) 在图像上重投影 refined_cluster
    """

    def __init__(self, model_path='yolov8n.pt'):
        # Load YOLO
        self.detector = YOLO(model_path)

        # LiDAR->Vehicle
        self.T_l2v = np.array([
            [0.99939639, 0.02547917, 0.023615, 1.1],
            [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
            [-0.02379784, 0.00689664, 0.999693, 1.95320223],
            [0, 0, 0, 1]
        ])

        # Camera Intrinsics
        self.K = np.array([
            [684.83331299, 0, 573.37109375],
            [0, 684.60968018, 363.70092773],
            [0, 0, 1]
        ])

        # LiDAR->Camera
        self.T_l2c = np.array([[-0.01909581, -0.9997844 , 0.0081547 , 0.24521313],
                                [ 0.06526397, -0.00938524, -0.9978239 , -0.80389025],
                                [ 0.9976853 , -0.01852205, 0.06542912, -0.6605772 ],
                                [ 0,         0,          0,          1          ]])

        # Tracking
        self.tracked_agents = {}
        self.pedestrian_counter = 0

    def process_frame(self, image, lidar_points, current_time=0.0, debug_reproj=False):
        agents = {}

        # 1) 下采样（可根据需要调整voxel_size）
        step1 = time.time()
        # lidar_down = downsample_points(lidar_points, voxel_size=0.1)
        lidar_down = lidar_points.copy()
        step2 = time.time()
        # 2) LiDAR->Camera
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        step3 = time.time()
        # 3) 投影到图像
        projected_pts = project_points(pts_cam, self.K, lidar_down)
        # projected_pts shape = (N,5): [u, v, X_l, Y_l, Z_l]
        step4 = time.time()

        print('step1: ', step2-step1, 'step2: ', step3-step2, 'step3: ', step4-step3)
        # 4) YOLO检测2D bounding boxes
        results = self.detector(image, conf=0.4, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu()) if len(results) > 0 else []

        # 5) 针对每个bounding box筛选投影点
        for box in boxes:
            cx, cy, w, h = box
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_2d_pts = projected_pts[mask]
            if roi_2d_pts.shape[0] < 5:
                continue

            # 取出 roi 中在 LiDAR 坐标系下的 3D 点，并过滤深度噪声
            points_3d = roi_2d_pts[:, 2:5]
            points_3d = filter_depth_points(points_3d, max_human_depth=0.9)
            display_reprojected_cluster(image, points_3d, self.T_l2c, self.K)

            # 6) 对筛选后的点云进行聚类并去除地面点
            refined_cluster = refine_cluster(points_3d, np.mean(points_3d, axis=0), eps=0.15, min_samples=10)
            refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.03)
            if refined_cluster.shape[0] < 5:
                continue

            # 7) 计算 oriented bounding box
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()  # 在 LiDAR 坐标系下

            # 转换到 Vehicle 坐标系
            refined_center_hom = np.append(refined_center, 1)
            refined_center_vehicle_hom = self.T_l2v @ refined_center_hom
            refined_center_vehicle = refined_center_vehicle_hom[:3]

            R_vehicle = self.T_l2v[:3, :3] @ R_lidar
            euler_vehicle = R.from_matrix(R_vehicle).as_euler('zyx', degrees=True)
            yaw, pitch, roll = euler_vehicle

            new_pose = ObjectPose(
                t=current_time,
                x=refined_center_vehicle[0],
                y=refined_center_vehicle[1],
                z=refined_center_vehicle[2],
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                frame=0
            )

            if debug_reproj:
                display_reprojected_cluster(image, refined_cluster, self.T_l2c, self.K)

            # 8) 跟踪匹配
            existing_id = match_existing_pedestrian(
                np.array([new_pose.x, new_pose.y, new_pose.z]),
                dims, self.tracked_agents, distance_threshold=1.0
            )
            if existing_id is not None:
                old_state = self.tracked_agents[existing_id]
                dt = current_time - old_state.pose.t
                vx, vy, vz = compute_velocity(old_state.pose, new_pose, dt)
                updated_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,
                    activity=0,
                    velocity=(vx, vy, vz),
                    yaw_rate=0
                )
                agents[existing_id] = updated_agent
                self.tracked_agents[existing_id] = updated_agent
            else:
                agent_id = f"pedestrian{self.pedestrian_counter}"
                self.pedestrian_counter += 1
                new_agent = AgentState(
                    pose=new_pose,
                    dimensions=dims,
                    outline=None,
                    type=0,
                    activity=0,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent

        return agents


def load_lidar_from_npz(file_path):
    data = np.load(file_path)
    return data['arr_0']


# -----------------------------
# 4) 主函数: 演示如何调用
# -----------------------------
def main():
    SHOW_VISUALIZATION = True
    detector = PedestrianDetector2D(model_path='yolov8n.pt')

    image_files = sorted(glob.glob(os.path.join('../data', 'color*.png')))
    lidar_files = sorted(glob.glob(os.path.join('../data', 'lidar*.npz')))
    num_frames = min(len(image_files), len(lidar_files))
    if num_frames == 0:
        print("No matching data found.")
        return

    print(f"Found {num_frames} matching frames.")

    for i in range(36, 43):
        print(f"\n--- Frame {i + 1}/{num_frames} ---")
        image_path = image_files[i]
        lidar_path = lidar_files[i]

        image = cv2.imread(f'../data/color{i}.png')
        if image is None:
            print(f"Failed to read image: {image_path}")
            continue
        lidar_points = load_lidar_from_npz(f'../data/lidar{i}.npz')

        agents = detector.process_frame(image, lidar_points, current_time=float(i), debug_reproj=True)

        if agents:
            print(f"Detected {len(agents)} agents:")
            for agent_id, agent_state in agents.items():
                p = agent_state.pose
                print(f"  {agent_id}: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
        else:
            print("No agents detected.")

        # 在图像上显示 YOLO 2D 边界框及蓝色点（已在 process_frame 内显示）
        cv2.imshow("YOLO + Blue cluster points", image)
        cv2.waitKey(1)

        if SHOW_VISUALIZATION:
            # 构造 LiDAR 点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(lidar_points)
            pcd.paint_uniform_color([0.7, 0.7, 0.7])
            geometries = [pcd]

            # 计算 Vehicle->LiDAR 的逆变换，用于将检测结果从 Vehicle 坐标系转换回 LiDAR 坐标系
            T_v2l = np.linalg.inv(detector.T_l2v)

            # 对每个检测到的 agent，添加中心点和 oriented bounding box到几何体列表中
            for agent_id, agent_state in agents.items():
                # 将 Vehicle 坐标下的目标中心转换到 LiDAR 坐标系
                pose_vehicle = np.array([agent_state.pose.x, agent_state.pose.y, agent_state.pose.z, 1])
                center_lidar = (T_v2l @ pose_vehicle)[:3]

                # 添加一个绿色小球表示目标中心
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
                sphere.translate(center_lidar)
                sphere.paint_uniform_color([0, 1, 0])
                geometries.append(sphere)

                dims = agent_state.dimensions
                if dims != (0, 0, 0):
                    yaw = agent_state.pose.yaw
                    pitch = agent_state.pose.pitch
                    roll = agent_state.pose.roll
                    # 计算 Vehicle 坐标系下的旋转矩阵
                    R_vehicle = R.from_euler('zyx', [yaw, pitch, roll], degrees=True).as_matrix()
                    # 将旋转矩阵转换到 LiDAR 坐标系
                    R_lidar = T_v2l[:3, :3] @ R_vehicle
                    obb = o3d.geometry.OrientedBoundingBox(center_lidar, R_lidar, dims)
                    obb.color = (1, 0, 1)  # 紫红色
                    geometries.append(obb)

            o3d.visualization.draw_geometries(geometries, window_name=f"Frame {i}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
