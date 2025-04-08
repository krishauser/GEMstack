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
def undistort_image(image, K, D):
    """对未校正的图像进行畸变校正，并返回校正后的图像及新的内参矩阵"""
    h, w = image.shape[:2]
    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, D, None, newK)
    return undistorted, newK

def downsample_points(lidar_points, voxel_size=0.15):
    """利用 Open3D voxel_down_sample 对点云下采样"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points)

def transform_points_l2c(lidar_points, T_l2c):
    """
    将 LiDAR 坐标系下的点 (N,3) 通过 4x4 齐次矩阵 T_l2c 转到相机坐标系，
    返回形状 (N,3) 的 numpy 数组。
    """
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))  # (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T  # (N,4)
    return pts_cam[:, :3]

def project_points(pts_cam, K, original_lidar_points):
    """
    将相机坐标系中的点投影到图像平面上，返回 (M,5) 数组 [u, v, X_lidar, Y_lidar, Z_lidar]，
    只保留 Z>0 的点。
    """
    mask = pts_cam[:, 2] > 0
    pts_cam_valid = pts_cam[mask]
    lidar_valid = original_lidar_points[mask]

    Xc = pts_cam_valid[:, 0]
    Yc = pts_cam_valid[:, 1]
    Zc = pts_cam_valid[:, 2]

    u = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(np.int32)
    v = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(np.int32)

    proj = np.column_stack((u, v, lidar_valid))
    return proj

def visualize_custom_points(image, points_lidar, T_l2c, K, color=(0, 255, 0)):
    """
    将 LiDAR 坐标系下的 3D 点投影到图像上，并绘制成圆点。

    :param image:  (H, W, 3) 已经过畸变校正的图像
    :param points_lidar: (N, 3) LiDAR 坐标系下的 3D 点数组
    :param T_l2c:  (4, 4) LiDAR->Camera 转换矩阵
    :param K:      (3, 3) 当前内参矩阵
    :param color:  BGR 颜色元组
    :return:       绘制了投影圆点的图像
    """
    N = points_lidar.shape[0]
    pts_hom = np.hstack((points_lidar, np.ones((N, 1))))  # (N,4)
    pts_cam = (T_l2c @ pts_hom.T).T[:, :3]

    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:
            continue
        u = int(K[0, 0] * (X / Z) + K[0, 2])
        v = int(K[1, 1] * (Y / Z) + K[1, 2])
        cv2.circle(image, (u, v), 5, color, -1)
    return image

def filter_points_within_threshold(points, threshold=15.0):
    """
    筛选出所有距离传感器小于或等于 threshold 米的点。

    参数：
      points: numpy 数组，形状为 (N,3)，每一行为一个三维点 [x, y, z]。
      threshold: 距离阈值（单位：米），默认值为 15.0 米。

    返回：
      过滤后的点云数据（仅保留距离小于等于 threshold 米的点）。
    """
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= threshold
    return points[mask]

def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    对 roi_points (N,3) 进行 DBSCAN 聚类，并返回与 center 最近的簇
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
    去除离最低 z 值在 z_range 内的点（假设它们是地面）
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    return cluster[cluster[:, 2] > (min_z + z_range)]

def match_existing_pedestrian(new_center, new_dims, existing_agents, distance_threshold=1.0):
    """
    尝试在 existing_agents 中找一个最近的 agent（欧氏距离 < distance_threshold）
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

def filter_depth_points(lidar_points, max_depth_diff=0.9, use_norm=True):
    """
    过滤超过 (min_depth + max_depth_diff) 范围之外的点。

    如果 use_norm 为 True，则按照每个点的欧氏距离（norm）来计算深度，
    否则默认使用第一列（x轴）作为深度。

    参数：
      lidar_points: (N, 3) 的点云数据
      max_depth_diff: 最小深度加上此阈值作为允许的深度范围
      use_norm: 是否使用欧氏距离作为深度（True）或直接使用 x 坐标（False）

    返回：
      筛选后的点云数据
    """
    if lidar_points.shape[0] == 0:
        return lidar_points

    if use_norm:
        depths = np.linalg.norm(lidar_points, axis=1)
    else:
        depths = lidar_points[:, 0]

    min_depth = np.min(depths)
    max_possible_depth = min_depth + max_depth_diff
    mask = depths < max_possible_depth
    return lidar_points[mask]

def display_reprojected_cluster(image, refined_cluster, T_l2c, K):
    """
    将 refined_cluster（形状为 (N,3) 的 LiDAR 坐标点）转换到相机坐标系，
    利用内参矩阵 K 投影到图像平面上，并在图像上用蓝色点标记
    """
    if refined_cluster is None or refined_cluster.shape[0] == 0:
        return image

    N = refined_cluster.shape[0]
    pts_hom = np.hstack((refined_cluster, np.ones((N, 1))))
    pts_cam = (T_l2c @ pts_hom.T).T[:, :3]

    for p in pts_cam:
        X, Y, Z = p
        if Z <= 0:
            continue
        u = int(K[0, 0] * X / Z + K[0, 2])
        v = int(K[1, 1] * Y / Z + K[1, 2])
        cv2.circle(image, (u, v), 2, (255, 0, 0), -1)
    return image

def pose_to_matrix(pose):
    """
    Compose a 4x4 transformation matrix from a pose state.
    Assumes pose has attributes: x, y, z, yaw, pitch, roll,
    where the angles are given in degrees.
    """
    x = pose.x if pose.x is not None else 0.0
    y = pose.y if pose.y is not None else 0.0
    z = pose.z if pose.z is not None else 0.0
    if pose.yaw is not None and pose.pitch is not None and pose.roll is not None:
        yaw = np.radians(pose.yaw)
        pitch = np.radians(pose.pitch)
        roll = np.radians(pose.roll)
    else:
        yaw = pitch = roll = 0.0
    R_mat = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = np.array([x, y, z])
    return T

# ----- New: Vectorized projection function -----
def project_points(pts_cam, K, original_lidar_points):
    """
    Vectorized version.
    pts_cam: (N,3) array of points in camera coordinates.
    original_lidar_points: (N,3) array of points in LiDAR coordinates.
    Returns a (M,5) array: [u, v, X_lidar, Y_lidar, Z_lidar] for all points with Z>0.
    """
    mask = pts_cam[:, 2] > 0
    pts_cam_valid = pts_cam[mask]
    lidar_valid = original_lidar_points[mask]
    Xc = pts_cam_valid[:, 0]
    Yc = pts_cam_valid[:, 1]
    Zc = pts_cam_valid[:, 2]
    u = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(np.int32)
    v = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(np.int32)
    proj = np.column_stack((u, v, lidar_valid))
    return proj

# -----------------------------
# 3) PedestrianDetector2D (新方法)
# -----------------------------
class PedestrianDetector2D:
    """
    新思路：
      1) 下采样 LiDAR 点云
      2) 转换到相机坐标系
      3) 投影所有点到图像平面
      4) 根据 YOLO 2D 检测框筛选对应点
      5) DBSCAN 聚类与去除地面点
      6) 计算有方向的边界框并转换到 Vehicle 坐标系
      7) (可选) 在图像上重投影 cluster 点，调试转换
      8) (可选) 可视化 3D frustum（反投影 2D 边界框）
    """
    def __init__(self, model_path='yolov8n.pt'):
        # 加载 YOLO 模型
        self.detector = YOLO(model_path)
        self.camera_front = False

        if self.camera_front:
            self.K = np.array([[684.83331299, 0., 573.37109375],
                               [0., 684.60968018, 363.70092773],
                               [0., 0., 1.]])
        else:
            self.K = np.array([[1230.144096, 0., 978.828508],
                               [0., 1230.630424, 605.794034],
                               [0., 0., 1.]])
        if self.camera_front:
            self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.D = np.array([-0.23751890570984993, 0.08452214195986749, -0.00035324203850054794, -0.0003762498910536819, 0.0])

        self.T_l2v = np.array([[0.99939639, 0.02547917, 0.023615, 1.1],
                                [-0.02530848, 0.99965156, -0.00749882, 0.03773583],
                                [-0.02379784, 0.00689664, 0.999693, 1.95320223],
                                [0., 0., 0., 1.]])
        if self.camera_front:
            self.T_l2c = np.array([
                [0.001090, -0.999489, -0.031941, 0.149698],
                [-0.007664, 0.031932, -0.999461, -0.397813],
                [0.999970, 0.001334, -0.007625, -0.691405],
                [0., 0., 0., 1.000000]
            ])
        else:
            self.T_l2c = np.array([[0.71082304, -0.70305212, -0.02608284, 0.17771596],
                                    [-0.13651802, -0.10076507, -0.98505595, -0.56321222],
                                    [0.68915595, 0.70388118, -0.1678969, -0.62027912],
                                    [0., 0., 0., 1.]])
        self.tracked_agents = {}
        self.pedestrian_counter = 0
        self.current_K = self.K.copy()
        self.debug_frustums = []
        # 存储每个 refined cluster 的点云，用于 Open3D 可视化上色
        self.cluster_geometries = []

    def process_frame(self, image, lidar_points, current_time=0.0, debug_reproj=False, debug_frustum=False):
        agents = {}
        self.cluster_geometries = []  # 重置 refined cluster 的集合

        image, newK = undistort_image(image, self.K, self.D)
        self.current_K = newK

        lidar_down = lidar_points.copy()

        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)

        results = self.detector(image, conf=0.4, classes=[0])
        boxes = np.array(results[0].boxes.xywh.cpu()) if len(results) > 0 else []

        for box in boxes:
            cx, cy, w, h = box
            left = int(cx - w/2)
            right = int(cx + w/2)
            top = int(cy - h/2)
            bottom = int(cy + h/2)
            cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 2)

        if debug_frustum:
            self.debug_frustums = []
        for box in boxes:
            cx, cy, w, h = box
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            if debug_frustum:
                bbox = [left, top, right, bottom]
                z_near = 0.5
                z_far = 10.0
                pts_frustum, lines_frustum = self.create_frustum_lines(bbox, self.current_K, z_near, z_far)
                frustum_lineset = o3d.geometry.LineSet()
                frustum_lineset.points = o3d.utility.Vector3dVector(pts_frustum)
                frustum_lineset.lines = o3d.utility.Vector2iVector(lines_frustum)
                frustum_lineset.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines_frustum))])
                self.debug_frustums.append(frustum_lineset)

            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_2d_pts = projected_pts[mask]

            points_3d = roi_2d_pts[:, 2:5]
            if debug_reproj:
                display_reprojected_cluster(image, points_3d, self.T_l2c, self.current_K)
            start = time.time()
            points_3d = filter_points_within_threshold(points_3d, 15)
            end = time.time()
            print(start-end)
            # points_3d = remove_ground_by_min_range(points_3d, z_range=0.01)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.3)
            # refined_cluster = refine_cluster(points_3d, np.mean(points_3d, axis=0), eps=0.15, min_samples=5)
            refined_cluster = remove_ground_by_min_range(points_3d, z_range=0.01)
            if refined_cluster.shape[0] < 4:
                continue

            # 创建 refined cluster 的点云并上色为红色
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(refined_cluster)
            pcd_cluster.paint_uniform_color([1, 0, 0])
            self.cluster_geometries.append(pcd_cluster)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            refined_center = obb.center
            dims = tuple(obb.extent)
            R_lidar = obb.R.copy()

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

            existing_id = match_existing_pedestrian(
                np.array([new_pose.x, new_pose.y, new_pose.z]),
                dims,
                self.tracked_agents,
                distance_threshold=1.0
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

        # 绘制 hard-code 的 3D 点（红色圆点）到图像上
        my_points_lidar = np.array([
            [4.730639, 10.195478, -1.941095],  # Point #189070
            [2.066050, 7.066111, -2.027844],   # Point #223837
            [1.341566, 7.118239, -1.994539],   # Point #223805
            [4.160140, 6.539731, -1.895874],   # Point #213689
            [6.469934, 6.128805, -1.859030],   # Point #201481
        ])
        visualize_custom_points(image, my_points_lidar, self.T_l2c, self.current_K, color=(0, 0, 255))

        cv2.imshow("Frame with Boxes and Custom Points", image)
        cv2.waitKey(1)
        return agents

    def create_frustum_lines(self, box, K, z_near, z_far):
        """
        输入 2D 边界框 [left, top, right, bottom]，利用相机内参 K，
        在近处和远处分别反投影出 4 个角点，返回所有 3D 点及其对应连线。
        """
        left, top, right, bottom = box
        corners = np.array([
            [left, top],
            [right, top],
            [right, bottom],
            [left, bottom]
        ])

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        near_points = []
        far_points = []
        for (u, v) in corners:
            x_near = (u - cx) / fx * z_near
            y_near = (v - cy) / fy * z_near
            near_pt = np.array([x_near, y_near, z_near, 1.0])
            near_points.append(near_pt)
            x_far = (u - cx) / fx * z_far
            y_far = (v - cy) / fy * z_far
            far_pt = np.array([x_far, y_far, z_far, 1.0])
            far_points.append(far_pt)

        near_points = np.array(near_points)
        far_points = np.array(far_points)
        T_c2l = np.linalg.inv(self.T_l2c)
        near_points_lidar = (T_c2l @ near_points.T).T[:, :3]
        far_points_lidar = (T_c2l @ far_points.T).T[:, :3]
        all_points = np.vstack((near_points_lidar, far_points_lidar))
        lines = [[i, i + 4] for i in range(4)]
        lines += [[0, 1], [1, 2], [2, 3], [3, 0]]
        lines += [[4, 5], [5, 6], [6, 7], [7, 4]]
        return all_points, lines

# -----------------------------
# 4) 加载 LiDAR 数据函数
# -----------------------------
def load_lidar_from_npz(file_path):
    data = np.load(file_path)
    return data['arr_0']

# -----------------------------
# 5) 主函数：调用 process_frame，并只显示 refined cluster 中的点（不显示其他几何体）
# -----------------------------
def main():
    SHOW_VISUALIZATION = True
    detector = PedestrianDetector2D(model_path='cone.pt')

    image_files = sorted(glob.glob(os.path.join('../data', 'color*.png')))
    lidar_files = sorted(glob.glob(os.path.join('../data', 'lidar*.npz')))
    num_frames = min(len(image_files), len(lidar_files))
    if num_frames == 0:
        print("No matching data found.")
        return

    print(f"Found {num_frames} matching frames.")
    for i in range(95, 96):
        print(f"\n--- Frame {i+1}/{num_frames} ---")
        image = cv2.imread(f'../parking_data/camera_fl{i}.png')
        lidar_points = load_lidar_from_npz(f'../parking_data/lidar_top{i}.npz')
        agents = detector.process_frame(image, lidar_points, current_time=float(i),
                                        debug_reproj=True, debug_frustum=True)

        # Print all agent information
        print("Detected Agents:")
        for agent_id, agent_state in agents.items():
            p = agent_state.pose
            print(f"Agent {agent_id}: Position=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                  f"Yaw={p.yaw:.2f}, Pitch={p.pitch:.2f}, Roll={p.roll:.2f}, "
                  f"Dimensions={agent_state.dimensions}, Velocity={agent_state.velocity}")

        # 在 Open3D 的可视化中，仅显示 refined cluster 点云（上色为红色）
        if SHOW_VISUALIZATION:
            geometries = detector.cluster_geometries if detector.cluster_geometries else []
            o3d.visualization.draw_geometries(geometries, window_name=f"Frame {i}")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
