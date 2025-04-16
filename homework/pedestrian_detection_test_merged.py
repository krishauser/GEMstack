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


class AgentActivityEnum():
    STOPPED = 0         # standing pedestrians, parked cars, etc. No need to predict motion.
    MOVING = 1          # standard motion.  Predictions will be used here
    FAST = 2            # indicates faster than usual motion, e.g., runners.
    UNDETERMINED = 3    # unknown activity
    STANDING = 4
    LEFT = 5
    RIGHT = 6

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
    h, w = image.shape[:2]
    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, D, None, newK)
    return undistorted, newK

def cylindrical_roi(points, center, radius, height):
    """
    从点云 points 中筛选出位于圆柱体 ROI 内的点。

    其中圆柱体在水平面上以 center[:2] 为圆心（假设 x,y 为水平坐标），
    半径为 radius，垂直方向上以 center[2] 为中点，上下各延伸 height/2 米。
    """
    horizontal_dist = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    vertical_diff = np.abs(points[:, 2] - center[2])
    mask = (horizontal_dist <= radius) & (vertical_diff <= height / 2)
    return points[mask]

def downsample_points(lidar_points, voxel_size=0.15):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points)

def transform_points_l2c(lidar_points, T_l2c):
    N = lidar_points.shape[0]
    pts_hom = np.hstack((lidar_points, np.ones((N, 1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    return pts_cam[:, :3]

def project_points(pts_cam, K, original_lidar_points):
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

def project_all_points(points, T_l2c, K):
    """
    对所有点计算投影，返回形状 (N,2) 的数组 [u, v]，
    对于 Z<=0 的点，返回 [-1, -1]。
    """
    N = points.shape[0]
    pts_hom = np.hstack((points, np.ones((N,1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    u = np.full((N,), -1, dtype=int)
    v = np.full((N,), -1, dtype=int)
    valid = pts_cam[:, 2] > 0
    if np.any(valid):
        Xc = pts_cam[valid, 0]
        Yc = pts_cam[valid, 1]
        Zc = pts_cam[valid, 2]
        u[valid] = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(int)
        v[valid] = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(int)
    return np.stack((u, v), axis=1)

def visualize_custom_points(image, points_lidar, T_l2c, K, color=(0, 255, 0)):
    N = points_lidar.shape[0]
    pts_hom = np.hstack((points_lidar, np.ones((N, 1))))
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
    distances = np.linalg.norm(points, axis=1)
    mask = distances <= threshold
    return points[mask]

def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
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
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    return cluster[cluster[:, 2] > (min_z + z_range)]

def match_existing_pedestrian(new_center, new_dims, existing_agents, distance_threshold=1.0):
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
    if dt <= 0:
        return (0, 0, 0)
    vx = (new_pose.x - old_pose.x) / dt
    vy = (new_pose.y - old_pose.y) / dt
    vz = (new_pose.z - old_pose.z) / dt
    return (vx, vy, vz)

def filter_depth_points(lidar_points, max_depth_diff=0.9, use_norm=True):
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

def project_points(pts_cam, K, original_lidar_points):
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

def remove_rows(A, B, decimals=3):
    A_round = np.round(A, decimals=decimals)
    B_round = np.round(B, decimals=decimals)
    A_view = {tuple(row) for row in A_round}
    B_view = {tuple(row) for row in B_round}
    diff = A_view - B_view
    diff = np.array(list(diff))
    return diff

# -----------------------------
# 3) PedestrianDetector2D (New Method)
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
        self.detector = YOLO(model_path)
        self.camera_front = True
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
        self.cluster_geometries = []
        self.cluster_obbs = []
        self.others_geometry = None
        # flag：是否启用圆柱体 ROI 功能
        self.use_cyl_roi = True

    def process_frame(self, image, lidar_points, current_time=0.0, debug_reproj=False, debug_frustum=False):
        agents = {}
        self.cluster_geometries = []
        self.cluster_obbs = []
        self.debug_frustums = []

        # 1. 对图像进行去畸变，并记录原图尺寸
        image, newK = undistort_image(image, self.K, self.D)
        self.current_K = newK
        orig_H, orig_W = image.shape[:2]

        # 2. 生成左右旋转图像用于检测（仅用于检测，不用于最终显示）
        image_left = cv2.rotate(image.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        image_right = cv2.rotate(image.copy(), cv2.ROTATE_90_CLOCKWISE)

        # 3. 分别调用 YOLO 进行检测（检测返回值 box 格式为 [cx, cy, w, h]）
        results_normal = self.detector(image, conf=0.5, classes=[0])
        results_left = self.detector(image_left, conf=0.5, classes=[0])
        results_right = self.detector(image_right, conf=0.5, classes=[0])

        boxes_normal = np.array(results_normal[0].boxes.xywh.cpu()) if len(results_normal) > 0 else []
        boxes_left = np.array(results_left[0].boxes.xywh.cpu()) if len(results_left) > 0 else []
        boxes_right = np.array(results_right[0].boxes.xywh.cpu()) if len(results_right) > 0 else []

        # 3.1 将来自不同视角的检测结果合并到一起，每个 box 附带对应的 activity 标签
        # 【注意映射公式说明】：
        # (1) 正常图像的检测结果：不做映射，activity = STANDING
        # (2) 左旋图像（逆时针90°）：
        #     在左旋图像中，检测结果的中心为 (c_x, c_y)；
        #     映射到原图坐标应满足：原图点 (x, y) 经过逆时针90°旋转得到 (c_x, c_y)，
        #     公式： (c_x, c_y) = (y, orig_W-1 - x)  → 逆映射： (x, y) = (orig_W-1 - c_y, c_x)
        #     同时，box 的宽高要互换：new_w = h, new_h = w，activity = RIGHT。
        # (3) 右旋图像（顺时针90°）：
        #     在右旋图像中，检测结果的中心为 (c_x, c_y)；
        #     原图点 (x, y) 经过顺时针90°旋转为 (c_x, c_y)：
        #     公式： (c_x, c_y) = (orig_H-1 - y, x)  → 逆映射： (x, y) = (c_y, orig_H-1 - c_x)
        #     同时，宽高互换：new_w = h, new_h = w，activity = LEFT。
        combined_boxes = []
        for box in boxes_normal:
            cx, cy, w, h = box
            combined_boxes.append((cx, cy, w, h, AgentActivityEnum.STANDING))
        for box in boxes_left:
            cx, cy, w, h = box
            new_cx = orig_W - 1 - cy
            new_cy = cx
            new_w = h  # 互换宽高
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.RIGHT))
        for box in boxes_right:
            cx, cy, w, h = box
            new_cx = cy
            new_cy = orig_H - 1 - cx
            new_w = h  # 互换宽高
            new_h = w
            combined_boxes.append((new_cx, new_cy, new_w, new_h, AgentActivityEnum.LEFT))

        # 4. 对 LiDAR 点云进行预处理
        lidar_down = lidar_points.copy()
        global_filtered = filter_points_within_threshold(lidar_down, 20)
        pts_cam = transform_points_l2c(lidar_down, self.T_l2c)
        projected_pts = project_points(pts_cam, self.current_K, lidar_down)

        # 5. 遍历每个检测 ROI，结合 2D 框与 LiDAR 点云进行 3D 定位与聚类
        refined_all = []
        for box_info in combined_boxes:
            cx, cy, w, h, activity = box_info
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)

            # ---【新增】投影 2D Bounding Box 到 3D：构造 3D frustum ---
            if debug_frustum:
                # 使用检测框构造 3D Frustum，设置 z_near 与 z_far（可调整）
                bbox = [left, top, right, bottom]
                z_near = 0.5
                z_far = 15.0
                pts_frustum, lines_frustum = self.create_frustum_lines(bbox, self.current_K, z_near, z_far)
                frustum_lineset = o3d.geometry.LineSet()
                frustum_lineset.points = o3d.utility.Vector3dVector(pts_frustum)
                frustum_lineset.lines = o3d.utility.Vector2iVector(lines_frustum)
                # 所有线条统一用红色显示
                frustum_lineset.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines_frustum))])
                self.debug_frustums.append(frustum_lineset)
            # --------------------------------------------------------

            # 在原图上绘制检测框及标签，颜色根据 activity 区分
            if activity == AgentActivityEnum.STANDING:
                color = (255, 0, 0)
                label_text = "STANDING"
            elif activity == AgentActivityEnum.RIGHT:
                color = (0, 255, 0)
                label_text = "RIGHT"
            elif activity == AgentActivityEnum.LEFT:
                color = (0, 0, 255)
                label_text = "LEFT"
            else:
                color = (255, 255, 255)
                label_text = "UNKNOWN"
            cv2.rectangle(image, (left, top), (right, bottom), color, 2)
            cv2.putText(image, label_text, (left, max(top - 5, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 根据 ROI，从投影后的点云中提取 2D 区域对应的 3D 点
            mask = (projected_pts[:, 0] >= left) & (projected_pts[:, 0] <= right) & \
                   (projected_pts[:, 1] >= top) & (projected_pts[:, 1] <= bottom)
            roi_2d_pts = projected_pts[mask]
            points_3d = roi_2d_pts[:, 2:5]  # 取出原始 3D 坐标

            if debug_reproj:
                display_reprojected_cluster(image, points_3d, self.T_l2c, self.current_K)

            points_3d = filter_points_within_threshold(points_3d, 15)
            points_3d = filter_depth_points(points_3d, max_depth_diff=0.3)
            if points_3d.shape[0] < 4:
                continue

            # 对 ROI 内的点进一步处理：若启用圆柱体 ROI，则利用全局过滤结果构造圆柱体 ROI
            center_roi = np.mean(points_3d, axis=0)
            if self.use_cyl_roi:
                roi_cyl = cylindrical_roi(global_filtered, center_roi, radius=0.3, height=1.2)
                refined_cluster = remove_ground_by_min_range(roi_cyl, z_range=0.01)
                refined_cluster = filter_depth_points(refined_cluster, max_depth_diff=0.2)
                center_roi = np.mean(refined_cluster, axis=0)
            else:
                refined_cluster = remove_ground_by_min_range(points_3d, z_range=0.05)
            if refined_cluster.shape[0] < 4:
                continue

            refined_all.append(refined_cluster)
            # 用 Open3D 创建红色点云表示 refined cluster
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(refined_cluster)
            pcd_cluster.paint_uniform_color([1, 0, 0])
            self.cluster_geometries.append(pcd_cluster)
            # 计算 refined cluster 的定向包围盒（OBB）并设置为紫红色
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd.get_oriented_bounding_box()
            obb.color = (1, 0, 1)
            self.cluster_obbs.append(obb)

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
            # 如果检测目标与已跟踪目标匹配，则更新，否则新建
            existing_id = match_existing_pedestrian(
                np.array([new_pose.x, new_pose.y, new_pose.z]),
                dims,
                self.tracked_agents,
                distance_threshold=0.5
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
                    activity=activity,
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
                    activity=activity,
                    velocity=(0, 0, 0),
                    yaw_rate=0
                )
                agents[agent_id] = new_agent
                self.tracked_agents[agent_id] = new_agent

        # 在图像上显示一些自定义调试点
        visualize_custom_points(image, np.array([
            [4.730639, 10.195478, -1.941095],
            [2.066050, 7.066111, -2.027844],
            [1.341566, 7.118239, -1.994539],
            [4.160140, 6.539731, -1.895874],
            [6.469934, 6.128805, -1.859030],
        ]), self.T_l2c, self.current_K, color=(0, 0, 255))
        cv2.imshow("Frame with Boxes and Custom Points", image)
        cv2.waitKey(1)

        # 处理“其他点”：从 global_filtered 中找出不落在任一 ROI 内的点
        projected_global = project_all_points(global_filtered, self.T_l2c, self.current_K)
        others_mask = np.zeros(len(global_filtered), dtype=bool)
        for box_info in combined_boxes:
            cx, cy, w, h, _ = box_info
            left = int(cx - w / 2)
            right = int(cx + w / 2)
            top = int(cy - h / 2)
            bottom = int(cy + h / 2)
            in_box = (projected_global[:, 0] >= left) & (projected_global[:, 0] <= right) & \
                     (projected_global[:, 1] >= top) & (projected_global[:, 1] <= bottom)
            others_mask = others_mask | in_box
        others_mask = ~others_mask
        others = global_filtered[others_mask]
        others_pc = o3d.geometry.PointCloud()
        others_pc.points = o3d.utility.Vector3dVector(others)
        others_pc.paint_uniform_color([0.5, 0.5, 0.5])
        self.others_geometry = others_pc

        # 返回最终生成的所有检测目标 agents 与在原图上绘制了 2D bounding box 和文本标签的图像
        return agents, image

    def create_frustum_lines(self, box, K, z_near, z_far):
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

# 新增：辅助函数，确保对所有 global_filtered 点计算投影，返回与输入长度相同
def project_all_points(points, T_l2c, K):
    N = points.shape[0]
    pts_hom = np.hstack((points, np.ones((N,1))))
    pts_cam = (T_l2c @ pts_hom.T).T
    u = np.full((N,), -1, dtype=int)
    v = np.full((N,), -1, dtype=int)
    valid = pts_cam[:, 2] > 0
    if np.any(valid):
        Xc = pts_cam[valid, 0]
        Yc = pts_cam[valid, 1]
        Zc = pts_cam[valid, 2]
        u[valid] = (K[0, 0] * (Xc / Zc) + K[0, 2]).astype(int)
        v[valid] = (K[1, 1] * (Yc / Zc) + K[1, 2]).astype(int)
    return np.stack((u, v), axis=1)

# -----------------------------
# 4) 加载 LiDAR 数据函数
# -----------------------------
def load_lidar_from_npz(file_path):
    data = np.load(file_path)
    print(data.keys())
    return data['lidar_points']

# -----------------------------
# 5) 主函数：调用 process_frame，并显示 refined cluster 的点云、OBB 与其他点，
#    每次运行时根据 flag use_cyl_roi 切换使用圆柱体 ROI，
#    并记录上次使用的视角参数。
# -----------------------------
def main():
    # 设置是否使用圆柱体 ROI 过滤
    use_cyl_roi_flag = False
    detector = PedestrianDetector2D(model_path='cone.pt')
    detector.use_cyl_roi = use_cyl_roi_flag

    # 初始化 Open3D 可视化（用于显示 lidar 点云相关的几何体）
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="3D Frame Display", width=800, height=600)
    view_path = "last_view.json"
    if os.path.exists(view_path):
        param = o3d.io.read_pinhole_camera_parameters(view_path)
    else:
        param = None

    # 示例读取图像与 lidar 数据（请根据实际数据路径修改）
    image_files = sorted(glob.glob(os.path.join('../data', 'color*.png')))
    lidar_files = sorted(glob.glob(os.path.join('../data', 'lidar*.npz')))
    num_frames = min(len(image_files), len(lidar_files))
    if num_frames == 0:
        print("No matching data found.")
        return

    print(f"Found {num_frames} matching frames.")
    for i in range(1):
        # 此处使用固定文件作为示例，请按实际情况修改帧索引与路径
        image_path = f'../perception_4.9/image_1744216667.538.png'
        lidar_path = f'../perception_4.9/lidar_1744216667.538.npz'
        image = cv2.imread(image_path)
        if image is None:
            print("Failed to load image:", image_path)
            continue
        lidar_points = load_lidar_from_npz(lidar_path)

        agents, processed_image = detector.process_frame(image, lidar_points, current_time=float(i),
                                                         debug_reproj=True, debug_frustum=True)
        # 输出检测到的 agents 信息
        print("Detected Agents:")
        for agent_id, agent_state in agents.items():
            p = agent_state.pose
            orientation = {AgentActivityEnum.STANDING: "STANDING",
                           AgentActivityEnum.LEFT: "LEFT",
                           AgentActivityEnum.RIGHT: "RIGHT"}.get(agent_state.activity, "UNKNOWN")
            print(
                f"Agent {agent_id}: Position=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), Orientation={orientation}, Dimensions={agent_state.dimensions}")

        # 显示标注了所有 bounding box 与朝向文字的 normal2D 图像
        cv2.imshow("Detection - Normal2D", processed_image)
        cv2.waitKey(1)

        # 以下为 Open3D 点云可视化（保留原代码，可按需要进行扩展）
        geometries = []
        if detector.cluster_geometries:
            geometries.extend(detector.cluster_geometries)
        if detector.cluster_obbs:
            geometries.extend(detector.cluster_obbs)
        if detector.others_geometry is not None:
            geometries.append(detector.others_geometry)
        if detector.debug_frustums:
            geometries.extend(detector.debug_frustums)

        vis.clear_geometries()
        for geom in geometries:
            vis.add_geometry(geom)
        if param is not None:
            vis.get_view_control().convert_from_pinhole_camera_parameters(param)
        else:
            print("No previous view, using default.")
        vis.run()  # 阻塞直到用户关闭可视化窗口
        param = vis.get_view_control().convert_to_pinhole_camera_parameters()
        o3d.io.write_pinhole_camera_parameters(view_path, param)
        vis.clear_geometries()
    vis.destroy_window()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
