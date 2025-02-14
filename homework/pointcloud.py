import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R  # For converting rotation matrix to Euler angles

# ----- Helper Functions -----

def fit_plane_ransac(points, threshold, min_inliers, iterations=100):
    """
    A simple RANSAC plane fitting implementation.
    """
    best_inliers_count = 0
    best_plane = None
    best_inliers = None
    n_points = points.shape[0]
    if n_points < 3:
        return None, None

    for _ in range(iterations):
        idx = np.random.choice(n_points, 3, replace=False)
        sample = points[idx]
        p1, p2, p3 = sample
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm == 0:
            continue
        normal = normal / norm
        d = -np.dot(normal, p1)
        plane = np.hstack([normal, d])
        distances = np.abs(points.dot(normal) + d)
        inliers = np.where(distances < threshold)[0]
        if len(inliers) > best_inliers_count:
            best_inliers_count = len(inliers)
            best_plane = plane
            best_inliers = inliers

    if best_inliers_count >= min_inliers:
        return best_plane, best_inliers
    else:
        return None, None

def backproject_pixel(u, v, K):
    """
    Backproject a pixel (u,v) into a normalized 3D ray in camera coordinates using the intrinsic matrix K.
    """
    cx = K[0, 2]
    cy = K[1, 2]
    fx = K[0, 0]
    fy = K[1, 1]
    x = (u - cx) / fx
    y = (v - cy) / fy
    ray_dir = np.array([x, y, 1.0])
    return ray_dir / np.linalg.norm(ray_dir)

def find_human_center_on_ray(lidar_pc, ray_origin, ray_direction,
                             t_min, t_max, t_step,
                             distance_threshold, min_points, ransac_threshold):
    """
    Sweep along the ray (from ray_origin in LiDAR coordinates) and search for a local planar cluster.
    """
    t_values = np.arange(t_min, t_max, t_step)
    for t in t_values:
        candidate = ray_origin + t * ray_direction
        dists = np.linalg.norm(lidar_pc - candidate, axis=1)
        nearby_points = lidar_pc[dists < distance_threshold]
        if nearby_points.shape[0] < min_points:
            continue
        plane_model, inliers = fit_plane_ransac(nearby_points, ransac_threshold, min_points)
        if plane_model is not None:
            a, b, c, d = plane_model
            denom = a * ray_direction[0] + b * ray_direction[1] + c * ray_direction[2]
            if abs(denom) < 1e-6:
                continue
            t_plane = -(a * ray_origin[0] + b * ray_origin[1] + c * ray_origin[2] + d) / denom
            if t_plane < 0:
                continue
            intersection = ray_origin + t_plane * ray_direction
            return intersection, plane_model, inliers
    return None, None, None

def extract_roi(pc, center, roi_radius):
    """
    Extract points from the point cloud (pc) that lie within roi_radius of center.
    """
    distances = np.linalg.norm(pc - center, axis=1)
    return pc[distances < roi_radius]

def refine_cluster(roi_points, center, eps=0.2, min_samples=10):
    """
    Further refine a cluster using DBSCAN and return the cluster whose centroid is closest to center.
    """
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(roi_points)
    labels = clustering.labels_
    unique_labels = set(labels)
    if -1 in unique_labels:
        unique_labels.remove(-1)
    if not unique_labels:
        return roi_points

    best_cluster = None
    best_distance = float('inf')
    for label in unique_labels:
        cluster = roi_points[labels == label]
        cluster_center = np.mean(cluster, axis=0)
        dist = np.linalg.norm(cluster_center - center)
        if dist < best_distance:
            best_distance = dist
            best_cluster = cluster
    return best_cluster

def remove_ground_by_min_range(cluster, z_range=0.05):
    """
    Remove ground points from the cluster by finding the minimum z value and eliminating
    all points within z_range of that minimum.
    """
    if cluster is None or cluster.shape[0] == 0:
        return cluster
    min_z = np.min(cluster[:, 2])
    filtered = cluster[cluster[:, 2] > (min_z + z_range)]
    return filtered

def get_bounding_box_center_and_dimensions(points):
    """
    Compute the bounding box center and dimensions (max - min) for the given points.
    """
    if points.shape[0] == 0:
        return None, None
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    center = (min_vals + max_vals) / 2
    dimensions = max_vals - min_vals
    return center, dimensions

def create_circle_line_set(center, radius, num_points=50):
    """
    Create a LineSet representing a circle (in the X-Y plane) with given center and radius.
    """
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    circle_points = []
    for angle in theta:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        circle_points.append([x, y, z])
    circle_points = np.array(circle_points)
    lines = [[i, (i + 1) % num_points] for i in range(num_points)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(circle_points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in range(len(lines))])
    return line_set

def create_ray_line_set(start, end):
    """
    Create a LineSet representing a ray from 'start' to 'end' (colored yellow).
    """
    points = [start, end]
    lines = [[0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[1, 1, 0]])
    return line_set

def visualize_geometries(geometries, window_name="Open3D", width=800, height=600, point_size=5.0):
    """
    Utility to visualize a list of Open3D geometries.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=width, height=height)
    for geom in geometries:
        vis.add_geometry(geom)
    opt = vis.get_render_option()
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()

# ----- Main Processing -----

def main():
    # Load the color image.
    idx = 5
    img = cv2.imread(f"../data/color{idx}.png")
    if img is None:
        print("Error: Could not load the color image.")
        return

    # Show the original YOLO detection results on the image.
    model = YOLO("yolov8n.pt")
    results = model.predict(img, classes=[0])  # detect only person (class id: 0)
    boxes = results[0].boxes.xywh.tolist()  # each box: [center_x, center_y, w, h]
    for box in boxes:
        cx, cy, w, h = box
        top_left = (int(cx - w/2), int(cy - h/2))
        bottom_right = (int(cx + w/2), int(cy + h/2))
        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
    cv2.imshow("YOLO Detection", img)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

    # Load LiDAR point cloud from NPZ (assumed key 'arr_0').
    lidar_data = np.load(f"../data/lidar{idx}.npz")
    lidar_pc = lidar_data['arr_0']  # (N,3) points in LiDAR coordinates

    # ----- Camera Parameters & Transformations -----
    K = np.array([
        [684.83331299, 0., 573.37109375],
        [0., 684.60968018, 363.70092773],
        [0., 0., 1.]
    ])
    T_l2c = np.array([
        [-0.01909581, -0.9997844, 0.0081547, 0.24521313],
        [0.06526397, -0.00938524, -0.9978239, -0.80389025],
        [0.9976853, -0.01852205, 0.06542912, -0.6605772],
        [0., 0., 0., 1.]
    ])
    T_c2l = np.linalg.inv(T_l2c)
    camera_origin_in_lidar = T_c2l[:3, 3]
    R_c2l = T_c2l[:3, :3]

    # Prepare lists for Open3D debugging visualization.
    # debug_all will contain all objects.
    # debug_filtered will exclude the full point cloud and the initial ROI.
    debug_all = []
    debug_filtered = []

    # Add the full LiDAR point cloud (gray) only to debug_all.
    pcd_full = o3d.geometry.PointCloud()
    pcd_full.points = o3d.utility.Vector3dVector(lidar_pc)
    pcd_full.paint_uniform_color([0.7, 0.7, 0.7])
    debug_all.append(pcd_full)

    # For each detected person:
    for box in boxes:
        cx, cy, w, h = box
        # The center pixel is (cx, cy) since YOLO gives center coordinates.
        center_u = cx
        center_v = cy

        # Backproject the center pixel into a ray (camera coordinates).
        ray_dir_cam = backproject_pixel(center_u, center_v, K)
        # Transform ray direction to LiDAR coordinates.
        ray_dir_lidar = R_c2l @ ray_dir_cam
        ray_dir_lidar = ray_dir_lidar / np.linalg.norm(ray_dir_lidar)

        # --- Method 1: Ray-sweeping with plane fitting ---
        t_min = 0.5      # minimum distance (m)
        t_max = 20.0     # maximum distance (m)
        t_step = 0.1     # step size (m)
        distance_threshold = 0.2   # neighborhood threshold (m)
        min_points = 20            # minimum points for plane fit
        ransac_threshold = 0.05    # RANSAC threshold (m)

        intersection, plane_model, inliers = find_human_center_on_ray(
            lidar_pc, camera_origin_in_lidar, ray_dir_lidar,
            t_min, t_max, t_step, distance_threshold, min_points, ransac_threshold
        )

        if intersection is None:
            print("No 3D intersection found along the ray for box:", box)
            continue

        # --- Method 2: Refine using ROI extraction & DBSCAN ---
        roi_radius = 1.0  # meters
        roi_points = extract_roi(lidar_pc, intersection, roi_radius)
        if roi_points.shape[0] < min_points:
            print("Not enough points in ROI for box:", box)
            continue
        # Refine the cluster using DBSCAN.
        refined_cluster = refine_cluster(roi_points, intersection, eps=0.15, min_samples=10)
        # Remove ground points by choosing the minimum z and eliminating all points within a small z range.
        refined_cluster = remove_ground_by_min_range(refined_cluster, z_range=0.05)
        if refined_cluster is None or refined_cluster.shape[0] == 0:
            refined_center = intersection
            bbox_dims = np.array([0, 0, 0])
        else:
            # Instead of mean, compute the bounding box (oriented) center and dimensions.
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(refined_cluster)
            obb = pcd_cluster.get_oriented_bounding_box()
            refined_center = obb.center
            bbox_dims = obb.extent  # dimensions along the principal axes
            # Compute Euler angles (yaw, pitch, roll) from the rotation matrix.
            euler_angles = R.from_matrix(obb.R.copy()).as_euler('zyx', degrees=True)
            # euler_angles[0] is yaw, euler_angles[1] is pitch, euler_angles[2] is roll.
            print(f"Detected human - Pose (yaw, pitch, roll): {euler_angles}")
            print(f"Bounding box center: {refined_center}, Dimensions: {bbox_dims}")

        # Project the refined 3D center back to the image.
        refined_center_h = np.hstack([refined_center, 1])
        cam_point = T_l2c @ refined_center_h
        cam_point = cam_point[:3] / cam_point[2]
        u_proj = int(round(K[0, 0] * cam_point[0] + K[0, 2]))
        v_proj = int(round(K[1, 1] * cam_point[1] + K[1, 2]))

        # Draw the bounding box and projected 3D center on the image.
        top_left = (int(cx - w/2), int(cy - h/2))
        bottom_right = (int(cx + w/2), int(cy + h/2))
        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
        cv2.circle(img, (u_proj, v_proj), radius=8, color=(0, 255, 0), thickness=2)
        cv2.putText(img, "3D Center", (u_proj+5, v_proj-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # For debugging in Open3D:
        # Add the ROI point cloud (red) only to debug_all.
        pcd_roi = o3d.geometry.PointCloud()
        pcd_roi.points = o3d.utility.Vector3dVector(roi_points)
        pcd_roi.paint_uniform_color([1, 0, 0])
        debug_all.append(pcd_roi)
        # Add the refined cluster point cloud (blue) to both lists.
        pcd_cluster_vis = o3d.geometry.PointCloud()
        pcd_cluster_vis.points = o3d.utility.Vector3dVector(refined_cluster)
        pcd_cluster_vis.paint_uniform_color([0, 0, 1])
        debug_all.append(pcd_cluster_vis)
        debug_filtered.append(pcd_cluster_vis)
        # Add the refined center marker (green sphere) to both lists.
        sphere_center = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        sphere_center.translate(refined_center)
        sphere_center.paint_uniform_color([0, 1, 0])
        debug_all.append(sphere_center)
        debug_filtered.append(sphere_center)
        # Add the ray (yellow) from camera origin to the refined center to both lists.
        ray_line = create_ray_line_set(camera_origin_in_lidar, refined_center)
        debug_all.append(ray_line)
        debug_filtered.append(ray_line)
        # Also add the oriented bounding box to the debug visualization.
        obb.color = (1, 0, 1)  # Magenta
        debug_all.append(obb)
        debug_filtered.append(obb)

    # Show final image with detections.
    cv2.imshow("3D Human Centers Projection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Open3D debugging visualizations.
    print("Launching Open3D debug visualization with ALL objects...")
    visualize_geometries(debug_all, window_name="LiDAR Debug Visualization (All)", point_size=5.0)
    print("Launching Open3D debug visualization without Full PointCloud and ROI...")
    visualize_geometries(debug_filtered, window_name="LiDAR Debug Visualization (Filtered)", point_size=5.0)

if __name__ == '__main__':
    main()
