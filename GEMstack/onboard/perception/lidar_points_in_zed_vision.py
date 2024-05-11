import os
import numpy as np
import open3d as o3d

"""
Run this file to see how your point cloud in lidar is mapped with the area under vision
in zed. 
"""


path = 'data/new_foam4/' 
file_type = 'lidar1.npz'
file_type_perpendicular_points = 'lidar1_picked_points.npy' # indices of points in parallel points
lidar_points = np.load(path + file_type)['arr_0']

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(lidar_points)

# editable visualization
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(point_cloud)
vis.run()
vis.destroy_window()

perp_point_indices = np.load(path + file_type_perpendicular_points)

all_points = o3d.geometry.PointCloud()
all_points.points = o3d.utility.Vector3dVector(lidar_points)

white_board_point_cloud = all_points.select_by_index(perp_point_indices)
wb_min = np.min(white_board_point_cloud.points, axis=0) - 0.1
wb_max = np.max(white_board_point_cloud.points, axis=0) + 0.1

cropped_lidar_pcd = all_points.crop(o3d.geometry.AxisAlignedBoundingBox(wb_min, wb_max))

plane_model, inliers = cropped_lidar_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

[a, b, c, d] = plane_model

normal_3 = np.array([a, b, c])

z_axis = np.array([1, 0, 0])
rotation_axis = np.cross(z_axis, normal_3)
rotation_angle = np.arccos(np.dot(z_axis, normal_3) / (np.linalg.norm(z_axis) * np.linalg.norm(normal_3)))
# get 3x3 rotation matrix from rotation angle
rotation = np.array([
    [np.cos(rotation_angle), -np.sin(rotation_angle), 0],\
    [np.sin(rotation_angle), np.cos(rotation_angle), 0],\
    [0, 0, 1]\
    ])
print(rotation) # rotation

translation_vector = np.array([[0.85], [0], [1.68]]) # measured
# print('translation_vector:', translation_vector)

lidar_vehicle_tranformation = np.hstack((rotation, translation_vector))
lidar_vehicle_tranformation = np.vstack((lidar_vehicle_tranformation, np.array([0, 0, 0, 1])))
print('lidar_vehicle_calibration:\n', lidar_vehicle_tranformation)

# save calibration matrix
calibration_matrix_path = 'data/lidar_vehicle_calibration.txt'
np.savetxt(calibration_matrix_path, lidar_vehicle_tranformation, delimiter=',', fmt='%1.4f')

icp_translation = np.array([[0.14], [0], [-0.1]]) # measured
icp_transformation = np.load(path + 'lidar1_translation.npy')
icp_rotation = icp_transformation[:3, :3]
lidar_zed_calibration = np.hstack((icp_rotation, icp_translation))
lidar_zed_calibration = np.vstack((lidar_zed_calibration, np.array([0, 0, 0, 1])))

print(lidar_zed_calibration) # lidar to zed transformation using icp and measured distances

# save calibration matrix
icp_matrix_path = 'data/lidar_zed_calibration.txt'
np.savetxt(icp_matrix_path, lidar_zed_calibration, delimiter=',', fmt='%1.4f')

# camera - vehicle
camera_vehicle_calibration = lidar_vehicle_tranformation @ lidar_zed_calibration
print(camera_vehicle_calibration) # camera to vehicle transformation
camera_vehicle_calibration_path = 'data/zed_vehicle_calibration.txt'
np.savetxt(camera_vehicle_calibration_path, camera_vehicle_calibration, delimiter=',', fmt='%1.4f')


    # detection_result = self.detector(self.zed_image,verbose=False, conf = 0.85)[0]
    # boxes = detection_result.boxes
    # self.detected_pedestrians = []
    # for i, class_idx in enumerate(boxes.cls):
    #     if class_idx == 0:
    #         self.detected_pedestrians.append(boxes.xywh[i].tolist())

path = "data/new_foam4/"
lidar_pattern = os.path.join(path,"lidar1.npz")
depth_pattern = os.path.join(path,"depth1.tif")

arr_compressed = np.load(lidar_pattern)
lidar_points = arr_compressed['arr_0']
arr_compressed.close()
xrange = (0, 1280)
yrange = (0, 720)
lidar_to_zed_points = np.linalg.inv(camera_vehicle_calibration) @ lidar_vehicle_tranformation 
point_cloud = np.hstack(\
    (lidar_points[:,:3],\
    np.array([ 1 for x in range(lidar_points.shape[0])]).reshape(-1, 1)))
point_cloud = (np.dot(lidar_to_zed_points, point_cloud.T).T)[:,:3]

#point_cloud_image, image_indices =  project_point_cloud(point_cloud,, xrange, yrange)

pc_with_ids_from_lidar = np.hstack((point_cloud, np.array([ [x] for x in range(lidar_points.shape[0])])))
pc_fwd = pc_with_ids_from_lidar[pc_with_ids_from_lidar[:,2] > 0] # in front of the car
camera_info = np.array([[527.1734008789062, 0, 616.2451782226562, 0],[0, 527.1734008789062, 359.2149353027344, 0],[0, 0, 1, 0]])
lidar_to_zed_points = pc_fwd[:,:3].dot(camera_info[:3,:3].T)
flattened_depth_image_2d = (lidar_to_zed_points[:,0:2].T/lidar_to_zed_points[:,2]).T
inds = np.logical_and(\
    np.logical_and(flattened_depth_image_2d[:,0] >= xrange[0],flattened_depth_image_2d[:,0] < xrange[1]),\
    np.logical_and(flattened_depth_image_2d[:,1] >= yrange[0],flattened_depth_image_2d[:,1] < yrange[1]))
point_cloud_image = flattened_depth_image_2d[inds]
ids_on_lidar = pc_fwd[inds,3].astype(int)

point_cloud_mapped_in_zed = point_cloud[ids_on_lidar]

point_cloud_mapped_in_zed = np.concatenate((point_cloud_mapped_in_zed,np.ones((point_cloud_mapped_in_zed.shape[0],1))),axis=1)
point_cloud_mapped_in_zed = (np.dot(camera_vehicle_calibration, point_cloud_mapped_in_zed.T).T)[:,:3]


point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(point_cloud_mapped_in_zed)

# editable visualization
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(point_cloud)
vis.run()
vis.destroy_window()

point_cloud_image = point_cloud_image
point_cloud_image_world = point_cloud_mapped_in_zed

# detected_agents = []
# for i,b in enumerate(self.detected_pedestrians):
#     agent = self.box_to_agent(b, point_cloud_image, point_cloud_mapped_in_zed)
#     detected_agents.append(agent)
# return detected_agents
