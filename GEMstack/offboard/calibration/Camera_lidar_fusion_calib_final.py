import cv2
import os
import numpy as np
import math
import time
import pickle
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation as R


def lidar_camera_point_cloud(cam_path, lidar_path, camera_info,index):
    # Load in color and depth image to create the point cloud
    print("Reading GEM dataset")
    
    # Load the npz file
    lidar_data = np.load(lidar_path+'lidar{}.npz'.format(index))

    # Loading .png image 
    png_img = cv2.imread(cam_path+'color{}.png'.format(index)) 
    tif_img = cv2.imread(lidar_path+'depth{}.tif'.format(index))
    
    cv2.imwrite(cam_path+'color{}_JPG.jpg'.format(index), png_img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    cv2.imwrite(lidar_path+'depth{}_PNG.png'.format(index), tif_img)

    color_raw = o3d.io.read_image(cam_path+'color{}_JPG.jpg'.format(index))
    depth_raw = o3d.io.read_image(lidar_path+'depth{}_PNG.png'.format(index))

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)


    fx = camera_info['fx']
    fy = camera_info['fy']
    cx = camera_info['cx']
    cy = camera_info['cy']
    width = camera_info['width']
    height = camera_info['height']
    camera_intrinsic = [[fx,0,cx],[0,fy,cy],[0,0,1]]

    # Set the intrinsic camera parameters
    camera_intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(width, height, fx=camera_intrinsic[0][0],fy=camera_intrinsic[1][1], cx=camera_intrinsic[0][2], cy=camera_intrinsic[1][2])

    # Create the point cloud from images and camera intrisic parameters
    pointcloud_camera = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic_o3d)
        
    # Flip it, otherwise the pointcloud will be upside down
    pointcloud_camera.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # o3d.visualization.draw_geometries([pointcloud_camera])




    # Extract the point cloud data
    points = lidar_data['arr_0']

    # Create a point cloud object
    pointcloud_lidar = o3d.geometry.PointCloud()

    # Add the points to the point cloud object
    pointcloud_lidar.points = o3d.utility.Vector3dVector(points)
    

    # # Visualize the point cloud
    # o3d.visualization.draw_geometries([pointcloud_lidar])

    return pointcloud_camera, pointcloud_lidar


def transformation_matrix(rvec, tvec):

    np_rodrigues = np.asarray(rvec[:,:],np.float64)
    rmat = cv2.Rodrigues(np_rodrigues)[0]
    T = np.zeros((3,4))
    T[:3,:3] = rmat
    T[:3,3] = np.transpose(tvec)

    # print(T)
    return T

def calculate_rotation_matrix(point1, point2):
    # Ensure points are numpy arrays
    point1 = np.array(point1)
    point2 = np.array(point2)

    # Calculate the rotation matrix
    vector1 = point1 / np.linalg.norm(point1)
    vector2 = point2 / np.linalg.norm(point2)

    # Calculate the rotation axis
    rotation_axis = np.cross(vector1, vector2)

    # Calculate the rotation angle
    rotation_angle = np.arccos(np.dot(vector1, vector2))

    # Create the rotation matrix
    rotation_matrix = R.from_rotvec(rotation_angle * rotation_axis)

    return rotation_matrix


if __name__ == '__main__':

    ##### STEP 1 #############
    cam_path = 'data/dataset'
    lidar_path = cam_path

    with open("GEMstack/knowledge/calibration/values.pickle", 'rb') as f:
            camera_info = pickle.load(f)

    index = 2    #This is for the image and lidar index you want to choose to create the point cloud for visualization, for the data set we have taken, it can range from 1-6.
    point_cloud_camera, point_cloud_lidar = lidar_camera_point_cloud(cam_path,lidar_path,camera_info, index)
    

    #uncomment this code to save the point cloud at a desired location and use it to get the reference point coordinates.
    # o3d.io.write_point_cloud("/path_to_save/lidar_pcd{}.pcd".format(index), point_cloud_lidar)
    
    
    # Visualize the point cloud camera
    # o3d.visualization.draw_geometries([point_cloud_camera])
    # Visualize the point cloud lidar
    # o3d.visualization.draw_geometries([point_cloud_lidar])

    fx = camera_info['fx']
    fy = camera_info['fy']
    cx = camera_info['cx']
    cy = camera_info['cy']
    width = camera_info['width']
    height = camera_info['height']
    camera_intrinsic = np.asarray([[fx,0,cx],[0,fy,cy],[0,0,1]])


    #Collected points from lidar point cloud for calibration using CloudCompare software
    objectPoints_5 = np.asarray([[2.90753865242,0.352366268635,-0.256237864494],[2.94093990326,-0.263506680727,-0.258329629898],[2.96369600296,-0.234288886189,0.470867812634],[2.94877886772,0.378269672394,0.470867812634],[4.910995,0.391678,-0.780295],[4.7923,-0.2445,-0.42],[4.863,-0.0246,-0.0849],[4.908957,0.248678,-0.085796]])
    objectPoints_1 = np.asarray([[2.94675588608,0.817762196064,-0.819421112537],[2.98203849792,0.197544395924,-0.800786137581],[2.908203125,0.206932991743,-0.0508912168443],[2.8709628582,0.811860620975,-0.0520779825747]])
    
    #Corresponding pixel coordinates in the corresponding zed images, used pixspy.com 
    imagePoints_5 = np.asarray([[583,381],[702,378],[696,224],[580,228],[609,421],[676,384],[649,341],[614,341]])
    imagePoints_1 = np.asarray([[498,481],[614,477],[610,330],[492,333]])
    objectPoints_5 = objectPoints_5.astype('float32')
    imagePoints_5 = imagePoints_5.astype('float32')
    
    distCoeffs = np.zeros((5,1))
    points_2d = np.array([np.concatenate((imagePoints_1 , imagePoints_5), axis=0)])   # Replace with actual 2D points
    points_3d =  np.array([np.concatenate((objectPoints_1 , objectPoints_5), axis=0)]) # Replace with actual 3D points
    
    #Used PNP solver to find the rotation and translation components of the transformation
    [success, rvec_comb, tvec_comb] = cv2.solvePnP(points_3d, points_2d, camera_intrinsic, distCoeffs)

    T_comb = transformation_matrix(rvec_comb,tvec_comb)



##################  STEP 2   ##################

    #Lidar calibration

    #Used a board to find points on the board in lidar point cloud to find the plane normal and then the     rotation matrix between lidar and rear axle frame.
    board_pts =  np.array([[1.546349,0.516498,-0.085308],[1.565219,-0.164511,-0.082481],[1.558125,0.518319,-0.44],[1.583414,-0.16360,-0.426534],[1.564425,0.137145,-0.248731],[1.58086979389,0.101399496198,-0.424463272095],[1.57320547104,0.0995287671685,-0.24966943264],[1.54885876179,0.0966323316097,-0.0813300833106],[1.55912876129,0.302499502897,-0.251546651125],[1.57887101173,-0.103761412203,-0.250608056784]])
    board_pts2 = np.array([[1.57175576687,0.0450020469725,-0.249043703079],[1.57592618465,-0.0870043560863,-0.249982297421],[1.55988907814,-0.0217815730721,-0.136485889554],[1.58587574959,-0.0370962880552,-0.308347344398]])
    
    #vectors in the plane of the board
    vec1 = board_pts[0]-board_pts[3]
    vec2= board_pts[0]-board_pts[1]
    vec3 = board_pts[0]-board_pts[3]
    vec4 = board_pts[2]-board_pts[4]
    vec5 = board_pts[2]-board_pts[3]
    vec6 = board_pts2[1]-board_pts2[0]
    vec7 = board_pts2[2]-board_pts2[3]

    # Normal vector to the plane containing these vectors.
    normal = np.cross(vec1, vec2)/np.linalg.norm(np.cross(vec1, vec2))
    normal2 = np.cross(vec3, vec4)/np.linalg.norm(np.cross(vec3, vec4))
    normal3 = np.cross(vec6, vec7)/np.linalg.norm(np.cross(vec6, vec7))
   
    
    rotation_matrix = calculate_rotation_matrix(normal3,[-1,0,0]).as_matrix()
    yaw = np.arctan2(rotation_matrix[1][0],rotation_matrix[0][0])
    pitch=np.arctan2(-rotation_matrix[2][0],np.sqrt(rotation_matrix[2][1]**2+rotation_matrix[2][2]**2))
    roll=np.arctan2(rotation_matrix[2][1],rotation_matrix[2][2])



    #Velo lidar to gem trans
    trans_vel_gem = np.array([0.85,0,1.71])
    T_vel_gem = np.zeros((4,4))
    T_vel_gem[:3,:3] = rotation_matrix
    T_vel_gem[:3,3] = np.transpose(trans_vel_gem)
    T_vel_gem[3,3] = 1  


    # Lidar to camera transformation
    T_vel_to_cam = np.zeros((4,4))   
    T_vel_to_cam[:3,:] = T_comb
    T_vel_to_cam[3,3] = 1


    #Cam to gem transformation
    T_cam_to_gem = np.matmul(np.linalg.inv(T_vel_to_cam) , T_vel_gem)
    
    
    #printing final matrices
    print("Matrices: \n")
    print("T_lidar_gem: \n",T_vel_gem,'\n','T_lidar_cam:\n', T_vel_to_cam,'\n','T_cam_gem:\n', T_cam_to_gem)






