I) Lidar - Zed calibration:

In order to find the transformation matrix between image pixel points and lidar point cloud points, we used a manual calibration method.

Step 1 : First we collected images and lidar scans of the grey boxes for calibration. But unfortunately, they being grey they weren't captured in the lidar scans. We had the images, but not the corresponding lidar points in the lidar point clouds. So, we took these image and lidar scans data from another group as time was limited. 

Step 2: Given the images and lidar scans, a brown board was used in the captured images for calibration purposes. We selected few reference points on the board which is held at different positions/orientations in different images and collected the pixel coordinates using an online tool(pixspy.com) and the lidar points from their corresponding lidar scan data. The lidar scan data is converted into a lidar point cloud, and we used CloudCompare software to visualize the point cloud and select the reference points and get their 3d (x,y,z) data.

Step 3: We collected the camera intrinsic parameters by subscribing to the Zed camera info msg as given in the website. Now with the pixel coordinates and the corresponding lidar points, the camera intrinsic matrix, we used the solvePNP function from OpenCV to get the rotation and translation vectors of the image_pixel to lidar_point_cloud transformation. 

Step 4: The rotation vector(1x3) needs to be converted into the standard 3x3 rotation matrix. We used the rodrigues method to get the rotation matrix(3x3) from rotation vector(1x3). Then combined the rotation matrix(3x3) and translation vector(1x3) to get the whole lidar to zed transformation matrix T_vel_to_cam.

II) Lidar rotation calibration:

Step 1: Again we used a board held almost vertical to the vehicle to calibrate this transform. The dataset(taken from group 2), consists of a white board infront of the vehicle. We selected few reference points on the board plane using CloudCompare to get the 3d (x,y,z) data.

Step 2: By subtracting one reference point from another to get the resulting vector which will lie in the plane of the board, since those two points lie in the plane of the board. Now, we computed the normal to the plane of the board. This normal wrt rear axle is nearly in line with the X-axis of the vehicle, but in the lidar reference frame will be aligned at some angle.

Step 3: We know the normal vector in rear_axle frame is [-1,0,0], and the normal vector in the lidar frame is calculated as described in step 2. Now we caculate the rotation matrix which transforms this vector in vehicle frame to lidar frame or vice versa. In order to calculate the rotation matrix, we find the rotation axis which is a normal of the plane containing these two vectors(used np.cross for the cross product to get normal)and then calculated the rotation angle by taking the cosine inverse of the dot product between the two vectors. Finally used scipy.spatial.transform.Rotation.from_rotvec() to get the rotation matrix from rotation angle and axis data.

Step 4: Used a tape to measure the x_axis distance and z_axis distance from the rear frame to get the position of the lidar wrt rear axle frame. Assumed y offset is zero.

Step 5: Combined the rotation and translation matrices to get the T_vel_gem transformation matrix, for tranforming points from lidar frame to gem veh rear axle frame.

Step 6: Used the T_vel_to_cam and T_vel_gem transformation matrices to get the T_cam_to_gem transformation matrix (T_cam_to_gem = inv(T_vel_to_cam)*T_vel_gem).


III) Results discussion:

1) While capturing the lidar scans, using the grey boxes for reference points was futile. The lidar point cloud missed the reference boxes in the point cloud. This can be because grey/black objects have high absorbance and low reflectance of light falling on them. As a result the lidar rays might have been absorbed and not reflected back to be captured by the sensor. Using white or light coloured objects can make the lidar better detect the objects and store the point cloud point.

2) Using manual calibration, there was slight error in the translation vec of the lidar-camera transformation matrix. When we compared the actual position of the zed camera relative to the lidar with the translation vector in the T matrix, we found around 4 cm error in y direction and 7-8 cm error in the z direction. But when tested on the world coordinates to camera pixel coordinates for a random point, the error in pixel coordinates was low. We found out that the error in relative position( i.e the translation vector error) has less influence on the final transformation into pixel coordinates.

3) The lidar-camera transformation matrix works fine for points near to the vehicle. But for far away points, there is considerable error.


IV) How to use the calibration file:

1) You need the lidar.npz file and the corresponding image, depth files. This approach was done with reference points from two images and corresponding lidar scans. 

2) For the reference points selection from point cloud, you can use CloudCompare software and for pixel selection from images you can use pixspy.com . 

3) For the lidar calibration, we used a dataset containing white board and the method is described above.