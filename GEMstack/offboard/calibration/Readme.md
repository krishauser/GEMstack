# Calibration

## Folders

- `get_extrinsic_matrix`: Step 1 usage - get the extrinsic matrix of the camera, which can transform the LiDAR point cloud into camera pixels.
- `verify_extrinsic_matrix`: Verify the extrinsic matrix, which can verify the matrix we get.
- `get_lidar_vehicle`: Get the rotation matrix of LiDAR, which can transform LiDAR points based on the world.

## STEP 1: LiDAR -> Camera Calibration

1. Extract corresponding points between 3D LiDAR point cloud and 2D image using `get_extrinsic_matrix/Opencv.py` and `get_extrinsic_matrix/Open3d.py`. Then store the points into a `txt` file.  
   **Note**: Select points by `shift + click` in Open3D.
2. By using the `txt` file obtained from step 1, `get_normal_vector.py` can get the extrinsic matrix using the PnP method.
3. Use `verify_extrinsic_matrix.py/verifyextrinsic` to verify the obtained extrinsic matrix. If it works, you can see the point cloud projected correctly into stop signs of the picture.

## STEP 2: LiDAR -> Vehicle Calibration

1. `get_normal_vector.py` can get the original vector of the LiDAR-vehicle.
2. `rotation_matrix.py` can get the point-point vector and then, based on the two vectors, compute the transformation matrix.
