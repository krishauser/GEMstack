# Calibration Usage

## For HW3
#### Folder Structure

#### HW3 Step 1 (Lidar -> Camera Calibration)
1. Manually extract corresponding feature points between 3D lidar point cloud and 2D image using `lidar_feature_extract.py` and `camera_feature_extract.py`. After extraction, scripts will output the `save/lidar.csv` and `save/image.csv` files respectively including the coordinate of feature points. Make sure two .csv files of lidar and image including the same number of feature points. <br><br>
    ```
    1. python GEMstack/offboard/calibration/camera_feature_extract.py -s <src_file>
    ex: python GEMstack/offboard/calibration/lidar_feature_extract.py -s data/step1/lidar6.npz

    2. python GEMstack/offboard/calibration/camera_feature_extract.py -s <src_file>
    ex: python GEMstack/offboard/calibration/camera_feature_extract.py -s data/step1/color6.png
    ```

    Note:
    - You can select points by `shift + click` in Open3D.
    - Humongous ball will show up on the pixel just being clicked. You can reduce the ball size by pressing three buttons altogether `ctrl + shift + minus`.
    - After finish point selection, press `esc` to exit and save the selected point coordinate to file.
    <br>

    Pipeline:
    <img width="941" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/74b20ea2-571c-4e1f-b95c-24a3f8583193">

2. Use `lidar_camera_calibration.py` to find optimized parameters of extrinsic matrix based on corresponding feature points extracted in previous step.

3. Verify the obtained extrinsic matrix. Change the extrinsic matrix variable in `testing/test_pedestrian_detection.py` then run the script. If it works, you can see point clouds well align with the image pixels on target object. Result will be saved in `save/lidar_to_image.png` <br>
    ```
    python testing/test_pedestrian_detection.py -t lidar_to_image
    ```
    <img width="540" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/7430e26e-f916-492e-8742-1732eca37a81">

#### HW3 Step 2 (Lidar -> Vehicle Calibration)
1. Use `lidar_vehicle_calibration.py` to select four corner points of the planar object manually, then press `esc` two times to get the transformation matrix. <br>
<img width="500" alt="image" src="https://github.com/krishauser/GEMstack/assets/22386566/bcdbfbac-ec13-4a44-966d-0931d26f2021">

#### HW3 Step 3
1. Use `check_target_lidar_range.py` to determine the lidar range we desire for agents to be detected.
