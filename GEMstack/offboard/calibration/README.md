Calibration
==============================================================================================================================
Folder 
===============================================================================================================================
In data folder, camera_info includes the camera instrinsic, and data_part1, data_part2 include all data relate to calibration.

We use open3d to help us to automatically calibrate our camera. When you run the code, it will display the LiDAR point cloud and the ZED camera image in sequence, 
and you just need to choose the feature points(the corner of an object) in the data. Step1 and Step2 are similar.

Open Terminal and cd into GEMstack
Use the command: "python3 GEMstack/offboard/calibration/interactive_visualization.py data/(calibration image name)"
Use "Ctrl + right click" to choose the feature points on lidar point cloud, you can use "Shift + Ctrl + right click" to undo the points.
Use "Ctrl + right click" to choose the feature points on Zed image, you need to choose the same points and same order with the previous step.
Close the window, and the extrinsct will show on the terminal
