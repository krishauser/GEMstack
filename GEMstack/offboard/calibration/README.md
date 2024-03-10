Calibration
==============================================================================================================================
Folder 
===============================================================================================================================
In data folder,
`camera_info`: includes the camera instrinsic.
`data_part1`: include all data relate to calibration.
`data_part2`: include all data relate to calibration.

Method
===================
We use ICP algorithm (`global_registration.py`) to help us to automatically calibrate our camera. When you run the code, it will display the LiDAR point cloud and the ZED camera image in sequence, and you just need to choose the feature points(the corner of an object) in the data. Step1 and Step2 are similar.

Steps
====================
1. Open Terminal and cd into GEMstack
2. Use the command: `python3 GEMstack/offboard/calibration/interactive_visualization.py data/(calibration image name)`
3. Use `Ctrl + right click` to choose the feature points on lidar point cloud, you can use` Shift + Ctrl + right click` to undo the points.
4. Use `Ctrl + right click` to choose the feature points on Zed image, you need to choose the same points and same order with the previous step.
5. Close the window, and the extrinsct will show on the terminal
