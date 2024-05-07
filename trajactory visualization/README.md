###  Trajectory Visualization

This util.py Python script is designed to visualize vehicle trajectories from ROS bag files. It overlays the planned trajectory on the vehicle's camera feed and generates a video showing the trajectory.

Requirements

* Python 3.x
* OpenCV (cv2)
* NumPy
* ROS (Robot Operating System) bag files      

#### Usage

Prepare ROS Bag File: Ensure you have a ROS bag file containing the vehicle's camera feed (/oak/rgb/image_raw) and the planned trajectory data.
**View Outpu**t: Once the script finishes execution, it will generate a video file (op.avi) containing the visualization of the trajectory overlaid on the camera feed.

#### Notes

Ensure that the planned trajectory data is in the correct format (a text file with each line containing a triplet of (x, y, z) coordinates).
Adjust the projection and coordinate transformation functions if necessary based on your camera setup and coordinate systems.

