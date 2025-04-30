# Porting GEMstack to ROS2
GEMstack does not rely on a lot of ROS nodes and hence the major changes needed to port GEMstack from ROS1 to ROS2 were in 
1. GEMstack/offboard/calibration/capture_lidar_zed.py
2. GEMstack/onboard/execution/execution.py
3. GEMstack/onboard/interface/gem_hardware.py

GEMHardwareInterface was defined as a class that subscribed to different topics and published to other topics. But in ROS2, we can only subscribe or publish 
if we have a node. Hence we declared a node inside the class. The other important change was with respect to the mapping of PACMOD messages.
The message type, topic names and the error flags have been renamed in the new version of PACMOD and the mapping for all of them are complete and added in gem_hardware.py.

Pending items:
1. gem_hardware.py files have the commands to drive the vehicle through new PACMOD version but it is still untested. Need to ensure that the flow of execution can drive the vehicle.
