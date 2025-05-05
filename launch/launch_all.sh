#!/bin/bash
<< COMMENTOUT
##########
- This script opens three new tabs in GNOME Terminal.
- Each tab will change to the GEMstack directory, source the ROS environment, and then run the specified command.

- Run these commands at each tab:
cd ~/catkin_ws/src/GEMstack
source ~/catkin_ws/devel/setup.bash && roslaunch basic_launch sensor_init.launch
source ~/catkin_ws/devel/setup.bash && roslaunch basic_launch dbw_joystick.launch
source ~/catkin_ws/devel/setup.bash && python3 main.py --variant=fake_real launch/pedestrian_detection.yaml

- If you don't want to detect virtual pedestrian, run this command instead:
source ~/catkin_ws/devel/setup.bash && python3 main.py launch/pedestrian_detection.yaml

- If roscore cannot run, run this command to kill all roscore:
killall -9 roscore rosmaster
##########
COMMENTOUT

cd ~/catkin_ws/src/GEMstack
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch basic_launch sensor_init.launch; exec bash"
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch basic_launch dbw_joystick.launch; exec bash"
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash && python3 main.py --variant=fake_real launch/pedestrian_detection.yaml; exec bash"