#!/bin/bash
sudo apt update
sudo apt-get install git python3 python3-pip wget zstd
sudo apt-get install ros-noetic-geographic-msgs
source /opt/ros/noetic/setup.bash

#install Zed SDK
wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu20 -O ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run
chmod +x ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run
./ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run -- silent

#create ROS Catkin workspace
mkdir catkin_ws
mkdir catkin_ws/src

#install ROS dependencies and packages
cd catkin_ws/src
git clone https://github.com/krishauser/POLARIS_GEM_e2.git
git clone https://github.com/astuff/pacmod2.git
git clone https://github.com/astuff/astuff_sensor_msgs.git
git clone https://github.com/ros-perception/radar_msgs.git
git clone https://github.com/bsb808/geonav_transform.git 
cd radar_msgs; git checkout noetic; cd ..

cd ..   #back to catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
cd .. #back to GEMstack

#install GEMstack Python dependencies
python3 -m pip install -r requirements.txt