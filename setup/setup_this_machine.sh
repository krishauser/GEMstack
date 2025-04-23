#!/bin/bash
sudo apt update
sudo apt-get install -y git python3 python3-pip wget zstd

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic not found. Installing ROS Noetic..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
    sudo apt-key add ros.asc
    sudo apt update
    sudo DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-desktop
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools
    sudo rosdep init
    rosdep update
fi
source /opt/ros/noetic/setup.bash

#install Zed SDK
echo "To install the ZED SDK, select the CUDA version:"
echo "1) CUDA 11.8"
echo "2) CUDA 12+"
echo "3) No GPU (Skip ZED SDK installation)"
read -p "Enter choice [1-3]: " choice

case $choice in
    1)
        wget https://download.stereolabs.com/zedsdk/4.0/cu118/ubuntu20 -O zed_sdk.run
        chmod +x zed_sdk.run
        ./zed_sdk.run -- silent
        ;;
    2)
        wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Ubuntu20_cuda12.1_v4.2.4.zstd.run -O zed_sdk.run
        chmod +x zed_sdk.run
        ./zed_sdk.run -- silent
        ;;
    3)
        echo "Skipping ZED SDK installation..."
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

#create ROS Catkin workspace
mkdir -p ~/catkin_ws/src

# Store current working directory
CURRENT_DIR=$(pwd)
echo "CURRENT_DIR: $CURRENT_DIR"

#install ROS dependencies and packages
cd ~/catkin_ws/src
git clone https://github.com/krishauser/POLARIS_GEM_e2.git
git clone https://github.com/astuff/pacmod2.git
git clone https://github.com/astuff/astuff_sensor_msgs.git && rm -rf astuff_sensor_msgs/ibeo_msgs
git clone https://github.com/ros-perception/radar_msgs.git && cd radar_msgs; git checkout noetic; cd ..

cd ..   #back to catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

cd $CURRENT_DIR
#install GEMstack Python dependencies

# Ask the user if they want to install ultralytics
read -p "Do you want to install ultralytics? (y/n): " install_ultralytics

# Create a temporary requirements file
temp_requirements="temp_requirements.txt"

# Copy all lines except ultralytics if the user chooses to skip it
if [ "$install_ultralytics" == "y" ]; then
    cp requirements.txt $temp_requirements
else
    grep -v "ultralytics" requirements.txt > $temp_requirements
fi

# Install the packages from the temporary requirements file
pip3 install -r $temp_requirements

# Clean up the temporary file
rm $temp_requirements

#install other dependencies
sudo apt-get install -y ros-noetic-septentrio-gnss-driver