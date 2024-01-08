FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
#use bash instead of sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -y git python3 python3-pip wget zstd
# Install ROS Noetic
RUN apt-get update && apt-get install -y lsb-release gnupg2
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
RUN apt-key add ros.asc
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop
RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools
RUN rosdep init
RUN rosdep update

#Install Cuda 11.8
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
#RUN sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
##add public keys
#RUN sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
#RUN sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
#RUN install cuda-toolkit-11-8


# install Zed SDK
RUN wget https://download.stereolabs.com/zedsdk/4.0/cu118/ubuntu20 -O zed_sdk.run
RUN chmod +x zed_sdk.run
RUN ./zed_sdk.run -- silent

# create ROS Catkin workspace
RUN mkdir -p /catkin_ws/src

# install ROS dependencies and packages
RUN cd /catkin_ws/src && git clone https://github.com/krishauser/POLARIS_GEM_e2.git
RUN cd /catkin_ws/src && git clone --recurse-submodules https://github.com/stereolabs/zed-ros-wrapper.git
RUN cd /catkin_ws/src && git clone https://github.com/astuff/pacmod2.git
   #for some reason the ibeo messages don't work?
RUN cd /catkin_ws/src && git clone https://github.com/astuff/astuff_sensor_msgs.git && rm -rf astuff_sensor_msgs/ibeo_msgs
RUN cd /catkin_ws/src && git clone https://github.com/ros-perception/radar_msgs.git \
   && cd radar_msgs && git checkout noetic 

RUN source /opt/ros/noetic/setup.bash && cd /catkin_ws && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release

# install GEMstack Python dependencies
RUN git clone https://github.com/krishauser/GEMstack.git
RUN cd GEMstack && pip3 install -r requirements.txt

RUN echo /catkin_ws/devel/setup.sh
