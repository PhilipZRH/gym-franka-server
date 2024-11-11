#!/usr/bin/env bash

sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon python3-wstool python3-rosdep python3-pip

sudo rosdep init
rosdep update

cd server_ws/src
wstool init .

# Dependencies
rosdep install -y --from-paths . --ignore-src --rosdistro noetic

# Build
cd ..
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
cd ..

echo "source $(pwd)/server_ws/devel/setup.bash" >> ~/.bashrc

pip3 install websockets scipy

