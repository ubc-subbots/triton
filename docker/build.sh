#!/bin/bash

# prepare sources
rosdep update --include-eol
sudo apt update

# Manual installation (debugging)
sudo apt install pip -y
# sudo apt install ros-foxy-cv-bridge

# fetch repo
git clone https://github.com/ubc-subbots/triton.git
cd triton

# install deps and build
rosdep install -i --from-path src --rosdistro foxy -y
source /opt/ros/foxy/setup.bash
pkg-config --modversion opencv #Debuging OpenCV Version
colcon build
