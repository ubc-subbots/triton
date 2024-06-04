#!/bin/bash

# prepare sources
rosdep update --include-eol
sudo apt update
sudo apt install pip -y

# fetch repo
git clone https://github.com/ubc-subbots/triton.git
cd triton

# install deps and build
rosdep install -i --from-path src --rosdistro foxy -y
source /opt/ros/foxy/setup.bash
colcon build
