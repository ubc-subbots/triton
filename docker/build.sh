#!/bin/bash

# prepare sources
rosdep update --include-eol
sudo apt update

# Manual installation (debugging)
sudo apt install pip -y
sudo apt update



### Installing Opencv --START
#!/bin/bash

# Install prerequisites
sudo apt update
sudo apt install -y cmake g++ wget unzip

# Download and unpack OpenCV sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.3.zip
unzip opencv.zip

# Create build directory
mkdir -p build && cd build

# Configure and build OpenCV
cmake ../opencv-4.5.3
make -j$(nproc)

# Install OpenCV
sudo make install

# Clean up
cd ..
rm -rf build opencv.zip opencv-4.5.3

### Installing Opencv --END




#sudo apt install libopencv-dev python3-opencv
# sudo apt install ros-foxy-cv-bridge

# fetch repo
git clone https://github.com/ubc-subbots/triton.git
cd triton

# install deps and build
rosdep install -i --from-path src --rosdistro foxy -y
source /opt/ros/foxy/setup.bash
pkg-config --modversion opencv #Debuging OpenCV Version
colcon build
