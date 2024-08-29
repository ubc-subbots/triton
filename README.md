# Triton AUV

This repository contains the ROS2 system for the UBC SubBots Triton AUV. It is meant be launched in Ubuntu 20.04 on the Jetson TX2 on board the Triton AUV. 

# Contents

- [Setup](#setup)
    - [Gazebo Installation](#gazebo-installation)
    - [OpenCV Installation](#opencv-installation)
    - [ROS2 Dependencies](#ros2-dependencies)
- [Contributing](#contributing)
- [Tips](#tips)
## Setup
To get started, first clone this repo to your computer running Ubuntu 20.04 into whatever directory you choose as such

    git clone https://github.com/ubc-subbots/triton.git

### Gazebo Installation
To install Gazebo, it is as simple as running the following command

    curl -sSL http://get.gazebosim.org | sh

To verify it was succesfully installed, run the following command

    gazebo --version

Next, we need to add the setup script to our `.bashrc` so that it is sourced on every new terminal, open up `~/.bashrc` in a text editor or in nano as such

    nano ~/.bashrc

Append the following line to the bottom of the file

    source /usr/share/gazebo/setup.sh

Now Gazebo is succesfully installed!
### OpenCV Installation
First, make sure you have all the dependencies installed for building and running OpenCV

    sudo apt-get install build-essential # compiler
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev # required
    sudo apt-get install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev # optional

Then download the OpenCV 4.5.3 source and create a build folder to navigate to as such

    cd ~
    git clone https://github.com/opencv/opencv
    cd opencv
    git checkout 4.5.3
    mkdir build
    cd build

Next configure CMake for the build as such

    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

Still from the build folder, make the project using parallel jobs, this will take a while

    make -j7 # runs 7 jobs in parallel

After the make command finishes successfully, install OpenCV as such

    sudo make install

OpenCV is now succesfully installed!
### ROS2 Dependencies
Source the global ROS2 setup script in the terminal

    source /opt/ros/foxy/setup.bash
  
Next, install rosdep as such
 
    sudo apt install python3-rosdep2
    rosdep update
  
Then, from the folder `triton`, resolve any dependency issues using the following command
 
    rosdep install -i --from-path src --rosdistro foxy -y
  
From the same folder, build all the packages using the following command

    colcon build
  
To finish the setup, edit your `.bashrc` file to source the global and local setup scripts whenever you open up a new terminal, this can be done using a text editor or by using `nano` as such
 
    nano ~/.bashrc
    
Navigate to the bottom of the file and add the following three lines, be sure to set `<PATH_TO_TRITON>` to whatever it is on your machine

    source /opt/ros/foxy/setup.bash                     # global setup script
    source <PATH_TO_TRITON>/triton/install/setup.bash   # local setup script
    export RCUTILS_COLORIZED_OUTPUT=1
    
The last line is helpful in that it colorizes ROS2 logging so that info/warn/error messages are easier to differentiate. Once this is done, open a new terminal for the `.bashrc` to be executed and the required scripts be sourced. To perform a sanity check that everything is working, launch the pipeline as such

    ros2 launch triton_pipeline pipeline_launch.py sequence:=example_sequence.yaml
   
If this command executes successfully, you are ready to develop!

## Contributing
To learn how to contribute to this repo, see the seperate [workflow](WORKFLOW.md) and [conventions](CONVENTIONS.md) documents.
    
## Tips
Here are some tips to be aware of when developing on this repository and when developing in ROS2 in general
- Make sure when you run any `colcon` command such as `colcon build` or `colcon test` that you do so in the root folder of this directory (i.e `triton`)
- After creating any new component nodes, you must either source the local setup script or simply open up a new terminal for them to show up under the command `ros2 component types` and be usable by the pipeline.
- Make sure you spell topics/services/actions correctly, be sure to debug by using `ros2 topic|service|action list`and `rqt_graph` to see that you are using the desired communcation channels.
- If you have added a dependency to a package by modifying the appropriate files (`CMakeLists.txt`, `package.xml`) and the build of that package is failing because it says it can't find the package, make sure you have it installed by running `rosdep install -i --from-path src --rosdistro foxy -y` in the `triton` folder, and also that a release for the distro we are using (`foxy`) exists on the ROS2 package index.
- For non-ROS2 dependencies, check [here](https://github.com/ros/rosdistro/tree/master/rosdep) to see the available system dependencies that can be used with `rosdep`.
=======
# Triton AUV

This repository contains the ROS2 system for the UBC SubBots Triton AUV. It is meant be launched in Ubuntu 20.04 on the Jetson TX2 on board the Triton AUV.

# Contents

- [Setup](#setup)
    - [Gazebo Installation](#gazebo-installation)
    - [OpenCV Installation](#opencv-installation)
    - [ROS2 Dependencies](#ros2-dependencies)
- [Contributing](#contributing)
- [Tips](#tips)
## Setup
To get started, first clone this repo to your computer running Ubuntu 20.04 into whatever directory you choose as such

    git clone https://github.com/ubc-subbots/triton.git

### Gazebo Installation
To install Gazebo, it is as simple as running the following command

    curl -sSL http://get.gazebosim.org | sh

To verify it was succesfully installed, run the following command

    gazebo --version

Next, we need to add the setup script to our `.bashrc` so that it is sourced on every new terminal, open up `~/.bashrc` in a text editor or in nano as such

    nano ~/.bashrc

Append the following line to the bottom of the file

    source /usr/share/gazebo/setup.sh

Now Gazebo is succesfully installed!
### OpenCV Installation
First, make sure you have all the dependencies installed for building and running OpenCV

    sudo apt-get install build-essential # compiler
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev # required
    sudo apt-get install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev # optional

Then download the OpenCV 4.5.3 source and create a build folder to navigate to as such

    cd ~
    git clone https://github.com/opencv/opencv
    cd opencv
    git checkout 4.5.3
    mkdir build
    cd build

Next configure CMake for the build as such

    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

Still from the build folder, make the project using parallel jobs, this will take a while

    make -j7 # runs 7 jobs in parallel

After the make command finishes successfully, install OpenCV as such

    sudo make install

OpenCV is now succesfully installed!
### ROS2 Dependencies
Source the global ROS2 setup script in the terminal

    source /opt/ros/foxy/setup.bash
  
Next, install rosdep as such
 
    sudo apt install python3-rosdep2
    rosdep update --include-eol #(Foxy is now at end of life)
  
Then, from the folder `triton`, resolve any dependency issues using the following command
 
    rosdep install -i --from-path src --rosdistro foxy -y
  
From the same folder, build all the packages using the following command

    colcon build
  
To finish the setup, edit your `.bashrc` file to source the global and local setup scripts whenever you open up a new terminal, this can be done using a text editor or by using `nano` as such
 
    nano ~/.bashrc
    
Navigate to the bottom of the file and add the following three lines, be sure to set `<PATH_TO_TRITON>` to whatever it is on your machine

    source /opt/ros/foxy/setup.bash                     # global setup script
    source <PATH_TO_TRITON>/triton/install/setup.bash   # local setup script
    export RCUTILS_COLORIZED_OUTPUT=1
    
The last line is helpful in that it colorizes ROS2 logging so that info/warn/error messages are easier to differentiate. Once this is done, open a new terminal for the `.bashrc` to be executed and the required scripts be sourced. To perform a sanity check that everything is working, launch the pipeline as such

    ros2 launch triton_pipeline pipeline_launch.py sequence:=example_sequence.yaml
   
If this command executes successfully, you are ready to develop!

## Contributing
To learn how to contribute to this repo, see the seperate [workflow](WORKFLOW.md) and [conventions](CONVENTIONS.md) documents.
    
## Tips
Here are some tips to be aware of when developing on this repository and when developing in ROS2 in general
- Make sure when you run any `colcon` command such as `colcon build` or `colcon test` that you do so in the root folder of this directory (i.e `triton`)
- After creating any new component nodes, you must either source the local setup script or simply open up a new terminal for them to show up under the command `ros2 component types` and be usable by the pipeline.
- Make sure you spell topics/services/actions correctly, be sure to debug by using `ros2 topic|service|action list`and `rqt_graph` to see that you are using the desired communcation channels.
- If you have added a dependency to a package by modifying the appropriate files (`CMakeLists.txt`, `package.xml`) and the build of that package is failing because it says it can't find the package, make sure you have it installed by running `rosdep install -i --from-path src --rosdistro foxy -y` in the `triton` folder, and also that a release for the distro we are using (`foxy`) exists on the ROS2 package index.
- For non-ROS2 dependencies, check [here](https://github.com/ros/rosdistro/tree/master/rosdep) to see the available system dependencies that can be used with `rosdep`.
>>>>>>> 22b7966d7b054d8c79bff6659b5aa571e2b2206a
