# Triton AUV

This repository contains the ROS2 system for the UBC SubBots Triton AUV. It can be launched on the Jetson TX2 on board the Triton AUV. 

## Setup
To get started, first clone this repo to your computer running Ubuntu 20.04 into whatever directory you choose as such

  git clone https://github.com/ubc-subbots/triton.git
  
Next, install rosdep as such
 
  sudo apt install python3-rosdep2
  rosdep update
  
Then, from the folder `triton`, resolve any dependency issues using the following command
 
  rosdep install -i --from-path src --rosdistro foxy -y
  
From the same folder, build all the packages using the following command

  colcon build
  
 If the above command does not fail, you are ready to develop!

## Tips
Here are some tips to be aware of when developing on this repository and when developing in ROS2 in general
- Make sure when you run any `colcon` command such as `colcon build` or `colcon test` that you do so in the ROOT folder of this directory (i.e `triton`)
- After creating any new component nodes, you must either source the local install script or simply open up a new terminal for them to show up under the command `ros2 component types` and be usable by the pipeline
- Make sure you spell topics/services/actions correctly, be sure to debug by using `ros2 topic|service|action list`and `rqt_graph` to see that you are using the desired communcation channels.
