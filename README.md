# Triton AUV

This repository contains the ROS2 system for the UBC SubBots Triton AUV. It is meant be launched in Ubuntu 20.04 on the Jetson TX2 on board the Triton AUV. 

## Setup
To get started, first clone this repo to your computer running Ubuntu 20.04 into whatever directory you choose as such

    git clone https://github.com/ubc-subbots/triton.git
    
Source the global ROS2 setup script in the terminal

    source /opt/ros/foxy/setup.bash
  
Next, install rosdep as such
 
    sudo apt install python3-rosdep2
    rosdep update
  
Then, from the folder `triton`, resolve any dependency issues using the following command
 
    rosdep install -i --from-path src --rosdistro foxy -y
  
From the same folder, build all the packages using the following command

    colcon build
  
 If the above command executes successfully, you are ready to develop! To finish the setup, edit your `.bashrc` file to source the global and local setup scripts whenever you open up a new terminal, this can be done using a text editor or by using `nano` as such
 
    nano ~/.bashrc
    
Navigate to the bottom of the file and add the following three lines, be sure to set `<PATH_TO_TRITON>` to whatever it is on your machine

    source /opt/ros/foxy/setup.bash                     # global setup script
    source <PATH_TO_TRITON>/triton/install/setup.bash   # local setup script
    export RCUTILS_COLORIZED_OUTPUT=1
    
The last line is helpful in that it colorizes ROS2 logging so that info/warn/error messages are easier to differentiate.
    
## Tips
Here are some tips to be aware of when developing on this repository and when developing in ROS2 in general
- Make sure when you run any `colcon` command such as `colcon build` or `colcon test` that you do so in the root folder of this directory (i.e `triton`)
- After creating any new component nodes, you must either source the local setup script or simply open up a new terminal for them to show up under the command `ros2 component types` and be usable by the pipeline.
- Make sure you spell topics/services/actions correctly, be sure to debug by using `ros2 topic|service|action list`and `rqt_graph` to see that you are using the desired communcation channels.
