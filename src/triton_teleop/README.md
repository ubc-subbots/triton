# triton_teleop
## Description

This package is for teleoperation (i.e manual control) of the AUV.

## Usage

To launch the keyboard teleoperation simulation of the AUV, use the following command

    ros2 launch triton_bringup teleop_sim_bringup_launch.py

This will open up Gazebo and allow you to control the AUV with a keyboard, below are the key controls 

<kbd>W</kbd>/<kbd>S</kbd> : Forwards/Backwards (X-Axis)  
<kbd>A</kbd>/<kbd>D</kbd> : Left/Right (Y-Axis)  
<kbd>Q</kbd>/<kbd>Z</kbd> : Up/Down (Z-Axis)  Up/Down (Z-Axis)  
<kbd>←</kbd>/<kbd>→</kbd> : Rotate Left/Rotate Right (Around Z-Axis)   
<kbd>↑</kbd>/<kbd>↓</kbd> : Rotate Left/Rotate Right (Around X-Axis)

## Nodes

- `keyboard_teleop` : A standalone node which listens on keyboard events and publishes forces corresponding to these events
    ### Published Topics
    - `/triton/controls/input_forces` (`geometry_msgs/msg/Wrench.msg`) : The wrench message which acts upon the body of the AUV

- `sim_thrust_mapper` : A standalone node which takes in input from the thrust allocator and multiplexes it to the gazebo driver topic for each simulated thruster

    ### Subscribed Topics
    - `/triton/controls/output_forces` (`std_msgs/msg/Float64MultiArray.msg`) : Control output, must be num_thrusters in length

    ### Published Topics
    - `/triton/gazebo_drivers/thruster_<N>` (`geometry_msgs/msg/Wrench.msg`) : A wrench message for each thruster, only the X axis force is used as it assumes all thruster's X axis is aligned to where their propeller's produce positive force

    ### Parameters
    - `num_thrusters` (`integer`): Number of thrusters to map the control output to


## Contributors

- Logan Fillo (logan.fillo@gmail.com)
