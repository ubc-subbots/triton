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

### Another control scheme

To launch the publisher for controlling the Teensy

    ros2 run triton_teleop key_publisher

It publishes to the `/motor_control` topic that the Teensy listens to. Key controls:  
<kbd>R</kbd>/<kbd>T</kbd> : Increment/Decrement power level of TLT  
<kbd>F</kbd>/<kbd>G</kbd> : Increment/Decrement power level of TLF  
<kbd>V</kbd>/<kbd>B</kbd> : Increment/Decrement power level of TLB  
<kbd>Y</kbd>/<kbd>U</kbd> : Increment/Decrement power level of TRT  
<kbd>H</kbd>/<kbd>J</kbd> : Increment/Decrement power level of TRF  
<kbd>N</kbd>/<kbd>M</kbd> : Increment/Decrement power level of TRB  
<kbd>W</kbd>/<kbd>E</kbd> : Increment/Decrement power level of TLT and TRT  
<kbd>S</kbd>/<kbd>D</kbd> : Increment/Decrement power level of TLF and TLB  
<kbd>X</kbd>/<kbd>C</kbd> : Increment/Decrement power level of TRF and TRB  

where 'TXX' stands for '**T**hruster **L**eft/**R**ight **T**op/**F**ront/**B**ack'
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

- `key_publisher` : A standalone node which listens on keyboard events and publishes interge values corresponding to power levels of thrusters
    ### Published Topics
    - `/motor_control` (`std_msgs/msg/UInt32`) : The unsigned integer message which consists of six 5-bit integers representing the power levels of the six thrusters.


## Contributors

- Logan Fillo (logan.fillo@gmail.com)
