# triton_teleop
## Description

This package is for teleoperation (i.e manual control) of the AUV.

## Usage

To launch the keyboard teleoperation simulation of the simple cube AUV, use the following command

    ros2 launch triton_teleop keyboard_teleop_sim_launch.py 

This will open up Gazebo and allow you to control the AUV with a keyboard, below are the key controls 

<kbd>W</kbd>/<kbd>S</kbd> : Forwards/Backwards (X-Axis)  
<kbd>A</kbd>/<kbd>D</kbd> : Left/Right (Y-Axis)  
<kbd>Q</kbd>/<kbd>Z</kbd> : Up/Down (Z-Axis)  Up/Down (Z-Axis)  
<kbd>←</kbd>/<kbd>→</kbd> : Rotate Left/Rotate Right (Around Z-Axis)   
<kbd>↑</kbd>/<kbd>↓</kbd> : Rotate Left/Rotate Right (Around X-Axis)

## Nodes

- `keyboard_teleop` : A standalone node which listens on keyboard events and publishes forces corresponding to these events

- `sim_thrust_mapper` : A standalone node which takes in input from the thrust allocator and multiplexes it to the gazebo driver topic for each simulated thruster

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
