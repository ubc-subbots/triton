# triton_pid_controller
## Description

This is a PID Controller for controlling thruster output forces based on the error between current pose and target pose. 

## Usage

To launch the `triton_pid_controller` node, use the following command

    ros2 launch triton_pid_controller triton_pid_controller_launch.py

You can change the P-, I-, and D-values in `config/pid_X.yaml`, where `X` is either `x`, `y`, `z`, or `yaw`. You can also change the `pid_files.yaml` file in the same directory to use other config files. 


## Nodes

- `triton_pid_controller` : A simple PID controller for the movement of the AUV in x, y, z, and yaw. 

    ### Subscribed Topics
    - `controls/input_pose` (`geometry_msgs/Pose`) : Error to target pose. For navigation. 
    ### Published Topics
    - `controls/output_forces` (`std_msgs/msg/Float64MultiArray.msg`) : Forces corresponding to each thruster
    - `controls/signals` (`std_msgs/msg/Float64MultiArray.msg`): PWM signals corresponding to each thruster
    ### Parameters 
    - `pid_files.yaml`: 
        - `pid_force_X_file` (`string`): The PID values for `X`, which is in [`x`, `y`, `z`, `yaw`]
    - `pid_X.yaml`: 
        - `Kp` (`float32`): The P of PID
        - `Ki` (`float32`): The I of PID
        - `Kd` (`float32`): The D of PID
