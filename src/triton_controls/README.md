# triton_controls
## Description

This package contains the nodes related to the control system. 

## Usage

To launch the `thrust_allocator` node, use the following command

    ros2 launch triton_controls thrust_allocator_launch.py

You can change the configuration file for the `thrust_allocator` in the launch file, just be sure that the configuration file you want to use exists in `triton_controls/config`.

### Robot Localization
To run the robot localization node, which estimates the pose, twist, and acceleration of the AUV, run

        ros2 launch triton_controls state_estimator_launch.py

## Nodes

- `thrust_allocator` : A standalone node which subscribes to the desired forces and torques to act upon the AUV and publishes an array which contains the allocated forces and associated PWM signals given to each thruster based on the config file passed in on launch

    ### Subscribed Topics
    - `controls/input_forces` (`geometry_msgs/msg/Wrench.msg`) : Input forces and torques
    ### Published Topics
    - `controls/output_forces` (`std_msgs/msg/Float64MultiArray.msg`) : Forces corresponding to each thruster
    - `controls/signals` (`std_msgs/msg/Float64MultiArray.msg`): PWM signals corresponding to each thruster
    ### Parameters 
    - `num_thrusters` (`int32`): The number of thrusters in the configuration
    - `ti.contrib.x` (`float32`): The x-axis contribution of thruster `i`
    - `ti.contrib.y` (`float32`): The y-axis contribution of thruster `i`
    - `ti.contrib.z` (`float32`): The z-axis contribution of thruster `i`
    - `ti.lx` (`float32`): The x-length of thruster `i` from the AUV's centre of mass
    - `ti.ly` (`float32`): The y-length of thruster `i` from the AUV's centre of mass
    - `ti.lz` (`float32`): The z-length of thruster `i` from the AUV's centre of mass
    ### Notes
    - In the config file, you MUST define the thrusters `t1,...,tn` in the config file where `n = num_thrusters`. See the config file for examples of this configuration
    - You can currently only configure up to **6** thrusters.
    - In the `config.yaml`, the root key must be the full name of the node being configured
    - Here is a diagram showing how to parameterize a thruster (assume it's in the `z = 0` plane of the centre of mass)
    ![Thruster Config](../../assets/thruster_config.jpg)  
    Be aware that in this example `t1.ly` would be negative, `t1.lx` would be positive, and `t1.contrib.x`, `t1.contrib.y` would both also be positive.

- `state_estimator` : A standalone node which subscribes to the IMU and sound localization position topics, and publishes an estimated state of the AUV, including position, velocity, and acceleration. 

    ### Subscribed Topics
    - `drivers/imu/out` (`sensor_msgs/Imu`) : IMU data: linear acceleration and angular velocity
    - `state` (`geometry_msgs/PoseWithCovarianceStamped`) : Position. This topic name will likely be updated. 
    ### Published Topics
    - `controls/ukf/odometry/filtered` (`nav_msgs/Odometry`) : AUV state
    ### Notes
    - This node is configured with the config file `state_estimator_config.yaml`.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Jared Chan (jaredchan42@gmail.com)
