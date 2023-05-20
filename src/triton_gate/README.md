# triton_gate
## Description

This package contains the nodes needed for performing gate detection and localization to be used on the gate task in competition

## Usage

Right now, you can launch the teleoperated simulation and see the gate detector working in RViz as such

    ros2 launch triton_bringup teleop_sim_bringup_launch.py

In RViz, you can see a row of images frame to the right, which has both raw camera views, the segmentation output, and gate bounding box output.

## Nodes

- `gate_detector` : A component node (`triton_gate::GateDetector`) which aims to detect the gate by drawing a tightly-fitting bounding box around it.
    ### Subscribed Topics
    - `/triton/drivers/front_camera/image_raw` (`sensor_msgs/msg/Image.msg`) : Input image
    
    ### Published Topics
    - `detector/gate_center` (`std_msgs/msg/Float32MultiArray.msg`) : Normalized position of the center of the gate. The first element is the x-coordinate of the center divided by the width of the image. The second element is the y-coordinate divided by the height of the image. The center of the gate is determined by the moments of the gate contour. 
    - `detector/debug/detection` (`sensor_msgs/msg/Image.msg`) : Detection debug output image.
    - `detector/debug/segment` (`sensor_msgs/msg/Image.msg`) : Segmentation debug output image.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Jared Chan (jaredchan42@gmail.com)
