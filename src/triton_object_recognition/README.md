# triton_object_recognition
## Description

This package contains a ROS2 node that can use a YOLOv3 network to detect and classify objects within an image.

## Usage

You can use the following command to run the node:

    ros2 launch triton_object_recognition base_yolov3.launch.py

You should see `ObjectRecognizer loaded successfully` if successful.

## Nodes

- `object_recognizer` : A component node (`object_recognition::ObjectRecognizer`) which recognizes objects in a received image.

    ### Subscribed Topics
    - `object_recognizer/in` (`sensor_msgs::msg::Image`) : Input image.
    
    ### Published Topics
    - `object_recognizer/out` (`triton_interfaces::msg::DetectionBoxArray`) : Output detection boxes.
    
    ### Service
    - `object_recognizer/recognize` (Input: `sensor_msgs::msg::Image`, Output, `triton_interfaces::msg::DetectionBoxArray`)

## Contributors

- Kevin Huang (kevinh42@student.ubc.ca)
