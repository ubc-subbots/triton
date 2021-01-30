# triton_object_recognition
## Description

This package contains a ROS2 node that can use a YOLOv3 network to detect and classify objects within an image. Note that based on the config file passed to the object recognition node, it might take a while for the node to start for the first time because it will download the appropriate config and weight files to the path `<PATH_TO_TRITON>/triton/install/triton_object_recognition/share/triton_object_recognition`. These files tend to be quite large but they should only have to be downloaded once provided you do not delete the `install` folder.

## Usage

You can use the following command to run the `object_recognizer` node using the yolov3 tiny model:

    ros2 launch triton_object_recognition tiny_yolov3_launch.py

You should see `Object Recognizer successfully started!` if successful.

## Nodes

- `object_recognizer` : A component node (`triton_object_recognition::ObjectRecognizer`) which recognizes objects in a received image.

    ### Subscribed Topics
    - `object_recognizer/in` (`sensor_msgs/msg/Image.msg`) : Input image.
    
    ### Published Topics
    - `object_recognizer/out` (`triton_interfaces/msg/DetectionBoxArray.msg`) : Output detection boxes.
    
    ### Services
    - `object_recognizer/recognize` (`triton_interfaces/srv/ObjectDetection.srv`): Takes in an image message and produces the detection boxes from that image.

## Contributors

- Kevin Huang (kevinh42@student.ubc.ca)
