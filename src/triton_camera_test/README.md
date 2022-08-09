# triton_camera_test
## Description

This package is for testing other modules (gate detector) by publishing a video stream to an image topic.

## Usage

To launch the video publisher, use the following command

    ros2 run triton_camera_test camera_test

To launch the video publisher, gate detector, and rviz:

    ros2 launch src/triton_camera_test/launch/camera_test_launch.py 


## Nodes

- `camera_test` : A standalone node which reads a video/uses a camera stream and publishes images
    ### Published Topics
    - `/triton/drivers/front_camera/image_raw` (`sensor_msgs/msg/Image.msg`)

    ### Subscribed Topics
    - `/triton/triton_camera_test/video_source` (`std_msgs/msg/String.msg`) : Path of the desired video source


## Contributors

- Jared Chan (jaredchan42@gmail.com)
