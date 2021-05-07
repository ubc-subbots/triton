## Docker

### Setup

How to develop with micro-ros on a Teensy 4.0

1. Install the Arduino IDE (v1.8.13 64-bit) on Linux Ubuntu 20.04 by downloading it [here](https://www.arduino.cc/en/software)
2. Install Teensyduino (v1.53 64-bit) by following the instructions [here](https://www.pjrc.com/teensy/td_download.html)
3. Apply the micro-ros-arduino patch as described [here](https://github.com/micro-ROS/micro_ros_arduino#patch-teensyduino) (the Arduino path is the folder from Arduino IDE installation, will probably be in Downloads if you installed as above)
4. Download the micro-ros-arduino Arduino library as described [here](https://github.com/micro-ROS/micro_ros_arduino#how-to-use-the-precompiled-library)
5. Verify that everything works by flashing one of the example sketches to the Teensy (the example sketches might show up under "INCOMPATIBLE" in the IDE)

### Usage
To launch the entire docker system, run the following command in this directory. Note that for this to work, you need to have the micro-controller connected to the host on `/dev/ttyACM0`.

    docker-compose up

To enter into the ROS2 container (which is running the `triton` system), use the following command

    docker exec -it docker_ros_1 /subbots/entrypoint.sh /bin/bash

To enter into the Micro-ROS agent container, run the following command

    docker exec -it docker_micro-ros_1  /bin/bash
