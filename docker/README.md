## Docker

We use `docker-compose` to orchestrate the `micro-ros` agent contianer with our custom built `triton` container. This Docker system can be launched on the TX2 and still be able to communicate with various devices as the `/dev` directory is mounted to every container.

### Setup Micro-Ros

To develop with micro-ros on a Teensy 4.0 (i.e be able to compile and upload Arduino sketches using micros-ros to a Teesny 4.0), you have to configure the Arduino IDE as detailed below.

1. Install the Arduino IDE (v1.8.13 64-bit) on Linux Ubuntu 20.04 by downloading it [here](https://www.arduino.cc/en/software)
2. Install Teensyduino (v1.53 64-bit) by following the instructions [here](https://www.pjrc.com/teensy/td_download.html)
3. Apply the micro-ros-arduino patch as described [here](https://github.com/micro-ROS/micro_ros_arduino#patch-teensyduino) (the Arduino path is the folder from Arduino IDE installation, will probably be in Downloads if you installed as above)
4. Download the micro-ros-arduino Arduino library as described [here](https://github.com/micro-ROS/micro_ros_arduino#how-to-use-the-precompiled-library)
5. Verify that everything works by flashing one of the example sketches to the Teensy (the example sketches might show up under "INCOMPATIBLE" in the IDE)

### Building

There is no need to build manually, on every push to master, the image `ubcsubbots/triton:latest` on `dockerhub` is updated. The credentials used are stored in Github secrets and have to belong to a member of the `ubcsubbots` organization on `dockerhub`.

### Usage
To launch the entire docker system, run the following command in this directory. Note that for this to work, you need to have the micro-controller connected to the host on `/dev/ttyACM0`.

    docker-compose up

To enter into the ROS2 container (which is running the `triton` system), use the following command

    docker exec -it docker_ros_1 /subbots/entrypoint.sh /bin/bash

To enter into the Micro-ROS agent container, run the following command

    docker exec -it docker_micro-ros_1  /bin/bash
