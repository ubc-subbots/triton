## Docker

### Usage
To launch the entire docker system, run the following command in this directory. Note that for this to work, you need to have the micro-controller connected to the host on `/dev/ttyACM0`.

    docker-compose up

To enter into the ROS2 container (which is running the `triton` system), use the following command

    docker exec -it docker_ros_1 /subbots/entrypoint.sh /bin/bash

To enter into the Micro-ROS agent container, run the following command

    docker exec -it docker_micro-ros_1  /bin/bash
