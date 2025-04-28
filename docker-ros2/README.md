# Docker container for ROS2 environment for Jetson TX2
This container contains everything you need to run the ROS2 system of the robot, so that a brand new in box TX2 can be up and running in less than an hour.

## How to run
This assumes you have followed the TX2 imaging guide in Wiki.
1. Grab the latest image.
2. Import into TX2 with `sudo docker import <path-to-image>`.
3. Run the container with `sudo docker run -it --net=host -v /dev:/dev --privileged ros2-foxy-triton:<version> bash`.
4. Update and `colcon build` the triton repo if necessary.
5. Profit!

## How to build
Copy `Dockerfile` to the a new folder in TX2, and run `sudo docker build -t "ros2-foxy-triton:<version>" . 2>&1 | tee build.log`.

## Todo
1. The build process still reports some errors when building but and not reproducible when in the container. Need to investigate.
2. Test the container with all necessary workloads.
3. Make it work with Orbitty carrier board. Need to update flashing TX2 instructions as well.
