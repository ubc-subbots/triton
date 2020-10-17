# triton_gazebo
## Description

This package contains the Gazebo models, worlds, and plugins needed to create a dynamic simulation of an accurate model of our AUV in a realistic environment. These simulations integrate into ROS2 so that full system simulations can be run.

## Usage

To run a Gazebo simulation with a world file given in the `worlds` directory, use the `gazebo_lanch.py` file as follows

    ros2 launch triton_gazebo gazebo_launch.py world:=<WORLD_FILE_NAME>

Where `<WORLD_FILE_NAME>` is the name of the world file you want to run in Gazebo (e.g `cube.world`). Remember to build this package (i.e `colcon build --packages-select triton_gazebo`) everytime you change a model or world and want that change to propogate when you relaunch Gazebo.

## Worlds

`cube.world` 
- A simple world with the `cube` model.

## Models
`cube`
- A simple cube which uses a ROS2 force plugin to accept `geometry_msgs/msg/Wrench` messages on the topic `/triton/gazebo_drivers/force`. Here is an example command to apply forces to the cube

        ros2 topic pub /triton/gazebo_drivers/force geometry_msgs/msg/Wrench "{force: {x: 1}}"



## Contributors

- Logan Fillo (logan.fillo@gmail.com)
