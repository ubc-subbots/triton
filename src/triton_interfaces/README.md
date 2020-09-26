# triton_interfaces
## Description

This package is for adding custom interface functionality such as custom messages, actions, and services. Any new interface must follow the ROS2 conventions given [here](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/). 

## Usage

This package provides no executables. See ROS2 tutorials (such as [this](https://index.ros.org/doc/ros2/Tutorials/Single-Package-Define-And-Use-Interface/#id6)) for how to use custom messages. Below provides a brief description of each interace in this package.
### Actions
- `RunPipeline` Action used for running the pipeline
### Messages
- `Pipelineeedback` Message used for nodes within the pipline to communicate with the manager
- `PipelineType` Message used for enumeration of pipeline types
### Services
- `ConfigurePipeline` Service used to configure the pipeline with a certain pipeline type

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
