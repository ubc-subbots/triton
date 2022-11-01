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
- `DetectionBox` Message used for a detection box in an object recognition system
- `DetectionBoxArray` Message used to represent an array of detection boxes
- `Waypoint` Message used to represent a waypoint
### Services
- `ConfigurePipeline` Service used to configure the pipeline with a certain pipeline type
- `ObjectDetection` Service used to detect objects in a given image

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Jared Chan (jaredchan42@gmail.com)
