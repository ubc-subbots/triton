# triton_gazebo
## Description

This package contains the Gazebo models, worlds, and plugins needed to create a dynamic simulation of an accurate model of our AUV in a realistic environment. These simulations integrate into ROS2 so that full system simulations can be run.

## Usage

### Gazebo
To run a Gazebo simulation with a world file given in the `worlds` directory, use the `gazebo_lanch.py` file as follows

    ros2 launch triton_gazebo gazebo_launch.py world:=<WORLD_FILE_NAME> headless:=<IS_HEADLESS>

Where `<WORLD_FILE_NAME>` is the name of the world file you want to run in Gazebo (e.g `cube.world`) and `<IS_HEADLESS>` is true if you don't want to run the GUI (defaults to false). Remember to build this package (i.e `colcon build --packages-select triton_gazebo`) everytime you change a model or world and want that change to propogate when you relaunch Gazebo.

### Underwater Camera
To run the underwater camera node, use the following

        ros2 launch triton_gazebo underwater_camera_launch.py

Sets of parameters for the underwater camera node (water transmission, spectral sensitivity, etc.) are stored in `config/underwater_camera.yaml`. To change which parameters are used, `launch/underwater_camera_launch.py` can be modified

### Generating Data with Bounding Box
To run the data generation node, use the following

        ros2 launch triton_gazebo gendata_launch.py

This runs the Gazebo world `uc_gendata.world`, runs the underwater camera node with a random water type, and saves the output images along with label files containing the bounding box of the tracked model defined in `uc_gendata.world` under `bounding_box_controller/model_name`. The images and label files are saved in a folder called `data` in the triton_gazebo shared folder.

Currently, the generated images are not always well-suited for training, and may not be rendered properly due to timing mismatches between the Gazebo render, the underwater camera node, and the bounding box node. Some manual cleaning (removing incorrect images) will need to be done to obtain a good dataset.

### Training Yolo v3 Model
The triton_gazebo shared folder should contain a folder called `data` with images with `.png` extensions (this can be changed in `train_yolo.py`). Each image should have a corresponding `.txt` file with the same name (other than extension) containing labels in standard Yolo format describing the bounding boxes (each line should be `<object class (integer)> <x centre> <y centre> <box width> <box height`, with x/y/width/height normalized between 0 and 1 using the image's dimensions).

The `yolov3_custom.cfg` file should be modified based on the requirements of the model. See `AlexeyAB/darknet` on GitHub for a guide. 

Run `train_yolo.py` to start training the model. By default, as training progresses backups will be saved in a folder called `backup` in the shared folder.

## Nodes/Plugins

- `underwater_camera`: A node which produces synthesized underwater images from a RGB/depth image pair

    ### Subscribed Topics
    - `front_camera/image_raw` (`sensor_msgs/msg/Image.msg`) : Input RGB image
    - `front_camera/depth/image_raw` (`sensor_msgs/msg/Image.msg`) : Input depth image
    - `front_camera/bounding_box` (`triton_interfaces/msg/DetectionBoxArray`) : Bounding box (only republishes to avoid data generation timing issues)

    ### Published Topics
    - `front_camera/underwater/image_raw` (`sensor_msgs/msg/Image.msg`) : Synthesized underwater image
    - `repub/image_raw` (`sensor_msgs/msg/Image.msg`) : Republished RGB image
    - `repub/depth/image_raw` (`sensor_msgs/msg/Image.msg`) : Republished depth image
    - `repub/bounding_box` (`triton_interfaces/msg/DetectionBoxArray`) : Republished bounding box

- `bounding_box_plugin`: A Gazebo plugin which produces the bounding box of a specified object in camera image coordinates

    ### Published Topics
    - `front_camera/bounding_box` (`triton_interfaces/msg/DetectionBoxArray`) : Bounding box

- `bounding_box_image_saver`: A node which saves images and bounding boxes for training.

    ### Subscribed Topics
    - `front_camera/underwater/image_raw` (`sensor_msgs/msg/Image.msg`) : Synthesized underwater image
    - `repub/bounding_box` (`triton_interfaces/msg/DetectionBoxArray`) : Bounding box

- `camera_orbit_plugin`: A Gazebo plugin which rotates the camera around a specified object for data collection (not recommended as training data)

- `random_camera_plugin`: A Gazebo plugin which randomly moves the camera around for data collection (currently no guarantee the object is in view)
     
## Worlds

`cube.world` 
- A simple world with the `cube` model.

`uc_gendata.world` 
- A world used for generating synthetic data for training.

## Models
`cube`
- A simple cube which uses a ROS2 force plugin to accept `geometry_msgs/msg/Wrench` messages on the topic `/triton/gazebo_drivers/force`. Here is an example command to apply forces to the cube

        ros2 topic pub /triton/gazebo_drivers/force geometry_msgs/msg/Wrench "{force: {x: 1}}"

`lenabox`
- A cube with the Lena test image as its texture.

##  Importing Models From SolidWorks 

This guide explains how we import mesh files and kinematic information from Solidworks into a Gazebo model.

### Required Software

1. Solidworks (Windows)
2. Blender 2.80 or higher(Ubuntu)

### Process

1. In Windows, install the SolidWorks [URDF exporter](http://wiki.ros.org/sw_urdf_exporter) and follow the default installation instructions.
2. In SolidWorks, open the part you wish to export.
    - In the "Evaluate" tab, open "Mass Properites" and verify that the inertia tensor, mass and center of mass are correct and in SI units. 
    - Click File->Export as URDF (This option should be available after installing the plugin)
    - Verify that the model properties are correct and click Finish, navigate to the created folder and verify that it contains "meshes" and "urdf".
    - Save these files somewhere outside Windows' filesystem like a Git branch, likely you will only need to save the "meshes" and "urdf" folders.
3. Boot your machine into Linux
4. Access the files saved remotely mentioned in the previous step, save these to `triton/src/triton_gazebo/models/`
6. To run the script for importing packages containing a .STL file and .urdf description, navigate to the triton repository and execute the command:
    `python3 <path-to-scripts>/scripts/import_mesh/import_mesh.py --model <model-name> --clean-up <True/False>`
   This command:
    - Converts the stl file into a dae using blender
    - converts the urdf to sdf
    - Adds additional information to make sure the model interacts with its environment properly
    - removed folders no longer needed

At this point your model should be ready to be added to a Gazebo world, you can view your model by running:
        `ros2 launch triton_gazebo gazebo_launch.py`
    and placing your model into the scene.

### Referenced Material

[How to import Solidworks model into Gazebo](https://docs.google.com/document/d/1LrzAUCPOdZPh-uzIDg-aMNJGaX13d3AJjA7XWy2pFQQ/edit)

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
- Jake Cronin (jcronin97@live.ca)
- Kevin Huang (kevinh42@student.ubc.ca)
