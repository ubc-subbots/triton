
# Importing Models From SolidWorks 

## Description
This guide explains how we import mesh files and kinematic information from Solidworks into a Gazebo model.

## Required Software
1. Solidworks (Windows)
2. Blender 2.80 or higher(Ubuntu)

## Process
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

## Referenced Material
[How to import Solidworks model into Gazebo](https://docs.google.com/document/d/1LrzAUCPOdZPh-uzIDg-aMNJGaX13d3AJjA7XWy2pFQQ/edit)
## Contributors
- Jake Cronin (jcronin97@live.ca)
