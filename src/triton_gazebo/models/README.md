
# Importing Models From SolidWorks 

## Description
This guide details how we import models mesh files and kinematic information from Solidworks.
## Requirements
1. Solidworks (Windows)
2. Blender (Ubuntu)
## Setup
1. In Windows, install the SolidWorks [URDF exporter](http://wiki.ros.org/sw_urdf_exporter)
2. In Linux, install ... 
## Usage
1. Open the part you wish to import in Solidworks and ensure that the center of mass, inertia tesor and mass value are corrent and in SI units.
2. 
To run the script for importing packages containing a .STL file and .urdf description, navigate to the triton repository and execute the command:
    python3 <path-to-scripts>/scripts/import_mesh/import_mesh.py --model <model-name> --clean-up <True/False>
## Contributors
- Jake Cronin (jcronin97@live.ca)
