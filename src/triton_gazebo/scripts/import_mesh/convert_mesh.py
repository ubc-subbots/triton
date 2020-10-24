import os
import sys


import bpy

def convert_stl_to_dae(file_no_extension):
    """
    Converts a given .STL file to .dae using Blender API
    """
    stl_file = file_no_extension + ".STL"
    dae_file = file_no_extension + ".dae"

    # Select all objects in Blender scene
    bpy.ops.object.select_all(action='SELECT')
    # Delete all objects in scene
    bpy.ops.object.delete() 
    # Import .STL file
    bpy.ops.import_mesh.stl(filepath=stl_file)
    # Export .STL file as .dae
    bpy.ops.wm.collada_export(filepath=dae_file, selected=True)


def main():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] # get all args after "--"

    filepath = argv[0]
    name = argv[1]

    file_no_extension = os.path.join(filepath, "meshes", name)

    convert_stl_to_dae(file_no_extension)


if __name__ == "__main__":
    main()