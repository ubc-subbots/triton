import os
import subprocess
import sys
import json
import argparse
import re

# Needs to change
REPO_ROOT = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode('utf-8')[:-1]


def convert_mesh(model_path, model):
    """
    Invoke Blender command line tooks with script containing commands to import and export
    """
    pre_check = subprocess.check_output(['blender', '--version']).decode('utf-8') 

    if 'Blender 2.8' not in pre_check:
        print('Blender pre-check failed, missing or invalid Blender installation')
        sys.exit(1)

    convert_mesh_path = os.path.join(REPO_ROOT, 'src/triton_gazebo/scripts/import_mesh/convert_mesh.py')
    run_cmd = ['blender', '--background', '--python', convert_mesh_path, '--', model_path, model]

    subprocess.run(run_cmd)


def convert_urdf(model_path, model_name):
    """
    Calls gz, converting .urdf file into .sdf
    """
    sdf_file = os.path.join(model_path, 'model.sdf')
    urdf_file = os.path.join(model_path, 'urdf', (model_name + '.urdf'))

    if not os.path.exists(urdf_file): 
        print('ERROR: urdf file, %s not found, please ensure directory was generated correctly, exiting' % urdf_file)
        sys.exit(1)

    run_cmd = ['gz', 'sdf', '-p' , urdf_file, '>', sdf_file]
    sdf_data = subprocess.check_output(run_cmd).decode('utf-8')[:-1]

    return sdf_data.splitlines()


def dict_to_sdf(parameters, sdf_fmt=[], depth=1):
    """
    converts a dictionary to a section of xml code
    """
    for key, data in parameters.items():
        indent = '\t'.join(['']*(depth+1))
        if isinstance(data, dict):
            sdf_fmt.append(indent + '<'+key+'>')
            dict_to_sdf(data, sdf_fmt, depth+1)
            sdf_fmt.append(indent + '</'+key+'>')
        elif isinstance(data, list):
            sdf_fmt.append(indent + '<'+key+'>'+' '.join(map(str, data))+'</'+key+'>')
        else:
            sdf_fmt.append(indent + '<'+key+'>'+str(data)+'</'+key+'>')
    
    return sdf_fmt


def main():
    parser = argparse.ArgumentParser(description='Creates a Gazebo model using a .STL file and a .urdf description.')
    parser.add_argument('--model', '-m', dest='model', required=True,
                        help='The name of the model directory')
    parser.add_argument('--params', '-p', dest='parameters', required=False,
                        default='import_mesh/config/default_parameters.json',
                        help='json file containing model parameters')

    args = parser.parse_args()

    model_path = os.path.join(REPO_ROOT, 'src/triton_gazebo/models', args.model)

    if not os.path.exists(model_path): 
        print('ERROR: model %s not found, exiting' % model_path)
        sys.exit(1)

    convert_mesh(model_path, args.model)

    sdf_data = convert_urdf(model_path, args.model)

    with open(os.path.join(REPO_ROOT, 'src/triton_gazebo/scripts/', args.parameters)) as param_file:
        params = json.load(param_file)



if __name__ == '__main__': 
    main()