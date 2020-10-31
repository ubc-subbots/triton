import os
import subprocess
import sys
import shutil
import json
import argparse

from xml_combiner import XMLCombiner

# Needs to change
REPO_ROOT = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode('utf-8')[:-1]

def convert_mesh(model_path, model):
    """
    Invoke Blender command line tooks with script containing commands to import and export
    """
    print("Blender Version:\n")
    pre_check = subprocess.check_output(['blender', '--version']).decode('utf-8') 

    if 'Blender 2.8' not in pre_check:
        print('Blender pre-check failed, missing or invalid Blender installation')
        sys.exit(1)

    convert_mesh_path = os.path.join(REPO_ROOT, 'src/triton_gazebo/scripts/import_mesh/convert_mesh.py')
    run_cmd = ['blender', '--background', '--python', convert_mesh_path, '--', model_path, model]
 
    # Need to catch errors here
    subprocess.run(run_cmd)
    print('Model mesh expoerted successfully\n')


def convert_urdf(model_path, model_name):
    """
    Calls gz, converting .urdf file into .sdf
    """
    sdf_file = os.path.join(model_path, 'model.sdf')
    urdf_file = os.path.join(model_path, 'urdf', (model_name + '.urdf'))

    print('Converting URDF to SDF format...\n')

    if not os.path.exists(urdf_file): 
        print('ERROR: urdf file, %s not found, please ensure directory was generated correctly, exiting' % urdf_file)
        sys.exit(1)

    run_cmd = ['gz', 'sdf', '-p' , urdf_file, '>', sdf_file]
    sdf_data = subprocess.check_output(run_cmd).decode('utf-8')[:-1]

    print('SDF converted successfully\n')

    return sdf_data.replace('.STL', '.dae').splitlines()


def dict_to_sdf(model_name, parameters, sdf_fmt=[], depth=1):
    """
    converts a dictionary to a section of xml code and applies model name to model, link, visual and collision 
    """
    for key, data in parameters.items():
        indent = '  '.join(['']*(depth+1))
        if isinstance(data, dict):
            if key == 'model' or key == 'link':
                sdf_fmt.append(indent + '<'+key+' name=\'%s\'>'%model_name)
            elif key == 'visual':
                sdf_fmt.append(indent + '<'+key+' name=\'%s_visual\'>'%model_name)
            elif key == 'collision':
                sdf_fmt.append(indent + '<'+key+' name=\'%s_collision\'>'%model_name)
            elif key == 'shader':
                sdf_fmt.append(indent + '<'+key+' type=\'pixel\'>')
            else:
                sdf_fmt.append(indent + '<'+key+'>')
            dict_to_sdf(model_name, data, sdf_fmt, depth+1)
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
    parser.add_argument('--clean-up', '-cu', dest='cleanup', required=False,
                        type=bool, default=False,
                        help='remove old files after successful conversion')
    args = parser.parse_args()

    model_path = os.path.join(REPO_ROOT, 'src/triton_gazebo/models', args.model)

    if not os.path.exists(model_path): 
        print('ERROR: model %s not found, exiting' % model_path)
        sys.exit(1)

    convert_mesh(model_path, args.model)

    sdf_data = ['<?xml version=\'1.0\'?>'] + convert_urdf(model_path, args.model)

    with open(os.path.join(REPO_ROOT, 'src/triton_gazebo/scripts/', args.parameters), 'r') as param_file:
        params = json.load(param_file)

    params_sdf = ['<?xml version=\'1.0\'?>', '<sdf version=\'1.7\'>']
    params_sdf += dict_to_sdf(args.model, params)
    params_sdf += ['</sdf>']

    if not os.path.exists(os.path.join(model_path, 'temp')):
        os.makedirs(os.path.join(model_path, 'temp'))

    with open(os.path.join(model_path, 'temp', 'params.sdf'), 'w') as sdf_file:
        sdf_file.write('\n'.join(params_sdf))
    with open(os.path.join(model_path, 'temp', 'model_org.sdf'), 'w') as sdf_file:
        sdf_file.write('\n'.join(sdf_data))


    result = '<?xml version=\'1.0\'?>\n' + XMLCombiner((os.path.join(model_path, 'temp', 'model_org.sdf'),
                                                        os.path.join(model_path, 'temp', 'params.sdf'))
                                                        ).combine().decode('utf-8')

    with open(os.path.join(model_path, 'model.sdf'), 'w') as sdf_file:
        sdf_file.write(result)

    model_config = [
        '<?xml version=\'1.0\'?>',
        '',
        '<model>',
        '  <name>%s</name>' % args.model,
        '  <version>1.0</version>',
        '  <sdf version=\'1.7\'>model.sdf</sdf>',
        '',
        '  <author>',
        '    <name></name>',
        '    <email></email>',
        '  </author>',
        '',
        '  <description>',
        '  </description>',
        '</model>'
        ]

    with open(os.path.join(model_path, 'model.config'), 'w') as config_file:
        config_file.write('\n'.join(model_config))

    if args.cleanup:
        shutil.rmtree(os.path.join(model_path, 'temp'))
        shutil.rmtree(os.path.join(model_path, 'urdf'))
        os.remove(os.path.join(model_path, 'meshes', args.model+'.STL'))
        os.remove(os.path.join(model_path, 'manifest.xml'))


if __name__ == '__main__': 
    main()