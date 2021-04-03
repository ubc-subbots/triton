import sys
import yaml
import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
import rclpy

"""
Helper module for loading a yaml file containing parameters for a node
into said node. Most of this code is from the following ros2 offical code

https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/api/__init__.py
"""

def load_parameter_file(node, node_name, parameter_file):
    """
    Uses the SetParameters service to load the parameter file into the node
    with the given node name

    Returns True if all params were loaded succesfully and false otherwise

    @param node: The node to create the clients that load the params 
    @param node_name: The name of the node which the params are loaded into
    @param parameter_file: The file containing the params to load into the node
    """
    internal_node_name = node_name.split('/')[-1] # only use node name, no namespaces
    with open(parameter_file, 'r') as f:
        param_file = yaml.safe_load(f)
        success = True
        if internal_node_name not in param_file:
            node.get_logger().warn('Param file does not contain parameters for {}, '
                               ' only for namespaces: {}' .format(internal_node_name,
                                                                  param_file.keys()))
            success = False
        value = param_file[internal_node_name]
        if type(value) != dict or 'ros__parameters' not in value:
            node.get_logger().warn('Invalid structure of parameter file in namespace {}'
                               'expected same format as provided by ros2 param dump'
                               .format(internal_node_name))
            success = False
        parameters = _parse_parameter_dict(namespace='', parameter_dict=value['ros__parameters'])
        return  _call_set_parameters(node=node, node_name=node_name, parameters=parameters)
    
        return success

def _parse_parameter_dict(namespace, parameter_dict):
    """
    Parses the parameters dictionary into a list of Parameters

    @param namespace: Parameters namespace
    @param parameter_dict: The ros__parameters key of the yaml
    """
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if type(param_value) == dict:
            parameters += _parse_parameter_dict(
                    namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                    parameter_dict=param_value)
        else:
            parameter = Parameter()
            parameter.name = full_param_name
            parameter.value = _get_parameter_value(string_value=str(param_value))
            parameters.append(parameter)
    return parameters

def _call_set_parameters(node, node_name, parameters):
    """
    Calls the SetParameters service to load the list of parameters

    Returns True if all params were loaded succesfully and false otherwise

    @param node: The node to create the clients that load the params 
    @param node_name: The name of the node which the params are loaded into
    @param parameters: List of Parameters
    """
    client = node.create_client(SetParameters, f'{node_name}/set_parameters')

    while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Parameter setting service not available, waiting again...')

    req = SetParameters.Request()
    req.parameters = parameters
    future = client.call_async(req)
    res = None
    success = True
    def callback(future):
        nonlocal res
        res = future.result()
    future.add_done_callback(callback)
    while rclpy.ok() and res is None:
        time.sleep(0.1)
    if len(res.results) != len(parameters):
        e = future.exception()
        node.get_logger().warn("Exception while calling set param service of node '{node_name}': {e}")
        success = False
    node.destroy_client(client)
    return success

def _get_value(parameter_value):
    """
    Get the value from a ParameterValue

    @param parameter_value: The ParameterValue 
    """
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        value = None
    return value

def _get_parameter_value(string_value):
    """
    Guess the desired type of the parameter based on the string value

    @param string_value: The string value of a parameter
    """
    value = ParameterValue()
    try:
        yaml_value = yaml.safe_load(string_value)
    except yaml.parser.ParserError:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = string_value
        return value
    if isinstance(yaml_value, bool):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = yaml_value
    elif isinstance(yaml_value, int):
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = yaml_value
    elif isinstance(yaml_value, float):
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = yaml_value
    elif isinstance(yaml_value, list):
        if all((isinstance(v, bool) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_BOOL_ARRAY
            value.bool_array_value = yaml_value
        elif all((isinstance(v, int) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            value.integer_array_value = yaml_value
        elif all((isinstance(v, float) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            value.double_array_value = yaml_value
        elif all((isinstance(v, str) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_STRING_ARRAY
            value.string_array_value = yaml_value
        else:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = string_value
    else:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = string_value
    return value

def _get_parameter_type_string(parameter_type):
    """
    Gets the string of the parameter type of a ParameterType

    @param parameter_type: The ParameterType
    """
    mapping = {
        ParameterType.PARAMETER_BOOL: 'boolean',
        ParameterType.PARAMETER_INTEGER: 'integer',
        ParameterType.PARAMETER_DOUBLE: 'double',
        ParameterType.PARAMETER_STRING: 'string',
        ParameterType.PARAMETER_BYTE_ARRAY: 'byte array',
        ParameterType.PARAMETER_BOOL_ARRAY: 'boolean array',
        ParameterType.PARAMETER_INTEGER_ARRAY: 'integer array',
        ParameterType.PARAMETER_DOUBLE_ARRAY: 'double array',
        ParameterType.PARAMETER_STRING_ARRAY: 'string array',
        ParameterType.PARAMETER_NOT_SET: 'not set',
    }
    return mapping[parameter_type]


