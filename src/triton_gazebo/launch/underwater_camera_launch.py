import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import random

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_gazebo'),
        'config',
        'underwater_camera.yaml'
    )

    params = yaml.load(open(config,"r"), Loader=yaml.FullLoader)

    camera_type = "SONY_IMX322"
    water_type = random.choice(["I","IA","IB","II",'III',"1C","3C","5C","7C","9C"])
    water_type = "IA"
    irradiance_type = "ASTM_G173_03"

    underwater_camera = Node(
        name='underwater_camera',
        namespace='/triton/gazebo_drivers',
        package='triton_gazebo',
        executable='underwater_camera',
        output='screen',
        remappings=[('/triton/gazebo_drivers/front_camera/underwater/image_raw',
                     '/triton/drivers/front_camera/image_raw')],
        parameters=[
            {"rho": params["rho"]["default"]},
            {"irradiance_transmission": params["irradiance_transmission"][water_type]},
            {"spectral_sensitivity_blue": params["spectral_sensitivity"][camera_type][0]},
            {"spectral_sensitivity_red": params["spectral_sensitivity"][camera_type][1]},
            {"spectral_sensitivity_green": params["spectral_sensitivity"][camera_type][2]},
            {"illumination_irradiance": params["illumination_irradiance"][irradiance_type]}
        ]
    )

    ld.add_action(underwater_camera)

    return ld