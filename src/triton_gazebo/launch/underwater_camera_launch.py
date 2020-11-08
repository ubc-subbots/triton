import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_gazebo'),
        'config',
        'underwater_camera.yaml'
    )

    params = yaml.load(open(config,"r"), Loader=yaml.FullLoader)

    underwater_camera = Node(
        name='underwater_camera',
        namespace='/triton/drivers/front_camera',
        package='triton_gazebo',
        executable='underwater_camera',
        output='screen',
        parameters=[
            {"rho": params["rho"]["default"]},
            {"irradiance_transmission": params["irradiance_transmission"]["I"]},
            {"spectral_sensitivity_blue": params["spectral_sensitivity"]["SONY_IMX322"][0]},
            {"spectral_sensitivity_red": params["spectral_sensitivity"]["SONY_IMX322"][1]},
            {"spectral_sensitivity_green": params["spectral_sensitivity"]["SONY_IMX322"][2]},
            {"illumination_irradiance": params["illumination_irradiance"]["ASTM_G173_03"]}
        ]
    )

    ld.add_action(underwater_camera)

    return ld